#include <AccelStepper.h>
#include <Controllino.h>
#include "EasyNextionLibrary.h"
#include "SensorModbusMaster.h"

//Stepper cutter settings
//Current: 4.0A RMS, Full Current
//Pulses / Rev: 1000
//SW1: Off
//SW2: Off
//SW3: Off
//SW4: On (full current)
//SW5: On
//SW6: On
//SW7: On
//SW8: Off

//Stepper feeder settings
//Current: 2.8 (2,7)A RMS, Full Current
//Pulses / Rev: 1000
//SW1: On
//SW2: On
//SW3: Off
//SW4: On (full current)
//SW5: On
//SW6: On
//SW7: On
//SW8: Off

EasyNex myNex(Serial2);

int cutterServoRPM = 200;
int straightenerServoRPM = 1400;
int cutterSteps = 5000;
int cutterMaxSpeedSetting = 1000;

#define safetyRelay CONTROLLINO_R0                //Relay to release safety
#define externalPowerOutlet CONTROLLINO_R2        //Relay for vaccuum
#define safetyStandbyRelay CONTROLLINO_R3         //Standby mode for safety
#define lightRelay CONTROLLINO_R4                 //Relay for light

#define cutterServoEnabled CONTROLLINO_R10        //Cutter servo enabled relay
#define straightenerServoEnabled CONTROLLINO_R12  //straightener servo enabled relay

#define cuttingTableBottomSensor CONTROLLINO_A3
#define cuttingTableTopSensor CONTROLLINO_A5

#define ledGreenPin CONTROLLINO_D10
#define ledRedPin CONTROLLINO_D11

#define safetyRelayInput CONTROLLINO_A1
#define safetyButtonInput CONTROLLINO_A2
bool safetyRelayStatus = false;
bool safetyButtonStatus = false;

//Modbus settings
long modbusBaudRate = 9600;
const int DEREPin = CONTROLLINO_RS485_DE;
HardwareSerial* modbusSerial = &Serial3;
modbusMaster modbus;
modbusMaster modbus2;
int motorId = 0;

//Proces vars
int lengthVar = 10;
int quantityVar = 10;

bool errorFlag = 0;
bool processingFlag = 0;
int processingCount = 0;

//Feeder stepper PINs
#define stepperFeederStepPin CONTROLLINO_D0
#define stepperFeerderDirPin CONTROLLINO_D1
#define stepperFeederEnablePin CONTROLLINO_D2  //Check if this is correct, if the steppers go the wrong way change to CONTROLLINO_D3
//CONTROLLINO_D3 is connected to DIR of the second stepper motor

//Cutter table stepper PINs
#define stepperCutterStepPin CONTROLLINO_D4
#define stepperCutterDirPin CONTROLLINO_D5
#define stepperCutterEnablePin CONTROLLINO_D6

#define motorInterfaceType 1

AccelStepper stepperFeeder = AccelStepper(motorInterfaceType, stepperFeederStepPin, stepperFeerderDirPin);
AccelStepper stepperCutter = AccelStepper(motorInterfaceType, stepperCutterStepPin, stepperCutterDirPin);

int feederMaxSpeedSetting = 5000;
int feederExtrudeAccel = 10000;
int cutterExtrudeAccel = 10000;

int currentPage = 0;
int lastPage = 0;

bool debug = true;

bool cutActive = false;

bool cutterTableMoveUp = 0;

String proccessingStatus = "";

bool stepperCutterHomedFlag = 0;
bool stepperCutterSafetyFlag = 0;

void setup() {
  //PINs settings
  pinMode(CONTROLLINO_D0, OUTPUT);
  pinMode(CONTROLLINO_D1, OUTPUT);
  pinMode(CONTROLLINO_D2, OUTPUT);
  pinMode(CONTROLLINO_D3, OUTPUT);
  pinMode(CONTROLLINO_D4, OUTPUT);
  pinMode(CONTROLLINO_D5, OUTPUT);
  pinMode(CONTROLLINO_D6, OUTPUT);
  pinMode(CONTROLLINO_D7, OUTPUT);
  pinMode(CONTROLLINO_D8, OUTPUT);
  pinMode(CONTROLLINO_D10, OUTPUT);
  pinMode(CONTROLLINO_D11, OUTPUT);


  pinMode(CONTROLLINO_R0, OUTPUT);
  pinMode(CONTROLLINO_R2, OUTPUT);
  pinMode(CONTROLLINO_R3, OUTPUT);
  pinMode(CONTROLLINO_R4, OUTPUT);
  pinMode(CONTROLLINO_R10, OUTPUT);
  pinMode(CONTROLLINO_R12, OUTPUT);

  pinMode(CONTROLLINO_A0, INPUT);
  pinMode(CONTROLLINO_A1, INPUT);
  pinMode(CONTROLLINO_A2, INPUT);

  pinMode(cuttingTableBottomSensor, INPUT);
  pinMode(cuttingTableTopSensor, INPUT);

  //Nextion setup
  myNex.begin(9600);
  delay(100);
  myNex.writeStr("page 0");
  Serial.begin(115200);

  resetSafety();
  setSafetyStandby();

  //  //Modbus settings
  pinMode(DEREPin, OUTPUT);
  Serial3.begin(9600);
  modbus.begin(0x01, modbusSerial, DEREPin);   //Cutter servo
  modbus2.begin(0x02, modbusSerial, DEREPin);  // Straigtner servo
  delay(50);
  Serial.print("Searching motor ");
  while (motorId == 0) {
    motorId = modbus.uint16FromRegister(0x03, 0, bigEndian);
    Serial.print(".");
    delay(50);
  }
  Serial.println("");
  Serial.println("Motor detected: " + String(motorId));
  delay(50);
  setCutterServoRPM(0);
  setStraightenerServoRPM(0);

  //  modbus.uint16ToRegister(1541, 2000, bigEndian);  //Cutter servo acceleration 2s
  //  modbus.uint16ToRegister(1542, 2000, bigEndian);  //Cutter servo deceleration 2s
  //  modbus2.uint16ToRegister(1541, 2000, bigEndian); //Straigtener servo acceleration 2s
  //  modbus2.uint16ToRegister(1542, 2000, bigEndian); //Straigtener servo deceleration 2s

  stepperFeeder.setMaxSpeed(feederMaxSpeedSetting);
  stepperFeeder.setAcceleration(feederExtrudeAccel);
  stepperFeeder.enableOutputs();

  stepperCutter.setMaxSpeed(cutterMaxSpeedSetting);
  stepperCutter.setAcceleration(cutterExtrudeAccel);
  stepperCutter.enableOutputs();

  //Light on
  digitalWrite(lightRelay, HIGH);

  delay(200);
  Serial.println("Setup complete");

  digitalWrite(CONTROLLINO_D10, HIGH);
  digitalWrite(CONTROLLINO_D11, HIGH);
}

void loop() {
  currentPage = myNex.currentPageId;
  safetyRelayStatus = digitalRead(safetyRelayInput);
  safetyButtonStatus = digitalRead(safetyButtonInput);


  if (safetyButtonStatus && currentPage != 14) {
    Serial.println("Safety button activated");
    myNex.writeStr("page 14");
    delay(100);
    myNex.writeStr("t5.txt", "Emergency button activated");
    errorFlag = 1;
  }

  if (!safetyRelayStatus && currentPage != 14) {
    Serial.println("Safety relay activated");
    myNex.writeStr("page 14");
    delay(100);
    myNex.writeStr("t5.txt", "Safety activated");
    errorFlag = 1;
  }

  if (currentPage != lastPage) {
    lastPage = currentPage;
    Serial.print("Current page: ");
    Serial.println(currentPage);

    if (currentPage == 2) {
      delay(500);
      myNex.writeStr("t0.txt", String(lengthVar));
      delay(50);
      myNex.writeStr("t1.txt", String(quantityVar));
    }
  }

  if (processingFlag && !errorFlag) {
    if (processingCount < quantityVar) {
      processingCount++;
      proccessingStatus = String(processingCount) + " / " + String(quantityVar);
      myNex.writeStr("t2.txt", proccessingStatus);
      enableStraightenerServo();
      delay(100);
      setStraightenerServoRPM(straightenerServoRPM);
      delay(500);
      stepperFeeder.setCurrentPosition(0);
      stepperFeeder.runToNewPosition(1500*lengthVar);
      setStraightenerServoRPM(0);
      delay(1000);
      disableStraightenerServo();
      enableExternalPower();
      enableCutterServo();
      delay(100);
      setCutterServoRPM(cutterServoRPM);
      delay(500);
      stepperCutter.runToNewPosition(-cutterSteps);
      stepperCutter.runToNewPosition(200);
      setCutterServoRPM(0);
      delay(1000);
      disableCutterServo();
      disableExternalPower();

      delay(500);
    } else {
      processingFlag = 0;
      myNex.writeStr("page 7");
    }

  }

  stepperFeeder.run();
  stepperCutter.run();
  myNex.NextionListen();
}

//Length -
void trigger0() {

  if (currentPage == 1)
    if (lengthVar > -1) {
      lengthVar--;
    }
  myNex.writeStr("t0.txt", String(lengthVar));


  if (currentPage == 5) {
    Serial.println("Start cut rotation");
    enableCutterServo();
    enableExternalPower();
    
    delay(100);
    setCutterServoRPM(cutterServoRPM);
    delay(500);
    Serial.println("Move table up");
    stepperCutter.runToNewPosition(-cutterSteps);
    Serial.println("Move table down");
    stepperCutter.runToNewPosition(200);
    Serial.println("Stop cut rotation");
    setCutterServoRPM(0);
    delay(1000);
    disableCutterServo();
    disableExternalPower();

    delay(500);
  }

  if (debug) {
    Serial.println("Button pressed: Length -");
  }
}

//Length +
void trigger1() {

  lengthVar++;
  myNex.writeStr("t0.txt", String(lengthVar));

  if (debug) {
    Serial.println("Button pressed: Length +");
  }
}

//Quantity -
void trigger2() {

  if (quantityVar >= 0) {
    quantityVar--;
  }
  myNex.writeStr("t1.txt", String(quantityVar));

  if (debug) {
    Serial.println("Button pressed: Quantity -");
  }
}

//Quantity +
void trigger3() {

  quantityVar++;
  myNex.writeStr("t1.txt", String(quantityVar));

  if (debug) {
    Serial.println("Button pressed: Quantity +");
  }
}

//Start button on setting screen
void trigger4() {
  delay(100);
  myNex.writeStr("page 3");
  homeCutter();
  delay(100);
  myNex.writeStr("page 4");
  if (debug) {
    Serial.println("Button pressed: Start button");
  }
}

//Return button in initialising screen
void trigger5() {
  if (debug) {
    Serial.println("Button pressed: Return button in init screen");
  }
  myNex.writeStr("page 2");
}

//Feed push button
void trigger6() {
  if (debug) {
    Serial.println("Button pressed: Feed push");
  }
  stepperFeeder.move(200000);
  if (debug) {
    Serial.println("Button pressed: Feed +");
  }
}

//Feed release button
void trigger22() {
  if (debug) {
    Serial.println("Button pressed: Feed release");
  }
  stepperFeeder.stop();

}

//Ref cut
void trigger7() {
  if (debug) {
    Serial.println("Button pressed: Ref cut");
  }

  setCutterServoRPM(200);
  delay(500);
  moveCutTableUp();
  delay(3000);
  moveCutTableDown();
  delay(3000);
  setCutterServoRPM(0);
  delay(500);


}

//Ref cut page next
void trigger8() {
  myNex.writeStr("page 6");
  if (debug) {
    Serial.println("Button pressed: Ref cut next");
  }
  delay(200);
  myNex.writeStr("page 6");
  processingCount = 0;
  stepperFeeder.setCurrentPosition(0);
  processingFlag = 1;

}

//Stop button at processing page
void trigger9() {
  if (debug) {
    Serial.println("Button pressed: Stop button");
  }
}

//Pause button at processing page
void trigger10() {
  if (debug) {
    Serial.println("Button pressed: Pause button");
  }
}

//Rotate -
void trigger11() {
  if (debug) {
    Serial.println("Button pressed: Rotate - ");
  }
  setStraightenerServoRPM(0);
  delay(1000);
  disableStraightenerServo();

}

//Rotate +
void trigger12() {
  if (debug) {
    Serial.println("Button pressed: Rotate +");
  }
  enableStraightenerServo();
  delay(100);
  setStraightenerServoRPM(straightenerServoRPM);
}

//Cut table -
void trigger13() {
  if (debug) {
    Serial.println("Button pressed: Cut table -");
  }

  moveCutTableDown();
}

//Cut table +
void trigger14() {
  if (debug) {
    Serial.println("Button pressed: Cut table +");
  }

  moveCutTableUp();
}

//Feed -
void trigger15() {
  stepperFeeder.move(-20000);

  if (debug) {
    Serial.println("Button pressed: Feed -");
  }
}

//Feed +
void trigger16() {
  stepperFeeder.move(60000);
  if (debug) {
    Serial.println("Button pressed: Feed +");
  }
}

//Cut push button
void trigger17() {
  if (debug) {
    Serial.println("Button pressed: Cut push");
  }
}

//Cut released button
void trigger23() {
  if (debug) {
    Serial.println("Button pressed: Cut released ");
  }

  if (cutActive == false) {
    enableCutterServo();
    delay(100);
    setCutterServoRPM(cutterServoRPM);
    cutActive = true;
  } else {
    setCutterServoRPM(0);
    delay(1000);
    disableCutterServo();
    cutActive = false;
  }
}


//Vacuum
void trigger18() {
  if (debug) {
    Serial.println("Button pressed: Vaccuum on / off");
  }
}

//Home cutter table
void trigger19() {
  if (debug) {
    Serial.println("Button pressed: Home cutter table");
  }
  homeCutter();
}

//Reset replace disc
void trigger20() {
  if (debug) {
    Serial.println("Button pressed: Reset at replace disc screen");
  }
}

//Reset after error
void trigger21() {
  processingFlag = 0;
  resetSafety();
  setSafetyStandby();
  myNex.writeStr("page 1");
  setCutterServoRPM(0);
  setStraightenerServoRPM(0);

  delay(100);

  errorFlag = 0;




  if (debug) {
    Serial.println("Button pressed: Reset at error screen");
  }
}

void resetSafety() {
  digitalWrite(safetyRelay, HIGH);
  delay(100);
  digitalWrite(safetyRelay, LOW);
}

void setSafetyStandby() {
  digitalWrite(safetyStandbyRelay, HIGH);
}

void enableCutterServo() {
  digitalWrite(cutterServoEnabled, HIGH);
}

void enableStraightenerServo() {
  digitalWrite(straightenerServoEnabled, HIGH);
}

void disableCutterServo() {
  digitalWrite(cutterServoEnabled, LOW);
}

void disableStraightenerServo() {
  digitalWrite(straightenerServoEnabled, LOW);
}

void enableExternalPower() {
  digitalWrite(externalPowerOutlet, HIGH);
}

void disableExternalPower() {
  digitalWrite(externalPowerOutlet, LOW);
}

int readCutterServoRPM() {
  return modbus.uint16FromRegister(0x03, 1539, bigEndian);
}

int readStraightenerServoRPM() {
  return modbus2.uint16FromRegister(0x03, 1539, bigEndian);
}

void setCutterServoRPM(int rpm) {
  modbus.uint16ToRegister(1539, rpm, bigEndian);
}

void setStraightenerServoRPM(int rpm) {
  modbus2.uint16ToRegister(1539, rpm, bigEndian);
}

void homeCutter() {
  stepperCutterHomedFlag = 0;
  stepperCutterSafetyFlag = 0;
  Serial.print("stepperCutterHomedFlag: ");
  Serial.println(stepperCutterHomedFlag);
  Serial.print("stepperCutterSafetyFlag: ");
  Serial.println(stepperCutterSafetyFlag);
  while (!stepperCutterHomedFlag) {
    if (!stepperCutterSafetyFlag) {
      if (digitalRead(cuttingTableBottomSensor)) {
        stepperCutter.setCurrentPosition(0);
        stepperCutter.runToNewPosition(-1500);
        stepperCutterSafetyFlag = 1;
      } else {
        stepperCutterSafetyFlag = 1;
      }
    } else {
      if (!digitalRead(cuttingTableBottomSensor)) {
        stepperCutter.setMaxSpeed(600);
        stepperCutter.move(10000);
      }
      if (digitalRead(cuttingTableBottomSensor)) {
        stepperCutter.stop();
        stepperCutter.setCurrentPosition(0);
        stepperCutter.setMaxSpeed(cutterMaxSpeedSetting);
        stepperCutter.move(-200);
        stepperCutterSafetyFlag = 0;
        stepperCutterHomedFlag = 1;
      }
    }
    stepperCutter.run();
  }
}

void moveCutTableUp() {
  if (stepperCutterHomedFlag) {
    stepperCutter.move(-cutterSteps);
  } else {
    Serial.println("Stepper not homed");
  }
}

void moveCutTableDown() {
  if (stepperCutterHomedFlag) {
    stepperCutter.move(cutterSteps);
  } else {
    Serial.println("Stepper not homed");
  }
}
