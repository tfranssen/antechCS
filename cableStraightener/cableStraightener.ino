#include <AccelStepper.h>
#include <Controllino.h>
#include "EasyNextionLibrary.h"
#include "SensorModbusMaster.h"
#include <Encoder.h>
#include "RingEEPROM.h"

#define iSV2servoDrive 1
#define inches 1

// Stepper cutter settings
// Current: 4.0A RMS, Full Current
// Pulses / Rev: 1000
// SW1: Off
// SW2: Off
// SW3: Off
// SW4: On (full current)
// SW5: On
// SW6: On
// SW7: On
// SW8: Off

// Stepper feeder settings
// Current: 2.8 (2,7)A RMS, Full Current
// Pulses / Rev: 1000
// SW1: On
// SW2: On
// SW3: Off
// SW4: On (full current)
// SW5: On
// SW6: On
// SW7: On
// SW8: Off

EasyNex myNex(Serial2);

int cutterServoRPM = 1150;
int straightenerServoRPM = 1400;
int cutterSteps = 7250;
int cutterMaxSpeedSettingDefault = 7;  // Divided by 100 because of multiplication of percentage
int cutterMaxSpeedSettingPercentage = 100;
int cutterMaxSpeedSetting = cutterMaxSpeedSettingDefault * cutterMaxSpeedSettingPercentage;
int cutterMaxSpeedSettingDown = 4000;  // Dit is de neergaande beweging
#ifdef inches
float stepsPerMM = 1524;                   // This is the number of steps on the stepper motors for feeding 1 mm of cable
float pulsesPerMM = 1224.28;               // This is the number of pulses on the rotary encoder for 1 mm of cable
#else
float stepsPerMM = 60;                   // This is the number of steps on the stepper motors for feeding 1 mm of cable
float pulsesPerMM = 48.2;               // This is the number of pulses on the rotary encoder for 1 mm of cable
#endif
unsigned int delayBeforeFeeding = 500;  // Delay in ms between starting straigther and start feeding
unsigned int delayAfterFeeding = 1000;  // Delay in ms between starting straigther and start feeding
unsigned int delayBeforeCutting = 100;  // Delay in ms between starting the cutter and moving the table
unsigned int delayAfterCutting = 100;   // Delay in ms between stopping the cutter and turning off the vacuum and power to servo (brake)

// Rotary encoder settings
Encoder myEnc(CONTROLLINO_IN0, CONTROLLINO_IN1);

long rotaryCount = 0;
long lastRotaryCount = 0;

#define safetyRelay CONTROLLINO_R0          // Relay to release safety
#define externalPowerOutlet CONTROLLINO_R2  // Relay for vaccuum
#define safetyStandbyRelay CONTROLLINO_R3   // Standby mode for safety
#define lightRelay CONTROLLINO_R4           // Relay for light

#define cutterServoEnabled CONTROLLINO_R12  // Cutter servo enabled relay
#define cutterServoSpeed1 CONTROLLINO_R13   // Cutter servo enabled relay

#define straightenerServoEnabled CONTROLLINO_R14  // straightener servo enabled relay
#define straightenerServoSpeed1 CONTROLLINO_R15

#define cuttingTableBottomSensor CONTROLLINO_A3
#define cuttingTableTopSensor CONTROLLINO_A5

#define ledGreenPin CONTROLLINO_D10
#define ledRedPin CONTROLLINO_D11

#define safetyRelayInput CONTROLLINO_A1
#define safetyButtonInput CONTROLLINO_A2
#define cuttingDiskVisibleSensor CONTROLLINO_A7

bool safetyRelayStatus = false;
bool safetyButtonStatus = false;

bool pneumaticActive = false;

// Modbus settings
#ifdef iSV2servoDrive
long modbusBaudRate = 19200;
#else
long modbusBaudRate = 9600;
#endif

const int DEREPin = CONTROLLINO_RS485_DE;
HardwareSerial *modbusSerial = &Serial3;
modbusMaster modbus;
modbusMaster modbus2;
int16_t cutterRPMRead = 0;

// Proces vars
float lengthVar = 1000;
unsigned int quantityVar = 10;
bool errorFlag = 0;
bool processingFlag = 0;
bool stopButtonFlag = 0;
bool homeCutterFlag = 0;
bool homeCutterInProduction = 0;
unsigned int processingCount = 0;
int processingStep = 0;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

// Feeder stepper PINs
#define stepperFeederStepPin CONTROLLINO_D0
#define stepperFeerderDirPin CONTROLLINO_D1
#define stepperFeederEnablePin CONTROLLINO_D2  // Check if this is correct, if the steppers go the wrong way change to CONTROLLINO_D3

// Cutter table stepper PINs
#define stepperCutterStepPin CONTROLLINO_D4
#define stepperCutterDirPin CONTROLLINO_D5
#define stepperCutterEnablePin CONTROLLINO_D6
#define motorInterfaceType 1

AccelStepper stepperFeeder = AccelStepper(motorInterfaceType, stepperFeederStepPin, stepperFeerderDirPin);
AccelStepper stepperCutter = AccelStepper(motorInterfaceType, stepperCutterStepPin, stepperCutterDirPin);

int feederMaxSpeedSetting = 5000;
int feederCorrectionMaxSpeedSetting = 1000;
int feederAccel = 5000;
int feederCorrectionAccel = 2000;
int cutterAccel = 10000;
int currentPage = 0;
int lastPage = 0;
bool debug = true;
bool cutActive = false;
bool cutterTableMoveUp = 0;

String processingStatus = "";

bool stepperCutterHomedFlag = 0;
bool stepperCutterSafetyFlag = 0;

int homeCutterState = 0;

bool refCutFlag = false;
enum CutState {
  INIT,
  MOVE_TABLE_UP,
  CHECK_MOVE_TABLE_UP,
  MOVE_TABLE_DOWN,
  CHECK_MOVE_TABLE_DOWN,
  STOP_CUT_ROTATION,
  FINISHED
};

// Initial state
CutState refCutStep = INIT;
unsigned long lastMillis = 0;

// EEPROM settings
#define FIRST_ADDR 0
#define BUFFER_SIZE 4
#define PARAM_PACKET_SIZE 6

struct cutCounter {
  int cuts;
  int totalCuts;
};

cutCounter counterWrite, counterRead;

byte writeBuffer[PARAM_PACKET_SIZE];
byte readBuffer[PARAM_PACKET_SIZE];

RingEEPROM myeepRom(FIRST_ADDR, BUFFER_SIZE, sizeof(cutCounter));

void setup() {
  // PINs settings
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
  pinMode(CONTROLLINO_R11, OUTPUT);
  pinMode(CONTROLLINO_R12, OUTPUT);
  pinMode(CONTROLLINO_R13, OUTPUT);
  pinMode(CONTROLLINO_R14, OUTPUT);
  pinMode(CONTROLLINO_R15, OUTPUT);

  pinMode(CONTROLLINO_A0, INPUT);
  pinMode(CONTROLLINO_A1, INPUT);
  pinMode(CONTROLLINO_A2, INPUT);
  pinMode(cuttingDiskVisibleSensor, INPUT);
  pinMode(cuttingTableBottomSensor, INPUT);
  pinMode(cuttingTableTopSensor, INPUT);

  // Nextion setup
  myNex.begin(9600);
  delay(100);
  myNex.writeStr("page 0");
  Serial.begin(115200);

  resetSafety();
  setSafetyStandby();
  delay(2000);
  // Modbus settings
  pinMode(DEREPin, OUTPUT);
  Serial3.begin(modbusBaudRate);
  modbus.begin(0x01, modbusSerial, DEREPin);   // Cutter servo
  modbus2.begin(0x02, modbusSerial, DEREPin);  // Straigtner servo
  delay(50);


  // COMMENT FOR TESTING WITOUT MACHINE!!!
  // Read RPM from cutter servo to detect if the program can communicatate with the motor
  Serial.print("Searching motor ");
  while (cutterRPMRead == 0) {
    cutterRPMRead = modbus.int16FromRegister(0x03, 0x053F);
    Serial.print("Modbus readout: ");
    Serial.println(cutterRPMRead);
    delay(50);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Motor detected, RPM: " + String(cutterRPMRead));
  delay(50);

  // // For servo 1 (cutter)
  // modbus.int16ToRegister(0x0003, 1);
  // modbus.int16ToRegister(0x0309, 3);
  // modbus.int16ToRegister(0x0309, 0);
  // modbus.int16ToRegister(0x0033, 0x2211);

  // // For servo 2 (straightener)
  // modbus2.int16ToRegister(0x0003, 1); // Set to velocity mode
  // modbus2.int16ToRegister(0x0309, 3); // Set to speed mode
  // modbus2.int16ToRegister(0x0309, 0); // Set to 0 RPM
  // modbus2.int16ToRegister(0x0033, 0x2211); // Save to EEPROM


  stepperFeeder.setMaxSpeed(feederMaxSpeedSetting);
  stepperFeeder.setAcceleration(feederAccel);
  stepperFeeder.enableOutputs();

  stepperCutter.setMaxSpeed(cutterMaxSpeedSetting);
  stepperCutter.setAcceleration(cutterAccel);
  stepperCutter.enableOutputs();

  // Light on
  digitalWrite(lightRelay, HIGH);

  delay(200);
  Serial.println("Setup complete");
  digitalWrite(CONTROLLINO_D10, HIGH);
  digitalWrite(CONTROLLINO_D11, HIGH);
  lastMillis = millis();  // Initialize the timer

  // First time
  //  counterWrite.cuts = 0;
  //  myeepRom.savePacket((byte*)&counterWrite);

  // Read latest cut count
  myeepRom.readPacket((byte *)&counterRead);
  Serial.println("Number of cuts: " + String(counterRead.cuts));
  Serial.println("Number of cuts total: " + String(counterRead.totalCuts));
}

void loop() {
  currentMillis = millis();
  unsigned long interval = 0;
  currentPage = myNex.currentPageId;

  safetyRelayStatus = digitalRead(safetyRelayInput);
  safetyButtonStatus = digitalRead(safetyButtonInput);
  //!!!!Changed to test without machine!!!!!!!
  // safetyRelayStatus = 1;
  // safetyButtonStatus = 0;

  if (safetyButtonStatus && currentPage != 14) {
    Serial.println("Safety button activated");
    myNex.writeStr("page 14");
    delay(100);
    myNex.writeStr("t5.txt", "Emergency button activated");
    disableExternalPower();
    processingStep = 0;
    processingFlag = false;
    errorFlag = 1;
    refCutFlag = 0;
    stepperCutter.stop();
  }

  if (!safetyRelayStatus && currentPage != 14) {
    Serial.println("Safety relay activated");
    myNex.writeStr("page 14");
    delay(100);
    myNex.writeStr("t5.txt", "Safety activated");
    disableExternalPower();
    processingStep = 0;
    processingFlag = false;
    errorFlag = 1;
    refCutFlag = 0;
    stepperCutter.stop();
  }

  if (currentPage != lastPage) {
    lastPage = currentPage;
    Serial.print("Current page: ");
    Serial.println(currentPage);

    if (currentPage == 2) {
      delay(200);
      myNex.writeStr("t0.txt", String(lengthVar,1));
      delay(50);
      myNex.writeStr("t1.txt", String(quantityVar));
    }

    if (currentPage == 15) {
      delay(250);
      myNex.writeStr("t6.txt", String(cutterServoRPM * 3));
      delay(50);
      myNex.writeStr("t7.txt", String(cutterMaxSpeedSettingPercentage));
    }

    if (currentPage == 12) {
      myeepRom.readPacket((byte *)&counterRead);
      Serial.println("Number of cuts: " + String(counterRead.cuts));
      delay(250);

      myNex.writeStr("t5.txt", "This disk: " + String(counterRead.cuts) + ". Total: " + String(counterRead.totalCuts) + ".");
    }
  }

  if (refCutFlag && !errorFlag) {
    switch (refCutStep) {
      case INIT:
        // Initial state
        Serial.println("Start cut rotation");
        enableExternalPower();
        enableCutterServo();
        setCutterSpeed1ON();
        interval = 5000;
        if (currentMillis - lastMillis >= interval) {
          lastMillis = currentMillis;
          refCutStep = MOVE_TABLE_UP;  // Move to the new state
        }
        break;

      case MOVE_TABLE_UP:
        // Moving the table up state
        Serial.println("Move table up");
        stepperCutter.moveTo(-cutterSteps);
        refCutStep = CHECK_MOVE_TABLE_UP;
        break;

      case CHECK_MOVE_TABLE_UP:
        // Check if the move is completed
        if (stepperCutter.distanceToGo() == 0) {
          refCutStep = MOVE_TABLE_DOWN;
        }
        break;

      case MOVE_TABLE_DOWN:
        // Moving the table down state
        Serial.println("Move table down");
        stepperCutter.moveTo(0);
        refCutStep = CHECK_MOVE_TABLE_DOWN;
        break;

      case CHECK_MOVE_TABLE_DOWN:
        // Check if the move is completed
        if (stepperCutter.distanceToGo() == 0) {
          // setCutterServoRPM(0);
          setCutterSpeed1OFF();
          refCutStep = STOP_CUT_ROTATION;
          lastMillis = currentMillis;
        }
        break;

      case STOP_CUT_ROTATION:
        // Stop cut rotation state
        interval = 1000;
        if (currentMillis - lastMillis >= interval) {
          Serial.println("Stop cut rotation");
          disableCutterServo();
          disableExternalPower();
          refCutStep = FINISHED;
        }
        break;

      case FINISHED:
        // Finished state - you can decide if you want to stay here or reset to INIT
        // refCutStep = INIT; // Uncomment this line to reset to INIT state
        refCutFlag = false;
        break;
    }
  }

  if (homeCutterFlag && !errorFlag) {
    switch (homeCutterState) {
      case 0:
        stepperCutterHomedFlag = 0;
        stepperCutterSafetyFlag = 0;
        Serial.println("HomeCutter: Starting homing process");
        homeCutterState = 1;
        break;

      case 1:
        if (digitalRead(cuttingTableBottomSensor)) {
          stepperCutter.moveTo(-1150);
          homeCutterState = 2;
          Serial.println("HomeCutter: Sensor active, moving cutter to start position.");
        } else {
          homeCutterState = 3;
          Serial.println("HomeCutter: Sensor not active, skipping to step 3.");
        }
        break;

      case 2:
        if (stepperCutter.distanceToGo() == 0) {
          homeCutterState = 3;
          Serial.println("HomeCutter: Cutter at starting position, proceeding to step 3.");
        }
        break;

      case 3:
        stepperCutter.move(10000);
        homeCutterState = 4;
        Serial.println("HomeCutter: Moving cutter down.");
        break;

      case 4:
        if (digitalRead(cuttingTableBottomSensor)) {
          homeCutterState = 5;
          Serial.println("HomeCutter: Waiting for sensor to become active again.");
        }
        break;

      case 5:
        if (digitalRead(cuttingTableBottomSensor)) {
          // Set the current position as the new zero position
          stepperCutter.setCurrentPosition(0);
          stepperCutterHomedFlag = 1;
          homeCutterState = 0;  // Reset for next use
          Serial.println("HomeCutter: Homing complete, resetting flags and positions.");
          stepperCutter.setMaxSpeed(cutterMaxSpeedSetting);
          homeCutterFlag = false;
          if (homeCutterInProduction) {
            // Check if the cutting disk is visible using the sensor
            if (digitalRead(cuttingDiskVisibleSensor)) {
              Serial.println("HomeCutter: Cutting disk is not visible.");
              processingStep = 0;
              processingFlag = false;
              errorFlag = 1;
              stepperCutter.setCurrentPosition(0);
              myNex.writeStr("page 14");
              delay(100);
              myNex.writeStr("t5.txt", "Cutting disk is not visible");
              break;
            } else {
              myNex.writeStr("page 4");
              homeCutterInProduction = false;
            }
          }
        }
        break;
    }
  }

  if (processingFlag && !errorFlag) {
    if (processingCount < quantityVar) {
      switch (processingStep) {
        case 0:
          // Step 0: Initialize processing
          Serial.println("Start processing");
          processingStatus = String(processingCount + 1) + " / " + String(quantityVar);
          myNex.writeStr("t2.txt", processingStatus);

          // Enable servo drivers
          enableStraightenerServo();
          enableCutterServo();

          // Set servo RPMs
          setCutterServoRPM(cutterServoRPM);
          setStraightenerServoRPM(straightenerServoRPM);

          setCutterSpeed1ON();
          myEnc.readAndReset();
          processingStep++;
          // Serial.println("Next step");
          break;

        case 1:
          // Step 1: Wait for 100 ms to set the RPM of the straightener servo
          if (currentMillis - previousMillis >= 100) {
            Serial.println("Turn on straightener");
            previousMillis = currentMillis;
            setStraightenerSpeed1ON();
            processingStep++;
            // Serial.println("Next step");
          }
          break;

        case 2:
          // Step 2: Wait for the straigtener to be on full speed
          if (currentMillis - previousMillis >= delayBeforeFeeding) {
            Serial.println("Turn on feeder");
            previousMillis = currentMillis;
            stepperFeeder.move(stepsPerMM * lengthVar);
            Serial.println("Move feeder: " + String(stepsPerMM * lengthVar) + " steps");
            Serial.println("Expected pulses: " + String(pulsesPerMM * lengthVar) + " pulses");
            processingStep++;
            lastMillis = millis();
          }
          break;

        case 3:
          // Step 3 checks if feeding is done, if real distance is smaller then expected move again until distance is reached
          if (stepperFeeder.distanceToGo() == 0) {
            if (-myEnc.read() < (pulsesPerMM * lengthVar)) {
              Serial.println("Rotary pulses: " + String(-myEnc.read()));
              float feedCorrection = (((pulsesPerMM * lengthVar) - -myEnc.read()) * (stepsPerMM / pulsesPerMM));
              if (feedCorrection >= 300) {
                stepperFeeder.setMaxSpeed(feederCorrectionMaxSpeedSetting);
                stepperFeeder.setAcceleration(feederCorrectionAccel);
                // Serial.println("Feed correction: " + String(int(round(feedCorrection))) + " steps");
                stepperFeeder.move(int(round(feedCorrection)));
              } else {
                // Serial.println("Feed correction: 1 steps");
                stepperFeeder.move(1);
              }
            } else {
              stepperFeeder.setMaxSpeed(feederMaxSpeedSetting);
              stepperFeeder.setAcceleration(feederAccel);
              setStraightenerSpeed1OFF();
              Serial.println("Rotary pulses after transport: " + String(-myEnc.read()));
              previousMillis = currentMillis;
              processingStep++;
            }
          }
          // Dynamical time out if feeding fails
          if (currentMillis - lastMillis >= (lengthVar * 200)) {
            Serial.println("Feeding failed (timeout)");
            processingStep = 0;
            processingFlag = false;
            errorFlag = 1;
            disableExternalPower();
            setStraightenerSpeed1OFF();
            setCutterSpeed1OFF();
            stepperFeeder.setCurrentPosition(0);
            delay(500);
            disableCutterServo();
            disableStraightenerServo();
            delay(500);
            myNex.writeStr("page 14");
            delay(100);
            myNex.writeStr("t5.txt", "Feeding failed (timeout)");
          }
          break;

        case 4:
          if (currentMillis - previousMillis >= delayAfterFeeding) {
            Serial.println("Rotary pulses after transport + delay: " + String(-myEnc.read()));
            disableStraightenerServo();
            enableExternalPower();
            previousMillis = currentMillis;
            processingStep++;
          }
          break;

        case 5:
          if (currentMillis - previousMillis >= 100) {
            previousMillis = currentMillis;
            processingStep++;
          }
          break;

        case 6:
          if (currentMillis - previousMillis >= delayBeforeCutting) {
            stepperCutter.setMaxSpeed(cutterMaxSpeedSetting);
            Serial.println("Rotary pulses before cut: " + String(-myEnc.read()));
            if (digitalRead(cuttingDiskVisibleSensor)) {
              Serial.println("HomeCutter: Cutting disk is not visible.");
              processingStep = 0;
              processingFlag = false;
              errorFlag = 1;
              disableExternalPower();
              disableCutterServo();
              disableStraightenerServo();
              myNex.writeStr("page 14");
              delay(100);
              myNex.writeStr("t5.txt", "Cutting disk is not visible");
              break;
            }
            stepperCutter.moveTo(-cutterSteps);
            previousMillis = currentMillis;
            processingStep++;
          }
          break;

        case 7:
          if (stepperCutter.distanceToGo() == 0) {
            stepperCutter.setMaxSpeed(cutterMaxSpeedSettingDown);
            stepperCutter.moveTo(0);
            previousMillis = currentMillis;
            processingStep++;
          }
          break;

        case 8:
          if (stepperCutter.distanceToGo() == 0) {
            stepperCutter.setMaxSpeed(cutterMaxSpeedSetting);
            myeepRom.readPacket((byte *)&counterRead);
            counterWrite.cuts = counterRead.cuts + 1;
            counterWrite.totalCuts = counterRead.totalCuts + 1;
            myeepRom.savePacket((byte *)&counterWrite);

            if (debug) {
              myeepRom.readPacket((byte *)&counterRead);
              Serial.println("Number of cuts: " + String(counterRead.cuts));
            }
            previousMillis = currentMillis;
            processingStep++;
          }
          break;

        case 9:
          if (currentMillis - previousMillis >= delayAfterCutting) {
            Serial.println("Rotary pulses after cut: " + String(-myEnc.read()));
            disableExternalPower();
            previousMillis = currentMillis;
            processingStep = 0;
            processingCount++;
            lastMillis = millis();
            if (stopButtonFlag) {
              processingCount = quantityVar;
              stopButtonFlag = 0;
            }
          }
          break;
      }
    } else {
      setCutterSpeed1OFF();
      if (currentMillis - lastMillis >= 3000) {
        disableCutterServo();
        myNex.writeStr("page 7");
        processingFlag = 0;
      }
    }
  }

  stepperFeeder.run();
  stepperCutter.run();
  myNex.NextionListen();
}

// Length -
void trigger0() {

  if (currentPage == 2) {
    if (lengthVar >= 0) {
      lengthVar--;
    }
    myNex.writeStr("t0.txt", String(lengthVar));
  }

  if (debug) {
    Serial.println("Button pressed: Length -");
  }
}

// Length +
void trigger1() {

  lengthVar++;
  myNex.writeStr("t0.txt", String(lengthVar));

  if (debug) {
    Serial.println("Button pressed: Length +");
  }
}

// Quantity -
void trigger2() {

  if (quantityVar >= 0) {
    quantityVar--;
  }
  myNex.writeStr("t1.txt", String(quantityVar));

  if (debug) {
    Serial.println("Button pressed: Quantity -");
  }
}

// Quantity +
void trigger3() {

  quantityVar++;
  myNex.writeStr("t1.txt", String(quantityVar));

  if (debug) {
    Serial.println("Button pressed: Quantity +");
  }
}

// Start button on setting screen
void trigger4() {
  delay(100);
  myNex.writeStr("page 3");
  homeCutter();
  homeCutterInProduction = true;
  if (debug) {
    Serial.println("Button pressed: Start button");
  }
}

// Return button in initialising screen
void trigger5() {
  if (debug) {
    Serial.println("Button pressed: Return button in init screen");
  }
  myNex.writeStr("page 2");
}

// Feed push button
void trigger6() {
  if (debug) {
    Serial.println("Button pressed: Feed push");
  }
  enableStraightenerServo();
  // setStraightenerServoRPM(straightenerServoRPM);
  setStraightenerSpeed1ON();
  delay(delayBeforeFeeding);
  stepperFeeder.move(200000);
}

// Feed release button
void trigger22() {
  if (debug) {
    Serial.println("Button pressed: Feed release");
  }
  // setStraightenerServoRPM(0);
  setStraightenerSpeed1OFF();
  delay(delayAfterFeeding);
  disableStraightenerServo();
  stepperFeeder.setCurrentPosition(0);
}

// Ref cut
void trigger7() {
  if (debug) {
    Serial.println("Button pressed: Ref cut");
  }
  refCutStep = INIT;
  lastMillis = currentMillis;
  refCutFlag = true;
}

// Ref cut page next
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

// Stop button at processing page
void trigger9() {
  if (debug) {
    Serial.println("Button pressed: Stop button");
  }

  if (stopButtonFlag == 0) {
    stopButtonFlag = 1;
    processingStatus = String(processingCount + 1) + " / " + String(quantityVar) + String(" (stopping)");
    myNex.writeStr("t2.txt", processingStatus);
  }
}

// Pause button at processing page
void trigger10() {
  if (debug) {
    Serial.println("Button pressed: Pause button");
  }
}

// Rotate -
void trigger11() {
  if (debug) {
    Serial.println("Button pressed: Rotate - ");
  }
  // setStraightenerServoRPM(0);
  setStraightenerSpeed1OFF();
  disableStraightenerServo();
}

// Rotate +
void trigger12() {
  if (debug) {
    Serial.println("Button pressed: Rotate +");
  }
  enableStraightenerServo();
  setStraightenerSpeed1ON();
}

// Cut table -
void trigger13() {
  if (debug) {
    Serial.println("Button pressed: Cut table -");
  }

  moveCutTableDown();
}

// Cut table +
void trigger14() {
  if (debug) {
    Serial.println("Button pressed: Cut table +");
  }

  moveCutTableUp();
}

// Feed -
void trigger15() {
  stepperFeeder.move(-2000);

  if (debug) {
    Serial.println("Button pressed: Feed -");
  }
}

// Feed +
void trigger16() {
  stepperFeeder.move(2000);
  if (debug) {
    Serial.println("Button pressed: Feed +");
  }
}

// Cut push button
void trigger17() {
  if (debug) {
    Serial.println("Button pressed: Cut push");
  }
}

// Cut released button
void trigger23() {
  if (debug) {
    Serial.println("Button pressed: Cut released ");
  }
  if (cutActive == false) {
    enableCutterServo();
    delay(100);
    // setCutterServoRPM(cutterServoRPM);
    setCutterSpeed1ON();
    cutActive = true;
  } else {
    // setCutterServoRPM(0);
    setCutterSpeed1OFF();
    delay(1000);
    disableCutterServo();
    cutActive = false;
  }
}

// Vacuum
void trigger18() {
  if (debug) {
    Serial.println("Button pressed: Vaccuum on / off");
  }
}

// Home cutter table
void trigger19() {
  if (debug) {
    Serial.println("Button pressed: Home cutter table");
  }
  homeCutter();
}

// Reset replace disc
void trigger20() {
  if (debug) {
    Serial.println("Button pressed: Reset at replace disc screen");
  }
  counterWrite.cuts = 0;
  myeepRom.savePacket((byte *)&counterWrite);
  myeepRom.readPacket((byte *)&counterRead);
  Serial.println("Number of cuts: " + String(counterRead.cuts));
  delay(250);
  myNex.writeStr("t5.txt", "This disk: " + String(counterRead.cuts) + ". Total: " + String(counterRead.totalCuts) + ".");
}

// Reset after error
void trigger21() {
  processingFlag = 0;
  refCutFlag = 0;
  refCutStep = INIT;
  processingStep = 0;
  processingCount = 0;
  stopButtonFlag = 0;
  resetSafety();
  setSafetyStandby();
  myNex.writeStr("page 1");
  // setCutterServoRPM(0);
  setCutterSpeed1OFF();
  // setStraightenerServoRPM(0);
  setStraightenerSpeed1OFF();
  stepperFeeder.setMaxSpeed(feederMaxSpeedSetting);
  stepperCutter.setMaxSpeed(cutterMaxSpeedSetting);
  delay(100);
  errorFlag = 0;

  if (debug) {
    Serial.println("Button pressed: Reset at error screen");
  }
}

// RPM -
void trigger24() {
  cutterServoRPM = cutterServoRPM - 10;
  myNex.writeStr("t6.txt", String(cutterServoRPM * 3));
  if (debug) {
    Serial.println("Button pressed: RPM -, new RPM: " + String(cutterServoRPM));
  }
}

// RPM +
void trigger25() {
  cutterServoRPM = cutterServoRPM + 10;
  myNex.writeStr("t6.txt", String(cutterServoRPM * 3));
  if (debug) {
    Serial.println("Button pressed: RPM +, new RPM: " + String(cutterServoRPM));
  }
}

// Lin speed -
void trigger26() {
  cutterMaxSpeedSettingPercentage--;
  cutterMaxSpeedSetting = cutterMaxSpeedSettingDefault * cutterMaxSpeedSettingPercentage;
  stepperCutter.setMaxSpeed(cutterMaxSpeedSetting);
  myNex.writeStr("t7.txt", String(cutterMaxSpeedSettingPercentage));
  if (debug) {
    Serial.println("Button pressed: Lin speed -, new speed: " + String(cutterMaxSpeedSetting));
  }
}

// Lin speed +
void trigger27() {
  cutterMaxSpeedSettingPercentage++;
  cutterMaxSpeedSetting = cutterMaxSpeedSettingDefault * cutterMaxSpeedSettingPercentage;
  stepperCutter.setMaxSpeed(cutterMaxSpeedSetting);
  myNex.writeStr("t7.txt", String(cutterMaxSpeedSettingPercentage));
  if (debug) {
    Serial.println("Button pressed: Lin speed +, new speed: " + String(cutterMaxSpeedSetting));
  }
}

// Length setting
void trigger28() {
  String length = myNex.readStr("t0.txt");
  float lengthNum = length.toFloat();
  lengthNum = round(lengthNum * 10) / 10.0;
  lengthVar = lengthNum;
  myNex.writeStr("page 2");
}

// Quantity setting
void trigger29() {
  String quantity = myNex.readStr("t0.txt");
  unsigned int quantityNum = quantity.toInt();
  quantityVar = quantityNum;
  myNex.writeStr("page 2");
}

// Pneumatic
void trigger30() {
  Serial.println("Pneumatic button pressed");
  if (pneumaticActive) {
    digitalWrite(CONTROLLINO_R5, LOW);
    pneumaticActive = false;
  } else {
    digitalWrite(CONTROLLINO_R5, HIGH);
    pneumaticActive = true;
  
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

void setStraightenerSpeed1ON() {
  digitalWrite(straightenerServoSpeed1, LOW);
}

void setStraightenerSpeed1OFF() {
  digitalWrite(straightenerServoSpeed1, HIGH);
}

void setCutterSpeed1ON() {
  digitalWrite(cutterServoSpeed1, HIGH);
}

void setCutterSpeed1OFF() {
  digitalWrite(cutterServoSpeed1, LOW);
}

void enableExternalPower() {
  digitalWrite(externalPowerOutlet, HIGH);
}

void disableExternalPower() {
  digitalWrite(externalPowerOutlet, LOW);
}

void homeCutter() {
  stepperCutterHomedFlag = false;
  homeCutterState = 0;
  homeCutterFlag = true;
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

void setCutterServoRPM(int RPM) {
#ifdef iSV2servoDrive
  modbus.int16ToRegister(0x0309, RPM * -1);
#else
  modbus.int16ToRegister(779, RPM * -1);
#endif
}

void setStraightenerServoRPM(int RPM) {
#ifdef iSV2servoDrive
  modbus2.int16ToRegister(0x0309, RPM * -1);
#else
  modbus2.int16ToRegister(779, RPM * -1);
#endif

  
}