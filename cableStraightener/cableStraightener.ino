#include <AccelStepper.h>
#include <Controllino.h>
#include "EasyNextionLibrary.h"

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
//Current: 2.8A RMS, Full Current
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

#define safetyRelay             CONTROLLINO_R0    //Relay to release safety
#define safetyStandbyRelay      CONTROLLINO_R3    //Standby mode for safety

#define lightRelay              CONTROLLINO_R4    //Standby mode for safety

//Feeder stepper PINs
#define stepperFeederStepPin    CONTROLLINO_D0
#define stepperFeerderDirPin    CONTROLLINO_D1
#define stepperFeederEnablePin  CONTROLLINO_D2 //Check if this is correct, if the steppers go the wrong way change to CONTROLLINO_D3
//CONTROLLINO_D3 is connected to DIR of the second stepper motor

//Cutter table stepper PINs
#define stepperCutterStepPin    CONTROLLINO_D4
#define stepperCutterDirPin     CONTROLLINO_D5
#define stepperCutterEnablePin  CONTROLLINO_D6

#define motorInterfaceType 1

AccelStepper stepperFeeder = AccelStepper(motorInterfaceType, stepperFeederStepPin, stepperFeerderDirPin);
AccelStepper stepperCutter = AccelStepper(motorInterfaceType, stepperCutterStepPin, stepperCutterDirPin);

int feederMaxSpeedSetting = 5000;
int feederExtrudeAccel = 10000;

int cutterMaxSpeedSetting = 50;
int cutterExtrudeAccel = 10000;

int currentPage = 0;

bool debug = true;

void setup() {
  //PINs settings
  pinMode(CONTROLLINO_D0, OUTPUT);
  pinMode(CONTROLLINO_D1, OUTPUT);
  pinMode(CONTROLLINO_D2, OUTPUT);
  pinMode(CONTROLLINO_D6, OUTPUT);
  pinMode(CONTROLLINO_D7, OUTPUT);
  pinMode(CONTROLLINO_D8, OUTPUT);

  pinMode(CONTROLLINO_D8, OUTPUT);


  //stepperFeeder.setEnablePin(stepperFeederEnablePin);
  stepperFeeder.setMaxSpeed(feederMaxSpeedSetting);
  stepperFeeder.setAcceleration(feederExtrudeAccel);
  stepperFeeder.enableOutputs();

  stepperCutter.setEnablePin(stepperCutterEnablePin);
  //digitalWrite(stepperCutterEnablePin, LOW);
  stepperCutter.setMaxSpeed(cutterMaxSpeedSetting);
  stepperCutter.setSpeed(5000);
  stepperCutter.setAcceleration(cutterExtrudeAccel);
  stepperCutter.enableOutputs();

  resetSafety();
  setSafetyStandby();

  //Light on
  digitalWrite(lightRelay, HIGH);


  myNex.begin(9600);
  delay(50);
  myNex.writeStr("page 0");
  Serial.begin(115200);
  delay(200);
  Serial.println("Setup complete");

}

void loop() {
  currentPage = myNex.currentPageId;
  stepperFeeder.run();
  stepperCutter.run();
  myNex.NextionListen();

}

//Length -
void trigger0() {
  if (debug) {
    Serial.println("Button pressed: Length -");
  }


}

//Length +
void trigger1() {
  if (debug) {
    Serial.println("Button pressed: Length +");
  }

}

//Quantity -
void trigger2() {
  if (debug) {
    Serial.println("Button pressed: Quantity -");
  }

}

//Quantity +
void trigger3() {
  if (debug) {
    Serial.println("Button pressed: Quantity +");
  }

}

//Start button on setting screen
void trigger4() {
  if (debug) {
    Serial.println("Button pressed: Start button");
  }

}

//Return button in initialising screen
void trigger5() {
  if (debug) {
    Serial.println("Button pressed: Return button in init screen");
  }

}

//Feed push button
void trigger6() {
  if (debug) {
    Serial.println("Button pressed: Feed push");
  }

}

//Feed release button
void trigger22() {
  if (debug) {
    Serial.println("Button pressed: Feed release");
  }

}

//Ref cut
void trigger7() {
  if (debug) {
    Serial.println("Button pressed: Ref cut");
  }
}

//Ref cut page next
void trigger8() {
  if (debug) {
    Serial.println("Button pressed: Ref cut next");
  }

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

}

//Rotate +
void trigger12() {
  if (debug) {
    Serial.println("Button pressed: Rotate +");
  }

}

//Cut table -
void trigger13() {
  if (debug) {
    Serial.println("Button pressed: Cut table -");
    stepperCutter.move(-50000);

  }

}

//Cut table +
void trigger14() {
  if (debug) {
    Serial.println("Button pressed: Cut table +");
    stepperCutter.move(50000);

  }

}

//Feed -
void trigger15() {
  stepperFeeder.move(-500);

  if (debug) {
    Serial.println("Button pressed: Feed -");
  }

}

//Feed +
void trigger16() {
  stepperFeeder.move(500);
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

}


//Vacuum
void trigger18() {
  resetSafety();
  setSafetyStandby();
  if (debug) {
    Serial.println("Button pressed: Vaccuum on / off");

  }

}

//Home cutter table
void trigger19() {
  if (debug) {
    Serial.println("Button pressed: Home cutter table");
  }

}

//Reset replace disc
void trigger20() {
  if (debug) {
    Serial.println("Button pressed: Reset at replace disc screen");
  }

}

//Reset after error
void trigger21() {
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
