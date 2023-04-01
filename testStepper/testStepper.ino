#include <Controllino.h>

#define safetyRelay             CONTROLLINO_R0    //Relay to release safety
#define safetyErrorRelay        CONTROLLINO_R4    //Standby mode for safety
#define safetyStandbyRelay      CONTROLLINO_R3    //Standby mode for safety


void setup() {
  pinMode(CONTROLLINO_D4, OUTPUT);

  resetSafety();
  setSafetyStandby();

  digitalWrite(9, LOW);
}

void loop() {
  digitalWrite(CONTROLLINO_D4, HIGH);
  delay(10);
  digitalWrite(CONTROLLINO_D4, LOW);
  delay(10);
}

void resetSafety() {
  digitalWrite(safetyRelay, HIGH);
  delay(100);
  digitalWrite(safetyRelay, LOW);
}

void setSafetyStandby() {
  digitalWrite(safetyErrorRelay, LOW);  
  digitalWrite(safetyStandbyRelay, HIGH);
}

void setSafetyStandbyError() {
  digitalWrite(safetyErrorRelay, HIGH);    
  digitalWrite(safetyStandbyRelay, LOW);
}
