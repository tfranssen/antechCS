#include <AccelStepper.h>
#include <Controllino.h>
#include "EasyNextionLibrary.h"

EasyNex myNex(Serial2);

int currentPage = 0;

bool debug = true;

void setup() {
  myNex.begin(9600);

  delay(50);
  myNex.writeStr("page 0");
  Serial.begin(115200);
  delay(200);
  Serial.println("Setup complete");

}

void loop() {
  currentPage = myNex.currentPageId;
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
  }

}

//Cut table +
void trigger14() {
  if (debug) {
    Serial.println("Button pressed: Cut table +");
  }

}

//Feed -
void trigger15() {
  if (debug) {
    Serial.println("Button pressed: Feed -");
  }

}

//Feed +
void trigger16() {
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
