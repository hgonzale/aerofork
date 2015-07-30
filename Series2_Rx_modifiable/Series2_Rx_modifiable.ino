
char foo;
#include <XBeeFSM.h>


unsigned long timer;

void setup() {

  timer = millis();

  initXBeeFSM();

  Serial.begin(9600);

}


void loop() {

	if (millis() - timer > 100) { // 10 Hz

		timer = millis();

		XBeeRead();

		if (checkEmergencyStop()) {

			Serial.print("Emergency Stop triggered!")
		}

	}

}
