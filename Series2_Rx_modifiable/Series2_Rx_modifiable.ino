
char foo;
#include <XBeeFSM.h>


unsigned long timer;

void setup() {

  timer = millis();

  initXBeeFSM();

  Serial.begin(115200);

}


void loop() {

	if (millis() - timer > 100) { // 10 Hz

		timer = millis();

		XBeeRead();

	}

}
