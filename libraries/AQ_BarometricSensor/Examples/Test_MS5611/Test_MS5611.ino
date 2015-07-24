


#include <Wire.h>
#include <AQMath.h>
#include <GlobalDefined.h>
#include <ArdupilotSPIExt.h>

int BARO_DETECTED = 0;
int vehicleState = 0;
bool calibrateReady = false;
#include <BarometricSensor_MS5611.h>


unsigned long timer = 0;


void setup() {

	Serial.begin(115200);
	Serial.println("Barometric sensor library test (MS5611)");

	Wire.begin();

	initializeBaro();

	Serial.println("Calibrating...");
	measureGroundBaro();
	Serial.println("...finished calibrating");


}


void loop() {

	if ((millis() - timer > 50)) { // 20Hz

		timer = millis();
		measureBaro();

		Serial.print("altitude: ");
		Serial.print(getBaroAltitude());
		Serial.println();

	}

}