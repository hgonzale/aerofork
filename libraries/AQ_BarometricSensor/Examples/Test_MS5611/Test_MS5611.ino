

#include <AQMath.h>
#include <GlobalDefined.h>
#include <ArdupilotSPIExt.h>

int BARO_DETECTED = 0;
int vehicleState = 0;
bool calibrateReady = false;
bool startBaroMeasure = false;
bool baroReady = false;



#include <BarometricSensor_MS5611.h>


unsigned long timer = 0;
unsigned long timerB = 0;
unsigned long timerC = 0;

unsigned long start = 0;
float calibSum = 0.0;
int calibSumCount = 0;
int counter= 0;
float altitude = 0.0;

int baroFlag = 0;
int newAltReady = 0;

int baroDataReady = 0;
bool evalBaro = false;

unsigned long tic = 0;
unsigned long toc = 0;


void setup() {

	Serial.begin(115200);
	Serial.println("Barometric sensor library test (MS5611)");

	initializeBaro();
	Serial.println("Calibrating...");
	measureGroundBaro();
	startBaroMeasure = true;
	Serial.println("...finished calibrating");

}


void loop () {

	if (millis() - timerC > 10) { // 100 hz
		timerC = millis();

		measureBaro();
	}

	if (millis() - timerB > 20) { // 50 hz
		timerB = millis();

		altitude = getBaroAltitude();

		if ( counter > 200 && counter < 251 ) {

			calibSum += altitude;
			calibSumCount++;

		}

		if ( counter == 251 ) {

			setZBias(calibSum/calibSumCount);
			Serial.println("--------------------- Bias Set ----------------------------");
		}

		// Serial.println(altitude);
		counter++;

	}
	

	if ( millis() - timer > 100 ) { // 10hz

		timer = millis();

		Serial.print("Altitude: "); 
		Serial.println( getBaroAltitude() );
		// Serial.print("\t Temp: ");
		// Serial.println(rawTemperature);
	}

}