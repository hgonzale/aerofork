


#include <Wire.h>
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
int counter = 0;
bool doit = false;
float altitude = 0.0;









void setup() {

	cli();

	TCCR5A = 0; // initialize TCRR5A register to 0
	TCCR5B = 0; // same for TCCR5B register
	TCNT5 = 0; // initialize counter value to 0 (for Timer 5)

	OCR5A = 624;
	TCCR5B |= (1 << WGM52); // Enable CTC interrupt
	TCCR5B |= (1 << CS52); // enable 256 pre-scaler
	TIMSK5 |= (1 << OCIE5A);

	sei();

	Serial.begin(9600);
	Serial.println("Barometric sensor library test (MS5611)");

	Wire.begin();

	initializeBaro();

	Serial.println("Calibrating...");
	measureGroundBaro();
	startBaroMeasure = true;
	Serial.println("...finished calibrating");

	requestRawPressure();

}

volatile float rawSum = 0.0;
volatile int rawSumCount = 0;


ISR(TIMER5_COMPA_vect) {

	if (startBaroMeasure) {

		measureBaroSum();
		
	}

}



void loop() {

	// if (millis() - timer > 10) { // 100 hz --> measure

	// 	timer = millis();

	// 	measureBaroSum();

	// }


	if (millis() - timerB > 20) { // 50 hz --> evaluate

		timerB = millis();
		
		rawPressureSum = rawSum;
		rawPressureSumCount = rawSumCount;

		evaluateBaroAltitude();

	}

	if (millis() - timerC > 100) { // 10 hz --> print

		timerC = millis();

		altitude = getBaroAltitude();

		Serial.println(altitude);
	}

}