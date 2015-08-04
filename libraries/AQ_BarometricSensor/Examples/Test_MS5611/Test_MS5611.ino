


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

  // cli(); //disable global interrupts for setup function
  // // Pre-Scale values for OCRXA register (16-bit):
  // // 624 --> 100Hz
  // // 1249 --> 50Hz
  // // 3124 --> 20Hz

  // /* Timer 1 interrupt setup */
  // TCCR1A = 0; // initialize TCRR1A register to 0
  // TCCR1B = 0; // same for TCCR1B register
  // TCNT1 = 0; // initialize counter value to 0 (for Timer 1)

  // OCR1A = 624; // 100Hz
  // TCCR1B |= (1 << WGM12); // Enable CTC interrupt
  // TCCR1B |= (1 << CS12); // enable 256 pre-scaler
  // TIMSK1 |= (1 << OCIE1A);     

  // /* Timer 5 interrupt setup */
  // TCCR5A = 0; // initialize TCRR5A register to 0
  // TCCR5B = 0; // same for TCCR5B register
  // TCNT5 = 0; // initialize counter value to 0 (for Timer 5)

  // OCR5A = 1249;
  // TCCR5B |= (1 << WGM52); // Enable CTC interrupt
  // TCCR5B |= (1 << CS52); // enable 256 pre-scaler
  // TIMSK5 |= (1 << OCIE5A);

  // sei(); // re-enable global interrupts


	Serial.begin(115200);
	Serial.println("Barometric sensor library test (MS5611)");

	Wire.begin();

	initializeBaro();
	Serial.println("Calibrating...");
	measureGroundBaro();
	startBaroMeasure = true;
	Serial.println("...finished calibrating");

}


// ISR( TIMER1_COMPA_vect ) { // 100 hz

// 	if (startBaroMeasure) {

// 		measureBaroSum();

// 	}

// }


void loop () {

	if (millis() - timerC > 10) {
		timerC = millis();

		measureBaro();
	}

	if (millis() - timerB > 20) {
		timerB = millis();

		// evaluateBaroAltitude();



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
	

	if ( millis() - timer > 100 ) {

		timer = millis();

		Serial.print("Altitude: "); 
		Serial.print( getBaroAltitude() );
		Serial.print("\t Temp: ");
		Serial.println(rawTemperature);
	}

}