


// #include <EEPROM.h>
#include <Wire.h>
#include <GlobalDefined.h>
#include <AQMath.h>
#include <APM_ADC.h>

boolean printFlag;
#include <ArdupilotSPIExt.h>
#include <APM_MPU6000.h>

unsigned long timer;
float indic;

// Gyroscope declaration
#include <Gyroscope_MPU6000.h>

// Accelerometer Declaration
// #include <Accelerometer_MPU6000.h>




void setup() {
	Serial.begin(115200);
	Serial.print("MPU6000 gyro test");

	printFlag = false;

	initializeADC();
	initializeGyro();
}

int count;

// ISR(SPI_STC_vect) {
// 	// measureGyro();
// 	SPDR = 
// 	count++;
// }

// void MPU_measure_int() {
// 	measureGyro();
// 	count++;
// }

void loop() {

	if (millis() - timer > 5) { // 200 Hz
		timer = millis();
		measureGyro();

		// Serial.println(count);

		// if (printFlag) {
		// 	printFlag = false;
		// 	Serial.print("MPU6000 data: \t");
		// 	Serial.print(snapshot.data.accel.x);
		// 	Serial.print(", ");
		// 	Serial.print(snapshot.data.accel.y);
		// 	Serial.print(", ");
		// 	Serial.print(snapshot.data.accel.z);
		// 	Serial.print("\t ");
		// 	Serial.print(snapshot.data.temperature);
		// 	Serial.print("\t ");
		// 	Serial.print(snapshot.data.gyro.x);
		// 	Serial.print(", ");
		// 	Serial.print(snapshot.data.gyro.y);
		// 	Serial.print(", ");
		// 	Serial.print(snapshot.data.gyro.z);
		// 	Serial.println();

		// }


		Serial.print("MPU6000 data: \t");
		Serial.print(MPU6000.data.accel.x);
		Serial.print(", ");
		Serial.print(MPU6000.data.accel.y);
		Serial.print(", ");
		Serial.print(MPU6000.data.accel.z);
		Serial.print("\t ");
		Serial.print(MPU6000.data.temperature);
		Serial.print("\t ");
		Serial.print(MPU6000.data.gyro.x);
		Serial.print(", ");
		Serial.print(MPU6000.data.gyro.y);
		Serial.print(", ");
		Serial.print(MPU6000.data.gyro.z);


		if (printFlag) { // error indicator -- easy to see in Serial Monitor
			Serial.print("\t +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
			printFlag = false;
		}


		Serial.println();
	}

}