// Arduino sketch that returns calibration offsets for MPU6050 //   Version 1.1  (31th January 2014)
// Done by Luis RÃ³denas <luisrodenaslorda@gmail.com>
// Based on the I2Cdev library and previous work by Jeff Rowberg <jeff@rowberg.net>
// Updates (of the library) should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

// These offsets were meant to calibrate MPU6050's internal DMP, but can be also useful for reading sensors. 
// The effect of temperature has not been taken into account so I can't promise that it will work if you 
// calibrate indoors and then use it outdoors. Best is to calibrate and use at the same room temperature.



// I2Cdev and MPU6050 must be installed as libraries
#include <Wire.h>             // Arduino IDE bug, needed because that the ITG3200 use Wire!
#include <APM_MPU6000.h>       // Arduino IDE bug, needed because that the ITG3200 use Wire!

#include <GlobalDefined.h>
#include <APM_ADC.h>
#include <AQMath.h>
#include <Accelerometer_MPU6000.h>

///////////////////////////////////   CONFIGURATION   /////////////////////////////
float mean_ax,mean_ay,mean_az=0.0;

///////////////////////////////////   SETUP   ////////////////////////////////////
void setup() {

  Serial.begin(115200);
  Serial.println("Accelerometer library test (APM)");
  
  initializeADC();
  
  initializeAccel();

  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available()){
    Serial.println(F("Send any character to start sketch.\n"));
    delay(1500);
  }                
  while (Serial.available() && Serial.read()); // empty buffer again

  // start message
  Serial.println("\nMPU6050 Calibration Sketch");
  delay(2000);
  Serial.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
  delay(3000);
  
}

///////////////////////////////////   LOOP   ////////////////////////////////////
void loop() {

  if (state==0){
  
    Serial.println("\nReading sensors for first time...");
    meansensors();
    state++;
    delay(1000);
	
  }

  if (state==1) { //Find Accelerometer scale factor for each axis.
  
    for (int i=0; i<3; i++){
      
      getScaleFactor(i);
      
      delay(1000);
      
    }
    
    state++;
    delay(1000);
	
  }
 
   if (state==2) {
     
     while (Serial.available() && Serial.read()); // empty buffer
     
     while (!Serial.available()){
       
       Serial.println(F("Send any character to continue.\n"));
       delay(1500);
       
     }   
     
     while (Serial.available() && Serial.read()); // empty buffer again  
     
     Serial.println("\nCalculating offsets...");
     computeAccelBias();
     state++;
     delay(1000);
	
  }

  if (state==3) {
  
    measureAccel();
	
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor readings with offsets:\t");
	
	for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
	
		Serial.print(meterPerSecSec[axis]); 
		Serial.print("\t");
		
	}
	
    while (Serial.available() && Serial.read()); // empty buffer
    
    while(!Serial.available()) {
	
		measureAccel();
		Serial.print("Roll: ");
		Serial.print(meterPerSecSec[XAXIS]);
		Serial.print(" Pitch: ");
		Serial.print(meterPerSecSec[YAXIS]);
		Serial.print(" Yaw: ");
		Serial.print(meterPerSecSec[ZAXIS]);
		Serial.println();
		delayMicroseconds(10000);
	  
    }
	
  }
  
}

///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void meansensors(){

  // read raw accel/gyro measurements from device
	for (int i=0; i<400; i++) {
	
		readMPU6000Sensors();
		measureAccelSum();
		delayMicroseconds(2500);
	
	}
	
    mean_ax = ((float)accelSample[XAXIS]) / 400.00;
    mean_ay = ((float)accelSample[YAXIS]) / 400.00;
    mean_az = ((float)accelSample[ZAXIS]) / 400.00;
	
	accelSample[XAXIS] = 0;
	accelSample[YAXIS] = 0;
	accelSample[ZAXIS] = 0;
	
	accelSampleCount = 0;
	  
}

void getScaleFactor(int axis) { //converts accelerometer readings to scale around g = 9.81 m/s2.

	float sum = 0.0;

	if (axis == XAXIS) {
	
		Serial.println("+x downwards");
                
        while (Serial.available() && Serial.read()); // empty buffer
		
		while(!Serial.available()) {
		
		}
		
		meansensors();
		
		sum += mean_ax;
                
        while (Serial.available() && Serial.read()); // empty buffer
		
		Serial.println("-x downwards");
		
		while(!Serial.available()) {
		
		}
		
		meansensors();	
		
		sum -= mean_ax;
		
		accelScaleFactor[XAXIS] = 2.0/sum;
		
		sum = 0.0;

        while (Serial.available() && Serial.read()); // empty buffer
		
	}

	else if (axis == YAXIS) {
  
        while (Serial.available() && Serial.read()); // empty buffer

		Serial.println("+y downwards");
		
		while(!Serial.available()) {
		
		}
		
		meansensors();
		
		sum += mean_ay;

        while (Serial.available() && Serial.read()); // empty buffer
		
		Serial.println("-y downwards");
		
		while(!Serial.available()) {
		
		}
		
		meansensors();	
		
		sum -= mean_ay;
		
		accelScaleFactor[YAXIS] = 2.0/sum;
		
		sum = 0.0;

        while (Serial.available() && Serial.read()); // empty buffer
		
	}	
	
	else if (axis == ZAXIS) {
  
        while (Serial.available() && Serial.read()); // empty buffer
	
		Serial.println("+z downwards");
		
		while(!Serial.available()) {
		
		}
		
		meansensors();
		
		sum += mean_az;

        while (Serial.available() && Serial.read()); // empty buffer
		
		Serial.println("-z downwards");
		
		while(!Serial.available()) {
		
		}
		
		meansensors();	
		
		sum -= mean_az;
		
		accelScaleFactor[ZAXIS] = 2.0/sum;
		
		sum = 0.0;

        while (Serial.available() && Serial.read()); // empty buffer
		
	}
	
	else {
	
	}

}	

