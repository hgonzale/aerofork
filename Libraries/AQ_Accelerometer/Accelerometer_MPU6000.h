/*
  AeroQuad v3.0 - May 2011
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _AEROQUAD_ACCELEROMETER_MPU6000_COMMON_H_
#define _AEROQUAD_ACCELEROMETER_MPU6000_COMMON_H_

#include <APM_MPU6000.h>
#include <Accelerometer.h>

void initializeAccel() {

  initializeMPU6000Sensors();

}


void measureAccel() {

  readMPU6000Accel();

  meterPerSecSec[XAXIS] = MPU6000.data.accel.x * accelScaleFactor[XAXIS] + runTimeAccelBias[XAXIS];
  meterPerSecSec[YAXIS] = MPU6000.data.accel.y * accelScaleFactor[YAXIS] + runTimeAccelBias[YAXIS];
  meterPerSecSec[ZAXIS] = MPU6000.data.accel.z * accelScaleFactor[ZAXIS] + runTimeAccelBias[ZAXIS];

}

void measureAccelSum() {

  readMPU6000Accel();
  accelSample[XAXIS] += MPU6000.data.accel.x;
  accelSample[YAXIS] += MPU6000.data.accel.y;
  accelSample[ZAXIS] += MPU6000.data.accel.z;

  accelSampleCount++;

}

void evaluateMetersPerSec() {

  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {

    meterPerSecSec[axis] = (accelSample[axis] / accelSampleCount) * accelScaleFactor[axis] + runTimeAccelBias[axis];
  	accelSample[axis] = 0;

  }

  accelSampleCount = 0;

}

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
		
		accelScaleFactor[XAXIS] = 19.62/sum;
		
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
		
		accelScaleFactor[YAXIS] = 19.62/sum;
		
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
		
		accelScaleFactor[ZAXIS] = 19.62/sum;
		
		sum = 0.0;

        while (Serial.available() && Serial.read()); // empty buffer
		
	}
	
	else {
	
	}

}	

void computeAccelBias() {

  for (int samples = 0; samples < SAMPLECOUNT; samples++) {

	readMPU6000Sensors();
    measureAccelSum();
    delayMicroseconds(2500);

  }

  for (byte axis = 0; axis < 3; axis++) {

    meterPerSecSec[axis] = (float(accelSample[axis])/SAMPLECOUNT) * accelScaleFactor[axis];
    accelSample[axis] = 0;

  }

  accelSampleCount = 0;

  runTimeAccelBias[XAXIS] = -meterPerSecSec[XAXIS];
  runTimeAccelBias[YAXIS] = -meterPerSecSec[YAXIS];
  runTimeAccelBias[ZAXIS] = -9.8065 - meterPerSecSec[ZAXIS];

  accelOneG = abs(meterPerSecSec[ZAXIS] + runTimeAccelBias[ZAXIS]);

}

void calibration() {

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
	 storeSensorsZeroToEEPROM();
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

#endif
