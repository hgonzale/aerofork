/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

// parts of this code were taken from AN520, an early version of fabio's library and the AQ BMP085 code

#ifndef _AQ_BAROMETRIC_SENSOR_MS5611_
#define _AQ_BAROMETRIC_SENSOR_MS5611_

#include "Arduino.h"
#include "BarometricSensor.h"
//#include "Device_I2C.h"
#include <AQMath.h>

#include <ArdupilotSPIExt.h>
ArdupilotSPIExt spiMS5611;

#define MS5611_CS 40
#define CMD_MS5611_RESET 0x1E
#define CMD_MS5611_PROM_Setup 0xA0
#define CMD_MS5611_PROM_C1 0xA2
#define CMD_MS5611_PROM_C2 0xA4
#define CMD_MS5611_PROM_C3 0xA6
#define CMD_MS5611_PROM_C4 0xA8
#define CMD_MS5611_PROM_C5 0xAA
#define CMD_MS5611_PROM_C6 0xAC
#define CMD_MS5611_PROM_CRC 0xAE
#define CMD_CONVERT_D1_OSR4096 0x48   // Maximum resolution (oversampling)
#define CMD_CONVERT_D2_OSR4096 0x58   // Maximum resolution (oversampling)

uint16_t C1;
uint16_t C2;
uint16_t C3;
uint16_t C4;
uint16_t C5;
uint16_t C6;

long MS5611lastRawTemperature;
long MS5611lastRawPressure;

int64_t MS5611_sens=0;
int64_t MS5611_offset=0;
int64_t dT;

float pressure           = 0;
long rawPressure         = 0;
long rawTemperature      = 0;
byte pressureCount       = 0;
float pressureFactor     = 1/5.255;
boolean isReadPressure   = false;
float rawPressureSum     = 0;
byte rawPressureSumCount = 0;

unsigned long startRead = 0;
unsigned long endRead = 0;

void requestRawTemperature() {

  spiMS5611.spi_write(CMD_CONVERT_D2_OSR4096);

}


unsigned long readRawTemperature() {
  
  MS5611lastRawTemperature= spiMS5611.spi_read_adc();

  dT = MS5611lastRawTemperature - (((uint32_t)C5) << 8);

  MS5611_offset  = (((int64_t)C2) << 16) + ((C4 * dT) >> 7);

  MS5611_sens    = (((int64_t)C1) << 15) + ((C3 * dT) >> 8);

  return MS5611lastRawTemperature;

}

void requestRawPressure() {

  spiMS5611.spi_write(CMD_CONVERT_D1_OSR4096);

}


unsigned long readRawPressure() {
  

  MS5611lastRawPressure = spiMS5611.spi_read_adc();

  return ((((MS5611lastRawPressure * MS5611_sens) >> 21) - MS5611_offset) >> (15-5)) / ((float)(1<<5));

}

float readTemperature() {

  return ((1<<5)*2000 + (((MS5611lastRawTemperature - ((int64_t)C5 << 8)) * C6) >> (23-5))) / ((1<<5) * 100.0);

}//formulas need verifications


bool baroGroundUpdateDone = false;
unsigned long baroStartTime;

//initialize
void initializeBaro() {

  pressure = 0.0;
  baroGroundAltitude = 0.0;
  pressureFactor = 1/5.255;
  
  spiMS5611.begin(SPI_CLOCK_DIV16, MSBFIRST, SPI_MODE3);

  pinMode(MS5611_CS, OUTPUT);          // Chip select Pin
  digitalWrite(MS5611_CS, HIGH);
  delay(20);

  spiMS5611.spi_write(CMD_MS5611_RESET);
  delay(20);

  byte val = spiMS5611.spi_read(CMD_MS5611_PROM_Setup);

  if(val == 0x00) {

	  vehicleState |= BARO_DETECTED;

  }

  delay(20);

  //conversion factors
  C1 = spiMS5611.spi_read_16bits(CMD_MS5611_PROM_C1);
  C2 = spiMS5611.spi_read_16bits(CMD_MS5611_PROM_C2);
  C3 = spiMS5611.spi_read_16bits(CMD_MS5611_PROM_C3);
  C4 = spiMS5611.spi_read_16bits(CMD_MS5611_PROM_C4);
  C5 = spiMS5611.spi_read_16bits(CMD_MS5611_PROM_C5);
  C6 = spiMS5611.spi_read_16bits(CMD_MS5611_PROM_C6);
  //

  /*if(MS5611readPROM(MS5611_I2C_ADDRESS) ) {
          vehicleState |= BARO_DETECTED;
  }*/

  requestRawTemperature(); // setup up next measure() for temperature

  isReadPressure = false;
  pressureCount = 0;
  delay(10);

  baroStartTime = micros();

  measureBaroSum(); // first read temperature
  delay(10);

  measureBaroSum(); // read pressure
  delay(10);
	
  //measureGroundBaro();
  
  //baroAltitude = baroGroundAltitude;
  
}

//
void measureBaro() {

  measureBaroSum();
  evaluateBaroAltitude();

}
//

//
void measureBaroSum() {

  // switch between pressure and temperature measurements
  if (isReadPressure) {

    rawPressureSum += readRawPressure();
    rawPressureSumCount++;

    if (pressureCount == 20) {

      requestRawTemperature();
      pressureCount = 0;
      isReadPressure = false;

    } 
        
	else {

      requestRawPressure();

    }

    pressureCount++;

  } 

  else { // select must equal TEMPERATURE

    readRawTemperature();

    requestRawPressure();

    isReadPressure = true;

  }

}
//

//
bool MS5611_first_read = true;
//



//return altitude
void evaluateBaroAltitude() {
	  
  if (rawPressureSumCount > 0) { // it may occur at init time that no pressure has been read yet!

    pressure = ((float)rawPressureSum)/ rawPressureSumCount;

  }

  baroRawAltitude = 44330.0 * (1 - pow(pressure/101325.0, pressureFactor)); // returns absolute baroAltitude in meters
  // use calculation below in case you need a smaller binary file for CPUs having just 32KB flash ROM
  //baroRawAltitude = (101325.0-pressure)/4096*346;
  
  startRead = micros();

  if(MS5611_first_read) {

    baroAltitude = baroRawAltitude;
    MS5611_first_read = false;

  } 

  else {

	baroAltitude = filterSmoothWithTime(baroRawAltitude, baroAltitude, baroSmoothFactor, ((startRead - endRead)/100000.0)); //100 ms per altitude read
	
  }
	
  rawPressureSum = 0.0;
  rawPressureSumCount = 0;
	
  endRead = startRead;

}
//



#endif