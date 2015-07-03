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

// parts of the init sequence were taken from AP_InertialSensor_MPU6000.h

#ifndef _AEROQUAD_APM_MPU6000_H_
#define _AEROQUAD_APM_MPU6000_H_

#include "Arduino.h"
#include <SensorsStatus.h>
#include <ArdupilotSPIExt.h>
ArdupilotSPIExt spiMPU6000;

#define DEBUG_INIT

//MPU6000 registers
#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_FIFO_EN			0x23
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74
//MPU6000 registers

// Configuration bits
#define BIT_SLEEP				0x40
#define BIT_H_RESET				0x80
#define BITS_CLKSEL				0x07
#define MPU_CLK_SEL_PLLGYROX	0x01
#define MPU_CLK_SEL_PLLGYROZ	0x03
#define MPU_EXT_SYNC_GYROX		0x02
#define BITS_FS_250DPS          0x00
#define BITS_FS_500DPS          0x08
#define BITS_FS_1000DPS         0x10
#define BITS_FS_2000DPS         0x18
#define BITS_FS_MASK            0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR    0x10
#define BIT_RAW_RDY_EN			0x01
#define BIT_I2C_IF_DIS          0x10
#define BIT_INT_STATUS_DATA		0x01
// Configuration bits

#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

#define SPI_MODE_MASK 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR
//

typedef struct {

  short x;
  short y;
  short z;

} tAxis;


union uMPU6000 {

  unsigned char rawByte[];
  unsigned short rawWord[];

  struct {

	tAxis accel;
	short temperature;
	tAxis gyro;

  } data;

} MPU6000, snapshot;


void MPU6000_SpiLowSpeed() { // 500kHz

	spiMPU6000.begin(SPI_CLOCK_DIV32, MSBFIRST, SPI_MODE3);

}



void MPU6000_SpiHighSpeed() { // 500kHz

	spiMPU6000.end();
  spiMPU6000.begin(SPI_CLOCK_DIV32, MSBFIRST, SPI_MODE3); // 500 kHz
}


void MPU6000_WriteReg(int addr, byte data) {

	spiMPU6000.Write(addr, data);

	delay(1);

}


byte MPU6000_ReadReg(int addr) {

	byte data = spiMPU6000.Read(addr);
	delay(1);

	return data;

}


void initializeMPU6000Sensors() {

  #ifdef DEBUG_INIT

    Serial.println("initializeMPU6000Sensors");

  #endif

    MPU6000_SpiLowSpeed();
    delay(10);

    unsigned char val;

    val = MPU6000_ReadReg(MPUREG_WHOAMI);

    if((val&0x7E) == 0x68) {

        vehicleState |= GYRO_DETECTED;
        vehicleState |= ACCEL_DETECTED;

		#ifdef DEBUG_INIT

		Serial.println("MPU6000 found");

		#endif

    } 
	
	else {

		#ifdef DEBUG_INIT

        Serial.println("MPU6000 not found");

		#endif

	}

  // Chip reset
  MPU6000_WriteReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
  delay(100);  
  // Startup time delay

  // Wake Up device and select GyroZ clock (better performance)
  MPU6000_WriteReg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
  delay(1);//
  MPU6000_WriteReg(MPUREG_PWR_MGMT_2, 0);
  delay(1);//

  // Disable I2C bus
  MPU6000_WriteReg(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
  delay(1);//

  // SAMPLE RATE    Fsample = 1Khz/(<value> + 1)
  MPU6000_WriteReg(MPUREG_SMPLRT_DIV, 0x4);     // Sample rate = 200 Hz.
  delay(1);//

  // FS & DLPF   FS=1000�/s, DLPF = 42Hz (low pass filter)
  MPU6000_WriteReg(MPUREG_CONFIG, BITS_DLPF_CFG_42HZ);
  delay(1);//

  MPU6000_WriteReg(MPUREG_GYRO_CONFIG,BITS_FS_1000DPS);  // Gyro scale 1000�/s
  delay(1);//
  MPU6000_WriteReg(MPUREG_ACCEL_CONFIG,0x08);   // Accel scale +/-4g (4096LSB/g)

  MPU6000_WriteReg(MPUREG_INT_ENABLE,BIT_RAW_RDY_EN); //INT: raw data ready
  MPU6000_WriteReg(MPUREG_INT_PIN_CFG,BIT_INT_ANYRD_2CLEAR); //INT: clear on any read

}


void MPU6000SwapData(unsigned char *data, int datalen) {

  datalen /= 2;

  while(datalen-- > 0) {

    unsigned char t = data[0];
    data[0] = data[1];
    data[1] = t;
    data += 2;
  }

}

void readMPU6000Sensors() {

    spiMPU6000.Read(MPUREG_ACCEL_XOUT_H, MPU6000.rawByte, sizeof(MPU6000));
    MPU6000SwapData(MPU6000.rawByte, sizeof(MPU6000));

}


int readMPU6000Count=0;
int readMPU6000AccelCount=0;
int readMPU6000GyroCount=0;

void readMPU6000Accel() {

  readMPU6000AccelCount++;

  if(readMPU6000AccelCount != readMPU6000Count) {

    readMPU6000Sensors();
    readMPU6000Count++;

  }

}

void readMPU6000Gyro() {

  readMPU6000GyroCount++;

  if(readMPU6000GyroCount != readMPU6000Count) {

    readMPU6000Sensors();
    readMPU6000GyroCount++;

  }

}

#endif
