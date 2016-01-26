
/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later versionarduco

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/****************************************************************************
   Before flight, select the different user options for your AeroQuad by
   editing UserConfiguration.h.

   If you need additional assistance go to ht tp://www.aeroquad.com/forum.php
   or talk to us live on IRC #aeroquad
*****************************************************************************/

#include "UserConfiguration.h" // Edit this file first before uploading to the AeroQuad

//
// Define Security Checks
//

#if defined(UseGPSNMEA) || defined(UseGPSUBLOX) || defined(UseGPSMTK) || defined(UseGPS406)
 #define UseGPS
#endif 

#if defined(UseGPSNavigator) && !defined(AltitudeHoldBaro)
  #error "GpsNavigation NEED AltitudeHoldBaro defined"
#endif

#include <EEPROM.h>
#include <Wire.h>
#include <Device_I2C.h>
#include <GlobalDefined.h>
#include "AeroQuad.h"
#include "PID.h"
#include <AQMath.h>
#include <FourtOrderFilter.h>

//********************************************************
//********************************************************
//********* PLATFORM SPECIFIC SECTION ********************
//********************************************************
//********************************************************


#define LED_Green 37
#define LED_Red 35
#define LED_Yellow 36
#define DEBUG_INIT
#define HeadingMagHold //uncomment to remove magnetometer
#include <APM_ADC.h>
#include <APM_RC.h>
#include <ArdupilotSPIExt.h>
#include <APM_MPU6000.h>

// Gyroscope declaration
#include <Gyroscope_MPU6000.h>

// Accelerometer Declaration
#include <Accelerometer_MPU6000.h>

// Receiver Declaration
#define RECEIVER_APM

// Motor Declaration
#define MOTOR_PWM_Timer

// heading mag hold declaration
#define HMC5883L
#include <Magnetometer_HMC5883L.h>
#define SPARKFUN_9DOF_5883L
//

// Altitude declaration
#include <BarometricSensor_MS5611.h>
//


#undef BattMonitorAutoDescent
#undef POWERED_BY_VIN

#ifndef UseGPS
  #undef UseGPSNavigator
#endif


/**
 * Put ArduCopter specific initialization need here
 */
 void initPlatform() {

  pinMode(LED_Red, OUTPUT);
  pinMode(LED_Yellow, OUTPUT);
  pinMode(LED_Green, OUTPUT);

  initializeADC();

  initRC();

  Wire.begin();

  TWBR = 12;

  initializeMPU6000Sensors();

  initializeBaro();

  initializeMagnetometer();

}

// called when eeprom is initialized
void initializePlatformSpecificAccelCalibration() {

  accelScaleFactor[XAXIS] = 1.0;
  runTimeAccelBias[XAXIS] = 0.0;
  accelScaleFactor[YAXIS] = 1.0;
  runTimeAccelBias[YAXIS] = 0.0;
  accelScaleFactor[ZAXIS] = 1.0;
  runTimeAccelBias[ZAXIS] = 0.0;

  #ifdef HeadingMagHold
  magBias[XAXIS]  = 0.0;
  magBias[YAXIS]  = 0.0;
  magBias[ZAXIS]  = 0.0;
  #endif

}


/**
 * Measure critical sensors
 */
 void measureCriticalSensors() {

  evaluateADC();

  measureGyroSum();

  measureAccelSum();

}



//********************************************************
//********************************************************
//********* HARDWARE GENERALIZATION SECTION **************
//********************************************************
//********************************************************

// default to 10bit ADC (AVR)
#ifndef ADC_NUMBER_OF_BITS
#define ADC_NUMBER_OF_BITS 10
#endif

//********************************************************
//****************** KINEMATICS DECLARATION **************
//********************************************************
#include "Kinematics.h"
#include "Kinematics_ARG.h"


//********************************************************
//******************** RECEIVER DECLARATION **************
//********************************************************
#if defined(RECEIVER_APM)
  #include <Receiver_APM.h>
#endif


//********************************************************
//********************** MOTORS DECLARATION **************
//********************************************************
  #include <Motors_PWM_Timer.h>

//********************************************************
//******* HEADING HOLD MAGNETOMETER DECLARATION **********
//********************************************************
// #if defined(HMC5843)
//   #include <HeadingFusionProcessorMARG.h>
//   #include <Magnetometer_HMC5843.h>

// #elif defined(SPARKFUN_9DOF_5883L) || defined(SPARKFUN_5883L_BOB) || defined(HMC5883L)
  #include <HeadingFusionProcessorMARG.h>
  #include <Magnetometer_HMC5883L.h>

// #endif

//********************************************************
//******* ALTITUDE HOLD BAROMETER DECLARATION ************
//********************************************************
#if defined(BMP085)
  #include <BarometricSensor_BMP085.h>
#elif defined(MS5611)
 #include <BarometricSensor_MS5611.h>
#endif


//********************************************************
//******** FLIGHT CONFIGURATION DECLARATION **************
//********************************************************
#include "FlightControlQuadX.h"

//********************************************************
//****************** GPS DECLARATION *********************
//********************************************************
#if defined(UseGPS)
  #if !defined(HeadingMagHold)
    #error We need the magnetometer to use the GPS
  #endif 
  #include <GpsAdapter.h>
  #include "GpsNavigator.h"
#endif


//********************************************************
//****************** SERIAL PORT DECLARATION *************
//********************************************************
#define SERIAL_PORT Serial

// Include this last as it contains objects from above declarations
#include "AltitudeControlProcessor.h"
#include "FlightControlProcessor.h"
#include "HeadingHoldProcessor.h"
#include "DataStorage.h"

#include <XBeeFSMnewProtocol.h>



/*******************************************************************
 * Main setup function, called one time at bootup
 * initialize all system and sub system of the
 * Aeroquad
 ******************************************************************/
 void setup() {


  cli(); //disable global interrupts for setup function
  // Pre-Scale values for OCRXA register (16-bit):
  // 624 --> 100Hz
  // 1249 --> 50Hz
  // 3124 --> 20Hz

  /* Timer 1 interrupt setup */
  TCCR1A = 0; // initialize TCRR1A register to 0
  TCCR1B = 0; // same for TCCR1B register
  TCNT1 = 0; // initialize counter value to 0 (for Timer 1)

  OCR1A = 1249; // 50Hz
  TCCR1B |= (1 << WGM12); // Enable CTC interrupt
  TCCR1B |= (1 << CS12); // enable 256 pre-scaler
  TIMSK1 |= (1 << OCIE1A);     

  /* Timer 5 interrupt setup */
  TCCR5A = 0; // initialize TCRR5A register to 0
  TCCR5B = 0; // same for TCCR5B register
  TCNT5 = 0; // initialize counter value to 0 (for Timer 5)

  OCR5A = 624;
  TCCR5B |= (1 << WGM52); // Enable CTC interrupt
  TCCR5B |= (1 << CS52); // enable 256 pre-scaler
  TIMSK5 |= (1 << OCIE5A);

  sei(); // re-enable global interrupts

  SERIAL_BEGIN(BAUD);
  #if defined WirelessTelemetry
    xbee.begin(SERIAL_PORT);
  #endif
  Serial.flush();

  pinMode(LED_Green, OUTPUT);
  digitalWrite(LED_Green, LOW);

  readEEPROM(); // defined in DataStorage.h
  boolean firstTimeBoot = false;

  // if (readFloat(SOFTWARE_VERSION_ADR) != SOFTWARE_VERSION) { // If we detect the wrong soft version, we init all parameters
  //   initializeEEPROM();
  //   writeEEPROM();
  //   firstTimeBoot = true;

  // }
  
  initPlatform();
  
  initializeMotors(FOUR_Motors);
  
  initializeGyro();

  while (!calibrateGyro()); // this make sure the craft is still befor to continue init process
  
  initializeAccel();
  
  if (firstTimeBoot) {

    computeAccelBias();
    writeEEPROM();

  }
  
  setupFourthOrder();
  initSensorsZeroFromEEPROM();
  
  initPID();

  initializeKinematics();

  initXBeeFSM();

  #ifdef HeadingMagHold
    vehicleState |= HEADINGHOLD_ENABLED;
    initializeMagnetometer();
    initializeHeadingFusion();
  #endif 

  previousTime = micros();
  digitalWrite(LED_Green, HIGH);
  safetyCheck = 0;

  commandAllMotors(1000);

}


/*Interrupt for motor actuation
  Runs at 50Hz
*/
  ISR(TIMER1_COMPA_vect) {

    //write directly to motor registers
    OCR3B = motorCommand[MOTOR3] * 2;
    OCR3C = motorCommand[MOTOR2] * 2;
    OCR3A = motorCommand[MOTOR1] * 2;
    OCR4A = motorCommand[MOTOR4] * 2;

  }


/*
Interrupt for pressure sensor reading
  Runs at 100Hz
  If SPI is busy during ISR, we measure
  the baro in the main loop
*/
  ISR(TIMER5_COMPA_vect) {

  if (startBaroMeasure) {

    if (digitalRead(MPUCS) != 0) {

      measureBaro();

      baroDataReady = 1;

    } else {

      baroReadFlag = 1;

    }

  }

}



/*******************************************************************
 * 100Hz task
 ******************************************************************/
 void process100HzTask() {

  G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
  hundredHZpreviousTime = currentTime;
  
  evaluateGyroRate();

  evaluateMetersPerSec();

  for (int axis = XAXIS; axis <= ZAXIS; axis++) {

    filteredAccel[axis] = computeFourthOrder(meterPerSecSec[axis], &fourthOrder[axis]);

  }
  
  // kinematicsAngle[] is updated here
  calculateKinematics(gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], G_Dt);
  
}

/*******************************************************************
 * 50Hz task
 ******************************************************************/
 void process50HzTask() {

  G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
  fiftyHZpreviousTime = currentTime;


  if (beginControl) {

    processStabilityControl();

  } else {

    //stop motors
    motorCommand[MOTOR3] = 1000;
    motorCommand[MOTOR2] = 1000;
    motorCommand[MOTOR1] = 1000;
    motorCommand[MOTOR4] = 1000;

  }

  if (startBaroMeasure && baroDataReady) { 

    baroDataReady = 0;

    if ( counter > 200 && counter < 251 ) {

      baroCalibSum += getBaroAltitude();
      baroCalibSumCount++;

    }

    if ( counter == 251 ) {

      setZBias(baroCalibSum/baroCalibSumCount);
      Serial.println("baro calibration complete");

    }

    counter++;
  }

 }

/*******************************************************************
 * 10Hz task
 ******************************************************************/
 void process10HzTask1() {

  XBeeRead();

  if ( beginControl && !checkEmergencyStop() ) emergencyStop(); 

}

/*******************************************************************
 * low priority 10Hz task 2
 ******************************************************************/
 void process10HzTask2() {

  G_Dt = (currentTime - lowPriorityTenHZpreviousTime) / 1000000.0;
  lowPriorityTenHZpreviousTime = currentTime;

}

/*******************************************************************
 * low priority 10Hz task 3
 ******************************************************************/
 void process10HzTask3() {

  G_Dt = (currentTime - lowPriorityTenHZpreviousTime2) / 1000000.0;
  lowPriorityTenHZpreviousTime2 = currentTime;

}


/*******************************************************************
 * 2Hz task 
 ******************************************************************/
void process2HzTask() {

  // XBeeRead();

  // readSerialCommand();
  // sendSerialTelemetry();

}


/*******************************************************************
 * 1Hz task 
 ******************************************************************/
 void process1HzTask() {

    G_Dt = (currentTime - oneHZpreviousTime) / 1000000.0;
    oneHZpreviousTime = currentTime;

  }


/*******************************************************************
 * Emergency Stop triggered by manual stopping command
 ******************************************************************/
 void emergencyStop() {

  // set motor commands to 0
  motorCommand[MOTOR3] = 1000;
  motorCommand[MOTOR2] = 1000;
  motorCommand[MOTOR1] = 1000;
  motorCommand[MOTOR4] = 1000;

  // kill the motors
  OCR3B = 2000;
  OCR3C = 2000;
  OCR3A = 2000;
  OCR4A = 2000;

  // disable control processes
  beginControl = false;

  // change system state (doesn't do anything yet)
  status = EMGSTOP;

  // disable all global interrupts
  // cli();

  // signal to base station that emergency stop occurred
  Serial.print('~');

  // do nothing forever
  while(1);
  
}


/*******************************************************************
 * Main loop funtions
 ******************************************************************/
 void loop () {

   currentTime = micros();
   deltaTime = currentTime - previousTime;

   measureCriticalSensors();

   if (baroReadFlag && baroDataReady == 0) { // check to see if we missed a baro-read

    measureBaro();
    baroDataReady = 1;
    baroReadFlag = 0;
   
   }

  // ================================================================
  // 100Hz task loop
  // ================================================================
  if (deltaTime >= 10000) {

    frameCounter++;

    process100HzTask();

    // ================================================================
    // 50Hz task loop
    // ================================================================

    if (frameCounter % TASK_50HZ == 0) {  //  50 Hz tasks

      process50HzTask();

    }

    // ================================================================
    // 10Hz task loop
    // ================================================================
    if (frameCounter % TASK_10HZ == 0) {  //   10 Hz tasks

      process10HzTask1();

    }

    else if ((currentTime - lowPriorityTenHZpreviousTime) > 100000) {

      process10HzTask2();

    }

    else if ((currentTime - lowPriorityTenHZpreviousTime2) > 100000) {

      process10HzTask3();

    }

    // ================================================================
    // 2Hz task loop
    // ================================================================
    if (frameCounter % TASK_2HZ == 0) {

      process2HzTask();
    }


    previousTime = currentTime;

 }

 if (frameCounter >= 100) {

  frameCounter = 0;

 }
}




