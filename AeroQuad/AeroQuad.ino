
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


   float indic = 0.0;
   int counterVar = 0;



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

// #if defined(ReceiverSBUS) && defined(SlowTelemetry)
//   #error "Receiver SWBUS and SlowTelemetry are in conflict for Seria2, they can't be used together"
// #endif

#include <EEPROM.h>
#include <Wire.h>
#include <Device_I2C.h>
#include <GlobalDefined.h>
#include "AeroQuad.h"
#include "PID.h"
#include <AQMath.h>
#include <FourtOrderFilter.h>


/* XBee setup */
XBee xbee = XBee();
XBeeResponse response = XBeeResponse():
ZBRxResponse msr = ModemStatusResponse();

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
// #if defined(quadXConfig)
  #include "FlightControlQuadX.h"
// #elif defined(quadPlusConfig)
//   #include "FlightControlQuadPlus.h"
// #endif

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
#if defined(WirelessTelemetry) 
  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    #define SERIAL_PORT Serial3
  #else    // force 328p to use the normal port
    #define SERIAL_PORT Serial
  #endif
#else  
  #if defined(SERIAL_USES_USB)   // STM32 Maple
    #define SERIAL_PORT SerialUSB
    #undef BAUD
    #define BAUD
  #else
    #define SERIAL_PORT Serial
  #endif
#endif  

// #ifdef SlowTelemetry
//   #include <AQ_RSCode.h>
// #endif

// #ifdef SoftModem
//   #include <AQ_SoftModem.h>
// #endif


// Include this last as it contains objects from above declarations
#include "AltitudeControlProcessor.h"
#include "FlightControlProcessor.h"
#include "FlightCommandProcessor.h"
#include "HeadingHoldProcessor.h"
#include "DataStorage.h"

#if defined(UseGPS) || defined(BattMonitor)
  #include "LedStatusProcessor.h"
#endif  

#if defined(MavLink)
  #include "MavLink.h"
#else
  #include "SerialCom.h"
#endif



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
  xbee.begin(BAUD);
  #endif

  pinMode(LED_Green, OUTPUT);
  digitalWrite(LED_Green, LOW);
  initCommunication();

  readEEPROM(); // defined in DataStorage.h
  boolean firstTimeBoot = false;
  
  if (readFloat(SOFTWARE_VERSION_ADR) != SOFTWARE_VERSION) { // If we detect the wrong soft version, we init all parameters

    initializeEEPROM();
    writeEEPROM();
    firstTimeBoot = true;

  }
  
  initPlatform();
  
  initializeMotors(FOUR_Motors);

  initializeReceiver(LASTCHANNEL);
  initReceiverFromEEPROM();
  
  while (!calibrateGyro()); // this make sure the craft is still befor to continue init process
  
  if (firstTimeBoot) {

    computeAccelBias();
    writeEEPROM();

  }
  
  setupFourthOrder();
  initSensorsZeroFromEEPROM();
  
  initializeKinematics();

  initPID();

  #ifdef HeadingMagHold
  vehicleState |= HEADINGHOLD_ENABLED;
  initializeMagnetometer();
  initializeHeadingFusion();
  #endif

  // #if defined(BinaryWrite) || defined(BinaryWritePID)
  //   #ifdef OpenlogBinaryWrite
  //     binaryPort = &Serial1;
  //     binaryPort->begin(115200);
  //     delay(1000);
  //   #else
  //    binaryPort = &Serial;
  //   #endif
  // #endif
  
  #if defined(UseGPS)
  initializeGps();
  #endif 

  // #ifdef SlowTelemetry
  //    initSlowTelemetry();
  // #endif

  previousTime = micros();
  digitalWrite(LED_Green, HIGH);
  safetyCheck = 0;
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


/*Interrupt for PID computation
  Runs at 100Hz
  Using DELTA_T = 10ms = 0.01s
  INV_DELTA_T = 1 / DELTA_T = 100 /s
*/
  ISR(TIMER5_COMPA_vect) {

  if (dataReady) { // if new data is available

    u_alt = updatePID(0, &PID[ALTITUDE_PID_IDX], true);
    u_roll = updatePID(0, roll, &PID[ROLL_PID_IDX], true);
    u_pitch = updatePID(0, pitch, &PID[PITCH_PID_IDX], true);
    u_yaw = updatePID(0, &PID[YAW_PID_IDX], true);

    dataReady = 0;
    pidReady = 1;
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

  if (startBaroMeasure && frameCounter % THROTTLE_ADJUST_TASK_SPEED == 0) {

    measureBaroSum();

  }
  
  if (beginControl) {
    processStabilityControl();
  }

}

/*******************************************************************
 * 50Hz task
 ******************************************************************/
 void process50HzTask() {

  G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
  fiftyHZpreviousTime = currentTime;

/* 		if (firstRead) {
		
  			xbee.begin(BAUD);
  			firstRead = false;
			
		} */

      }

/*******************************************************************
 * 10Hz task
 ******************************************************************/
 void process10HzTask1() {

  if (startBaroMeasure) {

    evaluateBaroAltitude();

    if (calibrateReady) {

      ENQueueSensorReading(getBaroAltitude(), 0);

    }

    else {

      if (startCalibrate) {

        Addz(getBaroAltitude());

      }	

    }

  }

}

/*******************************************************************
 * low priority 10Hz task 2
 ******************************************************************/
 void process10HzTask2() {

  G_Dt = (currentTime - lowPriorityTenHZpreviousTime) / 1000000.0;
  lowPriorityTenHZpreviousTime = currentTime;

  // Listen for configuration commands and reports telemetry
  readSerialCommand();
  sendSerialTelemetry();

}

/*******************************************************************
 * low priority 10Hz task 3
 ******************************************************************/
 void process10HzTask3() {

  G_Dt = (currentTime - lowPriorityTenHZpreviousTime2) / 1000000.0;
  lowPriorityTenHZpreviousTime2 = currentTime;

  #if defined(UseGPS) || defined(BattMonitor)
  processLedStatus();
  #endif

  #ifdef SlowTelemetry
  updateSlowTelemetry10Hz();
  #endif
}



void process2HzTask() {

 xbee.readPacket();

 if (xbee.getResponse().isAvailable()) {

  if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {

   xbee.getResponse().getModemStatusResponse(msr);
	// the local XBee sends this response on certain events, like association/dissociation

   if (msr.getStatus() == ASSOCIATED) {

   } 

   else if (msr.getStatus() == DISASSOCIATED) {

    beginControl = false;
    calibrateESC = 2;

  } 

  else {

  }

}

} 

// Serial heartbeat code
if (beginControl) {

  if (resetEmergencyStop) {

    countStop = 0;

  } else {

    countStop++;

  }

  resetEmergencyStop = false;

}

}



/*******************************************************************
 * 1Hz task 
 ******************************************************************/
 void process1HzTask() {

  /*#ifdef MavLink
    G_Dt = (currentTime - oneHZpreviousTime) / 1000000.0;
    oneHZpreviousTime = currentTime;
    
    sendSerialHeartbeat();   
  #endif*/

   //  if (beginControl) {

   //    if (msg == '>') {

   //     countStop = 0;

   //   } else {

   //     countStop++;

   //   }

   //   msg = ' ';

   // }

  }


/*******************************************************************
 * Emergency Stop triggered by manual stopping command
 ******************************************************************/
 void emergencyStop() {

  // kill the motors
  OCR3B = 0;
  OCR3C = 0;
  OCR3A = 0;
  OCR4A = 0;

  // disable control processes
  beginControl = false;

  // change system state (doesn't do anything yet)
  status = EMGSTOP;

  // set motor commands to 0
  motorCommand[MOTOR3] = 0;
  motorCommand[MOTOR2] = 0;
  motorCommand[MOTOR1] = 0;
  motorCommand[MOTOR4] = 0;

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

  // emergency stop check
   if (countStop > 5) {

    emergencyStop();

    calibrateESC = 2;

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
    if (frameCounter % TASK_2HZ == 0) {  //   2 Hz tasks

      if (beginControl) {

       process2HzTask();

     }

   }

   previousTime = currentTime;

 }

 if (frameCounter >= 100) {

  frameCounter = 0;

}
}




