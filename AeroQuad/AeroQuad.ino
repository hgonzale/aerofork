
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


//Interrupt test variables and functions
   volatile boolean togglePin = true;
   volatile int myFlag = 0;
   volatile int dummyCommand[8] = {0,0,0,0,0,0,0,0};

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

// #if defined(AutoLanding) && (!defined(AltitudeHoldBaro) || !defined(AltitudeHoldRangeFinder))
//   #error "AutoLanding NEED AltitudeHoldBaro and AltitudeHoldRangeFinder defined"
// #endif

// #if defined(ReceiverSBUS) && defined(SlowTelemetry)
//   #error "Receiver SWBUS and SlowTelemetry are in conflict for Seria2, they can't be used together"
// #endif

// #if defined (CameraTXControl) && !defined (CameraControl)
//   #error "CameraTXControl need to have CameraControl defined"
// #endif 

#include <EEPROM.h>
#include <Wire.h>
#include <Device_I2C.h>
#include <GlobalDefined.h>
#include "AeroQuad.h"
#include "PID.h"
#include <AQMath.h>
#include <FourtOrderFilter.h>

// #ifdef BattMonitor
//   #include <BatteryMonitorTypes.h>
// #endif

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
#include <controlLoop.h>

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

// #ifdef AltitudeHoldRangeFinder
//   #define XLMAXSONAR 
// #endif


// Altitude declaration
#include <BarometricSensor_MS5611.h>
//

// Battery monitor declaration
// #ifdef BattMonitor

//   #define BattDefaultConfig DEFINE_BATTERY(0, 0, 13.35, 0.31, BM_NOPIN, 0, 0)

// #else

  #undef BattMonitorAutoDescent
  #undef POWERED_BY_VIN

// #endif


// #undef CameraControl
// #undef OSD

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


// #if defined(ReceiverHWPPM)
//   #include <Receiver_HWPPM.h>
// #elif defined(ReceiverPPM)
//   #include <Receiver_PPM.h>
// #elif defined(AeroQuad_Mini) && (defined(hexPlusConfig) || defined(hexXConfig) || defined(hexY6Config))
//   #include <Receiver_PPM.h>
// #elif defined(RemotePCReceiver)
//   #include <Receiver_RemotePC.h>
// #elif defined(ReceiverSBUS)
//   #include <Receiver_SBUS.h>
// #elif defined(RECEIVER_328P)
//   #include <Receiver_328p.h>
// #elif defined(RECEIVER_MEGA)
//   #include <Receiver_MEGA.h>
#if defined(RECEIVER_APM)
  #include <Receiver_APM.h>
#endif
// #elif defined(RECEIVER_STM32PPM)
//   #include <Receiver_STM32PPM.h>  
// #elif defined(RECEIVER_STM32)
//   #include <Receiver_STM32.h>  
// #endif

// #if defined(UseAnalogRSSIReader) 
//   #include <AnalogRSSIReader.h>
// #elif defined(UseEzUHFRSSIReader)
//   #include <EzUHFRSSIReader.h>
// #elif defined(UseSBUSRSSIReader)
//   #include <SBUSRSSIReader.h>
// #endif



//********************************************************
//********************** MOTORS DECLARATION **************
//********************************************************
  #include <Motors_PWM_Timer.h>

// #if defined(MOTOR_PWM)
//   #include <Motors_PWM.h>
// #elif defined(MOTOR_PWM_Timer)
//   #include <Motors_PWM_Timer.h>
// #elif defined(MOTOR_APM)
//   #include <Motors_APM.h>
// #elif defined(MOTOR_I2C)
//   #include <Motors_I2C.h>
// #elif defined(MOTOR_STM32)
//   #include <Motors_STM32.h>    
// #endif

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
// #if defined(XLMAXSONAR)
//   #include <MaxSonarRangeFinder.h>
// #endif 
//********************************************************
//*************** BATTERY MONITOR DECLARATION ************
//********************************************************
// #ifdef BattMonitor
//   #include <BatteryMonitor.h>
//   #ifndef BattCustomConfig
//     #define BattCustomConfig BattDefaultConfig
//   #endif
//   struct BatteryData batteryData[] = {BattCustomConfig};
// #endif
//********************************************************
//************** CAMERA CONTROL DECLARATION **************
//********************************************************
// used only on mega for now
//#if defined(CameraControl_STM32)
//  #include <CameraStabilizer_STM32.h>
//#elif defined(CameraControl)
//  #include <CameraStabilizer_Aeroquad.h>
//#endif
//
//#if defined (CameraTXControl)
//  #include <CameraStabilizer_TXControl.h>
//#endif

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

    /* Timer 1 interrupt setup */
    TCCR1A = 0; // initialize TCRR1A register to 0
    TCCR1B = 0; // same for TCCR1B register
    TCNT1 = 0; // initialize counter value to 0 (for Timer 1)

    // Pre-Scale values for OCRXA register:
    // 624 --> 100Hz
    // 1249 --> 50Hz
    // 3124 --> 20Hz

    OCR1A = 645;
    TCCR1B |= (1 << WGM12); // Enable CTC interrupt
    TCCR1B |= (1 << CS12); // enable 256 pre-scaler
    TIMSK1 |= (1 << OCIE1A);     


    /* Timer 5 interrupt setup */
    TCCR5A = 0; // initialize TCRR5A register to 0
    TCCR5B = 0; // same for TCCR5B register
    TCNT5 = 0; // initialize counter value to 0 (for Timer 5)

    OCR5A = 624;
    TCCR5B |= (1 << WGM12); // Enable CTC interrupt
    TCCR5B |= (1 << CS12); // enable 256 pre-scaler
    TIMSK5 |= (1 << OCIE1A);
    
    sei(); // re-enable global interrupts

    SERIAL_BEGIN(BAUD);
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
  
  // Initialize sensors
  // If sensors have a common initialization routine
  // insert it into the gyro class because it executes first
  initializeGyro(); // defined in Gyro.h
  
  while (!calibrateGyro()); // this make sure the craft is still befor to continue init process
  
  initializeAccel(); // defined in Accel.h
  
  if (firstTimeBoot) {

    computeAccelBias();
    writeEEPROM();

  }
  
  setupFourthOrder();
  initSensorsZeroFromEEPROM();
  
  // Integral Limit for attitude mode
  // This overrides default set in readEEPROM()
  // Set for 1/2 max attitude command (+/-0.75 radians)
  // Rate integral not used for now
  // PID[ATTITUDE_XAXIS_PID_IDX].windupGuard = 0.375;
  // PID[ATTITUDE_YAXIS_PID_IDX].windupGuard = 0.375;
  
  // Flight angle estimation
  initializeKinematics();

  initPID();

  #ifdef HeadingMagHold
    vehicleState |= HEADINGHOLD_ENABLED;
    initializeMagnetometer();
    initializeHeadingFusion();
  #endif
  
  // // Optional Sensors
  // #ifdef AltitudeHoldBaro
    initializeBaro();
    vehicleState |= ALTITUDEHOLD_ENABLED;
  // #endif
  
  // #ifdef BattMonitor
  //   initializeBatteryMonitor(sizeof(batteryData) / sizeof(struct BatteryData), batteryMonitorAlarmVoltage);
  //   vehicleState |= BATTMONITOR_ENABLED;
  // #endif

  // #if defined(SERIAL_LCD)
  //   InitSerialLCD();
  // #endif

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
  Runs at 100Hz
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
    // u_alt = updatePID(1.0, altitude, &PID[ALTITUDE_PID_IDX], true);
    u_alt = updatePID(0, &PID[ALTITUDE_PID_IDX], true);
    u_roll = updatePID(0, roll, &PID[ROLL_PID_IDX], true);
    u_pitch = updatePID(0, pitch, &PID[PITCH_PID_IDX], true);
    u_yaw = updatePID(0, &PID[YAW_PID_IDX], true);
    //u_yaw = updatePID(0, yaw, &PID[YAW_PID_IDX], true);

    dataReady = 0;
    pidReady = 1;

    counterVar++;
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

	if (calibrateReadyTilt) {
		// ENQueueSensorReading(kinematicsAngle[0] - rollBias, 1);
		// ENQueueSensorReading(kinematicsAngle[1] - pitchBias, 2);
		// ENQueueSensorReading(kinematicsAngle[2] - yawBias, 3);
	}
	else {
    if (startCalibrate) {
			AddTilt(kinematicsAngle[0],kinematicsAngle[1],kinematicsAngle[2]);
		}
	}

  
 //  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder

 //    zVelocity = (filteredAccel[ZAXIS] * (1 - accelOneG * invSqrt(isq(filteredAccel[XAXIS]) + isq(filteredAccel[YAXIS]) + isq(filteredAccel[ZAXIS])))) - runTimeAccelBias[ZAXIS] - runtimeZBias;

	// if (!runtimaZBiasInitialized) {

 //      runtimeZBias = (filteredAccel[ZAXIS] * (1 - accelOneG * invSqrt(isq(filteredAccel[XAXIS]) + isq(filteredAccel[YAXIS]) + isq(filteredAccel[ZAXIS])))) - runTimeAccelBias[ZAXIS];
 //      runtimaZBiasInitialized = true;

 //    }

 //    estimatedZVelocity += zVelocity;
 //    estimatedZVelocity = (velocityCompFilter1 * zVelocity) + (velocityCompFilter2 * estimatedZVelocity);

 //  #endif    

  #if defined(AltitudeHoldBaro)

  if (startBaroMeasure) {
  	if (frameCounter % THROTTLE_ADJUST_TASK_SPEED == 0) {
  		measureBaroSum();
  	}
  }

  #endif

  if (startBaroMeasure) {
    measureBaro();
  }
  
  processFlightControl_alt();
  //processFlightControl();
  
  //
  // #if defined(BinaryWrite)
  //   if (fastTransfer == ON) {
  //     // write out fastTelemetry to Configurator or openLog
  //     fastTelemetry();
  //   }
  // #endif      
  // //

  // //
  // #ifdef SlowTelemetry
  //   updateSlowTelemetry100Hz();
  // #endif
  // //

  // //
  #if defined(UseGPS)
    updateGps();
  #endif      
  // //

}

/*******************************************************************
 * 50Hz task
 ******************************************************************/
 void process50HzTask() {

   if (beginControl) {

/* 		if (firstRead == true) {
		
			xbee.begin(BAUD);
			firstRead = false;
			
		} */

      G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
      fiftyHZpreviousTime = currentTime;

    }

  }

/*******************************************************************
 * 10Hz task
 ******************************************************************/
 void process10HzTask1() {

  // #if defined(HeadingMagHold)

  //   G_Dt = (currentTime - tenHZpreviousTime) / 1000000.0;
  //   tenHZpreviousTime = currentTime;

  //   measureMagnetometer(kinematicsAngle[XAXIS], kinematicsAngle[YAXIS]);

  //   calculateHeading();

  // #endif

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
	
    /* xbee.readPacket();
    
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
		
	} */
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

    if (beginControl) {

      if (msg == '>') {

       countStop = 0;

     } else {

       countStop++;

     }

     msg = ' ';

   }

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

  // keep QR from doing anything else
  stopAll = true;

  // change system state
  status = EMGSTOP;
  
}


/*******************************************************************
 * Main loop funtions
 ******************************************************************/
void loop () {

  while (stopAll) {}; // don't allow anything to happen

	currentTime = micros();
	deltaTime = currentTime - previousTime;


	measureCriticalSensors();


	if (countStop > 5) {

		beginControl = false;
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




