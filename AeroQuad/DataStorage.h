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

// Special thanks for 1k space optimization update from Ala42
// http://aeroquad.com/showthread.php?1369-The-big-enhancement-addition-to-2.0-code&p=13359&viewfull=1#post13359

#ifndef _AQ_DATA_STORAGE_H_
#define _AQ_DATA_STORAGE_H_

// Utilities for writing and reading from the EEPROM
float nvrReadFloat(int address) {
  union floatStore {
    byte floatByte[4];
    unsigned short floatUShort[2];
    float floatVal;
  } floatOut;

#ifdef EEPROM_USES_16BIT_WORDS
  for (int i = 0; i < 2; i++) {
    floatOut.floatUShort[i] = EEPROM.read(address + 2*i);
  }
#else
  for (int i = 0; i < 4; i++) {
    floatOut.floatByte[i] = EEPROM.read(address + i);
  }
#endif

  return floatOut.floatVal;
}

void nvrWriteFloat(float value, int address) {
  union floatStore {
    byte floatByte[4];
    unsigned short floatUShort[2];
    float floatVal;
  } floatIn;

  floatIn.floatVal = value;
#ifdef EEPROM_USES_16BIT_WORDS
  for (int i = 0; i < 2; i++) {
    EEPROM.write(address + 2*i, floatIn.floatUShort[i]);
  }
#else
  for (int i = 0; i < 4; i++) {
    EEPROM.write(address + i, floatIn.floatByte[i]);
  }
#endif
}

long nvrReadLong(int address) {
  union longStore {
    byte longByte[4];
    unsigned short longUShort[2];
    long longVal;
  } longOut;  

#ifdef EEPROM_USES_16BIT_WORDS
  for (int i = 0; i < 2; i++) {
    longOut.longUShort[i] = EEPROM.read(address + 2*i);
  }
#else
  for (byte i = 0; i < 4; i++) {
    longOut.longByte[i] = EEPROM.read(address + i);
  }
#endif
    
  return longOut.longVal;
}

void nvrWriteLong(long value, int address) {
  union longStore {
    byte longByte[4];
    unsigned short longUShort[2];
    long longVal;
  } longIn;  

  longIn.longVal = value;
  
#ifdef EEPROM_USES_16BIT_WORDS
  for (int i = 0; i < 2; i++) {
    EEPROM.write(address + 2*i, longIn.longUShort[i]);
  }
#else
  for (int i = 0; i < 4; i++) {
    EEPROM.write(address + i, longIn.longByte[i]);
  }
#endif
}

void nvrReadPID(unsigned char IDPid, unsigned int IDEeprom) {
  struct PIDdata* pid = &PID[IDPid];
  pid->P = nvrReadFloat(IDEeprom);
  pid->I = nvrReadFloat(IDEeprom+4);
  pid->D = nvrReadFloat(IDEeprom+8);
  pid->lastError = 0;
  pid->integratedError = 0;
}

void nvrWritePID(unsigned char IDPid, unsigned int IDEeprom) {
  struct PIDdata* pid = &PID[IDPid];
  nvrWriteFloat(pid->P, IDEeprom);
  nvrWriteFloat(pid->I, IDEeprom+4);
  nvrWriteFloat(pid->D, IDEeprom+8);
}

// contains all default values when re-writing EEPROM
void initializeEEPROM() {

  // new initialization
  PID[ALTITUDE_PID_IDX].P = PIDVARS_altitude[0];
  PID[ALTITUDE_PID_IDX].I = PIDVARS_altitude[1];
  PID[ALTITUDE_PID_IDX].D = PIDVARS_altitude[2];
  PID[ALTITUDE_PID_IDX].windupGuard = PIDVARS_altitude[3];

  PID[ROLL_PID_IDX].P = PIDVARS_roll[0];
  PID[ROLL_PID_IDX].I = PIDVARS_roll[1];
  PID[ROLL_PID_IDX].D = PIDVARS_roll[2];
  PID[ROLL_PID_IDX].windupGuard = PIDVARS_roll[3];

  PID[PITCH_PID_IDX].P = PIDVARS_pitch[0];
  PID[PITCH_PID_IDX].I = PIDVARS_pitch[1];
  PID[PITCH_PID_IDX].D = PIDVARS_pitch[2];
  PID[PITCH_PID_IDX].windupGuard = PIDVARS_pitch[3];

  PID[YAW_PID_IDX].P = PIDVARS_yaw[0];
  PID[YAW_PID_IDX].I = PIDVARS_yaw[1];
  PID[YAW_PID_IDX].D = PIDVARS_yaw[2];
  PID[YAW_PID_IDX].windupGuard = PIDVARS_yaw[3];

  PID[GYRO_X_PID_IDX].P = PIDVARS_gyroX[0];
  PID[GYRO_X_PID_IDX].I = PIDVARS_gyroX[1];
  PID[GYRO_X_PID_IDX].D = PIDVARS_gyroX[2];
  PID[GYRO_X_PID_IDX].windupGuard = PIDVARS_gyroX[3];

  PID[GYRO_Y_PID_IDX].P = PIDVARS_gyroY[0];
  PID[GYRO_Y_PID_IDX].I = PIDVARS_gyroY[1];
  PID[GYRO_Y_PID_IDX].D = PIDVARS_gyroY[2];
  PID[GYRO_Y_PID_IDX].windupGuard = PIDVARS_gyroY[3];

  PID[GYRO_Z_PID_IDX].P = PIDVARS_gyroZ[0];
  PID[GYRO_Z_PID_IDX].I = PIDVARS_gyroZ[1];
  PID[GYRO_Z_PID_IDX].D = PIDVARS_gyroZ[2];
  PID[GYRO_Z_PID_IDX].windupGuard = PIDVARS_gyroZ[3];

  initializePlatformSpecificAccelCalibration();
    

  receiverXmitFactor = 1.0;
  minArmedThrottle = 1150;
  // AKA - old setOneG not in SI - accel->setOneG(500);
  accelOneG = -9.80665; // AKA set one G to 9.8 m/s^2
  for (byte channel = XAXIS; channel < LASTCHANNEL; channel++) {
    receiverSlope[channel] = 1.0;
    receiverOffset[channel] = 0.0;
    receiverSmoothFactor[channel] = 1.0;
  }
  receiverSmoothFactor[ZAXIS] = 0.5;

  flightMode = RATE_FLIGHT_MODE;
  headingHoldConfig = ON;
  aref = 5.0; // Use 3.0 if using a v1.7 shield or use 2.8 for an AeroQuad Shield < v1.7
  
  // Battery Monitor
  #ifdef BattMonitor
    batteryMonitorAlarmVoltage = 3.33;
    batteryMonitorThrottleTarget = 1450;
    batteryMonitorGoingDownTime = 60000;
  #endif

}

void readEEPROM() {

  // readPID(ALTITUDE_PID_IDX, ALTITUDE_PID_GAIN_ADR);
  // readPID(ROLL_PID_IDX, ROLL_PID_GAIN_ADR);
  // readPID(PITCH_PID_IDX, PITCH_PID_GAIN_ADR);
  // readPID(YAW_PID_IDX, YAW_PID_GAIN_ADR);
  // readPID(GYRO_X_PID_IDX, GYRO_X_PID_GAIN_ADR);
  // readPID(GYRO_Y_PID_IDX, GYRO_Y_PID_GAIN_ADR);
  // readPID(GYRO_Z_PID_IDX, GYRO_Z_PID_GAIN_ADR);

  rotationSpeedFactor = readFloat(ROTATION_SPEED_FACTOR_ARD);
  
  // Leaving separate PID reads as commented for now
  // Previously had issue where EEPROM was not reading right data
  // #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
  //   readPID(BARO_ALTITUDE_HOLD_PID_IDX, ALTITUDE_PID_GAIN_ADR);
  //   PID[BARO_ALTITUDE_HOLD_PID_IDX].windupGuard = readFloat(ALTITUDE_WINDUP_ADR);
  //   #if defined AltitudeHoldBaro
  //     baroSmoothFactor = readFloat(ALTITUDE_SMOOTH_ADR);
  //   #endif  
  //   altitudeHoldBump = readFloat(ALTITUDE_BUMP_ADR);
  //   altitudeHoldPanicStickMovement = readFloat(ALTITUDE_PANIC_ADR);
  //   minThrottleAdjust = readFloat(ALTITUDE_MIN_THROTTLE_ADR);
  //   maxThrottleAdjust = readFloat(ALTITUDE_MAX_THROTTLE_ADR);
  //   readPID(ZDAMPENING_PID_IDX, ZDAMP_PID_GAIN_ADR);
  // #endif

  // Mag calibration
  #ifdef HeadingMagHold
    magBias[XAXIS]  = readFloat(XAXIS_MAG_BIAS_ADR);
    magBias[YAXIS]  = readFloat(YAXIS_MAG_BIAS_ADR);
    magBias[ZAXIS]  = readFloat(ZAXIS_MAG_BIAS_ADR);
  #endif
  
  // Battery Monitor
  #ifdef BattMonitor
    batteryMonitorAlarmVoltage = readFloat(BATT_ALARM_VOLTAGE_ADR);
    batteryMonitorThrottleTarget = readFloat(BATT_THROTTLE_TARGET_ADR);
    batteryMonitorGoingDownTime = readFloat(BATT_DOWN_TIME_ADR);
  #endif
  
  // windupGuard = readFloat(WINDUPGUARD_ADR);
  // AKA - added so that each PID has its own windupGuard, will need to be removed once each PID's range is established and put in the EEPROM
  // for (byte i = XAXIS; i < LAST_PID_IDX; i++ ) {
  //   #if defined AltitudeHoldBaro
  //     if (i != BARO_ALTITUDE_HOLD_PID_IDX) {
  //       PID[i].windupGuard = windupGuard;
  //     }
  //   #else
  //     PID[i].windupGuard = windupGuard;
  //   #endif      
  // }
    
  minArmedThrottle = readFloat(MINARMEDTHROTTLE_ADR);
  aref = readFloat(AREF_ADR);
  flightMode = readFloat(FLIGHTMODE_ADR);
  accelOneG = readFloat(ACCEL_1G_ADR);
  headingHoldConfig = readFloat(HEADINGHOLD_ADR);

  #if defined (UseGPSNavigator)
    missionNbPoint = readFloat(GPS_MISSION_NB_POINT_ADR);
    // readPID(GPSROLL_PID_IDX, GPSROLL_PID_GAIN_ADR);
    // readPID(GPSPITCH_PID_IDX, GPSPITCH_PID_GAIN_ADR);
    // readPID(GPSYAW_PID_IDX, GPSYAW_PID_GAIN_ADR);
    
    for (byte location = 0; location < MAX_WAYPOINTS; location++) {
      waypoint[location].longitude = readLong(WAYPOINT_ADR[location].longitude);
      waypoint[location].latitude = readLong(WAYPOINT_ADR[location].latitude);
      waypoint[location].altitude = readLong(WAYPOINT_ADR[location].altitude);
    }    
  #endif

}

void writeEEPROM(){
  cli(); // Needed so that APM sensor data does not overflow

  // writePID(ALTITUDE_PID_IDX, ALTITUDE_PID_GAIN_ADR);
  // writePID(ROLL_PID_IDX, ROLL_PID_GAIN_ADR);
  // writePID(PITCH_PID_IDX, PITCH_PID_GAIN_ADR);
  // writePID(YAW_PID_IDX, YAW_PID_GAIN_ADR);
  // writePID(GYRO_X_PID_IDX, GYRO_X_PID_GAIN_ADR);
  // writePID(GYRO_Y_PID_IDX, GYRO_Y_PID_GAIN_ADR);
  // writePID(GYRO_Z_PID_IDX, GYRO_Z_PID_GAIN_ADR);

  writeFloat(rotationSpeedFactor,ROTATION_SPEED_FACTOR_ARD);
  
  #if defined AltitudeHoldBaro
    // writePID(BARO_ALTITUDE_HOLD_PID_IDX, ALTITUDE_PID_GAIN_ADR);
    // writeFloat(PID[BARO_ALTITUDE_HOLD_PID_IDX].windupGuard, ALTITUDE_WINDUP_ADR);
  #endif

  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    #if defined AltitudeHoldBaro
      writeFloat(baroSmoothFactor, ALTITUDE_SMOOTH_ADR);
    #else
      writeFloat(0.0, ALTITUDE_SMOOTH_ADR);
    #endif
    writeFloat(altitudeHoldBump, ALTITUDE_BUMP_ADR);
    writeFloat(altitudeHoldPanicStickMovement, ALTITUDE_PANIC_ADR);
    writeFloat(minThrottleAdjust, ALTITUDE_MIN_THROTTLE_ADR);
    writeFloat(maxThrottleAdjust, ALTITUDE_MAX_THROTTLE_ADR);
    // writePID(ZDAMPENING_PID_IDX, ZDAMP_PID_GAIN_ADR);
  #else
    writeFloat(0.1, ALTITUDE_SMOOTH_ADR);
    writeFloat(90, ALTITUDE_BUMP_ADR);
    writeFloat(250, ALTITUDE_PANIC_ADR);
    writeFloat(-50, ALTITUDE_MIN_THROTTLE_ADR);
    writeFloat(50, ALTITUDE_MAX_THROTTLE_ADR);
    writeFloat(0.1, ALTITUDE_SMOOTH_ADR);
  #endif
  
  #ifdef HeadingMagHold
    writeFloat(magBias[XAXIS], XAXIS_MAG_BIAS_ADR);
    writeFloat(magBias[YAXIS], YAXIS_MAG_BIAS_ADR);
    writeFloat(magBias[ZAXIS], ZAXIS_MAG_BIAS_ADR);
  #endif

  // writeFloat(windupGuard, WINDUPGUARD_ADR);
  writeFloat(receiverXmitFactor, XMITFACTOR_ADR);

  for(byte channel = XAXIS; channel < LASTCHANNEL; channel++) {
    writeFloat(receiverSlope[channel],  RECEIVER_DATA[channel].slope);
    writeFloat(receiverOffset[channel], RECEIVER_DATA[channel].offset);
    writeFloat(receiverSmoothFactor[channel], RECEIVER_DATA[channel].smooth_factor);
  }

  writeFloat(minArmedThrottle, MINARMEDTHROTTLE_ADR);
  writeFloat(aref, AREF_ADR);
  writeFloat(flightMode, FLIGHTMODE_ADR);
  writeFloat(headingHoldConfig, HEADINGHOLD_ADR);
  writeFloat(accelOneG, ACCEL_1G_ADR);
  writeFloat(SOFTWARE_VERSION, SOFTWARE_VERSION_ADR);
  
  // Battery Monitor
  #ifdef BattMonitor
    writeFloat(batteryMonitorAlarmVoltage, BATT_ALARM_VOLTAGE_ADR);
    writeFloat(batteryMonitorThrottleTarget, BATT_THROTTLE_TARGET_ADR);
    writeFloat(batteryMonitorGoingDownTime, BATT_DOWN_TIME_ADR);
  #endif
  
  #if defined (UseGPSNavigator)
    writeFloat(missionNbPoint, GPS_MISSION_NB_POINT_ADR);
    // writePID(GPSROLL_PID_IDX, GPSROLL_PID_GAIN_ADR);
    // writePID(GPSPITCH_PID_IDX, GPSPITCH_PID_GAIN_ADR);
    // writePID(GPSYAW_PID_IDX, GPSYAW_PID_GAIN_ADR);
    
    for (byte location = 0; location < MAX_WAYPOINTS; location++) {
      writeLong(waypoint[location].longitude, WAYPOINT_ADR[location].longitude);
      writeLong(waypoint[location].latitude, WAYPOINT_ADR[location].latitude);
      writeLong(waypoint[location].altitude, WAYPOINT_ADR[location].altitude);
    }       
  #endif

  sei(); // Restart interrupts
}

void initSensorsZeroFromEEPROM() {

  // Accel initialization from EEPROM
  accelOneG = readFloat(ACCEL_1G_ADR);
  // Accel calibration
  accelScaleFactor[XAXIS] = readFloat(XAXIS_ACCEL_SCALE_FACTOR_ADR);
  runTimeAccelBias[XAXIS] = readFloat(XAXIS_ACCEL_BIAS_ADR);
  accelScaleFactor[YAXIS] = readFloat(YAXIS_ACCEL_SCALE_FACTOR_ADR);
  runTimeAccelBias[YAXIS] = readFloat(YAXIS_ACCEL_BIAS_ADR);
  accelScaleFactor[ZAXIS] = readFloat(ZAXIS_ACCEL_SCALE_FACTOR_ADR);
  runTimeAccelBias[ZAXIS] = readFloat(ZAXIS_ACCEL_BIAS_ADR);

  Serial.println("Read from EEPROM: ");
  Serial.println(accelOneG);
  Serial.println(accelScaleFactor[XAXIS]);
  Serial.println(runTimeAccelBias[XAXIS]);
  Serial.println(accelScaleFactor[YAXIS]);
  Serial.println(runTimeAccelBias[YAXIS]);
  Serial.println(accelScaleFactor[ZAXIS]);
  Serial.println(runTimeAccelBias[ZAXIS]);
}

void storeSensorsZeroToEEPROM() {
  
  // Store accel data to EEPROM
  writeFloat(accelOneG, ACCEL_1G_ADR);
  // Accel Cal
  writeFloat(accelScaleFactor[XAXIS], XAXIS_ACCEL_SCALE_FACTOR_ADR);
  writeFloat(runTimeAccelBias[XAXIS], XAXIS_ACCEL_BIAS_ADR);
  writeFloat(accelScaleFactor[YAXIS], YAXIS_ACCEL_SCALE_FACTOR_ADR);
  writeFloat(runTimeAccelBias[YAXIS], YAXIS_ACCEL_BIAS_ADR);
  writeFloat(accelScaleFactor[ZAXIS], ZAXIS_ACCEL_SCALE_FACTOR_ADR);
  writeFloat(runTimeAccelBias[ZAXIS], ZAXIS_ACCEL_BIAS_ADR);
}

void initReceiverFromEEPROM() {
  receiverXmitFactor = readFloat(XMITFACTOR_ADR);
  
  for(byte channel = XAXIS; channel < LASTCHANNEL; channel++) {
    receiverSlope[channel] = readFloat(RECEIVER_DATA[channel].slope);
    receiverOffset[channel] = readFloat(RECEIVER_DATA[channel].offset);
    receiverSmoothFactor[channel] = readFloat(RECEIVER_DATA[channel].smooth_factor);
  }
}

#endif // _AQ_DATA_STORAGE_H_

