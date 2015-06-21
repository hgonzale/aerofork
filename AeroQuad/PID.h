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

#ifndef _AQ_PID_H_
#define _AQ_PID_H_

enum {
  RATE_XAXIS_PID_IDX = 0,
  RATE_YAXIS_PID_IDX,
  ZAXIS_PID_IDX,
  ATTITUDE_XAXIS_PID_IDX,
  ATTITUDE_YAXIS_PID_IDX,
  HEADING_HOLD_PID_IDX,
  ATTITUDE_GYRO_XAXIS_PID_IDX,
  ATTITUDE_GYRO_YAXIS_PID_IDX,
  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    BARO_ALTITUDE_HOLD_PID_IDX,
    ZDAMPENING_PID_IDX,
  #endif
  LAST_PID_IDX  // keep this definition at the end of this enum
};

//// PID Variables
struct PIDdata {
  float P, I, D;
  float lastError; // error from previous iteration (@ k-1)
  float lastLastError; // error from two iterations ago (@ k-2)
  float output;
  float previousPIDTime;
  float integratedError;
  float satIntegratedError;
  float windupGuard;
  boolean integralSwitch;
} PID[LAST_PID_IDX];

// This struct above declares the variable PID[] to hold each of the PID values for various functions
// The following constants are declared in AeroQuad.h
// ROLL = 0, PITCH = 1, YAW = 2 (used for Arcobatic Mode, gyros only)
// ROLLLEVEL = 3, PITCHLEVEL = 4, LEVELGYROROLL = 6, LEVELGYROPITCH = 7 (used for Stable Mode, accels + gyros)
// HEADING = 5 (used for heading hold)
// ALTITUDE = 8 (used for altitude hold)
// ZDAMPENING = 9 (used in altitude hold to dampen vertical accelerations)
float windupGuard; // Read in from EEPROM


/************************************************************************************************************************************
* checkSwitch
*
* error: the current error fed into the PID controller
* errorDif: the difference between the true integrated error and the saturated integrated error (both from the previous iteration)
*
* Returns 'true' if the sign of the current error is opposite the sign of the difference in integrated errors. Simply put, returns
* 'true' if the current error will bring the integrated error back toward zero.
************************************************************************************************************************************/
boolean checkSwitch(float error, float errorDif) {
  return (error > 0 && errorDif < 0) || (error < 0 && errorDif > 0);
}


/**
* updatePID
*
* reference: the desired value of the variable being controlled
* state: the current value of the variable being controlled
* PIDparamters: the address of the PIDdata associated with the variable being controlled (e.g. &PIDdata[VAR_IDX])
*
* Executes an iteration of PID control. Important features are the use of an integral windup guard and recursive discrete-time
* output calculations. 
*/
float updatePID(float reference, float state, struct PIDdata *PIDparameters) {

  // time update
  const float deltaPIDTime = (currentTime - PIDparameters->previousPIDTime) / 1000000.0;
  PIDparameters->previousPIDTime = currentTime;

  float error = reference - state;

  // allow for error integration if there is no windup
  PIDparameters->integralSwitch = checkSwitch(error,PIDparameters->integratedError - PIDparameters->satIntegratedError);

  if (PIDparameters->integralSwitch) PIDparameters->integratedError += error * deltaPIDTime;

  // saturate integratedError
  PIDparameters->satIntegratedError = constrain(PIDparameters->integratedError,-PIDparameters->windupGuard,PIDparameters->windupGuard);

  // Controller outputs the recursive discrete-time signal:
  // u(k) += (Kp + Kd/delta) * e(k) - (Kp - Ki*delta + 2*Kd/delta) * e(k-1) + (Kd/delta) * e(k-2)
  PIDparameters->output += (PIDparameters->P + PIDparameters->D / deltaPIDTime) * error - (PIDparameters->P - PIDparameters->I * deltaPIDTime + 2 * PIDparameters->D / deltaPIDTime) * PIDparameters->lastError + (PIDparameters->D / deltaPIDTime) * PIDparameters->lastLastError; 
 
  // update errors
  PIDparameters->lastLastError = PIDparameters->lastError;
  PIDparameters->lastError = error;

  return PIDparameters->output;
}



void zeroIntegralError() __attribute__ ((noinline));
void zeroIntegralError() {
  for (byte axis = 0; axis <= ATTITUDE_YAXIS_PID_IDX; axis++) {
    PID[axis].integratedError = 0;
    PID[axis].previousPIDTime = currentTime;
  }
}

#endif // _AQ_PID_H_


