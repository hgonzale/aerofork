#ifndef _AQ_PID_H_
#define _AQ_PID_H_


/*
* Indices used to access PID for each controlled variable.
*/
enum {
  ALTITUDE_PID_IDX = 0,
  ROLL_PID_IDX,
  PITCH_PID_IDX,
  YAW_PID_IDX,
  GYRO_X_PID_IDX,
  GYRO_Y_PID_IDX,
  GYRO_Z_PID_IDX,

  LAST_PID_IDX // keep LAST_PID_IDX at the end of the enum definition
};


/*
* PID parameter declaration
*/
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


/*
* Variable-specific PID parameters
* Adjust the PID parameters for each variable here. 
*
* NOTE 1: The values are set during EEPROM initialization, 
*         which can be found in DataStorage.h.
*
* NOTE 2: The PID parameters for gyro readings are overridden
*         during Kinematics initialization, which can be found
*         in Kinematics_ARG.h
*/

// Altitude {P, I, D, windup}
float PIDVARS_altitude[] = {40.0, 2.0, 0.0, 100.0};

// Roll {P, I, D, windup}
float PIDVARS_roll[] = {5.0, 0.1, 0.0, 100.0};

// Pitch {P, I, D, windup}
float PIDVARS_pitch[] = {5.0, 0.1, 0.0, 100.0};

// Yaw {P, I, D, windup}
float PIDVARS_yaw[] = {0.0, 0.3, 0.0, 100.0};

// Gyro X {P, I, D, windup}
float PIDVARS_gyroX[] = {1.0, 1.0, 0.0, 100.0};

// Gyro Y {P, I, D, windup}
float PIDVARS_gyroY[] = {1.0, 1.0, 0.0, 100.0};

// Gyro Z {P, I, D, windup}
float PIDVARS_gyroZ[] = {1.0, 1.0, 0.0, 100.0};


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


/***********************************************************************************************************************************
* initPID
*
* Initialize all error terms to zero and begin time measurement.
************************************************************************************************************************************/
void initPID() {

  for (byte idx; idx < LAST_PID_IDX; idx++) {
    PID[idx].lastError = 0.0;
    PID[idx].lastLastError = 0.0;
    PID[idx].integratedError = 0.0;
    PID[idx].satIntegratedError = 0.0;
    PID[idx].previousPIDTime = currentTime;
    PID[idx].integralSwitch = true;
  }

}


/************************************************************************************************************************************
* updatePID
*
* reference: the desired value of the variable being controlled
* state: the current value of the variable being controlled
* PIDparamters: the address of the PIDdata associated with the variable being controlled (e.g. &PIDdata[VAR_PID_IDX])
* int_flag: true when updatePID is called by an interrupt. In this case, we know exactly what the 'dT' term is ahead of time.
*
* Executes an iteration of PID control. Important features are the use of an integral windup guard and recursive discrete-time
* output calculations. Additionally, the user may pass in just the error rather than the reference and state.
************************************************************************************************************************************/
float updatePID(float reference, float state, struct PIDdata *PIDparameters, boolean int_flag) {

  // time update -- use pre-computed values if updatePID has been called by an interrupt
  const float deltaPIDTime = (int_flag) ? DELTA_T : (currentTime - PIDparameters->previousPIDTime) / 1000000.0;
  const float inv_deltaPIDTime = (int_flag) ? INV_DELTA_T : 1 / deltaPIDTime;
  PIDparameters->previousPIDTime = currentTime;

  float error = reference - state;

  // allow for error integration if there is no windup
  PIDparameters->integralSwitch = checkSwitch(error,PIDparameters->integratedError - PIDparameters->satIntegratedError);

  if (PIDparameters->integralSwitch) PIDparameters->integratedError += error * deltaPIDTime;

  // saturate integratedError
  PIDparameters->satIntegratedError = constrain(PIDparameters->integratedError,-PIDparameters->windupGuard,PIDparameters->windupGuard);

  // Controller outputs the recursive discrete-time signal:
  // u(k) += (Kp + Kd/delta) * e(k) - (Kp - Ki*delta + 2*Kd/delta) * e(k-1) + (Kd/delta) * e(k-2)
  PIDparameters->output += (PIDparameters->P + PIDparameters->D * inv_deltaPIDTime) * error - (PIDparameters->P - PIDparameters->integralSwitch * PIDparameters->I * deltaPIDTime + 2 * PIDparameters->D * inv_deltaPIDTime) * PIDparameters->lastError + (PIDparameters->D * inv_deltaPIDTime) * PIDparameters->lastLastError; 
  
  // update errors
  PIDparameters->lastLastError = PIDparameters->lastError;
  PIDparameters->lastError = error;

  return PIDparameters->output;
}

/*
* Call updatePID when the error is known ahead of time.
*/
float updatePID(float error, struct PIDdata *PIDparameters, boolean int_flag) {
  return updatePID(error, 0, PIDparameters, int_flag);
}


/**
* zeroIntegralError
*
* Sets the integratedError to zero for all PID controllers.
* (Leftover from original AQ code)
*/
void zeroIntegralError() __attribute__ ((noinline));
void zeroIntegralError() {
  for (byte axis = 0; axis < LAST_PID_IDX; axis++) {
    PID[axis].integratedError = 0;
    PID[axis].previousPIDTime = currentTime;
  }
}

#endif // _AQ_PID_H_


