//
//  controlLoop.h
//  PID Controller
//
//  Created by Christa Stathopoulos on 3/19/14.
//  Copyright (c) 2014 Christa Stathopoulos. All rights reserved.
//	Modified by Xiaoyang Ye.
//

#ifndef _CONTROL_LOOP_H_
#define _CONTROL_LOOP_H_

//inputs to PID:
//(1) w: an array (size 4) with the applied inputs
//(2) states: an array of the states used to form the error signal (qz, qtheta, qphi, qpsi)
//(3) ukpo: u(k+1): represents the control signal
//(4) delta: the step size of the discretization
//(5) len: the length of all of the arrays (should be equal to 4)

//define global variables
volatile float ek[4] = {0.0, 0.0, 0.0, 0.0};
volatile float ekmo[4] = {0.0, 0.0, 0.0, 0.0}; //previous error.
volatile float ekpo[4] = {0.0, 0.0, 0.0, 0.0}; //current error.
volatile float uk[4] = {0.0, 0.0, 0.0, 0.0}; //control signal.
float yk[4] = {0.0, 0.0, 0.0, 0.0}; // current controller output
float yk_prev[4] = {0.0, 0.0, 0.0, 0.0}; // previous filtered controller output

// The maximum allowable change in controller output (since the last iteration)
const int FILTER_TOLERANCE = 50;

//the PID gains (proportional, integral, derivative for qz, qtheta, qphi, qpsi) 
float Kp[] = {40.0, 5.0, 5.0, 0.0};
float Ki[] = {2.0, 0.1, 0.1, 0.3};
float Kd[] = {0.0, 0.0, 0.0, 0.0};

//   float Kp[] = {25.0, 100.0, 100.0, 0.0};
//   float Ki[] = {0.6, 150.0, 150.0, 0.0};
//   float Kd[] = {0.0, -350.0, -350.0, 0.0};


// Position vectors used for PID control
float positionRef[4] = {0.0,0.0,0.0,0.0};
float positionReal[4] = {0.0,0.0,0.0,0.0};


/**
* updateError
*
* w[4] is the desired state: [qz, qtheta, qphi, qpsi] ~ [alt, roll, pitch, yaw]
* states[4] is the current state [qz, qtheta, qphi, qpsi] as determined by the barometer and kinematics angles
*/
void updateError(float w[4], float states[4]) {
    for (int i = 0; i<3; i++) {
        ekmo[i] = ek[i];
        ek[i] = ekpo[i];
        ekpo[i] = w[i] - states[i];
    }
}

/**
* PIDControl
*
* Maps the control signals (uk) to the output signals (yk) via multiplication by a pre-determined matrix.
* Output signals (yk) are constrained so as to limit dramatic changes in acceleration.
*
* NOTE: Due to the need for consistent delta values, updates to the control signals (uk) are handled by an ISR which can be in AeroQuad.ino.
*/
void PIDControl(float w[4], float states[4]) {
    
    //define an intermediate ukpo signal (pre multiplying by the matrix)
    //form the PID output:
    //implementation formula: ukpo_int - uk = (Kd/delta + K + Ki*delta)ekpo - (K + 2Kd/delta)ek + Kd/delta*ekmo

    // Save old yk values
    yk_prev[0] = yk[0];
    yk_prev[1] = yk[1];
    yk_prev[2] = yk[2];
    yk_prev[3] = yk[3];

    //multiply ukpo_int by the matrix to get your final values for ukpo
    //I computed the multiplication on paper, and the solutions are as follows:
    yk[0] = .25*uk[0] + .5*uk[1] + .5*uk[2] + .25*uk[3];
    yk[1] = .25*uk[0] - .5*uk[1] + .5*uk[2] - .25*uk[3];
    yk[2] = .25*uk[0] + .5*uk[1] - .5*uk[2] - .25*uk[3];
    yk[3] = .25*uk[0] - .5*uk[1] - .5*uk[2] + .25*uk[3];

    // Limit change in controller output
    yk[0] = constrain(yk[0], yk_prev[0] - FILTER_TOLERANCE, yk_prev[0] + FILTER_TOLERANCE);
    yk[1] = constrain(yk[1], yk_prev[1] - FILTER_TOLERANCE, yk_prev[1] + FILTER_TOLERANCE);
    yk[2] = constrain(yk[2], yk_prev[2] - FILTER_TOLERANCE, yk_prev[2] + FILTER_TOLERANCE);
    yk[3] = constrain(yk[3], yk_prev[3] - FILTER_TOLERANCE, yk_prev[3] + FILTER_TOLERANCE);
}

#endif