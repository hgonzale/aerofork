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

#include <Motors_PWM_Timer.h>
//
//  PID Controller
//
//  Created by Christa Stathopoulos on 3/19/14.
//  Copyright (c) 2014 Christa Stathopoulos. All rights reserved.
//
//inputs to PID:
//(1) w: an array (size 4) with the applied inputs
//(2) states: an array of the states used to form the error signal (qz, qtheta, qphi, qpsi)

//(3) ukpo: u(k+1): represents the control signal

//(4) delta: the step size of the discretization
//(5) len: the length of all of the arrays (should be equal to 4)

//define global variables
float ek[4] = {0.0, 0.0, 0.0, 0.0};
float ekmo[4] = {0.0, 0.0, 0.0, 0.0}; //previous error.
float ekpo[4] = {0.0, 0.0, 0.0, 0.0}; //next error.
float uk[4] = {0.0, 0.0, 0.0, 0.0}; //control signal.
float uk_int[4] = {0.0, 0.0, 0.0, 0.0}; //intermediate control signal.

//w[4] is the desired state: certain altitude and orientation.

void PIDControl(float w[], float states[4], float delta) {

	/*int len = (sizeof(w)/sizeof(w[0]));

    if (len != 4) {

        Serial.println("Error: you do not have the correct number of inputs");
    }*/

    //form the error signal:
        
    for (int i=0; i<4; i++) {

        ekpo[i] = w[i] - states[i];

    }
    
    
    //the PID gains (proportional, integral, derivative for qz, qtheta, qphi, qpsi)
    float Kp[] = {1000.00, 10.00, 10.00, 25.00};
        
    float Ki[] = {300.00, .50, 1.00, 5.00};
        
    float Kd[] = {1000.00, 10.00, 10.00, 25.00};
    
    //define an intermediate ukpo signal (pre multiplying by the matrix)
    
    //form the PID output:
    //implementation formulat: ukpo_int - uk = (Kd/delta + K + Ki*delta)ekpo - (K + 2Kd/delta)ek + Kd/delta*ekmo
        	
    for (int j=0; j<4; j++) {

        uk_int[j] = uk[j] + (Kd[j]/delta + Kp[j] + Ki[j]*delta)*ekpo[j] - (Kp[j] + 2*Kd[j]/delta)*ek[j] + (Kd[j]/delta)*ekmo[j];

    }
    
    //multiply ukpo_int by the matrix to get your final values for ukpo
    //I computed the multiplication on paper, and the solutions are as follows:
    uk[0] = .25*uk_int[0] -.5*uk_int[2] + .25*uk_int[3];
    uk[1] = .25*uk_int[0] +.5*uk_int[1] - .25*uk_int[3];
    uk[2] = .25*uk_int[0] +.5*uk_int[2] + .25*uk_int[3];
    uk[3] = .25*uk_int[0] -.5*uk_int[1] - .25*uk_int[3]; 
	//output control signals.

    //update the arrays:
    for (int a=0; a<4; a++) {

		ekmo[a] = ek[a];
        ek[a] = ekpo[a];

    }

}

#endif

//UPDATED QUESTOINS:
//1. are the indices on my for loops correct?
//2. does my final loop (for updating the global variables) look ok?
//3. how do the global variables get initialized?