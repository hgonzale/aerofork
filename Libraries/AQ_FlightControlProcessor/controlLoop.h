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
float ekpo[4] = {0.0, 0.0, 0.0, 0.0}; //current error.
float uk[4] = {0.0, 0.0, 0.0, 0.0}; //control signal.
float yk[4] = {0.0, 0.0, 0.0, 0.0};

//w[4] is the desired state: certain altitude and orientation.

void PIDControl(float w[4], float states[4], float delta) {

    //form the error signal:
        
    for (int i=0; i<4; i++) {

        ekpo[i] = w[i] - states[i];

    }
    
    
    //the PID gains (proportional, integral, derivative for qz, qtheta, qphi, qpsi)
 //   float Kp[] = {25.0, 100.0, 100.0, 0.0};
 //    
 //   float Ki[] = {0.6, 150.0, 150.0, 0.0};
 //       
	//float Kd[] = {0.0, -350.0, -350.0, 0.0};
    
	float Kp[] = {40.0, 5.0, 5.0, 0.0};
        
    float Ki[] = {2.0, 0.1, 0.1, 0.3}/*{0.0, 0.0, 0.0, 0.0}*/;
        
    float Kd[] = {0.0, 0.0, 0.0, 0.0};

    //define an intermediate ukpo signal (pre multiplying by the matrix)
    
    //form the PID output:
    //implementation formulat: ukpo_int - uk = (Kd/delta + K + Ki*delta)ekpo - (K + 2Kd/delta)ek + Kd/delta*ekmo
        	
    for (int j=0; j<4; j++) {

        uk[j] += (Kd[j]/delta + Kp[j] + Ki[j]*delta)*ekpo[j] - (Kp[j] + 2*Kd[j]/delta)*ek[j] + (Kd[j]/delta)*ekmo[j];

    }
    
    //multiply ukpo_int by the matrix to get your final values for ukpo
    //I computed the multiplication on paper, and the solutions are as follows:
    yk[0] = .25*uk[0] -.5*uk[2] + .25*uk[3];
    yk[1] = .25*uk[0] +.5*uk[1] - .25*uk[3];
    yk[2] = .25*uk[0] +.5*uk[2] + .25*uk[3];
    yk[3] = .25*uk[0] -.5*uk[1] - .25*uk[3];
    
	//output control signals.
/*  yk[0] = constrain(yk[0], 0.0, 200.0);
    yk[1] = constrain(yk[1], 0.0, 200.0);
    yk[2] = constrain(yk[2], 0.0, 200.0);
    yk[3] = constrain(yk[3], 0.0, 200.0); */
	
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