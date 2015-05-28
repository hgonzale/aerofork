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

#ifndef _AQ_KINEMATICS_ARG_
#define _AQ_KINEMATICS_ARG_

//=====================================================================================================
// IMU.c
// S.O.H. Madgwick
// 25th September 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'IMUupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz')
// and accelerometer ('ax', 'ay', 'ay') data.  Gyroscope units are radians/second, accelerometer 
// units are irrelevant as the vector is normalised.
//
//=====================================================================================================


////////////////////////////////////////////////////////////////////////////////
// ARG - Accelerometer, Rate Gyro
////////////////////////////////////////////////////////////////////////////////


#include "Kinematics.h"

#include <AQMath.h>

float Kp = 0.0;                   					// proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki = 0.0;                   					// integral gain governs rate of convergence of gyroscope biases
float halfT = 0.0;                					// half the sample period
float q0 = 0.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;       // quaternion elements representing the estimated orientation
float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;  		// scaled integral error
  
float previousEx = 0.0;
float previousEy = 0.0;
float previousEz = 0.0;




////////////////////////////////////////////////////////////////////////////////
// Defining variables for KF
////////////////////////////////////////////////////////////////////////////////

// 4x4 identity matrix -- Acts as Ak matrix for the KF
const int[16] idMat = {1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1};

// 4x4 sensor noise covariance matrix
const float[16] Rk = {0.1,0,0,0,
    0,0.1,0,0,
    0,0,0.1,0,
    0,0,0,0.1};

//  4x4 error covariance matrix (arbitrarily large diagonal)
float[16] Qkmin = {100000000000,0,0,0,
    0,10000000000,0,0,
    0,0,10000000000,0,
    0,0,0,10000000000};

// 4x1 a priori estimate of quaternion.
float [4] qkmin = {q0,q1,q2,q3};

// 4x1, arrange euler to measurement matrix
float[4] zk;

// Uninitialized 4x4 roll rate matrix
float [16] Ok;

// 4x4 transition matrix, transpose
float[16] Tk;
float[16] tk_trans;

// 4x4 Kalman gain
float[16] Kk;

// 4x1 estimate correction
float[4] qk;

// 4x4 holding matrices for multi-step KF calculations
float[16] A,B,C,D;

// 4x1 holding vectors for multi-step KF calculations
float[4] a,b;

// model noise, currently unused
//    int Wk = 0;



////////////////////////////////////////////////////////////////////////////////
// argUpdate
////////////////////////////////////////////////////////////////////////////////
void argUpdate(float gx, float gy, float gz, float ax, float ay, float az, float G_Dt) {
  
  float norm,vnorm;
  float vx, vy, vz;
  float q0i, q1i, q2i, q3i;
  float ex, ey, ez;
    
  halfT = G_Dt/2;
  
  // normalise the measurements
  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;
     	
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1*q3 - q0*q2);
  vy = 2*(q0*q1 + q2*q3);
  vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (vy*az - vz*ay);
  ey = (vz*ax - vx*az);
  ez = (vx*ay - vy*ax);
    
  // integral error scaled integral gain
  exInt = exInt + ex*Ki;
  if (isSwitched(previousEx,ex)) {
    exInt = 0.0;
  }
  previousEx = ex;
	
  eyInt = eyInt + ey*Ki;
  if (isSwitched(previousEy,ey)) {
    eyInt = 0.0;
  }
  previousEy = ey;

  ezInt = ezInt + ez*Ki;
  if (isSwitched(previousEz,ez)) {
    ezInt = 0.0;
  }
  previousEz = ez;

  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;
    
    // integrate quaternion rate and normalise
    q0i = (-q1*gx - q2*gy - q3*gz) * halfT;
    q1i = ( q0*gx + q2*gz - q3*gy) * halfT;
    q2i = ( q0*gy - q1*gz + q3*gx) * halfT;
    q3i = ( q0*gz + q1*gy - q2*gx) * halfT;
    q0 += q0i;
    q1 += q1i;
    q2 += q2i;
    q3 += q3i;
    
    // normalise quaternion
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
    
    // measurement vector
    zk = {q0,q1,q2,q3};
    
    // roll rate matrix
    Ok = {0,-gx,-gy,-gz,
        gx,0,gz,-gy,
        gy,-gz,0,gx,
        gz,gy,-gx,0};
    
    ///////////////////////////////////////
    /* Iteration of Kalman filter update */
    ///////////////////////////////////////
    
    matrixScale(4,4,Tk,halfT,Ok); // do Tk = halfT * Ok
    matrixAdd(4,4,4,Tk,idMat,Tk); // do Tk = I + TK
    
    
    matrixAdd(4,4,A,Qkmin,Rk); // do A = Qkmin + Rk
    matrixInverse4x4(B,A); // do B = inv(A)
    matrixMultiply(4,4,4,Kk,Qkmin,B); // do Kk = Qkmin * B
    
    
    vectorSubtract(4,a,zk,qkmin); // do a = zk - qkmin
    matrixMultiply(4,4,1,b,Kk,a); // do b = Kk * a
    vectorAdd(4,qk,qkmin,b); // do qk = qkmin + b
    
    
    matrixSubtract(4,4,C,idMat,Kk); // do C = I - Kk
    matrixMultiply(4,4,4,Qk,C,Qkmin); // do Qk = C * Qkmin
    
    
    matrixMultiply(4,4,1,qkmin,Tk,qk); // do qkmin = Tk * qk
    
    
    matrixMultiply(4,4,4,D,Tk,Qk); // do D = Tk * Qk
    matrixTranspose4x4(Tk_trans,Tk); // do Tk_trans = transpose(Tk)
    matrixMultiply(4,4,4,Qkmin,D,Tk_trans); // to Qkmin = D * Tk_trans
    // NOTE: to incorporate model noise, add Wk to the Qkmin term here
    
   
    vnorm = sqrt(qkmin[0]*qkmin[0] + qkmin[1]*qkmin[1] + qkmin[2]*qkmin[2] + qkmin[3]*qkmin[3]);
    qkmin[0] = qkmin[0] / vnorm;
    qkmin[1] = qkmin[1] / vnorm;
    qkmin[2] = qkmin[2] / vnorm;
    qkmin[3] = qkmin[3] / vnorm;
    // end of Kalman Filter section
 

}
  
void eulerAngles()
{
  kinematicsAngle[XAXIS]  = atan2(2 * (q0*q1 + q2*q3), 1 - 2 *(q1*q1 + q2*q2));
  kinematicsAngle[YAXIS] = asin(2 * (q0*q2 - q1*q3));
  kinematicsAngle[ZAXIS]   = atan2(2 * (q0*q3 + q1*q2), 1 - 2 *(q2*q2 + q3*q3));
}

////////////////////////////////////////////////////////////////////////////////
// Initialize ARG
////////////////////////////////////////////////////////////////////////////////

void initializeKinematics() 
{
  initializeBaseKinematicsParam();
  q0 = 1.0;
  q1 = 0.0;
  q2 = 0.0;
  q3 = 0.0;
  exInt = 0.0;
  eyInt = 0.0;
  ezInt = 0.0;
	
  previousEx = 0;
  previousEy = 0;
  previousEz = 0;

  Kp = 0.2; // 2.0;
  Ki = 0.0005; //0.005;
}
  
////////////////////////////////////////////////////////////////////////////////
// Calculate ARG
////////////////////////////////////////////////////////////////////////////////
void calculateKinematics(float rollRate,          float pitchRate,    float yawRate,  
                         float longitudinalAccel, float lateralAccel, float verticalAccel, 
                         float G_DT) {
    
  argUpdate(rollRate,          pitchRate,    yawRate, 
            longitudinalAccel, lateralAccel, verticalAccel,  
		    G_Dt);
  eulerAngles();
}
  
float getGyroUnbias(byte axis) {
  return correctedRateVector[axis];
}
  
void calibrateKinematics() {}


#endif