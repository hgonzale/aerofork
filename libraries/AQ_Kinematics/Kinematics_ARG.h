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

float K_p = 0.0;                   					// proportional gain governs rate of convergence to accelerometer/magnetometer
float K_i = 0.0;                   					// integral gain governs rate of convergence of gyroscope biases
float halfT = 0.0;                					// half the sample period
float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;       // quaternion elements representing the estimated orientation
float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;  		// scaled integral error
  
float previousEx = 0.0;
float previousEy = 0.0;
float previousEz = 0.0;


////////////////////////////////////////////////////////////////////////////////
// Defining variables for KF
////////////////////////////////////////////////////////////////////////////////

//// 4x4 identity matrix -- Acts as Ak matrix for the KF
float idMat[16] = {1,0,0,0,
   0,1,0,0,
   0,0,1,0,
   0,0,0,1};

// 4x4 sensor noise covariance matrix
float Rk[16] = {0.1,0,0,0,
   0,0.1,0,0,
   0,0,0.1,0,
   0,0,0,0.1};

//  4x4 error covariance matrix (arbitrarily large diagonal)
float Qkmin[16] = {1e6,0,0,0,
   0,1e6,0,0,
   0,0,1e6,0,
   0,0,0,1e6};

// 4x1 a priori estimate of quaternion.
float qkmin[4] = {q0,q1,q2,q3};

// 4x1 vector of quaternions
float q[4] = {1.0,0.0,0.0,0.0};

// 4x1, arrange euler to measurement matrix
float zk[4] = {0};

// Uninitialized 4x4 roll rate matrix
float Ok[16] = {0};

// Alternate roll rate matrix? (as per Jonathan's suggestion)
float M[16] = {0};

// 4x4 error covariance correction
float Qk[16] = {0};

// 4x4 transition matrix, transpose
float Tk[16] = {0};
float Tk_trans[16] = {0};

// 4x4 Kalman gain
float Kk[16] = {0};

// 4x1 estimate correction
float qk[4] = {0};

// 4x4 holding matrices for multi-step KF calculations
float A[16],B[16],C[16],D[16],E[16];

// 4x1 holding vectors for multi-step KF calculations
float a[4],b[4],c[4],d[4],dd[4],ddd[4];

//model noise, currently unused
float Wk[16] = {0.1,0,0,0,
   0,0.1,0,0,
   0,0,0.1,0,
   0,0,0,0.1};

  float mx,my,mz;

////////////////////////////////////////////////////////////////////////////////
// argUpdate
////////////////////////////////////////////////////////////////////////////////
void argUpdate(float gx, float gy, float gz, float ax, float ay, float az, float G_Dt) {
  float norm,vnorm;
  float vx, vy, vz;

  float ox, oy, oz;

  // float q0i, q1i, q2i, q3i;
  float ex, ey, ez;
    
  halfT = G_Dt/2;
  
  // normalise the measurements
  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;

  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1*q3 - q0*q2);
  vy = 2*(q0*q1 - q2*q3);
  vz = q0*q0 - q1*q1 - q2*q2 - q3*q3;
    
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (vy*az - vz*ay);
  ey = (vz*ax - vx*az);
  ez = (vx*ay - vy*ax);
    
  // integral error scaled integral gain
  exInt = exInt + ex*K_i;
  if (isSwitched(previousEx,ex)) {
    exInt = 0.0;
  }
  previousEx = ex;
	
  eyInt = eyInt + ey*K_i;
  if (isSwitched(previousEy,ey)) {
    eyInt = 0.0;
  }
  previousEy = ey;

  ezInt = ezInt + ez*K_i;
  if (isSwitched(previousEz,ez)) {
    ezInt = 0.0;
  }
  previousEz = ez;


  // adjusted gyroscope measurements
  mx = gx;
  my = gy;
  mz = gz;

  ox = gx + K_p*ex + exInt;
  oy = gy + K_p*ey + eyInt;
  oz = gz + K_p*ez + ezInt;


  // initial roll rate matrix (w/out adjusted gyro measurements)
    M[1] = -mx * halfT;
    M[2] = -my * halfT;
    M[3] = -mz * halfT;
    M[4] = mx * halfT;
    M[6] = mz * halfT;
    M[7] = -my * halfT;
    M[8] = my * halfT;
    M[9] = -mz * halfT;
    M[11] = mx * halfT;
    M[12] = mz * halfT;
    M[13] = my * halfT;
    M[14] = -mx * halfT;

    
    // integrate quaternion rate and normalise (leftovers from original AQ code)
    // q0i = (-q1*gx - q2*gy - q3*gz) * halfT;
    // q1i = ( q0*gx + q2*gz - q3*gy) * halfT;
    // q2i = ( q0*gy - q1*gz + q3*gx) * halfT;
    // q3i = ( q0*gz + q1*gy - q2*gx) * halfT;
    // q0 += q0i;
    // q1 += q1i;
    // q2 += q2i;
    // q3 += q3i;





    // // d(k-2)
    // ddd[0] = dd[0];
    // ddd[1] = dd[1];
    // ddd[2] = dd[2];
    // ddd[3] = dd[3];

    // // d(k-1)
    // dd[0] = d[0];
    // dd[1] = d[1];
    // dd[2] = d[2];
    // dd[3] = d[3];

    // // d(k)
    // d[0] = q[0];
    // d[1] = q[1];
    // d[2] = q[2];
    // d[3] = q[3];

    //q = (idMat + halfT*M)*q
    matrixAdd(4,4,E,idMat,M);
    // matrixMultiply(4,4,1,c,E,q);
    matrixMultiply_alt(4,4,1,c,E,q);
    q[0] = c[0];
    q[1] = c[1];
    q[2] = c[2];
    q[3] = c[3];

    q0 = q[0];
    q1 = q[1];
    q2 = q[2];
    q3 = q[3];
    
    // normalise quaternion
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
    
    // q[0] = q0;
    // q[1] = q1;
    // q[2] = q2;
    // q[3] = q3;


    ///////////////////////////////////////
    // Iteration of Kalman filter update //
    ///////////////////////////////////////
    
    // measurement vector
    zk[0] = q0;
    zk[1] = q1;
    zk[2] = q2;
    zk[3] = q3;
    
    // roll rate matrix
    Ok[1] = -ox;
    Ok[2] = -oy;
    Ok[3] = -oz;
    Ok[4] = ox;
    Ok[6] = oz;
    Ok[7] = -oy;
    Ok[8] = oy;
    Ok[9] = -oz;
    Ok[11] = ox;
    Ok[12] = oz;
    Ok[13] = oy;
    Ok[14] = -ox;

    // Kalman filter computations ...

    //Tk = idMat + halfT*Ok
    matrixScale(4,4,Tk,halfT,Ok);
    matrixAdd(4,4,Tk,idMat,Tk);

    //Kk = Qkmin * (Qkmin + Rk)^-1
    matrixAdd(4,4,A,Qkmin,Rk);
    matrixInverse4x4(B,A);
    matrixMultiply(4,4,4,Kk,Qkmin,B);

    //qk = qkmin + Kk*(zk - idMat*qkmin)
    vectorSubtract(4,a,zk,qkmin);
    matrixMultiply(4,4,1,b,Kk,a);
    vectorAdd(4,qk,qkmin,b);

    //Qk = (idMat - Kk) * Qkmin
    matrixSubtract(4,4,C,idMat,Kk);
    matrixMultiply(4,4,4,Qk,C,Qkmin);

    //qkmin = Tk*qk
    matrixMultiply(4,4,1,qkmin,Tk,qk);

    //Qkmin = Tk*Qk*Tk_trans
    matrixMultiply(4,4,4,D,Tk,Qk);
    matrixTranspose4x4(Tk_trans,Tk);
    matrixMultiply(4,4,4,Qkmin,D,Tk_trans);

    //Qkmin = Qkmin + Wk
    matrixAdd(4,4,Qkmin,Qkmin,Wk);

    // ... end KF computations
   
   // normalize quaternion estimates
    vnorm = sqrt(qkmin[0]*qkmin[0] + qkmin[1]*qkmin[1] + qkmin[2]*qkmin[2] + qkmin[3]*qkmin[3]);
    qkmin[0] = qkmin[0] / vnorm;
    qkmin[1] = qkmin[1] / vnorm;
    qkmin[2] = qkmin[2] / vnorm;
    qkmin[3] = qkmin[3] / vnorm;

    // set quaternions for next iteration
    q0 = qkmin[0];
    q1 = qkmin[1];
    q2 = qkmin[2];
    q3 = qkmin[3];
 
}
  
void eulerAngles()
{
  // kinematicsAngle[XAXIS]  = atan2(2 * (q0*q1 + q2*q3), 1 - 2 *(q1*q1 + q2*q2));
  // kinematicsAngle[YAXIS] = asin(2 * (q0*q2 - q1*q3));
  // kinematicsAngle[ZAXIS]   = atan2(2 * (q0*q3 + q1*q2), 1 - 2 *(q2*q2 + q3*q3));

  kinematicsAngle[XAXIS]  = atan2(2 * (qkmin[0]*qkmin[1] + qkmin[2]*qkmin[3]), 1 - 2 *(qkmin[1]*qkmin[1] + qkmin[2]*qkmin[2]));
  kinematicsAngle[YAXIS] = asin(2 * (qkmin[0]*qkmin[2] - qkmin[1]*qkmin[3]));
  kinematicsAngle[ZAXIS]   = atan2(2 * (qkmin[0]*qkmin[3] + qkmin[1]*qkmin[2]), 1 - 2 *(qkmin[2]*qkmin[2] + qkmin[3]*qkmin[3]));
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

  K_p = 0.2; // 2.0;
  K_i = 0.0; //0.005;
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