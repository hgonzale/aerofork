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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

/*
       CW  0....Front....0 CCW
           ......***......    
           ......***......    
           ......***......    
      CCW  0....Back.....0  CW
*/

#include "FlightControlVariable.h"

// #ifdef OLD_MOTOR_NUMBERING  
//   #define FRONT_LEFT  MOTOR1
//   #define REAR_RIGHT  MOTOR2
//   #define FRONT_RIGHT MOTOR3
//   #define REAR_LEFT   MOTOR4
// #else
//   #define FRONT_LEFT  MOTOR1
//   #define FRONT_RIGHT MOTOR2
//   #define REAR_RIGHT  MOTOR3
//   #define REAR_LEFT   MOTOR4
// #endif

// if needed, assign motor numbers here
#define FRONT_LEFT  MOTOR4
#define REAR_RIGHT  MOTOR2
#define FRONT_RIGHT MOTOR1
#define REAR_LEFT   MOTOR3

#define LASTMOTOR   (MOTOR4+1)

int minMotorValue = 1100;
int maxMotorValue = 1200;
int motorMaxCommand[4] = {0,0,0,0};
int motorMinCommand[4] = {0,0,0,0};
int motorConfiguratorCommand[4] = {0,0,0,0};

// The maximum allowable change in controller output (since the last iteration)
const int FILTER_TOLERANCE = 1000;


void applyMotorCommand() {

  // Save old yk values
  yk_prev[0] = yk[0];
  yk_prev[1] = yk[1];
  yk_prev[2] = yk[2];
  yk_prev[3] = yk[3];

  // map the control signals to the appropriate motors
  yk[0] = .25*u_alt + .50*u_roll + .50*u_pitch + .25*u_yaw;
  yk[1] = .25*u_alt - .50*u_roll + .50*u_pitch - .25*u_yaw;
  yk[2] = .25*u_alt - .50*u_roll - .50*u_pitch + .25*u_yaw;
  yk[3] = .25*u_alt + .50*u_roll - .50*u_pitch - .25*u_yaw;

  // Limit change in controller output
  yk[0] = constrain(yk[0], yk_prev[0] - FILTER_TOLERANCE, yk_prev[0] + FILTER_TOLERANCE);
  yk[1] = constrain(yk[1], yk_prev[1] - FILTER_TOLERANCE, yk_prev[1] + FILTER_TOLERANCE);
  yk[2] = constrain(yk[2], yk_prev[2] - FILTER_TOLERANCE, yk_prev[2] + FILTER_TOLERANCE);
  yk[3] = constrain(yk[3], yk_prev[3] - FILTER_TOLERANCE, yk_prev[3] + FILTER_TOLERANCE);

  // update motor commands -- NOTE: yk values (float) are truncated to nearest int
  motorCommand[FRONT_LEFT] += yk[0];
  motorCommand[FRONT_RIGHT] += yk[1];
  motorCommand[REAR_RIGHT] += yk[2];
  motorCommand[REAR_LEFT] += yk[3];

  // constrain motor commands
  motorCommand[FRONT_LEFT] = constrain(motorCommand[FRONT_LEFT],minMotorValue, maxMotorValue);
  motorCommand[FRONT_RIGHT] = constrain(motorCommand[FRONT_RIGHT],minMotorValue, maxMotorValue);
  motorCommand[REAR_RIGHT] = constrain(motorCommand[REAR_RIGHT],minMotorValue, maxMotorValue);
  motorCommand[REAR_LEFT] = constrain(motorCommand[REAR_LEFT],minMotorValue, maxMotorValue);

}


void setMotorCommand(int cmd) {

  motorCommand[FRONT_LEFT] = cmd;
  motorCommand[FRONT_RIGHT] = cmd;
  motorCommand[REAR_RIGHT] = cmd;
  motorCommand[REAR_LEFT] = cmd;

  // constrain motor commands
  motorCommand[FRONT_LEFT] = constrain(motorCommand[FRONT_LEFT],minMotorValue, maxMotorValue);
  motorCommand[FRONT_RIGHT] = constrain(motorCommand[FRONT_RIGHT],minMotorValue, maxMotorValue);
  motorCommand[REAR_RIGHT] = constrain(motorCommand[REAR_RIGHT],minMotorValue, maxMotorValue);
  motorCommand[REAR_LEFT] = constrain(motorCommand[REAR_LEFT],minMotorValue, maxMotorValue);

}


void incrementMotorCommand(int inc) {

  motorCommand[FRONT_LEFT] += inc;
  motorCommand[FRONT_RIGHT] += inc;
  motorCommand[REAR_RIGHT] += inc;
  motorCommand[REAR_LEFT] += inc;

  // constrain motor commands
  motorCommand[FRONT_LEFT] = constrain(motorCommand[FRONT_LEFT],minMotorValue, maxMotorValue);
  motorCommand[FRONT_RIGHT] = constrain(motorCommand[FRONT_RIGHT], minMotorValue, maxMotorValue);
  motorCommand[REAR_RIGHT] = constrain(motorCommand[REAR_RIGHT],minMotorValue, maxMotorValue);
  motorCommand[REAR_LEFT] = constrain(motorCommand[REAR_LEFT], minMotorValue, maxMotorValue);

}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

