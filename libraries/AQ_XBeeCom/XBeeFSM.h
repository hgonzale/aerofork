
#ifndef _XBEE_FSM_H_
#define _XBEE_FSM_H_


/***************************************************************************
* 	XBeeFSM.h
*
* 	OVERVIEW:
*	Implementation of a finite state machine (FSM) for handling incoming
*	Serial data. Assumes packets are sent using the 3-byte structure below.
*
*		[Header '#'] [Command <some char>] [Heartbeat '$']
*
*
*	NOTE:
*	When adding new commands, make sure to add case statements to both 
*	processCommand() and XBeeFSM().
*
****************************************************************************/


/* Define constants */
#define HEADER_BYTE 0x23
#define HEARTBEAT 0x24
// #define DEBUG

const unsigned long EMGSTOP_MAX_MILLIS = 3000; // emergency stop timer (milliseconds)

/* Define states of FSM */
typedef enum { 
	ExpectHeader, 
	ExpectCommand, 
	ExpectHeartbeat,
	ExpectValue
} state_t;

state_t currentState;
state_t nextState;


signed char nextCommand;
signed char rxByte;
float inValue;
int inValueType;
int inValueSign;
int pidDataIndex;
unsigned long lastHeartbeatTime;


/*
* initXBeeFSM
*
* Initialization function for the XBee FSM.
*/
void initXBeeFSM() {

	currentState = ExpectHeader;

	nextCommand = 'x';

	lastHeartbeatTime = 0;
}


/*
* printState
* 
* Prints [altitude], [pitch], [roll], [yaw].
*/
void printState() {

	Serial.print(altitude);
	Serial.print(", ");
	Serial.print(kinematicsAngle[XAXIS]);
	Serial.print(", ");
	Serial.print(kinematicsAngle[YAXIS]);
	Serial.print(", ");
	Serial.print(kinematicsAngle[ZAXIS]);
	Serial.print(" | \t ");
}

/*
* printPID
*
* Prints the PID outputs and the motor commands.
*/

void printPID() {

	Serial.print(u_alt);
	Serial.print(", ");
	Serial.print(u_roll);
	Serial.print(", ");
	Serial.print(u_pitch);
	Serial.print(", ");
	Serial.print(u_yaw);

	Serial.print(" | \t ");
}


void printMotorCommands() {
	Serial.print(motorCommand[MOTOR1]);
	Serial.print(", ");
	Serial.print(motorCommand[MOTOR2]);
	Serial.print(", ");
	Serial.print(motorCommand[MOTOR3]);
	Serial.print(", ");
	Serial.print(motorCommand[MOTOR4]);
	Serial.print(" | \t ");
}


/*
* resetEmergencyStop
*
* Resets the emergency stop timer to prevent triggering an 
* emergency stop.
*/
inline void resetEmergencyStop() {

	lastHeartbeatTime = millis();
}


/*
* checkEmergencyStop
*
* Returns 'true' if the emergency stop timer has not yet reached
* the EMGSTOP_MAX_MILLIS value. 
*
* Returning 'false' indicates that the emergency stop should be 
* triggered.
*/
inline bool checkEmergencyStop() {

	return (millis() - lastHeartbeatTime < EMGSTOP_MAX_MILLIS); // beware of overflow here
}


/*
* processCommand
*
* cmd : the char code for the command being executed
*
* Executes the last command read from a packet.
*/
void processCommand( signed char cmd ) {

	#ifdef DEBUG
		Serial.print("Executing command : ");
		Serial.println(cmd);
	#endif

	switch ( cmd ) {

		case 'a': // begin imu calibration
			status = CALIBRATE;
			calibration();
			break;

		case 'b': // begin baro calibration
			Serial.println("Calibrating barometer...");
			measureGroundBaro(); // takes about 40 seconds to run
			startBaroMeasure = true;
			Serial.println("...finished");
			break;

		case 'c': // begin control
			beginControl = true;
			resetEmergencyStop(); // <-- might be unnecessary
			nextCommand = 'x';
			break;

		case 'e': // echo back command
			Serial.println('e');
			nextCommand = 'x';
			break;

		case 'm': // pulse all motors
			motorCommand[MOTOR1] = 1200;
			delay(1000);
			motorCommand[MOTOR1] = 1000;
			motorCommand[MOTOR2] = 1200;
			delay(1000);
			motorCommand[MOTOR2] = 1000;
			motorCommand[MOTOR3] = 1200;
			delay(1000);
			motorCommand[MOTOR3] = 1000;
			motorCommand[MOTOR4] = 1200;
			delay(1000);
			motorCommand[MOTOR4] = 1000;
			nextCommand = 'x';
			break;

		case 'w': // select PID params to modify
			// 0 --> altitude
			// 1 --> roll
			// 2 --> pitch
			// 3 --> yaw
			// 4 --> gyrox (overridden in Kinematics_ARG)
			// 5 --> gyroy (overridden in Kinematics_ARG)
			// 6 --> gyroz (overridden in Kinematics_ARG)
			pidDataIndex = (int) inValue;
			Serial.print("pidDataIndex set to ");
			Serial.println(pidDataIndex);
			break;

		case 'p': // set P value of PID
			PID[pidDataIndex].P = inValue;
			Serial.print("P set to ");
			Serial.println(inValue);
			break;

		case 'i': // set I value of PID
			PID[pidDataIndex].I = inValue;
			Serial.print("I set to ");
			Serial.println(inValue);
			break;

		case 'd': // set D value of PID
			PID[pidDataIndex].D = inValue;
			Serial.print("D set to ");
			Serial.println(inValue);
			break;

		case 'r': // read current PID values (of selected PID)
			Serial.print("PID values for [");
			Serial.print(pidDataIndex);
			Serial.print("] | P: ");
			Serial.print(PID[pidDataIndex].P);
			Serial.print(", I: ");
			Serial.print(PID[pidDataIndex].I);
			Serial.print(", D: ");
			Serial.println(PID[pidDataIndex].D);
			break;

		case 'q': // read altitude and yaw references
			Serial.print("Altitude: ");
			Serial.print(alt_ref);
			Serial.print(" \t ");
			Serial.print("Yaw: ");
			Serial.println(yaw_ref);
			break;

		case 'y': // set yaw reference
			yaw_ref = inValue;
			nextCommand = 'x';
			break;

		case 'z': // set altitude reference --> input z+X.XX where X.XX is the altitude value
			alt_ref = inValue;
			nextCommand = 'x';
			break;

		case 'x': // do nothing
			break;

		case '~': // trigger emergency stop
			emergencyStop();
			break;

		case '?': // request quadrotor state data
			printState();
			printMotorCommands();
			printPID();
			Serial.println("");
			break;

		default:
			break;

	}
}






/*
* XBeeComFSM
*
* thisByte : a signed char representing the most recent byte
*			 read from the Serial stream.
*
* An implementation of a finite state machine that processes 
* incoming Serial data.
*/
void XBeeComFSM( signed char thisByte ) {

	switch( currentState ) {

		case ExpectHeader:

			if ( thisByte == HEADER_BYTE ) { // got header, wait for command

				nextState = ExpectCommand;

			} else { // no header, keep waiting for it

				nextState = ExpectHeader;
			}

			break;


		case ExpectCommand:

			if ( thisByte == 'a' || thisByte == 'b' || 
				 thisByte == 'c' || thisByte == 'e' ||
				 thisByte == 'm' || thisByte == 'q' || 
				 thisByte == 'x' || thisByte == '~' || 
				 thisByte == '?' || thisByte == 'r' ) { // (almost) all command cases go here

				nextState = ExpectHeartbeat;
				nextCommand = thisByte;

			} else if ( thisByte == 'y' || thisByte == 'z') { // command cases where we expect a value afterward

				inValueType = 1;
				nextState = ExpectValue;
				nextCommand = thisByte;

			} else if ( thisByte == 'w' ) {

				inValueType = 2;
				nextState = ExpectValue;
				nextCommand = thisByte;

			} else if ( thisByte == 'p' || thisByte == 'i' || thisByte == 'd' ) {

				inValueType = 3;
				nextState = ExpectValue;
				nextCommand = thisByte;

			} else { // command not recognized

				nextState = ExpectHeader;
				nextCommand = 'x';
			}

			break;


		case ExpectHeartbeat:

			if ( thisByte == HEARTBEAT ) { // got heartbeat, wait for next packet

				nextState = ExpectHeader;

				resetEmergencyStop();

				processCommand(nextCommand);

			} else { // no heartbeat, wait for next packet

				nextState = ExpectHeader;
			}

			break;


		case ExpectValue:

			nextState = ExpectHeader;

			if (inValueType == 1) { // +/- x.xx

				if (Serial.available() > 4) {

					if (Serial.read() == '-') {
						inValueSign = -1;
					} else {
						inValueSign = 1;
					}

					inValue = ((int) Serial.read()) - 48.0; // x.00
					Serial.read(); // '.'
					inValue += ((int) Serial.read() - 48.0) * 0.1; // 0.x0
					inValue += ((int) Serial.read() - 48.0) * 0.01; //0.0x

					inValue = inValueSign * inValue;

				}

			} else if (inValueType == 2) { // x

				if (Serial.available() > 0) {

					inValue = Serial.read() - 48.0;

					Serial.println(inValue);
				}

			} else if (inValueType == 3) { // xxx.xx

				if (Serial.available() > 5) {

					inValue = ((int) Serial.read() - 48.0) * 100.0; //x00.00
					inValue += ((int) Serial.read() - 48.0) * 10.0; //0x0.00
					inValue += ((int) Serial.read() - 48.0); //00x.00
					Serial.read(); // '.'
					inValue += ((int) Serial.read() - 48.0) * 0.1; //000.x0
					inValue += ((int) Serial.read() - 48.0) * 0.01; //000.0x

				}

			} else {
				nextCommand = 'x';
			}

			processCommand(nextCommand);
			inValue = 0;

			break;

	}

	currentState = nextState;
	return;
}


/*
* XBeeRead
*
* Top-level function for processing incoming Serial data. Passes 
* incoming bytes to XBeeFSM, and then executes the command read 
* from the packet.
*/
void XBeeRead() {

	#ifdef DEBUG
		Serial.println("-----------New Read------------");
	#endif

	while ( Serial.available() > 0 ) {

		rxByte = (signed char) Serial.read();

		#ifdef DEBUG
			Serial.print("Received : ");
			Serial.println(rxByte);
			Serial.print("\t");
		#endif

		XBeeComFSM(rxByte);

	}

}

#endif