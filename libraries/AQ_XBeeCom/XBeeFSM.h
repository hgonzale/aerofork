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
int inValueSign;
unsigned long lastHeartbeatTime;

int dataDisplayMode = 0;

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

	for (int i = 0; i < 4; i++) {
		Serial.print(yk[i]);
		Serial.print(", ");
	}
	
	Serial.print(" | \t ");
}

void printQs() {
	// Serial.print(qkmin[0]);
	// Serial.print(", ");
	// Serial.print(qkmin[1]);
	// Serial.print(", ");
	// Serial.print(qkmin[2]);
	// Serial.print(", ");
	// Serial.print(qkmin[3]);
	// Serial.print(" | \t ");
}

void printMotorCommands() {
	Serial.print((int) motorCommand[MOTOR1]);
	Serial.print(", ");
	Serial.print((int) motorCommand[MOTOR2]);
	Serial.print(", ");
	Serial.print((int) motorCommand[MOTOR3]);
	Serial.print(", ");
	Serial.print((int) motorCommand[MOTOR4]);
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
			// motorCommand[MOTOR1] = 1200;
			// delay(1000);
			// motorCommand[MOTOR1] = 1000;
			// motorCommand[MOTOR2] = 1200;
			// delay(1000);
			// motorCommand[MOTOR2] = 1000;
			// motorCommand[MOTOR3] = 1200;
			// delay(1000);
			// motorCommand[MOTOR3] = 1000;
			// motorCommand[MOTOR4] = 1200;
			// delay(1000);
			// motorCommand[MOTOR4] = 1000;
			if (dataDisplayMode == 0) {
				dataDisplayMode = 1;
			} else {
				dataDisplayMode = 0;
			}
			nextCommand = 'x';
			break;

		case 'p': // set altitude reference --> input p+X.XX where X.XX is the altitude value
			alt_ref = inValue;
			nextCommand = 'x';
			break;

		case 'q': // read PID info 2
			Serial.print(alt_ref);
			Serial.print(" \t ");
			Serial.println(yaw_ref);

			// Serial.print(accelScaleFactor[XAXIS]);
			// Serial.print(", \t");
			// Serial.print(runTimeAccelBias[XAXIS]);
			// Serial.print(", \t");
			// Serial.print(accelScaleFactor[YAXIS]);
			// Serial.print(", \t");
			// Serial.print(runTimeAccelBias[YAXIS]);
			// Serial.print(", \t");
			// Serial.print(accelScaleFactor[ZAXIS]);
			// Serial.print(", \t");
			// Serial.print(runTimeAccelBias[ZAXIS]);
			// Serial.print(", \t");
			// Serial.println("");	
    		nextCommand = 'x';
			break;

		case 'y': // set yaw reference
			yaw_ref = inValue;
			nextCommand = 'x';
			break;

		case 'x': // do nothing
			break;

		case '~': // trigger emergency stop
			emergencyStop();
			break;

		case '?': // request quadrotor state data
			if (dataDisplayMode == 0) {
				printState();
				printMotorCommands();
			} else {
				printPID();
				printMotorCommands();
			}

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
				 thisByte == 'x' || thisByte == '~' || thisByte == '?' ) { // (almost) all command cases go here

				nextState = ExpectHeartbeat;
				nextCommand = thisByte;

			} else if ( thisByte == 'p' || thisByte == 'y' ) { // command cases where we expect a value afterward

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