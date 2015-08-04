
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

const unsigned long EMGSTOP_MAX_MILLIS = 5000; // emergency stop timer (milliseconds)

/* Define states of FSM */
typedef enum { 
	ExpectHeader, 
	ExpectCommand, 
	ExpectHeartbeat 
} state_t;

state_t currentState;
state_t nextState;


signed char nextCommand;
signed char rxByte;
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


/* (for debugging) Prints the current state */
void printState() {

	if ( currentState == ExpectHeader ) {
		Serial.print("ExpectHeader");
	} else if ( currentState == ExpectCommand ) {
		Serial.print("ExpectCommand");
	} else if ( currentState == ExpectHeartbeat ) {
		Serial.print("ExpectHeartbeat");
	} else {
		Serial.print("State unknown");
	}

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
			pulseMotors(3);
			nextCommand = 'x';
			break;

		case 'p': // re-calibrate baro
			counter = 0;
			break;

		case 'q': // read PID info 2

			break;

		case 'x': // do nothing
			break;

		case '~': // trigger emergency stop
			emergencyStop();
			break;

		case '?': // request quadrotor state data
			Serial.print(getBaroAltitude());
			Serial.print(", ");
			Serial.print(kinematicsAngle[XAXIS]);
			Serial.print(", ");
			Serial.print(kinematicsAngle[YAXIS]);
			Serial.print(", ");
			Serial.println(kinematicsAngle[ZAXIS]);	
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
				 thisByte == 'm' || thisByte == 'p' ||
				 thisByte == 'q' || thisByte == 'x' || 
				 thisByte == '~' || thisByte == '?' ) { // all command cases go here

				nextState = ExpectHeartbeat;
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
			printState();
			Serial.print("\t");
		#endif

		XBeeComFSM(rxByte);

	}

}

#endif