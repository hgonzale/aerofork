
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

const unsigned long EMGSTOP_MAX_MILLIS = 5000; // emergency stop timer (milliseconds)
const int PACKET_LENGTH = 3; // expected length of incoming packets


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
bool DEBUG = false;


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

	if (currentState == ExpectHeader) {
		Serial.print("ExpectHeader");
	} else if (currentState == ExpectCommand) {
		Serial.print("ExpectCommand");
	} else if (currentState == ExpectHeartbeat) {
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
void resetEmergencyStop() {

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
bool checkEmergencyStop() {

	return (millis() - lastHeartbeatTime < EMGSTOP_MAX_MILLIS);
}


/*
* processCommand
*
* cmd : the char code for the command being executed
*
* Executes the last command read from a packet.
*/
void processCommand( signed char cmd ) {

	if (DEBUG) {

		Serial.print("Executing command : ");
		Serial.println(cmd);
	}

	switch (cmd) {

		case 'a': // echo back command
			Serial.println('a');
			nextCommand = 'x';
			break;

		case 'b': // begin control
			beginControl = true;
			resetEmergencyStop(); // <-- might be unnecessary
			nextCommand = 'x';
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

		case 'x': // do nothing
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

	switch(currentState) {

		case ExpectHeader:

			switch(thisByte) {

				case HEADER_BYTE: // got header, wait for command
					nextState = ExpectCommand;
					break;

				default: // no header, keep waiting for it
					nextState = ExpectHeader;
					break;
			}

			break;


		case ExpectCommand:

			switch(thisByte) {

				// all command cases go here
				// for example...

				case 'a':
					nextState = ExpectHeartbeat;
					nextCommand = 'a';
					break;

				case 'b':
					nextState = ExpectHeartbeat;
					nextCommand = 'b';
					break;

				case '?':
					nextState =ExpectHeartbeat;
					nextCommand = '?';
					break;

				case 'x':
					nextState = ExpectHeartbeat;
					nextCommand = 'x';
					break;

				default: // command not recognized
					nextState = ExpectHeader;
					nextCommand = 'x';
					break;
			}

			break;


		case ExpectHeartbeat:

			switch(thisByte) {

				case HEARTBEAT: // got heartbeat, wait for next packet
					nextState = ExpectHeader;
					resetEmergencyStop();

					if (DEBUG) Serial.println("Heartbeat received");

					break;

				default: // no heartbeat, wait for next packet
					nextState = ExpectHeader;
					break;
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

	if (DEBUG) Serial.println("-----------New Read------------");

	for (int i = 0; i < PACKET_LENGTH; i++) {

		rxByte = (signed char) Serial.read();

		if (DEBUG) {

			Serial.print("Received : ");
			Serial.println(rxByte);
			printState();
			Serial.print("\t");
			Serial.println(i);
		}

		XBeeComFSM(rxByte);

	}

	processCommand(nextCommand);

}

#endif