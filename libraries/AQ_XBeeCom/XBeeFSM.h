
#ifndef _XBEE_FSM_H_
#define _XBEE_FSM_H_


/* Constants definitions */
#define HEADER_BYTE 0x7E
#define HEARTBEAT 0x24
#define EMPTY_SERIAL -1 // reading an empty Serial port returns -1


/* State definitions */
typedef enum { 
	ExpectHeader, 
	ExpectCommand, 
	ExpectHeartbeat 
} state_t;

state_t currentState = ExpectHeader;
state_t nextState;
signed char nextCommand = 'x';
signed char rxByte;
const int packetLength = 3;




void XBeeRead() {

	for (int i = 0; i < packetLength; i++) {

		rxByte = (signed char) Serial.read();

		XBeeComFSM(rxByte);

	}

	processCommand(nextCommand);

}

void processCommand(signed char cmd) {}

void resetEmergencyStop() {}

bool checkEmergencyStop() {
	
	return (millis() - lastPulse < EMGSTOP_MAX_MILLIS);
}

/* Do state transition logic */
void XBeeComFSM( unsigned char thisByte ) {

	switch(currentState) {

		case ExpectHeader:

			switch(thisByte) {

				case HEADER_BYTE:
					nextState = ExpectCommand;
					break;

				default:
					nextState = ExpectHeader;
					break;
			}

			break;


		case ExpectCommand:

			switch(thisByte) {

				case EMPTY_SERIAL:
					nextState = ExpectHeader;
					break;

				// all command cases go here
				// for example...

				case 'a':
					nextState = ExpectHeartbeat;
					nextCommand = 'a';
					break;

				default: // unknown command
					nextState = ExpectHeader;
					break;
			}

			break;


		case ExpectHeartbeat:

			switch(thisByte) {

				case HEARTBEAT:
					nextState = ExpectHeader;
					//acknowledge HB
					resetEmergencyStop();
					break;

				default:
					nextState = ExpectHeader;
					// no HB
					break;
			}

			break;

	}

	currentState = nextState;
	return;

}


