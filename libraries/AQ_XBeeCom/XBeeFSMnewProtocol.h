
#ifndef _XBEE_FSM_H_
#define _XBEE_FSM_H_


/***************************************************************************
* 	XBeeFSM.h
*
* 	OVERVIEW:
*	Implementation of a finite state machine (FSM) for handling incoming
*	Serial messages. See the accompanying documentation for a complete 
*   description of the protocol.
*
*	NOTE:
*	When adding new commands, make sure to add case statements to both 
*	processCommand() and XBeeFSM().
*
****************************************************************************/


// Define constants 
//#define DEBUG

// #define SERIAL_PORT Serial

const int HEADER = 224;
const int ZERO = 0;
const unsigned long EMGSTOP_MAX_MILLIS = 3000; // emergency stop timer (milliseconds)

/* Define states of FSM */
typedef enum { 
    ExpectLeadByte,
    ExpectDataByte
} state_t;

state_t currentState;
state_t nextState;

signed char nextCommand;
unsigned char rxByte;
unsigned long lastHeartbeatTime;
int dataDisplayMode = 0;
float inValue;
signed char commands[16] = {'a','b','c','d','e','i','m','p','q','r','x','y','z','~','?',0};
///last five command number reserved for future assignment

// used for switching between float and byte[] representation of input values
union customFloat { // total size is 8 bytes
    byte asBytes[4];
    float asFloat;
} output;


/*
* initXBeeFSM
*
* Initialization function for the XBee FSM.
*/
void initXBeeFSM() {

	currentState = ExpectLeadByte;

	nextCommand = 'x';
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
    Serial.print(altitude);

    Serial.print(" | \t ");
    
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
		//Serial.print("Executing command : ");
		//Serial.println(cmd);
	#endif

    resetEmergencyStop();

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
			nextCommand = 'x';
			break;

        case 'd': // set D

            // PIDVARS_altitude[2] = inValue;
            // updatePIDParams(0);

            PIDVARS_roll[2] = inValue;
            updatePIDParams(1);

            PIDVARS_pitch[2] = inValue;
            updatePIDParams(2);

            nextCommand = 'x';
            break;

		case 'e': // echo back command
			Serial.println('e');
			nextCommand = 'x';
			break;

        case 'i': // set I

            // PIDVARS_altitude[1] = inValue;
            // updatePIDParams(0);

            PIDVARS_roll[1] = inValue;
            updatePIDParams(1);

            PIDVARS_pitch[1] = inValue;
            updatePIDParams(2);
            
            nextCommand = 'x';
            break;

		case 'm': // pulse all motors
            if (dataDisplayMode == 0) {
                dataDisplayMode = 1;
            } else {
                dataDisplayMode = 0;
            }
			nextCommand = 'x';
			break;

		case 'p': // set P

            // PIDVARS_altitude[0] = inValue;
            // updatePIDParams(0);

            PIDVARS_roll[0] = inValue;
            updatePIDParams(1);

			PIDVARS_pitch[0] = inValue;
            updatePIDParams(2);
            
			nextCommand = 'x';
			break;

        case 'q': //
            nextCommand = 'x';
            break;

        case 'r': // read PID values
            Serial.print("ALTITUDE (w/ ref): ");
            Serial.print(alt_ref);
            Serial.print(", \t ");
            Serial.print(PIDVARS_altitude[0]); //P
            Serial.print(", \t ");
            Serial.print(PIDVARS_altitude[1]); //I
            Serial.print(", \t ");
            Serial.println(PIDVARS_altitude[2]); //D

            Serial.print("ROLL: ");
            Serial.print(PIDVARS_roll[0]); //P
            Serial.print(", \t ");
            Serial.print(PIDVARS_roll[1]); //I
            Serial.print(", \t ");
            Serial.println(PIDVARS_roll[2]); //D

            Serial.print("PITCH: ");
            Serial.print(PIDVARS_pitch[0]); //P
            Serial.print(", \t ");
            Serial.print(PIDVARS_pitch[1]); //I
            Serial.print(", \t ");
            Serial.println(PIDVARS_pitch[2]); //D

            Serial.print("YAW (w/ ref): ");
            Serial.print(yaw_ref);
            Serial.print(", \t ");
            Serial.print(PIDVARS_yaw[0]); //P
            Serial.print(", \t ");
            Serial.print(PIDVARS_yaw[1]); //I
            Serial.print(", \t ");
            Serial.println(PIDVARS_yaw[2]); //D

            nextCommand = 'x';
            break;

		case 'y': // set yaw reference
			yaw_ref = inValue;
			nextCommand = 'x';
			break;

		case 'x': // do nothing
			break;

        case 'z': // set alt ref
            alt_ref = inValue;
            nextCommand = 'x';
            break;

		case '~': // trigger emergency stop
			emergencyStop();
			break;

		case '?': // request quadrotor state data
            if (dataDisplayMode == 1) {
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


/* Checks if the byte begins with designated header value ("111......")*/
bool CheckHeader(unsigned char Packet){
    unsigned char PacketHeader = (unsigned char)(Packet & HEADER);  //first three bits in a byte
    return ( PacketHeader == HEADER );
}

/* Compute the last bit(parity bit), of the incoming byte.
 If first 7 bits of incoming byte has odd number of 1's, return 1;
 else the parity bit is zero.*/
int computeParity(unsigned char a){
    int NumberOfOne = 0;
    unsigned char b = a;
    for (int i= 0; i<7;i++){
        b = (unsigned char)(ZERO|b>>1);
        if (b%2!=0) NumberOfOne++;
    }
    
    if ( NumberOfOne %2 != 0 )   return 1;
    else  return 0;
}


int extractParity(unsigned char a){
    if(a%2==0) return 0;
    else return 1;
}

/*Get the command number contained in the byte. This number ranges from 0 to 15*/
int getCommand(unsigned char a){
    int LastBitCanceler = 254;
    unsigned char temp = (unsigned char)(a&LastBitCanceler);
    unsigned char cmd = (unsigned char)(temp<<3);
    int command = cmd/16;
    return command;
}

int parsePacket(unsigned char Packet){
    if(CheckHeader(Packet)){
        int calculatedParity = computeParity(Packet);
        if(calculatedParity == extractParity(Packet)){
            return getCommand(Packet);
             }
      }
    return -1;
}


/*
* XBeeComFSM
*
* thisByte : a signed char representing the most recent byte
*            read from the Serial stream.
*
* An implementation of a finite state machine that processes 
* incoming Serial data.
*/
void XBeeComFSM( unsigned char thisByte ) {

	switch( currentState ) {

		case ExpectLeadByte:
            
			if ( CheckHeader(thisByte) ) { // got header
                
                int CommandNumber = parsePacket(thisByte);     //Command number ranges from 0 to 15
                nextCommand = commands[CommandNumber];
                #ifdef DEBUG
                // Serial.println("the command# is: ");
                // Serial.println(nextCommand);
                #endif
          
                if (nextCommand == 'a' || nextCommand == 'b' ||
                    nextCommand == 'c' || nextCommand == 'e' ||
                    nextCommand == 'm' || nextCommand == 'q' ||
                    nextCommand == 'x' || nextCommand == '~' || 
                    nextCommand == 'r' || nextCommand == '?' ) { // (almost) all command cases go here
                    
                    processCommand(nextCommand);
                    nextState = ExpectLeadByte;
                    
                } else if ( nextCommand == 'p' || nextCommand == 'i' ||
                            nextCommand == 'd' || nextCommand == 'y' || nextCommand == 'z' ) { // command cases where we expect a value afterward
                    
                    nextState = ExpectDataByte;
                    
                } else { // command not recognized
                    #ifdef DEBUG
                        Serial.print("Invalid command: ");
                        Serial.println(nextCommand);
                    #endif
                    nextState = ExpectLeadByte;
                    nextCommand = 'x';
                }

            }
            else { // no header, keep waiting for it

				nextState = ExpectLeadByte;
			}

			break;


		case ExpectDataByte:
            
            if (Serial.available() > 2) {
                
                output.asBytes[0] = thisByte;
                output.asBytes[1] = Serial.read();
                output.asBytes[2] = Serial.read();
                output.asBytes[3] = Serial.read();

                inValue = output.asFloat;

                #ifdef DEBUG
                    Serial.print("the inValue is: ");
                    Serial.println(inValue);
                #endif

            }
            
			processCommand(nextCommand);
            inValue = 0;
            
            nextState = ExpectLeadByte;

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
        
		rxByte = (unsigned char) Serial.read();

		#ifdef DEBUG
			Serial.print("Received : ");
			Serial.println(rxByte);
		#endif

		XBeeComFSM(rxByte);

	}

}

#endif