
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

#define SERIAL_PORT Serial

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


float inValue;
signed char commands[16] = {'a','b','c','e','m','p','q','y','x','~','?',0,0,0,0,0};
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

	switch ( cmd ) {

		case 'a': // begin imu calibration    //0
            Serial.write("got a");
            status = CALIBRATE;
			calibration();
			break;

		case 'b': // begin baro calibration     //1
			Serial.println("Calibrating barometer...");
			measureGroundBaro(); // takes about 40 seconds to run
			startBaroMeasure = true;
			Serial.println("...finished");
			break;

		case 'c': // begin control   //2
			beginControl = true;
			resetEmergencyStop(); // <-- might be unnecessary
			nextCommand = 'x';
			break;

		case 'e': // echo back command   //3
			Serial.println('e');
			nextCommand = 'x';
			break;

		case 'm': // pulse all motors   //4
            if (dataDisplayMode == 0) {
                dataDisplayMode = 1;
            } else {
                dataDisplayMode = 0;
            }
			nextCommand = 'x';
			break;

		case 'p': // set altitude reference --> input pX.XX where X.XX is the altitude value  //5
			alt_ref = inValue;
			nextCommand = 'x';
			break;

		case 'q': // read PID info 2   //6
//			Serial.print(alt_ref);
//			Serial.print(" \t ");
//			Serial.println(yaw_ref);
			break;

		case 'y': // set yaw reference   //7
//			yaw_ref = inValue;
			nextCommand = 'x';
			break;

		case 'x': // do nothing   //8
			break;

		case '~': // trigger emergency stop   //9
//			emergencyStop();
			break;

		case '?': // request quadrotor state data   //10
//			printState();
//			Serial.print(" | \t ");
//			printPID();
//			Serial.println("");
			break;

		default:
			break;

	}
}


/* Checks if the byte begins with designated header value ("111......")*/
bool CheckHeader(unsigned char Packet){
    int Header = 224;
    unsigned char PacketHeader = (unsigned char)(Packet & Header);  //first three bits in a byte
    return ( PacketHeader == Header );
}

/* Compute the last bit(parity bit), of the incoming byte.
 If first 7 bits of incoming byte has odd number of 1's, return 1;
 else the parity bit is zero.*/
int computeParity(unsigned char a){
    int NumberOfOne = 0;
    int Allzero = 0;
    unsigned char b = a;
    for (int i= 0; i<7;i++){
        b = (unsigned char)(Allzero|b>>1);
        if(b%2!=0) NumberOfOne+=1;
    }
    
    if(NumberOfOne%2 != 0)   return 1;
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
            
			if ( CheckHeader(thisByte) ) { // got header, wait for command
                
                int CommandNumber = parsePacket(thisByte);     //Command number ranges from 0 to 15
                nextCommand = commands[CommandNumber];
                #ifdef DEBUG
                // Serial.println("the command# is: ");
                // Serial.println(nextCommand);
                #endif
          
                if (nextCommand == 'a' || nextCommand == 'b' ||
                    nextCommand == 'c' || nextCommand == 'e' ||
                    nextCommand == 'm' || nextCommand == 'q' ||
                    nextCommand == 'x' || nextCommand == '~' || nextCommand == '?' ) { // (almost) all command cases go here
                    
                    processCommand(nextCommand);
                    nextState = ExpectLeadByte;
                    
                } else if ( nextCommand == 'p' || nextCommand == 'y' ) { // command cases where we expect a value afterward
                    
                    nextState = ExpectDataByte;
                    
                } else { // command not recognized
                    #ifdef DEBUG
                        //Serial.println("Error Command, please try again!");
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