

#ifndef _AEROQUAD_XBEE_COM_H_
#define _AEROQUAD_XBEE_COM_H_


// #define String WString::String
#include <XBee.h>
#include <string.h>

#define SIZEOF_ARRAY(x) (sizeof (x) / sizeof (x[0]))
// XBee and ZigBee communication setup
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();

uint8_t* payload;
int payloadLength = -1;

bool newDataReady = false;
bool doit = false;

char* xbeeTxMessage;
char charcat[1];

byte* float_buf;

const uint8_t INT_FLAG = 0xB0;
const uint8_t FLOAT_FLAG = 0xB1;

// union floatAsBytes_t {
// 	float n;
	
// } val_f;


union messageData {
	char query;
	float heartbeat;
} messageRx; // 5 bytes


/* XBee print functions */

void xbee_println() {

	Serial.write('\n');
}

void xbee_printStr(String str) {
	int len = str.length();
	char buf[len];
	str.toCharArray(buf,len);

	for (byte i = 0; i < len; i++) {
		Serial.write(buf[i]);
	}

	// strcat(xbeeTxMessage,buf);
}

void xbee_printInt(int n) {

	Serial.write(INT_FLAG);
	Serial.write(highByte(n));
	Serial.write(lowByte(n));

	// char cArr[2] = {highByte(n), lowByte(n)};
	// strcat(xbeeTxMessage,cArr);
}

void xbee_printFloat(float f) {

	Serial.write(FLOAT_FLAG);
	float_buf = (byte*) &f;

	Serial.write(float_buf[3]);
	Serial.write(float_buf[2]);
	Serial.write(float_buf[1]);
	Serial.write(float_buf[0]);

	// char fArr[4] = {float_buf[3], float_buf[2], float_buf[1], float_buf[0]};
	// strcat(xbeeTxMessage,fArr);
}

void xbee_printChar(char c) {

	Serial.write(c);
}


/*****************************************************************************************
* processXbeeCommand
*
* Do something based on the query recieved by the Xbee. 
******************************************************************************************/
void processXbeeCommand() {

	switch (messageRx.query) {

		case '^':// emergency stop
			emergencyStop();
			break;

		case 'X': // stop sending messages
			break;

		case 'x': //stop sending messages
			break;

		case '[': // begin execution of PID control-loop
			beginControl = true;
			calibrateESC = 5;
			countStop = 0;
    		status = FLIGHT;
    		break;	

    	case '@':
    		status = CALIBRATE;
    		calibration();
    		break;

    	case '*': // when all initial calibration procedures are complete for accelerometer, gyroscope and barometer.
			startCalibrate = true;
			break;

		case '/': // calibrate barometer
			xbee_printStr("calibrating barometer");
			measureGroundBaro();
			startBaroMeasure = true;
			xbee_printStr("finished");
			break;

		case '?': //custom command
			xbee_printChar('e');
			// xbee_printFloat(kinematicsAngle[XAXIS]);
			// xbee_printFloat(kinematicsAngle[YAXIS]);
			// xbee_printFloat(kinematicsAngle[ZAXIS]);
			// xbee_println();
			messageRx.query = 'x';
			break;

		case 'q':
			xbee_printStr("a string");
			messageRx.query = 'x';
			break;

		default:
			break;

	}

}

/*****************************************************************************************
* checkHeartBeat
*
******************************************************************************************/
bool checkHeartBeat() {
	return (messageRx.heartbeat == 78.9);
}


/*****************************************************************************************
* parseData
*
******************************************************************************************/
void parseData() {

	messageRx.query = payload[2];
	
	Serial.write(messageRx.query);

	for (int i = 0; i < payloadLength; i++) {

		Serial.write(payload[i]);
	}
	

	if (payloadLength > 6) {

		if (payload[3] == 7 && payload[4] == 8 && payload[6] == 9) messageRx.heartbeat = 78.9;
	}

	newDataReady = false;
	doit = true;
}



/*****************************************************************************************
* XbeeTx
*
******************************************************************************************/
void XbeeTx() {

	if (newDataReady) {

		for (byte i = 0; i < SIZEOF_ARRAY(xbeeTxMessage); i++) {

			Serial.write(xbeeTxMessage[i]);

		}

		charcat[0] = (char) 0x00;
		xbeeTxMessage = charcat;
	
		newDataReady = false;
		
	}
}


/*******************************************************************************************
* XbeeRx
*
* Handles all incoming communication for the XBee.
* NOTE: Assumes all incoming messages follow ZigBee RX_RESPONSE protocol.
********************************************************************************************/
void XbeeRx() {

	xbee.readPacket();

	if (xbee.getResponse().isAvailable()) {
		// got something

		if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
			// got a zb rx packet

			// now fill our zb rx class
			xbee.getResponse().getZBRxResponse(rx);

			//get the data
			payload = rx.getData();
			payloadLength = rx.getDataLength();

			newDataReady = true;
		}
	}

	if (newDataReady) parseData();

	processXbeeCommand();

	resetEmergencyStop = checkHeartBeat();
	messageRx.heartbeat = 0.0;
}



/*****************************************************************************************
* XbeeEcho
*
* A Useful function for debugging XBee communication. Reads in ZigBee RX_RESPONSE message
* and writes the payload back to the sender.
******************************************************************************************/
void XbeeEcho() {

	xbee.readPacket();

	if (xbee.getResponse().isAvailable()) {
		// got something

		if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
			// got a zb rx packet

			// now fill our zb rx class
			xbee.getResponse().getZBRxResponse(rx);

			//get the data
			payload = rx.getData();
			payloadLength = rx.getDataLength();

			newDataReady = true;
		}
	}

	// echo the payload back to sender
	if (newDataReady) {

		for (int i = 0; i < payloadLength; i++) {

			Serial.write(payload[i]);
		}

		newDataReady = false;

	}

}



#endif