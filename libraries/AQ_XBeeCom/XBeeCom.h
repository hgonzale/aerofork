

#ifndef _AEROQUAD_XBEE_COM_H_
#define _AEROQUAD_XBEE_COM_H_

#include <XBee.h>

// XBee and ZigBee communication setup
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();

uint8_t* payload;
int payloadLength = -1;

bool newDataReady = false;
bool continuousCommand = false;

union messageData {
	char query;
	float heartbeat;
} messageRx; // 5 bytes


/*****************************************************************************************
* processXbeeCommand
*
* Do something based on the query recieved by the Xbee. 
******************************************************************************************/
void processXbeeCommand() {

	switch (messageRx.query) {

		case '~':// emergency stop
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
			Serial.println("calibrating barometer");
			measureGroundBaro();
			startBaroMeasure = true;
			Serial.println("finished");
			break;

		case '?': //custom command
			
			Serial.print(kinematicsAngle[XAXIS]);
			Serial.print(" \t");
			Serial.print(kinematicsAngle[YAXIS]);
			Serial.print(" \t");
			Serial.println(kinematicsAngle[ZAXIS]);
			continuousCommand = true;
			break;

		case 'a': //see motor commands
			Serial.print(motorCommand[FRONT_LEFT]);
			Serial.print(", ");
			Serial.print(motorCommand[FRONT_RIGHT]);
			Serial.print(", ");
			Serial.print(motorCommand[REAR_LEFT]);
			Serial.print(", ");
			Serial.print(motorCommand[REAR_RIGHT]);
			Serial.println(" ");
			messageRx.query = 'x';
			break;

		case 'b': // add roll
			roll -= 0.1;
			messageRx.query = 'x';
			break;

		case 'c': // add pitch
			pitch -= 0.1;
			messageRx.query = 'x';
			break;

		case 'd': // zero roll/pitch
			roll = 0;
			pitch = 0;
			setMotorCommand(1000);
			messageRx.query = 'x';
			break;

		case 'e': // stop motors
			setMotorCommand(1000);
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

	if (payloadLength > 6) {

		if (payload[3] == 7 && payload[4] == 8 && payload[6] == 9) messageRx.heartbeat = 78.9;
	}

	newDataReady = false;
}


/*******************************************************************************************
* XbeeRx
*
* Handles all incoming communication for the XBee.
* NOTE: Assumes all incoming messages follow ZigBee RX_RESPONSE protocol.
********************************************************************************************/
void XbeeRx() {

	if (Serial.available()) {

		messageRx.query = Serial.read();

		processXbeeCommand();

		resetEmergencyStop = validateSerialStatus();

	}


	// xbee.readPacket();

	// if (xbee.getResponse().isAvailable()) {
	// 	// got something

	// 	if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
	// 		// got a zb rx packet

	// 		// now fill our zb rx class
	// 		xbee.getResponse().getZBRxResponse(rx);

	// 		//get the data
	// 		payload = rx.getData();
	// 		payloadLength = rx.getDataLength();

	// 		newDataReady = true;
	// 	}
	// }

	// if (newDataReady) parseData();

	// processXbeeCommand();

	// resetEmergencyStop = checkHeartBeat();
	// messageRx.heartbeat = 0.0;
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