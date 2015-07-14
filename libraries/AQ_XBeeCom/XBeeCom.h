

#ifndef _AEROQUAD_XBEE_COM_H_
#define _AEROQUAD_XBEE_COM_H_



#include <XBee.h>

// XBee and ZigBee communication setup
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();

uint8_t* payload;
int payloadLength = -1;
// XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x4092fd80);
// ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
// ZBTxStatusResponse txStatus = ZBTxStatusResponse();






/* getPacket() variables */
bool packetReceived = false;
bool badCheckSum = false;
int lengthMSB = -1;
int lengthLSB = -1;
int packetLength = -1;
uint8_t dataArray[19] = {0}; // assumes 5-byte payload
int checksum = -1;
int ck = 0;

int timeout = 0;

int delayTime = 200;

bool newDataReady = false;

bool gotSomething = false;

union messageData {
	char query;
	float heartbeat;
} messageRx; // 5 bytes





/*************************************************************************************************************
* getPacket
*
**************************************************************************************************************/
void getPacket() {

	// 20 tries to find the start byte
	while (Serial.read() != 0x7E && timeout < 22) {
		timeout++;
	}

	packetReceived = (timeout < 22); //true if we didn't time out
	timeout = 0;


	if (packetReceived) {
		lengthMSB = Serial.read();
		lengthLSB = Serial.read();
		packetLength = lengthLSB + (lengthMSB << 8);

		packetLength = min(packetLength, 19);
		for (int i = 0; i < packetLength; i++) {
			dataArray[i] = Serial.read();
			ck += dataArray[i];
		}

		// checksum
		ck = (0xFF & ck);
		ck = (0xFF - ck);
		checksum = Serial.read();

		badCheckSum = !(checksum == ck);
		ck = 0;

		packetReceived = false;
	}
}


/*************************************************************************************************************
* checkHeartBeat
*
**************************************************************************************************************/
bool checkHeartBeat() {
	return (messageRx.heartbeat = 78.9);
}


/*************************************************************************************************************
* parseData
*
**************************************************************************************************************/
void parseData(uint8_t* data) {

	messageRx.query = payload[2];

	if (payload[3] == 0x37 && payload[4] == 0x38 && payload[5] == 0x39) {
		messageRx.heartbeat = 78.9;
	} else {
		messageRx.heartbeat = 0.0;
	}

	newDataReady = true;
	
}



/*************************************************************************************************************
* XbeeTx
*
* 
**************************************************************************************************************/
void XbeeTx() {

	

}


/*************************************************************************************************************
* XbeeRx
*
* Look for incoming packets, analyze if available, and pull data from the packets.
**************************************************************************************************************/
void XbeeRx() {

	xbee.readPacket();

	if (xbee.getResponse().isAvailable()) {
		// got something

		gotSomething = true;

		if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
			// got a zb rx packet


			// now fill our zb rx class
			xbee.getResponse().getZBRxResponse(rx);

			//get the data
			payload = rx.getData();
			payloadLength = rx.getDataLength();

			newDataReady = true;

		}
	} else {
		// gotSomething = false;
	}





	// resetEmergencyStop = checkHeartBeat();
	// messageRx.heartbeat = 0.0;
}


void XbeeEcho() {

	getPacket();

	parseData(dataArray);

	if (newDataReady) {
		Serial.write(messageRx.query);
		newDataReady = false;
		gotSomething = true;
	}



	// XbeeRx();

	// if (newDataReady) {

	// 	messageRx.query = payload[0];

	// 	Serial.write(messageRx.query);

	// 	newDataReady = false;
	// }

}

#endif