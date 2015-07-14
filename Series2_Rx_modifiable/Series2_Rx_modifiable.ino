
#include <XBee.h>

#include <XBeeCom.h>

#define serial_write HardwareSerial::write

void setup() {
  messageRx.query = '*';
  Serial.begin(9600);
  Serial.flush();
  xbee.begin(Serial);

}


void loop() {

  XbeeRx();

  if (newDataReady) {

    parseData(payload);

    for (int i = 0; i < payloadLength; i++) {
     // Serial.serial_write(payload[i]);
      Serial.write(payload[i]);
    }
    // Serial.write(0x0a);
    // Serial.write(messageRx.query);
    // Serial.write(0x0a);

    checkHeartBeat();

    newDataReady = false;

  }

	delay(1000);
}


/*


[start_byte]

[lengthHI]

[lengthLO]

[apiID]

[frame ID]

[addrHI] [] [] []

[addrLO] [] [] []

[addr16] []

[broadcast radius]

[option]

[payload] ... []

[checksum]


TRANSMIT_REQUEST -->
				 <-- ZIGBEE_TX_STATUS


whenever you send something to the remote xbee, it sends the ZIGBEE_TX_STATUS back


0 17 | 144 | 0 | 19 162 0 64 | 152 217 203 58 | 137 65 | 104 101 108 108 111 | 38


We send : 

[start_byte | length hi | length lo | apiId | frame type | address hi | address lo | addr16 hi | addr16 lo | broadcast rad | options | payload | checksum]



*/
