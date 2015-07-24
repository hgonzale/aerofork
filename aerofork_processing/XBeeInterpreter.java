
import processing.core.*;
import processing.serial.*;
import java.nio.Buffer;
import java.nio.ByteBuffer;

/*
XBeeInterpreter.java
A Java class meant to assist a Processing sketch with XBee communication.

Written by Kjartan Brownell

Credit: 
Parts of this code are based off of "XBee API Library for Processing" by 
Rob Faludi and Dan Shiffman:
https://www.faludi.com/examples/xbee-api-library-for-processing/

*/


public class XBeeInterpreter {

	private boolean DEBUG = true;

	public static final byte ZIGBEE_RX_PACKET = (byte) 0x90;
	public static final byte START_BYTE = (byte) 0x7E;

	private ByteBuffer buf;
	private int addr64H, addr64L;


	PApplet parent;
	Serial port = null;

	/******************************************************************************
	* XBeeInterpreter
	*
	* Constructor for the XBeeInterpreter class.
	*
	* p: the PApplet parent
	* thisPort: the Serial port through which the XBee's are communicating.
	*
	* NOTE: In the Processing sketch, the PApplet is most likely 'this'.
	* 		(i.e. XBeeInterpreter xbee = new XBeeInterpreter(this, myPort) )
	******************************************************************************/
	public XBeeInterpreter (PApplet p, Serial thisPort) {

		port = thisPort;
		parent = p;

		buf = ByteBuffer.allocate(100);

	}

	/******************************************************************************
	* readIncoming
	*
	* Process the data coming in over the Serial stream. Checks for the emergency
	* stop flag '^'.
	******************************************************************************/
	public String readIncoming() {

		String out = new String();
		int b;

		while (port.available() > 0) {

			b = port.read();

			out += (char) (byte) b;

			// if (DEBUG) System.out.print(" " + b);

			// switch (b) {

			// 	case '^':
			// 		return "^";

			// 	default:
			// 		out += (char) (byte) b;
			// 		break;

			// }
		}

		return out;
	}

	/******************************************************************************
	* readFloat
	*
	* Uses the ByteBuffer to interpret four incoming bytes as a float.
	******************************************************************************/
	public float readFloat() {

		byte b;

		if (port.available() > 3) {

			for (int i = 0; i < 4; i++) { 

				b = (byte) port.read();

				buf.put(b); 

			}
			return buf.getFloat();
		}

		buf.clear();
		return (float) 0.0;
	}

	/******************************************************************************
	* readInt
	* 
	* Uses the ByteBuffer to interpret two incoming bytes as an int.
	******************************************************************************/
	public int readInt() {

		byte b;

		if (port.available() > 1) {

			for (int i = 0; i < 2; i++) { 

				b = (byte) port.read();

				buf.put(b); 
				
			}
			return (int)(buf.getShort());
		}

		buf.clear();
		return 0;
	}

	/******************************************************************************
	* sendPacket
	*
	* frame: the data and message information being sent to the remote XBee.
	*
	* Prepends the necessary packet information and appends the checksum. Writes 
	* the message packet to the Serial stream.
	******************************************************************************/
	private void sendPacket(byte[] frame) {

		int packetLength = frame.length + 4;

		byte[] packet = new byte[packetLength];
		packet[0] = START_BYTE;
		packet[1] = (byte) (frame.length / 256);
		packet[2] = (byte) (frame.length % 256);

		for (int b = 0; b < frame.length; b++) {

			packet[b+3] = frame[b];
		}

		packet[packetLength - 1] = (byte) checkSum(frame);

		if (DEBUG) System.out.print("< ");

		for (int i = 0; i < packet.length; i++) {

			port.write(packet[i]);
			if (DEBUG) System.out.print(parent.hex(packet[i], 2) + " ");
		}

		if (DEBUG) System.out.println();

	}

	/******************************************************************************
	* checkSum
	*
	* thisArray: the packet for which we are computing a checksum
	*
	* Compute the checksum of a given packet.
	******************************************************************************/
	private int checkSum(byte[] thisArray) {
		int ck = 0;

		for (int i = 0; i < thisArray.length; i++) {
			ck += thisArray[i];
		}

		ck = (0xFF & ck);
		ck = (0xFF - ck);
		return (int)(ck);
	}

	/******************************************************************************
	* sendData
	*
	* data: a String containing the payload of the XBee message.
	*
	* Break down the data into bytes and send to remote XBee.
	******************************************************************************/
	public void sendData(String data) {

		int payloadLength = data.length() + 2;
		int packetLength = payloadLength + 12;

		byte[] packet = new byte[packetLength];

		packet[0] = ZIGBEE_RX_PACKET;
		packet[1] = 0x01;

		short idx = 2;

		// set 64-bit address
		for (int b = 3; b > -1; b--) {

			packet[idx] = (byte) (this.addr64H >> (8*b));
			packet[idx+4] = (byte) (this.addr64L >> (8*b));
			idx++;
		}

		idx += 4;

		packet[idx++] = (byte) 0xFF; // 16-bit address hi
		packet[idx++] = (byte) 0xFE; // 16-bit address lo

		packet[idx++] = (byte) 0x00; // broadcast radius
		packet[idx++] = (byte) 0x00; // options

		// write the payload
		for (int i = 0; i < data.length(); i++) {

			packet[idx++] = (byte) data.charAt(i);
		}

		sendPacket(packet);
	}


	/******************************************************************************
	* setTargetAddr
	*
	* addressH: the high 32-bits of the remote XBee's 64-bit address
	* addressL: the low 32-bits of the remote XBee's 64-bit address
	*
	* Set the 64-bit address of the remote XBee.
	******************************************************************************/
	public void setTargetAddr(int addressH, int addressL) {

		this.addr64H = addressH;
		this.addr64L = addressL;

	}














}
