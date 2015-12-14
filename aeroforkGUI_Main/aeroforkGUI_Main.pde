import processing.core.*;
import processing.serial.*;


/*******************************************************************************************
aerofork_processing.pde
Processing sketch for communicating with an AeroQuad quadcopter.
Written by Kjartan Brownell


DESCRIPTION: Handles communcation between the user and the quadcopter, and provides a window
for viewing all incoming and outgoing Serial messages.

NOTE: An important function of this sketch is to send a regular "heartbeat" signal to the 
quadcopter. This signal prevents the quadcopter's emergency stop procedure from being 
triggered. 

IMPORTANT: If at any time the user wishes to stop the quadcopter mid-flight (i.e. trigger
an emergency stop), hit the 'END' button on the keyboard. Pressing the 'END' button will 
automatically send a message to the quadcopter triggering an emergency stop.

********************************************************************************************/

Serial myPort;
String portName = "/dev/ttyUSB0";
int BAUD = 115200;

boolean USE_XBEE = false;

char HEADER = '#';
char HEARTBEAT = '$';

String defaultMsg = "#x$";

String saved = "";
String userInp = ""; //User's serial command to be sent to the quadrotor

String msgDisplay = new String(); //message from quadrotor
String localMsg = new String(); //local message
boolean localMsgReady = false;
boolean sendReady = false;

boolean emergencyStop = false;
boolean calibrationComplete = false;
boolean flightDataIncoming = false;
int START_HERE = 185; // useful for keeping track of window sizing
float HB_FREQ = 2; // heartbeat frequency in Hz
int previousTime = 0;

int counterVar = 0;


/*************************************************
*          Define quadcopter states              *
**************************************************/
int BOOTUP = 0;
int CALIBRATE = 1;
int FLIGHT = 2;
int EMGSTOP = 3;
int status = BOOTUP; // default state


/*********************************************************************************
* setup
*
* Setup function of the Processing sketch.
*********************************************************************************/
void setup() {

  size(650,600);

  if (!USE_XBEE) portName = Serial.list()[0]; //port 8 on serial port.
  
  print("Opening port " + portName);

  myPort = new Serial(this, portName, BAUD);
  

}


/*********************************************************************************
* draw
*
* Main loop of the Processing sketch.
*********************************************************************************/
void draw() {
  
  drawBackground();

  processOutgoing();  

  processIncoming();

  if (emergencyStop) drawEmgStopWarning();

}


/*********************************************************************************
* processIncoming
*
* Pulls incoming messages from the Serial stream and displays them in the window.
**********************************************************************************/
void processIncoming() {

  // append incoming messages to the message string
  while (myPort.available() > 0) {

    String in = myPort.readString();
    msgDisplay += in;

    // check for emergency stop signal
    if (match(in,"~") != null) {

      emergencyStop = true;
      status = EMGSTOP;
      
    }

  }

  // finally, print all incoming messages to the window
  textSize(12);
  text(getLastLines(msgDisplay,20),8,START_HERE);
  
}


/*********************************************************************************
* processOutgoing
*
* Sends user input (if available) over the Serial port. If user input is being sent,
* the hearbeat signal is appended to the messsage. Otherwise, the heartbeat is sent
* at intervals determined by the HB_FREQ variable.
*********************************************************************************/
void processOutgoing() {
  
  if (sendReady) { // send the custom message AND heartbeat signal

    if (!saved.equals("?") && status != EMGSTOP) {

      flightDataIncoming = false;
      defaultMsg = "#x$";

    }

    send(saved);

    println(saved);
    sendReady = false;
    previousTime = millis();
    
  }  else { // or send just the heartbeat signal

    fill(250);
    textSize(26);
    text(userInp,75,82);

    // Send heartbeat signal if enough time has passed, and quadcopter is in FLIGHT mode
    if ((millis() - previousTime)/1000.0 > 1.0/HB_FREQ && ( status == FLIGHT || flightDataIncoming )) {

        previousTime = millis();

        sendDefault();
    }
  }
}


/*********************************************************************************
* getLastLines
*
* str: String from which we're extracting lines
* n: number of lines to extract from the end of str (as determined by # of '\n')
* 
* Returns a substring of str consisting only of the last n lines of str
*********************************************************************************/
String getLastLines(String str, int n) {
  int counter = 0;
  int idx = str.length() -1;
  if (idx > -1) {
    while (counter < n && idx > 0) {
       if (str.charAt(idx) == '\n') {
         counter++;
       }
       idx--;
    }
    return str.substring(idx);
  }
  return "";
}
    




  


























