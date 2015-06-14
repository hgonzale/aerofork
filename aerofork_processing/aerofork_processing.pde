import processing.serial.*;


/*******************************************************************************************
aerofork_processing.pde
Processing sketch for communicating with an AeroQuad quadcopter.


DESCRIPTION: Handles communcation between the user and the quadcopter, and provides a window
for viewing all incoming and outgoing Serial messages. A full list of available Serial 
commands can be found in SerialCom.h or AeroQuad_ICD.txt.

NOTE: An important function of this sketch is to send a regular "heartbeat" signal to the 
quadcopter. This signal prevents the quadcopter's emergency stop procedure from being 
triggered. 

IMPORTANT: If at any time the user wishes to stop the quadcopter mid-flight (i.e. trigger
an emergency stop), simply hit the 'END' button on the keyboard. Pressing the 'END' button
will automatically send a message to the quadcopter triggering an emergency stop.
********************************************************************************************/

Serial myPort;
String heartBeat = "78.9"; //Serial heartbeat for emergency stop.
String defaultMsg = "X78.9";
String saved = new String();
String userInp = new String(); //User serial command to be sent to Arduino.
String msgDisplay = new String();
int START_HERE = 185;
boolean sendReady = false;
boolean emergencyStop = false;
int HB_FREQ = 10; // heartbeat frequency in Hz
int previousTime = 0;


void setup() {
  size(485,530);
  String portName = Serial.list()[0]; //port 8 on serial port.
  myPort = new Serial(this, portName, 115200);
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
  if (emergencyStop) {
    drawEmgStopWarning();
  }
}

/*********************************************************************************
* processIncoming
*
* Pulls incoming messages from the Serial stream and displays them in the window.
*********************************************************************************/
void processIncoming() {

  // append incoming messages to the message string
  while (myPort.available() > 0) {
    String in = myPort.readString();
    msgDisplay += in;
  }
  
  // finally, print all incoming messages to the window
  textSize(12);
  text(getLastLines(msgDisplay,15),8,START_HERE);
}

/*********************************************************************************
* processOutgoing
*
* Sends user input (if available) over the Serial port. If user input is being sent,
* the hearbeat signal is appended to the messsage. Otherwise, the heartbeat is sent
* at intervals determined by the HB_FREQ variable.
*********************************************************************************/
void processOutgoing() {
  
  if (sendReady) { // send the custom message and heartbeat signal
    myPort.write(saved);
    println(saved);
    sendReady = false;
  }  else { // or just send the heartbeat signal
    fill(250);
    textSize(26);
    text(userInp,75,82);

    // Send heartbeat signal if enough time has passed
    if ((millis() - previousTime)/1000.0 > 1.0/HB_FREQ) {
          previousTime = millis();
          myPort.write(defaultMsg);
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
  if (idx > 0) {
    String msgOut = new String();
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

/*********************************************************************************
* drawBackground
* 
* Draws the background text, shading, and boxes in the window.
*********************************************************************************/
void drawBackground() {
  // Set background and permanent text
  background(51);
  fill(60);
  rect(0,0,width -1,35); //top rectangle
  rect(0,125 + 4,width -1,35); //middle rectangle
  rect(0,height-35,width-1,34); //bottom rectangle
  
  stroke(250);
  line(0,125,width,125);
  fill(250);
  textSize(14);
  text("Type your message below, and hit 'Enter' to send: ",6,23);
  text("Incoming messages...",8,125 + 25);
  text("Hit the 'END' key at any time to trigger an Emergency Stop",8,height-12);
}

/*********************************************************************************
* drawEmgStopWarning
* 
* Displays a message at the bottom of the window informing the user that the 
* emergency stop procedure has been triggered.
*********************************************************************************/
void drawEmgStopWarning() {
  fill(200);
  rect(0,height-35,width-1,34);
  fill(206,30,30);
  textSize(20);
  text("!!! Emergency Stop triggered !!!",80,height-10);
}

void drawHeart() {
  int x = width - 90;
  int y = height - 55;
  
  smooth();
  noStroke();
  fill(206,30,30);
  beginShape();
  vertex(50+x, 15+y); 
  bezierVertex(50+x, -5+y, 90+x, 5+y, 50+x, 40+y); 
  vertex(50+x, 15+y); 
  bezierVertex(50+x, -5+y, 10+x, 5+y, 50+x, 40+y); 
  endShape();
}
    
/*********************************************************************************
* keyPressed
* 
* Called whenever a key on the keyboard is pressed. Reads the pressed key and 
* interprets the key as a message or direct command.
*********************************************************************************/
void keyPressed() {
  if (keyCode == 35) { // check for emergency stop key (35 is the keyCode for END)
   saved = "~";
   sendReady = true;
   emergencyStop = true;
  } else if (keyCode != SHIFT) {
    // If the return key is pressed, save the String and clear user input.
    if (key == '\n' ) {
      saved = userInp + heartBeat;
      // A String can be cleared by setting it equal to ""
      userInp = ""; 
      sendReady = true;
    } else if (key == BACKSPACE && userInp.length() > 0) {
      userInp = userInp.substring(0, userInp.length() - 1);
      sendReady = false;
    } else {
      // Otherwise, concatenate the String
      // Each character typed by the user is added to the end of the String variable.
      userInp = userInp + key; 
      sendReady = false;
    }
  }
}  
