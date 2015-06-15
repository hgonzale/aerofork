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
boolean calibrationComplete = false;
boolean emergencyStop = false;
int HB_FREQ = 10; // heartbeat frequency in Hz
int previousTime = 0;

/*************************************************
*          Define quadcopter states              *
**************************************************/
int BOOTUP = 0;
int CALIBRATE = 1;
int FLIGHT = 2;
int EMGSTOP = 3;
int STATE = BOOTUP; // default state


/*************************************************
* Constants, functions used for building buttons *
**************************************************/
// Calibrate button
int CALIB_X = 491;
int CALIB_Y = 135;
int CALIB_WIDTH = 85;
int CALIB_HEIGHT = 25;
void drawButton_CALIB() {
  fill(250);
  rect(CALIB_X,CALIB_Y,CALIB_WIDTH,CALIB_HEIGHT,5);
  fill(20);
  textSize(14);
  text("Calibrate",CALIB_X +14,CALIB_Y +18);
}

// Clear button
int CLEAR_X = 380;
int CLEAR_Y = 135;
int CLEAR_WIDTH = 80;
int CLEAR_HEIGHT = 25;
void drawButton_CLEAR() {
  fill(250);
  rect(CLEAR_X,CLEAR_Y,CLEAR_WIDTH,CLEAR_HEIGHT,5);
  fill(20);
  textSize(14);
  text("Clear",CLEAR_X +24,CLEAR_Y +18);
}

// Fly button
int FLY_X = 491;
int FLY_Y = 180;
int FLY_WIDTH = 85;
int FLY_HEIGHT = 25;
void drawButton_FLY() {
   fill(250);
   rect(FLY_X,FLY_Y,FLY_WIDTH,FLY_HEIGHT,5);
   fill(20);
   textSize(14);
   text("Fly",FLY_X +33,FLY_Y +18);
}


/*********************************************************************************
* setup
*
* Setup function of the Processing sketch.
*********************************************************************************/
void setup() {
  size(585,530);
  String portName = Serial.list()[0]; //port 8 on serial port.
//  myPort = new Serial(this, portName, 115200);
}


void cycleState() {
 if (STATE == BOOTUP) {
  STATE = CALIBRATE;
 } else if (STATE == CALIBRATE) {
  STATE = FLIGHT;
 } else {
   STATE = BOOTUP;
 }
}


/*********************************************************************************
* draw
*
* Main loop of the Processing sketch.
*********************************************************************************/
void draw() {
  drawBackground();
//  processOutgoing();  
//  processIncoming();
  if (sendReady) {
    println(saved);
    sendReady = false;
  }
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

      // Send heartbeat signal if enough time has passed, and quadcopter is in FLIGHT mode
    if ((millis() - previousTime)/1000.0 > 1.0/HB_FREQ && STATE == FLIGHT) {
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
  //background(51);
  //background(0,43,54); // nice deep blue
  background(39,40,34); // sublime text2 background
  
  fill(50);
  rect(0,0,width -1,35); //top rectangle
  rect(0,125 + 4,width -1,35); //middle rectangle
  rect(0,height-35,width-1,34); //bottom rectangle
  rect(width-101,0,100,height-1); //sidebar
  
  
  stroke(220);
  line(0,120,width,120);
  fill(250);
  textSize(14);
  text("Type your message below, and hit 'Enter' to send: ",6,23);
  text("Incoming messages...",8,150);
  text("Hit the 'END' key at any time to trigger an Emergency Stop",8,height-12);
  
  drawStatusIndicator(STATE);
  
  drawButton_CLEAR();
  drawButton_CALIB();
  drawButton_FLY();
}

/*********************************************************************************
* drawStatusIndicator
* 
* Draws the status indicator in the window, indicating the current state.
*********************************************************************************/
void drawStatusIndicator(int status) {

  fill(0,43,54);
  rect(width-101,0,100,120); //base box
 
  textSize(10);
  fill(200); // default text color
  text("Bootup",width-75,25);
  text("Calibrate",width-75,50);
  text("Flight",width-75,75);
  text("EmgStop",width-75,100);
  
  switch (status) { //illuminate current status
    case 0: //bootup
      fill(54,226,83); //neonish green
      text("Bootup",width-75,25);
      break;
    
    case 1: //calibrate
      if (calibrationComplete) {
        fill(54,226,83); //neonish green
      } else {
        fill(255,196,0);
      }
      text("Calibrate",width-75,50);
      break;
  
    case 2: //flight
      fill(54,226,83); //neonish green
      text("Flight",width-75,75);
      break;
     
    case 3: //emgstop
      fill(250,0,50); //scarlet red
      text("EmgStop",width-75,100);
      break;
  }
}

/*********************************************************************************
* drawEmgStopWarning
* 
* Displays a message at the bottom of the window informing the user that the 
* emergency stop procedure has been triggered.
*********************************************************************************/
void drawEmgStopWarning() {
  fill(255,238,0);
  rect(0,height-35,width-101,34);
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
* mouseOverRect
*
* Determines if the mouse is over a rectangle.
* x: x coordinate of the rectangle
* y: y coordinate of the rectangle
* w: width of the rectangle
* h: height of the rectangle
*
* Returns true if mouse is over the rectangle.
*********************************************************************************/
boolean mouseOverRect(int x, int y, int w, int h) {
 return (mouseX >= x && mouseX <= x+w && mouseY >= y && mouseY <= y+h); 
}


/*********************************************************************************
* mouseClicked
* 
* Called whenever the mouse is clicked, executes when mouse is released.
*********************************************************************************/
void mouseClicked() {
 if (mouseOverRect(CALIB_X,CALIB_Y,CALIB_WIDTH,CALIB_HEIGHT) && STATE == BOOTUP) { // calibrate button
    // run calibration routine
    STATE = CALIBRATE;
    saved = "@"; // Serial command for accelerometer calibration
    sendReady = true;
  
  } else if (mouseOverRect(CLEAR_X,CLEAR_Y,CLEAR_WIDTH,CLEAR_HEIGHT)) { // clear button
    // clear all messages
    msgDisplay = "";

  } else if (mouseOverRect(FLY_X,FLY_Y,FLY_WIDTH,FLY_HEIGHT,5) && calibrationComplete) { // fly button
     // begin flight
    STATE = FLIGHT;
    saved = "]"; // Serial command for beginning control
    sendReady = true;
  }
  
  
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
    STATE = EMGSTOP;
  } else if (keyCode != SHIFT) { // we don't care if the SHIFT key is pressed
    
    // If the return key is pressed, save the String and clear user input.
    if (key == '\n' ) {
      
      saved = userInp;      
      if (STATE == FLIGHT) { // heartbeat signal sent only during FLIGHT state
         saved += heartBeat; 
      }     
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




  


























