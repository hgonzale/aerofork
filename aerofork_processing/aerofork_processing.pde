import processing.core.*;
import processing.serial.*;


/*******************************************************************************************
aerofork_processing.pde
Processing sketch for communicating with an AeroQuad quadcopter.
Written by Kjartan Brownell


DESCRIPTION: Handles communcation between the user and the quadcopter, and provides a window
for viewing all incoming and outgoing Serial messages. A full list of available Serial 
commands can be found in SerialCom.h or AeroQuad_ICD.txt.

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

boolean USE_XBEE = true;

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


/**************************************************
* Constants & functions used for building buttons *
***************************************************/
// Calibrate Baro button
int CALIBBARO_X = 556;
int CALIBBARO_Y = 135;
int CALIBBARO_WIDTH = 85;
int CALIBBARO_HEIGHT = 25;
void drawButton_CALIBBARO() {
  fill(250);
  rect(CALIBBARO_X,CALIBBARO_Y,CALIBBARO_WIDTH,CALIBBARO_HEIGHT,5);
  fill(20);
  textSize(14);
  text("Calibrate",CALIBBARO_X +14,CALIBBARO_Y +18);
}

// Calibrate Accel button
int CALIBACCEL_X = 556;
int CALIBACCEL_Y = 180;
int CALIBACCEL_WIDTH = 85;
int CALIBACCEL_HEIGHT = 25;
void drawButton_CALIBACCEL() {
  fill(250);
  rect(CALIBACCEL_X,CALIBACCEL_Y,CALIBACCEL_WIDTH,CALIBACCEL_HEIGHT,5);
  fill(20);
  textSize(14);
  text("Ready",CALIBACCEL_X +24,CALIBACCEL_Y +18);
}

// Clear button
int CLEAR_X = 450;
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
int FLY_X = 556;
int FLY_Y = 225;
int FLY_WIDTH = 85;
int FLY_HEIGHT = 25;
void drawButton_FLY() {
   fill(250);
   rect(FLY_X,FLY_Y,FLY_WIDTH,FLY_HEIGHT,5);
   fill(20);
   textSize(14);
   text("Fly",FLY_X +33,FLY_Y +18);
}

// Flight data button
int FLIGHTDATA_X = 556;
int FLIGHTDATA_Y = 270;
int FLIGHTDATA_WIDTH = 85;
int FLIGHTDATA_HEIGHT = 25;
void drawButton_FLIGHTDATA() {
  fill(250);
  rect(FLIGHTDATA_X,FLIGHTDATA_Y,FLIGHTDATA_WIDTH,FLIGHTDATA_HEIGHT,5);
  fill(20);
  textSize(14);
  text("Data", FLIGHTDATA_X +28, FLIGHTDATA_Y +18);
}

// Reset button
int RESET_X = 556;
int RESET_Y = 497;
int RESET_WIDTH = 85;
int RESET_HEIGHT = 25;
void drawButton_RESET() {
   fill(250);
   rect(RESET_X,RESET_Y,RESET_WIDTH,RESET_HEIGHT,5);
   fill(20);
   textSize(14);
   text("Reset", RESET_X +25, RESET_Y +18);
}


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


/*********************************************************************************
* drawBackground
* 
* Draws the background text, shading, and boxes in the window.
*********************************************************************************/
void drawBackground() {

  background(39,40,34); // sublime text2 charcoal background
  
  fill(50);
  fill(0,43,54);
  rect(0,height-35,width-1,34); //bottom rectangle
  rect(0,0,width -1,35); //top rectangle
  rect(0,125 + 4,width -1,35); //middle rectangle
  rect(width-101,0,100,height-1); //sidebar
  
  stroke(220);
  line(0,120,width,120);
  fill(250);
  textSize(14);
  text("Type your message below, and hit 'Enter' to send: ",6,23);
  text("Hit the 'END' key at any time to trigger an Emergency Stop",8,height-12);
  
  
  displayLocalMessage();
  
  drawStatusIndicator();
  
  drawButton_CLEAR();
  drawButton_CALIBBARO();
  drawButton_CALIBACCEL();
  drawButton_FLY();
  drawButton_RESET();
  drawButton_FLIGHTDATA();
}


/*********************************************************************************
* drawStatusIndicator
*
* Draws the status indicator in the window, indicating the current state.
*********************************************************************************/
void drawStatusIndicator() {

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
* displayLocalMessage
*
* Display a message locally on the GUI.
*********************************************************************************/
void displayLocalMessage() {
  
  if (localMsgReady) {

    fill(255,222,0); // yellow
    triangle(11,157,21,137,31,157);
    fill(39,40,34); // charcoal grey
    textSize(14);
    text("!",19,154);
    fill(250);
    text(localMsg,40,153);

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
* Called whenever the mouse is clicked. Executes when mouse is released.
* Provides functionality to the buttons.
*********************************************************************************/
void mouseClicked() {
 if (mouseOverRect(CALIBBARO_X,CALIBBARO_Y,CALIBBARO_WIDTH,CALIBBARO_HEIGHT) && status == BOOTUP) { // calibrate button
    
    status = CALIBRATE;
    localMsg = "Calibrating...";
    localMsgReady = true;
    
  
  } else if (mouseOverRect(CALIBACCEL_X,CALIBACCEL_Y,CALIBACCEL_WIDTH,CALIBACCEL_HEIGHT)) {
    calibrationComplete = true;
    localMsg = "Calibration complete...Press 'Fly' when ready!";
    

  } else if (mouseOverRect(CLEAR_X,CLEAR_Y,CLEAR_WIDTH,CLEAR_HEIGHT)) { // clear button
    // clear all messages
    msgDisplay = "";


  } else if (mouseOverRect(FLY_X,FLY_Y,FLY_WIDTH,FLY_HEIGHT) && calibrationComplete) { // fly button
     // begin flight
    status = FLIGHT;
    saved = "c"; // Serial command for beginning control
    sendReady = true;
    localMsg = "";
    localMsgReady = false;
    
    
  } else if (mouseOverRect(RESET_X,RESET_Y,RESET_WIDTH,RESET_HEIGHT)) {
   
    // shut down serial and restart -- NEEDS MORE HERE
    myPort.stop();
    msgDisplay = "";
    localMsg = "Restarting...";
    localMsgReady = true;
    displayLocalMessage();
    delay(500);
    
    myPort.clear();
    status = BOOTUP;
    emergencyStop = false;
    myPort = new Serial(this, Serial.list()[0], 115200);
    
  } else if (mouseOverRect(FLIGHTDATA_X,FLIGHTDATA_Y,FLIGHTDATA_WIDTH,FLIGHTDATA_HEIGHT) && status != EMGSTOP) {
    saved = "?"; // send flight data request
    sendReady = true;
    defaultMsg = "#?$";
    flightDataIncoming = true; 
    
  }
  
  
}

/*********************************************************************************
* send
*
* c : the command to be sent
*********************************************************************************/
void send(char c) {

  myPort.write(HEADER);
  myPort.write(c);
  myPort.write(HEARTBEAT);

}

void send(String s) {

  myPort.write(HEADER);
  myPort.write(s.charAt(0));

  if (s.length() == 6) {

    for (int i = 0; i < 6; i++) {
      myPort.write(s.charAt(i));
    }

  } else {

    myPort.write(HEARTBEAT);

  }

}

/*********************************************************************************
* sendDefault
*
* Send the default message.
*********************************************************************************/
void sendDefault() {

  myPort.write(defaultMsg);
}
    
/*********************************************************************************
* keyPressed
* 
* Called whenever a key on the keyboard is pressed. Reads the pressed key and 
* interprets the key as a message or direct command.
*********************************************************************************/
void keyPressed() {

  switch (keyCode) {

    case 35:
      // 'END' (emergency stop)
      saved = "~";
      defaultMsg = "#~$";
      sendReady = true;
      emergencyStop = true;
      status = EMGSTOP;
      break;

    case 16:
      // 'SHIFT'
      break;

    case ENTER:
      // 'ENTER'
      saved = userInp;      
      userInp = ""; 
      sendReady = true;
      break;

    case 8:
      // 'BACKSPACE'     
      userInp = "";
      sendReady = false;
      break;   

    default:
      userInp += key;
      sendReady = false;
      break;
  }
}




  


























