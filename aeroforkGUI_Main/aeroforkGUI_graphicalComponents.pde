import processing.core.*;

/*******************************************************************************************
aerofork graphical components

Contains all variables, parameters, and methods for the GUI's display.

Code layout: 
	- Buttons
	- Draw methods 

********************************************************************************************/

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