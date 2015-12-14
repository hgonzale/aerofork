import processing.core.*;

/*******************************************************************************************
aerofork GUI event handling

Processes user interaction with the GUI
********************************************************************************************/


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
      //myPort.write(byte(225));
      myPort.write(byte(235)); // 11101011
      int bits = Float.floatToIntBits(-12.45);
      myPort.write((byte)(bits & 0xff));
      myPort.write((byte)((bits >> 8) & 0xff));
      myPort.write((byte)((bits >> 16) & 0xff));
      myPort.write((byte)((bits >> 24) & 0xff));
      println("");
      println(binary(((byte)(bits & 0xff))));
      println(binary((byte)((bits >> 8) & 0xff)));
      println(binary((byte)((bits >> 16) & 0xff)));
      println(binary((byte)((bits >> 24) & 0xff)));
          
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
