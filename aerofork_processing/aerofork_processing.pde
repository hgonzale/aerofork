import processing.serial.*;

Serial myPort;
String heartBeat = "78.9"; //Serial heartbeat for emergency stop.
String defaultMsg = "X78.9";
String saved = new String();
String userInp = new String(); //User serial command to be sent to Arduino.

boolean sendReady = false;

void setup() {
  
  size(485,300);
  String portName = Serial.list()[0]; //port 8 on serial port.
  myPort = new Serial(this, portName, 115200);

}

void draw() {
  background(51);
  fill(250);
  textSize(14);
  text("Type your message below, and hit 'Enter' to send: ",8,25);
  if (sendReady) {
    myPort.write(saved);
    println(saved);
    sendReady = false;
  }  else {
    fill(250);
    textSize(26);
    text(userInp,100,100);
    myPort.write(defaultMsg);
  }

}
    

void keyPressed() {
  
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
