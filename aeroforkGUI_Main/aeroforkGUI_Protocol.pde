import processing.core.*;

 char[] commands = {'a','b','c','d','e','i','m','p','q','r','x','y','z','~','?',0};


/*********************************************************************************
* send
*
* s : the command to be sent
*********************************************************************************/

///send a string in the form of "Command +/- a.bc" or just a command
void send( String s ) {

   if ( s.charAt(0)=='a'||s.charAt(0)=='b'||s.charAt(0)=='c'||s.charAt(0)=='e'||
        s.charAt(0)=='m'||s.charAt(0)=='q'||s.charAt(0)=='x'||s.charAt(0)=='r'||
        s.charAt(0)=='~'||s.charAt(0)=='?') {

      int cmdNumber = -1;

      for ( int i= 0; i<16; i++ ) {
         if ( s.charAt(0) == commands[i] ) {
          cmdNumber = i;
          myPort.write(BuildPacket(cmdNumber)); 
        }
      }
     }
  
   else if( s.charAt(0)=='p'||s.charAt(0)=='i'||s.charAt(0)=='d'||
            s.charAt(0)=='y'||s.charAt(0)=='z' ){
       int cmdNumber = -1;

      if ( s.length() > 1 ) {
        for(int i= 0;i<16;i++){
          if (s.charAt(0) == commands[i]) {
            cmdNumber = i;
          }
        }

        myPort.write(BuildPacket(cmdNumber));  ///send header with the command

        int sign = 1;
        if(s.charAt(1) == '-')    sign = -1;   ///read the sign of the number

        int position = FindDecimalPosition(s);   ///find the decimal position of the String

        float data = Conversion(s, position, sign);  ///recover the data from char to float

        int bits = Float.floatToIntBits(data);
        byte[] bytes = new byte[4];
        bytes[0] = (byte)(bits & 0xff);
        bytes[1] = (byte)((bits >> 8) & 0xff);
        bytes[2] = (byte)((bits >> 16) & 0xff);
        bytes[3] = (byte)((bits >> 24) & 0xff);

        myPort.write(bytes[0]);  
        myPort.write(bytes[1]);
        myPort.write(bytes[2]);
        myPort.write(bytes[3]);

        println(binary(bytes[0]));
        println(binary(bytes[1]));
        println(binary(bytes[2]));
        println(binary(bytes[3]));

      }
    }
      
   else   println("Error Command, please try again");
   
}


/*********************************************************************************
* sendDefault
*
* Send the default message.
*********************************************************************************/
void sendDefault() {

  myPort.write(defaultMsg);
}

///helper function for finding the position of the decimal point
public int FindDecimalPosition(String p){
  int position = 0;
  for(int i = 0;i < p.length();i++){
    if(p.charAt(i) == '.'){
      position = i;
    }
  }
  return position;
}

///function for read the input char number as a float
public float Conversion(String s, int position, int sign){
    float multiplier1 = 1;
    float multiplier2 = 0.1;
    float result = 0;
    
    for(int i = position - 1;i > 1;i--){
      result += (((int)s.charAt(i)) - 48)*multiplier1;
      multiplier1 *= 10;
    }
    
    for(int j = position+1;j<s.length();j++){
      result += (((int)s.charAt(j)) - 48)*multiplier2;
      multiplier2 *= 0.1;
    }
    
    result *= sign;
    return result;
}

//helper functions for building a byte
public byte BuildPacket(int Command){
  int Header = 224;
  if(Command <= 15&&Command>= 0){
      byte Packet = byte(Header|Command<<1);
      return AddParity(Packet);
    }
  else{
      println("Invalid Command");
      return byte(0);
    }
}

public byte AddParity(byte a) {
    int NumberOfOne = 0;
    int Allzero = 0;
    int One = 1;
    byte b = a;
    
    for (int i= 0; i<7;i++){
      b= byte(Allzero|b>>1);
      if(b%2!=0) NumberOfOne+=1;
    }
    
    if(NumberOfOne%2 != 0)   return byte(a|One);
    else  return a;
}
