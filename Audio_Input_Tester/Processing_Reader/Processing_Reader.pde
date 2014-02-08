// Graphing sketch
 
 
 // This program takes ASCII-encoded strings
 // from the serial port at 9600 baud and graphs them. It expects values in the
 // range 0 to 1023, followed by a newline, or newline and carriage return
 
 // Created 20 Apr 2005
 // Updated 18 Jan 2008
 // by Tom Igoe
 // This example code is in the public domain.
 
 import processing.serial.*;
 
 Serial myPort;        // The serial port
 int xPos = 0;         // horizontal position of the graph
 
 void setup () {
 // set the window size:
 size(1090, 512);        
 
 // List all the available serial ports
 println(Serial.list());
 // I know that the first port in the serial list on my mac
 // is always my  Arduino, so I open Serial.list()[0].
 // Open whatever port is the one you're using.
 myPort = new Serial(this, Serial.list()[0], 2666667);
 // don't generate a serialEvent() unless you get a newline character:
 myPort.bufferUntil('\n');
 // set inital background:
 background(0);
 }
 void draw () {
 // everything happens in the serialEvent()
 }
 
 void serialEvent (Serial myPort) {
 // get the ASCII string:
 String inString = myPort.readStringUntil('\n');
  
 if (inString != null && int(inString) != 44444) 
 {
   // trim off any whitespace:
   inString = trim(inString);
   // convert to an int and map to the screen height:
   float inByte = float(inString);
   inByte = map(inByte, 0, 300, 0, height);
   
   // draw the line:
   stroke(127,34,255);
   line(xPos, height, xPos, height - inByte);
   xPos++;
 }
 // at the edge of the screen, go back to the beginning:
 if (int(inString) == 44444) 
   {
   xPos = 0;
   background(0); 
   } 
   // increment the horizontal position:
 }
 
