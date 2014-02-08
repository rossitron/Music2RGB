#include <Audio.h>
#include <Wire.h>
#include <SD.h>


// Create the Audio components.  These should be created in the
// order data flows, inputs/sources -> processing -> outputs
//
AudioInputAnalog analogPinInput(14); // analog A2 (pin 16)
//AudioOutputPWM   pwmOutput;          // audio output with PWM on pins 3 & 4

// Create Audio connections between the components
//

AudioAnalyzeFFT1024  myFFT(1);  // don't know what the 3 is doing, averages?
AudioConnection c1(analogPinInput, 0, myFFT, 0);

// Create an object to control the audio shield.
// 

unsigned long ffttimelast = 0;

void setup() {
  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(12);
  ffttimelast = micros();
  // Enable the audio shield and set the output volume.
}

void loop() {
  //unsigned long timenow = micros();
  //myFFT.update();
  if (myFFT.available()) {
    unsigned long ffttime = micros() - ffttimelast;
    ffttimelast = micros();
    //float Hz = 1000000/float(ffttime);
    //Serial.println(ffttime);
    //Serial.println(Hz);
    // each time new FFT data is available
    // print it all to the Arduino Serial Monitor
    Serial.println("44444");
    for (int i=0; i<512; i++) {Serial.println(myFFT.output[i]);}
    //for (int i=0; i<2090; i = i + 2) {Serial.println(myFFT.output[i]);}
    ///delay(150);
  }
  else
  {
    //Serial.println("Nope");
  }
}

