// GPLv3 
// Copyright 2014 Ross Melville

// IRhashdecode - decode an arbitrary IR code.
// Copyright 2010 Ken Shirriff

#define AVR                    // Using an AVR proc where the uber fast FHT lib can be used?
//#define Teensy3                // Using a Teensy 3.1 proc
#define Debug                  // Uncomment for debug info on serial port
#define NumOfFreqBins     32   // set to 32 point fft or fht
#ifdef AVR
  #define RedPin             6
  #define GreenPin           5
  #define BluePin            3
  #define IR_RECV_PIN        2
  #define IR_POW_PIN         4   // IR receiver powered off GPIO pin
  #define LIN_OUT            1   // use linear fht output function
  #define FHT_N  NumOfFreqBins 
  #define ADCReset        0xf5   // reset the adc, freq = 1000 kHz / 13.5 =~ 74 kHz sampling rate
  #define ADCFreeRun      0xe5   // set the adc to free running mode, freq = 1000 kHz / 13.5 =~ 74 kHz sampling rate
  #define PeakPWMValue     255
#endif
#ifdef Teensy3
  #define RedPin             6
  #define GreenPin           5
  #define BluePin            3
  #define IR_RECV_PIN        2
  #define IR_POW_PIN         4   // IR receiver powered off GPIO pin
  #define FFT_SIZE NumOfFreqBins
  #define AUDIO_PIN 14
  #define ANALOG_READ_RESOLUTION 12
  #define ANALOG_READ_AVERAGING 2
  #define PeakPWMValue   65536
#endif
#define PrintInterval  33334   // serial port print interval in uS
#define ButtonInterval 16667   // interval in uS to check for new button presses
#define MultiSample       10   // Number of audio/FHT loops are done before the largest values seen are used for determining RGB PWM levels.
/* FHT Freq / MultiSample = PWM update Freq *** With serial debug off: Use 30Hz or 24Hz for video recording *** 45-82Hz for human eyes ***
Over ~80Hz PWM refresh and a lot of content starts to look like flickering instead of smooth visual reaction. */

#define RedMinLimit      1350
#define GreenMinLimit    150
#define BlueMinLimit     325
#define PopThreshold     120   // large delta in ADC samples means the reading has lots of error

#ifdef Debug
  #define SerialBuad   2666667 // PuTTY works at odd bitrates like 2666667, 1843200 (both Win7 x64 and Ubuntu 12.04 32bit work fine)
  #define ANSIMax         16*2 // max char length of charts, doubled because of half blocks
#endif

#ifdef AVR
  #include <FHT.h>               // http://wiki.openmusiclabs.com/wiki/ArduinoFHT
#endif

#ifdef Teensy3
 // something FFT here
#endif

#include <IRremote.h>          // https://github.com/shirriff/Arduino-IRremote & http://www.righto.com/2009/08/multi-protocol-infrared-remote-library.html
#include "IRhashdecode.h"      // http://arcfn.com/files/IRhashdecode.pde
#include <EEPROM.h>            // used for saving settings between power cycles


unsigned int OutputGreen = 0;
unsigned int OutputRed = 0;
unsigned int OutputBlue = 0;
unsigned int RedPeak = 0;
unsigned int GreenPeak = 0;
unsigned int BluePeak = 0;
unsigned int RedMin = 65535;   // set to max so a new min is always found the first time
unsigned int GreenMin = 65535;
unsigned int BlueMin = 65535;
unsigned int RedFilterStorage = 0;
unsigned int GreenFilterStorage = 0;
unsigned int BlueFilterStorage = 0;

unsigned int MainArray[NumOfFreqBins/2];
unsigned int PeakArray[NumOfFreqBins/2];
unsigned int MinArray[NumOfFreqBins/2];

#ifdef Debug
  unsigned long TimeStarted = micros();
  unsigned long TimeLastPrintStart = micros();;
  unsigned int PrintingArray[NumOfFreqBins/2];
  unsigned int PrintingPeakArray[NumOfFreqBins/2];
  unsigned int PrintingMinArray[NumOfFreqBins/2];
  unsigned int LoopTimeArray[MultiSample];
  unsigned int TimeError = 0;
  unsigned int PeakRaw = 0;

  byte FoundPeakArray[NumOfFreqBins/2];
  byte FoundMinArray[NumOfFreqBins/2];
  byte DisablePrint = 0;
  int PeakDeltaBad = 0;
  int PeakDeltaGood = 0;
  unsigned int NumOfPops = 0;
  byte FullDebug = 0;
#endif

unsigned int ModeCounter = 0;
byte Mode = 0;
byte ColorWashSpeed = 0;
byte Mode2WashSpeedRed = 0;
byte Mode2WashSpeedGreen = 0;
byte Mode2WashSpeedBlue = 0;
byte RangeErrorRed = 0;
byte RangeErrorGreen = 0;
byte RangeErrorBlue = 0;
byte AutoScaleCounter = 0;
byte SampleCounter = 0;
byte PowerOn = 1;
byte BadSample = 0;
byte ADCTimeLast = 0;
int Delta = 0;

IRrecv irrecv(IR_RECV_PIN);
decode_results results;
byte ButtonDecay = 0;
#define ButtonDecayMax  7
byte ButtonHandled = 1; // 2 = handled, done with event, 0 = new unhandled, 1 = handled, but ready for new event 
String ButtonActive = "None";

void setup()
{
  ResetLEDValues();
  pinMode(RedPin, OUTPUT);
  pinMode(GreenPin, OUTPUT);
  pinMode(BluePin, OUTPUT);
  pinMode(IR_POW_PIN, OUTPUT); // IR sensor power pin is on I/O pin
  digitalWrite(IR_POW_PIN, HIGH); // turn on IR sensor
  #ifdef AVR
    ADCSRA = ADCFreeRun; // set the adc to free running mode
    ADMUX = 0x40; // use adc0
    DIDR0 = 0x01; // turn off the digital input for adc0
  #endif
  irrecv.enableIRIn(); // Start the receiver
  CheckEEPROM();  // checks eeprom memory for safely written settings bits
}

void loop()
{
  #ifdef Debug
    unsigned long TimeExitPrint = micros();     // need to load a vaule in the first time
    unsigned long TimeEnterPrint = micros();    // need to load a vaule in the first time
  #endif
  unsigned long TimeExitButtonCheck = micros(); // need to load a vaule in the first time
  while(1)
  {
    #ifdef Debug
    TimeStarted = micros();
    #endif
    int LastValue = 32767;
    BadSample = 0;
    #ifdef AVR
    for (byte i = 0 ; i < NumOfFreqBins ; i++) // save NumOfFreqBins samples for FHT
    #endif
    #ifdef Teensy3
    for (byte i = 0 ; i < NumOfFreqBins*2 ; i++) // save NumOfFreqBins * 2 samples for FFT (imaginary data)
    #endif
    {
      int Sample;
      if (LastValue != 32767)
      {
        Sample = ReadADC();
        Delta = Sample - LastValue;
        if (abs(Delta) > 100) // looks like a pop, don't process this sample
        {
          BadSample = 1;
          #ifdef Debug
          NumOfPops++;
          if (abs(Delta) > PeakDeltaBad)
            {
              PeakDeltaBad = abs(Delta);
            }
          #endif
        }
        #ifdef Debug
        else
        {
          if (abs(Delta) > PeakDeltaGood)
          {
            PeakDeltaGood = abs(Delta);
          }
        }
        #endif
      }
      else{Sample = ReadADC();} // it's the first sample, just read
      LastValue = Sample;
      #ifdef AVR
        Sample -= 0x01FF;                        // form into a signed int at the midrange point of mic input (511 = 0x01FF, 512 = 0x0200;)
        Sample <<= 6;                            // form into a 16b signed int
      #endif
      fht_input[i] = Sample;         // put real data into bins
      #ifdef Teensy3 // have to pad the array with imaginary numbers
        i++;
        fht_input[i] = 0;
      #endif
    }
    if (BadSample == 0)
    {
    #ifdef AVR
      fht_window();  // window the data for better frequency response
      fht_reorder(); // reorder the data before doing the fht
      fht_run();     // process the data in the fht
      fht_mag_lin(); // take the linear output of the fht
    #endif
    for (byte Index = 0; Index < (NumOfFreqBins/2); Index++)
    {
      #ifdef Debug
        if (abs(fht_input[Index]) > PeakRaw){PeakRaw = abs(fht_input[Index]);} // storing the peak raw value
      #endif
      if (fht_lin_out[Index] > MainArray[Index]){MainArray[Index] = fht_lin_out[Index];}
      if (MainArray[Index] > PeakArray[Index])
      {
        PeakArray[Index] = MainArray[Index];
        #ifdef Debug
          FoundPeakArray[Index] = 1;
        #endif
      }
    }
    if (Mode == 1)
    {
      if (ModeCounter == 0)
      {
        ModeCounter = ColorWashSpeed;
        if (RedFilterStorage > OutputRed){OutputRed++;}
        if (RedFilterStorage < OutputRed){OutputRed--;}
        if (RedFilterStorage == OutputRed)
        {
          randomSeed(MainArray[0]);
          RedFilterStorage = random(0, PeakPWMValue);
        }
        if (GreenFilterStorage > OutputGreen){OutputGreen++;}
        if (GreenFilterStorage < OutputGreen){OutputGreen--;}
        if (GreenFilterStorage == OutputGreen)
        {
          randomSeed(MainArray[1]);
          GreenFilterStorage = random(0, PeakPWMValue);
        }
        if (BlueFilterStorage > OutputBlue){OutputBlue++;}
        if (BlueFilterStorage < OutputBlue){OutputBlue--;}
        if (BlueFilterStorage == OutputBlue)
        {
          randomSeed(MainArray[2]);
          BlueFilterStorage = random(0, PeakPWMValue);
        }
      }
      else {ModeCounter--;}
    }
    else if (Mode == 2)
    {
      if (ModeCounter == 0)
      {
        ModeCounter = ColorWashSpeed;
        if (RedFilterStorage == OutputRed)
        {
          randomSeed(MainArray[0]);
          RedFilterStorage = random(0, PeakPWMValue);
        }
        else if (RedFilterStorage > OutputRed)
        {
          if (OutputRed + Mode2WashSpeedRed > RedFilterStorage || OutputRed + Mode2WashSpeedRed > PeakPWMValue){OutputRed = RedFilterStorage;}
          else {OutputRed = OutputRed + Mode2WashSpeedRed;}
        }
        else if (RedFilterStorage < OutputRed)
        {
          if (OutputRed - Mode2WashSpeedRed < RedFilterStorage || OutputRed - Mode2WashSpeedRed < 0 || OutputRed - Mode2WashSpeedRed > PeakPWMValue){OutputRed = RedFilterStorage;}
          else {OutputRed = OutputRed - Mode2WashSpeedRed;}
        }
        
        if (GreenFilterStorage == OutputGreen)
        {
          randomSeed(MainArray[1]);
          GreenFilterStorage = random(0, PeakPWMValue);
        }
        else if (GreenFilterStorage > OutputGreen)
        {
          if (OutputGreen + Mode2WashSpeedGreen > GreenFilterStorage || OutputGreen + Mode2WashSpeedGreen > PeakPWMValue){OutputGreen = GreenFilterStorage;}
          else {OutputGreen = OutputGreen + Mode2WashSpeedGreen;}
        }
        else if (GreenFilterStorage < OutputGreen)
        {
          if (OutputGreen - Mode2WashSpeedGreen < GreenFilterStorage || OutputGreen - Mode2WashSpeedGreen < 0 || OutputGreen - Mode2WashSpeedGreen > PeakPWMValue){OutputGreen = GreenFilterStorage;}
          else {OutputGreen = OutputGreen - Mode2WashSpeedGreen;}
        }
        if (BlueFilterStorage == OutputBlue)
        {
          randomSeed(MainArray[2]);
          BlueFilterStorage = random(0, PeakPWMValue);
        }
        else if (BlueFilterStorage > OutputBlue)
        {
          if (OutputBlue + Mode2WashSpeedBlue > BlueFilterStorage || OutputBlue + Mode2WashSpeedBlue > PeakPWMValue){OutputBlue = BlueFilterStorage;}
          else {OutputBlue = OutputBlue + Mode2WashSpeedBlue;}
        }
        else if (BlueFilterStorage < OutputBlue)
        {
          if (OutputBlue - Mode2WashSpeedBlue < BlueFilterStorage || OutputBlue - Mode2WashSpeedBlue < 0 || OutputBlue - Mode2WashSpeedBlue > PeakPWMValue){OutputBlue = BlueFilterStorage;}
          else {OutputBlue = OutputBlue - Mode2WashSpeedBlue;}
        }
      }
      else {ModeCounter--;}
    }
    
    if (SampleCounter == MultiSample)
    {
      for (byte Index = 0; Index < (NumOfFreqBins/2); Index++)
      {
        if (MainArray[Index] < MinArray[Index])
        {
          MinArray[Index] = MainArray[Index];
          #ifdef Debug
            FoundMinArray[Index] = 1;
          #endif
        }
      #ifdef Debug
        if (MainArray[Index] > PrintingArray[Index]){PrintingArray[Index] = MainArray[Index];}
        if (PeakArray[Index] > PrintingPeakArray[Index]){PrintingPeakArray[Index] = PeakArray[Index];}
        if (MinArray[Index] < PrintingMinArray[Index]){PrintingMinArray[Index] = MinArray[Index];}
      #endif
      }
  
      if (Mode == 0)
      {
        unsigned int RedMap = ArrayRedParser(MainArray);
        unsigned int GreenMap = ArrayGreenParser(MainArray);
        unsigned int BlueMap = ArrayBlueParser(MainArray);
        
        if (RedMap > RedPeak){RedPeak = RedMap;}
        if (RedMap < RedMin){RedMin = RedMap;}
        
        if (GreenMap > GreenPeak){GreenPeak = GreenMap;}
        if (GreenMap < GreenMin){GreenMin = GreenMap;}
        
        if (BlueMap > BluePeak){BluePeak = BlueMap;}
        if (BlueMap < BlueMin){BlueMin = BlueMap;}
        
        if (RedPeak < RedMinLimit){RedPeak = RedMinLimit;}
        if (GreenPeak < GreenMinLimit){GreenPeak = GreenMinLimit;}
        if (BluePeak < BlueMinLimit){BluePeak = BlueMinLimit;}
      
        RedFilterStorage = map(RedMap, RedMin, RedPeak, 0, PeakPWMValue);
        GreenFilterStorage = map(GreenMap, GreenMin, GreenPeak, 0, PeakPWMValue);
        BlueFilterStorage = map(BlueMap, BlueMin, BluePeak, 0, PeakPWMValue);
        if (RedFilterStorage > PeakPWMValue){RedFilterStorage = PeakPWMValue; RangeErrorRed = 1;}
        if (GreenFilterStorage > PeakPWMValue){GreenFilterStorage = PeakPWMValue; RangeErrorGreen = 1;}
        if (BlueFilterStorage > PeakPWMValue){BlueFilterStorage = PeakPWMValue; RangeErrorBlue = 1;}
        if (RedPeak == RedMinLimit)
        {
          OutputRed = (OutputRed * 4) + (RedFilterStorage * 4) >> 3;
          OutputGreen = (OutputGreen * 4) + (GreenFilterStorage * 4) >> 3;
          OutputBlue = (OutputBlue * 4) + (BlueFilterStorage * 4) >> 3;
        }
        else
        {
          OutputRed = RedFilterStorage;
          OutputGreen = GreenFilterStorage;
          OutputBlue = BlueFilterStorage;
        }
      }
      
      if (ButtonActive == "Red")
      {
        OutputRed = PeakPWMValue;
        OutputGreen = 0;
        OutputBlue = 0;
      }
      else if (ButtonActive == "Green")
      {
        OutputRed = 0;
        OutputGreen = PeakPWMValue;
        OutputBlue = 0;
      }
      else if (ButtonActive == "Blue")
      {
        OutputRed = 0;
        OutputGreen = 0;
        OutputBlue = PeakPWMValue;
      }
      else if (ButtonActive == "Yellow")
      {
        OutputRed = PeakPWMValue;
        OutputGreen = 100;
        OutputBlue = 0;
      }
      else if (ButtonActive == "Power" && ButtonHandled == 0)
      {
        if (PowerOn == 0)
        {
          PowerOn = 1;
          ButtonHandled = 2;
        }
        else
        {
          PowerOn = 0;
          ButtonHandled = 2;
        }
        EEPROM.write(4, 0);
        EEPROM.write(5, PowerOn);
        EEPROM.write(4, 1);
      }
      #ifdef Debug
      else if (ButtonActive == "Mute" && ButtonHandled == 0)
      {
        if (DisablePrint == 0)
        {
          DisablePrint = 1;
          ButtonHandled = 2;
        }
        else
        {
          DisablePrint = 0;
          ButtonHandled = 2;
        }
      }
      else if (ButtonActive == "Option" && ButtonHandled == 0)
      {
        if (FullDebug == 0)
        {
          FullDebug = 1;
          ButtonHandled = 2;
        }
        else
        {
          FullDebug = 0;
          ButtonHandled = 2;
        }
        EEPROM.write(256, 0);
        EEPROM.write(255, FullDebug); // save mode to be set on the next power on
        EEPROM.write(256, 1);
      }
      #endif
      else if (ButtonActive == "Mode" && ButtonHandled == 0)
      {
        if (Mode == 0 || Mode == 1)
        {
          randomSeed(MainArray[0]);
          RedFilterStorage = random(0, PeakPWMValue);
          randomSeed(MainArray[1]);
          GreenFilterStorage = random(0, PeakPWMValue);
          randomSeed(MainArray[2]);
          BlueFilterStorage = random(0, PeakPWMValue);
          Mode++;
          ButtonHandled = 2;
        }
        else
        {
          Mode = 0;
          ButtonHandled = 2;
        }
        EEPROM.write(0, 0);
        EEPROM.write(1, Mode); // save mode to be set on the next power on
        EEPROM.write(0, 1);
      }
      if (Mode == 1 || Mode == 2)
      {
        byte WriteNewEEPROM = 0;
        if (ButtonActive == "Up" && ButtonHandled == 0 && ColorWashSpeed <= PeakPWMValue)
        {
          WriteNewEEPROM = 1;
          if (ColorWashSpeed <= 1){ColorWashSpeed++;}
          else if (ColorWashSpeed * 1.5 >= PeakPWMValue)
          {
            ColorWashSpeed = PeakPWMValue;
          }
          else {ColorWashSpeed = ColorWashSpeed * 1.50;}
          ButtonHandled = 2;
        }
        if (ButtonActive == "Down" && ButtonHandled == 0 && ColorWashSpeed > 0)
        {
          WriteNewEEPROM = 1;
          if ((ColorWashSpeed * .75) < 0)
          {
            ColorWashSpeed = 0;
          }
          else
          {
            ColorWashSpeed = ColorWashSpeed * .75; 
          }
          ButtonHandled = 2;
        }
        if (WriteNewEEPROM == 1)
        {
          EEPROM.write(2, 0);
          EEPROM.write(3, ColorWashSpeed);
          EEPROM.write(2, 1);
        } 
      }
      if (Mode == 2)
      {
        Mode2WashSpeedRed = ArrayRedParser(PeakArray);
        Mode2WashSpeedRed = Mode2WashSpeedRed - ArrayRedParser(MainArray);
        Mode2WashSpeedGreen = ArrayGreenParser(PeakArray);
        Mode2WashSpeedBlue = ArrayBlueParser(MainArray);
      }
      if (PowerOn == 0)
      {
        OutputRed = 0;
        OutputGreen = 0;
        OutputBlue = 0;
      }
      analogWrite(RedPin, OutputRed);
      analogWrite(GreenPin, OutputGreen);
      analogWrite(BluePin, OutputBlue);
      SampleCounter = 0;
      for (byte Index = 0; Index < (NumOfFreqBins/2); Index++){MainArray[Index] = 0;} // Clean the main array
    }
    else {SampleCounter++;}
    unsigned long TimeNow = micros();
    if ((TimeNow - TimeExitButtonCheck) > ButtonInterval)
    {
      CheckRemote();
      TimeExitButtonCheck = micros();
      if (ButtonActive == "Format" && ButtonHandled == 0)
      {
        ButtonHandled = 2;
        ResetLEDValues();
      }
      
      if (AutoScaleCounter == 18) // Magic Number Warning!!!
      {
        for (byte Index = 0; Index < (NumOfFreqBins/2); Index++)
        {
          //if (PeakArray[Index] > 0){PeakArray[Index] = PeakArray[Index] * .985;}
          if (MinArray[Index] <= 20){MinArray[Index]++;}else {MinArray[Index] = MinArray[Index] * 1.025;} // Magic Number Warning!!!
        }
        AutoScaleCounter = 0;
      }
      else {AutoScaleCounter++;}   
    }
  
    #ifdef Debug
    if (DisablePrint == 0)
    {
      TimeNow = micros();
      LoopTimeArray[SampleCounter] = TimeNow - TimeStarted;
      if (SampleCounter == 0)
      {
        if ((TimeNow - TimeExitPrint) > PrintInterval - 5800) // Magic Number Warning! If changing the baud rate this and likely the interval need to be changed.
        {
          unsigned long TimeEnterLoop = micros();
          Serial.begin(SerialBuad);
          SerialColorWhite();
          TimeNow = micros();
          while((TimeNow - TimeExitPrint) < PrintInterval){TimeNow = micros();}
          unsigned long TimePrintStart = TimeNow;
          SerialClearScreen();
          SerialCursorHome();
          if (FullDebug == 1)  //(TimeNow - TimeExitPrint > PrintInterval + 1500)
          {
            TimeNow = micros();
            float LoopTimeFloater = LoopTimeArray[1];
            float Hz = 1000000 / (LoopTimeFloater * MultiSample);
            
            Serial.print(TimeNow - TimeExitPrint);
            Serial.println("µS print intvl");
            // Hz Sample and Hz PWM are accurate to what the speed would be with printing off.
            // With printing on it will be that speed between the serial prints
            Serial.print(1000000 / LoopTimeFloater);
            Serial.print("Hz FHT, ");
            Serial.print(Hz);
            Serial.println("Hz PWM");
          }
          TimeEnterPrint = TimeEnterLoop;
          /*
          for (byte x = 0; x < MultiSample; x ++)
          {
            Serial.print(LoopTimeArray[x]);
            Serial.print(" ");
          }
          */
          unsigned int RedPrint;
          unsigned int RedPeakPrint;
          unsigned int RedMinPrint;
          unsigned int GreenPrint;
          unsigned int GreenPeakPrint;
          unsigned int GreenMinPrint;
          unsigned int BluePrint;
          unsigned int BluePeakPrint;
          unsigned int BlueMinPrint;
          if (Mode == 1 || Mode == 2) // in color washer mode set the min and max to the 8bit PWM output depth range
          {
            RedPrint = OutputRed;
            RedPeakPrint = PeakPWMValue;
            RedMinPrint = 0;
            GreenPrint = OutputGreen;
            GreenPeakPrint = PeakPWMValue;
            GreenMinPrint = 0;
            BluePrint = OutputBlue;
            BluePeakPrint = PeakPWMValue;
            BlueMinPrint = 0;
          }
          else if (Mode == 0)
          {
            RedPrint = ArrayRedParser(PrintingArray);
            RedPeakPrint = ArrayRedParser(PrintingPeakArray);
            RedMinPrint = ArrayRedParser(PrintingMinArray);
            GreenPrint = ArrayGreenParser(PrintingArray);
            GreenPeakPrint = ArrayGreenParser(PrintingPeakArray);
            GreenMinPrint = ArrayGreenParser(PrintingMinArray);
            BluePrint = ArrayBlueParser(PrintingArray);
            BluePeakPrint = ArrayBlueParser(PrintingPeakArray);
            BlueMinPrint = ArrayBlueParser(PrintingMinArray);
            
            if (RedPeakPrint < RedMinLimit){RedPeakPrint = RedMinLimit;}
            if (GreenPeakPrint < GreenMinLimit){GreenPeakPrint = GreenMinLimit;}
            if (BluePeakPrint < BlueMinLimit){BluePeakPrint = BlueMinLimit;}
          }
          if (PeakRaw >= 14000){SerialColorRed();} // change peak prints to red when the input gets very close to clipping (15876 real peak)
          if (PeakRaw >= 12500 && PeakRaw < 14000){SerialColorYellow();}
          PrintBlocks(map(PeakRaw, 0, 15800, 0, ANSIMax)); 
          if (map(PeakRaw, 0, 15800, 0, ANSIMax*6) <= ANSIMax){PrintBlocks(map(PeakRaw, 0, 15800, 0, ANSIMax*6));}
          else {PrintBlocks(ANSIMax);}
          if (FullDebug == 1)
          {
            Serial.print("Raw Pk:");
            unsigned int PeakRaw10bit = PeakRaw;
            PeakRaw10bit = PeakRaw10bit / 32;
            Serial.println(PeakRaw10bit);
            Serial.print("PkGood:");
            Serial.print(PeakDeltaGood);
            Serial.print(" Pops:");
            Serial.print(NumOfPops);
            Serial.print(" PkBad:");
            Serial.println(PeakDeltaBad);
          }
          else{Serial.println();}
          if (PeakRaw >= 14000){SerialColorWhite();}
          if (PeakRaw >= 12500 && PeakRaw < 14000){SerialColorWhite();}
          PeakRaw = 0;
          PrintChart();
          Serial.println();
          SerialColorRed();
          byte blocks = 0;
          if (map(RedPrint, RedMinPrint, RedPeakPrint, 0, ANSIMax) != -1)
          {
            blocks = map(RedPrint, RedMinPrint, RedPeakPrint, 0, ANSIMax);
          }
          PrintBlocks(blocks);
          if (RangeErrorRed == 1 && FullDebug == 1){Serial.println("RangeError"); RangeErrorRed = 0;}
          if (FullDebug == 1)
          {
            Serial.println(RedPrint);
            Serial.print("Min:");
            Serial.println(RedMinPrint);
            Serial.print("Pk:");
            Serial.println(RedPeakPrint);
          }
          SerialColorGreen();
          if (map(GreenPrint, GreenMinPrint, GreenPeakPrint, 0, ANSIMax) != -1)
          {
            blocks = map(GreenPrint, GreenMinPrint, GreenPeakPrint, 0, ANSIMax);
          }
          PrintBlocks(blocks);
          if (RangeErrorGreen == 1){Serial.println("RangeError"); RangeErrorGreen = 0;}
          if (FullDebug == 1)
          {
            Serial.println(GreenPrint);
            Serial.print("Min:");
            Serial.println(GreenMinPrint);
            Serial.print("Pk:");
            Serial.println(GreenPeakPrint);
          }
          SerialColorCyan();
          if (map(BluePrint, BlueMinPrint, BluePeakPrint, 0, ANSIMax) != -1)
          {
            blocks = map(BluePrint, BlueMinPrint, BluePeakPrint, 0, ANSIMax);
          }
          PrintBlocks(blocks);
          if (RangeErrorBlue == 1){Serial.println("RangeError"); RangeErrorBlue = 0;}
          if (FullDebug == 1)
          {
            Serial.println(BluePrint);
            Serial.print("Min:");
            Serial.println(BlueMinPrint);
            Serial.print("Pk:");
            Serial.println(BluePeakPrint);
          }
          else {Serial.println();}
          
          if (FullDebug == 1)
          {
            SerialColorWhite();
            Serial.print("Btn:");
            Serial.print(ButtonActive);
            Serial.print(" "); 
            Serial.println(ButtonHandled);
            Serial.print("Pwr:"); 
            Serial.println(PowerOn);
            Serial.print("Mode:");
            Serial.println(Mode);
            if (Mode == 1 || Mode == 2)
            {
              Serial.print("ModeCounter:");
              Serial.println(ModeCounter);
              Serial.print("WashSpeed:");
              Serial.println(ColorWashSpeed);
            }
            if (Mode == 2)
            {
              Serial.print("Mode2Speeds:");
              SerialColorRed();
              Serial.print(Mode2WashSpeedRed);
              Serial.print(" ");
              SerialColorGreen();
              Serial.print(Mode2WashSpeedGreen);
              Serial.print(" ");
              SerialColorCyan();
              Serial.println(Mode2WashSpeedBlue);
              SerialColorWhite();
            }
            Serial.print("Print:");
            TimeNow = micros();
            Serial.print(TimeNow - TimePrintStart);
            Serial.println("µS");
          }
          Serial.end(); // make sure serial interrupts are off
          for (byte Index = 0; Index < (NumOfFreqBins/2); Index++)
          {
            PrintingArray[Index] = 0;
            PrintingMinArray[Index] = 65535;
            PrintingPeakArray[Index] = 0;
          }
          TimeNow = micros();
          TimeError = (TimeNow - TimeExitPrint) - PrintInterval;
          TimeExitPrint = TimeNow;
          TimeLastPrintStart = TimePrintStart;
        }
      }
    }
    else
    {
      
      if (BadSample == 1)
      {
        Serial.begin(SerialBuad);
        Serial.println(BadSample);
        Serial.end();
        BadSample = 0;
      }
    }
    #endif
  }
  }
}

unsigned int ArrayRedParser(unsigned int Array[])
{
  unsigned int RedParser = ((Array[0]) + Array[1] + (Array[2] * 0.65));
  return RedParser;
}

unsigned int ArrayGreenParser(unsigned int Array[])
{
  unsigned int GreenParser = ((Array[2] * 0.35) + Array[3] + (Array[4] * 0.75) + (Array[5]) * 0.25);
  return GreenParser;
}

unsigned int ArrayBlueParser(unsigned int Array[])
{
  unsigned int BlueParser = ((Array[4] * 0.25) + (Array[5] * 0.75));
  for (byte BlueIndex = 6; BlueIndex < (NumOfFreqBins/2); BlueIndex++){BlueParser = (BlueParser + Array[BlueIndex]);}
  return BlueParser;
}

#ifdef Debug
void PrintChart()
{
  for (int Index = ((NumOfFreqBins/2) - 1); Index > -1; Index--) // print array backwards so low freq is at bottom
  {
    if (map(PrintingArray[Index], PrintingMinArray[Index], PrintingPeakArray[Index], 0, ANSIMax) > ANSIMax) // checking for an error, shouldn't happen again
    {
      while(1)  // sit-n-spin on error so it's the last thing printed
      {
        Serial.println(PrintingArray[Index]);
        Serial.println(PrintingMinArray[Index]);
        Serial.println(PrintingPeakArray[Index]);
        Serial.println(map(PrintingArray[Index], PrintingMinArray[Index], PrintingPeakArray[Index], 0, ANSIMax));
        Serial.println();
        delay(5000);
      }
    }
    if (FoundPeakArray[Index] == 1 && FoundMinArray[Index] == 0)
    {
      if (FoundPeakArray[(Index + 1)] != 1){SerialColorCyan();}
    }
    if (FoundMinArray[Index] == 1 && FoundPeakArray[Index] == 0)
    {
      if (FoundMinArray[(Index + 1)] != 1){SerialColorBlue();}
    }
    if (FoundPeakArray[Index] == 1 && FoundMinArray[Index] == 1){SerialColorPurple();}
    if ((FoundMinArray[(Index + 1)] == 1 || FoundPeakArray[(Index + 1)] == 1) && FoundPeakArray[Index] == 0 && FoundMinArray[Index] == 0)
    {
      SerialColorWhite();
    }
    unsigned int PrintPeakArrayFiltered = 35;
    if (PrintingPeakArray[Index] > PrintPeakArrayFiltered){PrintPeakArrayFiltered = PrintingPeakArray[Index];}
    byte blocks = 0;
    if (map(PrintingArray[Index], PrintingMinArray[Index], PrintPeakArrayFiltered, 0, ANSIMax) != -1)
    {
      blocks = map(PrintingArray[Index], PrintingMinArray[Index], PrintPeakArrayFiltered, 0, ANSIMax);
    }
    PrintBlocks(blocks);
  }
  for (byte Index = 0; Index < (NumOfFreqBins/2); Index++)
  {
    FoundPeakArray[Index] = 0;
    FoundMinArray[Index] = 0;
  }
}
#endif

void CheckRemote()
{
  ButtonDecay++;
  if (irrecv.decode(&results))
  {
    unsigned long hash = decodeHash(&results);
    // Hashes for Philips TV remote, sometimes get a third code for some buttons
    if (hash == 0xC1D0902A || hash == 0x55EF31B6)  // Zero
    {
      ButtonActive = "Zero";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x56EF334B || hash == 0xC2D091BF) // One
    {
      ButtonActive = "One";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x17313DCE || hash == 0x528A5222) // Two
    {
      ButtonActive = "Two";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x50216988 || hash == 0xE4400B14) // Three
    {
      ButtonActive = "Three";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x321579D4 || hash == 0x6D6E8E28) // Four
    {
      ButtonActive = "Four";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x6E6E8FBB || hash == 0x33157B67) // Five
    {
      ButtonActive = "Five";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0xA41CE90D || hash == 0xDF75FD61 || hash == 0xB1C92A9E) // Six
    {
      ButtonActive = "Six";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0xFE084450 || hash == 0x69E9A2C4) // Seven
    {
      ButtonActive = "Seven";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x593503E0 || hash == 0x1DDBEF8C) // Eight
    {
      ButtonActive = "Eight";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x5634FF29 || hash == 0x1ADBEAD5) // Nine
    {
      ButtonActive = "Nine";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x4025F0FC || hash == 0xEE4F9030) // Prev Ch 
    {
      ButtonActive = "PrevCh";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x5BD246F || hash == 0xDB00F6FE || hash == 0x5BD246F) // Period
    {
      ButtonActive = "Period";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x1AB4A6E1 || hash == 0x560DBB35) // Mute
    {
      ButtonActive = "Mute";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x970EB328 || hash == 0x5BB59ED4) // Volume Plus
    {
      ButtonActive = "VolPlus";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x940EAE71 || hash == 0x58B59A1D) // Volume Minus
    {
      ButtonActive = "VolMinus";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x967BB80C || hash == 0xD1D4CC60) // Channel Plus
    {
      ButtonActive = "ChanPlus";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0xCED4C7A9 || hash == 0x937BB355) // Channel Minus
    {
      ButtonActive = "ChanMinus";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0xBFB8F2FE || hash == 0xD9F9700B) // Menu
    {
      ButtonActive = "Menu";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0xD6D52168 || hash == 0x6AF3C2F4) // Info
    {
      ButtonActive = "Info";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0xB411F6DE || hash == 0xDD53082F) // Ok
    {
      ButtonActive = "Ok";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x337A78DC || hash == 0x7B3AC669) // Option
    {
      ButtonActive = "Option";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x9F4C4CEA || hash == 0xA76C95D6) // Mode
    {
      ButtonActive = "Mode";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x1C102884 || hash == 0x4B995E59) // Red
    {
      ButtonActive = "Red";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0xE050BDE6 || hash == 0xEF4EDF53) // Green
    {
      ButtonActive = "Green";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0xDF50BC53 || hash == 0xF04EE0E6) // Yellow
    {
      ButtonActive = "Yellow";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x8533B77 || hash == 0x5F564B6A) // Blue
    {
      ButtonActive = "Blue";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x7BA067FF || hash == 0xCD76C8CB) // Sleep
    {
      ButtonActive = "Sleep";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0xD7B29883 || hash == 0x52D50A22) // Control
    {
      ButtonActive = "Control";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x8C3A8EF3 || hash == 0x54838C7A) // Fav
    {
      ButtonActive = "Fav";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0xF280BC5B || hash == 0xAD8C62D2) // Format
    {
      ButtonActive = "Format";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0xD0264A08 || hash == 0x94CD35B4) // Source
    {
      ButtonActive = "Source";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x530DB67C || hash == 0x17B4A228) // Power
    {
      ButtonActive = "Power";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0xAC516266 || hash == 0xE5139CA7) // Up
    {
      ButtonActive = "Up";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0xAD5163FB || hash == 0xE4139B12) // Down
    {
      ButtonActive = "Down";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0xDB9D3097 || hash == 0xBE15326E) // Left
    {
      ButtonActive = "Left";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else if (hash == 0x9FA96F || hash == 0x9912B99A) // Right
    {
      ButtonActive = "Right";
      ButtonDecay = 0;
      if (ButtonHandled == 1){ButtonHandled = 0;}
    }
    else
    {
      //Serial.println(decodeHash(&results), HEX); // Print what doesn't match
    }
    irrecv.resume(); // Resume decoding (necessary!)
  }
  if (ButtonActive != "None")
  {
    if (ButtonDecay == ButtonDecayMax){ButtonActive = "None"; ButtonDecay = 0; ButtonHandled = 1;}
  }
}

void SerialColorBlack()
{
  Serial.write(27);       // ESC command
  Serial.print("[30m");   // Change color to Black
}

void SerialColorRed()
{
  Serial.write(27);       // ESC command
  Serial.print("[31m");   // Change color to Red
}

void SerialColorGreen()
{
  Serial.write(27);     // ESC command
  Serial.print("[32m"); // Change color to Green
}

void SerialColorYellow()
{
  Serial.write(27);       // ESC command
  Serial.print("[33m");   // Change color to Yellow
}

void SerialColorBlue()
{
  Serial.write(27);     // ESC command
  Serial.print("[34m"); // Change color to Blue
}

void SerialColorPurple()
{
  Serial.write(27);       // ESC command
  Serial.print("[35m");   // Change color to Purple
}

void SerialColorCyan()
{
  Serial.write(27);        // ESC command
  Serial.print("[36m");    // Change color to Cyan
}

void SerialColorWhite()
{
  Serial.write(27);       // ESC command
  Serial.print("[37m");   // Change color to White
}

void SerialCursorHome()
{
  Serial.write(27);       // ESC command
  Serial.print("[H");     // cursor to home command
}

void SerialClearScreen()
{
  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
}

byte PrintBlocks(byte blocks)
{
  byte blockcount = blocks;
  Serial.print("▌");
  for (blocks; blocks > 0; blocks--)
  {
    if (blocks == 1 && blockcount % 2 != 0)
    {
      Serial.print("▌");
    }
    else if (blocks % 2 == 0)
    {
      Serial.print("█");
    }
  }
  Serial.println();
}

void ResetLEDValues()
{
  #ifdef Debug
    PeakDeltaBad = 0;
    PeakDeltaGood = 0;
  #endif
  RedPeak = 0;
  GreenPeak = 0;
  BluePeak = 0;
  RedMin = 65535; // set to max so a new min is always found the first time
  GreenMin = 65535;
  BlueMin = 65535;
  for (byte Index = 0; Index < (NumOfFreqBins/2); Index++)
  {
    PeakArray[Index] = 0;
    MinArray[Index] = 65535;
    #ifdef Debug
    PrintingArray[Index] = 0;
    PrintingMinArray[Index] = 65535;
    PrintingPeakArray[Index] = 0;
    #endif
  }
}

void CheckEEPROM()
{
  if (EEPROM.read(0) == 1) // mode EEPROM write finish?
  {
    Mode = EEPROM.read(1); // set the mode to the last used mode
  }
  else // power went out while writing, assume it's trash and set a new sane default
  {
    EEPROM.write(1, 0); // set the mode to default
    EEPROM.write(0, 1); // write complete, data good
  }
  if (EEPROM.read(2) == 1) // color EEPROM write finish?
  {
    ColorWashSpeed = EEPROM.read(3); // set the color speed to the last used mode
  }
  else // power went out while writing, assume it's trash and set a new sane default
  {
    ColorWashSpeed = 10;
    EEPROM.write(3, 10); // set the color speed to default
    EEPROM.write(2, 1); // write complete, data good
  }
  
  if (EEPROM.read(4) == 1) // Power on EEPROM write finish?
  {
    PowerOn = EEPROM.read(5); // set power on to the last used mode
  }
  else // power went out while writing, assume it's trash and set a new sane default
  {
    EEPROM.write(5, 1); // set power on to default
    EEPROM.write(4, 1); // write complete, data good
  }

  #ifdef Debug
    if (EEPROM.read(256) == 1) // Full debug EEPROM write finish?
    {
      FullDebug = EEPROM.read(255); // set Full debug to the last used mode
    }
    else // power went out while writing, assume it's trash and set a new sane default
    {
      EEPROM.write(255, 0); // set Full debug to default
      EEPROM.write(256, 1); // write complete, data good
    }
  #endif
}

int ReadADC()
{
  noInterrupts();           // get lots more nasty pops/error on the measurements without doing this
  while(!(ADCSRA & 0x10));  // wait for adc to be ready
  ADCSRA = ADCReset;        // reset the adc
  int Output = ADCW;        // read the adc
  interrupts();
  return Output;
}
