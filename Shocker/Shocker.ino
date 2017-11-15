#include <SPI.h>
#include <Servo.h>

// Shocker 3
// For new board

// Constants
// Pin assignments
const int pinSpiSCK       = 13;
const int pinSpiMISO      = 12;
const int pinSpiMOSI      = 11;
const int pinSpiSS        = 10;

const int pinPinEvent3    = 2;
const int pinPinEvent2    = 9;
const int pinPinEvent1    = 8;
const int pinPinEvent0    = 7;

const int pinServoMeter   = 6;  // Max=10, Min=95

const int pinVibroLeft    = A0;
const int pinVibroRight   = A1;

const int pinRemoteIn     = 5;
const int pinFlipperLIn   = 4;
const int pinFlipperRIn   = 3;
const int pinFlipperLOut  = A2;
const int pinFlipperROut  = A3;
const int pinStrobe       = A4;
const int pinAir          = A5;
const int pinAdcAudioIn   = A6;

// IO expander pin assignments
// Bank A
const word bitLamp1         = 0x0080;
const word bitLamp2         = 0x0040;
const word bitLamp3         = 0x0020;
const word bitLamp4         = 0x0010;
const word bitLamp5         = 0x0008;
const word bitRelayVibro    = 0x0004;  // Vibro Handle relay
const word bitRelaySmoke    = 0x0002;  // Smoke
const word bitRelayShaker   = 0x0001;  // Shaker

// Bank B
const word bitLED2          = 0x0400;
const word bitLED1          = 0x0200;

// Bank B Remaining Outputs
const word bit7             = 0x8000;
const word bit6             = 0x4000;
const word bit5             = 0x2000;
const word bit4             = 0x1000;
const word bit3             = 0x0800;
const word bit0             = 0x0100;

// Application constants
const int nAttractThreshold = 5;
const int nLampDutyCycle = 1;

const int nServoMeterMax = 95;
const int nServoMeterMin = 10;

const byte bitHandleCtlLeft = 0x01;
const byte bitHandleCtlRight = 0x02;

// Attract mode data
const byte abyAttract[] = {   0x80, 0x40, 0x20, 0x10, 0x08, 0x10, 0x20, 0x40,
                              0x80, 0x40, 0x20, 0x10, 0x08, 0x10, 0x20, 0x40,
                              0x80, 0x40, 0x20, 0x10, 0x08, 0x10, 0x20, 0x40,
                              0x80, 0x40, 0x20, 0x10, 0x08, 0x10, 0x20, 0x40,
                              0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0x78, 0x38, 0x18, 0x08,
                              0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0x78, 0x38, 0x18, 0x08,
                              0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0x78, 0x38, 0x18, 0x08,
                              0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0x78, 0x38, 0x18, 0x08,
                              0x88, 0x50, 0x20, 0x50,
                              0x88, 0x50, 0x20, 0x50,
                              0x88, 0x50, 0x20, 0x50,
                              0x88, 0x50, 0x20, 0x50,
                              0x88, 0x50, 0x20, 0x50,
                              0x88, 0x50, 0x20, 0x50,
                              0x88, 0x50, 0x20, 0x50,
                              0x88, 0x50, 0x20, 0x50,
                              0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00};

// Globals
// Servo Devices
Servo gServoMeter;

// Relay devices
boolean gRelayVibro = false; 
boolean gRelaySmoke = false;
boolean gRelayShaker = false; 

// LEDs
boolean gLED1 = false ;
boolean gLED2 = false ;

// Test mode flag
boolean gInTest = false ;

// Game mode flag
boolean gInGame = false ;

// Flipper mode flag
boolean gFlipperMirror = false ;

// Event mode IDs
byte gbyEventId = 0xFF;
byte gbyEventIdQueue = 0xFF ;
byte gbyEventIdLast = 0xFF ;

//**************
//**************
// ** ARDUINO **
//**************
//**************
//----------------
// Function: Setup
//----------------
void setup() {
  // Serial Debug
  Serial.begin(9600);
  
  // Connect to servos
  gServoMeter.attach(pinServoMeter);

  // Set pin directions
  //Output
  pinMode (pinSpiSS, OUTPUT);
  pinMode (pinFlipperLOut, OUTPUT);
  pinMode (pinFlipperROut, OUTPUT);

  pinMode (pinVibroLeft, OUTPUT);
  pinMode (pinVibroRight, OUTPUT);
  pinMode (pinStrobe, OUTPUT);
  pinMode (pinAir, OUTPUT);

  // Input
  pinMode (pinRemoteIn, INPUT);
  pinMode (pinFlipperLIn, INPUT_PULLUP);
  pinMode (pinFlipperRIn, INPUT_PULLUP);
  pinMode (pinPinEvent0, INPUT_PULLUP);
  pinMode (pinPinEvent1, INPUT_PULLUP);
  pinMode (pinPinEvent2, INPUT_PULLUP);
  pinMode (pinPinEvent3, INPUT_PULLUP);

  // I/O Expander is control by SPI
  // Initialise SPI
  SPI.begin(); 

  // Set I/O direction to all Output
  digitalWrite(pinSpiSS,LOW);
  SPI.transfer(0x40); /* Write */
  SPI.transfer(0x00); /* IODIR */
  SPI.transfer(0x00); /* All output */
  SPI.transfer(0x00); /* All output */
  digitalWrite(pinSpiSS,HIGH); 

  // Reset the event id to the current input value
  gbyEventId = eventGetPinEventId();
  
  // Reset all outputs
  doResetOutput();
}

//---------------
// Function: Loop
//---------------
void loop() {
  static unsigned long lSample = 0 ;
  static int          adcMaxValue = 0;
  static int          adcMinValue = 1024;
  static int          anThreshold[5] = {0,0,0,0,0};
  static int          nCurAttract = 0 ;
  static boolean      bAttract = false ;
  static int          nServoMeterPos = nServoMeterMin;
  static int          nServoMeterPosInc = 1;
  
  word        wValue       = 0x0000;
  int         adcAudioValue;
  int         nRange        = adcMaxValue - adcMinValue,
              nInterval     = nRange / 5;
  
  //----------
  // Test Mode
  //----------
  if(gInTest) {
    // Currently in test mode
    doModeTest();
    return ;
  }
  
  /* Go into test mode? */
  if(Serial.available() > 0)
  {
    if(Serial.read() == '\n')
    {
      doResetOutput();
      doTestHelp();
      gInTest = true ;      
      return ;
    }
  }

  //-------
  // Events
  //-------
  // Poll for change in event id from DMD Extender
  eventDoPinEvent();
  if(gbyEventId == 0xFF)
  {
    // No event currently in progress
    if(gbyEventIdQueue != 0xFF) {
      // Queued event, start it
      Serial.print("Starting queued event ");
      Serial.println(gbyEventIdQueue);
      
      gbyEventId = gbyEventIdQueue;
      gbyEventIdQueue = 0xFF;
      eventPerform(true);
    }
  }
  else
  {
    // Keep the current event rolling
    if(!eventPerform(false))
    {
      // Finished the event
      gbyEventId = 0xFF;
    }
  }

  // Flipper ripple event?
  if(!eventFlipperRipple(false))
  {
    // Else move the flippers as required
    doFlippers(3000);
  }

  // Remote button handler
  if(gbyEventId == 0xFF)
  {
    // Remote button pressed?
    if(doRemoteBtn())
    {
      // Smoke on
      gRelaySmoke = true;
    }
    else
    {
      // Smoke off
      gRelaySmoke = false;
    }
  }
  
  //-----------------
  // Lights and meter
  //-----------------
  // Read the analogue port
  adcAudioValue = analogRead(pinAdcAudioIn);    

  // Every 20000 samples perform the recalibration
  if(lSample % 20000 == 0)
  {
    // Debug
    Serial.print(adcMinValue);
    Serial.print(" ");
    Serial.print(adcMaxValue);
    Serial.print(" ");
    Serial.print(nRange);
    Serial.print(" ");
    Serial.println(nInterval);

    if(nInterval > nAttractThreshold)
    {
      /* Calculate threshold spread */
      anThreshold[0] = adcMinValue + (1 * nInterval);
      anThreshold[1] = adcMinValue + (2 * nInterval);
      anThreshold[2] = adcMinValue + (2.5 * nInterval);
      anThreshold[3] = adcMinValue + (3 * nInterval);
      anThreshold[4] = adcMinValue + (3.5 * nInterval);
      
      // Threshold good for VU meter
      bAttract = false ;
    }
    else
    {
      // Sound too low so enter attract mode
      bAttract = true ;
    }
    
    /* Reset to start the samples */
    adcMinValue = 1024;
    adcMaxValue = 0;
  }
  else
  {
    /* Track the min and max values */
    adcMinValue = (adcMinValue > adcAudioValue) ? adcAudioValue : adcMinValue ;    
    adcMaxValue = (adcMaxValue < adcAudioValue) ? adcAudioValue : adcMaxValue ;    
  }  
  
  if (bAttract)
  {
    /* No sound so go into attract mode */
    wValue = abyAttract[nCurAttract];
    
    // Change the lights every 500 samples
    if((lSample % 500) == 0)
    {
      // Next sequence of lights
      nCurAttract++;
      
      // At the end of all sequences? Back to the beginning
      if(nCurAttract == (sizeof(abyAttract)/sizeof(abyAttract[0])))
        nCurAttract = 0;

      // Set meter servo
      gServoMeter.write(nServoMeterPos);

      // Move it up and down
      if(nServoMeterPos == nServoMeterMax)
      {
        nServoMeterPosInc = -1;
      }
      else
      if(nServoMeterPos == nServoMeterMin)
      {
        nServoMeterPosInc = 1;
      }
  
      nServoMeterPos += nServoMeterPosInc;
    } 
  }
  else
  {
    // Sound detected so respond
    if(adcAudioValue > anThreshold[0]) wValue |= bitLamp1 ;
    if(adcAudioValue > anThreshold[1]) wValue |= bitLamp2 ;
    if(adcAudioValue > anThreshold[2]) wValue |= bitLamp3 ;
    if(adcAudioValue > anThreshold[3]) wValue |= bitLamp4 ;
    if(adcAudioValue > anThreshold[4]) wValue |= bitLamp5 ;

    // Every 350 samples update the meter servo
    if(lSample % 350 == 0 )
    {
      int nServoPos;

      nServoPos = nServoMeterMax - (((adcAudioValue - adcMinValue) * (nServoMeterMax - nServoMeterMin) * 3) / nRange);
      nServoPos = constrain(nServoPos, nServoMeterMin, nServoMeterMax);
      gServoMeter.write(nServoPos);
    }
  }

  // Set duty cycle of the lamps to limit the current draw
  if(nLampDutyCycle > 0 && (lSample % nLampDutyCycle))
  {
    wValue = 0x0000;
  }
    
  // Set the relays based on the global variable flags
  wValue |= gRelayVibro ? bitRelayVibro : 0x0000 ;
  wValue |= gRelaySmoke ? bitRelaySmoke : 0x0000 ;
  wValue |= gRelayShaker ? bitRelayShaker : 0x0000 ;
  // Set the LEDs based on the global variable flags
  wValue |= gLED1 ? bitLED1 : 0x0000 ;
  wValue |= gLED2 ? bitLED2 : 0x0000 ;

  // Set outputs on the IO Expander
  spiSetIO(wValue);

  // Next sample
  lSample++;
}

//---------------------
// Function: doFlippers
//---------------------
void doFlippers(unsigned long timeFlipperMax)
{
  byte          byFlipperLIn,
                byFlipperRIn,
                byFlipperLOut,
                byFlipperROut;

  // Read the flipper button inputs
  byFlipperLIn = digitalRead(pinFlipperLIn);
  byFlipperRIn = digitalRead(pinFlipperRIn);

  if(gFlipperMirror)
  {
    // Mirror flippers
    byFlipperLOut = byFlipperRIn ;   
    byFlipperROut = byFlipperLIn ;   
  }
  else
  {
    // Normal flippers
    byFlipperLOut = byFlipperLIn ;   
    byFlipperROut = byFlipperRIn ;   
  }
  
  // Set the flippers
  // Inputs need inverting as they pulled high and active low
  digitalWrite(pinFlipperLOut, !byFlipperLOut);
  digitalWrite(pinFlipperROut, !byFlipperROut);
}

//----------------------
// Function: doRemoteBtn
//----------------------
boolean doRemoteBtn()
{
  static unsigned long  timeBtnPress = 0;

  boolean longHoldBtn = false ;
  
  if(digitalRead(pinRemoteIn) == HIGH && timeBtnPress == 0)
  {
    // Button just pressed so start timing
    timeBtnPress = millis();
  }
  else
  if(timeBtnPress > 0)
  {
    if(digitalRead(pinRemoteIn) == HIGH)
    {
      // Button pressed
      if(millis() - timeBtnPress > 100)
      {
        // For more than 100 milliseconds
        longHoldBtn = true;
      }
    }
    else
    {
      // Button no longer pressed, reset the timer
      timeBtnPress = 0;
    }
  }

  return longHoldBtn;
}

//-------------------
// Function: spiSetIO
//-------------------
void spiSetIO(word wValue)
{
  // Set I/O direction to all Output
  digitalWrite(pinSpiSS,LOW);
  SPI.transfer(0x40); /* Write */
  SPI.transfer(0x00); /* IODIR */
  SPI.transfer(0x00); /* All output */
  SPI.transfer(0x00); /* All output */
  digitalWrite(pinSpiSS,HIGH);

  // Set the outputs 
  digitalWrite(pinSpiSS,LOW); 
  SPI.transfer(0x40); /* Write */
  SPI.transfer(0x14); /* OLAT */  
  SPI.transfer(wValue & 0x00FF);
  SPI.transfer((wValue & 0xFF00) >> 8);
  digitalWrite(pinSpiSS,HIGH);   
}

//------------------------
// Function: doResetOutput
//------------------------
void doResetOutput()
{
  // Reset lamps and relays
  spiSetIO(0x0000);
  gRelayVibro = false; 
  gRelaySmoke = false; 
  gRelayShaker = false; 

  // LEDs off
  gLED1 = false ;
  gLED2 = false ;

  // Normal flippers
  gFlipperMirror = false ;
  
  // Servos to off positions
  gServoMeter.write(nServoMeterMax);

  // All outputs to low
  digitalWrite(pinSpiSS, LOW);
  digitalWrite(pinVibroLeft, LOW);
  digitalWrite(pinVibroRight, LOW);
  digitalWrite(pinFlipperLOut, LOW);
  digitalWrite(pinFlipperROut, LOW);
  digitalWrite(pinStrobe, LOW);
  digitalWrite(pinAir, LOW);
}

// **************
// **************
// *** EVENTS ***
// **************
// **************
//--------------------------
// Function: eventDoPinEvent
//--------------------------
void eventDoPinEvent()
{
  byte byPinEventId ;

  byPinEventId = eventGetPinEventId();
  
  // Event different from the last one seen?
  if(gbyEventIdLast != byPinEventId)
  {
    // Record the current event as the last one seen
    gbyEventIdLast = byPinEventId;

    // Diasable queuing
    //if(gbyEventId != 0xFF)
    //{
      // Event currently in progress so queue it (overwrite any other queued event)
    //  gbyEventIdQueue = byPinEventId;
    //}
    //else
    if(gbyEventId == 0xFF)
    {    
      switch(byPinEventId)
      {
        case 0x0F:
          Serial.println("Game Start");
          gInGame = true ;            
          break ;
    
        case 0x00:
          Serial.println("Game Over");
          gInGame = false ;
          break ;
    
        default:
          Serial.print("doPinEvent: ");
          Serial.println(byPinEventId);

          if(gInGame)
          {
            // Start the event processing
            gbyEventId = byPinEventId;
            eventPerform(true);
          }
          else
          {
            Serial.println("Not in game");
          }

          break ;
       }
    }
  }
}

//-----------------------------
// Function: eventGetPinEventId
//-----------------------------
byte eventGetPinEventId()
{
  byte byPinEventId ;

  // Read the event pins
  byPinEventId = digitalRead(pinPinEvent0);
  byPinEventId |= (digitalRead(pinPinEvent1) << 1);
  byPinEventId |= (digitalRead(pinPinEvent2) << 2);
  byPinEventId |= (digitalRead(pinPinEvent3) << 3);

  return byPinEventId;
}

//-----------------------
// Function: eventPerform
//-----------------------
boolean eventPerform(boolean bStart)
{
  boolean bRc ;

  if(bStart)
  {
    // Initialise the state of the outputs
    eventInit();
  }
  
  switch(gbyEventId)
  {
    case 0x01: // G + GR
      bRc = eventHandleVibrateLeft(bStart, 500);
      break;
      
    case 0x02: // GRE
      bRc = eventHandleVibrateRight(bStart, 500);
      break;

    case 0x03: // GREE
      bRc = eventHandleVibrateBoth(bStart, 500);
      break ;
            
    case 0x04: // GREED
      bRc = eventHandleVibrateBoth(bStart, 1000);
      break ;
            
    case 0x05: // Mansion Award
      // Mirror flippers
      bRc = eventFlipperMirror(bStart, 10000);
      bRc |= eventShaker(bStart, 2000);
      break;
      
    case 0x06: //Train Wreck
      // Smoke and both vibrate - short
      bRc = eventSmoke(bStart, 3000);
      bRc |= eventStrobe(bStart, 10000,10, 240);
      bRc |= eventHandleVibrateBoth(bStart, 1000);
      break ;
      
    case 0x07: // Bonus
      bRc = eventShaker(bStart, 2000);
      break ;

    case 0x08: // SHOW
      // Smoke and both vibrate - short
      bRc = eventSmoke(bStart, 3000);
      bRc |= eventStrobe(bStart, 10000,10, 240);
      bRc |= eventHandleVibrateBoth(bStart, 1000);
      break ;

    case 0x09: // MULTIBALL
      bRc = eventFlipperRipple(bStart);
      bRc |= eventHandleVibrateBoth(bStart, 1000);
      bRc |= eventShaker(bStart, 2000);
      break ;
    
    case 0x0A: // JACKPOT
    case 0x0B: // DOUBLE JACKPOT
      // Smoke and both vibrate - long
      bRc = eventSmoke(bStart, 3000);
      bRc |= eventStrobe(bStart, 10000,10, 240);
      bRc |= eventHandleVibrateBoth(bStart, 1000);
      break ;

    case 0x0C: // Ball Lock
      bRc = eventFlipperRipple(bStart);
      break ;

    case 0x0D: // REPLAY
      bRc = eventHandleVibrateBoth(bStart, 1000);
      break ;

    case 0x0E: // WELL PLAYED THING
      bRc = eventFlipperRipple(bStart);
      bRc |= eventShaker(bStart, 2000);
      break ;

    // Non-pinball instigated events
    case 0x10:
      // Left vibrate
      bRc = eventHandleVibrateLeft(bStart, 750);
      break ;
      
    case 0x11:
      // Right vibrate
      bRc = eventHandleVibrateRight(bStart, 750);
      break ;

    case 0x12:
      // Both vibrate
      bRc = eventHandleVibrateBoth(bStart, 750);
      break ;

    case 0x13:
      // Strobe
      bRc = eventStrobe(bStart, 10000, 10, 240);
      break ;

    default:
      bRc = false ;
      break ;
  }

  return bRc;
}

//----------
// eventInit
//----------
void eventInit()
{
  // Smoke off
  gRelaySmoke = false ;

  // Vibro handles off
  gRelayVibro = false ;
  digitalWrite(pinVibroLeft, 0);
  digitalWrite(pinVibroRight, 0);
}

//-------------------
// eventFlipperMirror
//-------------------
boolean eventFlipperMirror(boolean bStart, unsigned long timeDuration)
{
  static unsigned long timeStart = 0,
                        timeDurationThis = 0;

  boolean bRc = false ;
  unsigned long timeElapsed ;
  
  if(bStart)
  {
    Serial.println("eventFlipperMirror Start");
    timeStart = millis();
    timeDurationThis = timeDuration;
    gFlipperMirror = true ;
  }

  if(timeStart > 0)
  {
    timeElapsed = millis() - timeStart ;

    if(timeElapsed > timeDurationThis)
    {
      Serial.println("eventFlipperMirror End");
      timeStart = 0;
      gFlipperMirror = false ;
    }
    
    bRc = true ;
  }

  return bRc;
}

//-----------
// eventSmoke
//-----------
boolean eventSmoke(boolean bStart, unsigned long timeDuration)
{
  static unsigned long timeStart = 0,
                        timeDurationThis = 0;

  unsigned long timeElapsed ;
  boolean bRc = false ;

  if(bStart)
  {
    Serial.println("eventSmoke Start");
    timeStart = millis();
    timeDurationThis = timeDuration ;
    gRelaySmoke = true ;
  }

  if(timeStart > 0)
  {
    timeElapsed = millis() - timeStart;
     
    if(timeElapsed > timeDurationThis)
    {
      Serial.println("eventSmoke End");
      timeStart = 0;
      gRelaySmoke = false ;
    }

    bRc = true ;
  }
  
  return bRc ;
}

//------------
// eventShaker
//------------
boolean eventShaker(boolean bStart, unsigned long timeDuration)
{
  static unsigned long timeStart = 0,
                        timeDurationThis = 0;

  unsigned long timeElapsed ;
  boolean bRc = false ;
  
  if(bStart)
  {
    Serial.println("eventShaker Start");
    timeStart = millis();
    timeDurationThis = timeDuration ;
    gRelayShaker = true ;
  }

  if(timeStart > 0)
  {
    timeElapsed = millis() - timeStart;
     
    if(timeElapsed > timeDurationThis)
    {
      Serial.println("eventShaker End");
      timeStart = 0;
      gRelayShaker = false ;
    }

    bRc = true ;
  }
  
  return bRc ;
}

//-------------------
// eventFlipperRipple
//-------------------
boolean eventFlipperRipple(boolean bStart)
{
  static unsigned long  timeStart = 0;
  
  unsigned long         timeElapsed ;
  byte                  byFlipperLOut,
                        byFlipperROut;
  boolean               bRc = false ;
  
  if(bStart)
  {
    Serial.println("eventFlipperRipple start");
    timeStart = millis();
  }

  if(timeStart > 0)
  {
    timeElapsed = millis() - timeStart;

    if(timeElapsed > 700)
    {
      Serial.println("eventFlipperRipple End");
      timeStart = 0;
      byFlipperLOut = 0x00;
      byFlipperROut = 0x00;
    }
    else
    if(timeElapsed > 600)
    {
      byFlipperLOut = 0x00;
      byFlipperROut = 0x01;
    }
    else
    if(timeElapsed > 500)
    {
      byFlipperLOut = 0x01;
      byFlipperROut = 0x01;
    }
    else
    if(timeElapsed > 400)
    {
      byFlipperLOut = 0x01;
      byFlipperROut = 0x00;
    }
    else
    if(timeElapsed > 300)
    {
      byFlipperLOut = 0x00;
      byFlipperROut = 0x00;
    }
    else
    if(timeElapsed > 200)
    {
      byFlipperLOut = 0x00;
      byFlipperROut = 0x01;
    }
    else
    if(timeElapsed > 100)
    {
      byFlipperLOut = 0x01;
      byFlipperROut = 0x01;
    }
    else
    {
      byFlipperLOut = 0x01;
      byFlipperROut = 0x00;
    }
  
    digitalWrite(pinFlipperLOut, byFlipperLOut);
    digitalWrite(pinFlipperROut, byFlipperROut);

    bRc = true ;
  }

  return bRc;
}

//-----------------------
// eventHandleVibrateLeft
//-----------------------
boolean eventHandleVibrateLeft(boolean bStart, unsigned long timeDuration)
{
  return eventHandleVibrate(bStart, timeDuration, bitHandleCtlLeft);
}

//------------------------
// eventHandleVibrateRight
//------------------------
boolean eventHandleVibrateRight(boolean bStart, unsigned long timeDuration)
{
  return eventHandleVibrate(bStart, timeDuration, bitHandleCtlRight);
}

//-----------------------
// eventHandleVibrateBoth
//-----------------------
boolean eventHandleVibrateBoth(boolean bStart, unsigned long timeDuration)
{
  return eventHandleVibrate(bStart, timeDuration, bitHandleCtlLeft | bitHandleCtlRight);
}

//-------------------
// eventHandleVibrate
//-------------------
boolean eventHandleVibrate(boolean bStart, unsigned long timeDuration, byte byHandleCtl)
{
  static unsigned long timeStart = 0,
                        timeDurationThis ;

  unsigned long timeElapsed ;
  boolean       bRc = false ;

  if(bStart)
  {
    Serial.println("eventHandleVibrate Start");
    timeStart = millis();
    timeDurationThis = timeDuration ;
    
    if(byHandleCtl & bitHandleCtlLeft)
    {
      digitalWrite(pinVibroLeft, 1);
    }
    else
    {
      digitalWrite(pinVibroLeft, 0);
    }

    if(byHandleCtl & bitHandleCtlRight)
    {
      digitalWrite(pinVibroRight, 1);      
    }
    else
    {
      digitalWrite(pinVibroRight, 0);
    }
      
    gRelayVibro = true ;
  }
  
  if(timeStart > 0)
  {
    timeElapsed = millis() - timeStart;
    
    if(timeElapsed > timeDurationThis)
    {
      Serial.println("eventHandleVibrate End");
      timeStart = 0;

      // Make sure both handles are off
      digitalWrite(pinVibroLeft, 0);
      digitalWrite(pinVibroRight, 0);
      // Relay off
      gRelayVibro = false ;
    }
    
    bRc = true ;
  }

  return bRc ;
}

//------------
// eventStrobe
//------------
boolean eventStrobe(boolean bStart, unsigned long timeDuration, unsigned long timeMark, unsigned long timeSpace)
{
  static unsigned long timeStart = 0,
                        timeDurationThis = 0,
                        timeMarkThis = 0,
                        timeSpaceThis = 0,
                        cycles = 0;

  unsigned long timeElapsed ;
  boolean bRc = false ;
  
  if(bStart)
  {
    Serial.println("eventStrobe Start");
    timeStart = millis();
    timeDurationThis = timeDuration ;
    timeMarkThis = timeMark;
    timeSpaceThis = timeSpace;
    cycles = 0;
  }

  if(timeStart > 0)
  {
    timeElapsed = millis() - timeStart;
     
    if(timeElapsed > timeDurationThis)
    {
      Serial.println("eventStrobe End");
      digitalWrite(pinStrobe, LOW);
      gLED1 = false;
      gLED2 = false;

      timeStart = 0;
    }
    else
    if(timeElapsed > ((cycles *(timeMarkThis + timeSpaceThis)) + timeMarkThis + timeSpaceThis))
    {
      cycles++;
    }
    else
    if(timeElapsed > ((cycles *(timeMarkThis + timeSpaceThis)) + timeMarkThis))
    {
      digitalWrite(pinStrobe, LOW);
      gLED1 = false;
      gLED2 = false;
    }
    else
    {
      digitalWrite(pinStrobe, HIGH);    
      gLED1 = true;
      gLED2 = true;
    }
    
    bRc = true ;
  }
  
  return bRc ;
}

//---------
// eventAir
//---------
boolean eventAir(boolean bStart, unsigned long timeDuration)
{
  static unsigned long timeStart = 0,
                        timeDurationThis = 0,
                        timeChange = 0;
  static bool airOn = false;
                        
  unsigned long timeElapsed,
                timeNow = millis();
  boolean bRc = false ;
  
  if(bStart)
  {
    Serial.println("eventAir Start");
    
    timeStart = timeNow;
    timeDurationThis = timeDuration ;

    timeChange = timeNow;
    airOn = false;
  }

  if(timeStart > 0)
  {
    timeElapsed = timeNow - timeStart;
     
    if(timeElapsed > timeDurationThis)
    {
      Serial.println("eventAir End");

      // Switch air OFF and finish
      digitalWrite(pinAir, LOW);
      timeStart = 0;
    }
    else
    if(airOn && (timeChange - timeNow) > 50) // On time
    {
      // Switch air OFF
      digitalWrite(pinAir, LOW);
      timeChange = timeNow;
      airOn = false;  
    }
    else
    if(!airOn && (timeChange - timeNow) > 250) // Off time
    {
      // Switch air ON
      digitalWrite(pinAir, HIGH);
      timeChange = timeNow;
      airOn = true;  
    }
    
    bRc = true ;
  }
  
  return bRc ;
}

// *****************
// *****************
// *** TEST MODE ***
// *****************
// *****************
//---------------------
// Function: doModeTest
//---------------------
void doModeTest()
{
  static unsigned long  lSample = 0 ;
  static word           wValue = 0x0000 ;

  char                  cDeviceType = '\0';
  char                  cDeviceId = '\0';
  char                  cEquals;
  unsigned int          nDeviceState ;
  word                  wBit = 0x0000;

  if(Serial.available() > 0)
  {
    Serial.readBytes(&cDeviceType, 1);
    if(cDeviceType >= 'a')
    {
      cDeviceType -= 0x20;
    }
    
    if(cDeviceType == '\n')
    {
      Serial.println("Quitting Test Mode");

      // Reset all outputs
      doResetOutput();
      
      // Quit test mode
      gInTest = false;
      goto EXIT;
    }

    if(cDeviceType != 'I' && cDeviceType != '?')
    {
      Serial.readBytes(&cDeviceId, 1);
      if(cDeviceId >= 'a')
      {
        cDeviceId -= 0x20;
      }
      
      Serial.readBytes(&cEquals, 1);
      if( cEquals != '=')
      {
         goto EXIT;
      }
      nDeviceState = Serial.parseInt();
      Serial.readStringUntil('\n');
    }
    else
    {
      Serial.readStringUntil('\n');
    }
  }
    
  switch(cDeviceType)
  {
    case 'L':
      Serial.println("Lamp");
      switch(cDeviceId)
      {
        case '1':
          wBit = bitLamp1 ;
          break ;  

        case '2':
          wBit = bitLamp2 ;
          break ;  
          
        case '3':
          wBit = bitLamp3 ;
          break ;  

        case '4':
          wBit = bitLamp4 ;
          break ;  

        case '5':
          wBit = bitLamp5 ;
          break ;

        case 'A':
          wBit = bitLED1;
          break ;

        case 'B':
          wBit = bitLED2 ;
          break ;

        default:
          wBit = 0x0000;
          break ;
      }

      // Set / Reset the bit
      if(nDeviceState)
      {
        wValue |= wBit;
      }
      else
      {
        wValue &= ~wBit;
      }
      break ;
          
    case 'R':
      Serial.println("Relay");
      if(cDeviceId == '4')
      {
        // Output driven relay - Air
        digitalWrite(pinAir, nDeviceState ? HIGH : LOW);
      }
      else
      {
        // IO Expander Relays
        switch(cDeviceId)
        {
          case '1':
            wBit = bitRelayShaker ;
            break ;  
  
          case '2':
            wBit = bitRelaySmoke ;
            break ;  
            
          case '3':
            wBit = bitRelayVibro ;
            break ;  
  
          default:
            wBit = 0x0000;
            break ;
        }
  
        // Set / Reset the bit
        if(nDeviceState) {
          wValue |= wBit;
        }
        else {
          wValue &= ~wBit;
        }
      }

      break ;

    case 'S':
      Serial.println("Servo");
      if (cDeviceId == 'M') {
        gServoMeter.write(nDeviceState);
      }
      break ;

    case 'H':
      Serial.println("Handle");
      if(cDeviceId == 'L')
      {
        digitalWrite(pinVibroLeft, nDeviceState ? HIGH : LOW);
      }
      else
      if(cDeviceId == 'R')
      {
        digitalWrite(pinVibroRight, nDeviceState ? HIGH : LOW);
      }
      break ;

    case 'X':
      Serial.println("Strobe");
      if(cDeviceId == '1')
      {
        digitalWrite(pinStrobe, nDeviceState ? HIGH : LOW);
      }
      break ;

    case 'F':
      Serial.println("Flipper");
      if(cDeviceId == 'L')
      {
        digitalWrite(pinFlipperLOut, nDeviceState ? HIGH : LOW);
      }
      else
      if(cDeviceId == 'R')
      {
        digitalWrite(pinFlipperROut, nDeviceState ? HIGH : LOW);
      }
      break ;

    case 'E':
      if(gbyEventId == 0xFF)
      {
        Serial.print("Starting Test Event: ");
        Serial.println(nDeviceState);
        gbyEventId = (nDeviceState & 0xFF);
        eventPerform(true);
        gInTest = false ;
      }
      else
      {
        Serial.print("Event already in progress: ");
        Serial.println(gbyEventId);
      }
      break ;

    case 'I':
      Serial.println("Inputs");
      Serial.print("PE0=");
      Serial.print(digitalRead(pinPinEvent0));
      Serial.print(" PE1=");
      Serial.print(digitalRead(pinPinEvent1));
      Serial.print(" PE2=");
      Serial.print(digitalRead(pinPinEvent2));
      Serial.print(" PE3=");
      Serial.print(digitalRead(pinPinEvent3));
      Serial.print(" FL=");
      Serial.print(digitalRead(pinFlipperLIn));
      Serial.print(" FR=");
      Serial.print(digitalRead(pinFlipperRIn));
      Serial.print(" RMT=");
      Serial.println(digitalRead(pinRemoteIn));
      break ;

    case '?':
      doTestHelp();
      break ;
      
    default:
      break ;
  }

  // Set duty cycle of the lamps to limit the current draw
  if(nLampDutyCycle > 0 && (lSample % nLampDutyCycle))
  {
    // Set SPI output
    spiSetIO(wValue & (bitRelayVibro | bitRelaySmoke | bitRelayShaker | bitLED1 | bitLED2));
  }
  else
  {
    // Set SPI output
    spiSetIO(wValue);
//    Serial.println(wValue);
  }

EXIT:
  lSample++ ;
  
  return ;
}

//---------------------
// Function: doTestHelp
//---------------------
void doTestHelp()
{
  Serial.println("L1 - L5 : Lamps");
  Serial.println("LA / LB : LEDs");
  Serial.println("R1      : Shaker Relay");
  Serial.println("R2      : Smoke Relay");
  Serial.println("R3      : Vibro Relays");
  Serial.println("R4      : Air Relay");
  Serial.println("FL / FR : Flippers");
  Serial.println("SM      : Servo Meter");
  Serial.println("HL / HR : Vibrate Handle");
  Serial.println("X1      : Strobe");
  Serial.println("EV      : Events");
  Serial.println("I       : Display inputs");
  Serial.println("?       : Display help");
  Serial.println("Press Enter to quit Test Mode");
  Serial.println("");
}

// End of file
