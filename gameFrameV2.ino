#include <SPI.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Time.h>
#include <DS1307RTC.h>
#include <SdFat.h>
#include <IniFileLite.h>
#include <FastLED.h>
#include <Entropy.h>
#include <IRremote.h>

/***************************************************
  Game Frame V2 Source Code
  Jeremy Williams, 2-26-2016

  Game Frame is available at LEDSEQ.COM

  NOTE: Altering your firmware voids your warranty. Have fun.

  BMP parsing code based on example sketch for the Adafruit
  1.8" SPI display library by Adafruit.
  http://www.adafruit.com/products/358

  In the SD card, place 24 or 32 bit color BMP files
  (Alpha channel is ignored in 32-bit files.)

  There are examples included
 ****************************************************/

#define SD_CS    10  // Chip select line for SD card
SdFat sd; // set filesystem
SdFile myFile; // set filesystem

#define BUFFPIXEL 16 // number of pixels to buffer when reading BMP files

// Time
tmElements_t tm;

// IR setup
const byte RECV_PIN = 4;
IRrecv irrecv(RECV_PIN);
decode_results results;
char irCommand;
char irLastCommand;
boolean understood = false;

// LED setup
#define DATA_PIN 2
#define CLOCK_PIN 3
#define NUM_LEDS    256
#define LED_TYPE    SK9822
#define COLOR_ORDER BGR
CRGB leds[NUM_LEDS];

//Button setup
const uint8_t buttonNextPin = 16; // "Next" button
const uint8_t buttonMenuPin = 17;  // "Menu" button

#define STATUS_LED 6

//Enable verbose prints?
const boolean debugMode = false;

//Global variables
boolean
clockIniRead = false,
framePowered = true, // virtual screen power, used w/remote power button
irMenuRepeat = false,
irNextRepeat = false,
irPowerRepeat = false,
abc = false, // auto brightness control toggle
logoPlayed = false, // plays logo animation correctly reardless of playMode
folderLoop = true, // animation looping
moveLoop = false, // translation/pan looping
buttonPressed = false, // control button check
buttonEnabled = true, // debounce guard
menuActive = false, // showing menus (set brightness, playback mode, etc.)
panoff = true, // movement scrolls off screen
singleGraphic = false, // single BMP file
abortImage = false, // image is corrupt; abort, retry, fail?
displayFolderCount = false, // output extra info to LEDs
statusLedState = false, // flicker tech
clockShown = false, // clock mode?
clockSet = false, // have we set the time?
clockSetBlink = false, // flicker digit
clockDigitSet = false, // is hours/minutes set?
enableSecondHand = true,
clockAnimationActive = false, // currently showing clock anim
clockAdjustState = false, // is clock in an adjustment state?
hour12 = true, // 12-hour clock?
finishBeforeProgressing = false, // finish the animation before progressing?
timerLapsed = false, // timer lapsed, queue next animation when current one finishes
breakout = false, // breakout playing?
ballMoving = false,
gameInitialized = false;

byte
abcMinute, // last minute ABC was checked
playMode = 0, // 0 = sequential, 1 = random, 2 = pause animations
displayMode = 0, // 0 = slideshow, 1 = clock
brightness = 4, // LED brightness
brightnessMultiplier = 8, // DO NOT CHANGE THIS!
cycleTimeSetting = 2, // time before next animation: 1=10 secs, 2=30 secs, 3=1 min... 8=infinity
menuMode = 0, // 0 = brightmess, 1 = play mode, 2 = cycle time
clockAnimationLength = 5, // seconds to play clock animations
secondHandX = 0,
secondHandY = 0,
secondOffset = 0,
lastSecond = 255, // a moment ago
currentHour = 12,
currentMinute = 0,
currentSecond = 255, // current second
paddleIndex = 230,
ballX = 112,
ballY = 208,
ballIndex = 216;

int16_t 
folderIndex = 0; // current folder

int
secondCounter = 0, // counts up every second
fileIndex = 0, // current frame
chainIndex = -1, // for chaining multiple folders
numFolders = 0, // number of folders on sd
cycleTime = 30, // seconds to wait before progressing to next folder
clockAdjust = 0, // seconds to adjust clock every 24 hours
offsetBufferX = 0, // for storing offset when entering menu
offsetBufferY = 0, // for storing offset when entering menu
offsetSpeedX = 0, // number of pixels to translate each frame
offsetSpeedY = 0, // number of pixels to translate each frame
offsetX = 0, // for translating images x pixels
offsetY = 0, // for translating images y pixels
imageWidth = 0,
imageHeight = 0,
ballAngle;

unsigned long
prevRemoteMillis = 0, // last time a valid remote code was received
colorCorrection = 0xFFFFFF, // color correction setting
colorTemperature = 0xFFFFFF, // color Temperature
remoteCodeMenu = 0,
remoteCodeNext = 0,
remoteCodePower = 0,
menuPowerCounter = 0, // counter for holding menu button to turn off power
lastTime = 0, // used to calculate draw time
drawTime = 0, // time to read from sd
holdTime = 200, // millisecods to hold each .bmp frame
swapTime = 0, // system time to advance to next frame
baseTime = 0, // system time logged at start of each new image sequence
buttonTime = 0, // time the last button was pressed (debounce code)
menuEndTime = 0, // pause animation while in menu mode
menuEnterTime = 0; // time we enter menu

char
chainRootFolder[9], // chain game
nextFolder[21]; // dictated next animation

CRGB secondHandColor = 0; // color grabbed from digits.bmp for second hand

// globals for automatic brightness control
struct abc {
  int m; // minute
  byte b; // brightness
};

typedef struct abc Abc;
Abc abc0 { -1, 0};
Abc abc1 { -1, 0};
Abc abc2 { -1, 0};
Abc abc3 { -1, 0};
Abc abc4 { -1, 0};
Abc abc5 { -1, 0};
Abc abc6 { -1, 0};
Abc abc7 { -1, 0};
Abc abc8 { -1, 0};
Abc abc9 { -1, 0};

void setup(void) {
  // debug LED setup
  pinMode(STATUS_LED, OUTPUT);
  analogWrite(STATUS_LED, 100);

  pinMode(buttonNextPin, INPUT);    // button as input
  pinMode(buttonMenuPin, INPUT);    // button as input
  digitalWrite(buttonNextPin, HIGH); // turns on pull-up resistor after input
  digitalWrite(buttonMenuPin, HIGH); // turns on pull-up resistor after input

  // teensy LED flash
  pinMode(13, OUTPUT);
  teensyLEDFlash();
  teensyLEDFlash();
  teensyLEDFlash();

  Entropy.Initialize();

  if (debugMode == true)
  {
    Serial.begin(57600);
    delay(2000);
    Serial.println("Hello there!");
  }

  // init clock and begin counting seconds

  setSyncProvider(RTC.get);
  setSyncInterval(300);

  // DS3231 EEPROM check & DS1307 hardware check
  if (EEPROM.read(4) == 1) clockSet = true;
  if (timeStatus() != timeSet) clockSet = false;

  // IR enable
  irrecv.enableIRIn();

  byte output = 0;

  // load last settings
  // read brightness setting from EEPROM
  output = EEPROM.read(0);
  if (output >= 1 && output <= 7) brightness = output;

  // read playMode setting from EEPROM
  output = EEPROM.read(1);
  if (output >= 0 && output <= 2) playMode = output;

  // read cycleTimeSetting setting from EEPROM
  output = EEPROM.read(2);
  if (output >= 1 && output <= 8) cycleTimeSetting = output;
  setCycleTime();

  // read displayMode setting from EEPROM
  output = EEPROM.read(3);
  if (output >= 0 && output <= 1) displayMode = output;

  // LED Init
  FastLED.addLeds<LED_TYPE, DATA_PIN, CLOCK_PIN, COLOR_ORDER>(leds, NUM_LEDS).setDither(0);
  stripSetBrightness();
  clearStripBuffer();
  FastLED.show();

  // run burn in test if both tactile buttons held on cold boot
  if ((digitalRead(buttonNextPin) == LOW) && (digitalRead(buttonMenuPin) == LOW))
  {
    brightness = 7;
    stripSetBrightness();
    while (true)
    {
      testScreen();
    }
  }

  // revert to these values if MENU tactile button held on cold boot
  if (digitalRead(buttonMenuPin) == LOW)
  {
    brightness = 1;
    stripSetBrightness();
    playMode = 0;
    cycleTimeSetting = 2;
    setCycleTime();
    displayMode = 0;
  }

  // SD Init
  Serial.print("Init SD: ");
  if (!sd.begin(SD_CS, SPI_FULL_SPEED)) {
    Serial.println("fail");
    // SD error message
    sdErrorMessage();
    return;
  }
  Serial.println("OK!");

  // load automatic brightness control settings
  readABC();

  FastLED.setCorrection(colorCorrection);
  FastLED.setTemperature(colorTemperature);

  // load IR codes
  readRemoteIni();
  
  initGameFrame();
}

// This runs every time power is restored (cold or warm boot)
void initGameFrame()
{
  closeMyFile();
  // apply automatic brightness if enabled
  if (abc) applyCurrentABC();
  // reset vars
  displayFolderCount = false;
  logoPlayed = false;
  fileIndex = 0;
  offsetX = 0;
  offsetY = 0;
  folderIndex = 0;
  singleGraphic = false;
  clockAnimationActive = false;
  clockShown = false;
  nextFolder[0] = '\0';
  sd.chdir("/");

  // show test screens and folder count if NEXT tactile button held on boot
  if (digitalRead(buttonNextPin) == LOW)
  {
    displayFolderCount = true;
    testScreen();
  }

  char folder[13];

  if (numFolders == 0 || displayFolderCount)
  {
    numFolders = 0;
    // file indexes appear to loop after 2048
    for (int fileIndex = 0; fileIndex < 2048; fileIndex++)
    {
      myFile.open(sd.vwd(), fileIndex, O_READ);
      if (myFile.isDir()) {
        Serial.println("---");
        if (displayFolderCount == true)
        {
          leds[numFolders] = CRGB(128, 255, 0);
          FastLED.show();
        }
        numFolders++;
        Serial.print("File Index: ");
        Serial.println(fileIndex);
        myFile.getName(folder, 13);
        Serial.print("Folder: ");
        Serial.println(folder);
        closeMyFile();
      }
      else closeMyFile();
    }
    Serial.print(numFolders);
    Serial.println(" folders found.");
  }
  if (displayFolderCount == true)
  {
    delay(5000);
    irrecv.resume(); // Receive the next value
    remoteTest();
  }
  
  // play logo animation
  sd.chdir("/00system/logo");
  readIniFile();
  drawFrame();
}

void applyCurrentABC()
{
  getCurrentTime();
  int dayInMinutes = minuteCounter(currentHour, currentMinute);
  int newBrightness = -1;
  int triggerTimes[10] = {abc0.m, abc1.m, abc2.m, abc3.m, abc4.m, abc5.m, abc6.m, abc7.m, abc8.m, abc9.m};
  byte BrightnessSettings[10] = {abc0.b, abc1.b, abc2.b, abc3.b, abc4.b, abc5.b, abc6.b, abc7.b, abc8.b, abc9.b};
  for (byte x = 0; x < 9; x++)
  {
    if (dayInMinutes > triggerTimes[x] && triggerTimes[x] != -1) newBrightness = BrightnessSettings[x];
  }
  if (newBrightness == -1)
  {
    // current time is earlier than any triggers; use latest trigger before midnight
    newBrightness = BrightnessSettings[0];
    for (byte x = 1; x < 9; x++)
    {
      if (triggerTimes[x] > triggerTimes[x - 1]) newBrightness = BrightnessSettings[x];
    }
  }
  if (newBrightness > -1)
  {
    if (newBrightness == 0) newBrightness = 1;
    brightness = newBrightness;
    stripSetBrightness();
    saveSettingsToEEPROM();
  }
}

void stripSetBrightness()
{
  if (brightness > 7) brightness = 7;
  else if (brightness < 0) brightness = 0;
  FastLED.setBrightness(brightness * brightnessMultiplier);
  //  FastLED.setBrightness(2);
}

void remoteTest()
{
  clearStripBuffer();
  FastLED.show();
  sd.chdir("/00system");
  int graphicShown = 0;
  bmpDraw("irZapr.bmp", 0, 0);
  long nextIRCheck = 0;
  while (true)
  {
    irReceiver();
    if (irCommand == 'P' || irCommand == 'M' || irCommand == 'N' || irPowerRepeat || irMenuRepeat || irNextRepeat)
    {
      nextIRCheck = millis() + 125;
      if (irCommand == 'P' || irPowerRepeat && graphicShown != 1)
      {
        bmpDraw("irPowr.bmp", 0, 0);
        graphicShown = 1;
      }
      else if (irCommand == 'M' || irMenuRepeat && graphicShown != 2)
      {
        bmpDraw("irMenu.bmp", 0, 0);
        graphicShown = 2;
      }
      else if (irCommand == 'N' || irNextRepeat && graphicShown != 3)
      {
        bmpDraw("irNext.bmp", 0, 0);
        graphicShown = 3;
      }
    }
    // no remote button pressed
    if (millis() > nextIRCheck && graphicShown != 0)
    {
      bmpDraw("irZapr.bmp", 0, 0);
      graphicShown = 0;
    }
    // record new IR codes if NEXT tactile button is presssed
    if (digitalRead(buttonNextPin) == LOW)
    {
      recordIRCodes();
      bmpDraw("irZapr.bmp", 0, 0);
    }
    // exit if MENU tactile button is presssed
    if (digitalRead(buttonMenuPin) == LOW)
    {
      return;
    }
  }
}

void teensyLEDFlash()
{
  // teensy LED flash
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(100);               // wait for a second
}

// draw a red box around the screen to indicate recording active
void redBox()
{
  for (int i = 0; i < 15; i++)
  {
    leds[i] = CRGB(255, 0, 0);
  }
  for (int i = 0; i < 16; i++)
  {
    leds[getIndex(16, i)] = CRGB(255, 0, 0);
  }
  for (int i = 0; i < 15; i++)
  {
    leds[getIndex(i, 15)] = CRGB(255, 0, 0);
  }
  for (int i = 15; i > 0; i--)
  {
    leds[getIndex(0, i)] = CRGB(255, 0, 0);
  }
  FastLED.show();
}

void printIRCode()
{
  Serial.print("IR code: ");
  Serial.println(results.value, DEC);
}

void recordIRCodes()
{
  remoteCodeMenu = 0;
  remoteCodeNext = 0;
  remoteCodePower = 0;

  // record POWR
  bmpDraw("irPowr.bmp", 0, 0);
  redBox();
  while (remoteCodePower == 0)
  {
    if (irrecv.decode(&results)) {
      if (results.decode_type == NEC && results.value != 4294967295) // ignore repeat code (0xFFFFFFFF)
      {
        printIRCode();
        remoteCodePower = results.value;
      }
      irrecv.resume(); // Receive the next value
    }
  }

  // record MENU
  bmpDraw("irMenu.bmp", 0, 0);
  redBox();
  while (remoteCodeMenu == 0)
  {
    if (irrecv.decode(&results)) {
      if (results.decode_type == NEC && results.value != 4294967295 && results.value != remoteCodePower) // ignore repeat code (0xFFFFFFFF)
      {
        printIRCode();
        remoteCodeMenu = results.value;
      }
      irrecv.resume(); // Receive the next value
    }
  }

  // record NEXT
  bmpDraw("irNext.bmp", 0, 0);
  redBox();
  while (remoteCodeNext == 0)
  {
    if (irrecv.decode(&results)) {
      if (results.decode_type == NEC && results.value != 4294967295 && results.value != remoteCodePower && results.value != remoteCodeMenu) // ignore repeat code (0xFFFFFFFF)
      {
        printIRCode();
        remoteCodeNext = results.value;
      }
      irrecv.resume(); // Receive the next value
    }
  }

  // save codes to SD
  if (!myFile.open("remote.ini", O_CREAT | O_RDWR)) {
    Serial.println("File open failed");
    sdErrorMessage();
    return;
  }
  myFile.rewind();
  myFile.seekSet(352);
  myFile.println("[remote]");
  myFile.println("");
  myFile.println("# Learned Codes");
  myFile.print("power = ");
  myFile.println(remoteCodePower);
  myFile.print("menu = ");
  myFile.println(remoteCodeMenu);
  myFile.print("next = ");
  myFile.println(remoteCodeNext);
  closeMyFile();
}

void testScreen()
{
  // white
  for (int i = 0; i < 256; i++)
  {
    leds[i] = CRGB(255, 255, 255);
  }
  FastLED.show();
  delay(5000);

  // red
  for (int i = 0; i < 256; i++)
  {
    leds[i] = CRGB(255, 0, 0);
  }
  FastLED.show();
  delay(1000);

  // green
  for (int i = 0; i < 256; i++)
  {
    leds[i] = CRGB(0, 255, 0);
  }
  FastLED.show();
  delay(1000);

  // blue
  for (int i = 0; i < 256; i++)
  {
    leds[i] = CRGB(0, 0, 255);
  }
  FastLED.show();
  delay(1000);
}

void sdErrorMessage()
{
  // red bars
  for (int index = 64; index < 80; index++)
  {
    leds[index] = CRGB(255, 0, 0);
  }
  for (int index = 80; index < 192; index++)
  {
    leds[index] = CRGB(0, 0, 0);
  }
  for (int index = 192; index < 208; index++)
  {
    leds[index] = CRGB(255, 0, 0);
  }
  // S
  yellowDot(7, 6);
  yellowDot(6, 6);
  yellowDot(5, 6);
  yellowDot(4, 7);
  yellowDot(5, 8);
  yellowDot(6, 8);
  yellowDot(7, 9);
  yellowDot(6, 10);
  yellowDot(5, 10);
  yellowDot(4, 10);

  // D
  yellowDot(9, 6);
  yellowDot(10, 6);
  yellowDot(11, 7);
  yellowDot(11, 8);
  yellowDot(11, 9);
  yellowDot(10, 10);
  yellowDot(9, 10);
  yellowDot(9, 7);
  yellowDot(9, 8);
  yellowDot(9, 9);

  stripSetBrightness();
  FastLED.show();

  while (true)
  {
    for (int i = 255; i >= 0; i--)
    {
      analogWrite(STATUS_LED, i);
      printRemoteCode();
      delay(1);
    }
    for (int i = 0; i <= 254; i++)
    {
      analogWrite(STATUS_LED, i);
      printRemoteCode();
      delay(1);
    }
  }
}

void printRemoteCode()
{
  if (irrecv.decode(&results)) {
    printIRCode();
    irrecv.resume(); // Receive the next value
  }
}

void yellowDot(byte x, byte y)
{
  leds[getIndex(x, y)] = CRGB(255, 255, 0);
}

void setCycleTime()
{
  if (cycleTimeSetting == 2)
  {
    cycleTime = 30;
  }
  else if (cycleTimeSetting == 3)
  {
    cycleTime = 60;
  }
  else if (cycleTimeSetting == 4)
  {
    cycleTime = 300;
  }
  else if (cycleTimeSetting == 5)
  {
    cycleTime = 900;
  }
  else if (cycleTimeSetting == 6)
  {
    cycleTime = 1800;
  }
  else if (cycleTimeSetting == 7)
  {
    cycleTime = 3600;
  }
  else if (cycleTimeSetting == 8)
  {
    cycleTime = -1;
  }
  else
  {
    cycleTime = 10;
  }
}

void statusLedFlicker()
{
  if (statusLedState == false)
  {
    analogWrite(STATUS_LED, 200);
  }
  else
  {
    analogWrite(STATUS_LED, 255);
  }
  statusLedState = !statusLedState;
}

/* teensy 3 clock code */
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

/*  code to process time sync messages from the serial port   */
#define TIME_HEADER  "T"   // Header tag for serial time sync message

  unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    return pctime;
    if ( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
      pctime = 0L; // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}

void saveSettingsToEEPROM()
{
  // save any new settings to EEPROM
  EEPROM.update(0, brightness);
  EEPROM.update(1, playMode);
  EEPROM.update(2, cycleTimeSetting);
  EEPROM.update(3, displayMode);
}

void irReceiver()
{
  irMenuRepeat = false;
  irNextRepeat = false;
  irPowerRepeat = false;
  irCommand = 'Z';
  if (irrecv.decode(&results)) {
    Serial.print("IR code received: ");
    Serial.println(results.value);
    understood = true;
    // menu
    if (results.value == remoteCodeMenu)
    {
      irCommand = 'M';
      prevRemoteMillis = millis();
    }
    // next
    else if (results.value == remoteCodeNext)
    {
      irCommand = 'N';
      prevRemoteMillis = millis();
    }
    // power
    else if (results.value == remoteCodePower)
    {
      irCommand = 'P';
      prevRemoteMillis = millis();
    }
    // repeat/held
    else if (results.value == 4294967295 && millis() - prevRemoteMillis < 250) // require recent prime signal to assume repeat 
    {
      irCommand = 'R';
      prevRemoteMillis = millis();
      if (irLastCommand == 'M') irMenuRepeat = true;
      else if (irLastCommand == 'N') irNextRepeat = true;
      else if (irLastCommand == 'P') irPowerRepeat = true;
    }
    else understood = false;
    if (understood)
    {
      Serial.print("Interpreted as: ");
      Serial.println(irCommand);
      if (irCommand != 'R') irLastCommand = irCommand;
    }

    irrecv.resume(); // Receive the next value
  }
}

void powerControl()
{
  if (irCommand == 'P')
  {
    // abc brightness set to zero; turn display on
    if (brightness == 0)
    {
      brightness = 1;
      stripSetBrightness();
      if (displayMode == 0) drawFrame(); // refrech the screen
      else initClock();
    }
    // toggle system power
    else
    {
      framePowered = !framePowered;
      // power down
      if (!framePowered) framePowerDown();
      // power restored
      else initGameFrame();
    }
  }
  // allow power toggle by holding down physical Menu button on PCB
  if (framePowered)
  {
    if (millis() > menuPowerCounter && digitalRead(buttonMenuPin) == LOW && menuActive)
    {
      framePowered = false;
      framePowerDown();
    }
  }
  // reset MENU button after power off
  else if (digitalRead(buttonMenuPin) == HIGH && buttonPressed == true) buttonPressed = false;
  // power on
  else if (digitalRead(buttonMenuPin) == LOW && buttonPressed == false)
  {
    framePowered = true;
  }
}

void framePowerDown()
{
  menuActive = false;
  if (breakout == true)
  {
    breakout = false;
    ballMoving = false;
  }
  clearStripBuffer();
  FastLED.show();
}

void loop() {
  irReceiver();
  powerControl();
  getCurrentTime();
  clockDriftAdjustment();

  if (framePowered)
  {
    runABC();
    if (brightness > 0)
    {
      if (breakout == false)
      {
        mainLoop();
      }

      else
      {
        breakoutLoop();
        if (breakout == false)
        {
          if (displayMode == 0)
          {
            nextImage();
            drawFrame();
          }
          else initClock();
        }
      }
    }
  }
}

void mainLoop()
{
  buttonDebounce();

  // next button
  if ((digitalRead(buttonNextPin) == LOW || irCommand == 'N') && buttonPressed == false && buttonEnabled == true && !clockShown)
  {
    buttonPressed = true;
    if (menuActive == false)
    {
      // exit chaining if necessary
      if (chainIndex > -1)
      {
        chainIndex = -1;
        chainRootFolder[0] = '\0';
        sd.chdir("/");
      }
      if (displayMode == 0)
      {
        nextImage();
        drawFrame();
      }
      else if (displayMode == 1)
      {
        // just displayed logo, enter clock mode
        initClock();
        return;
      }
    }
    else
    {
      menuEndTime = millis() + 3000;

      // adjust brightness
      if (menuMode == 0)
      {
        brightness += 1;
        if (brightness > 7) brightness = 1;
        char brightChar[2];
        char brightFile[23];
        strcpy_P(brightFile, PSTR("/00system/bright_"));
        itoa(brightness, brightChar, 10);
        strcat(brightFile, brightChar);
        strcat(brightFile, ".bmp");
        stripSetBrightness();
        bmpDraw(brightFile, 0, 0);
      }

      // adjust play mode
      else if (menuMode == 1)
      {
        playMode++;
        if (playMode > 2) playMode = 0;
        char playChar[2];
        char playFile[21];
        strcpy_P(playFile, PSTR("/00system/play_"));
        itoa(playMode, playChar, 10);
        strcat(playFile, playChar);
        strcat(playFile, ".bmp");
        bmpDraw(playFile, 0, 0);
      }

      // adjust cycle time
      else if (menuMode == 2)
      {
        cycleTimeSetting++;
        if (cycleTimeSetting > 8) cycleTimeSetting = 1;
        setCycleTime();
        char timeChar[2];
        char timeFile[21];
        strcpy_P(timeFile, PSTR("/00system/time_"));
        itoa(cycleTimeSetting, timeChar, 10);
        strcat(timeFile, timeChar);
        strcat(timeFile, ".bmp");
        bmpDraw(timeFile, 0, 0);
      }

      // clock/gallery mode
      else if (menuMode == 3)
      {
        displayMode++;
        if (displayMode > 1) displayMode = 0;
        char modeChar[2];
        char modeFile[21];
        strcpy_P(modeFile, PSTR("/00system/mode_"));
        itoa(displayMode, modeChar, 10);
        strcat(modeFile, modeChar);
        strcat(modeFile, ".bmp");
        bmpDraw(modeFile, 0, 0);
      }

      // breakout time
      else if (menuMode == 4)
      {
        menuActive = false;
        saveSettingsToEEPROM();

        // return to brightness menu next time
        menuMode = 0;

        buttonTime = millis();
        breakout = true;
        gameInitialized = false;
        buttonEnabled = false;

        char tmp[23];
        strcpy_P(tmp, PSTR("/00system/breakout.bmp"));
        bmpDraw(tmp, 0, 0);

        paddleIndex = 230,
        ballX = 112,
        ballY = 208,
        ballIndex = 216;
        holdTime = 0;
        fileIndex = 0;
        leds[ballIndex] = CRGB(175, 255, 15);
        leds[paddleIndex] = CRGB(200, 200, 200);
        leds[paddleIndex + 1] = CRGB(200, 200, 200);
        leds[paddleIndex + 2] = CRGB(200, 200, 200);
        FastLED.show();
        return;
      }
    }
  }

  // reset clock
  if ((digitalRead(buttonNextPin) == LOW || irCommand == 'N') && buttonPressed == false && buttonEnabled == true && clockShown)
  {
    buttonPressed = true;
    clockSet = false;
    clockSetBlink = false;
    getCurrentTime();
    setClock();
  }

  // menu button
  else if ((digitalRead(buttonMenuPin) == LOW || irCommand == 'M') && buttonPressed == false && buttonEnabled == true)
  {
    buttonPressed = true;
    menuEndTime = millis() + 3000;
    menuPowerCounter = millis() + 1500;

    if (menuActive == false)
    {
      menuActive = true;
      menuEnterTime = millis();
      offsetBufferX = offsetX;
      offsetBufferY = offsetY;
      offsetX = 0;
      offsetY = 0;
      closeMyFile();
      if (clockShown || clockAnimationActive)
      {
        clockAnimationActive = false;
        clockShown = false;
        abortImage = true;
        nextFolder[0] = '\0';
      }
    }
    else
    {
      menuMode++;
      if (menuMode > 4) menuMode = 0;
    }
    if (menuMode == 0)
    {
      char brightChar[2];
      char brightFile[23];
      strcpy_P(brightFile, PSTR("/00system/bright_"));
      itoa(brightness, brightChar, 10);
      strcat(brightFile, brightChar);
      strcat(brightFile, ".bmp");
      bmpDraw(brightFile, 0, 0);
    }
    else if (menuMode == 1)
    {
      char playChar[2];
      char playFile[21];
      strcpy_P(playFile, PSTR("/00system/play_"));
      itoa(playMode, playChar, 10);
      strcat(playFile, playChar);
      strcat(playFile, ".bmp");
      bmpDraw(playFile, 0, 0);
    }
    else if (menuMode == 2)
    {
      char timeChar[2];
      char timeFile[21];
      strcpy_P(timeFile, PSTR("/00system/time_"));
      itoa(cycleTimeSetting, timeChar, 10);
      strcat(timeFile, timeChar);
      strcat(timeFile, ".bmp");
      bmpDraw(timeFile, 0, 0);
    }
    else if (menuMode == 3)
    {
      char modeChar[2];
      char modeFile[21];
      strcpy_P(modeFile, PSTR("/00system/mode_"));
      itoa(displayMode, modeChar, 10);
      strcat(modeFile, modeChar);
      strcat(modeFile, ".bmp");
      bmpDraw(modeFile, 0, 0);
    }
    else if (menuMode == 4)
    {
      char gameFile[21];
      strcpy_P(gameFile, PSTR("/00system/game.bmp"));
      bmpDraw(gameFile, 0, 0);
    }
  }

  // time to exit menu mode?
  if (menuActive == true)
  {
    if (millis() > menuEndTime)
    {
      if (displayMode == 1)
      {
        initClock();
        return;
      }

      menuActive = false;

      saveSettingsToEEPROM();

      // return to brightness menu next time
      menuMode = 0;

      offsetX = offsetBufferX;
      offsetY = offsetBufferY;
      if (playMode == 2)
      {
        offsetX = imageWidth / -2 + 8;
        offsetY = imageHeight / 2 - 8;
      }
      swapTime = swapTime + (millis() - menuEnterTime);
      baseTime = baseTime + (millis() - menuEnterTime);
      if (abortImage == false)
      {
        drawFrame();
      }
    }
  }

  // currently playing images?
  if (menuActive == false && breakout == false)
  {
    if (clockShown == false || clockAnimationActive == true)
    {
      // advance counter
      if (currentSecond != lastSecond)
      {
        lastSecond = currentSecond;
        secondCounter++;
        // revert to clock display if animation played for 5 seconds
        if (clockAnimationActive == true && secondCounter >= clockAnimationLength)
        {
          initClock();
        }
      }

      // did image load fail?
      if (abortImage == true && clockShown == false && logoPlayed == true)
      {
        abortImage = false;
        nextImage();
        drawFrame();
      }

      // progress to next folder if cycleTime is up
      // check for infinite mode
      else if (cycleTimeSetting != 8  && clockShown == false && clockAnimationActive == false)
      {
        if (secondCounter >= cycleTime)
        {
          if (finishBeforeProgressing == true)
          {
            if (timerLapsed == false) timerLapsed = true;
          }
          else
          {
            nextImage();
            drawFrame();
          }
        }
      }

      // animate if not a single-frame & animations are on
      if (holdTime != -1 && playMode != 2 || logoPlayed == false)
      {
        if (millis() >= swapTime && clockShown == false)
        {
          statusLedFlicker();
          swapTime = millis() + holdTime;
          fileIndex++;
          drawFrame();
        }
      }
    }

    // show clock
    else if (clockShown == true && clockAnimationActive == false)
    {
      showClock();
    }
  }
}

void nextImage()
{
  Serial.println(F("---"));
  Serial.println(F("Next Folder..."));
  closeMyFile();
  boolean foundNewFolder = false;
  // reset secondCounter if not playing clock animations
  if (!clockAnimationActive) secondCounter = 0;
  baseTime = millis();
  holdTime = 0;
  char folder[9];
  sd.chdir("/");
  fileIndex = 0;
  offsetX = 0;
  offsetY = 0;
  singleGraphic = false;
  finishBeforeProgressing = false;
  timerLapsed = false;
  if (!logoPlayed) logoPlayed = true;

  // are we chaining folders?
  if (chainIndex > -1)
  {
    char chainChar[6];
    char chainDir[23];
    strcpy_P(chainDir, PSTR("/"));
    strcat(chainDir, chainRootFolder);
    strcat(chainDir, "/");
    itoa(chainIndex, chainChar, 10);
    strcat(chainDir, chainChar);
    if (sd.exists(chainDir))
    {
      Serial.print(F("Chaining: "));
      Serial.println(chainDir);
      sd.chdir(chainDir);
      chainIndex++;
    }
    else
    {
      // chaining concluded
      chainIndex = -1;
      chainRootFolder[0] = '\0';
      sd.chdir("/");
    }
  }

  // has the next animation has been dictated by the previous .INI file?
  if (nextFolder[0] != '\0' && chainIndex == -1)
  {
    Serial.print(F("Forcing next: "));
    Serial.println(nextFolder);
    if (sd.exists(nextFolder))
    {
      sd.chdir(nextFolder);
    }
    else
    {
      nextFolder[0] = '\0';
      Serial.println(F("Not exists!"));
    }
  }

  // next folder not assigned by .INI
  if (nextFolder[0] == '\0' && chainIndex == -1)
  {
    // Getting next folder
    // shuffle playback using random number
    if (playMode != 0) // check we're not in a sequential play mode
    {
      int targetFolder = Entropy.random(0, numFolders);

      // don't repeat the same image, please.
      if (targetFolder <= 0 or targetFolder == numFolders or targetFolder == numFolders - 1)
      {
        // Repeat image detected! Incrementing targetFolder.
        targetFolder = targetFolder + 2;
      }

      Serial.print(F("Randomly advancing "));
      Serial.print(targetFolder);
      Serial.println(F(" folder(s)."));
      int i = 1;
      while (i < targetFolder)
      {
        foundNewFolder = false;
        while (foundNewFolder == false)
        {
          myFile.open(sd.vwd(), folderIndex, O_READ);
          if (myFile.isDir()) {
            foundNewFolder = true;
            i++;
          }
          closeMyFile();
          folderIndex++;
        }
      }
    }

    foundNewFolder = false;

    while (foundNewFolder == false)
    {
      myFile.open(sd.vwd(), folderIndex, O_READ);
      myFile.getName(folder, 13);

      // ignore system folders that start with "00"
      if (myFile.isDir() && folder[0] != 48 && folder[1] != 48) {
        foundNewFolder = true;
        Serial.print(F("Folder Index: "));
        Serial.println(folderIndex);
        Serial.print(F("Opening Folder: "));
        Serial.println(folder);

        sd.chdir(folder);
        closeMyFile();
      }
      else closeMyFile();
      folderIndex++;
    }
  }

  // is this the start of a folder chain?
  char chainDir[2];
  strcpy_P(chainDir, PSTR("0"));
  if (sd.exists(chainDir))
  {
    Serial.print(F("Chaining detected: "));
    Serial.println(folder);
    memcpy(chainRootFolder, folder, 8);
    sd.chdir(chainDir);
    chainIndex = 1;
  }

  char firstImage[6];
  strcpy_P(firstImage, PSTR("0.bmp"));
  if (sd.exists(firstImage))
  {
    Serial.print(F("Opening File: "));
    Serial.print(folder);
    Serial.println(F("/config.ini"));
    readIniFile();

    char tmp[6];
    strcpy_P(tmp, PSTR("0.bmp"));
    refreshImageDimensions(tmp);

    Serial.print(F("Hold (in ms): "));
    Serial.println(holdTime);
    swapTime = millis() + holdTime;

    // setup image for x/y translation as needed if animations aren't paused
    if (playMode != 2)
    {
      if (offsetSpeedX > 0)
      {
        if (panoff == true) offsetX = (imageWidth * -1);
        else offsetX = (imageWidth * -1 + 16);
      }
      else if (offsetSpeedX < 0)
      {
        if (panoff == true) offsetX = 16;
        else offsetX = 0;
      }
      if (offsetSpeedY > 0)
      {
        if (panoff == true) offsetY = -16;
        else offsetY = 0;
      }
      else if (offsetSpeedY < 0)
      {
        if (panoff == true) offsetY = imageHeight;
        else offsetY = imageHeight - 16;
      }
    }
    // center image if animations are paused
    else
    {
      offsetX = imageWidth / -2 + 8;
      offsetY = imageHeight / 2 - 8;
    }

    // test for single frame

    char tmp_0[6];
    char tmp_1[6];
    strcpy_P(tmp_0, PSTR("0.bmp"));
    strcpy_P(tmp_1, PSTR("1.bmp"));
    if (sd.exists(tmp_0) && (!sd.exists(tmp_1)))
    {
      singleGraphic = true;
      // check for pan settings
      if (offsetSpeedX == 0 && offsetSpeedY == 0)
      {
        // single frame still
        holdTime = -1;
      }
    }
  }

  // empty folder
  else
  {
    Serial.println(F("Empty folder!"));
    nextImage();
  }
}

void drawFrame()
{
  if (panoff == true)
  {
    if (offsetX > 16 || offsetX < (imageWidth * -1) || offsetY > imageHeight || offsetY < -16)
    {
      if (moveLoop == false || finishBeforeProgressing == true)
      {
        fileIndex = 0;
        nextImage();
      }
      else
      {
        if (offsetSpeedX > 0 && offsetX >= 16)
        {
          offsetX = (imageWidth * -1);
        }
        else if (offsetSpeedX < 0 && offsetX <= imageWidth * -1)
        {
          offsetX = 16;
        }
        if (offsetSpeedY > 0 && offsetY >= imageHeight)
        {
          offsetY = -16;
        }
        else if (offsetSpeedY < 0 && offsetY <= -16)
        {
          offsetY = imageHeight;
        }
      }
    }
  }
  else
  {
    if (offsetX > 0 || offsetX < (imageWidth * -1 + 16) || offsetY > imageHeight - 16 || offsetY < 0)
    {
      if (moveLoop == false || finishBeforeProgressing == true)
      {
        fileIndex = 0;
        nextImage();
      }
      else
      {
        if (offsetSpeedX > 0 && offsetX >= 0)
        {
          offsetX = (imageWidth * -1 + 16);
        }
        else if (offsetSpeedX < 0 && offsetX <= imageWidth - 16)
        {
          offsetX = 0;
        }
        if (offsetSpeedY > 0 && offsetY >= imageHeight - 16)
        {
          offsetY = 0;
        }
        else if (offsetSpeedY < 0 && offsetY <= 0)
        {
          offsetY = imageHeight - 16;
        }
      }
    }
  }

  if (singleGraphic == false)
  {
    char bmpFile[13]; // 8-digit number + .bmp + null byte
    itoa(fileIndex, bmpFile, 10);
    strcat(bmpFile, ".bmp");
    if (!sd.exists(bmpFile))
    {
      fileIndex = 0;
      itoa(fileIndex, bmpFile, 10);
      strcat(bmpFile, ".bmp");
      if (finishBeforeProgressing && (offsetSpeedX != 0 || offsetSpeedY != 0)); // translating image - continue animating until moved off screen
      else if (folderLoop == false || timerLapsed == true)
      {
        if (displayMode == 0) nextImage();
        else if (displayMode == 1)
        {
          // just displayed logo, enter clock mode
          initClock();
          return;
        }
      }
    }
    bmpDraw(bmpFile, 0, 0);
  }
  else bmpDraw("0.bmp", 0, 0);

  // print draw time in milliseconds
  drawTime = millis() - lastTime;
  lastTime = millis();
  Serial.print(F("ttd: "));
  Serial.println(drawTime);

  if (offsetSpeedX != 0) offsetX += offsetSpeedX;
  if (offsetSpeedY != 0) offsetY += offsetSpeedY;
}

void refreshImageDimensions(char *filename) {

  const uint8_t  gridWidth = 16;
  const uint8_t  gridHeight = 16;

  if ((0 >= gridWidth) || (0 >= gridHeight)) {
    Serial.print(F("Abort."));
    return;
  }

  // storing dimentions for image
  // Open requested file on SD card
  if (!myFile.open(filename, O_READ)) {
    Serial.println(F("File open failed"));
    sdErrorMessage();
    return;
  }

  // Parse BMP header
  if (read16(myFile) == 0x4D42) { // BMP signature
    (void)read32(myFile); // Read & ignore file size
    (void)read32(myFile); // Read & ignore creator bytes
    (void)read32(myFile); // skip data
    // Read DIB header
    (void)read32(myFile); // Read & ignore Header size
    imageWidth  = read32(myFile);
    imageHeight = read32(myFile);
    Serial.print(F("Image resolution: "));
    Serial.print(imageWidth);
    Serial.print(F("x"));
    Serial.println(imageHeight);
  }
  closeMyFile();
}

// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

void bmpDraw(char *filename, uint8_t x, uint8_t y) {

  int  bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24 or 32)
  uint8_t  sdbuffer[3 * BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t  rowSize;               // Not always = bmpWidth; may have padding
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int  w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0;
  const uint8_t  gridWidth = 16;
  const uint8_t  gridHeight = 16;

  if ((x >= gridWidth) || (y >= gridHeight)) {
    Serial.print(F("Abort."));
    return;
  }

  if (!myFile.isOpen())
  {
    Serial.println();
    Serial.print(F("Loading image '"));
    Serial.print(filename);
    Serial.println('\'');
    // Open requested file on SD card
    if (!myFile.open(filename, O_READ)) {
      Serial.println(F("File open failed"));
      sdErrorMessage();
      return;
    }
  }
  else myFile.rewind();

  // Parse BMP header
  if (read16(myFile) == 0x4D42) { // BMP signature
    if (debugMode)
    {
      Serial.print(F("File size: ")); Serial.println(read32(myFile));
    }
    else (void)read32(myFile);
    (void)read32(myFile); // Read & ignore creator bytes
    bmpImageoffset = read32(myFile); // Start of image data
    //    Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    (void)read32(myFile); // Read & ignore Header size
    bmpWidth  = read32(myFile);
    bmpHeight = read32(myFile);
    if (read16(myFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(myFile); // bits per pixel
      //      Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      if ((bmpDepth == 24 || bmpDepth == 32) && (read32(myFile) == 0)) { // 0 = uncompressed
        goodBmp = true; // Supported BMP format -- proceed!
        if (debugMode)
        {
          Serial.print(F("Image size: "));
          Serial.print(bmpWidth);
          Serial.print('x');
          Serial.println(bmpHeight);
        }

        // image smaller than 16x16?
        if ((bmpWidth < 16 && bmpWidth > -16) || (bmpHeight < 16 && bmpHeight > -16))
        {
          clearStripBuffer();
        }

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * bmpDepth/8 + bmpDepth/8) & ~(bmpDepth/8); // 32-bit BMP support
        //        Serial.print(F("Row size: "));
        //        Serial.println(rowSize);

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if (bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // initialize our pixel index
        byte index = 0; // a byte is perfect for a 16x16 grid

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if ((x + w - 1) >= gridWidth)  w = gridWidth - x;
        if ((y + h - 1) >= gridHeight) h = gridHeight - y;

        for (row = 0; row < h; row++) { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).

          if (flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = (bmpImageoffset + (offsetX * -3) + (bmpHeight - 1 - (row + offsetY)) * rowSize);
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if (myFile.curPosition() != pos) { // Need seek?
            myFile.seekSet(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col = 0; col < w; col++) { // For each pixel...
            // Time to read more pixel data?
            
            if (bmpDepth == 24)
            {
              if (buffidx >= sizeof(sdbuffer)) { // Indeed
                myFile.read(sdbuffer, sizeof(sdbuffer));
                buffidx = 0; // Set index to beginning
              }
            }
            else if (bmpDepth == 32)
            {
              if (buffidx >= 3) { // 32-bit file compatibility forces buffer to 1 pixel at a time
                myFile.read(sdbuffer, 3); 
                myFile.read(); // eat the alpha channel
                buffidx = 0; // Set index to beginning
              }
            }

            // push to LED buffer
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];

            // apply contrast
            r = dim8_jer(r);
            g = dim8_jer(g);
            b = dim8_jer(b);

            // offsetY is beyond bmpHeight
            if (row >= bmpHeight - offsetY)
            {
              // black pixel
              leds[getIndex(col, row)] = CRGB(0, 0, 0);
            }
            // offsetY is negative
            else if (row < offsetY * -1)
            {
              // black pixel
              leds[getIndex(col, row)] = CRGB(0, 0, 0);
            }
            // offserX is beyond bmpWidth
            else if (col >= bmpWidth + offsetX)
            {
              // black pixel
              leds[getIndex(col, row)] = CRGB(0, 0, 0);
            }
            // offsetX is positive
            else if (col < offsetX)
            {
              // black pixel
              leds[getIndex(col, row)] = CRGB(0, 0, 0);
            }
            // all good
            else leds[getIndex(col + x, row)] = CRGB(r, g, b);
            // paint pixel color
          } // end pixel
        } // end scanline
      } // end goodBmp
    }
  }
  if (!clockShown || breakout == true)
  {
    FastLED.show();
  }
  if (singleGraphic == false || menuActive == true || breakout == true)
  {
    closeMyFile();
  }
  if (!goodBmp) Serial.println(F("Format unrecognized"));
}

uint8_t dim8_jer( uint8_t x )
{
  return ((uint16_t)x * (uint16_t)(x) ) >> 8;
}

void closeMyFile()
{
  if (myFile.isOpen()) 
  {
    if (debugMode) Serial.println(F("Closing Image..."));
    myFile.close();
  }
}

byte getIndex(byte x, byte y)
{
  byte index;
  if (y == 0)
  {
    index = x;
  }
  else if (y % 2 == 0)
  {
    index = y * 16 + x;
  }
  else
  {
    index = (y * 16 + 15) - x;
  }
  return index;
}

void clearStripBuffer()
{
  for (int i = 0; i < 256; i++)
  {
    leds[i] = CRGB(0, 0, 0);
  }
}

void buttonDebounce()
{
  // button debounce -- no false positives
  if (((digitalRead(buttonMenuPin) == HIGH) && digitalRead(buttonNextPin) == HIGH) && buttonPressed == true)
  {
    buttonPressed = false;
    buttonEnabled = false;
    buttonTime = millis();
  }
  if ((buttonEnabled == false) && buttonPressed == false)
  {
    if (millis() > buttonTime + 50) buttonEnabled = true;
  }
}

// automatic brightness control
void runABC()
{
  if (currentSecond == 0)
  {
    if (abc && clockSet && currentMinute != abcMinute)
    {
      abcMinute = currentMinute;
      int dayInMinutes = minuteCounter(currentHour, currentMinute);
      int triggerTimes[10] = {abc0.m, abc1.m, abc2.m, abc3.m, abc4.m, abc5.m, abc6.m, abc7.m, abc8.m, abc9.m};
      byte brightnessSettings[10] = {abc0.b, abc1.b, abc2.b, abc3.b, abc4.b, abc5.b, abc6.b, abc7.b, abc8.b, abc9.b};
      for (byte x = 0; x < 9; x++)
      {
        if (dayInMinutes == triggerTimes[x])
        {
          brightness = brightnessSettings[x];
          stripSetBrightness();
          if (brightness == 0)
          {
            clearStripBuffer();
            FastLED.show();
          }
          else if (displayMode == 1)
          {
            initClock();
          }
          // refresh the screen
          else
          {
            drawFrame();
          }
        }
      }
    }
  }
}

// Clock

void debugClockDisplay()
{
  // digital clock display of the time
  Serial.print(currentHour);
  printDigits(currentMinute);
  printDigits(currentSecond);
  Serial.println();
}

void printDigits(int digits)
{
  // utility function for clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void initClock()
{
  saveSettingsToEEPROM();

  secondCounter = 0;
  menuActive = false;
  clockShown = true;
  clockAnimationActive = false;
  buttonTime = millis();
  menuMode = 0;
  lastSecond = 255;

  closeMyFile();
  holdTime = 0;
  sd.chdir("/");
  fileIndex = 0;
  singleGraphic = true;
  if (!logoPlayed) logoPlayed = true;

  if (!clockIniRead) readClockIni();

  if (!clockSet)
  {
    currentHour = 12;
    currentMinute = 0;
    setClock();
  }
  if (!enableSecondHand)
  {
    // 24 hour conversion
    if (hour12 && currentHour > 12) currentHour -= 12;
    if (hour12 && currentHour == 0) currentHour = 12;
    drawDigitsAndShow();
  }
}

void setClock()
{
  clockSet = false;
  clockAdjustState = false;
  currentSecond = 0;
  setClockHour();
  setClockMinute();
  adjustClock();
  lastSecond = 255;
  if (EEPROM.read(4) != 1) EEPROM.write(4, 1);
  // don't adjust second hand at midnight if set manually AT midnight
  if (currentHour == 0 && currentMinute == 0) clockAdjustState = true;
}

void drawDigitsAndShow()
{
  drawDigits();
  FastLED.show();
  debugClockDisplay();
}

void setClockHour()
{
  // set hour
  long lastButtonCheck = 0;
  long lastDigitFlash = 0;
  while (clockDigitSet == false)
  {
    buttonDebounce();
    irReceiver();

    // menu button
    if ((digitalRead(buttonMenuPin) == LOW || irCommand == 'M' || irMenuRepeat == true) && buttonPressed == false && buttonEnabled == true && millis() - lastButtonCheck > 100)
    {
      clockSetBlink = true;
      currentHour++;
      if (currentHour > 23) currentHour = 0;
      lastButtonCheck = millis();
      lastDigitFlash = millis();
      drawDigitsAndShow();
    }

    // flash digit
    if (millis() - lastDigitFlash > 250)
    {
      lastDigitFlash = millis();
      drawDigitsAndShow();
    }

    // next button
    if ((digitalRead(buttonNextPin) == LOW || irCommand == 'N') && buttonPressed == false && buttonEnabled == true)
    {
      buttonPressed = true;
      clockDigitSet = true;
    }
  }
}

void setClockMinute()
{
  // set minutes
  long lastButtonCheck = 0;
  long lastDigitFlash = 0;
  while (clockDigitSet == true)
  {
    buttonDebounce();
    irReceiver();
    // menu button
    if ((digitalRead(buttonMenuPin) == LOW  || irCommand == 'M' || irMenuRepeat == true) && buttonPressed == false && buttonEnabled == true && millis() - lastButtonCheck > 100)
    {
      clockSetBlink = true;
      currentMinute++;
      if (currentMinute > 59) currentMinute = 0;
      lastButtonCheck = millis();
      lastDigitFlash = millis();
      drawDigitsAndShow();
    }

    // flash digit
    if (millis() - lastDigitFlash > 250)
    {
      lastDigitFlash = millis();
      drawDigitsAndShow();
    }

    // next button
    if ((digitalRead(buttonNextPin) == LOW || irCommand == 'N') && buttonPressed == false && buttonEnabled == true)
    {
      buttonPressed = true;
      clockDigitSet = false;
      clockSetBlink = true;
    }
  }
}

void adjustClock()
{
  tm.Hour = currentHour;
  tm.Minute = currentMinute;
  tm.Second = currentSecond;
  RTC.write(tm);
  setTime(RTC.get());
}

// adjust the clock at midnight by the number of seconds indicated in clock.ini file
void clockDriftAdjustment()
{
  if (clockAdjust < 0)
  {
    if (hour() == 0 && minute() == 0 && second() == 0 && !clockAdjustState)
    {
      currentHour = 23;
      currentMinute = 59;
      currentSecond = 60 + clockAdjust;
      adjustClock();
      lastSecond = 255;
      clockAdjustState = true;
    }
  }
  else if (clockAdjust > 0)
  {
    if (hour() == 0 && minute() == 0 && second() == 0)
    {
      currentHour = 0;
      currentMinute = 0;
      currentSecond = clockAdjust;
      adjustClock();
      lastSecond = 255;
    }
  }
  if (hour() == 0 && minute() == 0 && second() > 0 && clockAdjustState) clockAdjustState = false;
}

void getCurrentTime()
{
  currentSecond = second();
  currentMinute = minute();
  currentHour = hour();
}

void showClock()
{
  if (currentSecond != lastSecond)
  {
    secondCounter++;
    lastSecond = currentSecond;
    statusLedFlicker();
    debugClockDisplay();

    // 24 hour conversion
    if (hour12 && currentHour > 12) currentHour -= 12;
    if (hour12 && currentHour == 0) currentHour = 12;

    // draw time
    if (enableSecondHand)
    {
      // offset second hand if required
      currentSecond = currentSecond + secondOffset;
      if (currentSecond >= 60) currentSecond -= 60;
      storeSecondHandColor();
      drawDigits();
      secondHand();
      FastLED.show();
    }
    // second hand disabled, so only draw time on new minute
    else if (currentSecond == 0)
    {
      drawDigitsAndShow();
    }

//    debugClockDisplay();

    // show an animation
    if (cycleTime != -1 && clockAnimationLength > 0 && (secondsIntoHour() % cycleTime) == 0 && clockSet == true)
    {
      secondCounter = 0;
      currentSecond = second();
      clockAnimationActive = true;
      clockShown = false;
      closeMyFile();
      abortImage = true;
      nextFolder[0] = '\0';
    }
    // this boolean is set here to avoid showing an animation immediately after clock being set
    else if (!clockSet) clockSet = true;
  }
}

int secondsIntoHour()
{
  return (currentMinute * 60) + currentSecond;
}

void drawDigits()
{
  clockDigit_1();
  clockDigit_2();
  clockDigit_3();
  clockDigit_4();
  if (!clockSet) clockSetBlink = !clockSetBlink;
  closeMyFile();
}

void storeSecondHandColor()
{
  getSecondHandIndex();
  offsetX = 0;
  offsetY = 176;
  strcpy_P(nextFolder, PSTR("/00system/digits.bmp"));
  // max brightness in order to store correct color value
  FastLED.setBrightness(255);
  bmpDraw(nextFolder, 0, 0);
  secondHandColor = leds[getIndex(secondHandX, secondHandY)];
  // restore brightness
  stripSetBrightness();
}

void clockDigit_1()
{
  char numChar[3];
  itoa(currentHour, numChar, 10);
  byte singleDigit = numChar[0] - '0';
  offsetX = -1;
  if (currentHour >= 10)
  {
    offsetY = singleDigit * 16;
  }
  else offsetY = 160;
  if (!clockSet && !clockDigitSet && !clockSetBlink)
  {
    offsetY = 160;
  }
  strcpy_P(nextFolder, PSTR("/00system/digits.bmp"));
  bmpDraw(nextFolder, 0, 0);
}

void clockDigit_2()
{
  char numChar[3];
  itoa(currentHour, numChar, 10);
  byte singleDigit;
  if (currentHour >= 10)
  {
    singleDigit = numChar[1] - '0';
  }
  else singleDigit = numChar[0] - '0';
  offsetX = 0;
  offsetY = singleDigit * 16;
  if (!clockSet && !clockDigitSet && !clockSetBlink)
  {
    offsetY = 160;
  }
  strcpy_P(nextFolder, PSTR("/00system/digits.bmp"));
  bmpDraw(nextFolder, 3, 0);
}

void clockDigit_3()
{
  char numChar[3];
  itoa(currentMinute, numChar, 10);
  byte singleDigit;
  if (currentMinute >= 10)
  {
    singleDigit = numChar[0] - '0';
  }
  else singleDigit = 0;
  offsetY = singleDigit * 16;
  if (!clockSet && clockDigitSet && !clockSetBlink)
  {
    offsetY = 160;
  }
  strcpy_P(nextFolder, PSTR("/00system/digits.bmp"));
  bmpDraw(nextFolder, 8, 0);
}

void clockDigit_4()
{
  char numChar[3];
  itoa(currentMinute, numChar, 10);
  byte singleDigit;
  if (currentMinute >= 10)
  {
    singleDigit = numChar[1] - '0';
  }
  else singleDigit = numChar[0] - '0';
  offsetY = singleDigit * 16;
  if (!clockSet && clockDigitSet && !clockSetBlink)
  {
    offsetY = 160;
  }
  strcpy_P(nextFolder, PSTR("/00system/digits.bmp"));
  bmpDraw(nextFolder, 12, 0);
}

void getSecondHandIndex()
{
  if (currentSecond < 16)
  {
    secondHandX = currentSecond;
    secondHandY = 0;
  }
  else if (currentSecond >= 16 && currentSecond < 30)
  {
    secondHandX = 15;
    secondHandY = (currentSecond - 15);
  }
  else if (currentSecond >= 30 && currentSecond < 46)
  {
    secondHandX = (15 - (currentSecond - 30));
    secondHandY = 15;
  }
  else if (currentSecond >= 46)
  {
    secondHandX = 0;
    secondHandY = (15 - (currentSecond - 45));
  }
}

void secondHand()
{
  getSecondHandIndex();
  leds[getIndex(secondHandX, secondHandY)] = secondHandColor;
}

// .INI file support

void printErrorMessage(uint8_t e, bool eol = true)
{
  switch (e) {
    case IniFile::errorNoError:
      Serial.print(F("no error"));
      break;
    case IniFile::errorFileNotFound:
      Serial.print(F("fnf"));
      break;
    case IniFile::errorFileNotOpen:
      Serial.print(F("fno"));
      break;
    case IniFile::errorBufferTooSmall:
      Serial.print(F("bts"));
      break;
    case IniFile::errorSeekError:
      Serial.print(F("se"));
      break;
    case IniFile::errorSectionNotFound:
      Serial.print(F("snf"));
      break;
    case IniFile::errorKeyNotFound:
      Serial.print(F("knf"));
      break;
    case IniFile::errorEndOfFile:
      Serial.print(F("eof"));
      break;
    case IniFile::errorUnknownError:
      Serial.print(F("unknown"));
      break;
    default:
      Serial.print(F("unknown error value"));
      break;
  }
  if (eol)
    Serial.println();
}

void readIniFile()
{
  const size_t bufferLen = 50;
  char buffer[bufferLen];
  char configFile[11];
  strcpy_P(configFile, PSTR("config.ini"));
  const char *filename = configFile;
  IniFile ini(filename);
  if (!ini.open()) {
    Serial.print(filename);
    Serial.println(F(" does not exist"));
    // Cannot do anything else
  }
  else
  {
    Serial.println(F("Ini file exists"));
  }

  // Check the file is valid. This can be used to warn if any lines
  // are longer than the buffer.
  if (!ini.validate(buffer, bufferLen)) {
    Serial.print(F("ini file "));
    Serial.print(ini.getFilename());
    Serial.print(F(" not valid: "));
    printErrorMessage(ini.getError());
    // Cannot do anything else
  }
  char section[10];
  strcpy_P(section, PSTR("animation"));
  char entry[11];
  strcpy_P(entry, PSTR("hold"));

  // Fetch a value from a key which is present
  if (ini.getValue(section, entry, buffer, bufferLen)) {
    Serial.print(F("hold value: "));
    Serial.println(buffer);
    holdTime = atol(buffer);
  }
  else {
    printErrorMessage(ini.getError());
    holdTime = 200;
  }

  strcpy_P(entry, PSTR("loop"));

  // Fetch a boolean value
  bool loopCheck;
  bool found = ini.getValue(section, entry, buffer, bufferLen, loopCheck);
  if (found) {
    Serial.print(F("animation loop value: "));
    // Print value, converting boolean to a string
    Serial.println(loopCheck ? F("TRUE") : F("FALSE"));
    folderLoop = loopCheck;
  }
  else {
    printErrorMessage(ini.getError());
    folderLoop = true;
  }

  strcpy_P(entry, PSTR("finish"));

  // Fetch a boolean value
  bool finishCheck;
  bool found4 = ini.getValue(section, entry, buffer, bufferLen, finishCheck);
  if (found4) {
    Serial.print(F("finish value: "));
    // Print value, converting boolean to a string
    Serial.println(finishCheck ? F("TRUE") : F("FALSE"));
    finishBeforeProgressing = finishCheck;
  }
  else {
    printErrorMessage(ini.getError());
    finishBeforeProgressing = false;
  }

  strcpy_P(section, PSTR("translate"));
  strcpy_P(entry, PSTR("moveX"));

  // Fetch a value from a key which is present
  if (ini.getValue(section, entry, buffer, bufferLen)) {
    Serial.print(F("moveX value: "));
    Serial.println(buffer);
    offsetSpeedX = atoi(buffer);
  }
  else {
    printErrorMessage(ini.getError());
    offsetSpeedX = 0;
  }

  strcpy_P(entry, PSTR("moveY"));

  // Fetch a value from a key which is present
  if (ini.getValue(section, entry, buffer, bufferLen)) {
    Serial.print(F("moveY value: "));
    Serial.println(buffer);
    offsetSpeedY = atoi(buffer);
  }
  else {
    printErrorMessage(ini.getError());
    offsetSpeedY = 0;
  }

  strcpy_P(entry, PSTR("loop"));

  // Fetch a boolean value
  bool loopCheck2;
  bool found2 = ini.getValue(section, entry, buffer, bufferLen, loopCheck2);
  if (found2) {
    Serial.print(F("translate loop value: "));
    // Print value, converting boolean to a string
    Serial.println(loopCheck2 ? F("TRUE") : F("FALSE"));
    moveLoop = loopCheck2;
  }
  else {
    printErrorMessage(ini.getError());
    moveLoop = false;
  }

  strcpy_P(entry, PSTR("panoff"));

  // Fetch a boolean value
  bool loopCheck3;
  bool found3 = ini.getValue(section, entry, buffer, bufferLen, loopCheck3);
  if (found3) {
    Serial.print(F("panoff value: "));
    // Print value, converting boolean to a string
    Serial.println(loopCheck3 ? F("TRUE") : F("FALSE"));
    panoff = loopCheck3;
  }
  else {
    printErrorMessage(ini.getError());
    panoff = true;
  }

  strcpy_P(entry, PSTR("nextFolder"));

  // Fetch a value from a key which is present
  if (ini.getValue(section, entry, buffer, bufferLen)) {
    Serial.print(F("nextFolder value: "));
    Serial.println(buffer);
    memcpy(nextFolder, buffer, 8);
  }
  else {
    printErrorMessage(ini.getError());
    nextFolder[0] = '\0';
  }

  if (ini.isOpen()) ini.close();
}

void readClockIni()
{
  clockIniRead = true;
  const size_t bufferLen = 50;
  char buffer[bufferLen];
  char configFile[11];
  strcpy_P(configFile, PSTR("clock.ini"));
  const char *filename = configFile;
  IniFile ini(filename);
  sd.chdir("/00system");
  if (!ini.open()) {
    Serial.print(filename);
    Serial.println(F(" does not exist"));
    // Cannot do anything else
  }
  else
  {
    Serial.println(F("Ini file exists"));
  }

  // Check the file is valid. This can be used to warn if any lines
  // are longer than the buffer.
  if (!ini.validate(buffer, bufferLen)) {
    Serial.print(F("ini file "));
    Serial.print(ini.getFilename());
    Serial.print(F(" not valid: "));
    printErrorMessage(ini.getError());
    // Cannot do anything else
  }
  char section[10];
  strcpy_P(section, PSTR("clock"));
  char entry[11];
  strcpy_P(entry, PSTR("hour12"));

  // Fetch a boolean value
  // 12/24 hour mode
  bool loopCheck;
  bool found = ini.getValue(section, entry, buffer, bufferLen, loopCheck);
  if (found) {
    Serial.print(F("hour12 value: "));
    // Print value, converting boolean to a string
    Serial.println(loopCheck ? F("TRUE") : F("FALSE"));
    hour12 = loopCheck;
  }
  else {
    printErrorMessage(ini.getError());
    hour12 = true;
  }

  strcpy_P(entry, PSTR("second"));

  // Fetch a boolean value
  // show second hand
  found = ini.getValue(section, entry, buffer, bufferLen, loopCheck);
  if (found) {
    Serial.print(F("second value: "));
    // Print value, converting boolean to a string
    Serial.println(loopCheck ? F("TRUE") : F("FALSE"));
    enableSecondHand = loopCheck;
  }
  else {
    printErrorMessage(ini.getError());
    enableSecondHand = true;
  }

  strcpy_P(entry, PSTR("offset"));

  // Fetch a value from a key which is present
  // second hand position offset
  if (ini.getValue(section, entry, buffer, bufferLen)) {
    Serial.print(F("offset value: "));
    Serial.println(buffer);
    secondOffset = atoi(buffer);
  }
  else {
    printErrorMessage(ini.getError());
    secondOffset = 0;
  }

  strcpy_P(entry, PSTR("anim"));

  // Fetch a value from a key which is present
  // clock animation length in seconds
  if (ini.getValue(section, entry, buffer, bufferLen)) {
    Serial.print(F("anim value: "));
    Serial.println(buffer);
    clockAnimationLength = atoi(buffer);
  }
  else {
    printErrorMessage(ini.getError());
    clockAnimationLength = 5;
  }

  strcpy_P(entry, PSTR("adjust"));

  // Fetch a value from a key which is present
  // daily midnight offset in seconds
  if (ini.getValue(section, entry, buffer, bufferLen)) {
    Serial.print(F("adjust value: "));
    Serial.println(buffer);
    clockAdjust = atoi(buffer);
  }
  else {
    printErrorMessage(ini.getError());
    clockAdjust = 0;
  }

  if (ini.isOpen()) ini.close();
  sd.chdir("/");
}

// Auto Brightness Control
void readABC()
{
  const size_t bufferLen = 50;
  char buffer[bufferLen];
  char configFile[11];
  strcpy_P(configFile, PSTR("clock.ini"));
  const char *filename = configFile;
  IniFile ini(filename);
  sd.chdir("/00system");
  if (!ini.open()) {
    Serial.print(filename);
    Serial.println(F(" does not exist"));
    // Cannot do anything else
  }
  else
  {
    Serial.println(F("Ini file exists"));
  }

  // Check the file is valid. This can be used to warn if any lines
  // are longer than the buffer.
  if (!ini.validate(buffer, bufferLen)) {
    Serial.print(F("ini file "));
    Serial.print(ini.getFilename());
    Serial.print(F(" not valid: "));
    printErrorMessage(ini.getError());
    // Cannot do anything else
  }
  char section[10];
  strcpy_P(section, PSTR("abc"));
  char entry[11];
  strcpy_P(entry, PSTR("abc"));

  // Fetch a boolean value
  // Use ABC?
  bool boolcheck;
  bool found = ini.getValue(section, entry, buffer, bufferLen, boolcheck);
  if (found) {
    Serial.print(F("abc value: "));
    // Print value, converting boolean to a string
    Serial.println(boolcheck ? F("TRUE") : F("FALSE"));
    abc = boolcheck;
  }
  else {
    printErrorMessage(ini.getError());
    abc = false;
  }

  //  check INI for 1440 time of day entries (takes too long)
  //  int minuteInDay = 0;
  //
  //  for (int h=0; h<24; h++)
  //  {
  //    for (int m=0; m<60; m++)
  //    {
  //      // construct time of day character array
  //      char timeOfDay[6];
  //      char minuteChar[3];
  //      itoa(h, timeOfDay, 10);
  //      strcat(timeOfDay, ":");
  //      itoa(m, minuteChar, 10);
  //      if (strlen(minuteChar) == 1) strcat(timeOfDay, "0");
  //      strcat(timeOfDay, minuteChar);
  //
  //      // Fetch a value from a key which is present
  //      // abc brightness level 0
  //      buffer[0] = '\0';
  //      if (ini.getValue(section, timeOfDay, buffer, bufferLen)) {
  //        if (strlen(buffer) > 0)
  //        {
  //          Abc abc0 = {minuteInDay, atoi(buffer)};
  //          Serial.print(timeOfDay);
  //          Serial.print(F(" brightness: "));
  //          Serial.println(atoi(buffer));
  //        }
  //      }
  //      minuteInDay++;
  //    }
  //  }

  // Fetch a value from a key which is present
  if (ini.getValue(section, "abc0", buffer, bufferLen)) {
    if (strlen(buffer) > 0)
    {
      int h = atoi(strtok(buffer, ":"));
      int m = atoi(strtok(NULL, ":,"));
      abc0.m = minuteCounter(h, m);
      abc0.b = atoi(strtok(NULL, " ,"));
    }
    Serial.print("abc0 value: ");
    Serial.print(abc0.m);
    Serial.print(", ");
    Serial.println(abc0.b);
  }
  else {
    printErrorMessage(ini.getError());
  }

  // Fetch a value from a key which is present
  if (ini.getValue(section, "abc1", buffer, bufferLen)) {
    if (strlen(buffer) > 0)
    {
      int h = atoi(strtok(buffer, ":"));
      int m = atoi(strtok(NULL, ":,"));
      abc1.m = minuteCounter(h, m);
      abc1.b = atoi(strtok(NULL, " ,"));
    }
    Serial.print("abc1 value: ");
    Serial.print(abc1.m);
    Serial.print(", ");
    Serial.println(abc1.b);
  }
  else {
    printErrorMessage(ini.getError());
  }

  // Fetch a value from a key which is present
  if (ini.getValue(section, "abc2", buffer, bufferLen)) {
    if (strlen(buffer) > 0)
    {
      int h = atoi(strtok(buffer, ":"));
      int m = atoi(strtok(NULL, ":,"));
      abc2.m = minuteCounter(h, m);
      abc2.b = atoi(strtok(NULL, " ,"));
    }
    Serial.print("abc2 value: ");
    Serial.print(abc2.m);
    Serial.print(", ");
    Serial.println(abc2.b);
  }
  else {
    printErrorMessage(ini.getError());
  }

  // Fetch a value from a key which is present
  if (ini.getValue(section, "abc3", buffer, bufferLen)) {
    if (strlen(buffer) > 0)
    {
      int h = atoi(strtok(buffer, ":"));
      int m = atoi(strtok(NULL, ":,"));
      abc3.m = minuteCounter(h, m);
      abc3.b = atoi(strtok(NULL, " ,"));
    }
    Serial.print("abc3 value: ");
    Serial.print(abc3.m);
    Serial.print(", ");
    Serial.println(abc3.b);
  }
  else {
    printErrorMessage(ini.getError());
  }

  // Fetch a value from a key which is present
  if (ini.getValue(section, "abc4", buffer, bufferLen)) {
    if (strlen(buffer) > 0)
    {
      int h = atoi(strtok(buffer, ":"));
      int m = atoi(strtok(NULL, ":,"));
      abc4.m = minuteCounter(h, m);
      abc4.b = atoi(strtok(NULL, " ,"));
    }
    Serial.print("abc4 value: ");
    Serial.print(abc4.m);
    Serial.print(", ");
    Serial.println(abc4.b);
  }
  else {
    printErrorMessage(ini.getError());
  }

  // Fetch a value from a key which is present
  if (ini.getValue(section, "abc5", buffer, bufferLen)) {
    if (strlen(buffer) > 0)
    {
      int h = atoi(strtok(buffer, ":"));
      int m = atoi(strtok(NULL, ":,"));
      abc5.m = minuteCounter(h, m);
      abc5.b = atoi(strtok(NULL, " ,"));
    }
    Serial.print("abc5 value: ");
    Serial.print(abc5.m);
    Serial.print(", ");
    Serial.println(abc5.b);
  }
  else {
    printErrorMessage(ini.getError());
  }

  // Fetch a value from a key which is present
  if (ini.getValue(section, "abc6", buffer, bufferLen)) {
    if (strlen(buffer) > 0)
    {
      int h = atoi(strtok(buffer, ":"));
      int m = atoi(strtok(NULL, ":,"));
      abc6.m = minuteCounter(h, m);
      abc6.b = atoi(strtok(NULL, " ,"));
    }
    Serial.print("abc6 value: ");
    Serial.print(abc6.m);
    Serial.print(", ");
    Serial.println(abc6.b);
  }
  else {
    printErrorMessage(ini.getError());
  }

  // Fetch a value from a key which is present
  if (ini.getValue(section, "abc7", buffer, bufferLen)) {
    if (strlen(buffer) > 0)
    {
      int h = atoi(strtok(buffer, ":"));
      int m = atoi(strtok(NULL, ":,"));
      abc7.m = minuteCounter(h, m);
      abc7.b = atoi(strtok(NULL, " ,"));
    }
    Serial.print("abc7 value: ");
    Serial.print(abc7.m);
    Serial.print(", ");
    Serial.println(abc7.b);
  }
  else {
    printErrorMessage(ini.getError());
  }

  // Fetch a value from a key which is present
  if (ini.getValue(section, "abc8", buffer, bufferLen)) {
    if (strlen(buffer) > 0)
    {
      int h = atoi(strtok(buffer, ":"));
      int m = atoi(strtok(NULL, ":,"));
      abc8.m = minuteCounter(h, m);
      abc8.b = atoi(strtok(NULL, " ,"));
    }
    Serial.print("abc8 value: ");
    Serial.print(abc8.m);
    Serial.print(", ");
    Serial.println(abc8.b);
  }
  else {
    printErrorMessage(ini.getError());
  }

  // Fetch a value from a key which is present
  if (ini.getValue(section, "abc9", buffer, bufferLen)) {
    if (strlen(buffer) > 0)
    {
      int h = atoi(strtok(buffer, ":"));
      int m = atoi(strtok(NULL, ":,"));
      abc9.m = minuteCounter(h, m);
      abc9.b = atoi(strtok(NULL, " ,"));
    }
    Serial.print("abc9 value: ");
    Serial.print(abc9.m);
    Serial.print(", ");
    Serial.println(abc9.b);
  }
  else {
    printErrorMessage(ini.getError());
  }

  // Fetch a value from a key which is present
  if (ini.getValue(section, "colorCorrection", buffer, bufferLen)) {
    if (strlen(buffer) > 0)
    {
      colorCorrection = strtoul(buffer, NULL, 16);
    }
    Serial.print("Color Correction: ");
    Serial.println(buffer);
  }
  else {
    printErrorMessage(ini.getError());
    colorCorrection = 0xFFFFFF;
  }

  // Fetch a value from a key which is present
  if (ini.getValue(section, "colorTemperature", buffer, bufferLen)) {
    if (strlen(buffer) > 0)
    {
      colorTemperature = strtoul(buffer, NULL, 16);
    }
    Serial.print("Color Temperature: ");
    Serial.println(buffer);
  }
  else {
    printErrorMessage(ini.getError());
    colorTemperature = 0xFFFFFF;
  }

  if (ini.isOpen()) ini.close();
  sd.chdir("/");
}

void readRemoteIni()
{
  const size_t bufferLen = 50;
  char buffer[bufferLen];
  char configFile[11];
  strcpy_P(configFile, PSTR("remote.ini"));
  const char *filename = configFile;
  IniFile ini(filename);
  sd.chdir("/00system");
  if (!ini.open()) {
    Serial.print(filename);
    Serial.println(F(" does not exist"));
    // Cannot do anything else
  }
  else
  {
    Serial.println(F("Ini file exists"));
  }

  // Check the file is valid. This can be used to warn if any lines
  // are longer than the buffer.
  if (!ini.validate(buffer, bufferLen)) {
    Serial.print(F("ini file "));
    Serial.print(ini.getFilename());
    Serial.print(F(" not valid: "));
    printErrorMessage(ini.getError());
    // Cannot do anything else
  }
  char section[7];
  strcpy_P(section, PSTR("remote"));

  // Fetch a value from a key which is present
  // second hand position offset
  if (ini.getValue(section, "menu", buffer, bufferLen)) {
    Serial.print(F("IR Menu code: "));
    Serial.println(buffer);
    remoteCodeMenu = strtoull(buffer, NULL, 10);
  }
  else {
    printErrorMessage(ini.getError());
    remoteCodeMenu = 2155864095;
  }

  // Fetch a value from a key which is present
  // clock animation length in seconds
  if (ini.getValue(section, "next", buffer, bufferLen)) {
    Serial.print(F("IR Next code: "));
    Serial.println(buffer);
    remoteCodeNext = strtoull(buffer, NULL, 10);
  }
  else {
    printErrorMessage(ini.getError());
    remoteCodeNext = 2155831455;
  }

  // Fetch a value from a key which is present
  // daily midnight offset in seconds
  if (ini.getValue(section, "power", buffer, bufferLen)) {
    Serial.print(F("IR Power code: "));
    Serial.println(buffer);
    remoteCodePower = strtoull(buffer, NULL, 10);
  }
  else {
    printErrorMessage(ini.getError());
    remoteCodePower = 2155819215;
  }

  if (ini.isOpen()) ini.close();
  sd.chdir("/");
}

int minuteCounter(byte h, byte m)
{
  int minutes = 0;
  minutes = h * 60;
  minutes += m;
  return minutes;
}

// breakout code
void drawPaddle()
{
  leds[paddleIndex] = CRGB(200, 200, 200);
  leds[paddleIndex + 1] = CRGB(200, 200, 200);
  leds[paddleIndex + 2] = CRGB(200, 200, 200);
  FastLED.show();
}

void breakoutLoop()
{
  if (holdTime > 0) holdTime--;
  if (fileIndex > 0) fileIndex--;

  if (buttonEnabled == false)
  {
    if (millis() > buttonTime + 50) buttonEnabled = true;
  }

  // menu button
  if ((digitalRead(buttonNextPin) == LOW || irCommand == 'N' || irNextRepeat == true) && holdTime == 0 && paddleIndex < 237 && gameInitialized == true && buttonEnabled == true)
  {
    paddleIndex++;
    leds[paddleIndex - 1] = CRGB(0, 0, 0);
    drawPaddle();
    // paddle speed
    holdTime = 2500;
    if (ballMoving == false)
    {
      ballMoving = true;
      // ball speed
      swapTime = 5000;
      ballAngle = random8(190, 225);
    }
  }

  // next button
  else if ((digitalRead(buttonMenuPin) == LOW || irCommand == 'M' || irMenuRepeat == true) && holdTime == 0 && paddleIndex > 224 && buttonEnabled == true)
  {
    paddleIndex--;
    leds[paddleIndex + 3] = CRGB(0, 0, 0);
    drawPaddle();
    // paddle speed
    holdTime = 2500;
    if (ballMoving == false)
    {
      ballMoving = true;
      // ball speed
      swapTime = 5000;
      ballAngle = random8(135, 170);
    }
  }

  else if ((digitalRead(buttonNextPin) == HIGH || digitalRead(buttonMenuPin) == HIGH || irCommand == 'M' || irCommand == 'N') && gameInitialized == false) gameInitialized = true;

  // ball logic
  if (ballMoving == true && fileIndex == 0)
  {
    fileIndex = swapTime;
    leds[ballIndex] = CRGB(0, 0, 0);

    // did the player lose?
    if (ballIndex >= 239)
    {
      ballMoving = false;
      breakout = false;
      Serial.print(F("Lose!!!"));
      for (int c = 250; c >= 0; c = c - 10)
      {
        for (int i = 0; i < NUM_LEDS; i++)
        {
          byte r = 0;
          byte g = 0;
          byte b = 0;
          if (random8(0, 2) == 1)
          {
            r = c;
          }
          if (random8(0, 2) == 1)
          {
            g = c;
          }
          if (random8(0, 2) == 1)
          {
            b = c;
          }
          leds[i] = CRGB(r, g, b);
        }
        FastLED.show();
        delay(50);
      }
    }

    // ball still in play
    else
    {
      if (ballAngle < 180)
      {
        if ((ballX + sin(degToRad(ballAngle)) * 16) + .5 > 256) swapXdirection();
      }
      else
      {
        if ((ballX + sin(degToRad(ballAngle)) * 16) + .5 < 0) swapXdirection();
      }
      ballX = ballX + sin(degToRad(ballAngle)) * 16 + .5;

      if (ballAngle > 90 && ballAngle < 270)
      {
        if ((ballY + cos(degToRad(ballAngle)) * 16) + .5 < 0) swapYdirection();
      }
      else
      {
        if ((ballY + cos(degToRad(ballAngle)) * 16) + .5 > 256) swapYdirection();
      }
      ballY = ballY + cos(degToRad(ballAngle)) * 16 + .5;
      ballIndex = getScreenIndex(ballX, ballY);

      // paddle hit?
      if (ballIndex == paddleIndex or ballIndex == paddleIndex + 1 or ballIndex == paddleIndex + 2)
      {
        // move the ball back in time one step
        ballX = ballX + ((sin(degToRad(ballAngle)) * 16 + .5) * -1);
        ballY = ballY + ((cos(degToRad(ballAngle)) * 16 + .5) * -1);
        swapYdirection();
        if (ballIndex == paddleIndex)
        {
          ballAngle = random8(115, 170);
        }
        else if (ballIndex == paddleIndex + 2)
        {
          ballAngle = random8(190, 245);
        }
        ballIndex = getScreenIndex(ballX, ballY);
        leds[paddleIndex] = CRGB(200, 200, 200);
        leds[paddleIndex + 1] = CRGB(200, 200, 200);
        leds[paddleIndex + 2] = CRGB(200, 200, 200);
      }

      // brick hit?
      if (leds[ballIndex].r > 0 || leds[ballIndex].g > 0 || leds[ballIndex].b > 0)
      {
        // speed up and change direction
        swapTime -= 35;
        swapYdirection();
        if (winCheck())
        {
          Serial.print(F("Win!!!"));
          for (byte flashes=0; flashes < 30; flashes++)
          {
            leds[ballIndex] = CRGB(random8(), random8(), random8());
            FastLED.show();
            delay(50);
          }
          ballMoving = false;
          chdirFirework();
          char bmpFile[7]; // 2-digit number + .bmp + null byte
          for (byte fileIndex = 0; fileIndex < 37; fileIndex++)
          {
            itoa(fileIndex, bmpFile, 10);
            strcat(bmpFile, ".bmp");
            bmpDraw(bmpFile, 0, 0);
            delay(90);
          }
          breakout = false;
        }
      }

      // check for preceeding win
      if (breakout == true)
      {
        leds[ballIndex] = CRGB(175, 255, 15);
        FastLED.show();
      }
    }
  }
}

void chdirFirework()
{
  char tmp[20];
  strcpy_P(tmp, PSTR("/00system/firework"));
  sd.chdir(tmp);
}

boolean winCheck()
{
  byte numberOfLitPixels = 0;
  for (byte i = 0; i < 255; i++)
  {
    if (leds[i].r > 0 || leds[i].b > 0 || leds[i].g > 0)
    {
      numberOfLitPixels++;
    }
  }
  if (numberOfLitPixels <= 4)
  {
    // why is this delay needed?? Function always returns true without it.
    delay(1);
    return true;
  }
}

byte getScreenIndex(byte x, byte y)
{
  byte screenX = x / 16;
  byte screenY = y / 16;
  byte index;
  index = screenY * 16;
  if (screenY == 0)
  {
    index = 15 - screenX;
  }
  else if (screenY % 2 != 0)
  {
    index = (screenY * 16) + screenX;
  }
  else
  {
    index = (screenY * 16 + 15) - screenX;
  }
  return index;
}

void swapYdirection()
{
  if (ballAngle > 90 && ballAngle < 270)
  {
    if (ballAngle > 180)
    {
      ballAngle = 360 - (ballAngle - 180);
    }
    else
    {
      ballAngle = 90 - (ballAngle - 90);
    }
  }
  else
  {
    if (ballAngle < 90)
    {
      ballAngle = 90 + (90 - ballAngle);
    }
    else
    {
      ballAngle = 180 + (360 - ballAngle);
    }
  }
}

void swapXdirection()
{
  if (ballAngle < 180)
  {
    if (ballAngle < 90)
    {
      ballAngle = 270 + (90 - ballAngle);
    }
    else ballAngle = 270 - (ballAngle - 90);
  }
  else
  {
    if (ballAngle > 270)
    {
      ballAngle = 360 - ballAngle;
    }
    else ballAngle = 180 - (ballAngle - 180);
  }
}

float degToRad(float deg)
{
  float result;
  result = deg * PI / 180;
  return result;
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(SdFile& f) {
  uint16_t result;
  f.read(&result, 2);
  return result;
}

uint32_t read32(SdFile& f) {
  uint32_t result;
  f.read(&result, 4);
  return result;
}

// clock debug function
void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}
