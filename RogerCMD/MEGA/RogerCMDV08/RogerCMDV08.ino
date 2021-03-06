////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  RogerCMDV08
//
//  Roger Command interfaces to the RogerCMD01 Navigational Subsystem via Bluetooth LE. Commands and Navigational data can
//  be entered via one keypad:
//      1)  4x4 Keypad (used for UI/user commands)
//
//  Along with the two keypads, the main Arduino MEGA 2560 microcomputer is also connected to the following devices:
//      a)  Micro SD Card Adapter
//      b)  Bluetooth LE Interface
//      c)  Serial Rx/TX Connection to an Adafruit Feather M0 Proto connected to an Adafruit Music Maker Featherwing
//          via an Adafruit Feather Doubler - acting as a VRU
//      d)  Open HW Serial Interface (for use by a cell modem for IoT/Geotracking capability)
//      e)  I2C Multiplexer, two 2605 Haptic drivers, and Haptic pancake buzzers
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  #includes
#include <Arduino.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <SD.h>
#include <Key.h>
#include <Wire.h>
extern "C" {
  #include "utility/twi.h"      // from Wire library, so we can do bus scanning
  }
#include "Adafruit_DRV2605.h"
#include <Keypad.h>
#include <TI_TCA9548A.h>
#include "RTClib.h"
#include <ArduinoJson.h>
#include <ctype.h>
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  #defines
#define bleCentral              Serial1      // Bluefruit HW Serial Name
#define vruSerial               Serial2      // Adafruit Music Maker HW Serial Name
#define cellSerial              Serial3      // cell module Serial Name
#define cardSelect              53           // Card Select on the MEGA for SD Card SPI comm
#define VBATPIN                 A7           // Voltage at the IMU backup battery pin
#define VERBOSE_MODE            true         // If set to 'true' enables debug output
#define BUFSIZE                 255          // Size of the read buffer for incoming data
#define tcaADDR                 0x70         // Address of TCA MUX
#define sysLED                  13           // Onboard SYSTEM LED and Piezo Buzzer
#define AUTO_RECORD_INT_MS      10000        // 10 sec interval for auto record waypoint
#define PLAYBACK_INT_MS         3000         // 3 sec interval for computing course during playback
#define BLE_WAIT_MS             300          // Time to wait for BLE response n ms
#define TRIP_COMPLETE_INT_MS    10000        // 10 sec interval to let user know trip is complete
#define MODEM_INIT_WAIT         5000         // Time for modem to initialize
#define WAYPOINTS_INIT_LEN      200
#define WAYPOINT_PAGES          10
#define LOC_UPDATE_INT          10000
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  haptic object
Adafruit_DRV2605 drvHap;
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
RTC_PCF8523 rtc;                        //  Assign the RTC_PCF8523 to the rtc object name
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
File tripFile;                           //  Assign the File object to the tripFile object name
File playbackFile;
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  global data/variables
byte effect            = 109;          // effect requested by Ahmet
char keyPadInput       = ' ';
uint32_t timer         = millis();     // unsigned 32bit interger variable called timer assigned to the millisecond function
uint32_t timer2        = millis();
//uint32_t tripRecSeq    = 0;            // used to sequence the record numbers in the log file
uint16_t voiceRec      = 0;            // number being sent to VRU for voice playback
uint8_t navHour        = 0;            // current UCT (GMT) hour data from GPS
uint8_t navMinute      = 0;            // current minute data from GPS
uint8_t navSeconds     = 0;            // current seconds data from GPS
//uint16_t navMillsec    = 0;            // current milliseconds data from GPS
uint8_t localHour      = 0;            // local hour data
uint8_t localMinute    = 0;            // local minute data
uint8_t localSeconds   = 0;            // local seconds data
uint8_t navMonth       = 0;            // current month data from GPS
uint8_t navDay         = 0;            // current day data from GPS
uint16_t navYear       = 0;            // current year data from GPS
uint8_t localMonth     = 0;            // local month data
uint8_t localDay       = 0;            // local day data
uint8_t localYear      = 0;            // local year data
uint8_t gpsFix         = 0;            // GPS Sat fix indicator
uint8_t gpsQual        = 0;            // GPS signal quality
float decimalDegLat    = 0;            // Latitude in decimal degrees
float gpsLatitude      = 0;            // GPS Latitude (DDMM.MMMM)
int gpsLatDeg          = 0;            // GPS Latitude Degrees (+/- 90)
uint8_t gpsLatMin      = 0;            // GPS Latitude Minutes (0 - 59)
float gpsLatSec        = 0;            // GPS Latitude Seconds (0 - 59.59)
float decimalDegLon    = 0;            // Longitude in decimal degrees
float gpsLongitude     = 0;            // GPS Longitude (DDDMM.MMMM)
int gpsLonDeg          = 0;            // GPS Longitude Degrees (+/- 179.99)
uint8_t gpsLonMin      = 0;            // GPS Longitude Degrees (0 - 59)
float gpsLonSec        = 0;            // GPS Longitude Seconds (0 - 59.59)
char gpsLat            = ' ';          // GPS North or South indicator
char gpsLon            = ' ';          // GPS East or West indicator
float gpsKnots         = 0;            // GPS Speed in Knots
byte gpsLatError       = 0;            // GPS Latitude Data Conversion Error
byte gpsLonError       = 0;            // GPS Longitude Data Conversion Error
float tripMiles        = 0;            // accumulated trip miles traveled
float tripKnots        = 0;            // accumulated trip nautical miles traveled
float tripMeters       = 0;            // accumulated trip meters traveled
float segmentMiles     = 0;            // segment miles traveled (current waypoint to next waypoint)
float segmentKnots     = 0;            // segment nautical miles traveled
float segmentMeters    = 0;            // segment meters traveled
float feetToNextWP     = 0;            // distance in feet to next segment destination point/waypoint
float knotsToNextWP    = 0;            // distance in nautical miles to next segment destination point/waypoint
float milesToNextWP    = 0;            // distance in miles to next segment destination point/waypoint
float metersToNextWP   = 0;            // distance in meters to next segment destination point/waypoint
float gpsAngle         = 0;            // GPS Angle
//int gpsAltitude        = 0;            // GPS Altitude (in meters)
uint8_t gpsSats        = 0;            // GPS Satellites
//float roll             = 0;            // variable to hold roll data from the IMU
//float pitch            = 0;            // variable to hold the pitch data from the IMU
float heading          = 0;            // variable to hold the heading data from the IMU
float qHead            = 0;            // float to hold computed heading from the Quaternion angle
float qW               = 0;            // float to hold the Quaternion angle for heading from the IMU
float qX               = 0;            // variable to hold Quaternion X data from the IMU
float qY               = 0;            // variable to hold Quaternion Y data from the IMU
float qZ               = 0;            // variable to hold Quaternion Z data from the IMU
//int utcOffset          = 0;            // offset for local time
//int dstAdjust          = 0;            // adjustment for daylight saving time
float measuredVbat     = 0;            // float to hold the measured batttery voltage
const char navName[11] = "RogerNAV01"; // name of the bluetooth LE nav module
const char cmdName[11] = "RogerCMD01"; // name of the bluetoothe LE command module
//const int bleBuffSize  = 127;          // size of bluetooth buffer size
int bleRSSI            = 0;            // BLE Signal Strength
//
//
const byte ROWS = 4;                   //four rows
const byte COLS = 4;                   //four columns
char keys[ROWS][COLS] = {
  {'1','2','3', 'A'},
  {'4','5','6', 'B'},
  {'7','8','9', 'C'},
  {'*','0','#', 'D'}
};
//
//
byte rowPins[ROWS] = {2, 3, 4, 5};        //connect to the row pinouts of the keypad
byte colPins[COLS] = {6, 7, 8, 9};        //connect to the column pinouts of the keypad
//
//
//byte bleBuffer[bleBuffSize];              // buffer to hold data to/from the bluetooth comm connection
//
//
//boolean stringComplete    = false;        // whether the string is complete
boolean dateRefreshed     = false;        // updated when the date is refreshed from GPS
//boolean dsTime            = false;        // standard time when false.  daylight saving when true.
//boolean keyPadPressed     = false;
//boolean buzzRight         = false;
//boolean buzzLeft          = false;
boolean navConnected      = false;
boolean cellReq           = false;
boolean trace             = true;
boolean logPlayback       = true;
boolean modemReady        = false;
boolean connectionGood    = false;
//
//
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
char filename[15] = "CMDT0000.TXT";
//
//
//
Keypad cmdKeyPad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

enum Mode {
  MANUAL_REC,
  AUTO_REC,
  PLAYBACK,
  NONE
};

enum PlaybackStep {
  SELECT_FILE,
  FILE_LOADED,
  WAYPOINTS_LOADED,
  IN_PROGRESS,
  COMPLETE
};

enum Direction {
  LEFT,
  RIGHT,
  FRONT,
  BACK
};

typedef struct waypoint {
  int seqNum;
  uint32_t time;
  float gpsLatDeg;
  float gpsLonDeg;
} Waypoint;

Mode currMode = NONE;
String numpadEntry = "";
String currentString = "";
String modemResponse = "";
String iccid = "";
PlaybackStep currPlaybackStep = SELECT_FILE;
//QueueArray<Waypoint> wayPointQueue;
//Waypoint waypoints[WAYPOINTS_INIT_LEN];
int waypointsLen = WAYPOINTS_INIT_LEN;
int currWaypointInd = 0;
int lastWaypointInd = 0;
int totalWaypoints = 0;
uint32_t filePos = 0;
//Waypoint startingWaypoints[WAYPOINT_PAGES];
Waypoint currWaypoint;
Waypoint nextWaypoint;
float currHeading = 0;
int startingWaypointsInd = 0;
int totalWaypointsInd = 0;
int numWaypointPages = 0;
int bleErrCnt = 0;
//
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Haptic Functions and Routines
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void playAll() {
  playRight();
  playLeft();

}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void playRight() {
  tcaSelect(0);                                 // Right Side
  playBuzzer();
  delay(200);
  queueVoiceResponse(66);                       //   Right
  delay(300);
  if (trace) {
    Serial.print("Right Effect #");
    Serial.println(effect);
    }
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void playLeft() {
  tcaSelect(1);
  playBuzzer();
  delay(200);
  queueVoiceResponse(67);                        //  Left
  delay(300);
  if (trace) {
    Serial.print("Left Effect #");
    Serial.println(effect);
    }
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void playBuzzer() {
// set the effect to play
  drvHap.setWaveform(0, effect);  // play effect
  drvHap.setWaveform(1, 0);       // end waveform
// play the effect!
  drvHap.go();
}
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void tcaSelect(uint8_t i) {
  if (i > 7) return;
//
  Wire.beginTransmission(tcaADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void readyBuzz() {
  effect = 109;
  for (int a = 0; a < 3; a++) {
    playAll();
    }
//  effect = 1;
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void hapBegin() {
  tcaSelect(0);
  drvHap.begin();
  configHapticDrv();
  tcaSelect(1);
  drvHap.begin();
  configHapticDrv();
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void configHapticDrv() {
  drvHap.selectLibrary(1);
// I2C trigger by sending 'go' command
// default, internal trigger when sending GO command
  drvHap.setMode(DRV2605_MODE_INTTRIG);
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
byte scanI2CBus() {
  byte portsFound = 0;
  if (trace) {
    Serial.println("\nTCAScanner ready!");
    }
  for (uint8_t t=0; t<8; t++) {
    tcaSelect(t);
    if (trace) {
      Serial.print("TCA Port #");
      Serial.println(t);
      }
    for (uint8_t addr = 0; addr<=127; addr++) {
      if (addr == tcaADDR) continue;
//
      uint8_t data;
      if (! twi_writeTo(addr, &data, 0, 1, 1)) {
         portsFound++;
         if (trace) {
           Serial.print("Found I2C 0x");
           Serial.println(addr,HEX);
           }
      }
    }
  }
  if (trace) {
    Serial.println("\ndone");
    }
  return(portsFound);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// blink out an error code
void error(uint8_t errno) {
  while(errno != 0) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void sdCardInit() {
// see if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    Serial.println("Card init. failed!");
    error(2);
  }
}
//
//
void sdCardOpenNext() {
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
// create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
//
  tripFile = SD.open(filename, FILE_WRITE);
  if( ! tripFile ) {
    Serial.print("Couldnt create ");
    Serial.println(filename);
    error(3);
  }
  vruSayFileNumber();
  Serial.print("Writing to ");
  Serial.println(filename);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//     Various VRU Feedback functions
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void queueVoiceResponse(uint8_t vRec ) {
  vruSerial.print("Q = ");
  vruSerial.print(vRec, DEC);
  vruSerial.println(';');
  delay(50);
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruMainMenu() {
  delay(1500);
  queueVoiceResponse(193);                        //  Main Menu, Press A for Manual Recording, Press B for Auto Recording, Press C to play
                                                  //  back a trip file, press D to end current mode and return to the Main Menu
  queueVoiceResponse(254);                        //  VRU pause 250ms 
}
// 
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruManRecMode() {
  delay(1500);
  queueVoiceResponse(194);                 //   starting manual recording
  for (int a = 0; a < 3; a++) {
    queueVoiceResponse(254);               //   VRU pause 250ms 
    }
  queueVoiceResponse(195);                 //  Press Star to record a waypoint, Press D to end and return to the main menu
  queueVoiceResponse(254);                 //  VRU pause 250ms 
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruAutoRecMode() {
  delay(1500);
  queueVoiceResponse(196);                 //  starting auto recording, Press D to end and return to the main menu
  queueVoiceResponse(254);                 //   VRU pause 250ms 
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruPlayTripMode() {
  delay(1500);
  queueVoiceResponse(197);                    // To load trip file enter the two digit file number and press pound (#)
  queueVoiceResponse(254);                        //   VRU pause 250ms 
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruPressPoundAgain() {
  delay(2000);
  queueVoiceResponse(198);                   //  Press # again to start trip playback, Press D to end and return to the main menu.              
  queueVoiceResponse(254);                  //   VRU pause 250ms 
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruWayPointRecorded() {
  delay(500);
  queueVoiceResponse(199);                    //  Waypoint Recorded
  queueVoiceResponse(254);                   //   VRU pause 250ms 
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruWayPointReached() {
  delay(500);
  queueVoiceResponse(200);                // waypoint reached
  queueVoiceResponse(254);               //   VRU pause 250ms 
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruWayPointSkipped() {
  delay(500);
  queueVoiceResponse(40);                //   point 
  queueVoiceResponse(183);               //   skipped
  queueVoiceResponse(254);               //   VRU pause 250ms 
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruTripPlayStart() {
  delay(1500);
  queueVoiceResponse(201);                 //  starting trip file playback,  Press D to end and return to the main menu
  queueVoiceResponse(254);                        //   VRU pause 250ms 
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruTripFileReady() {
  delay(1500);
  queueVoiceResponse(202);                   //  trip file loaded and ready to play
  queueVoiceResponse(254);                   //   VRU pause 250ms 
  vruPressPoundAgain();
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruTripComplete() {
  delay(1500);
  queueVoiceResponse(78);                    // you have arrived
  queueVoiceResponse(254);                        //   VRU pause 250ms 
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruTripFileLoadErr() {
  queueVoiceResponse(254);                        //   VRU pause 250ms 
  queueVoiceResponse(203);                   //  trip file load error
  queueVoiceResponse(254);                        //   VRU pause 250ms 
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruTripFileNumErr() {
  queueVoiceResponse(254);                        //   VRU pause 250ms 
  queueVoiceResponse(204);                   //  trip file number error
  queueVoiceResponse(254);                        //   VRU pause 250ms 
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruEmptyWayPoint() {
  queueVoiceResponse(254);                        //   VRU pause 250ms 
  queueVoiceResponse(205);                   //  skipped recording way point
  queueVoiceResponse(254);                        //   VRU pause 250ms 
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruNotRecordingMode() {
  queueVoiceResponse(254);                        //   VRU pause 250ms 
  queueVoiceResponse(206);                   //  Recording Mode Error. 
  queueVoiceResponse(254);                        //   VRU pause 250ms 
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruSayFileNumber() {
  delay(500);
  queueVoiceResponse(207);                        //  File Number is 
  delay(500);
  int fn = (filename[6] - 48);
  queueVoiceResponse(fn);
  delay(100); 
  fn = (filename[7] - 48);
  queueVoiceResponse(fn);
  queueVoiceResponse(254);                        //   VRU pause 250ms 
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void convertGPSToDMS() {
  gpsLatError       = 0;
  gpsLonError       = 0;
  gpsLatDeg = (gpsLatitude / 100);                          //  divide GPS Latitude by 100 to get the degrees
  if (gpsLatDeg > 90) {
    gpsLatError = 1;                                        //  Type 1 Latitude Error
    }
  gpsLatMin = (gpsLatitude - (gpsLatDeg * 100));            //  get the MM portion of the GPS Latitude data by getting the remainder
  if (gpsLatMin > 59) {
    gpsLatError = (gpsLatError + 2);                        //  Type 2 Latitude Error
    }
  float tmpLatSec = (gpsLatitude - ((gpsLatDeg * 100) + gpsLatMin));
  gpsLatSec = (60.00 * tmpLatSec);
  if (gpsLatSec > 59.99) {
    gpsLatError = (gpsLatError + 4);                        //  Type 4 Latitude Error
    }
  if (gpsLat == 'S') {
    gpsLatDeg = (gpsLatDeg * -1);                           //  make negative if South of the equator
    }
//
  gpsLonDeg = (gpsLongitude / 100);                         //  divide GPS Longitude by 100 to get the degrees
  if (gpsLonDeg > 180) {
    gpsLonError = 1;                                        //  Type 1 Longitude Error
    }
  gpsLonMin = (gpsLongitude - (gpsLonDeg * 100));           //  get the MM portion of the GPS Longitude data by getting the remainder
  if (gpsLonMin > 59) {
    gpsLonError = (gpsLonError + 2);                        //  Type 2 Longitude Error
    }
  float tmpLonSec = (gpsLongitude - ((gpsLonDeg * 100) + gpsLonMin));
  gpsLonSec = (60.00 * tmpLonSec);
  if (gpsLonSec > 59.99) {
    gpsLonError = (gpsLonError + 4);                        //  Type 4 Longitude Error
    }
  if (gpsLon == 'W') {
    gpsLonDeg = (gpsLonDeg * -1);                           //  make negative if South of the equator
    }
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void computeDecimalDeg() {
  decimalDegLat = (abs(gpsLatDeg) + ((gpsLatitude - (abs(gpsLatDeg) * 100.00)) / 59.99));
  if (gpsLat == 'S') {
    decimalDegLat = (decimalDegLat * -1);
    }
  decimalDegLon = (abs(gpsLonDeg) + ((gpsLongitude - (abs(gpsLonDeg) * 100.00)) / 59.99));
  if (gpsLon == 'W') {
    decimalDegLon = (decimalDegLon * -1);
    }
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
void batteryCheck() {
  measuredVbat = analogRead(VBATPIN);
  measuredVbat *= 5;                           // Multiply by 5V, our reference voltage
  measuredVbat /= 1023;                        // convert to voltage
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void computeHeading() {
  qHead = abs(qW * 180.00);
  if ((qHead == 0.00) || (qHead == 180.00)) {
    return;
    }
  if (heading > 180.00) {
    qHead = (360.00 - qHead);
    }
}
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void checkForRTC() {
  if (!rtc.begin()) {
    if (trace) {
      Serial.println("Couldn't find RTC");
      error(4);
    }
    dateRefreshed = true;
    return;
  }
  //
  if (!rtc.initialized()) {
    if (trace) {
      Serial.println("RTC is NOT running!");
    }
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  getRTCData();
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void refreshRTC() {
  if (dateRefreshed) {
    return;
  }
  //
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  rtc.adjust(DateTime((2000 + navYear), navMonth, navDay, navHour, navMinute, navSeconds));
  getRTCData();
  dateRefreshed = true;
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void getRTCData() {
  if (dateRefreshed) {
    return;
  }
  DateTime now = rtc.now();
  //
  if (!trace) {
    return;
  }
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  //
  Serial.print(" since midnight 1/1/1970 = ");
  Serial.print(now.unixtime());
  Serial.print("s = ");
  Serial.print(now.unixtime() / 86400L);
  Serial.println('d');
}
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void getRogerNAVData() {
  while (bleCentral.available() ) {
    delay(2);
    size_t len = bleCentral.available();
    uint8_t sbuf[len];
    bleCentral.readBytes(sbuf, len);
    Serial.write(sbuf, len);
    Serial.flush();
    }
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void checkTripComplete() {
  if ((currMode == PLAYBACK) && (currPlaybackStep == COMPLETE)) {
    // Let user know trip is complete
    uint32_t currTime = millis();
    if ((currTime - timer) >= TRIP_COMPLETE_INT_MS) {
      timer = currTime;
      vruTripComplete();
      Serial.println("Trip complete!");
    }
  }
}

int findStartingWaypointInd() {
  // Get current location
  uint32_t currTime2 = millis();
  uint32_t elapsedTime = 0;
  char bleResp[50];
  int bleRespInd = 0;
  // Get GPS coordinates from NAV server
  int bytesWritten = bleCentral.println('o');
  bleCentral.flush();
  Serial.print("bytesWritten = ");
  Serial.println(bytesWritten);
  do {
    if (bleCentral.available()) {
      if (bleRespInd < 49) {
        bleResp[bleRespInd++] = bleCentral.read();
      }
      else {
        bleCentral.read();
      }
    }
    elapsedTime = millis() - currTime2;
  } while (elapsedTime < BLE_WAIT_MS); // Wait for 200ms max
  bleResp[bleRespInd] = '\0';
  //Serial.println(bleResp);

  while (bleCentral.available()) {
    bleCentral.read();
  }

  if (strlen(bleResp) < 45) {
    bleErrCnt++;
    return 0;
  }
  bleErrCnt = 0;

  float latDeg = 0.0;
  float lonDeg = 0.0;
  parseGPSString(&bleResp[0], &latDeg, &lonDeg);
  if ((latDeg!=0.0) && (lonDeg!=0.0) && (latDeg<90.0) && (latDeg>-90.0) && (lonDeg<180.0) && (lonDeg>-180.0)) {
    // Valid
    Serial.print("GPS: ");
    Serial.print(latDeg, 6);
    Serial.print(",");
    Serial.println(lonDeg, 6);
  }
  else {
    Serial.println("Invalid GPS, start at waypoint 0");
    return 0;
  }

  char line[31];
  int index = 0;
  Waypoint finalWaypoint = {seqNum:0, time:0, gpsLatDeg:0.0, gpsLonDeg:0.0};
  for (int i=0; i<totalWaypoints; i++) {
    while (playbackFile.available() && index < 30) {
      char c = playbackFile.read();
      //Serial.print(c);

      line[index++] = c;
      if (c == '\n') {
        //Serial.println(line);
        /*
        Serial.print("seqNum: ");
        Serial.print(i);
        Serial.print(", totalWaypoints: ");
        Serial.println(totalWaypoints);
        */
        line[--index] = '\0';
        //Serial.println(line);
        //Serial.println(strlen(line));

        if (i == totalWaypoints-1) {
          finalWaypoint = parseWaypoint(&line[0]);
          Serial.print("Final dest: ");
          Serial.print(finalWaypoint.gpsLatDeg, 6);
          Serial.print(",");
          Serial.println(finalWaypoint.gpsLonDeg, 6);
        }

        memset(line, 0, 31);
        index = 0;
        break;
      }
    }
  }
  playbackFile.seek(0);

  float distToDest = dist_between(latDeg, lonDeg, finalWaypoint.gpsLatDeg, finalWaypoint.gpsLonDeg);
  Serial.print("Distance to final destination: ");
  Serial.println(distToDest);

  index = 0;
  memset(line, 0, 31);
  Waypoint waypoint = {seqNum:0, time:0, gpsLatDeg:0.0, gpsLonDeg:0.0};
  for (int i=0; i<totalWaypoints; i++) {
    while (playbackFile.available() && index < 30) {
      char c = playbackFile.read();
      //Serial.print(c);

      line[index++] = c;
      if (c == '\n') {
        line[--index] = '\0';
        
        waypoint = parseWaypoint(&line[0]);
        float totalDist = dist_between(waypoint.gpsLatDeg, waypoint.gpsLonDeg, finalWaypoint.gpsLatDeg, finalWaypoint.gpsLonDeg);
        if (totalDist < distToDest) {
          Serial.print("Start at waypoint ");
          Serial.println(i);
          playbackFile.seek(0);

          return i;
        }

        memset(line, 0, 31);
        index = 0;
        break;
      }
    }
  }
  playbackFile.seek(0);

  // Get nearest waypoint
  /*
  for (int i=currWaypointInd; i<lastWaypointInd; i++) {
    Waypoint waypoint = waypoints[i];
    float totalDist = dist_between(waypoint.gpsLatDeg, waypoint.gpsLonDeg, destination.gpsLatDeg, destination.gpsLonDeg);
    if (totalDist < distToDest) {
      Serial.print("Start at waypoint ");
      Serial.println(currWaypointInd);

      return currWaypointInd;
    }
  }
  */

  Serial.print("Start at waypoint 0");
  return 0;
}

void computeTripInfo() {
  if ((currMode == PLAYBACK) && (currPlaybackStep == IN_PROGRESS) && (totalWaypointsInd <= totalWaypoints)) {
    uint32_t currTime = millis();

    if ((currTime - timer) >= PLAYBACK_INT_MS) {
      timer = currTime;
      Serial.println(currTime);

      // Get current location
      uint32_t currTime2 = millis();
      uint32_t elapsedTime = 0;
      char bleResp[50];
      int bleRespInd = 0;
      // Get GPS coordinates from NAV server
      int bytesWritten = bleCentral.println('o');
      bleCentral.flush();
      Serial.print("bytesWritten = ");
      Serial.println(bytesWritten);
      do {
        if (bleCentral.available()) {
          if (bleRespInd < 49) {
            bleResp[bleRespInd++] = bleCentral.read();
          }
          else {
            bleCentral.read();
          }
        }
        elapsedTime = millis() - currTime2;
      } while (elapsedTime < BLE_WAIT_MS); // Wait for 200ms max
      bleResp[bleRespInd] = '\0';
      
      while (bleCentral.available()) {
        bleCentral.read();
      }
      
      //Serial.println(bleResp);
      Serial.print("3");

      if (strlen(bleResp) < 45) {
        //Serial.println(strlen(bleResp));
        //Serial.println(bleResp);
        bleErrCnt++;
        return;
      }
      bleErrCnt = 0;
      Serial.print("4");

      float latDeg = 0.0;
      float lonDeg = 0.0;
      parseGPSString(&bleResp[0], &latDeg, &lonDeg);
      Serial.print("5");
      if ((latDeg!=0.0) && (lonDeg!=0.0) && (latDeg<90.0) && (latDeg>-90.0) && (lonDeg<180.0) && (lonDeg>-180.0)) {
        // Valid
        Serial.print("GPS: ");
        Serial.print(latDeg, 6);
        Serial.print(",");
        Serial.println(lonDeg, 6);
      }
      else {
        Serial.println("Invalid GPS");
        return;
      }

      Serial.print("Next waypoint: ");
      Serial.print(currWaypoint.seqNum);
      Serial.print("/");
      Serial.print(totalWaypoints-1);
      Serial.print(" - ");
      Serial.print(currWaypoint.gpsLatDeg, 6);
      Serial.print(",");
      Serial.println(currWaypoint.gpsLonDeg, 6);

      float distToWaypoint = dist_between(latDeg, lonDeg, currWaypoint.gpsLatDeg, currWaypoint.gpsLonDeg);
      Serial.print("Distance to next waypoint: ");
      Serial.print(distToWaypoint);
      Serial.print(", ");

      if (totalWaypointsInd < totalWaypoints - 1) {
        float distToNextWaypoint = dist_between(latDeg, lonDeg, nextWaypoint.gpsLatDeg, nextWaypoint.gpsLonDeg);
        Serial.print("Distance to next next waypoint: ");
        Serial.println(distToNextWaypoint);

        if (distToNextWaypoint <= distToWaypoint) {
          // Skip waypoint
          vruWayPointSkipped();
          Serial.println("Skip to next waypoint");
          totalWaypointsInd++;
          currWaypoint = nextWaypoint;
          nextWaypoint = loadNextWaypoint();

          return;
        }
      }

      float courseHeading = course_to(latDeg, lonDeg, currWaypoint.gpsLatDeg, currWaypoint.gpsLonDeg);
      Serial.print("Course heading: ");
      Serial.print(courseHeading);
      Serial.print(", ");

      // Get current heading
      currTime2 = millis();
      elapsedTime = 0;
      bleRespInd = 0;
      memset(bleResp, 0, 50);
      delay(100);
      
      bytesWritten = bleCentral.println('H');
      bleCentral.flush();
      Serial.print("bytesWritten = ");
      Serial.println(bytesWritten);
      do {
        if (bleCentral.available()) {
          if (bleRespInd < 49) {
            bleResp[bleRespInd++] = bleCentral.read();
          }
          else {
            bleCentral.read();
          }
        }
        elapsedTime = millis() - currTime2;
      } while (elapsedTime < BLE_WAIT_MS); // Wait for 200ms max
      bleResp[bleRespInd] = '\0';
      //Serial.println(bleResp);

      while (bleCentral.available()) {
        bleCentral.read();
      }

      if (strlen(bleResp) < 15) {
        bleErrCnt++;
        return;
      }
      bleErrCnt = 0;

      currHeading = parseHeading(&bleResp[0]);
      Serial.print("Heading: ");
      Serial.println(currHeading);

      if (currHeading >= 0.0f) {
        // Tell user what to do based on nav info
        computeUserAction(distToWaypoint, courseHeading, currHeading);
      }

      // Check if waypoint reached
      if (distToWaypoint <= 10.0) {
        Serial.println("Waypoint reached.");
        
        // Check if playback complete
        if (totalWaypointsInd == totalWaypoints) {
          currPlaybackStep = COMPLETE;
          playbackFile.close();
          //WaitForResponse("+++", "OK", 1000, modemResponse, 0);
          vruTripComplete();
          Serial.println("Trip complete!");
          currWaypoint.gpsLonDeg = 0.0;
          currWaypoint.gpsLatDeg = 0.0;
          currHeading = 0.0;
          
          return;
        }

        currWaypoint = nextWaypoint;
        if (totalWaypointsInd < totalWaypoints) {
          totalWaypointsInd++;
          nextWaypoint = loadNextWaypoint();
        }
        vruWayPointReached();
      }
      Serial.println();
    }
  }
}

void computeUserAction(float dist, float heading, float currHeading) {
  Direction turnDirection = FRONT;
  // Get difference between heading
  float headingDiff = heading - currHeading;
  float absHeadingDiff = abs(headingDiff);

  if (headingDiff > 0.0) {
    if (absHeadingDiff < 180.0) {
      // turn right
      turnDirection = RIGHT;
    }
    else {
      // turn left
      turnDirection = LEFT;
    }
  }
  else {
    if (absHeadingDiff < 180.0) {
      // turn left
      turnDirection = LEFT;
    }
    else {
      // turn right
      turnDirection = RIGHT;
    }
  }
  Serial.print("Off course by: ");
  Serial.print(absHeadingDiff, 2);
  Serial.print(", ");

  // How far off course they are
  if (absHeadingDiff <= 5.0) {
    queueVoiceResponse(189);                        //  Forward
    delay(200);
    // Minor error
    Serial.println("Keep straight");
  }
  else if (absHeadingDiff <= 10.0) {
    // Needs course adjustment
    queueVoiceResponse(189);                        //  Forward
    delay(200);
    Serial.println("Keep straight");
  }
  else if (absHeadingDiff <= 20.0) {
    // Needs course adjustment
    makeTurn(turnDirection);
  }
  else if (absHeadingDiff <= 30.0) {
    // Needs course adjustment
    makeTurn(turnDirection);
  }
  else if (absHeadingDiff <= 60.0) {
    // Off course
    makeTurn(turnDirection);
  }
  else if (absHeadingDiff <= 90.0) {
    // Far off course
    makeTurn(turnDirection);
  }
  else if (absHeadingDiff <= 120.0) {
    // Pretty far off course
    makeTurn(turnDirection);
  }
  else if (absHeadingDiff <= 150.0) {
    // Way off course
    makeTurn(turnDirection);
  }
  else {
    makeTurn(turnDirection);
  }
}

void makeTurn(Direction dir) {
  Serial.print("Turn ");
  switch (dir) {
    case LEFT:
      playLeft();
      Serial.println("left");
      break;
    case RIGHT:
      playRight();
      Serial.println("right");
      break;
    case FRONT:
      break;
    case BACK:
      break;
  }
}
//
//
float parseHeading(char *str) {
  int startInd = charArrIndexOf(str, '=') + 2;
  int decimalInd = charArrLastIndexOf(str, '.');
  int len = (decimalInd-startInd) + 3;
  char headingStr[9];
  int index = 0;
  /*
  Serial.println("parseHeading()");
  Serial.println(str);
  Serial.println(startInd);
  Serial.println(decimalInd);
  Serial.println(len);
  */
  if ((startInd<decimalInd) && (decimalInd<strlen(str)-2) && (len>0)) {
    while (index < len) {
      char c = str[startInd++];
      if (isdigit(c) || c == '.') {
        headingStr[index++] = c;
      }
    }
    headingStr[index] = '\0';

    return (float)atof(headingStr);
  }

  return -1.0f;
}
//
//
float parseRSSI(String str) {
  int startInd = str.lastIndexOf('=') + 2;
  int len = 4;
  String rssiStr = "";

  while (rssiStr.length() < len) {
    char c = str.charAt(startInd++);
    if (isDigit(c) || c == '-') {
      rssiStr += c;
    }
  }

  return rssiStr.toFloat();
}
//
//
float dist_between(float lat1, float long1, float lat2, float long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  float delta = radians(long1-long2);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}
//
//
float course_to(float lat1, float long1, float lat2, float long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  float dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float a1 = sin(dlon) * cos(lat2);
  float a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void setMode(Mode mode) {
  if (currMode == NONE) {
    currMode = mode;
    Serial.print("Mode set to ");
    switch (currMode) {
      case MANUAL_REC:
        Serial.println("Manual record");
        sdCardOpenNext();
        break;
      case AUTO_REC:
        Serial.println("Auto record");
        sdCardOpenNext();
        recordWaypoint();
        break;
      case PLAYBACK:
        Serial.println("Playback");
        break;
      case NONE:
        break;
    }
  }
  else {
    // Need to exit existing mode before setting new
    Serial.println("Error, exit current mode before setting new mode");
  }
}
//
//
void processNumInput(char num) {
  uint8_t vru = (num - 48);
  queueVoiceResponse(vru);
  if (currMode == PLAYBACK) {
    if (currPlaybackStep == SELECT_FILE) {
      if (numpadEntry.length() < 2) {
        numpadEntry += num;
        Serial.print("numpadEntry = ");
        Serial.println(numpadEntry);
      }
      else {
        // max valid entry is 99, error
        Serial.println("Error, max valid entry is 99");
      }
    }
  }
}
//
//
void confirmAction() {
  if ((currMode == PLAYBACK) && (currPlaybackStep == SELECT_FILE)) {
    if (numpadEntry.length() > 0) {
      totalWaypoints = 0;

      if (loadFileForPlayback()) {
        // success
        currPlaybackStep = FILE_LOADED;
        lastWaypointInd = 0;
        currWaypointInd = 0;
        totalWaypointsInd = 0;
        startingWaypointsInd = findStartingWaypointInd();

        //loadWaypointsFromFile();
        seekToStartingWaypoint();
        totalWaypointsInd = startingWaypointsInd;
        currWaypoint = loadNextWaypoint();
        totalWaypointsInd++;
        nextWaypoint = loadNextWaypoint();
        currPlaybackStep = WAYPOINTS_LOADED;
        vruTripFileReady();
      }
      else {
        vruTripFileLoadErr();
        // trip file load failed
      }
    }
    else {
       vruTripFileNumErr();
      // invalid file number entry
    }
  }
  else if ((currMode == PLAYBACK) && (currPlaybackStep == WAYPOINTS_LOADED)) {
    currPlaybackStep = IN_PROGRESS;
    //WaitForResponse("AT#SO=1\r", "CONNECT", 1000, modemResponse, 0);
    vruTripPlayStart();
  }
}
//
//
void seekToStartingWaypoint() {
  for (int i=0; i<startingWaypointsInd; i++) {
    while (playbackFile.available()) {
      char c = playbackFile.read();
      //Serial.print(c);

      if (c == '\n') {
        vruWayPointSkipped();
        Serial.print("Skipped waypoint ");
        Serial.println(i);

        break;
      }
    }
  }
}

void loadWaypointsFromFile() {
  String line = "";
  int seqNum = 0;

  if (totalWaypointsInd == 0) {
    seqNum = startingWaypointsInd;

    for (int i=0; i<startingWaypointsInd; i++) {
      while (playbackFile.available()) {
        char c = playbackFile.read();
        //Serial.print(c);

        line += c;
        if (c == '\n') {
          vruWayPointSkipped();
          Serial.print("Skipped waypoint ");
          Serial.println(i);

          line = "";
          break;
        }
      }
    }
  }
  else {
    seqNum = totalWaypointsInd;
  }

  line = "";
  int count = 0;
  for (int i=0; i<WAYPOINTS_INIT_LEN; i++) {
    while (playbackFile.available()) {
      char c = playbackFile.read();
      //Serial.print(c);

      line += c;
      if (c == '\n') {
        Waypoint waypoint = parseWaypoint(&line[0]);
        waypoint.seqNum = seqNum++;
        count++;
        //waypoints[i] = waypoint;

        Serial.print("Waypoint ");
        Serial.print(waypoint.seqNum);
        Serial.print(": ");
        Serial.print(waypoint.gpsLatDeg, 6);
        Serial.print(",");
        Serial.println(waypoint.gpsLonDeg, 6);
        line = "";
        break;
      }
    }
  }

  Serial.print(count);
  Serial.println(" wayloads loaded");
  if (seqNum == totalWaypoints) {
    playbackFile.close();
  }
}

Waypoint loadNextWaypoint() {
  char line[31];
  int index = 0;
  Waypoint waypoint = {seqNum:0, time:0, gpsLatDeg:0.0, gpsLonDeg:0.0};

  while (playbackFile.available() && index < 30) {
    char c = playbackFile.read();
    //Serial.print(c);

    line[index++] = c;
    if (c == '\n') {
      line[--index] = '\0';

      if (strlen(line) > 22) {
        waypoint = parseWaypoint(&line[0]);
        waypoint.seqNum = totalWaypointsInd;
        Serial.print("Loaded waypoint: ");
        Serial.print(totalWaypointsInd);
        Serial.print("/");
        Serial.print(totalWaypoints-1);
        Serial.print(" - ");
        Serial.print(waypoint.gpsLatDeg, 6);
        Serial.print(",");
        Serial.println(waypoint.gpsLonDeg, 6);
        break;
      }
      else {
        vruWayPointSkipped();
        Serial.print("Skip waypoint ");
        Serial.print(totalWaypointsInd);
        Serial.println(", bad data");
        
        totalWaypointsInd++;
        memset(line, 0, 31);
        index = 0;
      }
    }
  }

  return waypoint;
}
//
//
Waypoint parseWaypoint(char *str) {
  Waypoint waypoint = {seqNum:0, time:0, gpsLatDeg:0.0, gpsLonDeg:0.0};
  int startInd = 0;
  int delimInd = charArrIndexOf(str, ',');
  int len = delimInd - startInd;
  //Serial.println(startInd);
  //Serial.println(delimInd);
  //Serial.println(len);
  
  char temp[11];
  memcpy(temp, &str[startInd], len);
  temp[len] = '\0';
  //Serial.println(temp);
  // part 1 longitude
  waypoint.gpsLatDeg = (float)atof(temp);

  memset(temp, 0, 11);
  startInd = delimInd + 1;
  delimInd = charArrIndexOf(str, ':');
  len = delimInd - startInd;
  //Serial.println(startInd);
  //Serial.println(len);
  memcpy(temp, &str[startInd], len);
  temp[len] = '\0';
  //Serial.println(temp);
  // part 2 latitude
  waypoint.gpsLonDeg = (float)atof(temp);

  return waypoint;
}
//
//
bool loadFileForPlayback() {
  String filename = "CMDT00" + numpadEntry + ".TXT";
  if (!SD.exists(filename)) {
    return false;
  }

  playbackFile = SD.open(filename, FILE_READ);
  if (!playbackFile) {
    Serial.print("Couldnt open ");
    Serial.println(filename);
    return false;
  }
  Serial.print("Reading from ");
  Serial.println(filename);

  // Get # lines (waypoints)
  while (playbackFile.available()) {
    char c = playbackFile.read();
    //Serial.print(c);

    if (c == '\n') {
      totalWaypoints++;
    }
  }
  Serial.print("Total waypoints: ");
  Serial.println(totalWaypoints);

  numWaypointPages = totalWaypoints / WAYPOINTS_INIT_LEN;
  if (totalWaypoints % WAYPOINTS_INIT_LEN > 0) {
    numWaypointPages++;
  }
  Serial.print("# memory pages: ");
  Serial.println(numWaypointPages);

  // Seek back to start
  playbackFile.seek(0);

  return true;
}

void checkBLEError() {
  if (bleErrCnt >= 3) {
    digitalWrite(12, LOW);
    delay(250);
    digitalWrite(12, HIGH);
    
    Serial.println("Reset");
    bleErrCnt = 0;
  }
}
//
//
void recordWaypoint() {
  if ((currMode == MANUAL_REC) || (currMode == AUTO_REC)) {
    uint32_t currTime = millis();
    uint32_t elapsedTime = 0;
    Serial.println(currTime);
    
    char bleResp[50];
    int bleRespInd = 0;
    // Get GPS coordinates from NAV server
    int bytesWritten = bleCentral.println('o');
    bleCentral.flush();
    Serial.print("bytesWritten = ");
    Serial.println(bytesWritten);
    do {
      if (bleCentral.available()) {
        if (bleRespInd < 49) {
          bleResp[bleRespInd++] = bleCentral.read();
        }
        else {
          bleCentral.read();
        }
      }
      elapsedTime = millis() - currTime;
    } while (elapsedTime < BLE_WAIT_MS); // Wait for 200ms max
    bleResp[bleRespInd] = '\0';
    //Serial.println(bleResp);
    //Serial.println(strlen(bleResp));

    while (bleCentral.available()) {
      bleCentral.read();
    }
    Serial.print("3");

    if (strlen(bleResp) < 45) {
      bleErrCnt++;
      return;
    }
    bleErrCnt = 0;

    Serial.print("4");
    float latDeg = 0.0;
    float lonDeg = 0.0;
    parseGPSString(&bleResp[0], &latDeg, &lonDeg);
    //Serial.println(latDeg);
    //Serial.println(lonDeg);
    Serial.print("5");

    char temp1[11];
    char temp2[11];
    dtostrf(latDeg, 8, 6, temp1);
    dtostrf(lonDeg, 8, 6, temp2);
    char parsedCoord[25];
    bool waypointRecorded = false;
    if ((latDeg!=0.0) && (lonDeg!=0.0) && (latDeg<90.0) && (latDeg>-90.0) && (lonDeg<180.0) && (lonDeg>-180.0)) {
      // Valid
      sprintf(parsedCoord, "%s,%s", temp1, temp2);
      //Serial.println(parsedCoord);
      //Serial.println(strlen(parsedCoord));
      tripFile.print(parsedCoord);
      tripFile.flush();
      waypointRecorded = true;

      Serial.print("Waypoint: ");
      Serial.println(parsedCoord);
      Serial.println("Waypoint saved");
    }
    else {
      Serial.println("Invalid waypoint");
      return;
    }

    //currWaypoint.gpsLatDeg = 29.55;
    //currWaypoint.gpsLonDeg = -95.38;

    // Get current heading
    currTime = millis();
    elapsedTime = 0;
    bleRespInd = 0;
    memset(bleResp, 0, 50);
    delay(100);
    
    bytesWritten = bleCentral.println('H');
    bleCentral.flush();
    Serial.print("bytesWritten = ");
    Serial.println(bytesWritten);
    do {
      if (bleCentral.available()) {
        if (bleRespInd < 49) {
          bleResp[bleRespInd++] = bleCentral.read();
        }
        else {
          bleCentral.read();
        }
      }
      elapsedTime = millis() - currTime;
    } while (elapsedTime < BLE_WAIT_MS); // Wait for 200ms max
    bleResp[bleRespInd] = '\0';
    //Serial.println(bleResp);
    //Serial.println(strlen(bleResp));

    while (bleCentral.available()) {
      bleCentral.read();
    }

    if (strlen(bleResp) < 15) {
      bleErrCnt++;
      if (waypointRecorded) {
        tripFile.println();
        tripFile.flush();
      }
      return;
    }
    bleErrCnt = 0;
    
    currHeading = parseHeading(&bleResp[0]);
    Serial.print("Heading: ");
    Serial.println(currHeading);
    
    /*
    String saveString = parsedCoord + ":" + String(currHeading, 2);
    Serial.print("saveString: ");
    Serial.println(saveString);
    */
    
    if (currHeading >= 0.0f) {
      dtostrf(currHeading, 4, 2, temp1);
      //Serial.println(temp1);
      char currHeadingStr[8];
      sprintf(currHeadingStr, ":%s", temp1);
      //Serial.println(currHeadingStr);
      
      tripFile.print(currHeadingStr);
      tripFile.flush();
      vruWayPointRecorded();
    }
    else {
      vruEmptyWayPoint();
    }
    tripFile.println();
    tripFile.flush();
  }
  else {
    vruNotRecordingMode();
    //Serial.println("Error, not in recording mode");
  }
}
//
//
void autoRecordWaypoint() {
  uint32_t currTime = millis();
  if ((currTime - timer) >= AUTO_RECORD_INT_MS) {
    // timer elapsed, record waypoint if in auto record mode
    if (currMode == AUTO_REC) {
      recordWaypoint();
    }
    timer = currTime;
  }
}
//
//
void parseGPSString(char *gpsStr, float *latDeg, float *lonDeg) {
  int firstInd = charArrIndexOf(gpsStr, '=');
  int lastInd = charArrLastIndexOf(gpsStr, '=');
  /*
  Serial.println(strlen(gpsStr));
  Serial.println(gpsStr);
  Serial.println(firstInd);
  Serial.println(lastInd);
  */
  if ((firstInd<lastInd) && (lastInd<strlen(gpsStr)-10)) {
    // Valid, expect format DD.DDDDDD
    char lat[11];
    char lon[11];
    int index = 0;
    int startInd = firstInd + 2;
    //Serial.println(startInd);
    
    int len = (gpsStr[startInd] == '-') ? 10 : 9;
    //Serial.println(len);
    while(index < len) {
      char c = gpsStr[startInd++];
      //Serial.print(c);
      if (isdigit((int)c) || c == '.' || c == '-') {
        lat[index++] = c;
        //Serial.println(index);
      }
    }
    lat[index] = '\0';
    //Serial.println(lat);

    index = 0;
    startInd = lastInd + 2;
    len = (gpsStr[startInd] == '-') ? 10 : 9;
    while(index < len) {
      char c = gpsStr[startInd++];
      if (isdigit((int)c) || c == '.' || c == '-') {
        lon[index++] = c;
      }
    }
    lon[index] = '\0';
    //Serial.println(lon);

    *latDeg = (float)atof(lat);
    *lonDeg = (float)atof(lon);
  }
}

int charArrIndexOf(char *arr, char charToFind)
{
  int i;
  for (i=0; i<strlen(arr); i++) {
    if (arr[i] == charToFind) {
      return i;
    }
  }

  return -1;
}

int charArrLastIndexOf(char *arr, char charToFind)
{
  int i;
  for (i=strlen(arr); i>0; i--) {
    if (arr[i] == charToFind) {
      return i;
    }
  }

  return -1;
}
//
//
void chkForCMDInput() {
  keyPadInput = cmdKeyPad.getKey();
//
  if (keyPadInput){
    if (trace) {
      Serial.println(keyPadInput);
      }
    switch (keyPadInput) {
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
        processNumInput(keyPadInput);
        break;
      case 'A':
        // For entering manual record mode
        queueVoiceResponse(30);                                          // say "A"
        setMode(MANUAL_REC);
        vruManRecMode();
        break;
      case 'B':
        // For entering auto record mode
        queueVoiceResponse(31);                                          // say "B"
        setMode(AUTO_REC);
        vruAutoRecMode();
        break;
      case 'C':
        // For entering playback mode
        queueVoiceResponse(32);                                          // say "C"
        setMode(PLAYBACK);
        vruPlayTripMode();
        break;
      case 'D':
        queueVoiceResponse(33);                                          // say "D"
        // For exiting current mode
        // If recording, close files and cleanup
        if ((currMode == MANUAL_REC) || (currMode == AUTO_REC)) {
          recordWaypoint();
          tripFile.close();
        }
        currMode = NONE;
        currPlaybackStep = SELECT_FILE;
        numpadEntry = "";
        currWaypoint.gpsLonDeg = 0.0;
        currWaypoint.gpsLatDeg = 0.0;
        currHeading = 0.0;
        Serial.println("Mode reset");
        vruMainMenu();
        break;
      case '*':
        // For manually recording waypoints
        queueVoiceResponse(43);                                          // say "Star"
        recordWaypoint();
        vruWayPointRecorded();
        break;
      case '#':
        queueVoiceResponse(44);                                          // say "Pound"
        // For confirming actions
        confirmAction();
        break;
    }
  }
}

void sendLocationToFlow()
{
  uint32_t currTime = millis();
  if ((currTime - timer2) >= LOC_UPDATE_INT) {
    timer2 = currTime;

    if ((currWaypoint.gpsLonDeg != 0.0) && (currWaypoint.gpsLatDeg != 0.0)) {
      if (connectionGood) {
        StaticJsonBuffer<100> jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();
        root["iccid"] = iccid;
        root["lon"] = RawJson(String(currWaypoint.gpsLonDeg, 6));
        root["lat"] = RawJson(String(currWaypoint.gpsLatDeg, 6));
        root["heading"] = heading;

        char buff[root.measureLength() + 1];
        root.printTo(buff, sizeof(buff));
        String strBuff(buff);
        strBuff += "\n";

        String http_command = "POST " + (String)"/dd596c7308855/dc1280f4be05/4e543e946806914/in/flow/updateLocation" + " HTTP/1.1\r\n" +
        "Host: runm-east.att.io\r\n" +
        "Content-Type: application/json\r\n" +
        "Content-Length: " + strBuff.length() + "\r\n\r\n" + strBuff;

        WaitForResponse(http_command, "200 OK", 1000, modemResponse, 0);
      }
    }
  }
}

String parseICCID(String modemResp)
{
  int startInd = modemResp.indexOf(':') + 2;
  return modemResp.substring(startInd, startInd+20);
}

void checkModem() {
  if (millis() > MODEM_INIT_WAIT) {
    if (!modemReady) {
      Serial.println("Test AT command");
      if (!WaitForResponse("AT\r", "OK", 250, modemResponse, 0)) {
        return;
      }

      Serial.println("Reseting modem");
      if (!WaitForResponse("ATZ\r", "OK", 250, modemResponse, 0)) {
        return;
      }

      Serial.println("Turn on verbose error messages");
      if (!WaitForResponse("AT+CMEE=2\r", "OK", 250, modemResponse, 0)) {
        return;
      }

      //Serial.println("Check baud rate");
      //WaitForResponse("AT+IPR?\r", "OK", 250, modemResponse, 0);

      Serial.println("Enable SIM detect");
      if (!WaitForResponse("AT#SIMDET=1\r", "OK", 250, modemResponse, 0)) {
        return;
      }

      Serial.println("Get ICCID");
      if (!WaitForResponse("AT#CCID\r", "OK", 250, modemResponse, 0)) {
        return;
      }
      iccid = parseICCID(modemResponse);

      if (!WaitForResponse("AT#SCFG=1,1,1000,65535,600,50\r", "OK", 250, modemResponse, 0)) {
        return;
      }

      if (!WaitForResponse("AT#SGACT=1,0\r", "OK", 250, modemResponse, 0)) {
        return;
      }

      Serial.println("Setup PDP");
      if (!WaitForResponse("AT+CGDCONT=1,\"IP\",\"m2m.com.attz\"\r", "OK", 250, modemResponse, 0)) {
        return;
      }
      //WaitForResponse("AT%PDNSET=1,\"m2m.com.attz\",\"IP\"\r", "OK", 1000, modemResponse, 0);

      Serial.println("Check signal strength");
      if (!WaitForResponse("AT+CSQ\r", "OK", 250, modemResponse, 0)) {
        return;
      }

      //Serial.println("Check firmware version");
      //WaitForResponse("AT+CGMR\r", "OK", 500, modemResponse, 0);
      modemReady = true;
      //WaitForResponse("+++", "OK", 1000, modemResponse, 0);
    }
    else if (!connectionGood) {
      //Serial.println("Waiting for network connection");
      cellSerial.print("AT+CGREG?\r");
      cellSerial.flush();
      currentString = "";
      delay(250);

      // Read cellSerial port buffer1 for UART connected to modem and print that message back out to debug cellSerial over USB
      while(cellSerial.available() > 0)
      {
        //read incoming byte from modem
        char incomingByte = cellSerial.read();
        //write byte out to debug cellSerial over USB
        Serial.print(incomingByte);

        // add current byte to the string we are building
        currentString += char(incomingByte);

        // check currentString to see if network status is "0,1" or "0,5" which means we are connected
        if((currentString.substring(currentString.length()-3, currentString.length()) == "0,1") ||
           (currentString.substring(currentString.length()-3, currentString.length()) == "0,5"))
        {
          while(PrintModemResponse() > 0);  // consume rest of message once 0,1 or 0,5 is found

          Serial.println("Get IP");
          if (WaitForResponse("AT#SGACT=1,1\r", "OK", 5000, modemResponse, 0)) {
            if (WaitForResponse("AT#SD=1,0,80,\"runm-east.att.io\",0,1,0\r", "CONNECT", 2000, modemResponse, 0)) {
              connectionGood = true;
            }
          }
        }
      }
    }
  }
}

// sends a command to the modem, waits for the specified number of milliseconds,
// checks whether the modem response contains the expected response, and
// appends the remaining response characters to the out parameter respOut
// returns true if command received the expected response
bool SendModemCommand(String command, String expectedResp, int msToWait, String& respOut, byte b)
{
  int cmd_timeout = 0;
  if(b)
  {
    cellSerial.write(b);
  }
  else
  {
    cellSerial.print(command);
  }
  cellSerial.flush();        // just in case any characters weren't transmitted
  delay(msToWait);

  // wait for data to become available, but timeout eventually if no response is received
  while(!cellSerial.available())
  {
    cmd_timeout++;
    if (cmd_timeout == 10)
    {
      Serial.println("command timeout");
      //connectionGood = false;
      //modemReady = false;
      return false;
    }
    delay(10);
  }

  // read response from modem
  String resp = "";
  respOut = "";
  while(cellSerial.available() > 0)
  {
    resp += char(cellSerial.read());
    if(resp.endsWith(expectedResp))
    {
      respOut = resp;
      while(cellSerial.available() > 0)
        respOut += char(cellSerial.read());  // append remaining response characters (if any)
      return true;
    }
    else if (resp.endsWith("ERROR")) {
      ConsumeModemResponse();
      connectionGood = false;
      modemReady = false;
      delay(100);
      return false;
    }
  }
  respOut = resp;
  return false;
}

// repeatedly sends command to the modem until correct response is received
bool WaitForResponse(String command, String expectedResp, int msToWait, String& respOut, byte b)
{
  bool isExpectedResp;
  isExpectedResp = SendModemCommand(command, expectedResp, msToWait, respOut, b);
  Serial.println(respOut);
  ConsumeModemResponse();   // just in case any characters remain in RX buffer

  return isExpectedResp;
}

// empty read buffer
void ConsumeModemResponse()
{
  while(cellSerial.available())
    cellSerial.read();
}

// returns modem response as a String
String GetModemResponse()
{
  String resp = "";
  while(cellSerial.available() > 0)
  {
    resp += char(cellSerial.read());
  }
  return resp;
}

// consumes and prints modem response
int PrintModemResponse()
{
  String resp = "";
  while(cellSerial.available() > 0)
  {
    // read incoming modem response into temporary string
    resp += char(cellSerial.read());
  }
  Serial.println(resp);

  //return number of characters in modem response buffer -- should be zero, but some may have come in since last test
  return cellSerial.available();
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void chkForNAVServer() {
  delay(1000);
  uint32_t currTime3;
  uint32_t elapsedTime = 0;
  String bleResp = "";
  int bleConnectCount = 0;
  do {
    queueVoiceResponse(175);                          // connecting to the nav server
    delay(100);
    queueVoiceResponse(2);
//    delay(100);
    queueVoiceResponse(158);
    delay(200);
    queueVoiceResponse(174);
    delay(100);
    queueVoiceResponse(253);
    delay(500);
    elapsedTime = 0;
    currTime3 = millis();
    bleCentral.println('H');
    do {
      while (bleCentral.available()) {
        bleResp += (char)bleCentral.read();
       }
       elapsedTime = millis() - currTime3;
      } while (elapsedTime < BLE_WAIT_MS);             // Wait for 200ms max
      delay(250);
//
      elapsedTime = 0;
      bleResp = "";
      bleRSSI = -200;
      currTime3 = millis();
      bleCentral.println('R');
      do {
        while (bleCentral.available()) {
          bleResp += (char)bleCentral.read();
        }
        elapsedTime = millis() - currTime3;
      } while (elapsedTime < BLE_WAIT_MS);              // Wait for 200ms max
      float rssi = parseRSSI(bleResp);
      bleRSSI = rssi;
      if (bleRSSI > -120.00) {
        navConnected = true; }                         // Signal greater than -120dBm is good
      bleConnectCount++;
    } while (!navConnected && bleConnectCount < 5);
  delay(2000);
  if (navConnected) {
    queueVoiceResponse(176);                             //  Connected to ROGERNAV
    delay(100);
    queueVoiceResponse(2);
    queueVoiceResponse(133);
    } else {
    queueVoiceResponse(182);                            //   Bypassed Connecting to ROGERNAV
    delay(200);
    queueVoiceResponse(175);
    delay(100);
    queueVoiceResponse(2);
    queueVoiceResponse(133);
  }
  delay(1500);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void kbdVRUHapticCheck() {
  queueVoiceResponse(42);                       //   Press
  queueVoiceResponse(1);                        //   One (1)
  delay(100);
  queueVoiceResponse(4);                        //   For (or 4)
  queueVoiceResponse(67);                       //   Left
  do {
    chkForCMDInput();
    } while (keyPadInput != '1');
  delay(200);
  for (byte a = 0; a <= 2; a++) {
    playLeft();                                 //   Buzz Left
//    delay(100);
    }
  delay(800);
  queueVoiceResponse(253);
  queueVoiceResponse(42);                       //   Press
  queueVoiceResponse(3);                        //   Three (3)
  delay(100);
  queueVoiceResponse(4);                        //   For (or 4)
  queueVoiceResponse(66);                       //   Right
  do {
    chkForCMDInput();
    } while (keyPadInput != '3');
  delay(200);
  for (byte a = 0; a <= 2; a++) {
    playRight();                                 //   Buzz Right
//    delay(100);
    }
  delay(800);
  queueVoiceResponse(253);
  queueVoiceResponse(135);                       //   Congratulations
  delay(1000);                                   //   delay 1000ms
  queueVoiceResponse(104);                       //   System
  delay(200);
  queueVoiceResponse(46);                        //   On
  delay(1000);
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void configRogerCMD() {
  if (trace) {
    Serial.println("Starting CMD Config");
    }
  kbdVRUHapticCheck();
  chkForNAVServer();
  vruMainMenu();
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void testVRUCom() {
  if (!trace) {
    return;
    }
  while (Serial.available()) {
    long voiceLRec = Serial.parseInt();                        // get the data
    voiceRec = voiceLRec;
    queueVoiceResponse(voiceRec);
    Serial.print("Voice Record: ");
    Serial.println(voiceRec, DEC);
    Serial.flush();
    char qvr = Serial.read();
    if (qvr == '\n') {
      voiceRec = 999;
      break;
    }
   }
  voiceRec = 999;
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////
//
void testBLECom() {
  if (!trace) {
    return;
    }
  while (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == '\n') {
      break;
      }
    bleCentral.println(cmd);
//    bleCentral.write(cmd);
//    bleCentral.flush();
   }
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////
//
void testCellCom() {
  cellReq = false;
  if (!trace) {
    return;
    }
  while (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == '\n') {
      break;
      }
    cellSerial.write(cmd);
    cellSerial.flush();
    cellReq = true;
   }
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void configIOPins() {
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(250);
  digitalWrite(13, LOW);
  pinMode(41, OUTPUT);
  digitalWrite(41, LOW);
  delay(250);
  digitalWrite(41, HIGH);
  pinMode(43, OUTPUT);
  digitalWrite(43, LOW);
  pinMode(45, OUTPUT);
  digitalWrite(45, LOW);
  pinMode(47, OUTPUT);
  digitalWrite(47, LOW);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  delay(500);
  digitalWrite(11, HIGH);
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void configSerialPorts() {
//  trace = false;
  delay(250);
  if (trace) {
    Serial.begin(115200);                                                // start the Serial trace port
    delay(250);
    Serial.println(F("Starting Kayak Command System"));
    delay(250);
    }
  bleCentral.begin(115200);                                              // start the Bluetooth LE serial port
  delay(250);
  vruSerial.begin(115200);                                                 // start the Adafruit Music Maker serial port
  delay(250);
  cellSerial.begin(115200);                                              // start the Adafruit Music Maker serial port
  delay(250);
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void chkHapticDrv() {
  byte devCount = scanI2CBus();
  if (devCount <= 1) {
    if (trace) {
      Serial.println("No HAPTIC DRIVERS FOUND");
      }
    } else {
      hapBegin();
      readyBuzz();
      if (trace) {
        Serial.println("Haptic DRV Dev");
        }}
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void setup() {
  configSerialPorts();               // initialize serial:
  configIOPins();                    // configure the IO pins
  checkForRTC();                     // Check for an RTC
  delay(250);
  sdCardInit();                      // initialize the SD card
  delay(7000);
  chkHapticDrv();
  digitalWrite(11, LOW);
  delay(5000);
  digitalWrite(11, HIGH);
  delay(2000);
  if (trace) {
    Serial.println("Ready!");
    delay(250);
    }
//  trace = false;
  delay(5000);
  digitalWrite(11, HIGH);
  configRogerCMD();
//  if (navConnected) {
//    refreshRTC();
  //configRogerCMD();
  //wdt_enable(WDTO_8S);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void loop() {
//  testVRUCom();
  checkModem();
  sendLocationToFlow();
//  testBLECom();
//  testCellCom();
  chkForCMDInput();
  autoRecordWaypoint();
  computeTripInfo();
  checkTripComplete();
  checkBLEError();
  //wdt_reset();
}
//
//



