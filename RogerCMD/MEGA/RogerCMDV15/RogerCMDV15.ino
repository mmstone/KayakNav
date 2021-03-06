////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  RogerCMDV15 - Updated to provide GPS coordinates verbally with the push of the star "*" button.  Also added code to provide
//                degrees variance from target verbally in addition to "Left" or "Right".
//
//  Roger Command interfaces to the RogerNAV Navigational Subsystem via Bluetooth LE. Commands and Navigational data can
//  be entered via one keypad:
//      1)  4x4 Keypad (used for UI/user commands)- REMAPPED
//
//  Along with the keypad, the main Arduino MEGA 2560 microcomputer is also connected to the following devices:
//      a)  Micro SD Card Adapter
//      b)  Bluetooth LE Interface
//      c)  Serial Rx/TX Connection to an Adafruit Feather M0 Proto connected to an Adafruit Music Maker Featherwing
//          via an Adafruit Feather Doubler - acting as a VRU
//      d)  Serial HW Serial Interface used by a cell modem for IoT/Geotracking capability
//      e)  REMOVED I2C Multiplexer, two 2605 Haptic drivers, and Haptic pancake buzzers
//
//  Feature Updates:
//  CMD14:  Updated to allow for 3 digit file numbers and ability to delete a selected file
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  #includes
#include <Arduino.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <SD.h>
#include <Key.h>
#include <Wire.h>
#include <Keypad.h>
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
#define LOWBATPIN               A8           // Low Bat Pin.  Goes LOW when the main battery goes below 3.14V
#define VERBOSE_MODE            true         // If set to 'true' enables debug output
#define BUFSIZE                 255          // Size of the read buffer for incoming data
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
//+
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
uint32_t timer         = 0;            // unsigned 32bit interger variable called timer assigned to the millisecond function
uint32_t timer2        = 0;
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
float decimalDegLat    = 0;            // Latitude in decimal degrees (DD.DDDDDD)
float gpsLatitude      = 0;            // GPS Latitude (DDMM.MMMM)
int gpsLatDeg          = 0;            // GPS Latitude Degrees (+/- 90)
uint16_t gpsLatMin     = 0;            // GPS Latitude Minutes (0 - 59)
float gpsLatSec        = 0;            // GPS Latitude Seconds (0 - 59.59)
float decimalDegLon    = 0;            // Longitude in decimal degrees (DDD.DDDDDD)
float gpsLongitude     = 0;            // GPS Longitude (DDDMM.MMMM)
int gpsLonDeg          = 0;            // GPS Longitude Degrees (+/- 179.99)
uint16_t gpsLonMin     = 0;            // GPS Longitude Degrees (0 - 59)
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
//int utcOffset          = 0;           // offset for local time
//int dstAdjust          = 0;           // adjustment for daylight saving time
int mainVolt           = 0;             //  main battery voltage
float measuredVbat     = 0;             // float to hold the measured batttery voltage
const char navName[11] = "RogerNAV01";  // name of the bluetooth LE nav module
const char cmdName[11] = "RogerCMD01";  // name of the bluetooth LE command module
//const int bleBuffSize  = 127;          // size of bluetooth buffer size
int bleRSSI             = 0;             // BLE Signal Strength
int lowBatCount         = 0;             //  used to control how often the low battery warning is sounded
int battTraceCount      = 9000;          // used to show battery value every 10 minutes
byte badConnectionCount = 0;             // used to control cell modem calls if there is no connection
uint16_t reCheckCellCount = 0;           // used to automatically try the cell modem connection when it has been disconnected
//
//
const byte ROWS = 4;                   //four rows
const byte COLS = 4;                   //four columns
char keys[ROWS][COLS] = {
  {'1', '4', '7', '*'},
  {'2', '5', '8', '0'},
  {'3', '6', '9', '#'},
  {'A', 'B', 'C', 'D'}
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
boolean navConnected      = false;
boolean cellReq           = false;
boolean trace             = true;
boolean logPlayback       = true;
boolean modemReady        = false;
boolean connectionGood    = false;
boolean mainBatteryLow    = false;
boolean skipLocationSend  = false;
//
//
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
char filename[15] = "CMDT0000.TXT";
//
//
//
Keypad cmdKeyPad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
//
enum Mode {
  RECORD,
  MANUAL_REC,
  AUTO_REC,
  PLAYBACK,
  MR_BEEP,
  NONE
};
//
enum PlaybackStep {
  CHOOSE_STEP,
  SELECT_FILE,
  FILE_LOADED,
  FILE_DELETE,
  WAYPOINTS_LOADED,
  IN_PROGRESS,
  COMPLETE
};
//
enum Direction {
  LEFT,
  RIGHT,
  FRONT,
  BACK
};
//
typedef struct waypoint {
  int seqNum;
  uint32_t time;
  float gpsLatDeg;
  float gpsLonDeg;
} Waypoint;
//
Mode currMode = NONE;
String numpadEntry = "";
String currentString = "";
String modemResponse = "";
String iccid = "";
//PlaybackStep currPlaybackStep = SELECT_FILE;
PlaybackStep currPlaybackStep = CHOOSE_STEP;
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
Waypoint finalWaypoint;
float currHeading = 0;
float currSpeed = 0;
uint32_t currHeadingAcqTime = 0;
uint32_t currSpeedAcqTime = 0;
int startingWaypointsInd = 0;
int totalWaypointsInd = 0;
int numWaypointPages = 0;
int bleErrCnt = 0;
float mrBeepHeading = 0;
unsigned int headingVar = 0;
boolean mrBeepHeadingSet = false;
//
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Logic and Function Routines
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// blink out an error code
//
void error(uint8_t errno) {
  while (errno != 0) {
    uint8_t i;
    for (i = 0; i < errno; i++) {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
    for (i = errno; i < 10; i++) {
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void sdCardOpenNext() {
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i / 10;
    filename[7] = '0' + i % 10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  //
  tripFile = SD.open(filename, FILE_WRITE);
  if ( ! tripFile ) {
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void sdDeleteTripFile() {
  String delFilename = "CMDT0" + numpadEntry + ".TXT";
  if (!SD.exists(delFilename)) {
      delay(350);
      queueVoiceResponse(204);                        //  Trip File Number Entered is Invalid
      delay(550);
      currPlaybackStep = CHOOSE_STEP;
      return;
    }
  if (!SD.remove(delFilename)) {
      delay(350);
      queueVoiceResponse(204);                        //  Trip File Number Entered is Invalid
      delay(550);
    } else {
      strcpy(filename, delFilename.c_str());
      delay(750);
      queueVoiceResponse(215);                        //  Trip File
      delay(1750);
      uint8_t fn = (filename[5] - 48);                //  Number
      queueVoiceResponse(fn);
      delay(450);
      fn = (filename[6] - 48);
      queueVoiceResponse(fn);
      delay(450);
      fn = (filename[7] - 48);
      queueVoiceResponse(fn);
      delay(450);
      queueVoiceResponse(216);                        //  Deleted
      delay(200);
      Serial.print("Trip File Deleted: ");
      Serial.println(filename);
      }
  currPlaybackStep = CHOOSE_STEP;
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//     Various VRU Feedback functions
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void turnRight() {
  if (trace) {
    Serial.println(" Right");
    }
  delay(600);
  queueVoiceResponse(66);                       //   Right
  delay(150);
  vruSayHeadingVariance();
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void turnLeft() {
  if (trace) {
    Serial.println(" Left");
    }
  delay(600);
  queueVoiceResponse(67);                        //  Left
  delay(150);
  vruSayHeadingVariance();
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void queueVoiceResponse(uint8_t vRec ) {
  vruSerial.print("Q = ");
  vruSerial.print(vRec, DEC);
  vruSerial.println(';');
  //  delay(50);
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruMainMenu() {
  delay(1500);
  queueVoiceResponse(193);                        //  Main Menu, Press A for Mr. Beep or Manual Recording, Press B for Auto Recording, Press C to play
                                                  //  back or delete a trip file, press D to end current mode and return to the Main Menu
  delay(500);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruBeepOrManMode() {
  delay(1000);
  queueVoiceResponse(208);                       //  Press 1 to start Mr. Beep Mode, or Press 2 to start Manual Recording Mode
  delay(3000);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruMrBeepMode() {
  delay(1000);
  queueVoiceResponse(209);                     //  Starting Mr. Beep.  Point your kayak in the direction you want and press 1 to hear your heading.
                                               //  Press Pound once you are on the right heading to confirm and start your trip
  delay(5000);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruHeadConfirmAndStart() {
  delay(750);
  queueVoiceResponse(188);                //  Confirmed
  delay(2500);
  queueVoiceResponse(76);                 //  Starting
  delay(750);
  queueVoiceResponse(62);                 //  Trip
  delay(2000);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruManRecMode() {
  delay(1000);
  queueVoiceResponse(194);                 //   starting manual recording
  delay(2500);
  queueVoiceResponse(195);                 //  Press Star to record a waypoint, Press D to end and return to the main menu
  delay(4500);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruAutoRecMode() {
  delay(1000);
  queueVoiceResponse(196);                 //  starting auto recording, Press D to end and return to the main menu
  delay(5500);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruPlayTripMode() {
  delay(1000);
  queueVoiceResponse(197);                 //   To load a trip file enter the three digit file number and press pound (#)
  delay(4500);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruPressPoundAgain() {
  delay(1500);
  queueVoiceResponse(198);                   //  Press # again to start trip playback, Press D to end and return to the main menu.
  delay(4500);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruWayPointRecorded() {
  delay(1000);
  queueVoiceResponse(199);                    //  Waypoint Recorded
  delay(1500);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruWayPointReached() {
  delay(1000);
  queueVoiceResponse(200);                   // waypoint reached
  delay(1500);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruWayPointSkipped() {
  delay(750);
  queueVoiceResponse(212);                  //   skipping waypoint
  delay(1350);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruTripFileSelect() {
  delay(750);
  queueVoiceResponse(213);                 //  Press 1 to select and play back a trip file, Press 2 to select and delete a trip file
  delay(5500);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruTripFileDelete() {
  delay(750);
  queueVoiceResponse(214);                 //  To delete a trip file, enter the three digit file number and press Pound
  delay(5500);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruTripPlayStart() {
  delay(750);
  queueVoiceResponse(201);                 //  starting trip file playback,  Press D to end and return to the main menu
  delay(5500);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruTripFileReady() {
  delay(1000);
  queueVoiceResponse(202);                   //  trip file loaded and ready to play
  delay(1000);
  vruPressPoundAgain();
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruTripComplete() {
  delay(500);
  queueVoiceResponse(78);                    // you have arrived
  delay(1500);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruTripFileLoadErr() {
  delay(1000);
  queueVoiceResponse(203);                   //  trip file load error
  delay(2500);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruInvalidEntry() {
  delay(500);
  queueVoiceResponse(84);                    //  entry
  delay(350);
  queueVoiceResponse(107);                   //  error
  delay(250);
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruTripFileNumErr() {
  delay(1000);
  queueVoiceResponse(204);                   //  trip file number error
  delay(3500);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruEmptyWayPoint() {
  delay(1000);
  queueVoiceResponse(205);                        //  skipped recording way point
  delay(3500);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruNotRecordingMode() {
  delay(1000);
  queueVoiceResponse(206);                        //  Recording Mode Error.
  delay(2500);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruInvalidModeEntry() {
  delay(750);
  queueVoiceResponse(103);                        //  Incorrect
  delay(350);
  queueVoiceResponse(171);                        //  Mode
  delay(200);
  queueVoiceResponse(84);                         //  Entry
  delay(2500);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruCellModemConnected() {
  delay(1500);
  queueVoiceResponse(106);                         //  Data
  delay(550);
  queueVoiceResponse(113);                         //  Signal
  delay(250);
  queueVoiceResponse(69);                          //  Up
  delay(2500);
}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruCellModemDown() {
  delay(1500);
  queueVoiceResponse(106);                         //  Data
  delay(550);
  queueVoiceResponse(113);                         //  Signal
  delay(250);
  queueVoiceResponse(70);                          //  Down
  delay(2500);
}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruBluetoothError() {
  delay(1500);
  queueVoiceResponse(180);                        //  Bluetooth
  delay(150);
  queueVoiceResponse(106);                         //  Data
  delay(450);
  queueVoiceResponse(107);                         //  Error
  delay(1500);
}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruFileEntryError() {
  delay(1500);
  queueVoiceResponse(160);                        //  File
  delay(150);
  queueVoiceResponse(84);                         //  Entry
  delay(450);
  queueVoiceResponse(107);                        //  Error
  delay(1500);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruModeExitError() {
  delay(1500);
  queueVoiceResponse(57);                         //  End
  delay(150);
  queueVoiceResponse(184);                        //  Current
  delay(250);
  queueVoiceResponse(171);                        //  Mode
  delay(1500);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruDistToDestError(float dtdError) {
  delay(1500);
  queueVoiceResponse(94);                        //  Estimated
  delay(450);
  queueVoiceResponse(93);                        //  Distance
  delay(550);
  queueVoiceResponse(107);                       //  Error
  delay(350);
  if (dtdError < 4000000000.00F) {               //  Say the error if not over 4B meters
    vruSayDistToDest(dtdError);
  }
  delay(1000);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruSayFileNumber() {
  delay(500);
  queueVoiceResponse(207);                        //  File Number is
  delay(2500);
  uint8_t fn = (filename[5] - 48);
  queueVoiceResponse(fn);
  delay(350);
  fn = (filename[6] - 48);
  queueVoiceResponse(fn);
  delay(250);
  fn = (filename[7] - 48);
  queueVoiceResponse(fn);
  delay(1500);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruBatteryLow() {
  if (lowBatCount < 1200) {                       // approximately 2 minutes
    return;
  }
  delay(1500);
  queueVoiceResponse(110);                        //  Battery
  delay(750);
  queueVoiceResponse(111);                        //  Low
  delay(550);
  Serial.print("Computed VRU Voltage: ");
  Serial.println(measuredVbat, DEC);
  if (!mainBatteryLow) {
    lowBatCount = 0;
  }
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruMainBatteryLow() {
  if (lowBatCount < 1000) {
    return;
  }
  delay(1500);
  queueVoiceResponse(168);                        //  Main
  delay(550);
  queueVoiceResponse(110);                        //  Battery
  delay(350);
  queueVoiceResponse(111);                        //  Low
  delay(550);
  lowBatCount = 0;
  Serial.println("Main Battery Low");
  Serial.print("Main Battery Voltage: ");
  Serial.println(mainVolt);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruSayHeading() {
  if (currHeading == 0.0) {
    getCurrHeading();
    mrBeepHeading = currHeading;
  }
  boolean leadingZero = true;
  delay(1500);
  queueVoiceResponse(37);                        //  Heading
  delay(750);
  uint16_t sayHeading = currHeading;
  uint16_t fn = (sayHeading / 100);
  if (fn != 0) {
    delay(150);
    queueVoiceResponse(fn);
  }
  sayHeading = (sayHeading % 100);
  fn = (sayHeading / 10);
  delay(150);
  if (leadingZero && (fn == 0)) {
    delay(1);
  } else {
    queueVoiceResponse(fn);
    leadingZero = false;
    delay(150);
    }
  fn = (sayHeading % 10);
  delay(350);
  queueVoiceResponse(fn);
  delay(650);
  queueVoiceResponse(38);                         //  Degrees
  delay(1500);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruSayDistToDest(float distToDestination) {
  boolean leadingZero = true;
  unsigned long msdDTD = distToDestination;
  delay(1500);
  queueVoiceResponse(94);                             //  Estimated
  delay(350);
  //  queueVoiceResponse(95);                         //  Remaining
  //  delay(850);
  queueVoiceResponse(93);                             //  Distance
  delay(450);
//
  unsigned long fn = (msdDTD / 1000000000);           //  Divide
  if (fn != 0) {
    leadingZero = false;
    queueVoiceResponse(fn);
    delay(200);
  }
  msdDTD = (msdDTD % 1000000000);                     // Get Remainder
//
  fn = (msdDTD / 100000000);                          // Divide Again
  if (leadingZero && (fn == 0)) {
    delay(1);
  } else {
    queueVoiceResponse(fn);
    leadingZero = false;
    delay(150);
  }
  msdDTD = (msdDTD % 100000000);                     // Get Remainder
//  
  fn = (msdDTD / 10000000);                          // Divide Again
  if (leadingZero && (fn == 0)) {
    delay(1);
  } else {
    queueVoiceResponse(fn);
    leadingZero = false;
    delay(150);
  }
  msdDTD = (msdDTD % 10000000);                      // Get Remainder
//
  fn = (msdDTD / 1000000);                           // Divide Again
  if (leadingZero && (fn == 0)) {  
    delay(1);
  } else {
    queueVoiceResponse(fn);
    leadingZero = false;
    delay(150);
  }
  msdDTD = (msdDTD % 1000000);                      // Get Remainder
//  
  fn = (msdDTD / 100000);                           // Divide Again
  if (leadingZero && (fn == 0)) {
    delay(1);
  } else {
    queueVoiceResponse(fn);
    leadingZero = false;
    delay(150);
  }
  msdDTD = (msdDTD % 100000);                      // Get Remainder
//  
  fn = (msdDTD / 10000);                           // Divide Again
  if (leadingZero && (fn == 0)) {
    delay(1);
  } else {
    queueVoiceResponse(fn);
    leadingZero = false;
    delay(150);
  }
  msdDTD = (msdDTD % 10000);                      // Get Remainder
//  
  fn = (msdDTD / 1000);                           // Divide Again
  if (leadingZero && (fn == 0)) {
    delay(1);
  } else {
    queueVoiceResponse(fn);
    leadingZero = false;
    delay(150);
  }
  msdDTD = (msdDTD % 1000);                      // Get Remainder
//  
  fn = (msdDTD / 100);                           // Divide Again
  if (leadingZero && (fn == 0)) {
    delay(1);
  } else {
    queueVoiceResponse(fn);
    leadingZero = false;
    delay(150);
  }
  msdDTD = (msdDTD % 100);                      // Get Remainder
//  
  fn = (msdDTD / 10);                           // Divide Again
  if (leadingZero && (fn == 0)) {
    delay(1);
  } else {
    queueVoiceResponse(fn);
    leadingZero = false;
    delay(150);
  }
  msdDTD = (msdDTD % 10);                       // Get Remainder
//  
  delay(150);
  queueVoiceResponse(msdDTD);
  delay(300);
  queueVoiceResponse(117);                     //  Meters
  delay(1500);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruSaySpeed() {
  boolean leadingZero = true;
  delay(1500);
  queueVoiceResponse(49);                        //  Speed is
  delay(550);
  queueVoiceResponse(172);                       //
  delay(150);
  uint16_t msdSpeed = currSpeed;
  uint8_t lsdSpeed = ((currSpeed - msdSpeed) * 10);
  int fn = (msdSpeed / 100);
  if (fn != 0) {
    leadingZero = false;
    queueVoiceResponse(fn);
    delay(150);
  }
  msdSpeed = (msdSpeed % 100);
  fn = (msdSpeed / 10);
  if (leadingZero && (fn == 0)) {
    delay(1);
  } else {
    queueVoiceResponse(fn);
    leadingZero = false;
    delay(150);
  }
  fn = (msdSpeed % 10);
  queueVoiceResponse(fn);
  delay(250);
  queueVoiceResponse(40);                        // Point
  delay(350);
  queueVoiceResponse(lsdSpeed);
  delay(150);
  queueVoiceResponse(120);                       //  Knots
  delay(1000);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruSayGPSLocation() {
  getGPSLocation();
  vruSayLatitude();
  vruSayLongitude();
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruSayLatitude() {
  delay(150);
  queueVoiceResponse(96);                          //  Latitude
  delay(1000);
  vruSayLatDegrees();
  vruSayLatMinutes();
  vruSayLatSeconds();
  delay(250);
  if (gpsLatDeg > 0) {
    queueVoiceResponse(98);                       //  North
    } else {
    queueVoiceResponse(99);                       //  South
    }
  delay(2500);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruSayLatDegrees() {
  delay(1500);
  uint16_t degLat = abs(gpsLatDeg);
  uint16_t fn = abs(degLat / 10);
  if (fn != 0) {
    delay(150);
    queueVoiceResponse(fn);
  }
  fn = (degLat % 10);
  delay(150);
  queueVoiceResponse(fn);
  delay(350);
  queueVoiceResponse(38);                         //  Degrees
  delay(1500);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruSayLatMinutes() {
  delay(1500);
  uint16_t minLat = abs(gpsLatMin);
  uint16_t fn = abs(minLat / 10);
  if (fn != 0) {
    delay(150);
    queueVoiceResponse(fn);
  }
  fn = (minLat % 10);
  delay(150);
  queueVoiceResponse(fn);
  delay(650);
  queueVoiceResponse(87);                         //  Minutes
  delay(1500);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruSayLatSeconds() {
  delay(1500);
  uint16_t msdSec = abs(gpsLatSec);
  uint8_t lsdSec = ((abs(gpsLatSec) - msdSec) * 100);
  int fn = (msdSec / 10);
  if (fn != 0) {
    queueVoiceResponse(fn);
    delay(150);
    }
  fn = (msdSec % 10);
  queueVoiceResponse(fn);
  delay(150);
  queueVoiceResponse(40);                        // Point
  delay(350);
  fn = (lsdSec / 10);
  queueVoiceResponse(fn);
  delay(350);
  fn = (lsdSec % 10);
  queueVoiceResponse(fn);
  delay(350);
  queueVoiceResponse(88);                       //  Seconds
  delay(1000);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruSayLongitude() {
  delay(150);
  queueVoiceResponse(97);                          //  Longitude
  delay(1000);
  vruSayLongDegrees();
  vruSayLongMinutes();
  vruSayLongSeconds();
  delay(250);
  if (gpsLonDeg > 0) {
    queueVoiceResponse(100);                       //  East
    } else {
    queueVoiceResponse(101);                       //  West
    }
  delay(2500);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruSayLongDegrees() {
  boolean leadingZero = true;
  delay(1500);
  uint16_t msdDeg = abs(gpsLonDeg);
  int fn = (msdDeg / 100);
  if (fn != 0) {
    leadingZero = false;
    queueVoiceResponse(fn);
    delay(150);
  }
  msdDeg = (msdDeg % 100);
  fn = (msdDeg / 10);
  if (leadingZero && (fn == 0)) {
    delay(1);
  } else {
    queueVoiceResponse(fn);
    leadingZero = false;
    delay(150);
  }
  fn = (msdDeg % 10);
  queueVoiceResponse(fn);
  delay(350);
  queueVoiceResponse(38);                         //  Degrees
  delay(1500);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruSayLongMinutes() {
  delay(1500);
  uint16_t minLon = abs(gpsLonMin);
  uint16_t fn = abs(minLon / 10);
  if (fn != 0) {
    delay(150);
    queueVoiceResponse(fn);
  }
  fn = (minLon % 10);
  delay(150);
  queueVoiceResponse(fn);
  delay(650);
  queueVoiceResponse(87);                         //  Minutes
  delay(1500);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruSayLongSeconds() {
  delay(1500);
  uint16_t msdSec = abs(gpsLonSec);
  uint8_t lsdSec = ((abs(gpsLonSec) - msdSec) * 100);
  int fn = (msdSec / 10);
  if (fn != 0) {
    queueVoiceResponse(fn);
    delay(150);
  }
  fn = (msdSec % 10);
  queueVoiceResponse(fn);
  delay(150);
  queueVoiceResponse(40);                        // Point
  delay(350);
  fn = (lsdSec / 10);
  queueVoiceResponse(fn);
  delay(350);
  fn = (lsdSec % 10);
  queueVoiceResponse(fn);
  delay(350);
  queueVoiceResponse(88);                       //  Seconds
  delay(1000);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void vruSayHeadingVariance() {
  if ((currMode == PLAYBACK) || (currMode == MR_BEEP)) {
    boolean leadingZero = true;
    delay(750);
    if (headingVar > 180) {
      headingVar = 360 - headingVar;
      }
    uint16_t sayHeading = headingVar;
    uint16_t fn = (sayHeading / 100);
    if (fn != 0) {
      leadingZero = false;
      delay(150);
      queueVoiceResponse(fn);
      }
    sayHeading = (sayHeading % 100);
    fn = (sayHeading / 10);
    delay(150);
    if (leadingZero && (fn == 0)) {
      delay(1);
    } else {
      queueVoiceResponse(fn);
      leadingZero = false;
      delay(150);
      }
    fn = (sayHeading % 10);
    delay(350);
    queueVoiceResponse(fn);
    delay(650);
    queueVoiceResponse(38);                         //  Degrees
    delay(1500);
  }
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void getGPSLocation() {
  uint32_t currTime = millis();
  uint32_t elapsedTime = 0;
  char bleResp[50];
  int bleRespInd = 0;
  // Get GPS coordinates from NAV server
  int bytesWritten = bleCentral.println('o');
  bleCentral.flush();
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

  while (bleCentral.available()) {
    bleCentral.read();
    }

  if (strlen(bleResp) < 43) {
    bleErrCnt++;
    return;
    }
  bleErrCnt = 0;
  float latDeg = 0.0;
  float lonDeg = 0.0;
  parseGPSString(&bleResp[0], &latDeg, &lonDeg);
//
  gpsLatError = 0;
  gpsLonError = 0;
  gpsLatDeg = latDeg;                                       //  move from float to int to get degrees
  if (abs(gpsLatDeg) > 90) {
    gpsLatError = 1;                                        //  Type 1 Latitude Error
    }
  float tmpLatMin = abs((latDeg - gpsLatDeg) * 60);        //  subtract to get the Min/Sec portion and multiply by 60 for Min
  gpsLatMin = tmpLatMin;                   
  if (gpsLatMin > 59) {
    gpsLatError = (gpsLatError + 2);                        //  Type 2 Latitude Error
    }
  if (latDeg >= 0) {
    gpsLatSec = abs((((latDeg - gpsLatDeg) * 60) - gpsLatMin) * 60);      //  
    } else {
    gpsLatSec = abs((((latDeg - gpsLatDeg) * 60) + gpsLatMin) * 60);      //  
    }
  if (gpsLatSec > 59.99) {
    gpsLatError = (gpsLatError + 4);                        //  Type 4 Latitude Error
    }
//
//
  gpsLonDeg = lonDeg;                                       //  move float to int to get degrees
  if (abs(gpsLonDeg) > 180) {
    gpsLonError = 1;                                        //  Type 1 Longitude Error
  }
  float tmpLonMin = abs((lonDeg - gpsLonDeg) * 60);            //  subtract to get the Min/Sec portion and multiply by 60 for Min
  gpsLonMin = tmpLonMin;                                    //  subtract to get the Min/Sec portion and multiply by 60 for Min
  if (gpsLonMin > 59) {
    gpsLonError = (gpsLonError + 2);                        //  Type 2 Longitude Error
  }
  if (lonDeg >= 0) {
    gpsLonSec = abs((((lonDeg - gpsLonDeg) * 60) - gpsLonMin) * 60);      // 
    } else {
     gpsLonSec = abs((((lonDeg - gpsLonDeg) * 60) + gpsLonMin) * 60);      // 
    }
  if (gpsLonSec > 59.59) {
    gpsLonError = (gpsLonError + 4);                        //  Type 4 Longitude Error
  }
  Serial.print("Lat Degrees: ");
  Serial.print(gpsLatDeg, DEC);
  Serial.print("  Min: ");
  Serial.print(gpsLatMin, DEC);  
  Serial.print("  Sec: ");
  Serial.print(gpsLatSec, 2);
  Serial.print("  Long Degrees: ");
  Serial.print(gpsLonDeg, DEC);
  Serial.print("  Min: ");
  Serial.print(gpsLonMin, DEC);  
  Serial.print("  Sec: ");
  Serial.println(gpsLonSec, 2);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void getCurrHeading() {
  uint32_t currTime2 = millis();
  uint32_t elapsedTime = 0;
  int bleRespInd = 0;
  char bleResp[50];
  int bytesWritten = 0;
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
  } while (elapsedTime < BLE_WAIT_MS);                    // Wait for 200ms max
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
  currHeadingAcqTime = millis();
  Serial.print("Heading: ");
  Serial.println(currHeading);
}
//
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
  }
  // following line sets the RTC to the date & time this sketch was compiled
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
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
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
int findStartingWaypointInd() {
  // Get final waypoint
  char line[31];
  int index = 0;
  finalWaypoint = {seqNum: 0, time: 0, gpsLatDeg: 0.0, gpsLonDeg: 0.0};
  Serial.print("totalWaypoints: ");
  Serial.println(totalWaypoints);

  for (int i = 0; i < totalWaypoints; i++) {
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

        if (i == totalWaypoints - 1) {
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

  //Serial.println(strlen(bleResp));
  if (strlen(bleResp) < 43) {
    bleErrCnt++;
    return 0;
  }
  bleErrCnt = 0;

  float latDeg = 0.0;
  float lonDeg = 0.0;
  parseGPSString(&bleResp[0], &latDeg, &lonDeg);
  if ((latDeg != 0.0) && (lonDeg != 0.0) && (latDeg < 90.0) && (latDeg > -90.0) && (lonDeg < 180.0) && (lonDeg > -180.0)) {
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

  float distToDest = dist_between(latDeg, lonDeg, finalWaypoint.gpsLatDeg, finalWaypoint.gpsLonDeg);
  Serial.print("Distance to final destination: ");
  Serial.println(distToDest);
  vruSayDistToDest(distToDest);
  //
  index = 0;
  memset(line, 0, 31);
  Waypoint waypoint = {seqNum: 0, time: 0, gpsLatDeg: 0.0, gpsLonDeg: 0.0};
  for (int i = 0; i < totalWaypoints; i++) {
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
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
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

      if (strlen(bleResp) < 43) {
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
      if ((latDeg != 0.0) && (lonDeg != 0.0) && (latDeg < 90.0) && (latDeg > -90.0) && (lonDeg < 180.0) && (lonDeg > -180.0)) {
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
      Serial.print(totalWaypoints - 1);
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
      currHeadingAcqTime = millis();
      Serial.print("Heading: ");
      Serial.println(currHeading);
      //    vruSayHeading();

      if (currHeading >= 0.0f) {
        // Tell user what to do based on nav info
        computeUserAction(distToWaypoint, courseHeading, currHeading);
      }

      // Check if waypoint reached
      if (distToWaypoint <= 10.0) {
        Serial.println("Waypoint reached.");
        vruWayPointReached();
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
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void computeUserAction(float dist, float heading, float currHeading) {
  Direction turnDirection = FRONT;
  // Get difference between heading
  float headingDiff = heading - currHeading;
//  if (headingDiff > 180.00) {
//    headingDiff = 360.00 - headingDiff;
//    }
  float absHeadingDiff = abs(headingDiff);
  headingVar = absHeadingDiff;
  
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
    delay(350);
    // Minor error
    Serial.println("Keep straight");
  }
  else if (absHeadingDiff <= 10.0) {
    // Needs course adjustment
    queueVoiceResponse(189);                        //  Forward
    delay(350);
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
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void makeTurn(Direction dir) {
  Serial.print("Turn ");
  switch (dir) {
    case LEFT:
      turnLeft();
      Serial.println("left");
      break;
    case RIGHT:
      turnRight();
      Serial.println("right");
      break;
    case FRONT:
      break;
    case BACK:
      break;
  }
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//
float parseHeading(char *str) {
  int startInd = charArrIndexOf(str, '=') + 2;
  int decimalInd = charArrLastIndexOf(str, '.');
  int len = (decimalInd - startInd) + 3;
  char headingStr[9];
  int index = 0;
  /*
    Serial.println("parseHeading()");
    Serial.println(str);
    Serial.println(startInd);
    Serial.println(decimalInd);
    Serial.println(len);
  */
  if ((startInd < decimalInd) && (decimalInd < strlen(str) - 2) && (len > 0)) {
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
float dist_between(float lat1, float long1, float lat2, float long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  float delta = radians(long1 - long2);
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
float course_to(float lat1, float long1, float lat2, float long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  float dlon = radians(long2 - long1);
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
      case RECORD:
        Serial.println("Manual record or Mr. Beep");
        mrBeepHeading = 0;
        mrBeepHeadingSet = false;
        //        vruBeepOrManMode();
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
    vruModeExitError();
    Serial.println("Error, exit current mode before setting new mode");
  }
}
//
////////////////////////////////////////////////////////////////////////////////////////////////////
//
void processNumInput(char num) {
  uint8_t vru = (num - 48);
  queueVoiceResponse(vru);
  if (currMode == PLAYBACK) {
    if ((currPlaybackStep == CHOOSE_STEP)) {
      switch (num) {
        case '1':                  // Select a file for playback
          currPlaybackStep = SELECT_FILE;
          vruPlayTripMode();
          Serial.println("Select file for playback");
          break;
        case '2':                  // Select file to delete
          currPlaybackStep = FILE_DELETE;
          vruTripFileDelete();
          Serial.println("Select file for delete");
          break;
        default:
          vruInvalidEntry();
          break;
      }
    }
    else if (currPlaybackStep == SELECT_FILE || FILE_DELETE) {
      if (numpadEntry.length() < 3) {
        numpadEntry += num;
        Serial.print("numpadEntry = ");
        Serial.println(numpadEntry);
      }
      else {
        // max valid entry is 999, error
        vruFileEntryError();
        Serial.println("Error, max valid entry is 999");
      }
    }
    else if (currPlaybackStep == IN_PROGRESS) {
      float distToDest = 0;

      switch (num) {
        case '1': // current heading
          vruSayHeading();
          Serial.print("Current heading:");
          Serial.println(currHeading);
          break;
        case '2': // distance to final waypoint
          distToDest = dist_between(currWaypoint.gpsLatDeg, currWaypoint.gpsLonDeg, finalWaypoint.gpsLatDeg, finalWaypoint.gpsLonDeg);
          if (distToDest < 20000) {
            Serial.print("Distance to final destination: ");
            Serial.println(distToDest);
            vruSayDistToDest(distToDest);
          }
          else {
            Serial.println("Error in distance calculation");
            vruDistToDestError(distToDest);
          }
          break;
        case '3': // Get speed from NAV GPS
          getSpeedKnots();
          vruSaySpeed();
          break;
      }
    }
  }
  else if (currMode == MANUAL_REC || currMode == AUTO_REC) {
    switch (num) {
      case '1': // current heading
        if ((millis() - currHeadingAcqTime) >= 3000) {
          uint32_t currTime = millis();
          uint32_t elapsedTime = 0;
          char bleResp[50];
          int bleRespInd = 0;

          int bytesWritten = bleCentral.println('H');
          bleCentral.flush();
          //Serial.print("bytesWritten = ");
          //Serial.println(bytesWritten);
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
            return;
          }
          bleErrCnt = 0;

          currHeading = parseHeading(&bleResp[0]);
          currHeadingAcqTime = millis();
        }
        vruSayHeading();
        Serial.print("Current heading: ");
        Serial.println(currHeading);
        break;
      case '2': // N/A for record mode
        break;
      case '3': // Get speed from NAV GPS
        getSpeedKnots();
        vruSaySpeed();
        break;
    }
  }
  else if (currMode == RECORD) {
    if (num == '1') {
      // Mr. Beep mode
      Serial.println("Mr. Beep mode");
      currMode = MR_BEEP;
      vruMrBeepMode();                                       //  Starting Mr. Beep
    }
    else if (num == '2') {
      // Manual record mode
      Serial.println("Manual record mode");
      currMode = MANUAL_REC;
      sdCardOpenNext();
      vruManRecMode();                                       //  Starting Manual Recording
      recordWaypoint();
    }
    else {
      vruInvalidModeEntry();
      Serial.println("Invalid mode, try again");
      numpadEntry = "";
    }
  }
  else if ((currMode == MR_BEEP) && mrBeepHeadingSet) {
    switch (num) {
      case '1': // current heading
        Serial.print("Current heading:");
        Serial.println(currHeading);
        vruSayHeading();
        break;
      case '2': // distance to final waypoint
        // N/A for Mr. Beep mode
        break;
      case '3': // Get speed from NAV GPS
        getSpeedKnots();
        vruSaySpeed();
        break;
    }
  }
  else if ((currMode == MR_BEEP) && !mrBeepHeadingSet) {
    switch (num) {
      case '1':
        uint32_t currTime = millis();
        uint32_t elapsedTime = 0;
        char bleResp[50];
        int bleRespInd = 0;

        int bytesWritten = bleCentral.println('H');
        bleCentral.flush();
        //Serial.print("bytesWritten = ");
        //Serial.println(bytesWritten);
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
          Serial.println("BLE error");
          vruBluetoothError();
          return;
        }
        bleErrCnt = 0;

        currHeading = parseHeading(&bleResp[0]);
        currHeadingAcqTime = millis();
        Serial.print("Current heading: ");
        Serial.println(currHeading);
        vruSayHeading();
        break;
    }
  }
  else if ((currMode == MR_BEEP) && !mrBeepHeadingSet) {
    switch (num) {
      case '1':
        uint32_t currTime = millis();
        uint32_t elapsedTime = 0;
        char bleResp[50];
        int bleRespInd = 0;

        int bytesWritten = bleCentral.println('H');
        bleCentral.flush();
        //Serial.print("bytesWritten = ");
        //Serial.println(bytesWritten);
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
          Serial.println("BLE error");

          return;
        }
        bleErrCnt = 0;

        currHeading = parseHeading(&bleResp[0]);
        currHeadingAcqTime = millis();
        Serial.print("Current heading: ");
        Serial.println(currHeading);
        break;
    }
  }
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void getSpeedKnots() {
  if ((millis() - currSpeedAcqTime) >= 5000) {
    uint32_t currTime = millis();
    uint32_t elapsedTime = 0;
    char bleResp[50];
    int bleRespInd = 0;

    int bytesWritten = bleCentral.println('V');
    bleCentral.flush();
    //Serial.print("bytesWritten = ");
    //Serial.println(bytesWritten);
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
      return;
    }
    bleErrCnt = 0;

    currSpeed = parseHeading(&bleResp[0]);
    //Serial.println(currSpeed);
    currSpeedAcqTime = millis();
  }
  Serial.print("Current speed: ");
  Serial.println(currSpeed);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
float parseSpeed(char *str) {
  int startInd = charArrIndexOf(str, '=') + 2;
  int decimalInd = charArrLastIndexOf(str, '.');
  int len = (decimalInd - startInd) + 3;
  char speedStr[9];
  int index = 0;
  /*
    Serial.println("parseHeading()");
    Serial.println(str);
    Serial.println(startInd);
    Serial.println(decimalInd);
    Serial.println(len);
  */
  if ((startInd < decimalInd) && (decimalInd < strlen(str) - 2) && (len > 0)) {
    while (index < len) {
      char c = str[startInd++];
      if (isdigit(c) || c == '.') {
        speedStr[index++] = c;
      }
    }
    speedStr[index] = '\0';

    return (float)atof(speedStr);
  }

  return 0.0f;
}
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////
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
  else if ((currMode == PLAYBACK) && (currPlaybackStep == FILE_DELETE)) {
    if (numpadEntry.length() > 0) {
      sdDeleteTripFile();    
      } else {
      vruTripFileNumErr();
      // invalid file number entry
      }
    }
  else if ((currMode == PLAYBACK) && (currPlaybackStep == WAYPOINTS_LOADED)) {
    currPlaybackStep = IN_PROGRESS;
    //WaitForResponse("AT#SO=1\r", "CONNECT", 1000, modemResponse, 0);
    vruTripPlayStart();
    }
  else if ((currMode == MR_BEEP) && !mrBeepHeadingSet) {
    uint32_t currTime = millis();
    uint32_t elapsedTime = 0;
    char bleResp[50];
    int bleRespInd = 0;

    int bytesWritten = bleCentral.println('H');
    bleCentral.flush();
    //Serial.print("bytesWritten = ");
    //Serial.println(bytesWritten);
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
      Serial.println("BLE error");
      vruBluetoothError();
      return;
    }
    bleErrCnt = 0;

    //    mrBeepHeading = parseHeading(&bleResp[0]);
    mrBeepHeading = currHeading;
    mrBeepHeadingSet = true;
    currHeadingAcqTime = millis();
    vruSayHeading();
    Serial.print("Mr. Beep set heading: ");
    Serial.println(mrBeepHeading);
    vruHeadConfirmAndStart();
  }
}
//
////////////////////////////////////////////////////////////////////////////////////////////////////
//
void seekToStartingWaypoint() {
  for (int i = 0; i < startingWaypointsInd; i++) {
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
//
///////////////////////////////////////////////////////////////////////////////////////////////////////
//
void loadWaypointsFromFile() {
  String line = "";
  int seqNum = 0;

  if (totalWaypointsInd == 0) {
    seqNum = startingWaypointsInd;

    for (int i = 0; i < startingWaypointsInd; i++) {
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
  for (int i = 0; i < WAYPOINTS_INIT_LEN; i++) {
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
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
Waypoint loadNextWaypoint() {
  char line[31];
  int index = 0;
  Waypoint waypoint = {seqNum: 0, time: 0, gpsLatDeg: 0.0, gpsLonDeg: 0.0};

  while (playbackFile.available() && index < 30) {
    char c = playbackFile.read();
    //Serial.print(c);

    line[index++] = c;
    if (c == '\n') {
      line[--index] = '\0';
      //Serial.println(strlen(line));

      if (strlen(line) > 20) {
        waypoint = parseWaypoint(&line[0]);
        waypoint.seqNum = totalWaypointsInd;
        Serial.print("Loaded waypoint: ");
        Serial.print(totalWaypointsInd);
        Serial.print("/");
        Serial.print(totalWaypoints - 1);
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
Waypoint parseWaypoint(char *str) {
  Waypoint waypoint = {seqNum: 0, time: 0, gpsLatDeg: 0.0, gpsLonDeg: 0.0};
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
  //delimInd = charArrIndexOf(str, ':');
  delimInd = strlen(str) - 1;
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
bool loadFileForPlayback() {
  String filename = "CMDT0" + numpadEntry + ".TXT";
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
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void mrBeep() {
  if ((currMode == MR_BEEP) && mrBeepHeadingSet) {
    uint32_t currTime = millis();

    if ((currTime - timer) >= PLAYBACK_INT_MS) {
      timer = currTime;

      // Get current location
      uint32_t currTime2 = millis();
      uint32_t elapsedTime = 0;
      char bleResp[50];
      int bleRespInd = 0;
      // Get GPS coordinates from NAV server
      int bytesWritten = bleCentral.println('o');
      bleCentral.flush();
      //Serial.print("bytesWritten = ");
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
      //Serial.print("3");

      if (strlen(bleResp) < 43) {
        //Serial.println(strlen(bleResp));
        //Serial.println(bleResp);
        bleErrCnt++;
        return;
      }
      bleErrCnt = 0;
      //Serial.print("4");

      float latDeg = 0.0;
      float lonDeg = 0.0;
      parseGPSString(&bleResp[0], &latDeg, &lonDeg);
      if ((latDeg != 0.0) && (lonDeg != 0.0) && (latDeg < 90.0) && (latDeg > -90.0) && (lonDeg < 180.0) && (lonDeg > -180.0)) {
        // Valid
        /*
          Serial.print("GPS: ");
          Serial.print(latDeg, 6);
          Serial.print(",");
          Serial.println(lonDeg, 6);
        */
        currWaypoint.gpsLatDeg = latDeg;
        currWaypoint.gpsLonDeg = lonDeg;
      }
      else {
        //Serial.println("Invalid GPS");
        //return;
      }

      delay(100);
      currTime2 = millis();
      elapsedTime = 0;
      bleRespInd = 0;
      memset(bleResp, 0, 50);
      bytesWritten = bleCentral.println('H');
      bleCentral.flush();
      //Serial.print("bytesWritten = ");
      //Serial.println(bytesWritten);
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
      //Serial.println(strlen(bleResp));

      while (bleCentral.available()) {
        bleCentral.read();
      }

      if (strlen(bleResp) < 15) {
        bleErrCnt++;
        return;
      }
      bleErrCnt = 0;

      currHeading = parseHeading(&bleResp[0]);
      currHeadingAcqTime = millis();

      Direction turnDirection = FRONT;
      // Get difference between heading
      float headingDiff = mrBeepHeading - currHeading;
//      if (headingDiff > 180.00) {
//        headingDiff = 360.00 - headingDiff;
//        }
      float absHeadingDiff = abs(headingDiff);
      headingVar = absHeadingDiff;

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
      if (absHeadingDiff <= 2.0) {
        queueVoiceResponse(189);                        //  Forward
        delay(350);
        // Minor error
        Serial.println("Keep straight");
      }
      else {
        // Needs course adjustment
        makeTurn(turnDirection);
      }
    }
  }
}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    if (strlen(bleResp) < 43) {
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
    if ((latDeg != 0.0) && (lonDeg != 0.0) && (latDeg < 90.0) && (latDeg > -90.0) && (lonDeg < 180.0) && (lonDeg > -180.0)) {
      // Valid
      sprintf(parsedCoord, "%s,%s", temp1, temp2);
      //Serial.println(parsedCoord);
      //Serial.println(strlen(parsedCoord));
      tripFile.println(parsedCoord);
      tripFile.flush();
      waypointRecorded = true;
      vruWayPointRecorded();

      Serial.print("Waypoint: ");
      Serial.println(parsedCoord);
      Serial.println("Waypoint saved");
      currWaypoint.gpsLonDeg = lonDeg;
      currWaypoint.gpsLatDeg = latDeg;
    }
    else {
      Serial.println("Invalid waypoint");
      vruEmptyWayPoint();
      return;
    }

    //currWaypoint.gpsLatDeg = 29.55;
    //currWaypoint.gpsLonDeg = -95.38;

    // Get current heading
    /*
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
      currHeadingAcqTime = millis();
      Serial.print("Heading: ");
      Serial.println(currHeading);

      /*
      String saveString = parsedCoord + ":" + String(currHeading, 2);
      Serial.print("saveString: ");
      Serial.println(saveString);


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
    */
  }
//  else {
//    vruNotRecordingMode();
//    //Serial.println("Error, not in recording mode");
//  }
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
  if ((firstInd < lastInd) && (lastInd < strlen(gpsStr) - 10)) {
    // Valid, expect format DD.DDDDDD
    char lat[11];
    char lon[11];
    int index = 0;
    int startInd = firstInd + 2;
    //Serial.println(startInd);

    int len = (gpsStr[startInd] == '-') ? 10 : 9;
    //Serial.println(len);
    while (index < len) {
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
    while (index < len) {
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
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
int charArrIndexOf(char *arr, char charToFind)
{
  int i;
  for (i = 0; i < strlen(arr); i++) {
    if (arr[i] == charToFind) {
      return i;
    }
  }

  return -1;
}
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
int charArrLastIndexOf(char *arr, char charToFind)
{
  int i;
  for (i = strlen(arr); i > 0; i--) {
    if (arr[i] == charToFind) {
      return i;
    }
  }

  return -1;
}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void chkForCMDInput() {
  keyPadInput = cmdKeyPad.getKey();
  //
  if (keyPadInput) {
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
        // For entering auto record mode
        queueVoiceResponse(30);                                          // say "A"
        setMode(AUTO_REC);
        vruAutoRecMode();
        break;
      case 'B':
        // For entering manual record mode
        queueVoiceResponse(31);                                          // say "B"
        setMode(RECORD);
        vruBeepOrManMode();
        break;
      case 'C':
        // For entering playback mode
        queueVoiceResponse(32);                                          // say "C"
        setMode(PLAYBACK);
        vruTripFileSelect();
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
        currPlaybackStep = CHOOSE_STEP;
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
        if (currMode != MANUAL_REC) {
          vruSayGPSLocation();                                             // say GPS location (Degrees, Minutes, Seconds)
          }
        break;
      case '#':
        queueVoiceResponse(44);                                          // say "Pound"
        // For confirming actions
        confirmAction();
        break;
    }
  }
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
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
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
String parseICCID(String modemResp)
{
  int startInd = modemResp.indexOf(':') + 2;
  return modemResp.substring(startInd, startInd + 20);
}
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void checkModem() {
  if (reCheckCellCount > 9000) {                            // turn on and recheck modem and connection every 15 minutes
    digitalWrite(11, HIGH);
    delay(2500);
    skipLocationSend = false;
    badConnectionCount = 0;
    reCheckCellCount = 0;
  }
  if (badConnectionCount > 5) {                           // having repeated trouble connecting?
    reCheckCellCount++;                                   // increment a counter to recheck the cell modem and connection later
    digitalWrite(11, LOW);                                // turn off modem to save power
    skipLocationSend = true;                              // bypass sending data to the cloud
    if (reCheckCellCount == 1) {
      vruCellModemDown();
    }
    return;
  }
  if (millis() > MODEM_INIT_WAIT) {
    //    skipLocationSend = true;
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
      skipLocationSend = false;
      //WaitForResponse("+++", "OK", 1000, modemResponse, 0);
    }
    else if (!connectionGood) {
      badConnectionCount++;
      //Serial.println("Waiting for network connection");
      cellSerial.print("AT+CGREG?\r");
      cellSerial.flush();
      currentString = "";
      delay(250);

      // Read cellSerial port buffer1 for UART connected to modem and print that message back out to debug cellSerial over USB
      while (cellSerial.available() > 0)
      {
        //read incoming byte from modem
        char incomingByte = cellSerial.read();
        //write byte out to debug cellSerial over USB
        Serial.print(incomingByte);

        // add current byte to the string we are building
        currentString += char(incomingByte);

        // check currentString to see if network status is "0,1" or "0,5" which means we are connected
        if ((currentString.substring(currentString.length() - 3, currentString.length()) == "0,1") ||
            (currentString.substring(currentString.length() - 3, currentString.length()) == "0,5"))
        {
          while (PrintModemResponse() > 0); // consume rest of message once 0,1 or 0,5 is found

          Serial.println("Get IP");
          if (WaitForResponse("AT#SGACT=1,1\r", "OK", 5000, modemResponse, 0)) {
            if (WaitForResponse("AT#SD=1,0,80,\"runm-east.att.io\",0,1,0\r", "CONNECT", 2000, modemResponse, 0)) {
              vruCellModemConnected();
              connectionGood = true;
              skipLocationSend = false;
              badConnectionCount = 0;
              reCheckCellCount = 0;
            } else {
              vruCellModemDown();
            }
          }
          //          else {
          //            vruCellModemDown();
          //            }
        }
      }
    }
  }
}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// sends a command to the modem, waits for the specified number of milliseconds,
// checks whether the modem response contains the expected response, and
// appends the remaining response characters to the out parameter respOut
// returns true if command received the expected response
bool SendModemCommand(String command, String expectedResp, int msToWait, String& respOut, byte b)
{
  int cmd_timeout = 0;
  if (b)
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
  while (!cellSerial.available())
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
  while (cellSerial.available() > 0)
  {
    resp += char(cellSerial.read());
    if (resp.endsWith(expectedResp))
    {
      respOut = resp;
      while (cellSerial.available() > 0)
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
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
  while (cellSerial.available())
    cellSerial.read();
}

// returns modem response as a String
String GetModemResponse()
{
  String resp = "";
  while (cellSerial.available() > 0)
  {
    resp += char(cellSerial.read());
  }
  return resp;
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// consumes and prints modem response
int PrintModemResponse()
{
  String resp = "";
  while (cellSerial.available() > 0)
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
    queueVoiceResponse(210);                          // connecting to the nav server
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
    Serial.print("Heading BLE Connect Data: ");
    Serial.println(bleResp);
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
    delay(250);
    Serial.print("Signal Strength BLE Connect Data: ");
    Serial.println(bleResp);
    float rssi = parseRSSI(bleResp);
    bleRSSI = rssi;
    if (bleRSSI > -120.00) {
      navConnected = true;
    }                         // Signal greater than -120dBm is good
    bleConnectCount++;
  } while (!navConnected && bleConnectCount < 5);
  delay(2000);
  if (navConnected) {
    queueVoiceResponse(211);                             //  Connected to ROGERNAV
    delay(750);
  } else {
    queueVoiceResponse(182);                            //   Bypassed Connecting to ROGERNAV
    delay(750);
    queueVoiceResponse(175);
    delay(350);
    queueVoiceResponse(2);
    queueVoiceResponse(133);
  }
  delay(1500);
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void kbdVRUCheck() {
  delay(1500);
  queueVoiceResponse(42);                       //   Press
  delay(250);
  queueVoiceResponse(1);                        //   One (1)
  delay(200);
  queueVoiceResponse(4);                        //   For (or 4)
  delay(150);
  queueVoiceResponse(67);                       //   Left
  do {
    chkForCMDInput();
  } while (keyPadInput != '1');
  delay(200);
  turnLeft();                                   //   Say Left
  delay(800);
  queueVoiceResponse(42);                       //   Press
  delay(250);
  queueVoiceResponse(3);                        //   Three (3)
  delay(150);
  queueVoiceResponse(4);                        //   For (or 4)
  delay(250);
  queueVoiceResponse(66);                       //   Right
  do {
    chkForCMDInput();
  } while (keyPadInput != '3');
  delay(200);
  turnRight();                                   //   Say Right
  delay(800);
  queueVoiceResponse(135);                       //   Congratulations
  delay(2500);                                   //   delay 1500ms
  queueVoiceResponse(104);                       //   System
  delay(50);
  queueVoiceResponse(46);                        //   On
  delay(1000);
}
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void configRogerCMD() {
  if (trace) {
    Serial.println("Starting CMD Config");
  }
  kbdVRUCheck();
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
    unsigned int voiceLRec = Serial.parseInt();                        // get the data
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void testVRUTiming() {
  delay(350);
  queueVoiceResponse(76);                       //   Starting 
  delay(150);
  queueVoiceResponse(104);                      //   System
  delay(150);
  queueVoiceResponse(124);                      //   Sequence
  delay(150);
  queueVoiceResponse(165);                      //   Run
  delay(150);
//
  delay(150);
  queueVoiceResponse(1);                       //   1
  turnRight();
  delay(150);
  queueVoiceResponse(2);                       //   2
  turnLeft();
  delay(150);
  queueVoiceResponse(3);                       //   3
  currHeading = 359;
  vruSayHeading();
  vruHeadConfirmAndStart();
  delay(150);
  queueVoiceResponse(4);                       //   4
  vruWayPointRecorded();
  delay(150);
  queueVoiceResponse(5);                       //   5
  vruWayPointReached();
  delay(150);
  queueVoiceResponse(6);                       //   6
  vruWayPointSkipped();
  delay(150);
  queueVoiceResponse(7);                       //   7
  vruTripPlayStart();
  delay(150);
  queueVoiceResponse(8);                       //   8
  vruTripFileReady();
  delay(150);
  queueVoiceResponse(9);                       //   9
  vruTripComplete();
  delay(150);
  queueVoiceResponse(10);                       //   10
  vruTripFileLoadErr();
  delay(150);
  queueVoiceResponse(11);                       //   11
  vruEmptyWayPoint();
  delay(150);
  queueVoiceResponse(12);                       //   12
  vruNotRecordingMode();
  delay(150);
  queueVoiceResponse(13);                       //   13
  vruInvalidModeEntry();
  delay(150);
  queueVoiceResponse(14);                       //   14
  vruCellModemConnected();
  delay(150);
  queueVoiceResponse(15);                       //   15
  vruCellModemDown();
  delay(150);
  queueVoiceResponse(16);                       //   16
  vruBluetoothError();
  delay(150);
  queueVoiceResponse(17);                       //   17
  vruFileEntryError();
  delay(250);
  queueVoiceResponse(18);                       //   18
  vruModeExitError();
  delay(150);
  queueVoiceResponse(19);                       //   19
  vruDistToDestError(999999.00F);
  delay(150);
  queueVoiceResponse(20);                       //   20
  vruSayFileNumber();
  delay(150);
  queueVoiceResponse(20);                       //  20
  delay(350);
  queueVoiceResponse(1);                       //   1
  lowBatCount = 1300;
  vruBatteryLow();
  delay(250);
  queueVoiceResponse(20);                       //  20
  delay(450);
  queueVoiceResponse(2);                       //   2
  lowBatCount = 1300;
  vruMainBatteryLow();
  delay(250);
  queueVoiceResponse(20);                       //  20
  delay(350);
  queueVoiceResponse(3);                       //   3
  currHeading = 359;
  vruSayHeading();
  delay(250);
  queueVoiceResponse(20);                       //  20
  delay(250);
  queueVoiceResponse(4);                       //   4
  vruSayDistToDest(987654321.0F);
  delay(250);
  queueVoiceResponse(20);                       //  20
  delay(250);
  queueVoiceResponse(5);                       //   5
  currSpeed = 999;
  vruSaySpeed();
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
///////////////////////////////////////////////////////////////////////////////////////////
//
void chkForLowBattery() {
  chkMainBattery();
  battTraceCount++;
  if (mainBatteryLow) {
    lowBatCount++;
    vruMainBatteryLow();
  } else {
    if (battTraceCount > 9000) {
      battTraceCount = 0;
      Serial.print("Main Battery Voltage: ");
      Serial.println(mainVolt);
    }
  }
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////
//
void chkMainBattery() {
  mainVolt = analogRead(LOWBATPIN);
  delay(50);
  if (mainVolt <= 60) {                      //  Had been 60 on the other lab systems and 380 previously on Ahmet's system
    mainBatteryLow = true;
  } else {
    mainBatteryLow = false;
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
  vruSerial.begin(4800);                                                 // start the Adafruit Music Maker serial port
  delay(250);
  cellSerial.begin(115200);                                              // start the Adafruit Music Maker serial port
  delay(250);
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void setup() {
  configSerialPorts();               // initialize serial:
  configIOPins();                    // configure the IO pins
  delay(5000);
  checkForRTC();                     // Check for an RTC
  delay(1000);
  sdCardInit();                      // initialize the SD card
  delay(1000);
  digitalWrite(11, LOW);
  delay(2500);
  digitalWrite(11, HIGH);
  if (trace) {
    Serial.println("Ready!");
    delay(250);
  }
//  testVRUTiming();
  delay(4000);
  //  trace = false;
  configRogerCMD();
  lowBatCount = 0;
  //wdt_enable(WDTO_8S);
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void loop() {
  //  testVRUCom();
  checkModem();
  if (!skipLocationSend) {
    sendLocationToFlow();
  }
  //  testBLECom();
  //  testCellCom();
  chkForCMDInput();
  chkForLowBattery();
  autoRecordWaypoint();
  mrBeep();
  computeTripInfo();
  checkTripComplete();
  checkBLEError();
  //wdt_reset();
}
//
//
