////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  RogerCMDV07
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
#include <QueueArray.h>
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
#define AUTO_RECORD_INT_MS      10000        // 10 sec
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
byte effect            = 1;
char keyPadInput       = ' ';
uint32_t timer         = millis();     // unsigned 32bit interger variable called timer assigned to the millisecond function
uint32_t tripRecSeq    = 0;            // used to sequence the record numbers in the log file
uint16_t voiceRec      = 0;            // number being sent to VRU for voice playback
uint8_t navHour        = 0;            // current UCT (GMT) hour data from GPS
uint8_t navMinute      = 0;            // current minute data from GPS
uint8_t navSeconds     = 0;            // current seconds data from GPS
uint16_t navMillsec    = 0;            // current milliseconds data from GPS
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
int gpsAltitude        = 0;            // GPS Altitude (in meters)
uint8_t gpsSats        = 0;            // GPS Satellites
float roll             = 0;            // variable to hold roll data from the IMU
float pitch            = 0;            // variable to hold the pitch data from the IMU
float heading          = 0;            // variable to hold the heading data from the IMU
float qHead            = 0;            // float to hold computed heading from the Quaternion angle
float qW               = 0;            // float to hold the Quaternion angle for heading from the IMU
float qX               = 0;            // variable to hold Quaternion X data from the IMU
float qY               = 0;            // variable to hold Quaternion Y data from the IMU
float qZ               = 0;            // variable to hold Quaternion Z data from the IMU
int utcOffset          = 0;            // offset for local time
int dstAdjust          = 0;            // adjustment for daylight saving time
float measuredVbat     = 0;            // float to hold the measured batttery voltage
const char navName[11] = "RogerNAV01"; // name of the bluetooth LE nav module
const char cmdName[11] = "RogerCMD01"; // name of the bluetoothe LE command module
const int bleBuffSize  = 127;          // size of bluetooth buffer size
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
byte rowPins[ROWS] = {2, 3, 4, 5};       //connect to the row pinouts of the keypad
byte colPins[COLS] = {6, 7, 8, 9};        //connect to the column pinouts of the keypad
//
//
byte bleBuffer[bleBuffSize];           // buffer to hold data to/from the bluetooth comm connection
//
//
boolean stringComplete    = false;        // whether the string is complete
boolean dateRefreshed     = false;        // updated when the date is refreshed from GPS
boolean dsTime            = false;        // standard time when false.  daylight saving when true.
boolean keyPadPressed     = false;
boolean buzzRight         = false;
boolean buzzLeft          = false;
boolean cellReq           = false;
boolean trace             = true;
//
//
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
//
String inputString     = "";           // a string to hold incoming heading data.  Space is allocated in the setup function
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

typedef struct waypoint {
  int seqNum;
  uint32_t time;
  float gpsLatDeg;
  float gpsLonDeg;
} Waypoint;

Mode currMode = NONE;
String numpadEntry = "";
PlaybackStep currPlaybackStep = SELECT_FILE;
QueueArray<Waypoint> wayPointQueue;

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
  if (trace) {
    Serial.print("Right Effect #");
    Serial.println(effect);
    }
  tcaSelect(0);                    // Right Side
  playBuzzer();
  delay(250);
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void playLeft() {
  if (trace) {
    Serial.print("Left Effect #");
    Serial.println(effect);
    }
  tcaSelect(1);
  playBuzzer();
  delay(250);
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
  effect = 1;
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
  char filename[15];
  strcpy(filename, "CMDT0000.TXT");
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
  Serial.print("Writing to ");
  Serial.println(filename);
  //writeSDTripData();
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void writeSDTripData() {
  tripFile.print("Sequence:");      tripFile.print(",Time:");
  tripFile.print(",Date:");         tripFile.print(",Fix:");
  tripFile.print(",Quality:");      tripFile.print(",Latitude:");
  tripFile.print(",Longitude:");    tripFile.print(",Speed(knots):");
  tripFile.print(",Angle:");        tripFile.print(",Altitude:");
  tripFile.print(",Satellites:");   tripFile.print(",Roll:");
  tripFile.print(",Pitch:");        tripFile.print(",Heading:");
  tripFile.print(",CompHead:");     tripFile.print(",QW:");
  tripFile.print(",QX:");           tripFile.print(",QY:");
  tripFile.print(",QZ:");           tripFile.print(",BattVolts:");
  tripFile.println();               tripFile.flush();
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void spare1() {


}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void spare2() {



}
//
//
/* ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void loadGPSData() {
    navHour =
    navMinute =
    navSeconds =
    navMillsec =
    navMonth =
    navDay =
    navYear =
    gpsFix =
    gpsQual =
    if (   ) {
      gpsLatitude  =
      gpsLat       =
      gpsLongitude =
      gpsLon       =
      gpsKnots     =
      gpsAngle     =
      gpsAltitude  =
      gpsSats      =
      convertGPSToDMS();
      computeDecimalDeg();
      }
}
*/
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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void printNAVData() {
  if (!trace) {
    return;
    }
  Serial.print("\nTime: ");
  Serial.print(navHour, DEC); Serial.print(':');
  Serial.print(navMinute, DEC); Serial.print(':');
  Serial.print(navSeconds, DEC);
  Serial.print("  Date: ");
  Serial.print(navMonth, DEC); Serial.print('/');
  Serial.print(navDay, DEC); Serial.print("/20");
  Serial.println(navYear, DEC);
  Serial.print("Fix: "); Serial.print(gpsFix, DEC);
  Serial.print(" Qual: "); Serial.println(gpsQual, DEC);
  if (gpsFix) {
    Serial.print(" Lat: "); Serial.print(gpsLatitude, 4);
    Serial.println(gpsLat); Serial.print(" Lat DecDeg: ");
    Serial.println(decimalDegLat, 6);  Serial.print(" Lat Deg, Min, Sec: ");
    Serial.print(gpsLatDeg, DEC); Serial.print(", ");
    Serial.print(gpsLatMin, DEC); Serial.print(", ");
    Serial.println(gpsLatSec, 2); Serial.print(" Long: ");
    Serial.print(gpsLongitude, 4); Serial.println(gpsLon);
    Serial.print(" Long DecDeg: "); Serial.println(decimalDegLon, 6);
    Serial.print(" Long Deg, Min, Sec: "); Serial.print(gpsLonDeg, DEC);
    Serial.print(", "); Serial.print(gpsLonMin, DEC);
    Serial.print(", "); Serial.println(gpsLonSec, 2);
    Serial.print(" Knots: "); Serial.println(gpsKnots, 2);
    Serial.print(" Ang: "); Serial.println(gpsAngle, 2);
    Serial.print(" Alt: "); Serial.println(gpsAltitude);
    Serial.print(" Sat: "); Serial.println(gpsSats, DEC);
    Serial.print(" Roll: "); Serial.print(roll);
    Serial.print(" Pitch: "); Serial.print(pitch);
    Serial.print(" Head: ");  Serial.print(heading);
    Serial.print(" QHead: "); Serial.print(qHead);
    Serial.print(" QW: "); Serial.print(qW);
    Serial.print(" QX: "); Serial.print(qX);
    Serial.print(" QY: "); Serial.print(qY);
    Serial.print(" QZ: "); Serial.println(qZ);
    Serial.print(" VBat: " ); Serial.println(measuredVbat);
  }
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void chkBLE() {                           /* Check the BLE Central interface */
  Serial.println(F("Checking the Bluefruit LE Central module: "));
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
void bleKBDDevOff() {




}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void setNavName() {




}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void setBLEdBm(int dBm) {





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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void checkForCMDDataReq() {
  stringComplete = false;
  while (bleCentral.available() && !stringComplete) {
    char bleChar = (char)bleCentral.read();            // get the new byte:
    if (bleChar >= 'a' && bleChar <= 'z') {
      bleChar = bleChar &~ (0x20);
      }
    switch (bleChar) {                                // check for data type indicator
      case ('A'):
        Altitude();
        break;
      case ('B'):
        Battery();
        break;
      case ('D'):
        Date();
        break;
      case ('E'):
        Error();
        break;
      case ('G'):
        GPSLoc();
        break;
      case ('H'):
        Heading();
        break;
      case ('L'):
        DegMinSec();
        break;
      case ('N'):
        NavData();
        break;
      case ('O'):
        DecDegLoc();
        break;
      case ('Q'):
        RecSequence();
        break;
      case ('R'):
        bleSignal();
        break;
      case ('S'):
        Satellites();
        break;
      case ('T'):
        Time();
        break;
      case ('V'):
        Velocity();
        break;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
      case '\n':
        stringComplete = true;
        break;
      default:
        bleCentral.println("CMD ERROR" );
        bleCentral.flush();
        stringComplete = false;
        break;
      }
    }
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void Altitude() {
  bleCentral.print("ALT = " );
  bleCentral.println(gpsAltitude, DEC );
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void Battery() {
  bleCentral.print("BATT = " );
  bleCentral.println(measuredVbat, 2);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void Date() {
  bleCentral.print("DATE = " );
  bleCentral.print(navMonth, DEC);
  bleCentral.print('/');
  bleCentral.print(navDay, DEC);
  bleCentral.print("/20");
  bleCentral.println(navYear, DEC);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void Error() {
  bleCentral.print("ERR = " );
  bleCentral.print(gpsLatError, DEC);
  bleCentral.print(',');
  bleCentral.println(gpsLonError, DEC);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void GPSLoc() {
  bleCentral.print("GPSLAT = " );
  bleCentral.print(gpsLatitude, 4);
  bleCentral.println(gpsLat);
  bleCentral.print("GPSLON = " );
  bleCentral.print(gpsLongitude, 4);
  bleCentral.println(gpsLon);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void Heading() {
  bleCentral.print("HEAD = " );
  bleCentral.println(heading, 2);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void DegMinSec() {
  bleCentral.print("LATDMS = " );
  bleCentral.print(gpsLatDeg); bleCentral.print(',');
  bleCentral.print(gpsLatMin); bleCentral.print(',');
  bleCentral.println(gpsLatSec, 2);
  bleCentral.print("LONDMS = " );
  bleCentral.print(gpsLonDeg); bleCentral.print(',');
  bleCentral.print(gpsLonMin); bleCentral.print(',');
  bleCentral.println(gpsLonSec, 2);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void NavData() {
  Date();
  Time();
  GPSLoc();
  Heading();
  Velocity();
  Satellites();
  bleSignal();
  Error();
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void DecDegLoc() {
  bleCentral.print("DECLAT = " );
  bleCentral.println(decimalDegLat, 6);
  bleCentral.print("DECLON = " );
  bleCentral.println(decimalDegLon, 6);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void RecSequence() {
  bleCentral.print("RSEQ = " );
  bleCentral.println(tripRecSeq);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void bleSignal() {
  bleCentral.print("RSSI = " );
  bleCentral.println(bleRSSI, DEC);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
void Satellites() {
  bleCentral.print("SAT = " );
  bleCentral.println(gpsSats);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void Time() {
  bleCentral.print("TIME = " );
  bleCentral.print(navHour, DEC); bleCentral.print(':');
  bleCentral.print(navMinute, DEC); bleCentral.print(':');
  bleCentral.println(navSeconds, DEC);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void Velocity() {
  bleCentral.print("KNOTS = " );
  bleCentral.println(gpsKnots, 2);
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void getBLERSSI() {




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
void provideHapticFeedback() {

}
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
void computeTripInfo() {
  if ((currMode == PLAYBACK) && (currPlaybackStep == IN_PROGRESS)) {
    Waypoint nextWaypoint = wayPointQueue.peek();

  }
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//

void setMode(Mode mode) {
  if (currMode == NONE) {
    currMode = mode;
    Serial.print("Mode set to ");
    Serial.println(mode);
  }
  else {
    // Need to exit existing mode before setting new
    Serial.println("Error, exit current mode before setting new mode");
  }
}

void processNumInput(char num) {
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

void confirmAction() {
  if ((currMode == PLAYBACK) && (currPlaybackStep == SELECT_FILE)) {
    if (numpadEntry.length() > 0) {
      if (loadFileForPlayback()) {
        // success
        currPlaybackStep = FILE_LOADED;
        loadWaypointsFromFile();
      }
      else {
        // fail
      }
    }
    else {
      // invalid entry
    }
  }
  else if ((currMode == PLAYBACK) && (currPlaybackStep == WAYPOINTS_LOADED)) {
    currPlaybackStep = IN_PROGRESS;
  }
}

void loadWaypointsFromFile() {
  String line = "";
  int seqNum = 0;

  while (playbackFile.available()) {
    char c = playbackFile.read();
    Serial.print(c);
    
    line += c;
    if (c == '\n') {
      Waypoint waypoint = parseWaypoint(line);
      waypoint.seqNum = seqNum++;
      wayPointQueue.enqueue(waypoint);

      Serial.print("Waypoint ");
      Serial.print(waypoint.seqNum);
      Serial.print(": ");
      Serial.print(waypoint.gpsLatDeg, 6);
      Serial.print(",");
      Serial.println(waypoint.gpsLonDeg, 6);
      line = "";
    }
    
  }
  playbackFile.close();

  if (wayPointQueue.count() > 0) {
    currPlaybackStep = WAYPOINTS_LOADED;
    Serial.print("# waypoints: ");
    Serial.println(wayPointQueue.count());
  }
}

Waypoint parseWaypoint(String str) {
  Waypoint waypoint = {seqNum:0, time:0, gpsLatDeg:0.0, gpsLonDeg:0.0};
  int startInd = 0;
  int delimInd = str.indexOf(',');
  String part = str.substring(startInd, delimInd);
  // part 1 longitude
  waypoint.gpsLatDeg = part.toFloat();
  Serial.print("part1: ");
  Serial.println(part);
  Serial.println(part.toFloat(), 6);
  char temp[9];
  part.toCharArray(temp, part.length()+1);
  float v = atof(temp);
  Serial.println(v, 6);
  Serial.println(waypoint.gpsLatDeg, 6);

  startInd = delimInd + 1;
  part = str.substring(startInd);
  // part 2 latitude
  waypoint.gpsLonDeg = part.toFloat();
  Serial.print("part2: ");
  Serial.println(part);
  Serial.println(part.toFloat(), 6);
  Serial.println(waypoint.gpsLonDeg, 6);

  return waypoint;
}

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

  return true;
}

void recordWaypoint() {
  if ((currMode == MANUAL_REC) || (currMode == AUTO_REC)) {
    uint32_t currTime = millis();
    uint32_t elapsedTime = 0;
    String bleResp = "";
    // Get GPS coordinates from NAV server
    bleCentral.println("o");
    do {
      while (bleCentral.available()) {
        bleResp += (char)bleCentral.read();
      }
      elapsedTime = millis() - currTime;
    } while (elapsedTime < 200); // Wait for 200ms max

    float latDeg = 0.0;
    float lonDeg = 0.0;
    String parsedCoord = parseGPSString(bleResp, &latDeg, &lonDeg);
    tripFile.println(parsedCoord);
    tripFile.flush();
    Serial.print("Waypoint: ");
    Serial.println(parsedCoord);
    Serial.println("Waypoint saved");
  }
  else {
    //Serial.println("Error, not in recording mode");
  }
}

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

String parseGPSString(String gpsStr, float *latDeg, float *lonDeg) {
  int firstInd = gpsStr.indexOf('=');
  int lastInd = gpsStr.lastIndexOf('=');
  
  if (firstInd < lastInd) {
    // Valid, expect format DD.DDDDDD
    String lat = "";
    String lon = "";
    int startInd = firstInd + 2;

    while(lat.length() < 9) {
      char c = gpsStr.charAt(startInd++);
      if (isDigit(c) || c == '.' || c == '-') {
        lat += c;
      }
    }

    startInd = lastInd + 2;
    while(lon.length() < 9) {
      char c = gpsStr.charAt(startInd++);
      if (isDigit(c) || c == '.' || c == '-') {
        lon += c;
      }
    }

    *latDeg = lat.toFloat();
    *lonDeg = lon.toFloat();
    return lat + "," + lon;
  }

  return "";
}

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
        setMode(MANUAL_REC);
        break;
      case 'B':
        // For entering auto record mode
        setMode(AUTO_REC);
        break;
      case 'C':
        // For entering playback mode
        setMode(PLAYBACK);
        break;
      case 'D':
        // For exiting current mode
        currMode = NONE;
        Serial.println("Mode reset");
        break;
      case '*':
        // For manually recording waypoints
        recordWaypoint();
        break;
      case '#':
        // For confirming actions
        confirmAction();
        break;
    }
  }
}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void kbdVRUHapticCheck() {
  queueVoiceResponse(42);                       //   Press
  queueVoiceResponse(1);                        //   One (1)
  queueVoiceResponse(4);                        //   For (or 4)
  queueVoiceResponse(67);                       //   Left
  do {
    chkForCMDInput();
    } while (keyPadInput != '1');
  playLeft();                                   //   Buzz Left
  delay(100);
  queueVoiceResponse(42);                       //   Press
  queueVoiceResponse(3);                        //   Three (3)
  queueVoiceResponse(4);                        //   For (or 4)
  queueVoiceResponse(66);                       //   Right
  do {
    chkForCMDInput();
    } while (keyPadInput != '3');
  playRight();                                   //  Buzz Right
  delay(100);
  queueVoiceResponse(135);                       //   Congratulations
  queueVoiceResponse(104);                       //   System
  queueVoiceResponse(46);                        //   On
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void configRogerCMD() {
  kbdVRUHapticCheck();
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
  while (cellSerial.available()) {
    char cmd = cellSerial.read();
    if (cmd == '\n') {
      break;
      }
    bleCentral.write(cmd);
    bleCentral.flush();
    cellReq = true;
   }
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void queueVoiceResponse(uint16_t vRec ) {
  vruSerial.print('Q');
  vruSerial.println(vRec, DEC);
  vruSerial.flush();
}
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void sendVoiceResponse() {



}
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void configIOPins() {
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);
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
  bleCentral.begin(115200);                                                // start the Bluetooth LE serial port
  delay(250);
  vruSerial.begin(9600);                                                 // start the Adafruit Music Maker serial port
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
/*
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);
  delay(250);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
*/
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void setup() {
  configSerialPorts();               // initialize serial:
  configIOPins();                    // configure the IO pins
  inputString.reserve(BUFSIZE);      // reserve 256 bytes for the inputString:
  //checkForRTC();                     // Check for an RTC
  delay(250);
  sdCardInit();                      // initialize the SD card
  delay(250);
  chkBLE();                          // start the Bluetooth LE interface
  delay(250);
  //chkHapticDrv();
  if (trace) {
    Serial.println("Ready!");
    delay(250);
    }
//  trace = false;
  //configRogerCMD();
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void loop() {
//  testVRUCom();
  testBLECom();
//  testCellCom();
  chkForCMDInput();
  getRogerNAVData();
  autoRecordWaypoint();
  computeTripInfo();
  provideHapticFeedback();
  sendVoiceResponse();
//
}
//
//

