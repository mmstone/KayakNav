////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  RogerNAV001_V1d7
//
//  Basic navigational sensors and systems integrated into a data recording and navigational server platform
//  which can be accessed via simple commands via a Bluetooth LE interface.  Navigational sensors include:
//  1)  9-DOF Integrated Sensor (3-axis Magnetometer, 3-axis Accelerometer, 3-axis Gyroscope) w/dedicated M0 processor
//  connected with
//  2)  Arduino MEGA 2560 with:
//      a)  Multi-channel GPS Satellite receiver/signal processor
//      b)  Real-Time Clock (RTC)
//      c)  Micro SD Card Adapter
//      d)  Bluetooth LE Interface
//
//  Changed routine to use the Interrupts
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  #includes
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include "RTClib.h"
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  #defines
#define bleSerial               Serial1  // Bluefruit HW Serial Name
#define imuSerial               Serial2  // IMU HW Serial Name
#define gpsSerial               Serial3  // what's the name of the gps hardware serial port?
#define cardSelect              53       // Card Select on the MEGA for SD Card SPI comm
#define VBATPIN                 A7       // Voltage at the IMU backup battery pin
#define GPSECHO                 false    // Do not echo GPS serial comm data
#define VERBOSE_MODE            true     // If set to 'true' enables debug output
#define FACTORYRESET_ENABLE     1        // Reset = 1.  No resetNo need to reset set it here
#define BLUEFRUIT_UART_CTS_PIN  8        // Pulled High.  Take low to disable data comm
#define BLUEFRUIT_UART_MODE_PIN 12       // Not used with HW serial
#define BUFSIZE                 255      // Size of the read buffer for incoming data
#define gpsLock                 9        // Pin for LED indicating a GPS Fix.  Take high to activate.
#define sdWrite                 2        // Pin for LED indicating an SD Card write.  Take low to activate.
#define bleXfer                 3        // Pin for LED indicating BLE data is being transferred.  Take low to activate.
#define msgRed                  5        // Pin for Red channel on MSG LED.  Take low to activate.
#define msgGreen                6        // Pin for Green channel on MSG LED.  Take low to activate.
#define msgBlue                 7        // Pin for Blue channel on MSG LED.  Take low to activate.
#define msgSounder              11       // Pin for activating the Piezo sounder.  Take high to activate. 
#define sysLED                  13       // Pin for onboard Arduino LED.  Take high to activate.
//
//
//
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&gpsSerial);           // Assign the GPS object to the gpsSerial port's address
//
//
RTC_PCF8523 rtc;                        //  Assign the RTC_PCF8523 to the rtc name
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
File logfile;                           //  Assign the File object to the logfile name
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  global data/variables
uint32_t timer         = millis();     // unsigned 32bit interger variable called timer assigned to the millisecond function
uint32_t logRecSeq     = 0;            // used to sequence the record numbers in the log file
uint8_t navHour        = 0;            // current UCT (GMT) hour data from GPS
uint8_t navMinute      = 0;            // current minute data from GPS
uint8_t navSeconds     = 0;            // current seconds data from GPS
uint16_t navMillsec    = 0;            // current milliseconds data from GPS
uint8_t navMonth       = 0;            // current month data from GPS
uint8_t navDay         = 0;            // current day data from GPS
uint16_t navYear       = 0;            // current year data from GPS
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
byte batErrCount       = 0;            // Used to throttle the low voltage error message 
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
float measuredVBat     = 0;            // float to hold the measured batttery voltage
const char navName[11] = "RogerNAV01"; // name of the bluetooth LE nav module
const char cmdName[11] = "RogerCMD01"; // name of the bluetoothe LE command module
const int bleBuffSize  = 127;          // size of bluetooth buffer size
int bleRSSI            = 0;            // BLE Signal Strength
//
//
byte bleBuffer[bleBuffSize];           // buffer to hold data to/from the bluetooth comm connection
//
boolean usingInterrupt = false;
boolean bleCMDMode     = false;        // tells you if the Bluetooth LE interface is in CMD mode
boolean stringComplete = false;        // whether the string is complete
boolean dateRefreshed  = false;        // updated when the date is refreshed from GPS
boolean dsTime         = false;        // standard time when false.  daylight saving when true.
boolean trace          = true;         // shows serial print data when true
//
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
//
String inputString     = "";           // a string to hold incoming heading data.  Space is allocated in the setup function
//
//  Using HW Serial (Serial1)
Adafruit_BluefruitLE_UART ble(bleSerial, BLUEFRUIT_UART_MODE_PIN);
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//GPS Interrupt Routines
//
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}
//
//
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
//
void useInterrupt(boolean);   // Func prototype keeps Arduino 0023 happy
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// blink out an error code
void error(uint8_t errno) {
  while (errno != 0) {
    uint8_t i;
    for (i = 0; i < errno; i++) {
      digitalWrite(sysLED, HIGH);
      digitalWrite(msgRed, LOW);
      digitalWrite(msgGreen, HIGH);
      digitalWrite(msgBlue, HIGH);
      sounder(50);
      delay(100);
      digitalWrite(sysLED, LOW);
      digitalWrite(msgRed, HIGH);
      digitalWrite(msgGreen, HIGH);
      digitalWrite(msgBlue, HIGH);
      delay(100);
    }
    for (i = errno; i < 10; i++) {
      delay(300);
    }
    if (errno > 6) {
      errno = 0;
      return;
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
    if (trace) {
      Serial.println("Card init. failed!");
    }
    error(2);
  }
  char filename[15];
  strcpy(filename, "NAVDAT00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i / 10;
    filename[7] = '0' + i % 10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  //
  logfile = SD.open(filename, FILE_WRITE);
  if ( ! logfile ) {
    if (trace) {
      Serial.print("Couldnt create ");
      Serial.println(filename);
    }
    error(3);
  }
  if (trace) {
    Serial.print("Writing to ");
    Serial.println(filename);
  }
  writeHeadingRow();
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void writeHeadingRow() {
  logfile.print("Sequence:");      logfile.print(",Time:");
  logfile.print(",Date:");         logfile.print(",Fix:");
  logfile.print(",Quality:");      logfile.print(",Latitude:");
  logfile.print(",Longitude:");    logfile.print(",Speed(knots):");
  logfile.print(",Angle:");        logfile.print(",Altitude:");
  logfile.print(",Satellites:");   logfile.print(",Roll:");
  logfile.print(",Pitch:");        logfile.print(",Heading:");
  logfile.print(",QW:");           logfile.print(",QX:");           
  logfile.print(",QY:");           logfile.print(",QZ:");           
  logfile.print(",BattVolts:");    logfile.println();
  logfile.flush();
}
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void initGPS() {
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  delay(250);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  delay(250);
  // uncomment this line to turn on only the "minimum recommended" data
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  delay(250);
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
   useInterrupt(true);
   delay(1000);
   // Ask for firmware version
   gpsSerial.println(PMTK_Q_RELEASE);
   delay(250);
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void parseGPSData() {
//  char c = GPS.read();
  if (GPS.newNMEAreceived()) {          // if a sentence is received, we can check the checksum, parse it...
//  Serial.println(GPS.lastNMEA());     // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA()))     // this also sets the newNMEAreceived() flag to false
       return;
    }                                   // a tricky thing here is if we print the NMEA sentence, or data
}                                       // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void parseAndStoreNAVData() {
  if (timer > millis()) {
    timer = millis();                      // if millis() or timer wraps around, we'll just reset it
    }
  getHeadingInfo();
  parseGPSData();                          // Parse GPS data
  if ((millis() - timer) > 2000) {          // approximately every 2 seconds or so, get and store/print out the current dasta
    timer = millis();                      // reset the timer
    loadGPSData();                         // Load GPS data
    if (gpsFix) {
      digitalWrite(gpsLock, HIGH);
      refreshRTC();
//      computeHeading();
      batteryCheck();
      digitalWrite(sysLED, HIGH);
      printGPSData();
      delay(50);
      writeNavData();
      digitalWrite(sysLED, LOW);
    } else {
      digitalWrite(gpsLock, LOW);
      //sounder(100);   
    }
  }
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void loadGPSData() {
/*  uint8_t gpsQualHold = GPS.fixquality;
  if ((gpsQualHold < 1) || (gpsQualHold > 2)) {   //  check to see if the data quality value is not equal to 1 or 2
    return;                                       //  if so, then don't refresh the data in global storage and exit this routine       
    }
  float gpsLatitudeHold  = GPS.latitude;
  if (gpsLatitudeHold < 1.00) {                   //  temporary code to reduce data errors  Not for use within 66 miles of north pole!
    return;
    }
  float gpsLongitude  = GPS.longitude;
  if (gpsLongitude ==  0.00) {                   //  temporary code to reduce data errors  Never going to be exactly on the prime meridian
    return;
    }
 */
  navHour = GPS.hour;
  navMinute = GPS.minute;
  navSeconds = GPS.seconds;
  navMillsec = GPS.milliseconds;
  navMonth = GPS.month;
  navDay = GPS.day;
  navYear = GPS.year;
  uint8_t gpsFixHold = GPS.fix;
  if (gpsFixHold) {
    gpsFix       = GPS.fix;
    gpsQual      = GPS.fixquality;
    gpsLatitude  = GPS.latitude;
    gpsLat       = GPS.lat;
    gpsLongitude = GPS.longitude;
    gpsLon       = GPS.lon;
    gpsKnots     = GPS.speed;
    gpsAngle     = GPS.angle;
    gpsAltitude  = GPS.altitude;
    gpsSats      = GPS.satellites;
    convertGPSToDMS();
    computeDecimalDeg();
  }
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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void printGPSData() {
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
//    Serial.print(" QHead: "); Serial.print(qHead);
    Serial.print(" QW: "); Serial.print(qW);
    Serial.print(" QX: "); Serial.print(qX);
    Serial.print(" QY: "); Serial.print(qY);
    Serial.print(" QZ: "); Serial.println(qZ);
    Serial.print(" VBat: " ); Serial.println(measuredVBat, 2);
  }
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void initBLE() {                           /* Initialise the module */
  if (trace) {
    Serial.println(F("Initialising the Bluefruit LE module: ")); }
  digitalWrite(msgRed, HIGH);
  digitalWrite(msgGreen, LOW); 
  digitalWrite(msgBlue, HIGH);
  delay(250);  
  if (!bleCMDMode) {
    bleCMDOn();
    }
  if (!ble.begin(VERBOSE_MODE) ) {
    delay(250);
    digitalWrite(msgGreen, HIGH);
    digitalWrite(msgRed, LOW);
    digitalWrite(msgBlue, HIGH);
    if (trace) {
      Serial.println(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
      }
    error(5);
  } else {
    delay(250);
    if (trace) {
      Serial.println(F("OK!"));
    }
  }
  if ( FACTORYRESET_ENABLE )   {            /* Perform a factory reset to make sure everything is in a known state */
    if (trace) {
      Serial.println(F("Performing a factory reset: "));
    }
    if (!ble.factoryReset() ) {
      digitalWrite(msgGreen, HIGH);
      digitalWrite(msgRed, LOW);
      digitalWrite(msgBlue, HIGH);
      if (trace) {
        Serial.println(F("Couldn't factory reset"));
        }      
      error(6);
      }
    }
  ble.echo(false);                         /* Disable command echo from Bluefruit */
  if (trace) {
    Serial.println("Requesting Bluefruit info:");
    ble.info();                             /* Print Bluefruit information */
    }
  delay(250);
  bleCMDOff();
  digitalWrite(msgRed, HIGH);
  digitalWrite(msgGreen, HIGH);
  digitalWrite(msgBlue, LOW);
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
void bleCMDOn() {
  digitalWrite(BLUEFRUIT_UART_MODE_PIN, HIGH);
  bleCMDMode = true;
  delay(25);
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
void bleCMDOff() {
  digitalWrite(BLUEFRUIT_UART_MODE_PIN, LOW);
  bleCMDMode = false;
  delay(25);
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
void bleKBDDevOff() {
  if (!bleCMDMode) {
    bleCMDOn();
  }
  bleSerial.println("AT+BLEKEYBOARDEN=0");
  ble.waitForOK();
  bleSerial.println("ATZ");
  ble.waitForOK();
  bleCMDOff();
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void setNavName() {
  uint8_t count = 0;
  uint8_t nameSize = 10;
  boolean eod = false;
  if (!bleCMDMode)  {  
    bleCMDOn();
  }
  bleSerial.println("AT+GAPDEVNAME");
  while (!bleSerial.available()) {
    delay(1);
  }
  for (count = 0; (count < 126 && !eod); count++) {
    bleBuffer[count] = bleSerial.read();
    if (bleBuffer[count] == '\n') {
      eod = true;
    }
  }
  boolean nameMatch = false;
  count = 0;
  do {
    if (bleBuffer[count] == navName[count]) {
      nameMatch = true;
      count++;
    } else {
      nameMatch = false;
    }
  } while (nameMatch && count < 10);
  if (!nameMatch) {
    bleSerial.print("AT+GAPDEVNAME=");
    bleSerial.println(navName);
    ble.waitForOK();
    bleSerial.println("ATZ");
    ble.waitForOK();
  } else {
    if (trace) {
      Serial.print("BLE Name: ");
      Serial.println(navName);
    }
  }
  bleCMDOff();
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void setBLEdBm(int dBm) {
  if (!bleCMDMode) {
    bleCMDOn();
    }
//  if (dBm != -40 || -20 || -16 || -12 ||
//      -8 ||  -4 ||   0 ||  4) {
    dBm = 4;
//    }
  bleSerial.print("AT+BLEPOWERLEVEL=");
  bleSerial.print(dBm);
  ble.waitForOK();
  bleCMDOff();
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
void batteryCheck() {
  delay(10);
  measuredVBat = analogRead(VBATPIN);
  if (trace) {
    Serial.print("Measured A7 Pin Value: ");
    Serial.println(measuredVBat, 2);
    }
  measuredVBat = ((measuredVBat * 3.7) / 1023);                // Multiply by 3.7, our reference voltage and divide by 1023, which will give us the voltage
  if (measuredVBat < 3.0) {
    batErrCount++;
    }
  if (batErrCount > 50) {
    batErrCount = 0;
    error(9);
    }
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void getHeadingInfo() {
  stringComplete = false;
  while (imuSerial.available() && !stringComplete) {
    char inChar = (char)imuSerial.read();   // get the new byte:
    switch (inChar) {                       // check for data type indicator
      case 'H':
        heading = imuSerial.parseFloat();
        break;
      case 'R':
        roll = imuSerial.parseFloat();
        break;
      case 'P':
        pitch = imuSerial.parseFloat();
        break;
      case 'W':
        qW = imuSerial.parseFloat();
        break;
      case 'X':
        qX = imuSerial.parseFloat();
        break;
      case 'Y':
        qY = imuSerial.parseFloat();
        break;
      case 'Z':
        qZ = imuSerial.parseFloat();
        break;
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it:
      case '\n':
        stringComplete = true;
        break;
      default:
        stringComplete = false;
        break;
    }
  }
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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void writeNavData() {
  digitalWrite(sdWrite, LOW);
// SEQUENCE
  logRecSeq++;
  logfile.print(logRecSeq, DEC); logfile.print(',');
// TIME
  logfile.print(navHour, DEC); logfile.print(':');
  logfile.print(navMinute, DEC); logfile.print(':');
  logfile.print(navSeconds, DEC); logfile.print(',');
//DATE
  logfile.print(navMonth, DEC); logfile.print('/');
  logfile.print(navDay, DEC); logfile.print("/20");
  logfile.print(navYear, DEC); logfile.print(',');
//FIX
  logfile.print(gpsFix); logfile.print(',');
//QUALITY
  logfile.print(gpsQual); logfile.print(',');
//LOCATION (Lat/Long)
  logfile.print(gpsLatitude, 4); logfile.print(gpsLat);
  logfile.print(',');
  logfile.print(gpsLongitude, 4); logfile.print(gpsLon);
  logfile.print(',');
//SPEED
  logfile.print(gpsKnots); logfile.print(',');
//ANGLE
  logfile.print(gpsAngle, 2); logfile.print(',');
//ALTITUDE
  logfile.print(gpsAltitude); logfile.print(',');
//SATELITES
  logfile.print(gpsSats); logfile.print(',');
//ROLL
  logfile.print(roll); logfile.print(',');
//PITCH
  logfile.print(pitch); logfile.print(',');
//HEADING
  logfile.print(heading); logfile.print(',');
//COMPUTED QUATERNION HEADING
//  logfile.print(qHead); logfile.print(',');
//QUATERNION W
  logfile.print(qW); logfile.print(',');
//QUATERNION X
  logfile.print(qX); logfile.print(',');
//QUATERNION Y
  logfile.print(qY); logfile.print(',');
//QUATERNION Z
  logfile.print(qZ); logfile.print(',');
//BATTERY VOLTAGE
  logfile.print(measuredVBat, 2);
  logfile.println();
//FLUSH THE BUFFER
  logfile.flush();
  digitalWrite(sdWrite, HIGH);
}
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void checkForCMDDataReq() {
  if (bleCMDMode) {
    bleCMDOff();
  }
  stringComplete = false;
  while (bleSerial.available() && !stringComplete) {
    char bleChar = (char)bleSerial.read();            // get the new byte:
    if (bleChar >= 'a' && bleChar <= 'z') {
      bleChar = bleChar & ~ (0x20);
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
        dataError();
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
      case '\r':
      case '\n':
        stringComplete = true;
        bleSerial.flush();

        while (bleSerial.available()) {
          bleSerial.read();
        }
        break;
      default:
        bleSerial.println("CMD ERROR" );
        bleSerial.flush();
        stringComplete = false;
        break;
    }
  }
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void Altitude() {
  bleSerial.print("ALT = " );
  bleSerial.println(gpsAltitude, DEC );
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void Battery() {
  bleSerial.print("BATT = " );
  bleSerial.println(measuredVBat, 2);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void Date() {
  bleSerial.print("DATE = " );
  bleSerial.print(navMonth, DEC);
  bleSerial.print('/');
  bleSerial.print(navDay, DEC);
  bleSerial.print("/20");
  bleSerial.println(navYear, DEC);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void dataError() {
  bleSerial.print("ERR = " );
  bleSerial.print(gpsLatError, DEC);
  bleSerial.print(',');
  bleSerial.println(gpsLonError, DEC);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void GPSLoc() {
  bleSerial.print("GPSLAT = " );
  bleSerial.print(gpsLatitude, 4);
  bleSerial.println(gpsLat);
  bleSerial.print("GPSLON = " );
  bleSerial.print(gpsLongitude, 4);
  bleSerial.println(gpsLon);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void Heading() {
  bleSerial.print("HEAD = " );
  bleSerial.println(heading, 2);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void DegMinSec() {
  bleSerial.print("LATDMS = " );
  bleSerial.print(gpsLatDeg); bleSerial.print(',');
  bleSerial.print(gpsLatMin); bleSerial.print(',');
  bleSerial.println(gpsLatSec, 2);
  bleSerial.print("LONDMS = " );
  bleSerial.print(gpsLonDeg); bleSerial.print(',');
  bleSerial.print(gpsLonMin); bleSerial.print(',');
  bleSerial.println(gpsLonSec, 2);
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
  Altitude();
  Satellites();
  bleSignal();
  dataError();
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void DecDegLoc() {
  bleSerial.print("DECLAT = " );
  bleSerial.println(decimalDegLat, 6);
  bleSerial.print("DECLON = " );
  bleSerial.println(decimalDegLon, 6);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void RecSequence() {
  bleSerial.print("RSEQ = " );
  bleSerial.println(logRecSeq);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void bleSignal() {
  bleSerial.print("RSSI = " );
  bleSerial.println(bleRSSI, DEC);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
void Satellites() {
  bleSerial.print("SAT = " );
  bleSerial.println(gpsSats);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void Time() {
  bleSerial.print("TIME = " );
  bleSerial.print(navHour, DEC); bleSerial.print(':');
  bleSerial.print(navMinute, DEC); bleSerial.print(':');
  bleSerial.println(navSeconds, DEC);
}
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void Velocity() {
  bleSerial.print("KNOTS = " );
  bleSerial.println(gpsKnots, 2);
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
void getBLERSSI() {
  if (!bleCMDMode) {
    bleCMDOn();
    }
  bleSerial.println("AT+BLEGETRSSI");
  while (!bleSerial.available()) {
    delay(1);
    }
  bleRSSI = bleSerial.parseInt();
  ble.waitForOK();
  if (bleCMDMode) {
    bleCMDOff();
    }
  if (bleRSSI >= 0) {
    digitalWrite(msgRed, HIGH);
    digitalWrite(msgGreen, HIGH);
    digitalWrite(msgBlue, LOW);
    return;
    }
  if (bleRSSI > -24) {
    digitalWrite(msgRed, LOW);
    digitalWrite(msgGreen, LOW);
    digitalWrite(msgBlue, LOW);
    return;
    } 
  if (bleRSSI > -48) {
    digitalWrite(msgRed, HIGH);
    digitalWrite(msgGreen, LOW);
    digitalWrite(msgBlue, LOW);
    return;
    }
  if (bleRSSI > -96) {
    digitalWrite(msgRed, HIGH);
    digitalWrite(msgGreen, LOW);
    digitalWrite(msgBlue, HIGH);
    return;
    }
  digitalWrite(msgRed, LOW);
  digitalWrite(msgGreen, HIGH);
  digitalWrite(msgBlue, HIGH);
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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void bleStart() {
  initBLE();
  delay(250);
  bleKBDDevOff();
  delay(250);
  setNavName();
  delay(250);
  setBLEdBm(-4);
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void configIOPins() {
  pinMode(BLUEFRUIT_UART_MODE_PIN, OUTPUT);
  if (!bleCMDMode) { 
    bleCMDOn(); }
  pinMode(gpsLock, OUTPUT);
  digitalWrite(gpsLock, LOW);
  pinMode(sdWrite, OUTPUT);
  digitalWrite(sdWrite, HIGH);
  pinMode(bleXfer, OUTPUT);
  digitalWrite(bleXfer, HIGH);
  pinMode(msgRed, OUTPUT);
  digitalWrite(msgRed, HIGH);
  pinMode(msgGreen, OUTPUT);
  digitalWrite(msgGreen, HIGH);
  pinMode(msgBlue, OUTPUT);
  digitalWrite(msgBlue, HIGH);
  pinMode(msgSounder, OUTPUT);
  pinMode(sysLED, OUTPUT);
//
  testLEDsSounder();
}
//
//
void testLEDsSounder() {
  digitalWrite(sysLED, HIGH);
  sounder(500);
  digitalWrite(sysLED, LOW);
//  
  digitalWrite(gpsLock, HIGH);
  delay(250);
  digitalWrite(gpsLock, LOW);
//
  digitalWrite(sdWrite, LOW);
  delay(250);
  digitalWrite(sdWrite, HIGH);
//  
  digitalWrite(bleXfer, LOW);
  delay(250);
  digitalWrite(bleXfer, HIGH);
//
  digitalWrite(msgRed, LOW);
  delay(500);  
  digitalWrite(msgRed, HIGH);
  digitalWrite(msgGreen, LOW);
  delay(500);    
  digitalWrite(msgGreen, HIGH);
  digitalWrite(msgBlue, LOW);
  delay(500);  
  digitalWrite(msgBlue, HIGH);    
}
//
//
void sounder(int period) {
  digitalWrite(msgSounder, HIGH);
  delay(period);
  digitalWrite(msgSounder, LOW);
}
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void configSerialPorts() {
  trace = false;
  delay(250);
  if (trace) {
    Serial.begin(115200);                                                // start the Serial trace port
    delay(250);
    Serial.println(F("Starting Kayak Navigational Subsystem"));
    delay(250);
  }
  bleSerial.begin(9600);                                              // start the Bluetooth LE serial port
  delay(250);
  imuSerial.begin(9600);                                              // start the IMU serial port
  delay(250);
}
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void setup() {
  configSerialPorts();               // initialize serial:
  configIOPins();                    // configure the IO pins
  inputString.reserve(BUFSIZE);      // reserve 256 bytes for the inputString:
  checkForRTC();                     // Check for an RTC
  delay(250);
  getRTCData();                      // get RTC date/time
  delay(250);
  sdCardInit();                      // initialize the SD card
  delay(250);
  initGPS();                         // initialize the GPS module
  delay(250);
  bleStart();                        // start the Bluetooth LE interface
  delay(250);
  if (trace) {
    Serial.println("Ready!");
    delay(250);
  }
}
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void loop() {
  parseAndStoreNAVData();                // parse and store the navigational data on the SD card and update voliatle memory
  if (bleSerial.available()) {
    digitalWrite(bleXfer, LOW);
    checkForCMDDataReq();                // check for and respond to BLE inteface/serial port for data commands/requests
    bleSerial.flush();                   // flush the ble serial port before doing anything else
    //getBLERSSI();                        // get the latest RSSI/dBm signal strength reading
  }  else {
    digitalWrite(bleXfer, HIGH);
  }
}
//
//
//

