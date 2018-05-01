/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//  RogerVRU  V05m
//  Will queue VRU mp3 files upon demand via Serial Port (Serial1) requests  
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
// Specifically for use with the Adafruit Feather, the pins are pre-set here!
//
// include SPI, MP3 and SD libraries
#include <SPI.h>
#include <SD.h>
#include <Adafruit_VS1053.h>
#include <stdio.h>
//
//
#define cmdSerial      Serial1    //   Serial comm to RogerCMD MEGA
//
// These are the pins used
#define VS1053_RESET   -1     // VS1053 reset pin (not used!)
//
// Feather M0 or 32u4
#define VS1053_CS       6     // VS1053 chip select pin (output)
#define VS1053_DCS     10     // VS1053 Data/command select pin (output)
#define CARDCS          5     // Card chip select pin
  // DREQ should be an Int pin *if possible* (not possible on 32u4)
#define VS1053_DREQ     9     // VS1053 Data request, ideally an Interrupt pin
//
//
//
uint16_t vruFSNum      = 0;            // File sequence number as a number
uint16_t vruVSeq       = 0;            // File number to play
//
//
boolean voiceRequest   = false;          
boolean cmdError       = false;
boolean trace          = false;
//
//
uint8_t vruSDVocNum    = 0;     // number of vru voice files on the SD card, incremented as read in printDirectory function
uint8_t vruAppVocNum   = 187;   // relative to zero there should be 188 voice files on the SD card for this version of this sketch
//
//
Adafruit_VS1053_FilePlayer musicPlayer = 
  Adafruit_VS1053_FilePlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, CARDCS);
//
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// File listing helper
void printDirectory(File dir, int numTabs) {
   while(true) {
     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       if (trace) {
        Serial.println("**no more files**");
        }
        break;
       }
     for (uint8_t i=0; i<numTabs; i++) {
       if (trace) {
         Serial.print('\t');
         }}
       if (trace) {
         Serial.print(entry.name());
         }
       if (entry.isDirectory()) {
         if (trace) {
           Serial.println("/");
           }
         printDirectory(entry, numTabs+1);
       } else {
       // files have sizes, directories do not
         vruSDVocNum++;
         if (trace) {
           Serial.print("\t\t");
           Serial.println(entry.size(), DEC);
           }}
     entry.close();
   }
}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  Play VRU Files
void playVRUFile(uint16_t fileNum) {
// Play a file in the background, REQUIRES interrupts!
  char vFileSeq[5];
  String vFileName = "RNAV";
  sprintf(vFileSeq, "%.4d", fileNum);
  String vFileType = ".MP3";
  String vruFileStr = vFileName + vFileSeq + vFileType;
  if (trace) {
    Serial.print(F("Playing track: "));
    Serial.println(vruFileStr);
    }
  char vruFileName[13];
  for (int a = 0; a < 13; a++) {
    vruFileName[a] = vruFileStr[a]; 
    }
  if (trace) {
    Serial.println(vruFileName);
    }
  musicPlayer.playFullFile(vruFileName);
//  musicPlayer.startPlayingFile(vruFileName);
//
} 
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void initVRU() {
  if (trace) {
    Serial.println("\n\nAdafruit VS1053 Feather Test");
    }
//  
  if (! musicPlayer.begin()) { // initialise the music player
     if (trace) {
       Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
       }
     while (1);
  }
//
  if (trace) {
    Serial.println(F("VS1053 found")); 
    }
// 
//  
// Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(10,10);
  delay(250);
//  
//
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int
  delay(100);
//  
//
  musicPlayer.sineTest(0x44, 500);    // Make a tone to indicate VS1053 is working
  delay(500);
//  
}
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void checkSDCard() {
  if (!SD.begin(CARDCS)) {
    if (trace) {
      Serial.println(F("SD failed, or not present"));
      }
    do {
      musicPlayer.sineTest(0x44, 500);    // Make a tone to indicate SD Card is found and is OK
      delay(500);
    } while (1); } 
  if (trace) {       
    Serial.println("SD OK!");
    }
//  
  musicPlayer.sineTest(0x44, 500);    // Make a tone to indicate SD Card is found and is OK
  delay(500);
//  
  // list files
  printDirectory(SD.open("/"), 0);
//
  if (vruSDVocNum != (vruAppVocNum + 3)) {
    if (trace) {
      Serial.println(F("Number of Voice Files does not match number required"));
      Serial.print(F("Number of VRU SD Card Vocal Files: "));
      Serial.print(vruSDVocNum, DEC);
      Serial.print("  Number of VRU Files Allowed: ");
      Serial.println(vruAppVocNum, DEC);
    }
    do {
      musicPlayer.sineTest(0x44, 500);    // Make a tone to indicate SD Card is found and is OK
      delay(500);
    } while (1);
  }
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  
void testPlayback() {
  while (vruVSeq < vruAppVocNum) {
    if (musicPlayer.playingMusic) {
      if (trace) {
        Serial.print(".");
        }
      delay(50);    
      } else {
      playVRUFile(vruVSeq);
      vruVSeq++;
      delay(10);
      }
  }
  vruVSeq = 255;
  playVRUFile(vruVSeq);
}
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  
void checkForVRUReq() {
  voiceRequest = false;
  while (cmdSerial.available()) {
    char vru = cmdSerial.read();
    if (vru =='\n') {
       voiceRequest = false;
       break;
      }
    if (vru == 'Q')  {
      vruVSeq = cmdSerial.parseInt();
      if (trace) {
        Serial.println(vruVSeq, DEC);
        }
      voiceRequest = true;
      break;
      }
  }
}
//  
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  
void checkForSerialReq() {
  if (!trace) {
    return;
    }
  voiceRequest = false;
  while (Serial.available()) {
    byte vru = Serial.read();
    if (vru == '\n') {
       voiceRequest = false;
       break;
      }
    if (vru == 'Q')  {
      vruVSeq = Serial.parseInt();
        if (trace) {
          Serial.println(vruVSeq, DEC);
          }
      voiceRequest = true;
      break;
      }
  }
}
//  
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void playWelcome() {
  playVRUFile(131);   //  Welcome is 131
  playVRUFile(255);   //  Ahmet is 255
  delay(500);
  playVRUFile(76);    //  Starting is 76
  playVRUFile(132);   //  RogerCMD is 132
  delay(100);
}
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void setup() {
//  trace = true;
  delay(1000);
  if (trace) {
    Serial.begin(115200);
    while (!Serial) { 
      delay(1); } }                 // Wait for serial port to be opened, remove this line for 'standalone' operation
  if (trace) {
    Serial.println("Starting CMD Serial port");
    }
  cmdSerial.begin(9600);    
  delay(100);  
  initVRU();
  delay(100);
  checkSDCard();
//  testPlayback();
  playWelcome();
  delay(500);                       //  give the serial port time to stabilize for first use
  if (trace) {
    Serial.println("Starting Main Loop");
    }
}
//
// 
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void loop() {
  checkForVRUReq();
  if (voiceRequest) {
    playVRUFile(vruVSeq);
    voiceRequest = false;
    }
  if (trace) {
    checkForSerialReq();
    if (voiceRequest) {
      playVRUFile(vruVSeq);
      voiceRequest = false;
      }
    }   
}
//
//
//

