#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>

#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// this is a large buffer for replies
//char replybuffer[255];

// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

uint8_t type;
//--------------------------------------------------------------------------SETUP-----------------------------------------------------------------------------------------
void setup() {
  while (!Serial);

  Serial.begin(115200);
  Serial.println(F("FONA basic test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  fonaSerial->begin(4800);

  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }

  type = fona.type();
  Serial.println(F("FONA is OK"));
  if( type == FONA808_V2 ){
    Serial.println(F("Found FONA 808 (v2)"));
  }
  else{
    Serial.println(F("No FONA found :("));

  // Print SIM card IMEI number.
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("SIM card IMEI: "); Serial.println(imei);
  }
}
}
//-----------------------------------------------------------------------LOOP---------------------------------------------------------------------------------------------
void loop() {
  //lazyShow();
  //eagerShow();
  //obtainingGPSsignalAttempt();
  //eagerObtainingGPSsignalAttempt();
  sendSMSwithGPSdata();
}
//-----------------------------------------------------------------------LAZY SHOW----------------------------------------------------------------------------------------
void lazyShow(){
  printLazyShowMenu();
  Serial.print(F("FONA> "));
  while (! Serial.available() ) {
    if (fona.available()) {
      Serial.write(fona.read());
    }
  }

  char command = Serial.read();
  Serial.println(command);

  sendSMS();
}
//-----------------------------------------------------------------------EAGER SHOW---------------------------------------------------------------------------------------
void eagerShow() {
  sendSMS();
  delay(5000);
}
//-----------------------------------------------------------------OBTAINING GPS SIGNAL IN LOOP------------------------------------------------------------------
void eagerObtainingGPSsignalAttempt() {
  turnGPSon();
  while (1) {
    Serial.println(F("Waiting a minute..."));
    delay(60000);     //wait a minute
    getGPSdata();
  }
}
//-----------------------------------------------------------------OBTAINING GPS SIGNAL (LAZY)-----------------------------------------------------------------
void obtainingGPSsignalAttempt() {
  printGPSSignalAttemptMenu();
  Serial.print(F("FONA> "));
  while (! Serial.available() ) {
    if (fona.available()) {
      Serial.write(fona.read());
    }
  }

  char command = Serial.read();
  Serial.println(command);

  turnGPSon();
  Serial.println(F("Waiting 10 seconds for GPS to start"));
  delay(10000);
  getGPSdata();
  turnGPSoff();
}
//-----------------------------------------------------------------------SENDING HARDCODED SMS----------------------------------------------------------------------------
void sendSMS() {
  char message[141] = "no czesc";
  char sendto[10] = "600549956";
  flushSerial();
  if (!fona.sendSMS(sendto, message)) {
    Serial.println(F("Failed"));
  }
  else {
    Serial.println(F("Sent!"));
  }
}
//------------------------------
void turnGPSoff() {
  if (!fona.enableGPS(false))
    Serial.println(F("Failed to turn off"));
  else
    Serial.println(F("Turned GPS off"));
}
//--------------------------------------------------
void turnGPSon() {
  if (!fona.enableGPS(true))
    Serial.println(F("Failed to turn on"));
  else
    Serial.println(F("Turned GPS on"));
}
//-----------------------------------------------------
void getGPSdata() {
  // check for GPS location
  char gpsdata[80];
  fona.getGPS(0, gpsdata, 80);
  Serial.println(F("Reply in format: mode,fixstatus,utctime(yyyymmddHHMMSS),latitude,longitude,altitude,speed,course,fixmode,reserved1,HDOP,PDOP,VDOP,reserved2,view_satellites,used_satellites,reserved3,C/N0max,HPA,VPA"));
  Serial.println(gpsdata);
}
//-------------------------------------------------------------
void sendSMSwithGPSdata(){
  char gpsdata[80];
  turnGPSon();
  fona.getGPS(0, gpsdata, 80);
  //parse gpsdata somehow - extract coordinates and speed
  char sendto[10] = "600549956";
  flushSerial();
  if (!fona.sendSMS(sendto, gpsdata)) {
    Serial.println(F("Failed"));
  }
  else {
    Serial.println(F("Sent!"));
  }
  delay(10000);
}
//----------------------------------------------------------------------------GPS STUFF (END)----------------------------------------------------------------------------
//-----------------------------------------------------------------------PRINTING MENU (FOR LAZY SHOW PURPOSES)-----------------------------------------------------------
void printLazyShowMenu(void) {
  //todo
  Serial.println(F("-------------------------------------"));
  Serial.println(F("Type anything to send SMS 'no czesc' to 600549956"));
  Serial.println(F("-------------------------------------"));
  Serial.println(F(""));
}
//----------------------------------------------------------------------
void printGPSSignalAttemptMenu(void) {
  //todo
  Serial.println(F("-------------------------------------"));
  Serial.println(F("Type anything to make an attempt of obtaining GPS data"));
  Serial.println(F("-------------------------------------"));
  Serial.println(F(""));
}
//----------------------------------------------------------------------------------------------------------------------------------------

void flushSerial() {
  while (Serial.available())
    Serial.read();
}

//this function will be deleted at the end of project
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;

  while (true) {
    if (buffidx > maxbuff) {
      //Serial.println(F("SPACE"));
      break;
    }

    while (Serial.available()) {
      char c =  Serial.read();

      //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);

      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)   // the first 0x0A is ignored
          continue;

        timeout = 0;         // the second 0x0A is the end of the line
        timeoutvalid = true;
        break;
      }
      buff[buffidx] = c;
      buffidx++;
    }

    if (timeoutvalid && timeout == 0) {
      //Serial.println(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;  // null term
  return buffidx;
}
