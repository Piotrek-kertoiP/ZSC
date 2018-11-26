#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>

#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// this is a large buffer for replies
char replybuffer[255];

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
  Serial.print(F("Found "));
  
  switch (type) {
    case FONA808_V2:
      Serial.println(F("FONA 808 (v2)")); break;
    default: 
      Serial.println(F("???")); break;
  }
  
  // Print SIM card IMEI number.
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("SIM card IMEI: "); Serial.println(imei);
  }
  printMenu();
}
//-----------------------------------------------------------------------LOOP---------------------------------------------------------------------------------------------
void loop() {
  //lazyShow();
  //eagerShow();
  //showWithGPS();  //doesn't work yet
  obtainingGPSsignalAttempt();
}
//-----------------------------------------------------------------------PRINTING MENU (FOR LAZY SHOW PURPOSES)-----------------------------------------------------------
void printMenu(void) {
  Serial.println(F("-------------------------------------"));
  
  // SMS
  Serial.println(F("Type anything to send SMS 'no czesc' to 600549956"));

  Serial.println(F("-------------------------------------"));
  Serial.println(F(""));

}
//-----------------------------------------------------------------------LAZY SHOW----------------------------------------------------------------------------------------
void lazyShow(){
  Serial.print(F("FONA> "));
  while (! Serial.available() ) {
    if (fona.available()) {
      Serial.write(fona.read());
    }
  }

  char command = Serial.read();
  Serial.println(command);
  
  sendSMS();
  printMenu();
}
//-----------------------------------------------------------------------EAGER SHOW---------------------------------------------------------------------------------------
void eagerShow(){
  sendSMS();
  delay(5000);
}
//-----------------------------------------------------------------------SENDING HARDCODED SMS----------------------------------------------------------------------------
void sendSMS(){
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
//--------------------------------------------------------------------------GPS STUFF (START)------------------------------------------------------------------------------
void obtainingGPSsignalAttempt(){
  Serial.print(F("FONA> "));
  while (! Serial.available() ) {
    if (fona.available()) {
      Serial.write(fona.read());
    }
  }

  Serial.println(F("Type anything to make an attempt of obtaining GPS data"));

  char command = Serial.read();
  Serial.println(command);
  
  turnGPSon();
  getGPSdata();
  turnGPSoff();
}

/*void showWithGPS(){
  //doesn't work yet
  char x_coord[80];
  char y_coord[80];
  char velocity[80];
  
  turnGPSon();
  getGPSdata(&x_coord, &y_coord, &velocity);
  turnGPSoff();
  
  sendSMSwithGPSdata(x_coord, y_coord, velocity);
}*/

void turnGPSoff(){
  if (!fona.enableGPS(false))
    Serial.println(F("Failed to turn off"));
}

void turnGPSon(){
  if (!fona.enableGPS(true))
    Serial.println(F("Failed to turn on"));
}

void getGPSdata(){
  // check for GPS location
  char gpsdata[80];
  fona.getGPS(0, gpsdata, 80);
  Serial.println(F("Reply in format: mode,fixstatus,utctime(yyyymmddHHMMSS),latitude,longitude,altitude,speed,course,fixmode,reserved1,HDOP,PDOP,VDOP,reserved2,view_satellites,used_satellites,reserved3,C/N0max,HPA,VPA"));
  Serial.println(gpsdata);
}

/*void sendSMSwithGPSdata(x_coord, y_coord, velocity){
  //todo
}*/
//----------------------------------------------------------------------------GPS STUFF (END)----------------------------------------------------------------------------
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
