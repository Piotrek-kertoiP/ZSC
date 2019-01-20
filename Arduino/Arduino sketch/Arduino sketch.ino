#include "Adafruit_FONA.h"
#include <Wire.h>
#include <Accelerometer.h>


//#define LIS3DH_RANGE_4_G  0b01        //zakomentowane Piotrek 13.01.2019
Accelerometer lis = Accelerometer();

#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4

#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

uint8_t type;

void setup() {

 //----------------------FONA SHIELD INIT--------------------------------------
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
  
  fona.setGPRSNetworkSettings(F("internet"), F(""), F(""));

  //------------------------------------------------ACCELEROMETER INIT-----------------------------------------------

  Serial.begin(115200);
  Serial.println("LIS3DH test!");
  
  if (! lis.begin()) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");
  
  // Ustawienie zakresu odczytywanych wartości odczytywanych przeciążeń na +-4G
  lis.setRange(LIS3DH_RANGE_4_G);                                    //zamienione na funkcję, Piotrek, 13.01.2019
  
  Serial.print("Range = "); Serial.print(2 << LIS3DH_RANGE_4_G);     //LIS3DH_RANGE_4_G = 0b01 = 1 (decimal) => 2 << LIS3DH_RANGE_4_G = 4
  Serial.println("G");
}

//-----------------------MAIN LOOP--------------------------------------------------------

void loop() {
  // char mystr[255];
   String mystr;
   double x2=0,y2=0,z2=0;
   bool i = false;
   
while(1){
    lis.read();      // get X Y and Z data at once
    Serial.print("X:  "); Serial.print(lis.x); 
    Serial.print("  \tY:  "); Serial.print(lis.y); 
    Serial.print("  \tZ:  "); Serial.print(lis.z); 
    Serial.println();

    if(i && (abs(x2-lis.x)>20000 || abs(y2-lis.y)>20000 || abs(z2-lis.z)>20000))
    {
        sendSMSwithGPRSdata(mystr);
        if (!fona.enableGPRS(false))
          Serial.println(F("Failed to turn off"));
        else Serial.println(F("Turned off"));
        delay(20000);
    }

    i=true;
    x2=lis.x;y2=lis.y;z2=lis.z;
    /* Or....get a new sensor event, normalized */ 
    //sensors_event_t event; 
    //lis.getEvent(&event);
    delay(200); 
  }
 
   //for(int i=0;i<100;i++) Serial.print(mystr[i]);
   Serial.print(mystr);
}


//------------------------------------------------------------------------

void sendSMSwithGPRSdata(String gpsdata2){
  char gpsdata[255];
  
  gpsdata2+="ALERT BEZPIECZENSTWA!\n ";
  uint16_t returncode;
 delay(10000);
 if (!fona.enableGPRS(true))
          Serial.println(F("Failed to turn on"));
      else Serial.println(F("success"));
 
 
    fona.getGSMLoc(&returncode, gpsdata, 250);
    gpsdata2+=gpsdata;
    Serial.println(gpsdata2);
    char sendto[10] = "664913864";
     for(int i=0;i<100;i++)gpsdata[i]=gpsdata2[i];
    flushSerial();
   if (!fona.sendSMS(sendto, gpsdata)) {
      Serial.println(F("Failed"));
    }
    else {
      Serial.println(F("Sent!"));
    } 
}





//-----------------------------------Aditional funcs-------------------------------------------

void flushSerial() {
  while (Serial.available())
    Serial.read();
}

char readBlocking() {
  while (!Serial.available());
  return Serial.read();
}
uint16_t readnumber() {
  uint16_t x = 0;
  char c;
  while (! isdigit(c = readBlocking())) {
    //Serial.print(c);
  }
  Serial.print(c);
  x = c - '0';
  while (isdigit(c = readBlocking())) {
    Serial.print(c);
    x *= 10;
    x += c - '0';
  }
  return x;
}

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
