void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void ReadGPSandSendSMS(char sendto[12])
{
        char gpsdata[80];
        fona.getGPS(0, gpsdata, 80);
        if (type == FONA808_V1)
          Serial.println(F("Reply in format: mode,longitude,latitude,altitude,utctime(yyyymmddHHMMSS),ttff,satellites,speed,course"));
        else 
          Serial.println(F("Reply in format: mode,fixstatus,utctime(yyyymmddHHMMSS),latitude,longitude,altitude,speed,course,fixmode,reserved1,HDOP,PDOP,VDOP,reserved2,view_satellites,used_satellites,reserved3,C/N0max,HPA,VPA"));
        Serial.println(gpsdata);
        
        if (!fona.sendSMS(sendto, gpsdata)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
        }
}
