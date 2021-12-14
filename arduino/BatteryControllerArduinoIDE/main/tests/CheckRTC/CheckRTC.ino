 #include "RTClib.h"


// RTC date time module
 RTC_DS1307 rtc;

void setup()
{
     
  Serial.begin(19200);



  if (! rtc.begin()) {
     Serial.println("Couldn't find RTC");
   //  Serial.flush();
    while (1) delay(10);
  }
  if (! rtc.isrunning()) {
     Serial.println(F("RTC NOT running"));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
   }

}

void loop() {
  Serial.print("RTC is running : ");
  int isRunning = rtc.isrunning();
  Serial.println(isRunning);
  Serial.println(getDateTime());
  delay(2000);

}

String getDateTime()
{

  // Serial.println("getDateTime");
   DateTime now = rtc.now();
  char heure[50];

  // Affiche l'heure courante retournee par le module RTC
  // Note : le %02d permet d'afficher les chiffres sur 2 digits (01, 02, ....)
  sprintf(heure, "%4d\/%02d\/%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  
 //Serial.println(heure);
   return heure;

}
