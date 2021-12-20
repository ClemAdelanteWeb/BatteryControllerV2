#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>

void setup() {
  Serial.begin(9600);
}

void loop() {
  tmElements_t tm;

  if (RTC.read(tm)) {
       Serial.print(tmYearToCalendar(tm.Year));
    Serial.print('/');
       Serial.print(tm.Month);
    Serial.print('/');
       Serial.print(tm.Day);
       Serial.print(' ');
      Serial.print(tm.Hour);
    Serial.print(':');
      Serial.print(tm.Minute);
    Serial.print(':');
      Serial.print(tm.Second);
  } 

  Serial.println();
  delay(3000);
}
