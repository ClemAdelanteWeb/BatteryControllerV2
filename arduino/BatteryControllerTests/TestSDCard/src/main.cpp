#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>

SdFat sd;
SdFile logFile;

uint8_t SDCardPinSelect = 10;
#define DataLogFile "LOGTEST.TXT"
int count = 0;

void readSDCard();
void writeData();
void deleteFile();

void setup() {
  Serial.begin(38200);
   if (!sd.begin(SDCardPinSelect))
  {
    Serial.println("SD CARD ERROR SETUP");

    sd.initErrorHalt();
  }

   deleteFile();
}

void loop() {
    readSDCard();
    writeData();
  delay(5000);
}


void writeData() {
  
   if (!logFile.open(DataLogFile, O_RDWR | O_CREAT | O_AT_END))
  {
     sd.errorHalt("opening test.txt for write failed");
     Serial.println("ERROR OPENING FILE");
  }
  else
  {
    String newData = "New Data "+ (String) count;
    logFile.println(newData);
    logFile.close();
  }

  count++;
}

void readSDCard()
{
  int data;


  if (!logFile.open(DataLogFile, O_READ))
  {
    Serial.println(F("NO SD FILE"));
  }

  Serial.println(F("SD CARD OK"));


  while ((data = logFile.read()) >= 0)
  {
    Serial.write(data);

    delay(2);
  }

  // close the file:
  logFile.close();
}

void deleteFile()
{
  sd.remove(DataLogFile);
}