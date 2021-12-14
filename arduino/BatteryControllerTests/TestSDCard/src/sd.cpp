#include <Arduino.h>

#include <SPI.h>
#include <SD.h>

const int chipSelect = 10;

void setup() {
  Serial.begin(19200);


  while (!Serial);

  if (!SD.begin(chipSelect)) {
    while (true);
  }
}

void loop() {

  File dataFile = SD.open("log.txt", FILE_WRITE);

  if (dataFile) {
    dataFile.println("unelignededata");
    dataFile.close();
  }
}