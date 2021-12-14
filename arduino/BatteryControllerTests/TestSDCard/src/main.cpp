#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>

#if ENABLE_DEDICATED_SPI
#error "Edit SdFatConfig.h and set ENABLE_DEDICATED_SPI zero"
#endif

SdFat sd;
#define SD_CS_PIN 10  // use your Chip Select pin
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(8))

void setup() {
  Serial.begin(9600);
  while(!Serial) {}
  Serial.println("Type any character to begin");
  while (!Serial.available()) {}
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
  }
  Serial.println("Success");
}

void loop() {}