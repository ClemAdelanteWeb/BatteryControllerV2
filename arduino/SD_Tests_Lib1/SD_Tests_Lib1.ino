// Ported to SdFat from the native Arduino SD library example by Bill Greiman
// On the Ethernet Shield, CS is pin 4. SdFat handles setting SS
const int chipSelect = 10;


#include <SdFat.h>
SdFat sd;
SdFile myFile;

char* filname = "test3.txt";

void setup() {
  Serial.begin(19200);
   if (!sd.begin(chipSelect, SPI_HALF_SPEED)) sd.initErrorHalt();

    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
    readData();
    writeData();
  delay(5000);
}


void writeData() {
  
  if (!myFile.open(filname, O_RDWR | O_CREAT | O_AT_END)) {
   // sd.errorHalt("opening test.txt for write failed");
  }
  
  myFile.println("new data");
  myFile.close();
}
void readData() {
  Serial.println("read data");
  // read from the file until there's nothing else in it:
  int data;

  // re-open the file for reading:
  if (!myFile.open(filname, O_READ)) {
  //  sd.errorHalt("opening test.txt for read failed");
  }
  
  while ((data = myFile.read()) >= 0) Serial.write(data);
  // close the file:
  myFile.close();
  
}
