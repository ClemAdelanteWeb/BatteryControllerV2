


#include <SPI.h>
#include <SD.h>

const int chipSelect = 10;

File myFile;

char* filname = "test4.txt";

void setup() {
  Serial.begin(19200);

    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

   if (!SD.begin(chipSelect)) {
     while (1);
   }
}

void loop() {
    readData();
    writeData();
  delay(5000);
}


void writeData() {
  myFile = SD.open(filname, FILE_WRITE);
 if (myFile) {
    myFile.println("new data");
    // close the file:
    myFile.close();
  }
}
void readData() {
  Serial.println("read data");
  
  myFile = SD.open(filname);

  if (myFile) {
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  }
  
}
