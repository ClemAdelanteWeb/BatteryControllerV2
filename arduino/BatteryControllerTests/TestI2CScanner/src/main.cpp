#include  <Arduino.h>
// SDA = A4
// SCL = A5
#include <Wire.h>
void setup()
{
  Wire.begin();
  Serial.begin(19200);
  Serial.println("\nI2C Scanner");
}

void loop()
{
  byte error, address;
  int nDevices;
  Serial.println("Recherche en cours...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("Equiment I2C trouve a l'addresse 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Erreur inconnue a l'address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("Aucun Equipement I2C trouve\n");
  else
    Serial.println("Fini\n");

  delay(5000);           
}
