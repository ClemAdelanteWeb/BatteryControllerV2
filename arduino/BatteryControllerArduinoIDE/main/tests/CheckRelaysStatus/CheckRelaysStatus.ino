/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogReadSerial
*/

const byte ChargeRelayStatus = A6;
const byte LoadRelayStatus = A7;


// the setup routine runs once when you press reset:
void setup() {
 

  
  pinMode(ChargeRelayStatus, INPUT);
  pinMode(LoadRelayStatus, INPUT);
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(19200);
}

// the loop routine runs over and over again forever:
void loop() {
  

  // read the input on analog pin 0:

// Close charging
  Serial.print("Charge status : ");
  Serial.println(analogRead(ChargeRelayStatus));

    Serial.print("Load status : ");
  Serial.println(analogRead(LoadRelayStatus));
  delay(1000);
  
}
