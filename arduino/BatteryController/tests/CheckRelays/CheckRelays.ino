/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogReadSerial
*/

byte loadRelayClose = A1;
byte loadRelayOpen = A2;

byte chargeRelayClose = 4;
byte chargeRelayOpen = 5;

// the setup routine runs once when you press reset:
void setup() {
 

  
  pinMode(loadRelayClose, OUTPUT);
  pinMode(loadRelayOpen, OUTPUT);
  
  pinMode(chargeRelayClose, OUTPUT);
  pinMode(chargeRelayOpen, OUTPUT);

  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(19200);
}

// the loop routine runs over and over again forever:
void loop() {
  

  // read the input on analog pin 0:

// Close charging
  Serial.println("--- Charge relay close");
  Serial.println("Closing");
  digitalWrite(chargeRelayClose, HIGH);
  delay(1000);
  Serial.println(digitalRead(chargeRelayClose));
  Serial.println(digitalRead(chargeRelayOpen));
  digitalWrite(chargeRelayClose, LOW);
delay(1000);


  Serial.println("--- Charge relay open");
  Serial.println("opening");
  digitalWrite(chargeRelayOpen, HIGH);
  delay(1000);
  Serial.println(digitalRead(chargeRelayClose));
  Serial.println(digitalRead(chargeRelayOpen));
  digitalWrite(chargeRelayOpen, LOW);
delay(1000);



// Close loading
  Serial.println("--- Load relay close");
  Serial.println("Closing");
  digitalWrite(loadRelayClose, HIGH);
  delay(1000);
  Serial.println(digitalRead(loadRelayClose));
  Serial.println(digitalRead(loadRelayOpen));
  digitalWrite(loadRelayClose, LOW);
delay(1000);


  Serial.println("--- Load relay open");
  Serial.println("Opening");
  digitalWrite(loadRelayOpen, HIGH);
  delay(1000);
  Serial.println(digitalRead(loadRelayClose));
  Serial.println(digitalRead(loadRelayOpen));
  digitalWrite(loadRelayOpen, LOW);
delay(1000);


// Close loading
  Serial.println("--- Load relay close");
  Serial.println("Closing");
  digitalWrite(A2, HIGH);
  delay(1000);
  Serial.println(digitalRead(4));
  Serial.println(digitalRead(5));
  digitalWrite(4, LOW);
delay(1000);


  Serial.println("--- Load relay open");
  Serial.println("Closing");
  digitalWrite(5, HIGH);
  delay(1000);
  Serial.println(digitalRead(4));
  Serial.println(digitalRead(5));
  digitalWrite(5, LOW);
delay(1000);
}
