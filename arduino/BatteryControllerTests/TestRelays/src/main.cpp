#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <BlueSeaLatchingRelay.h>

// N'afficher que l'état des relais
#define CHECK_STATUS_ONLY 1

const byte LoadRelayClosePin = A1; // 
const byte LoadRelayOpenPin = A2;  //
const byte LoadRelayStatePin = A7;

const byte ChargeRelayClosePin = 4; //
const byte ChargeRelayOpenPin = 5;  //
const byte ChargeRelayStatePin = A6;

const int DELAY_RELAY = 2000;

// Relays declaration
BlueSeaLatchingRelay LoadRelay;
BlueSeaLatchingRelay ChargeRelay;

void printStatus();

// the setup routine runs once when you press reset:
void setup()
{

  pinMode(LoadRelayClosePin, OUTPUT);
  pinMode(LoadRelayOpenPin, OUTPUT);
  pinMode(ChargeRelayClosePin, OUTPUT);
  pinMode(ChargeRelayOpenPin, OUTPUT);
  pinMode(ChargeRelayStatePin, INPUT_PULLUP);
  pinMode(LoadRelayStatePin, INPUT_PULLUP);

  // Load Relay declaration
  LoadRelay.name = "LR";
  LoadRelay.openPin = LoadRelayOpenPin;
  LoadRelay.closePin = LoadRelayClosePin;
  LoadRelay.statePin = LoadRelayStatePin;

  // Charge Relay declaration
  ChargeRelay.name = "ChR";
  ChargeRelay.openPin = ChargeRelayOpenPin;
  ChargeRelay.closePin = ChargeRelayClosePin;
  ChargeRelay.statePin = ChargeRelayStatePin;

  // initialize serial communication at 9600 bits per second:
  Serial.begin(19200);

  Serial.println("Setup ended");
}

// the loop routine runs over and over again forever:
void loop()
{

  // Affichage de l'état des relais
  Serial.println("starting loop");
  printStatus();

#if CHECK_STATUS_ONLY == 0

  // Close charging
  Serial.println("Charge relay Closing");
  ChargeRelay.setReadyToClose();

  // applying actions on relays
  LoadRelay.applyReadyActions();
  ChargeRelay.applyReadyActions();
  printStatus();
  delay(DELAY_RELAY);


  // // Close loading
  // Serial.println("Load relay closing");
  // LoadRelay.setReadyToClose();

  // // applying actions on relays
  // LoadRelay.applyReadyActions();
  // ChargeRelay.applyReadyActions();
  // printStatus();

  // delay(DELAY_RELAY);

  // open charging
  Serial.println("Charge relay opening");
  ChargeRelay.setReadyToOpen();

  // applying actions on relays
  LoadRelay.applyReadyActions();
  ChargeRelay.applyReadyActions();
  printStatus();

  delay(DELAY_RELAY);

  // // open loading
  // Serial.println("Load relay opening");
  // LoadRelay.setReadyToOpen();


  // // applying actions on relays
  // LoadRelay.applyReadyActions();
  // ChargeRelay.applyReadyActions();
  //   printStatus();

  // delay(DELAY_RELAY);

#else
  delay(3000);
#endif

  
}

void printStatus()
{
  char buffer[40];

  sprintf(buffer, "Charge relay status : %x ", ChargeRelay.getState());
  Serial.println(buffer);

  sprintf(buffer, "Load relay status : %x ", LoadRelay.getState());
  Serial.println(buffer);
   Serial.println("---");
}