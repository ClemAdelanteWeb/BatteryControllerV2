#include "AltSoftSerial.h"
#include "Thread.h"
#include "BlueSeaLatchingRelay.h"
#include "ADS1X15.h"
#include "SD.h"
#include "RTClib.h"
 
//------
// SETTINGS

const byte activatePrintStatus = 0;

// Checking cells differences
// var boolean 1 OR 0
const byte activateCheckingCellsVoltageDifference = 1;

// Opening charge relay for SOC >= SOCMax
const int SOCMax = 1000;

// Re-Close Charge Relay when SOCMaxReset is reached
const int SOCMaxReset = 950;

// Opening Load relay if SOC <= SOCmin
const int SOCMin = 200;

// Re-Close Load Relay when SOCMaxReset is reached
const int SOCMinReset = 250;

// SOC Maximum time considerated valid 
// in mS
const int SOCMaxTimeValid = 10000;

// Maximum Voltage security
const int BatteryVoltageMax = 13800;  // 13,8v = 3,45v / Cell

// Waiting for Max Reset Voltage after reaching Max Voltage (time to discharge the battery enough to use it)
const int BatteryVoltageMaxReset = 13360;  

// Minimum Voltage security
const int BatteryVoltageMin = 12400; // 12,4v = 3,1v / Cell

// Waiting for Min Reset Voltage after reaching Min Voltage (time to re-charge the battery enough to use it)
const int BatteryVoltageMinReset = 12800; // 12,9  = 3,225v / Cell

// Minimum operating cell voltage
const int CellVoltageMin = 310;

// Maximum operating cell voltage
const int CellVoltageMax = 360;

// Voltage difference between cells  or batteries
// Absolute value (-100mV) = 100mV).
// in mV
const int CellsDifferenceMaxLimit = 300;

// Voltage difference maximum to considere that the battery bank can be starting using again
// in mV
const int CellsDifferenceMaxReset = 150; 

//------
// PINS PARAMS
const byte LoadRelayClosePin = 4;
const byte LoadRelayOpenPin = 5;
const byte LoadRelayStatePin = A7;

const byte ChargeRelayClosePin = A1;
const byte ChargeRelayOpenPin = A2;
const byte ChargeRelayStatePin = A6;

const byte BuzzerPin = 7;

// if pin = 1, BMV infos are collected
const byte ActivateBmvSerialPin = A3;

const byte ThermistorSensorPin = A0;

const byte RS485Pin1 = 2;
const byte RS485Pin2 = 3;

const byte ChargingStatusOutputPin = 6;

// END PINS PARAMS
//------


// Number of cells in the battery
const int unsigned cellsNumber = 4;

// ADS1115 Calibration at 10v *1000
// Ex: 10 / 18107 = 0,000552273
// cell 1, 2, 3, 4 etc
float adc_calibration[cellsNumber] = {
  0.1780487,
  0.2963265,
  0.4323278,
  0.5925805
};


// LOGGING ON SD CARD
const int SDCardPinSelect = 10;

//------
// Temperature sensor settings

const int ThermistorNominal = 10000;     // // resistance at 25 degrees C
const int ThermistorTemperatureNominal = 25;   // temp. for nominal resistance (almost always 25 C)
const int ThermistorNbrSamples = 5; // how many samples to take and average, more takes longer
const int ThermistorBetaCoefficient = 3950; // The beta coefficient of the thermistor (usually 3000-4000)
const int ThermistorSerieResistor = 10000;    // the value of the 'other' resistor
const int TemperatureMinCharge = 1; // valeur en degré Celsus
const int TemperatureMinChargeReset = 2; // valeur à partir de laquelle on autorise à nouveau la charge de la batterie

// END SETTINGS
//-------

// Program declarations

// RX pin 8 Tx pin 9
AltSoftSerial Bmv; 

// SoftwareSerial Bmv(BmvRxPin,BmvTxPin); // RX, TX
Thread RunApplication = Thread();

// ADS1115 on I2C0x48 adress
ADS1115 ADS(0x48);

// Relays declaration
BlueSeaLatchingRelay LoadRelay;
BlueSeaLatchingRelay ChargeRelay;

unsigned int BatteryVoltage;
unsigned long BatteryVoltageUpdatedTime;

int CellsDifferenceMax;
unsigned long CellsDifferenceMaxUpdatedTime;

unsigned int SOC;
unsigned int SOCTemp;
unsigned int SOCCurrent;
unsigned long SOCUpdatedTime;

// If SOCDisSOCChargeCycling = true : the battery is full and Charge Relay is open 
bool SOCDisSOCChargeCycling = false;

// if SOCChargeCycling = true : the battery is empty and Load relay is opened
bool SOCChargeCycling = false;

// If High voltage has been detected (waiting for HighVoltageReset)
// Charge relay is closed
bool HighVoltageDetected = false;

// If Low voltage has been detected (waiting for LowVoltageReset)
// Load relay is closed
bool LowVoltageDetected = false;

// If a High voltage difference has been detected between cells
// waiting for a lower difference 
// Charge And Load relay are closed, Alarm is ON
bool CellsDifferenceDetected = false;

// If an individual cell voltage is too low
// waiting for a higher value
// force charging, alarm is ON
bool CellVoltageMinDetected = false;

// If an individual cell voltage is too high
// waiting for a lower value
// force discharging, alarm is ON
bool CellVoltageMaxDetected = false;


// If a low temperature is detected
// waiting for a higher value
// charging forbidden
bool LowBatteryTemperatureDetected = false;

// victron checksum 
byte checksum = 0;
String V_buffer;
char c;

// RTC date time module
RTC_DS1307 rtc;

// Temperature
float BatteryTemperature;

String MessageTemp;

int isfirstrun = 1;

void logData(String message, byte buzz, byte buzzperiode = 100);


void setup()
{
  pinMode(LoadRelayClosePin, OUTPUT);
  pinMode(LoadRelayOpenPin, OUTPUT);
  pinMode(ChargeRelayClosePin, OUTPUT);
  pinMode(ChargeRelayOpenPin, OUTPUT);

  pinMode(ChargeRelayStatePin, INPUT_PULLUP);
  pinMode(LoadRelayStatePin, INPUT_PULLUP);

  pinMode(ActivateBmvSerialPin, INPUT);
  pinMode(BuzzerPin, OUTPUT);

  // Pour la lecture de la température
  analogReference(EXTERNAL);
  
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
  
  Serial.begin(19200);
  Bmv.begin(19200); 

  RunApplication.onRun(run);
  RunApplication.setInterval(10000); // 10 sec

  ADS.begin();
  ADS.setGain(0);       // 6.144 volt
  ADS.setMode(1);       // mesures à la demande
  ADS.setDataRate(6);   // vitesse de mesure de 1 à 7
  ADS.readADC(0);       // Et on fait une lecture à vide, pour envoyer tous ces paramètres

  // Initialisation Carte SD
  if (!SD.begin(SDCardPinSelect)) {
    bip(50);
    delay(200);
    bip(5000);

  
    while (1);
  }


   // Lance le communication I2C avec le module RTC et 
  // attend que la connection soit operationelle
  while (! rtc.begin()) {
    bip(5000);
    delay(1000);
  }

  // Mise a jour de l'horloge du module RTC si elle n'a pas
  // ete reglee au prealable
  if (! rtc.isrunning()) {
    // La date et l'heure de la compilation de ce croquis 
    // est utilisee pour mettre a jour l'horloge
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}
 
void loop() {
  if(isfirstrun) {
    bip(50);
    delay(50);
    bip(50);
    
    logData(F("Booting"), 0);  
    isfirstrun = 0;             
  }
  
  if(isEnabledBMVSerialInfos()) {
    readBmvData();
  }

  if(RunApplication.shouldRun()) {
    RunApplication.run();
  }
  
}


/**
 * Checking Voltage AND SOC and close or open relays
 */
void run() {
  ChargeRelay.startCycle();
  LoadRelay.startCycle();

  
  // storing BatteryVoltage in temp variable
  int CurrentBatteryVoltage = getBatteryVoltage();

  // Get actual SOC
  SOCCurrent = getBatterySOC();

  // Get Actual battery  temperature
  BatteryTemperature = getBatteryTemperature();

  
  // --- Normal LOAD routines
  // checking if Load relay should be closed
  
  // first general condition
  // Normal operating range voltage and relay open
  if((CurrentBatteryVoltage > BatteryVoltageMin) && (LoadRelay.getState() != LoadRelay.RELAY_CLOSE)) {
    
      // not in special event
      if((SOCChargeCycling == false)  && (LowVoltageDetected == false)) {

          // if using BMV SOC
          if(isUseBMVSerialInfos()){
            SOCCurrent = getBatterySOC();
            
            if((SOCCurrent > SOCMin)) {
              LoadRelay.setReadyToClose();        
              logData(F("LR closing, routine"), 0);   
            }
            
          } 
          // Without SOC
          else {
            LoadRelay.setReadyToClose();        
            logData(F("LR closing, routine"), 0);   
          }          
      }   
  }  


  // ---- Normal CHARGE routines
  // checking if Charge relay must be closed

  // first general condition
  if((CurrentBatteryVoltage < BatteryVoltageMax) && (ChargeRelay.getState() != ChargeRelay.RELAY_CLOSE)) {
    
      // not in special event
      if((SOCDisSOCChargeCycling == false) && (HighVoltageDetected == false)) {

          // if using BMV SOC
          if(isUseBMVSerialInfos()){
            SOCCurrent = getBatterySOC();
            
            if((SOCCurrent < SOCMax)) {
              ChargeRelay.setReadyToClose();        
              logData(F("ChR closing, routine"), 0);   
            }
            
          } 
          // Without SOC
          else {
            ChargeRelay.setReadyToClose();        
            logData(F("ChR closing, routine"), 0);   
          }          
      }   
  }
  

   
  //---
  // Cancelling Charge Cycling
  // 
  if(SOCChargeCycling == true) {
    
    if(isUseBMVSerialInfos()) {
           
        // if SOC > SOCMinReset
        if(SOCCurrent >= SOCMinReset) {
          SOCChargeCycling = false;
          LoadRelay.setReadyToClose();  

          MessageTemp = F("SOC min RST reached : current/min : ");
          MessageTemp += (String)SOCCurrent+" % /"+(String)SOCMinReset;       
          logData(MessageTemp, 0);   
        }      
              
      } else {
          // Case IF Charge Cycling = true while it shouldn't
          // could append after disconnecting SOC check and SOCChargeCycling was ON.            
          // if Voltage battery high enough, we close the Load Relay
          
           if(LowVoltageDetected == false) {
            SOCChargeCycling = false;
            LoadRelay.setReadyToClose();
            logData(F("LR closing, routine without SOC"), 0);   
          } 
      }
  }



  //---
  // Cancelling DisCharge Cycling
  // 
  if(SOCDisSOCChargeCycling == true) {
      if(isUseBMVSerialInfos()) {      
          // if SOC < SOCMaxReset
          if(SOCCurrent <= SOCMaxReset) {
            SOCDisSOCChargeCycling = false;
            ChargeRelay.setReadyToClose();  

            MessageTemp = F("SOC max RST reached : current/max : ");
            MessageTemp += (String)SOCCurrent+" % /"+(String)SOCMaxReset;       
            logData(MessageTemp, 0);   
          }   
                   
      } else {
          // Case IF SOCDisSOCChargeCycling = true while it shouldn't
          // could append after disconnecting SOC check and SOCDisSOCChargeCycling was ON. 
          // if Voltage battery Low enough, we close the Charge Relay
          if(HighVoltageDetected == false) {
              SOCDisSOCChargeCycling = false;
              ChargeRelay.setReadyToClose();
              logData(F("ChR closing, routine without SOC"), 0);   
           }     
      }    
  }
   
   
 // END NORMAL ROUTINES
 // -------------------------

  // if Charge relay has been manualy closed and doesn't match with the code
  // Relay must be opened
 if((SOCDisSOCChargeCycling == true) || (HighVoltageDetected  == true)) {
    if(ChargeRelay.getState() != ChargeRelay.RELAY_OPEN) {
      ChargeRelay.setReadyToOpen();
      logData(F("ChR state doesn't match : open ChR"), 0);   
    }
 }

  // if Load relay has been manualy closed
  // Relay must be opened
  if((SOCChargeCycling == true) || (LowVoltageDetected  == true)) {
    if(LoadRelay.getState() != LoadRelay.RELAY_OPEN) {
      LoadRelay.setReadyToOpen();
      logData(F("LR state doesn't match : open LR"), 0);   
    }
 }



  // STARTING EXCEPTIONAL EVENTS
  if(isUseBMVSerialInfos()) {
  
      // SOC Max detection
      if((SOCCurrent >= SOCMax) && (SOCDisSOCChargeCycling == false)) {
        
        // Open Charge Relay
        SOCDisSOCChargeCycling = true;    
        ChargeRelay.setReadyToOpen();
        
        MessageTemp = F("SOC max reached : ");
        MessageTemp += (String) (SOCCurrent/10.0);
        MessageTemp += F(" %");  
        logData(MessageTemp, 0);     
        
      }
  
  
       // SOC Min detection
      if((SOCCurrent <= SOCMin) && (SOCChargeCycling == false)) {
        
        // Open Load Relay
        SOCChargeCycling = true;
    
        LoadRelay.setReadyToOpen();

        MessageTemp = F("SOC min reached : current/min : ");
        MessageTemp += (String)SOCCurrent+" % / "+(String)SOCMin;   
        logData(MessageTemp, 0);         
    } 
  } 
  


  //---
  // Voltage verifications
  if((millis() - BatteryVoltageUpdatedTime) < 6000) {

    // high voltage detection
     if((CurrentBatteryVoltage >= BatteryVoltageMax) && (HighVoltageDetected == false)) {
        // Open Charge Relay
        HighVoltageDetected = true;
        ChargeRelay.forceToOpen();
        MessageTemp = F("High V detected : ");
        MessageTemp += (String) (CurrentBatteryVoltage/1000.0)+" V";
        logData(MessageTemp, 0);
    } else {
      
      if(HighVoltageDetected == true) {

        if(ChargeRelay.getState() == ChargeRelay.RELAY_CLOSE) {       
          ChargeRelay.forceToOpen();  
          logData(F("High V Detected, ChR FOpen"), 0);   
        }

        // if Voltage battery low enough, we close the Charge Relay
        if(CurrentBatteryVoltage <= BatteryVoltageMaxReset) {          
          HighVoltageDetected = false;
          ChargeRelay.setReadyToClose();  
          logData(F("V Max Rst reached, ChR close"), 0);   
        }       
      }      
    }

    // Low voltage detection
    if((CurrentBatteryVoltage <= BatteryVoltageMin)  && (LowVoltageDetected == false)) {
       // Open LoadRelay
        LowVoltageDetected = true;
        LoadRelay.forceToOpen();
        logData(F("Low V Detected"), 0);   

        // Constrain battery to charge
        // in case of SOC >= max SOC, charge relay is open (can happen when very low consumption is not detected by Victron monitor)
        if(ChargeRelay.getState() == ChargeRelay.RELAY_OPEN) { 
            ChargeRelay.forceToClose();
            MessageTemp = F("ChR closing. SOC : ");
            MessageTemp += (String) getBatterySOC();
            MessageTemp += F(" . V : "); 
            MessageTemp += (String) CurrentBatteryVoltage;
            logData(MessageTemp, 0); 
        }

      
    } else {
      
      if(LowVoltageDetected == true) {

         if(LoadRelay.getState() == LoadRelay.RELAY_CLOSE) {       
          LoadRelay.forceToOpen();  
          logData(F("Low V Detected, LR FOpen"), 0);   
        }

         // Constrain battery to charge
        if(ChargeRelay.getState() == ChargeRelay.RELAY_OPEN) { 
            ChargeRelay.forceToClose();
            MessageTemp = F("ChR FClose. 2nd atempt");
            logData(MessageTemp, 0); 
        }

        
        // if Voltage battery high enough, we close the Load Relay
        if(CurrentBatteryVoltage >= BatteryVoltageMinReset) {
          LowVoltageDetected = false;
          LoadRelay.setReadyToClose();
          logData(F("V Min Rst reached. LR closing"), 0);   
        }       
      }      
    }
    
  } else {
    MessageTemp = F("V. upd > 6s (");
    MessageTemp += (String) (BatteryVoltageUpdatedTime/1000);
    MessageTemp += F(" ms)");   
    logData(MessageTemp, 1, 3000);

    LoadRelay.forceToOpen();
    logData(F("No V Detected LR FOpen"), 0);   
  }



  //---
  // Checking Cells / Batteries differences
  if(activateCheckingCellsVoltageDifference) {  
    CellsDifferenceMax = getMaxCellVoltageDifference();

    if((millis() - CellsDifferenceMaxUpdatedTime) < 10000) {
      
      // Too much voltage difference detected
       if((CellsDifferenceMax >= CellsDifferenceMaxLimit) && (CellsDifferenceDetected == false)) {
          // Open Load relay
          CellsDifferenceDetected = true;
          LoadRelay.forceToOpen();

          MessageTemp = F("Cells V >< too high");
          MessageTemp += (String) CellsDifferenceMax;
          MessageTemp += F(" Mv)");   
          logData(MessageTemp, 1, 5000);
      } else {
        
        if(CellsDifferenceDetected == true) {
  
          // if Votage battery low enough, we close the LoadRelay
          if(CellsDifferenceMax <= CellsDifferenceMaxReset) {          
            CellsDifferenceDetected = false;
            LoadRelay.setReadyToClose();  
            
            logData(F("Rst OK Cells V >< < Rst"), 0);
          }       
          
        }      
      }    
    } else {
      logData(F("Cells diff upd > 10s"), 1, 2500);
    }
  }

  // #########
   // checking for Individual cell voltage detection
    int i;
    int cellVoltage;
    for (i = (cellsNumber-1); i >= 0; i--) {
        if(i>0) {
           cellVoltage = getAdsCellVoltage(i) - getAdsCellVoltage((i-1));
        } else {
          cellVoltage = getAdsCellVoltage(i);
        }

        // HIGH individual voltage cell detected
        if((cellVoltage > CellVoltageMax) && (CellVoltageMaxDetected == false)) {
           // Open Charge Relay 
          CellVoltageMaxDetected = true;
          ChargeRelay.forceToOpen();
          
          // force discharging
          LoadRelay.forceToClose();
    
          MessageTemp = F("High individual cell V detected on ");
          MessageTemp += (String) (i);
          MessageTemp += F(" . V : )");
          MessageTemp += (String) cellVoltage;  
          logData(MessageTemp, 1, 2000);
          
        } else {    
          
          if(CellVoltageMaxDetected == true) {
      
            // if voltage cell low enough, we close the LoadRelay
            if(cellVoltage <= CellVoltageMax) {          
              CellVoltageMaxDetected = false; 
              logData(F("RST OK : high V cell low enough"), 0);
            }                   
          }      
        }


          // LOW individual voltage cell detected
        if((cellVoltage < CellVoltageMin) && (CellVoltageMinDetected == false)) {
           // Open load Relay, Close charge relay 
          CellVoltageMinDetected = true;
          LoadRelay.forceToOpen();

          // force charging
          ChargeRelay.forceToClose();
    
          MessageTemp = F("Low individual cell V detected ");
          MessageTemp += (String) (i);
          MessageTemp += F(" . V : )");
          MessageTemp += (String) cellVoltage;  
          logData(MessageTemp, 1, 2000);
          
        } else {    
          
          if(CellVoltageMinDetected == true) {
      
            // if voltage cell high enough
            if(cellVoltage >= CellVoltageMin) {          
              CellVoltageMinDetected = false;        
              logData(F("RST Cell V OK"), 0);
            }          
          }      
        }  
    }

   

  // Serial.print("Cells voltage max difference : \t");
  // Serial.println(CellsDifferenceMax);
  // Serial.println('--');


    // Checking Battery temperature
    if((BatteryTemperature <= TemperatureMinCharge) && (LowBatteryTemperatureDetected == false)) {
       // Open Charge Relay
       LowBatteryTemperatureDetected = true;
       ChargeRelay.forceToOpen();
       MessageTemp = F("Low T° detected : ");
       MessageTemp += (String) (BatteryTemperature)+" C";
       logData(MessageTemp, 0);
   } else {

     if(LowBatteryTemperatureDetected == true) {

       if(ChargeRelay.getState() == ChargeRelay.RELAY_CLOSE) {
         ChargeRelay.forceToOpen();
         logData(F("Low T° detected, ChR FOpen"), 0);
       }

       // if T° high enough, we close the Charge Relay
       if(BatteryTemperature >= TemperatureMinChargeReset) {
         LowBatteryTemperatureDetected = false;
         ChargeRelay.setReadyToClose();
         logData(F("T° min RST reached, ChR closing"), 0);
       }
     }
   }


  // applying actions on relays
  LoadRelay.applyReadyActions();
  ChargeRelay.applyReadyActions();
  
  if(activatePrintStatus) {
    printStatus();
  }

  if(ChargeRelay->state = BlueSeaLatchingRelay::RELAY_CLOSE) {
    /coco
    
  } else {
    
  }
}



/**
 * Get Cell voltage from ADS1115
 * 
 */
int getAdsCellVoltage(unsigned int cellNumber) {

    int16_t adc;
    
    // waiting for correct values
    int unsigned attempts = 0;
    do {
      adc = ADS.readADC(cellNumber);

      if(attempts > 0) {
        Serial.println("Attemp : "+(String)+attempts+" / "+(String)+attempts);
      }
      
      attempts++;

      if(attempts >= 50) {
          MessageTemp = F("Cell ");
          MessageTemp += (String) cellNumber;
          MessageTemp += F(" attempts to get V > 50)");   
          logData(MessageTemp, 1, 3000);
      }
      
    } while(adc <= 0 && attempts <= 50);

 // Serial.println("Cell "+(String)cellNumber+" : "+(String)adc+" : v "+(String) (adc * adc_calibration[cellNumber]));
 
  return adc * adc_calibration[cellNumber];
}

/**
 * Calculate max cell difference between all cells
 * Return value in mV
 */
float getMaxCellVoltageDifference() {
    float maxDiff = 0;

    // Cells voltages
    float cellsVoltage[(cellsNumber-1)];

    int i;
    int cellVoltage;
    for (i = (cellsNumber-1); i >= 0; i--) {
        if(i>0) {
           cellVoltage = getAdsCellVoltage(i) - getAdsCellVoltage((i-1));
        } else {
          cellVoltage = getAdsCellVoltage(i);
        }
       
        cellsVoltage[i] = cellVoltage;
    }

    CellsDifferenceMaxUpdatedTime = millis();

    return getDiffBtwMaxMin(cellsVoltage, (cellsNumber-1));
}


/**
 * Get Battery voltage
 * try with ADS1115 First, Victron BMV next
 */
int getBatteryVoltage() {
  int BatteryVoltageTemp = getAdsBatteryVoltage();
  
  if(BatteryVoltageTemp) {
    return BatteryVoltageTemp;
  }

  return 0;
}

/**
 * Return Battery Voltage (12v) via ADS 
 */
int getAdsBatteryVoltage() {
  int BatteryVoltageTemp = getAdsCellVoltage((cellsNumber-1));

  if(BatteryVoltageTemp > 100) {
      BatteryVoltageUpdatedTime = millis();
  } else {
    logData(F("ADS C3 V not available"),1, 5000);
  }
  
  return BatteryVoltageTemp;
}


/**
 * Return SOC value
 */
int getBatterySOC() {
  if((millis() - SOCUpdatedTime) < SOCMaxTimeValid) {
      return SOC;  
  } else {
      return 0;
  }
}


// Reading Victron Datas
// And extracting SOC and Voltage values
void readBmvData() {

  
  if (Bmv.available()) {
    c = Bmv.read();
    checksum += c;

    if (V_buffer.length() < 80) {
      V_buffer += c;
    }
      
    // end of line
    if (c == '\n') { 
    
      // Serial.println(V_buffer);
       if (V_buffer.startsWith("SOC")) {
        String temp_string = V_buffer.substring(V_buffer.indexOf("\t")+1);
        SOCTemp = temp_string.toInt();
       }
    
        // end of serie
      if (V_buffer.startsWith("Checksum")) {       
        
          byte result = checksum % 256;
    
          // checksum OK
         if(result == 0) {          
           SOC = SOCTemp;
           SOCUpdatedTime = millis();
         } else {
           // Checksum error
         }
    
          // begin new serie
         checksum = 0;
       }
    
      // begin new line
      V_buffer="";
    }     
  }
}


/**
 * Trigger Buzzer 
 */
void bip(int duration) {
    digitalWrite(BuzzerPin, HIGH); 
    delay(duration);    
    digitalWrite(BuzzerPin, LOW); 
}


/**
 * Return the value between Max and min values in an array
 */
float getDiffBtwMaxMin(float *values, int sizeOfArray) {
  float maxValue=0; 
  float minValue=0;
  float diffValue;
  
  for (byte k=0; k < sizeOfArray; k++) {
    if (values[k] > maxValue) {
       maxValue = values[k];  
    }
  }
  
  for (byte k=0; k < sizeOfArray; k++){
    if ((values[k] < minValue) || (minValue == 0)) {
       minValue = values[k];
    }
  }

  diffValue = (maxValue-minValue);
  
  return diffValue;
}

// Check if SOC Value is valid
boolean isSOCValid() {
   if((millis() - SOCUpdatedTime) < SOCMaxTimeValid) {
      return true;
   }

   return false;  
}

// Return battery temperature
// average of 5 samples
float getBatteryTemperature() {
  int samples[ThermistorNbrSamples];

  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i=0; i< ThermistorNbrSamples; i++) {
   samples[i] = analogRead(ThermistorSensorPin);
   delay(10);
  }
  
  // average all the samples out
  average = 0;
  for (i=0; i< ThermistorNbrSamples; i++) {
     average += samples[i];
  }
  average /= ThermistorNbrSamples;

  // convert the value to resistance
  average = 1023 / average - 1;
  average = ThermistorSerieResistor / average;

  float steinhart;
  steinhart = average / ThermistorNominal;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= ThermistorBetaCoefficient;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (ThermistorNominal + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;        

       return steinhart;
}

void logData(String message, byte buzz, byte buzzperiode = 100) {

  String messageDate = getDateTime() + F(" ") + message;

  // Open Datalog file on SD Card
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(messageDate);
    dataFile.close();
  }
  else {
    bip(50);
    delay(200);
    bip(5000);
  }

  if(buzz) {
    bip(buzzperiode);
    delay(buzzperiode);
    bip(buzzperiode);
    delay(buzzperiode);
    bip(buzzperiode);
    delay(buzzperiode);
  
    Serial.print(F("ALARME : "));
  } else {
    Serial.print(F("ALERTE : "));
  }


  Serial.println(messageDate);
  Serial.println(F("-"));
}

String getDateTime()
{
    DateTime now = rtc.now();
    char heure[10];
 
    // Affiche l'heure courante retournee par le module RTC
    // Note : le %02d permet d'afficher les chiffres sur 2 digits (01, 02, ....)
    sprintf(heure, "%4d\/%02d\/%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    return heure;
  
}

void printStatus() {
  Serial.println();
  //Serial.println(" ------- Status --- ");
   Serial.print(F("Detected V : Low / High  ")); Serial.print(LowVoltageDetected);Serial.print(F(" / ")); Serial.println(HighVoltageDetected);
  Serial.print(F("V : ")); Serial.println(getBatteryVoltage());
  Serial.print(F("V Max / Rst : ")); Serial.print(BatteryVoltageMax);  Serial.print(F(" / ")); Serial.println(BatteryVoltageMaxReset);
  Serial.print(F("V Min / Rst : ")); Serial.print(BatteryVoltageMin);Serial.print(F(" / ")); Serial.println(BatteryVoltageMinReset);
  Serial.print(F("ChR status : ")); Serial.println(ChargeRelay.getState());
  Serial.print(F("LR status : "));  Serial.println(LoadRelay.getState());
  Serial.print(F("SOC Enable / Used / value: ")); Serial.print(isEnabledBMVSerialInfos()); Serial.print(F(" / ")); Serial.print(isUseBMVSerialInfos()); Serial.print(F(" / ")); Serial.println(getBatterySOC());  
  Serial.print(F("SOC Max/ Max Rst : ")); Serial.print(SOCMax); Serial.print(F(" / ")); Serial.println(SOCMaxReset);    
  Serial.print(F("SOC Min/ Min Rst : ")); Serial.print(SOCMin); Serial.print(F(" / ")); Serial.println(SOCMinReset);  

  Serial.print(F("Cycling : Discharge / Charge ")); Serial.print(SOCDisSOCChargeCycling);Serial.print(F(" / ")); Serial.println(SOCChargeCycling);

  // Cells Voltage
  int i;
  int cellVoltage;
  for (i = (cellsNumber-1); i >= 0; i--) {
      if(i>0) {
         cellVoltage = getAdsCellVoltage(i) - getAdsCellVoltage((i-1));
      } else {
        cellVoltage = getAdsCellVoltage(i);
      }
     
    Serial.print(F("Cell : "));
    Serial.print(i);
    Serial.print(F(" / "));
      
    Serial.print((cellVoltage/1000.0),3);
    Serial.print(F(" :  "));
    Serial.println(getAdsCellVoltage(i));
  }

  Serial.print(F("Cells Diff/Max/Rst : "));  Serial.print(CellsDifferenceMax);  Serial.print(F(" mV / ")); Serial.print(CellsDifferenceMaxLimit);  Serial.print(F(" / ")); Serial.println(CellsDifferenceMaxReset);
  Serial.println();
    
}

/**
 * Detection if BMV Serial should be collected and used
 */
boolean isEnabledBMVSerialInfos() {

   // If PIn activated
   if(digitalRead(ActivateBmvSerialPin) == HIGH) {
      return true;
  }
    
  return false;  
}

/**
 * Check if BMV Infos can be used
 */
boolean isUseBMVSerialInfos() {

   // If PIn activated
   if(isEnabledBMVSerialInfos()) {

      // AND SOC valid
      if(isSOCValid()) {
        return true;
      } 
  }
    
  return false;  
}
