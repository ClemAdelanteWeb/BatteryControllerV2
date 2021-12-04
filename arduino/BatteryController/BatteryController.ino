#include "AltSoftSerial.h"
//#include "SoftwareSerial.h"
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

const byte RS485PinRx = 2;
const byte RS485PinTx = 3;

const byte ChargingStatusOutputPin = 6;

// END PINS PARAMS
//------


// Number of cells in the battery
const byte unsigned cellsNumber = 4;

// ADS1115 Calibration at 10v *1000
// Ex: 10 / 18107 = 0,000552273
// cell 1, 2, 3, 4 etc
const float adc_calibration[cellsNumber] = {
  0.1780487,
  0.2963265,
  0.4323278,
  0.5925805
};


// LOGGING ON SD CARD
const byte SDCardPinSelect = 10;

const String DataLogFile = "datalog.txt";

//------
// Temperature sensor settings

const int ThermistorNominal = 11700;     // // resistance at 25 degrees C
const int ThermistorTemperatureNominal = 20;   // temp. for nominal resistance (almost always 25 C)
const int ThermistorNbrSamples = 5; // how many samples to take and average, more takes longer
const int ThermistorBetaCoefficient = 4370; // The beta coefficient of the thermistor (usually 3000-4000)
const int ThermistorSerieResistor = 10000;    // the value of the 'other' resistor
const int TemperatureMinCharge = 1; // valeur en degré Celsus
const int TemperatureMinChargeReset = 2; // valeur à partir de laquelle on autorise à nouveau la charge de la batterie

// END SETTINGS
//-------

// TEXTES

// Program declarations

// RX pin 8 Tx pin 9 for Serial Victron BMV communication
AltSoftSerial Bmv;

// RS485 communication
// SoftwareSerial RS485(RS485PinRx, RS485PinTx); // RX, TX

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

// If SOCDischargeCycling = true : the battery is full and Charge Relay is open
bool SOCDischargeCycling = false;

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

// Serial communication
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;

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

  // Serial.begin(19200); // fonctionnait sur la v2 à cette vitesse
  Serial.begin(19200);
  Bmv.begin(19200);

  RunApplication.onRun(run);
  RunApplication.setInterval(1000); // 10 sec

  ADS.begin();
  ADS.setGain(0);       // 6.144 volt
  ADS.setMode(1);       // mesures à la demande
  ADS.setDataRate(6);   // vitesse de mesure de 1 à 7
  ADS.readADC(0);       // Et on fait une lecture à vide, pour envoyer tous ces paramètres

  // Initialisation Carte SD
  if (!SD.begin(SDCardPinSelect)) {
    // Serial.println(F("sdcard wait"));
    while (1);
  } 

  if (! rtc.begin()) {
    // Serial.println("Couldn't find RTC");
   //  Serial.flush();
    while (1) delay(10);
  }
  if (! rtc.isrunning()) {
    // Serial.println(F("RTC NOT running"));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

}

void loop() {

  if (isfirstrun) {
     logData(F("Booting"), 0);
    isfirstrun = 0;
  }


  if (isEnabledBMVSerialInfos()) {
    // readBmvData();
  }

  // Serial commandes buffering
  checkSerialCommands();

  // check for new command
  showNewData();


    if(RunApplication.shouldRun()) {
      RunApplication.run();
     }

}


// Receiving Serial commands
void checkSerialCommands() {

  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }


}


void showNewData() {
  if (newData == true) {
    //  Serial.print("This just in ... ");
    //  Serial.println(receivedChars);

    if (strcmp(receivedChars, "l")  == 0)  {
      readSDCard();
    } else if (strcmp(receivedChars, "s")  == 0) {
      printStatus();
    } else if (strcmp(receivedChars, "d")  == 0) {
      Serial.println(getDateTime());
    } else if (strcmp(receivedChars, "t")  == 0) {
      Serial.println(getBatteryTemperature());
    } else if (strcmp(receivedChars, "p")  == 0) {
      printParams();
    }



    newData = false;
  }
}

/**
   Checking Voltage AND SOC and close or open relays
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
  if ((CurrentBatteryVoltage > BatteryVoltageMin) && (LoadRelay.getState() != LoadRelay.RELAY_CLOSE)) {

    // not in special event
    if ((SOCChargeCycling == false)  && (LowVoltageDetected == false)) {

      // if using BMV SOC
      if (isUseBMVSerialInfos()) {
        SOCCurrent = getBatterySOC();

        if ((SOCCurrent > SOCMin)) {
          LoadRelay.setReadyToClose();
          //              logData(F("LR C/, routine"), 0);
        }

      }
      // Without SOC
      else {
        LoadRelay.setReadyToClose();
        //              logData(F("LR C/, routine"), 0);
      }
    }
  }


  // ---- Normal CHARGE routines
  // checking if Charge relay must be closed

  // first general condition
  if ((CurrentBatteryVoltage < BatteryVoltageMax) && (ChargeRelay.getState() != ChargeRelay.RELAY_CLOSE)) {

    // not in special event
    if ((SOCDischargeCycling == false) && (HighVoltageDetected == false)) {

      // if using BMV SOC
      if (isUseBMVSerialInfos()) {
        SOCCurrent = getBatterySOC();

        if ((SOCCurrent < SOCMax)) {
          ChargeRelay.setReadyToClose();
          //logData(F("ChR C/, routine"), 0);
        }

      }
      // Without SOC
      else {
        ChargeRelay.setReadyToClose();
        //logData(F("ChR C/, routine"), 0);
      }
    }
  }



  //---
  // Cancelling Charge Cycling
  //
  if (SOCChargeCycling == true) {

    if (isUseBMVSerialInfos()) {

      // if SOC > SOCMinReset
      if (SOCCurrent >= SOCMinReset) {
        SOCChargeCycling = false;
        LoadRelay.setReadyToClose();

        //          MessageTemp = F("SOC min RST reached : current/min : ");
        //          MessageTemp += (String)SOCCurrent+" % /"+(String)SOCMinReset;
        //          logData(MessageTemp, 0);
      }

    } else {
      // Case IF Charge Cycling = true while it shouldn't
      // could append after disconnecting SOC check and SOCChargeCycling was ON.
      // if Voltage battery high enough, we close the Load Relay

      if (LowVoltageDetected == false) {
        SOCChargeCycling = false;
        LoadRelay.setReadyToClose();
        //            logData(F("LR C/ w/ SOC"), 0);
      }
    }
  }



  //---
  // Cancelling DisCharge Cycling
  //
  if (SOCDischargeCycling == true) {
    if (isUseBMVSerialInfos()) {
      // if SOC < SOCMaxReset
      if (SOCCurrent <= SOCMaxReset) {
        SOCDischargeCycling = false;
        ChargeRelay.setReadyToClose();

        //            MessageTemp = F("SOC max RST reached : current/max : ");
        //            MessageTemp += (String)SOCCurrent+" % /"+(String)SOCMaxReset;
        //            logData(MessageTemp, 0);
      }

    } else {
      // Case IF SOCDischargeCycling = true while it shouldn't
      // could append after disconnecting SOC check and SOCDischargeCycling was ON.
      // if Voltage battery Low enough, we close the Charge Relay
      if (HighVoltageDetected == false) {
        SOCDischargeCycling = false;
        ChargeRelay.setReadyToClose();
        //              logData(F("ChR C/, routine w/ SOC"), 0);
      }
    }
  }


  // END NORMAL ROUTINES
  // -------------------------

  // if Charge relay has been manualy closed and doesn't match with the code
  // Relay must be opened
  if ((SOCDischargeCycling == true) || (HighVoltageDetected  == true)) {
    if (ChargeRelay.getState() != ChargeRelay.RELAY_OPEN) {
      ChargeRelay.setReadyToOpen();
      //      logData(F("ChR wrong state : O/"), 0);
    }
  }

  // if Load relay has been manualy closed
  // Relay must be opened
  if ((SOCChargeCycling == true) || (LowVoltageDetected  == true)) {
    if (LoadRelay.getState() != LoadRelay.RELAY_OPEN) {
      LoadRelay.setReadyToOpen();
      //      logData(F("LR wrong state : O/"), 0);
    }
  }



  // STARTING EXCEPTIONAL EVENTS
  if (isUseBMVSerialInfos()) {

    // SOC Max detection
    if ((SOCCurrent >= SOCMax) && (SOCDischargeCycling == false)) {

      // Open Charge Relay
      SOCDischargeCycling = true;
      ChargeRelay.setReadyToOpen();

      //        MessageTemp = F("SOC max : ");
      //        MessageTemp += (String) (SOCCurrent/10.0);
      //        MessageTemp += F(" %");
      //        logData(MessageTemp, 0);
    }


    // SOC Min detection
    if ((SOCCurrent <= SOCMin) && (SOCChargeCycling == false)) {

      // Open Load Relay
      SOCChargeCycling = true;

      LoadRelay.setReadyToOpen();

      //        MessageTemp = F("SOC min : now/min : ");
      //        MessageTemp += (String)SOCCurrent+" % / "+(String)SOCMin;
      //        logData(MessageTemp, 0);
    }
  }



  //---
  // Voltage verifications
  if ((millis() - BatteryVoltageUpdatedTime) < 6000) {

    // high voltage detection
    if ((CurrentBatteryVoltage >= BatteryVoltageMax) && (HighVoltageDetected == false)) {
      // Open Charge Relay
      HighVoltageDetected = true;
      ChargeRelay.forceToOpen();
      MessageTemp = F("High V : ");
      MessageTemp += (String) (CurrentBatteryVoltage / 1000.0) + " V";
      logData(MessageTemp, 0);
    } else {

      if (HighVoltageDetected == true) {

        if (ChargeRelay.getState() == ChargeRelay.RELAY_CLOSE) {
          ChargeRelay.forceToOpen();
          logData(F("High V, ChR FOpen"), 0);
        }

        // if Voltage battery low enough, we close the Charge Relay
        if (CurrentBatteryVoltage <= BatteryVoltageMaxReset) {
          HighVoltageDetected = false;
          ChargeRelay.setReadyToClose();
          logData(F("V Max Rst reached, ChR C/"), 0);
        }
      }
    }

    // Low voltage detection
    if ((CurrentBatteryVoltage <= BatteryVoltageMin)  && (LowVoltageDetected == false)) {
      // Open LoadRelay
      LowVoltageDetected = true;
      LoadRelay.forceToOpen();
      logData(F("Low V Detected"), 0);

      // Constrain battery to charge
      // in case of SOC >= max SOC, charge relay is open (can happen when very low consumption is not detected by Victron monitor)
      if (ChargeRelay.getState() == ChargeRelay.RELAY_OPEN) {
        ChargeRelay.forceToClose();
        MessageTemp = F("ChR C/. SOC : ");
        MessageTemp += (String) getBatterySOC();
        MessageTemp += F(" . V : ");
        MessageTemp += (String) CurrentBatteryVoltage;
        logData(MessageTemp, 0);
      }


    } else {

      if (LowVoltageDetected == true) {

        if (LoadRelay.getState() == LoadRelay.RELAY_CLOSE) {
          LoadRelay.forceToOpen();
          logData(F("Low V, LR FOpen"), 0);
        }

        // Constrain battery to charge
        if (ChargeRelay.getState() == ChargeRelay.RELAY_OPEN) {
          ChargeRelay.forceToClose();
          MessageTemp = F("ChR FClose. 2nd atempt");
          logData(MessageTemp, 0);
        }


        // if Voltage battery high enough, we close the Load Relay
        if (CurrentBatteryVoltage >= BatteryVoltageMinReset) {
          LowVoltageDetected = false;
          LoadRelay.setReadyToClose();
          logData(F("V Min Rst reached. LR closing"), 0);
        }
      }
    }

  } else {
    MessageTemp = F("V. upd > 6s (");
    MessageTemp += (String) (BatteryVoltageUpdatedTime / 1000);
    MessageTemp += F(" ms)");
    logData(MessageTemp, 1, 3000);

    LoadRelay.forceToOpen();
    logData(F("No V Detected LR FOpen"), 0);
  }



  //---
  // Checking Cells / Batteries differences
  if (activateCheckingCellsVoltageDifference) {
    CellsDifferenceMax = getMaxCellVoltageDifference();

    if ((millis() - CellsDifferenceMaxUpdatedTime) < 10000) {

      // Too much voltage difference detected
      if ((CellsDifferenceMax >= CellsDifferenceMaxLimit) && (CellsDifferenceDetected == false)) {
        // Open Load relay
        CellsDifferenceDetected = true;
        LoadRelay.forceToOpen();

        MessageTemp = F("Cells V diff high");
        MessageTemp += (String) CellsDifferenceMax;
        MessageTemp += F(" Mv)");
        logData(MessageTemp, 1, 5000);
      } else {

        if (CellsDifferenceDetected == true) {

          // if Votage battery low enough, we close the LoadRelay
          if (CellsDifferenceMax <= CellsDifferenceMaxReset) {
            CellsDifferenceDetected = false;
            LoadRelay.setReadyToClose();

            logData(F("Rst OK Cells V diff < Rst"), 0);
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
  for (i = (cellsNumber - 1); i >= 0; i--) {
    if (i > 0) {
      cellVoltage = getAdsCellVoltage(i) - getAdsCellVoltage((i - 1));
    } else {
      cellVoltage = getAdsCellVoltage(i);
    }

    // HIGH individual voltage cell detected
    if ((cellVoltage > CellVoltageMax) && (CellVoltageMaxDetected == false)) {
      // Open Charge Relay
      CellVoltageMaxDetected = true;
      ChargeRelay.forceToOpen();

      // force discharging
      LoadRelay.forceToClose();

      MessageTemp = F("High cell V on ");
      MessageTemp += (String) (i);
      MessageTemp += F(" . V : ");
      MessageTemp += (String) cellVoltage;
      logData(MessageTemp, 1, 2000);

    } else {

      if (CellVoltageMaxDetected == true) {

        // if voltage cell low enough, we close the LoadRelay
        if (cellVoltage <= CellVoltageMax) {
          CellVoltageMaxDetected = false;
          logData(F("RST OK : high V cell good"), 0);
        }
      }
    }


    // LOW individual voltage cell detected
    if ((cellVoltage < CellVoltageMin) && (CellVoltageMinDetected == false)) {
      // Open load Relay, Close charge relay
      CellVoltageMinDetected = true;
      LoadRelay.forceToOpen();

      // force charging
      ChargeRelay.forceToClose();

      MessageTemp = F("Low cell ");
      MessageTemp += (String) (i);
      MessageTemp += F(" . V : ");
      MessageTemp += (String) cellVoltage;
      logData(MessageTemp, 1, 2000);

    } else {

      if (CellVoltageMinDetected == true) {

        // if voltage cell high enough
        if (cellVoltage >= CellVoltageMin) {
          CellVoltageMinDetected = false;
          logData(F("RST Cell V"), 0);
        }
      }
    }
  }



  // Serial.print("Cells voltage max difference : \t");
  // Serial.println(CellsDifferenceMax);
  // Serial.println('--');


  // Checking Battery temperature
  if ((BatteryTemperature <= TemperatureMinCharge) && (LowBatteryTemperatureDetected == false)) {
    // Open Charge Relay
    LowBatteryTemperatureDetected = true;
    ChargeRelay.forceToOpen();
    MessageTemp = F("Low T° : ");
    MessageTemp += (String) (BatteryTemperature) + " C";
    logData(MessageTemp, 0);
  } else {

    if (LowBatteryTemperatureDetected == true) {

      if (ChargeRelay.getState() == ChargeRelay.RELAY_CLOSE) {
        ChargeRelay.forceToOpen();
        logData(F("Low T°, ChR FOpen"), 0);
      }

      // if T° high enough, we close the Charge Relay
      if (BatteryTemperature >= TemperatureMinChargeReset) {
        LowBatteryTemperatureDetected = false;
        ChargeRelay.setReadyToClose();
        logData(F("T° min RST, ChR C/"), 0);
      }
    }
  }


  // applying actions on relays
  LoadRelay.applyReadyActions();
  ChargeRelay.applyReadyActions();


  // PUT Charging Status Output = HIGH if charging relay is closed
  // Relay closed : getState() == 1
  if (ChargeRelay.getState() == 1) {
    digitalWrite(ChargingStatusOutputPin, 1);
  } else {
    digitalWrite(ChargingStatusOutputPin, 0);
  }
}



/**
   Get Cell voltage from ADS1115

*/
int getAdsCellVoltage(unsigned int cellNumber) {

  int16_t adc;
  int unsigned maxAttemps = 5;

  // waiting for correct values
  int unsigned attempts = 0;
  //do {
  adc = ADS.readADC(cellNumber);

  if (attempts > 0) {
     //Serial.println(F("Atp : "+(String)+attempts+" / "+(String)+attempts+" "));
  }

  attempts++;

  if (attempts >= maxAttemps) {
    MessageTemp = F("Cell ");
    MessageTemp += (String) cellNumber;
    MessageTemp += F(" atp to get V > ");
    MessageTemp += maxAttemps;
    logData(MessageTemp, 1, 3000);
  }

  //} while(adc <= 0 && attempts <= 5);

  // Serial.println("Cell "+(String)cellNumber+" : "+(String)adc+" : v "+(String) (adc * adc_calibration[cellNumber]));

   int unsigned vTemp = adc * adc_calibration[cellNumber];
   if(vTemp < 0) {
    return 0;
   }
   
  return vTemp;
}

/**
   Calculate max cell difference between all cells
   Return value in mV
*/
float getMaxCellVoltageDifference() {
  float maxDiff = 0;

  // Cells voltages
  float cellsVoltage[(cellsNumber - 1)];

  int i;
  int cellVoltage;
  for (i = (cellsNumber - 1); i >= 0; i--) {
    if (i > 0) {
      cellVoltage = getAdsCellVoltage(i) - getAdsCellVoltage((i - 1));
    } else {
      cellVoltage = getAdsCellVoltage(i);
    }

    cellsVoltage[i] = cellVoltage;
  }

  CellsDifferenceMaxUpdatedTime = millis();

  return getDiffBtwMaxMin(cellsVoltage, (cellsNumber - 1));
}


/**
   Get Battery voltage
   try with ADS1115 First, Victron BMV next
*/
int getBatteryVoltage() {
  int BatteryVoltageTemp = getAdsBatteryVoltage();

  if (BatteryVoltageTemp) {
    return BatteryVoltageTemp;
  }

  return 0;
}

/**
   Return Battery Voltage (12v) via ADS
*/
int getAdsBatteryVoltage() {
  int BatteryVoltageTemp = getAdsCellVoltage((cellsNumber - 1));
  
  if (BatteryVoltageTemp > 100) {
    BatteryVoltageUpdatedTime = millis();
  } else {
     logData(F("ADS A3 no dispo"), 1, 5000);
  }

  return BatteryVoltageTemp;
}


/**
   Return SOC value
*/
int getBatterySOC() {
  if ((millis() - SOCUpdatedTime) < SOCMaxTimeValid) {
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
        String temp_string = V_buffer.substring(V_buffer.indexOf("\t") + 1);
        SOCTemp = temp_string.toInt();
      }

      // end of serie
      if (V_buffer.startsWith("Checksum")) {

        byte result = checksum % 256;

        // checksum OK
        if (result == 0) {
          SOC = SOCTemp;
          SOCUpdatedTime = millis();
        } else {
          // Checksum error
        }

        // begin new serie
        checksum = 0;
      }

      // begin new line
      V_buffer = "";
    }
  }
}


/**
   Trigger Buzzer
*/
void bip(int duration) {
  digitalWrite(BuzzerPin, HIGH);
  delay(duration);
  digitalWrite(BuzzerPin, LOW);
}


/**
   Return the value between Max and min values in an array
*/
float getDiffBtwMaxMin(float *values, int sizeOfArray) {
  float maxValue = 0;
  float minValue = 0;
  float diffValue;

  for (byte k = 0; k < sizeOfArray; k++) {
    if (values[k] > maxValue) {
      maxValue = values[k];
    }
  }

  for (byte k = 0; k < sizeOfArray; k++) {
    if ((values[k] < minValue) || (minValue == 0)) {
      minValue = values[k];
    }
  }

  diffValue = (maxValue - minValue);

  return diffValue;
}

// Check if SOC Value is valid
boolean isSOCValid() {
  if ((millis() - SOCUpdatedTime) < SOCMaxTimeValid) {
    return true;
  }

  return false;
}


void readSDCard() {
  File dataFile = SD.open(DataLogFile);

  // if the file is available, write to it:
  if (dataFile) {
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }

    dataFile.close();
  } else {
    // Serial.println("No SD card File");
  }
}

// Return battery temperature
// average of 5 samples
float getBatteryTemperature() {


  int samples[ThermistorNbrSamples];
  //-----
  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i = 0; i < ThermistorNbrSamples; i++) {
    samples[i] = analogRead(ThermistorSensorPin);
    delay(10);
  }

  // average all the samples out
  average = 0;
  for (i = 0; i < ThermistorNbrSamples; i++) {
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
  steinhart += 1.0 / (ThermistorTemperatureNominal + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert absolute temp to C
  //
  //  Serial.print("Temperature ");
  //  Serial.print(steinhart);
  //  Serial.println(" *C");

  return steinhart;
}


// Enregistre les messages sur la carte SD
void logData(String message, byte buzz, byte buzzperiode = 100) {
  String messageDate = getDateTime() + F(" ") + message;

  // Open Datalog file on SD Card
  File dataFile = SD.open(DataLogFile, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(messageDate);
    dataFile.close();
  }
  else {
    // Serial.println("error opening "+DataLogFile);
    // bip(50);
    //delay(200);
    // bip(5000);
  }

  if (buzz == 3) {
    //    bip(buzzperiode);
    //    delay(buzzperiode);
    //    bip(buzzperiode);
    //    delay(buzzperiode);
    //    bip(buzzperiode);
    //    delay(buzzperiode);
    //
    //    Serial.print(F("ALARME : "));
  } else {
    //    Serial.print(F("ALERTE : "));
  }


    Serial.println(messageDate);
  //  Serial.println(F("-"));
}

String getDateTime()
{
  // Serial.println("getDateTime");
  DateTime now = rtc.now();
  char heure[50];

  // Affiche l'heure courante retournee par le module RTC
  // Note : le %02d permet d'afficher les chiffres sur 2 digits (01, 02, ....)
  sprintf(heure, "%4d\/%02d\/%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  
 // Serial.println(heure);
  return heure;

}

void printParams() {
  char outputParams[60];  

    // affichage des paramètres actuels
  // VBat Max ; VBat Max reset; VBat Min; VBat Min Reset;; SOC Max; SOC Max Reset; SOC Min; SOC Min Reset ;; Cells diff Max, Cells diff Max Reset
  sprintf(outputParams, "$%d;%d;%d;%d;;%d;%d;%d;%d;;%d;%d#",
          BatteryVoltageMax,
          BatteryVoltageMaxReset,
          BatteryVoltageMin,
          BatteryVoltageMinReset,
          SOCMax,
          SOCMaxReset,
          SOCMin,
          SOCMinReset,
          CellsDifferenceMaxLimit,
          CellsDifferenceMaxReset
         );

  Serial.println(outputParams);
}

void printStatus() {
  char output[60];

  // Cells Voltage
//  int i;
//  int cellVoltage;
//  for (i = (cellsNumber - 1); i >= 0; i--) {
//    if (i > 0) {
//      cellVoltage = getAdsCellVoltage(i) - getAdsCellVoltage((i - 1));
//    } else {
//      cellVoltage = getAdsCellVoltage(i);
//    }

    // Serial.print(F("Cell : "));
    // Serial.print(i);
    // Serial.print(F(" / "));

    // Serial.print((cellVoltage/1000.0),3);
    // Serial.print(F(" :  "));
    // Serial.println(getAdsCellVoltage(i));
//  }

    char VBattS[6]; dtostrf( (getBatteryVoltage() / 1000.0), 5, 3, VBattS);
  char VCell1S[6]; dtostrf( (getAdsCellVoltage(0) / 1000.0), 5, 3, VCell1S);
  char VCell2S[6]; dtostrf( (getAdsCellVoltage(1) / 1000.0), 5, 3, VCell2S);
  char VCell3S[6]; dtostrf( (getAdsCellVoltage(2) / 1000.0), 5, 3, VCell3S);
  char VCell4S[6]; dtostrf( (getAdsCellVoltage(3) / 1000.0), 5, 3, VCell4S);
  char TempS[5]; dtostrf( (getBatteryTemperature()), 4, 2, TempS);
  char MaxCellDiff[5]; dtostrf(CellsDifferenceMax, 5, 3, MaxCellDiff);

//  Serial.println(VBattS);
//Serial.println(VCell1S);
//Serial.println(VCell2S);
//Serial.println(VCell3S);
//Serial.println(VCell4S);
//Serial.println(TempS);
  // affichage des données actuelles
  // V Batt ; T° Batt ;; V Cell1 ; V Cell2 ; V Cell3 ; V Cell4 ; V Cell Max Diff ;; Ch Relay Status ; Load Relay Status ;; SOC Charge Cycling ; SOC Discharge Cycling;; Use SOC data ; Using SOC data ? ; SOC Value ;; Low Voltage detected ; High voltage detected
  sprintf(output, "$%s;%s;;%s;%s;%s;%s;;%d;%d;;%d;%d;;%d;%d;%d;;%d;%d#",
          VBattS,
          TempS,
          VCell1S,
          VCell2S,
          VCell3S,
          VCell4S,
          MaxCellDiff,
          ChargeRelay.getState(),
          LoadRelay.getState(),
          SOCChargeCycling,
          SOCDischargeCycling,
          isEnabledBMVSerialInfos(),
          isUseBMVSerialInfos(),
          getBatterySOC(),
          LowVoltageDetected,
          HighVoltageDetected
         );

  Serial.println(output);
}

/**
   Detection if BMV Serial should be collected and used
*/
boolean isEnabledBMVSerialInfos() {

  // If PIn activated
  if (digitalRead(ActivateBmvSerialPin) == HIGH) {
    return true;
  }

  return false;
}

/**
   Check if BMV Infos can be used
*/
boolean isUseBMVSerialInfos() {

  // If PIn activated
  if (isEnabledBMVSerialInfos()) {

    // AND SOC valid
    if (isSOCValid()) {
      return true;
    }
  }

  return false;
}
