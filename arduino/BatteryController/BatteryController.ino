#include "AltSoftSerial.h"
//#include "SoftwareSerial.h"
#include "Thread.h"
#include "BlueSeaLatchingRelay.h"
#include "ADS1X15.h"
// #include "SD.h"
#include "RTClib.h"
//------
// SETTINGS


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
const byte LoadRelayClosePin = 4; // 4
const byte LoadRelayOpenPin = 5; //5
const byte LoadRelayStatePin = A7;

const byte ChargeRelayClosePin = A1; //A1
const byte ChargeRelayOpenPin = A2; //A2
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
const int cellsNumber = 4;

// ADS1115 Calibration at 10v *1000
// Ex: 10 / 18107 = 0,000552273
// cell 1, 2, 3, 4 etc
const float adc_calibration[cellsNumber] = {
  0.1780487,
  0.2134769,
  0.4977307,
  0.5890515
};


// LOGGING ON SD CARD
const byte SDCardPinSelect = 10;

const String  DataLogFile = "log.txt";

//------
// Temperature sensor settings

const int ThermistorNominal = 11700;     // // resistance at 25 degrees C
const byte ThermistorTemperatureNominal = 20;   // temp. for nominal resistance (almost always 25 C)
const byte ThermistorNbrSamples = 5; // how many samples to take and average, more takes longer
const int ThermistorBetaCoefficient = 4370; // The beta coefficient of the thermistor (usually 3000-4000)
const int ThermistorSerieResistor = 10000;    // the value of the 'other' resistor
const byte TemperatureMinCharge = 1; // valeur en degré Celsus
const byte TemperatureMinChargeReset = 2; // valeur à partir de laquelle on autorise à nouveau la charge de la batterie

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


uint16_t BatteryCellsVoltage[cellsNumber];

uint16_t BatteryVoltage;
unsigned long BatteryVoltageUpdatedTime;

int CellsDifferenceMax;

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
char cBmv;

// Serial communication
const byte numChars = 2;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;

// RTC date time module
RTC_DS1307 rtc;

// Temperature
float BatteryTemperature;

String MessageTemp;

byte isfirstrun = 1;
const char TxtSpacer = " ";
int i;

void logData(String message, byte buzz, byte buzzperiode = 100);
void logDataMessNum(char num, String values = "", byte buzz = 0, byte buzzperiode = 100);

#define PRIu8 "hu"
#define PRId8 "hd"
#define PRIx8 "hx"
#define PRIu16 "hu"
#define PRId16 "hd"
#define PRIx16 "hx"
#define PRIu32 "u"
#define PRId32 "d"
#define PRIx32 "x"
#define PRIu64 "llu" // or possibly "lu"
#define PRId64 "lld" // or possibly "ld"
#define PRIx64 "llx" // or possibly "lx"


void setup()
{
  Serial.println(F("BEGIN STP"));

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
  //  if (!SD.begin(SDCardPinSelect)) {
  //     Serial.println(F("sdcard wait"));
  //    while (1);
  //  }

  if (! rtc.begin()) {
    //Serial.println(F("Couldn't find RTC"));
    //  Serial.flush();
    while (1) delay(10);
  }
  if (! rtc.isrunning()) {
    // Serial.println(F("RTC NOT running"));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }


  Serial.println(F("END STP"));

}

void loop() {

  if (isfirstrun) {
    isfirstrun = 0;
    // première lecture des tensions
    checkCellsVoltage();

    // logData(F("Booting"), 0);


  }

  readBmvData();

  if (isEnabledBMVSerialInfos()) {
    // readBmvData();
  }

  // Serial commandes buffering
  readSerialData();

  // check for new command
  checkCommands();

  //Serial.println(ADS.readADC(1));
  //Serial.println(ADS.readADC(2));
  //Serial.println(ADS.readADC(1));
  //Serial.println(ADS.readADC(1));
  //ChargeRelay.getState()
  //Serial.println("__");
  //

  //  if(RunApplication.shouldRun()) {
  //    RunApplication.run();
  //  }

}


/**
   Checking Voltage AND SOC and close or open relays
*/
void run() {

  Serial.println("run");
  ChargeRelay.startCycle();
  LoadRelay.startCycle();

  // checkCellsVoltage();


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
        //        logData(F("LR C/ w/ SOC"), 0);
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
      logData(F("ChR wrong state : O/"), 0);
    }
  }

  // if Load relay has been manualy closed
  // Relay must be opened
  if ((SOCChargeCycling == true) || (LowVoltageDetected  == true)) {
    if (LoadRelay.getState() != LoadRelay.RELAY_OPEN) {
      LoadRelay.setReadyToOpen();
      logData(F("LR wrong state : O/"), 0);
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
      MessageTemp = (String)SOCCurrent + F(",") + TxtSpacer + (String)SOCMin;
      logDataMessNum(1, MessageTemp, 0);
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
      MessageTemp = (String) (CurrentBatteryVoltage / 1000.0);
      logDataMessNum(2, MessageTemp, 0);
    } else {

      if (HighVoltageDetected == true) {

        if (ChargeRelay.getState() == ChargeRelay.RELAY_CLOSE) {
          ChargeRelay.forceToOpen();
          logDataMessNum(3, "", 0);
        }

        // if Voltage battery low enough, we close the Charge Relay
        if (CurrentBatteryVoltage <= BatteryVoltageMaxReset) {
          HighVoltageDetected = false;
          ChargeRelay.setReadyToClose();

          logDataMessNum(4, "", 0);
        }
      }
    }

    // Low voltage detection
    if ((CurrentBatteryVoltage <= BatteryVoltageMin)  && (LowVoltageDetected == false)) {
      // Open LoadRelay
      LowVoltageDetected = true;
      LoadRelay.forceToOpen();
      logDataMessNum(5, "", 0);

      // Constrain battery to charge
      // in case of SOC >= max SOC, charge relay is open (can happen when very low consumption is not detected by Victron monitor)
      if (ChargeRelay.getState() == ChargeRelay.RELAY_OPEN) {
        ChargeRelay.forceToClose();

        MessageTemp = (String) getBatterySOC();
        MessageTemp += F(",");
        MessageTemp += (String) CurrentBatteryVoltage;
        logDataMessNum(6, MessageTemp, 0);
      }


    } else {

      if (LowVoltageDetected == true) {

        if (LoadRelay.getState() == LoadRelay.RELAY_CLOSE) {
          LoadRelay.forceToOpen();
          logDataMessNum(7, "", 0);
        }

        // Constrain battery to charge
        if (ChargeRelay.getState() == ChargeRelay.RELAY_OPEN) {
          ChargeRelay.forceToClose();

          logDataMessNum(8, "", 0);
        }


        // if Voltage battery high enough, we close the Load Relay
        if (CurrentBatteryVoltage >= BatteryVoltageMinReset) {
          LowVoltageDetected = false;
          LoadRelay.setReadyToClose();

          logDataMessNum(9, "", 0);
        }
      }
    }

  } else {
    // Erreur !! Temps écoulé depuis dernière valeur valide
    LoadRelay.forceToOpen();

    MessageTemp = (String) (BatteryVoltageUpdatedTime / 1000);
    logDataMessNum(10, MessageTemp, 1, 3000);
  }



  //---
  // Checking Cells / Batteries differences
  if (activateCheckingCellsVoltageDifference) {
    CellsDifferenceMax = getMaxCellVoltageDifference();

    // Too much voltage difference detected
    if ((CellsDifferenceMax >= CellsDifferenceMaxLimit) && (CellsDifferenceDetected == false)) {
      // Open Load relay
      CellsDifferenceDetected = true;
      LoadRelay.forceToOpen();
      MessageTemp = (String) CellsDifferenceMax;
      logDataMessNum(11, MessageTemp , 1, 5000);
    } else {

      if (CellsDifferenceDetected == true) {

        // if Votage battery low enough, we close the LoadRelay
        if (CellsDifferenceMax <= CellsDifferenceMaxReset) {
          CellsDifferenceDetected = false;
          LoadRelay.setReadyToClose();

          logDataMessNum(12);
        }

      }
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


// Enregistrement de toutes les tensions des cellules
void checkCellsVoltage() {
  unsigned long averageCell;
  int iCell, iTemp2;
  uint16_t vTemp;

  for (iCell = (cellsNumber - 1); iCell >= 0; iCell--) {
    averageCell = 0;

    // take N samples in a row, with a slight delay
    for (iTemp2 = 0; iTemp2 < 5; iTemp2++) {
      averageCell += ADS.readADC(iCell);
      // Serial.println(averageCell);
      delay(10);
    }

    // moyenne des échantillons
    averageCell = averageCell / 5;

    vTemp = (int)  (averageCell * adc_calibration[iCell]);

    if (vTemp < 0) {
      vTemp = 0;
    }
    BatteryCellsVoltage[iCell] = vTemp;
  }
}


/**
  Return cell Voltage
*/
uint16_t getAdsCellVoltage(int cellNumber) {

  //  for (int element : BatteryCellsVoltage) // for each element in the array
  //    Serial.println(element);

  return BatteryCellsVoltage[cellNumber];

}




/**
   Get Battery voltage
   try with ADS1115 First, Victron BMV next
*/
uint16_t getBatteryVoltage() {
  uint16_t BatteryVoltageTemp = getAdsBatteryVoltage();

  if (BatteryVoltageTemp) {
    return BatteryVoltageTemp;
  }

  return 0;
}

/**
   Return Battery Voltage (12v) for current loop
*/
uint16_t getAdsBatteryVoltage() {
  uint16_t BatteryVoltageTemp2 = getAdsCellVoltage((cellsNumber - 1));

  if (BatteryVoltageTemp2 > 100) {
    BatteryVoltageUpdatedTime = millis();
  } else {
    logData(F("ADS A3 no dispo"), 1, 5000);
  }

  return BatteryVoltageTemp2;
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
    cBmv = Bmv.read();
    checksum += cBmv;

    if (V_buffer.length() < 80) {
      V_buffer += cBmv;
    }

    // end of line
    if (cBmv == '\n') {
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

          Serial.println(SOC);

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




// Check if SOC Value is valid
boolean isSOCValid() {
  if ((millis() - SOCUpdatedTime) < SOCMaxTimeValid) {
    return true;
  }

  return false;
}


void readSDCard() {
  //  File dataFile = SD.open(DataLogFile);
  //
  //  // if the file is available, write to it:
  //  if (dataFile) {
  //    while (dataFile.available()) {
  //      Serial.write(dataFile.read());
  //    }
  //
  //    dataFile.close();
  //  } else {
  //    // Serial.println("No SD card File");
  //  }
}

// Return battery temperature
// average of 5 samples
float getBatteryTemperature() {


  int samplesTemperature[ThermistorNbrSamples];
  int iTemp3;
  //-----
  float averageTemperature;

  // take N samples in a row, with a slight delay
  for (iTemp3 = 0; iTemp3 < ThermistorNbrSamples; iTemp3++) {
    averageTemperature += analogRead(ThermistorSensorPin);
    delay(10);
  }

  averageTemperature = averageTemperature / ThermistorNbrSamples;

  // convert the value to resistance
  averageTemperature = 1023 / averageTemperature - 1;
  averageTemperature = ThermistorSerieResistor / averageTemperature;

  float steinhart;
  steinhart = averageTemperature / ThermistorNominal;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= ThermistorBetaCoefficient;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (ThermistorTemperatureNominal + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert absolute temp to C
  //
  //    Serial.print("Temperature ");
  //    Serial.print(steinhart);
  //    Serial.println(" *C");

  return steinhart;
}


void logDataMessNum(char num, String values, byte buzz, byte buzzperiode = 100) {
  MessageTemp = F("#");
  MessageTemp += num;
  MessageTemp += (";");
  MessageTemp += values;
  logData(MessageTemp, buzz, buzzperiode);
}

// Enregistre les messages sur la carte SD
void logData(String message, byte buzz, byte buzzperiode = 100) {
  String messageDate = getDateTime() + F(" ") + message;

  // Open Datalog file on SD Card
  //  File dataFile = SD.open(DataLogFile, FILE_WRITE);
  //
  //  // if the file is available, write to it:
  //  if (dataFile) {
  //    dataFile.println(messageDate);
  //    dataFile.close();
  //  }
  //  else {
  //    // Serial.println("error opening "+DataLogFile);
  //    // bip(50);
  //    //delay(200);
  //    // bip(5000);
  //  }

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
  char heure[10];

  // Affiche l'heure courante retournee par le module RTC
  // Note : le %02d permet d'afficher les chiffres sur 2 digits (01, 02, ....)
  //ex : "2021/12/08 18:12:20"
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




// Receiving Serial commands
void readSerialData() {

  static byte ndx2 = 0;
  char endMarker2 = '\n';
  char rc2;

  while (Serial.available() > 0 && newData == false) {
    rc2 = Serial.read();

    if (rc2 != endMarker2) {
      receivedChars[ndx2] = rc2;
      ndx2++;
      if (ndx2 >= numChars) {
        ndx2 = numChars - 1;
      }
    }
    else {
      receivedChars[ndx2] = '\0'; // terminate the string
      ndx2 = 0;
      newData = true;
    }
  }


}


void checkCommands() {
  if (newData == true) {
    Serial.print("This just in ... ");
    Serial.println(receivedChars);

    if (strcmp(receivedChars, "l")  == 0)  {
      readSDCard();
    } else if (strcmp(receivedChars, "s")  == 0) {
      printStatus();
    } else if (strcmp(receivedChars, "t")  == 0) {
      Serial.println(getBatteryTemperature());
    } else if (strcmp(receivedChars, "p")  == 0) {
      printParams();
    } else if (strcmp(receivedChars, "a")  == 0) {
      checkCellsVoltage();
    }



    newData = false;
  }
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


/**
   Calculate max cell difference between all cells
   Return value in mV
*/
int getMaxCellVoltageDifference() {

  // Cells voltages
  int  cellsVoltage[(4 - 1)];
  int  minValue = 0;
  int  maxValue = 0;
  int iTemp4;

  for (iTemp4 = (4 - 1); iTemp4 >= 0; iTemp4--) {
    if (iTemp4 > 0) {
      cellsVoltage[iTemp4] = getAdsCellVoltage(iTemp4) - getAdsCellVoltage((iTemp4 - 1));
    } else {
      cellsVoltage[iTemp4] = getAdsCellVoltage(iTemp4);
    }
    //
    //    Serial.print("item ");
    //Serial.println( cellsVoltage[iTemp4]);
    //
    //
    //    // enregistrement de la plus grande valeur
    if (cellsVoltage[iTemp4] > maxValue) {
      maxValue = cellsVoltage[iTemp4];
    }
    //
    //
    if ((cellsVoltage[iTemp4] < minValue) || (minValue == 0)) {
      minValue = cellsVoltage[iTemp4];
    }
  }


  return maxValue - minValue;
}


int getMaxVoltageTest() {

  byte cellsNumber2 = 4;
  // Cells voltages
  int  cellsVoltage[(cellsNumber2 - 1)];
  int  minValue = 0;
  int  maxValue = 0;
  int iTemp4;

  for (iTemp4 = (cellsNumber2 - 1); iTemp4 >= 0; iTemp4--) {
   if (iTemp4 > 0) {
      cellsVoltage[iTemp4] = getAdsCellVoltage(iTemp4) - getAdsCellVoltage((iTemp4 - 1));
    } else {
      cellsVoltage[iTemp4] = getAdsCellVoltage(iTemp4);
    }
    //
        Serial.print("item ");
    Serial.println( cellsVoltage[iTemp4]);
    //
    //
    //    // enregistrement de la plus grande valeur
    if (cellsVoltage[iTemp4] > maxValue) {
      maxValue = cellsVoltage[iTemp4];
    }
    //
    //
    if ((cellsVoltage[iTemp4] < minValue) || (minValue == 0)) {
      minValue = cellsVoltage[iTemp4];
    }
  }
        Serial.print("max ");
    Serial.println( maxValue);
        Serial.print("min ");
    Serial.println( minValue);
  return maxValue - minValue;
}



void printStatus() {
  char output[60];


  char TempS[5]; dtostrf( (getBatteryTemperature()), 4, 2, TempS);
  uint16_t testAB = 550;


  // affichage des données actuelles
  //
  //  V Batt ; T° Batt ;;
  //  V Cell1 ; V Cell2 ; V Cell3 ; V Cell4 ; V Cell Max Diff ;;
  //  Ch Relay Status ; Load Relay Status ;;
  //  SOC Charge Cycling ; SOC Discharge Cycling;;
  //  Use SOC data ; Using SOC data ? ; SOC Value ;;
  //  Low Voltage detected ; High voltage detected
  //$%d;%s;%d;;%d;%d;%d;%d;%d;;%d;%d;;%d;%d;;%d;%d;;%d;%d#
  //  sprintf(output, "$%d;%s;%d;;%d;%d;%d;%d;%d",
  //          getBatteryVoltage(),
  //         TempS,
  //          getBatterySOC(),
  //
  //          getAdsCellVoltage(0),
  //          getAdsCellVoltage(1),
  //          getAdsCellVoltage(2),
  //          getAdsCellVoltage(3),
  //          getMaxCellVoltageDifference()
  //
  //          ChargeRelay.getState(),
  //          LoadRelay.getState(),
  //
  //          SOCChargeCycling,
  //          SOCDischargeCycling,
  //
  //          isEnabledBMVSerialInfos(),
  //          isUseBMVSerialInfos(),
  //
  //          LowVoltageDetected,
  //          HighVoltageDetected
  //   );
  // Serial.println(output);
  int  tempVal = getMaxVoltageTest();
 //   int  tempVal = getMaxCellVoltageDifference();
  
  Serial.print(getBatteryVoltage()); Serial.print(";");
  Serial.print(TempS); Serial.print(";");
  Serial.print(getBatterySOC()); Serial.print(";;");
  Serial.print(getAdsCellVoltage(0)); Serial.print(";");
  Serial.print(getAdsCellVoltage(1)); Serial.print(";");
  Serial.print(getAdsCellVoltage(2)); Serial.print(";");
  Serial.print(getAdsCellVoltage(3)); Serial.print(";");
  Serial.print(tempVal); Serial.print(";;");
  Serial.print(ChargeRelay.getState()); Serial.print(";");
  Serial.print(LoadRelay.getState()); Serial.print(";");

  Serial.println();

}
