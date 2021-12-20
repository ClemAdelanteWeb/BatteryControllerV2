#include <Arduino.h>
#include <SPI.h>
#include "AltSoftSerial.h"

#include "SoftwareSerial.h"
#include "Thread.h"
#include "BlueSeaLatchingRelay.h"
#include "ADS1X15.h"

// Utilisation du module RTC qui permet de logger avec l'heure réelle
#define IS_RTC 1

#define IS_SDCARD 0

// Utilisation de serial
#define IS_SERIAL 1

#define IS_RS485 0

// Définit la date dans le module RTC
#define SET_DATE 0

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
const int SOCMin = 180;

// Re-Close Load Relay when SOCMinReset is reached
const int SOCMinReset = 200;

// SOC Maximum time considerated valid
// in mS
const int SOCMaxTimeValid = 10000;

// Maximum Voltage security
const int BatteryVoltageMax = 13800; // 13,8v = 3,45v / Cell

// Waiting for Max Reset Voltage after reaching Max Voltage (time to discharge the battery enough to use it)
const int BatteryVoltageMaxReset = 13360;

// Minimum Voltage security
const int BatteryVoltageMin = 12000; // 12V = 3v / Cell

// Waiting for Min Reset Voltage after reaching Min Voltage (time to re-charge the battery enough to use it)
const int BatteryVoltageMinReset = 12800; // 12,9  = 3,225v / Cell

// Minimum operating cell voltage
const int CellVoltageMin = 3100;

// Maximum operating cell voltage
const int CellVoltageMax = 3600;

// Voltage difference between cells  or batteries
// Absolute value (-100mV) = 100mV).
// in mV
// default 300
const int CellsDifferenceMaxLimit = 500;

// Voltage difference maximum to considere that the battery bank can be starting using again
// in mV
// default 250
const int CellsDifferenceMaxReset = 450;

// Delay time before opening the Charge Relay
// The charge Output status will be HIGH right away and the charge relay will be opened after the delay
// default 5 sec (5000)
const unsigned int delayBeforeChargeOpening = 5000;

//------
// PINS PARAMS
const byte LoadRelayClosePin = A1; // A1
const byte LoadRelayOpenPin = A2;  //A2
const byte LoadRelayStatePin = A7; //A7

const byte ChargeRelayClosePin = 4;  //4
const byte ChargeRelayOpenPin = 5;   //5
const byte ChargeRelayStatePin = A6; //A6

// if pin HIGH, BMV infos are collected
const byte ActivateBmvSerialPin = A3;

const byte ThermistorSensorPin = A0;

const byte RS485PinRx = 3;
const byte RS485PinTx = 2;
const byte RS485PinDE = 7;

const byte ChargingStatusOutputPin = 6;

// END PINS PARAMS
//------

// Number of cells in the battery
const byte cellsNumber = 4;

// ADS1115 Calibration at 10v *1000
// Ex: 10 / 18107 = 0,000552273
// cell 1, 2, 3, 4 etc
const float adc_calibration[cellsNumber] = {
    0.1780487,
    0.2142193,
    0.4980978,
    0.5896120};

// LOGGING Params
#if IS_RTC
#include <TimeLib.h>
#include <DS1307RTC.h>
// RTC date time module
tmElements_t tm;
#endif

#if SET_DATE
const char *monthName[12] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
#endif

// Log on SD Card

#if IS_SDCARD
#include <SdFat.h>
SdFat sd;
SdFile logFile;

uint8_t SDCardPinSelect = 10;
#define DataLogFile "log5.txt"
#endif

//------
// Temperature sensor settings

const int ThermistorNominal = 11700;          // // resistance at 25 degrees C
const byte ThermistorTemperatureNominal = 20; // temp. for nominal resistance (almost always 25 C)
const byte ThermistorNbrSamples = 5;          // how many samples to take and average, more takes longer
const int ThermistorBetaCoefficient = 4370;   // The beta coefficient of the thermistor (usually 3000-4000)
const int ThermistorSerieResistor = 10000;    // the value of the 'other' resistor

// Température minimum pour autorisation de charge
// => ne pas charger une batterie au lithium en dessous de 0°
// la sonde n'étant pas parfaitement calibrée, 1° réel = 4° sur la sonde
const byte TemperatureMinCharge = 4;
const byte TemperatureMinChargeReset = 5; // valeur à partir de laquelle on autorise à nouveau la charge de la batterie

// END SETTINGS
//-------

// TEXTES

// Program declarations

// RX pin 8 Tx pin 9 for Serial Victron BMV communication
AltSoftSerial Bmv;

#if IS_RS485
// RS485 communication
SoftwareSerial RS485(RS485PinRx, RS485PinTx);
#endif

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
char receivedChars[numChars]; // an array to store the received data
boolean newData = false;

// Temperature
int BatteryTemperature;

String MessageTemp;

byte isfirstrun = 1;

#define ValuesSpacer F("/")
//static const char ValuesSpacer[] = F("/");
int i;

void logData(String message, byte nivel = 0);
void logDataMessNum(int num, String values = "", byte nivel = 0);

String getDateTime();

#if IS_SERIAL
void printStatus();
#endif

void printRS485Status();
int getMaxCellVoltageDifference();
uint16_t getAdsBatteryVoltage();
void run();

#if SET_DATE
bool getTime(const char *str);
bool getDate(const char *str);
#endif

void setup()
{
  pinMode(LoadRelayClosePin, OUTPUT);
  pinMode(LoadRelayOpenPin, OUTPUT);
  pinMode(ChargeRelayClosePin, OUTPUT);
  pinMode(ChargeRelayOpenPin, OUTPUT);
  pinMode(ChargeRelayStatePin, INPUT_PULLUP);
  pinMode(LoadRelayStatePin, INPUT_PULLUP);

  pinMode(ActivateBmvSerialPin, INPUT);

  // Pour la lecture de la température
  analogReference(EXTERNAL);

  // Load Relay declaration
  // LoadRelay.name = F("LR");
  LoadRelay.openPin = LoadRelayOpenPin;
  LoadRelay.closePin = LoadRelayClosePin;
  LoadRelay.statePin = LoadRelayStatePin;

  // Charge Relay declaration
  // ChargeRelay.name = F("ChR");
  ChargeRelay.openPin = ChargeRelayOpenPin;
  ChargeRelay.closePin = ChargeRelayClosePin;
  ChargeRelay.statePin = ChargeRelayStatePin;
  ChargeRelay.delayBeforeOpening = delayBeforeChargeOpening;

// Serial.begin(19200); // fonctionnait sur la v2 à cette vitesse
#if IS_SERIAL
  Serial.begin(19200);
#endif

  Bmv.begin(19200); // Victron Baudrate = 19200

#if IS_RS485
  RS485.begin(38400);
  pinMode(RS485PinDE, OUTPUT);
  digitalWrite(RS485PinDE, LOW); // receiving mode
#endif

  RunApplication.onRun(run);
  RunApplication.setInterval(1000); // 10 sec

  ADS.begin();
  ADS.setGain(0);     // 4V volt
  ADS.setMode(1);     // mesures à la demande
  ADS.setDataRate(5); // vitesse de mesure de 1 à 7
  ADS.readADC(0);     // Et on fait une lecture à vide, pour envoyer tous ces paramètres

#if IS_SDCARD
  // Initialisation Carte SD
  if (!sd.begin(SDCardPinSelect))
  {
    // sd.initErrorHalt();
  }
#endif

#if SET_DATE
  bool parse = false;
  bool config = false;
  // get the date and time the compiler was run
  if (getDate(__DATE__) && getTime(__TIME__))
  {
    parse = true;
    // and configure the RTC with this info
    if (RTC.write(tm))
    {
      config = true;
    }
  }

#endif

#if IS_SERIAL
  Serial.println(F("STP END"));
#endif

  // Serial.println(F("END STP"));
}

// Enregistrement de toutes les tensions des cellules
void checkCellsVoltage()
{
  unsigned long averageCell;
  byte iTemp2;
  int iCell; // ne pas toucher le type
  uint16_t vTemp;

  for (iCell = (cellsNumber - 1); iCell >= 0; iCell--)
  {
    averageCell = 0;

    // take N samples in a row, with a slight delay
    for (iTemp2 = 0; iTemp2 < 5; iTemp2++)
    {
      averageCell += ADS.readADC(iCell);
      // Serial.println(averageCell);
      delay(10);
    }

    // moyenne des échantillons
    averageCell = averageCell / 5;

    vTemp = (int)(averageCell * adc_calibration[iCell]);

    if (vTemp < 0)
    {
      vTemp = 0;
    }
    BatteryCellsVoltage[iCell] = vTemp;
  }
}

/**
  Return cell Voltage
*/
uint16_t getAdsCellVoltage(int cellNumber)
{

  //  for (int element : BatteryCellsVoltage) // for each element in the array
  //    Serial.println(element);

  return BatteryCellsVoltage[cellNumber];
}

/**
   Get Battery voltage
   try with ADS1115 First, Victron BMV next
*/
uint16_t getBatteryVoltage()
{
  uint16_t BatteryVoltageTemp = getAdsBatteryVoltage();

  if (BatteryVoltageTemp)
  {
    return BatteryVoltageTemp;
  }

  return 0;
}

/**
   Return Battery Voltage (12v) for current loop
*/
uint16_t getAdsBatteryVoltage()
{
  uint16_t BatteryVoltageTemp2 = getAdsCellVoltage((cellsNumber - 1));

  if (BatteryVoltageTemp2 > 100)
  {
    BatteryVoltageUpdatedTime = millis();
  }
  else
  {
    // ADS 3 cell voltage not available
    logDataMessNum(14, "", 1);
  }

  return BatteryVoltageTemp2;
}

/**
   Return SOC value
*/
int getBatterySOC()
{
  if ((millis() - SOCUpdatedTime) < SOCMaxTimeValid)
  {
    return SOC;
  }
  else
  {
    return 0;
  }
}

// Reading Victron Datas
// And extracting SOC and Voltage values
void readBmvData()
{

  if (Bmv.available())
  {
    cBmv = Bmv.read();
    checksum += cBmv;

    if (V_buffer.length() < 80)
    {
      V_buffer += cBmv;
    }

    // end of line
    if (cBmv == '\n')
    {
      // Serial.println(V_buffer);
      if (V_buffer.startsWith(F("SOC")))
      {
        String temp_string = V_buffer.substring(V_buffer.indexOf("\t") + 1);
        SOCTemp = temp_string.toInt();
      }

      // end of serie
      if (V_buffer.startsWith(F("Checksum")))
      {

        byte result = checksum % 256;

        // checksum OK
        if (result == 0)
        {
          SOC = SOCTemp;
          SOCUpdatedTime = millis();
        }

        // begin new serie
        checksum = 0;
      }

      // begin new line
      V_buffer = "";
    }
  }
}

// Check if SOC Value is valid
boolean isSOCValid()
{
  if ((millis() - SOCUpdatedTime) < SOCMaxTimeValid)
  {
    return true;
  }

  return false;
}

#if IS_SDCARD
void readSDCard()
{
  int data;
  if (!logFile.open(DataLogFile, O_READ))
  {
    //  sd.errorHalt("opening test.txt for read failed");
  }

  while ((data = logFile.read()) >= 0)
  {
#if IS_SERIAL
    Serial.write(data);
#endif
  }

  // close the file:
  logFile.close();
}

void deleteFile()
{
  sd.remove(DataLogFile);
}
#endif

// Return battery temperature
// average of 5 samples
int getBatteryTemperature()
{

  byte iTemp3;
  //-----
  float averageTemperature = 0;

  // take N samples in a row, with a slight delay
  for (iTemp3 = 0; iTemp3 < ThermistorNbrSamples; iTemp3++)
  {
    averageTemperature += analogRead(ThermistorSensorPin);
    delay(10);
  }

  averageTemperature = averageTemperature / ThermistorNbrSamples;

  // convert the value to resistance
  averageTemperature = 1023.0 / averageTemperature - 1.0;
  averageTemperature = ThermistorSerieResistor / averageTemperature;

  // Serial.println(averageTemperature);
  // Serial.print("--");

  float steinhart;
  steinhart = averageTemperature / ThermistorNominal;         // (R/Ro)
  steinhart = log(steinhart);                                 // ln(R/Ro)
  steinhart /= ThermistorBetaCoefficient;                     // 1/B * ln(R/Ro)
  steinhart += 1.0 / (ThermistorTemperatureNominal + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                                // Invert
  steinhart -= 273.15;

  // Serial.println(steinhart);
  int degrees = (steinhart);
  return degrees;
}

void logDataMessNum(int num, String values, byte nivel)
{
  String MessageLog;
  MessageLog += F("#");
  MessageLog += (String)num;
  MessageLog += F(";");
  MessageLog += values;

  logData(MessageLog, nivel);
}

// Enregistre les messages sur la carte SD
void logData(String message, byte nivel)
{
#if IS_SDCARD
  String messageDate = getDateTime() + F(" ") + message;

  if (!logFile.open(DataLogFile, O_RDWR | O_CREAT | O_AT_END))
  {
    // sd.errorHalt("opening test.txt for write failed");
  }

  logFile.println(messageDate);
  logFile.close();
#endif

  // Serial.println(message);
  //  Serial.println(F("-"));
}

String getDateTime()
{
#if IS_RTC
  // Serial.println("getDateTime");
  char heure[10];
  if (RTC.read(tm))
  {

    // Affiche l'heure courante retournee par le module RTC
    // Note : le %02d permet d'afficher les chiffres sur 2 digits (01, 02, ....)
    //ex : "2021/12/08 18:12:20"
    sprintf(heure, "%4d\/%02d\/%02d %02d:%02d:%02d", tmYearToCalendar(tm.Year), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second);
  }
  // Serial.println(heure);
  return heure;
#else
  return "";
#endif
}

void printParams()
{
  // char outputParams[60];

  // // affichage des paramètres actuels
  // // VBat Max ; VBat Max reset; VBat Min; VBat Min Reset;; SOC Max; SOC Max Reset; SOC Min; SOC Min Reset ;; Cells diff Max, Cells diff Max Reset
  // sprintf(outputParams, "$%d;%d;%d;%d;;%d;%d;%d;%d;;%d;%d#",
  //         BatteryVoltageMax,
  //         BatteryVoltageMaxReset,
  //         BatteryVoltageMin,
  //         BatteryVoltageMinReset,
  //         SOCMax,
  //         SOCMaxReset,
  //         SOCMin,
  //         SOCMinReset,
  //         CellsDifferenceMaxLimit,
  //         CellsDifferenceMaxReset);

  // Serial.println(outputParams);
}

#if IS_RS485
void handleRs485()
{
  while (RS485.available() > 0)
  {
    char incomingCharacter = RS485.read();

    switch (incomingCharacter)
    {
    case '0':
      printRS485Status();
      break;
    case '1':
#if IS_SDCARD
      readSDCard();
#endif
      break;
    case '9':
#if IS_SDCARD
      deleteFile();
#endif
      break;
    }
  }
}
#endif

/**
   Detection if BMV Serial should be collected and used
*/
boolean isEnabledBMVSerialInfos()
{

  // If PIn activated
  if (digitalRead(ActivateBmvSerialPin) == HIGH)
  {
    return true;
  }

  return false;
}

/**
   Check if BMV Infos can be used
*/
boolean isUseBMVSerialInfos()
{

  // If PIn activated
  if (isEnabledBMVSerialInfos())
  {

    // AND SOC valid
    if (isSOCValid())
    {
      return true;
    }
  }

  return false;
}

/**
   Calculate max cell difference between all cells
   Return value in mV
*/
int getMaxCellVoltageDifference()
{

  // Cells voltages
  // unsigned int cellsVoltage[(cellsNumber - 1)];
  unsigned int minValue = 0;
  unsigned int maxValue = 0;
  unsigned int tempCellV = 0;
  int iTemp4; // ne pas toucher le type

  for (iTemp4 = (cellsNumber - 1); iTemp4 >= 0; iTemp4--)
  {
    if (iTemp4 > 0)
    {
      tempCellV = getAdsCellVoltage(iTemp4) - getAdsCellVoltage((iTemp4 - 1));
    }
    else
    {
      tempCellV = getAdsCellVoltage(iTemp4);
    }

    //    // enregistrement de la plus grande valeur
    if (tempCellV > maxValue)
    {
      maxValue = tempCellV;
    }

    if ((tempCellV < minValue) || (minValue == 0))
    {
      minValue = tempCellV;
    }
  }

  return maxValue - minValue;
}

#if IS_RS485
void printRS485Status()
{

  digitalWrite(RS485PinDE, HIGH);
  RS485.print(getBatteryVoltage());
  RS485.print(F(";"));
  RS485.print(getBatteryTemperature());
  RS485.print(F(";"));
  RS485.print(getBatterySOC());
  RS485.print(F("  "));
  // Cells
  RS485.print(getAdsCellVoltage(0));
  RS485.print(F(";"));
  RS485.print(getAdsCellVoltage(1));
  RS485.print(F(";"));
  RS485.print(getAdsCellVoltage(2));
  RS485.print(F(";"));
  RS485.print(getAdsCellVoltage(3));
  RS485.print(F(";"));
  RS485.print(getMaxCellVoltageDifference());
  RS485.print(F("  "));
  //Relais
  RS485.print(ChargeRelay.getState());
  RS485.print(F(";"));
  RS485.print(LoadRelay.getState());
  RS485.print(F("  "));

  //SOC Infos
  RS485.print(SOCChargeCycling);
  RS485.print(F(";"));
  RS485.print(SOCDischargeCycling);
  RS485.print(F(";"));
  RS485.print(isEnabledBMVSerialInfos());
  RS485.print(F(";"));
  RS485.print(isUseBMVSerialInfos());
  RS485.print(F("  "));
  //RS485.print(ChargeRelay.waitingForOpening);
  //RS485.print(F("  "));

  // HIGH / LOW voltage / LOW T°
  RS485.print(LowVoltageDetected);
  RS485.print(F(";"));
  RS485.print(HighVoltageDetected);
  RS485.print(F(";"));
  RS485.print(CellsDifferenceDetected);
  RS485.print(F(";"));
  RS485.print(CellVoltageMinDetected);
  RS485.print(F(";"));
  RS485.print(CellVoltageMaxDetected);
  RS485.print(F(";"));
  RS485.print((int)+LowBatteryTemperatureDetected);
  RS485.println();
  digitalWrite(RS485PinDE, LOW);
}
#endif

#if IS_SERIAL
void printStatus()
{

  //char output[36];

  // affichage des données actuelles
  //
  //  V Batt ; T° Batt ;;
  //  V Cell1 ; V Cell2 ; V Cell3 ; V Cell4 ; V Cell Max Diff ;;
  //  Ch Relay Status ; Load Relay Status ;;
  //  SOC Charge Cycling ; SOC Discharge Cycling;;
  //  Use SOC data ; Using SOC data ? ; SOC Value ;;
  //  Low Voltage detected ; High voltage detected
  //$%d;%d;%d;;%d;%d;%d;%d;%d;;%d;%d;;%d;%d;;%d;%d;;%d;%d#
  // sprintf(output, "$%d;%s;%d;;%d;%d;%d;%d;;%d;%d",
  //         getBatteryVoltage(),
  //         getBatteryTemperature(),
  //         getBatterySOC(),

  //         getAdsCellVoltage(0),
  //         getAdsCellVoltage(1),
  //         getAdsCellVoltage(2),
  //         getAdsCellVoltage(3),
  //         // getMaxCellVoltageDifference(),

  //         ChargeRelay.getState(),
  //         LoadRelay.getState()

  //        // SOCChargeCycling,
  //        // SOCDischargeCycling

  //         //  isEnabledBMVSerialInfos(),
  //         //  isUseBMVSerialInfos(),

  //         //  LowVoltageDetected,
  //         //  HighVoltageDetected
  // );
  // Serial.println(output);
  // int tempVal = getMaxVoltageTest();
  //   int  tempVal = getMaxCellVoltageDifference();

  // version 2

  Serial.print(getBatteryVoltage());
  Serial.print(F(";"));
  Serial.print(getBatteryTemperature());
  Serial.print(F(";"));
  Serial.print(getBatterySOC());
  Serial.print(F("  "));
  // Cells
  Serial.print(getAdsCellVoltage(0));
  Serial.print(F(";"));
  Serial.print(getAdsCellVoltage(1));
  Serial.print(F(";"));
  Serial.print(getAdsCellVoltage(2));
  Serial.print(F(";"));
  Serial.print(getAdsCellVoltage(3));
  Serial.print(F(";"));
  Serial.print(getMaxCellVoltageDifference());
  Serial.print(F("  "));
  //Relais
  Serial.print(ChargeRelay.getState());
  Serial.print(F(";"));
  Serial.print(LoadRelay.getState());
  Serial.print(F("  "));

  //SOC Infos
  Serial.print(SOCChargeCycling);
  Serial.print(F(";"));
  Serial.print(SOCDischargeCycling);
  Serial.print(F(";"));
  Serial.print(isEnabledBMVSerialInfos());
  Serial.print(F(";"));
  Serial.print(isUseBMVSerialInfos());
  Serial.print(F("  "));
  Serial.print(ChargeRelay.waitingForOpening);
  Serial.print(F("  "));

  // HIGH / LOW voltage / LOW T°
  Serial.print(LowVoltageDetected);
  Serial.print(F(";"));
  Serial.print(HighVoltageDetected);
  Serial.print(F(";"));
  Serial.print(CellsDifferenceDetected);
  Serial.print(F(";"));
  Serial.print(CellVoltageMinDetected);
  Serial.print(F(";"));
  Serial.print(CellVoltageMaxDetected);
  Serial.print(F(";"));
  Serial.print((int)+LowBatteryTemperatureDetected);
  Serial.println();
}
#endif

void loop()
{

  if (isfirstrun)
  {

    isfirstrun = 0;

    // première lecture des tensions
    checkCellsVoltage();

    // pour éviter d'avoir un SOC à 0 au lancement du programe
    SOC = 500;
    SOCUpdatedTime = millis();
    logData(F("Start"), 0);
  }

  // Collecte des infos Victron BMV702 si bouton ON
  if (isEnabledBMVSerialInfos())
  {
    readBmvData();
  }

#if IS_RS485
  // listening RS485 line
  handleRs485();
#endif

  if (RunApplication.shouldRun())
  {
    RunApplication.run();
  }
}

/**
   Checking Voltage AND SOC and close or open relays
*/
void run()
{

  // Ne pas retirer, remet les états à appliquer à 0
  ChargeRelay.startCycle();
  LoadRelay.startCycle();

  // Va rechercher les valeurs des cellules pour cette boucle
  checkCellsVoltage();

  // storing BatteryVoltage in temp variable
  int CurrentBatteryVoltage = getBatteryVoltage();

  // Get Actual battery  temperature
  BatteryTemperature = getBatteryTemperature();

  // Get actual SOC
  SOCCurrent = getBatterySOC();

  // --- Normal LOAD routines
  // checking if Load relay should be closed

  // first general condition
  // Normal operating range voltage and relay open
  if ((CurrentBatteryVoltage > BatteryVoltageMin) && (LoadRelay.getState() != LoadRelay.RELAY_CLOSE))
  {

    // not in special event
    if ((SOCChargeCycling == false) && (LowVoltageDetected == false))
    {

      // if using BMV SOC
      if (isUseBMVSerialInfos())
      {
        SOCCurrent = getBatterySOC();

        if ((SOCCurrent > SOCMin))
        {

          LoadRelay.setReadyToClose();

          // #15 SOC Current > SOC Min
          logDataMessNum(15);
        }
      }
      // Without SOC
      else
      {

        LoadRelay.setReadyToClose();

        // #16 Load relay closing, routine without SOC
        logDataMessNum(16);
      }
    }
  }

  // ---- Normal CHARGE routines
  // checking if Charge relay must be closed

  // first general condition
  if ((CurrentBatteryVoltage < BatteryVoltageMax) && (ChargeRelay.getState() != ChargeRelay.RELAY_CLOSE))
  {

    // not in special event
    if ((SOCDischargeCycling == false) && (HighVoltageDetected == false) && (LowBatteryTemperatureDetected == false))
    {

      // if using BMV SOC
      if (isUseBMVSerialInfos())
      {
        SOCCurrent = getBatterySOC();

        if ((SOCCurrent < SOCMax))
        {

          ChargeRelay.setReadyToClose();

          // #17 SOC Current < SOC Max Charge relay closing
          logDataMessNum(17);
        }
      }
      // Without SOC
      else
      {

        ChargeRelay.setReadyToClose();

        // #18 SOC Charge relay closing, routine without SOC
        logDataMessNum(18);
      }
    }
  }

  //---
  // Cancelling Charge Cycling
  //
  if (SOCChargeCycling == true)
  {

    if (isUseBMVSerialInfos())
    {

      // if SOC > SOCMinReset
      if (SOCCurrent >= SOCMinReset)
      {

        SOCChargeCycling = false;
        LoadRelay.setReadyToClose();

        // #19 SOC min reset reached
        MessageTemp = (String)SOCCurrent + ValuesSpacer + (String)SOCMinReset;
        logDataMessNum(19, MessageTemp);
      }
    }
    else
    {
      // Case IF Charge Cycling = true while it shouldn't
      // could append after disconnecting SOC check and SOCChargeCycling was ON.
      // if Voltage battery high enough, we close the Load Relay

      if (LowVoltageDetected == false)
      {

        SOCChargeCycling = false;
        LoadRelay.setReadyToClose();

        // #20 Loading relay closing without SOC
        logDataMessNum(20);
      }
    }
  }

  //---
  // Cancelling DisCharge Cycling
  //
  if (SOCDischargeCycling == true)
  {
    if (isUseBMVSerialInfos())
    {
      // if SOC < SOCMaxReset
      if (SOCCurrent <= SOCMaxReset)
      {
        SOCDischargeCycling = false;
        ChargeRelay.setReadyToClose();

        // #21 Cancelling Discharge Cycling
        MessageTemp = (String)SOCCurrent + ValuesSpacer + (String)SOCMaxReset;
        logDataMessNum(21, MessageTemp);
      }
    }
    else
    {
      // Case IF SOCDischargeCycling = true while it shouldn't
      // could append after disconnecting SOC check and SOCDischargeCycling was ON.
      // if Voltage battery Low enough, we close the Charge Relay
      if (HighVoltageDetected == false)
      {
        SOCDischargeCycling = false;
        ChargeRelay.setReadyToClose();

        // #22 Cancelling Discharge Cycling without SOC
        logDataMessNum(22);
      }
    }
  }

  // END NORMAL ROUTINES
  // -------------------------

  // if Charge relay has been manualy closed and doesn't match with the code
  // Relay must be opened
  if ((SOCDischargeCycling == true) || (HighVoltageDetected == true))
  {
    if (ChargeRelay.getState() != ChargeRelay.RELAY_OPEN)
    {

      ChargeRelay.setReadyToOpen();

      // #23 Wrong Charging Relay state, opening
      MessageTemp = (String)ChargeRelay.getState() + F("!=") + (String)ChargeRelay.RELAY_OPEN;
      logDataMessNum(23, MessageTemp);
    }
  }

  // if Load relay has been manualy closed
  // Relay must be opened
  if ((SOCChargeCycling == true) || (LowVoltageDetected == true))
  {
    if (LoadRelay.getState() != LoadRelay.RELAY_OPEN)
    {
      LoadRelay.setReadyToOpen();

      // #23 Wrong Loading Relay state, opening
      logDataMessNum(24);
    }
  }

  // STARTING EXCEPTIONAL EVENTS
  if (isUseBMVSerialInfos())
  {

    // SOC Max detection
    if ((SOCCurrent >= SOCMax) && (SOCDischargeCycling == false))
    {

      // Open Charge Relay
      SOCDischargeCycling = true;
      ChargeRelay.setReadyToOpen();

      // #25 SOC Current > Max : current/max"
      MessageTemp = (String)SOCCurrent + ValuesSpacer + (String)SOCMax;
      logDataMessNum(25, MessageTemp);
    }

    // SOC Min detection
    if ((SOCCurrent <= SOCMin) && (SOCChargeCycling == false))
    {

      // Open Load Relay
      SOCChargeCycling = true;

      LoadRelay.setReadyToOpen();
      MessageTemp = (String)SOCCurrent + ValuesSpacer + (String)SOCMin;
      logDataMessNum(1, MessageTemp, 0);
    }
  }

  //---
  // Voltage verifications
  if ((millis() - BatteryVoltageUpdatedTime) < 6000)
  {

    // high voltage detection
    if ((CurrentBatteryVoltage >= BatteryVoltageMax) && (HighVoltageDetected == false))
    {
      // Open Charge Relay
      HighVoltageDetected = true;

      ChargeRelay.forceToOpen();
      MessageTemp = (String)(CurrentBatteryVoltage);
      logDataMessNum(2, MessageTemp, 0);
    }
    else
    {

      if (HighVoltageDetected == true)
      {

        if (ChargeRelay.getState() == ChargeRelay.RELAY_CLOSE)
        {

          ChargeRelay.forceToOpen();
          logDataMessNum(3);
        }

        // if Voltage battery low enough, we close the Charge Relay
        if (CurrentBatteryVoltage <= BatteryVoltageMaxReset)
        {
          HighVoltageDetected = false;
          ChargeRelay.setReadyToClose();

          logDataMessNum(4);
        }
      }
    }

    // Low voltage detection
    if ((CurrentBatteryVoltage <= BatteryVoltageMin) && (LowVoltageDetected == false))
    {
      // Open LoadRelay
      LowVoltageDetected = true;

      LoadRelay.forceToOpen();
      logDataMessNum(5);

      // Constrain battery to charge
      // in case of SOC >= max SOC, charge relay is open (can happen when very low consumption is not detected by Victron monitor)
      if (ChargeRelay.getState() == ChargeRelay.RELAY_OPEN)
      {

        ChargeRelay.forceToClose();

        MessageTemp = (String)getBatterySOC();
        MessageTemp += ValuesSpacer;
        MessageTemp += (String)CurrentBatteryVoltage;
        logDataMessNum(6, MessageTemp, 0);
      }
    }
    else
    {

      if (LowVoltageDetected == true)
      {

        if (LoadRelay.getState() == LoadRelay.RELAY_CLOSE)
        {

          LoadRelay.forceToOpen();
          logDataMessNum(7);
        }

        // Constrain battery to charge
        if (ChargeRelay.getState() == ChargeRelay.RELAY_OPEN)
        {

          ChargeRelay.forceToClose();

          logDataMessNum(8);
        }

        // if Voltage battery high enough, we close the Load Relay
        if (CurrentBatteryVoltage >= BatteryVoltageMinReset)
        {
          LowVoltageDetected = false;

          LoadRelay.setReadyToClose();

          logDataMessNum(9);
        }
      }
    }
  }
  else
  {
    // Erreur !! Temps écoulé depuis dernière valeur valide

    LoadRelay.forceToOpen();

    MessageTemp = (String)(BatteryVoltageUpdatedTime / 1000);
    logDataMessNum(10, MessageTemp, 1);
  }

  //---
  // Checking Cells / Batteries differences
  if (activateCheckingCellsVoltageDifference)
  {
    CellsDifferenceMax = getMaxCellVoltageDifference();

    // Too much voltage difference detected
    if ((CellsDifferenceMax >= CellsDifferenceMaxLimit) && (CellsDifferenceDetected == false))
    {
      // Open Load relay
      CellsDifferenceDetected = true;

      LoadRelay.forceToOpen();
      MessageTemp = (String)CellsDifferenceMax;
      logDataMessNum(11, MessageTemp, 1);
    }
    else
    {

      if (CellsDifferenceDetected == true)
      {

        // if Votage battery low enough, we close the LoadRelay
        if (CellsDifferenceMax <= CellsDifferenceMaxReset)
        {
          CellsDifferenceDetected = false;

          LoadRelay.setReadyToClose();

          logDataMessNum(12);
        }
      }
    }
  }

  //
  // checking for Individual cell voltage detection
  int i; // ne pas toucher le type
  unsigned int cellVoltage;
  for (i = (cellsNumber - 1); i >= 0; i--)
  {
    if (i > 0)
    {
      cellVoltage = getAdsCellVoltage(i) - getAdsCellVoltage((i - 1));
    }
    else
    {
      cellVoltage = getAdsCellVoltage(i);
    }

    // HIGH individual voltage cell detected
    if ((cellVoltage > CellVoltageMax) && (CellVoltageMaxDetected == false))
    {
      // Open Charge Relay
      CellVoltageMaxDetected = true;
      ChargeRelay.forceToOpen();

      // force discharging
      LoadRelay.forceToClose();

      // #26 High individual cell voltage
      MessageTemp = (String)(i) + ValuesSpacer + (String)cellVoltage;
      logDataMessNum(26, MessageTemp, 1);
    }
    else
    {

      if (CellVoltageMaxDetected == true)
      {

        // if voltage cell low enough, we close the LoadRelay
        if (cellVoltage <= CellVoltageMax)
        {
          CellVoltageMaxDetected = false;

          // #27 RST OK : high V cell good
          MessageTemp = (String)(i) + ValuesSpacer + (String)cellVoltage;
          logDataMessNum(27, MessageTemp, 1);
        }
      }
    }

    // LOW individual voltage cell detected
    if ((cellVoltage < CellVoltageMin) && (CellVoltageMinDetected == false))
    {
      // Open load Relay, Close charge relay
      CellVoltageMinDetected = true;

      LoadRelay.forceToOpen();

      // force charging
      ChargeRelay.forceToClose();

      MessageTemp = (String)(i) + ValuesSpacer + (String)cellVoltage;
      logDataMessNum(28, MessageTemp, 1);
    }
    else
    {

      if (CellVoltageMinDetected == true)
      {

        // if voltage cell high enough
        if (cellVoltage >= CellVoltageMin)
        {
          CellVoltageMinDetected = false;

          // #29 RST Cell Voltage
          MessageTemp = (String)(i) + ValuesSpacer + (String)cellVoltage;
          logDataMessNum(29, MessageTemp, 0);
        }
      }
    }
  }

  // Checking Battery temperature
  if ((BatteryTemperature <= TemperatureMinCharge) && (LowBatteryTemperatureDetected == false))
  {
    // Open Charge Relay
    LowBatteryTemperatureDetected = true;
    ChargeRelay.forceToOpen();

    // #30 Low T°
    MessageTemp = (String)(BatteryTemperature);
    logDataMessNum(30, MessageTemp, 1);
  }
  else
  {

    if (LowBatteryTemperatureDetected == true)
    {

      if (ChargeRelay.getState() == ChargeRelay.RELAY_CLOSE)
      {

        ChargeRelay.forceToOpen();

        // #31 Low T°
        MessageTemp = (String)(BatteryTemperature);
        logDataMessNum(31, MessageTemp, 1);
      }

      // if T° high enough, we close the Charge Relay
      if (BatteryTemperature >= TemperatureMinChargeReset)
      {
        LowBatteryTemperatureDetected = false;
        ChargeRelay.setReadyToClose();

        // #32 T° min reset, closing charging relay
        MessageTemp = (String)(BatteryTemperature);
        logDataMessNum(32, MessageTemp, 0);
      }
    }
  }

  // applying actions on relays
  LoadRelay.applyReadyActions();
  ChargeRelay.applyReadyActions();

  // PUT Charging Status Output = LOW if charging relay is opened OR is waiting for being opened
  // Relay opened : getState() == 0
  if (ChargeRelay.waitingForOpening == true || ChargeRelay.getState() == 0)
  {
    digitalWrite(ChargingStatusOutputPin, 0);
  }
  else
  {
    digitalWrite(ChargingStatusOutputPin, 1);
  }

  // Send status on Serial line
#if IS_SERIAL
  printStatus();
#endif

  // Send status on RS485 bus
#if IS_RS485
  printRS485Status();
#endif
}

#if SET_DATE
bool getTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3)
    return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool getDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3)
    return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++)
  {
    if (strcmp(Month, monthName[monthIndex]) == 0)
      break;
  }
  if (monthIndex >= 12)
    return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}
#endif