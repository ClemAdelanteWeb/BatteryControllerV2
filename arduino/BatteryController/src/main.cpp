#include <Arduino.h>
#include <SPI.h>
#include "AltSoftSerial.h"

#include "SoftwareSerial.h"
#include "Thread.h"
#include "BlueSeaLatchingRelay.h"
#include "ADS1X15.h"

// Utilisation du module RTC qui permet de logger avec l'heure réelle
#define IS_RTC 1

#define IS_SDCARD 1

// Utilisation de serial
#define IS_SERIAL 0

#define IS_RS485 1

// Définit la date dans le module RTC
#define SET_DATE 0

// print memory free
#define IS_PRINT_MEMORY 1

//------
// SETTINGS

// Opening charge relay for SOC >= SOCMax
const int SOCMax = 1000;

// Re-Close Charge Relay when SOCMaxReset is reached
const int SOCMaxReset = 950;

// Opening Load relay if SOC <= SOCmin
const int SOCMin = 180;

// Re-Close Load Relay when SOCMinReset is reached
const int SOCMinReset = 200;

// SOC Maximum time considerated valid
// must be > 10 000ms
// in mS
const int SOCMaxTimeValid = 30000;

// Maximum Voltage security
// default 13800
const int BatteryVoltageMax = 13800; // 13,8v = 3,45v / Cell

// Waiting for Max Reset Voltage after reaching Max Voltage (time to discharge the battery enough to use it)
// default 13360
const int BatteryVoltageMaxReset = 13200;

// Minimum Voltage security
// default 12000
const int BatteryVoltageMin = 12000; // 12V = 3v / Cell

// Waiting for Min Reset Voltage after reaching Min Voltage (time to re-charge the battery enough to use it)
// default 12800
const int BatteryVoltageMinReset = 12800; // 12,9  = 3,225v / Cell

// Minimum operating cell voltage
// default 2800
const int CellVoltageMin = 2800;

// Minimum cell voltage after a min detected
const int CellVoltageMinReset = 2900;

// Maximum operating cell voltage
// default 3600
const int CellVoltageMax = 3600;

// Maximum cell voltage after max dected
const int CellVoltageMaxReset = 3350;

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
    0.11947112,
    0.22990510,
    0.32871799,
    0.422045};

// number of sample taken to average cell voltage
// mode=4 so 128 samples per seconde = 8ms/sample
// delay of 2ms between samples so
#define ADS_SAMPLE_NBR 10

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

#if IS_PRINT_MEMORY
#include <MemoryFree.h>
#endif
// Log on SD Card

#if IS_SDCARD
#include <SdFat.h>
SdFat sd;
SdFile logFile;

uint8_t SDCardPinSelect = 10;
#define DataLogFile "log.txt"
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

uint16_t LowestCellVoltage;
uint16_t HighestCellVoltage;

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
byte checksumBmv = 0;
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

void printParams();

#if IS_PRINT_MEMORY
void printMemory();
#endif

void printStatus();

// int getMaxCellVoltageDifference();
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
#if IS_SERIAL
  LoadRelay.name = "LR";
#endif

  LoadRelay.openPin = LoadRelayOpenPin;
  LoadRelay.closePin = LoadRelayClosePin;
  LoadRelay.statePin = LoadRelayStatePin;

// Charge Relay declaration
#if IS_SERIAL
  ChargeRelay.name = "ChR";
#endif
  ChargeRelay.openPin = ChargeRelayOpenPin;
  ChargeRelay.closePin = ChargeRelayClosePin;
  ChargeRelay.statePin = ChargeRelayStatePin;
  ChargeRelay.delayBeforeOpening = delayBeforeChargeOpening;

// Serial.begin(19200); // fonctionnait sur la v2 à cette vitesse
#if IS_SERIAL
  Serial.begin(38400);
#endif

  Bmv.begin(19200); // Victron Baudrate = 19200

#if IS_RS485
  RS485.begin(57600);
  pinMode(RS485PinDE, OUTPUT);
  digitalWrite(RS485PinDE, LOW); // receiving mode
#endif

  RunApplication.onRun(run);
  RunApplication.setInterval(1500); // 1.5 sec loop

  ADS.begin();
  ADS.setGain(1);     // 4V volt max for each entry
  ADS.setMode(1);     // mesures à la demande
  ADS.setDataRate(4); // vitesse de mesure 4 = 128 sample / Seconde
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
  uint16_t tempCellV = 0;
  for (iCell = (cellsNumber - 1); iCell >= 0; iCell--)
  {
    averageCell = 0;

    // take N samples in a row, with a slight delay
    for (iTemp2 = 0; iTemp2 < ADS_SAMPLE_NBR; iTemp2++)
    {
      averageCell += ADS.readADC(iCell);
      // Serial.println(averageCell);
      delay(9);
    }

    // moyenne des échantillons
    averageCell = averageCell / ADS_SAMPLE_NBR;

    vTemp = (int)(averageCell * adc_calibration[iCell]);

    if (vTemp < 0)
    {
      vTemp = 0;
    }
    BatteryCellsVoltage[iCell] = vTemp;
  }

  // Getting highest Cell and lowest Cell voltage
  LowestCellVoltage = 0;
  HighestCellVoltage = 0;
  uint16_t tempCellVA = 0;
  uint16_t tempCellVB = 0;
  for (iCell = (cellsNumber - 1); iCell >= 0; iCell--)
  {

    tempCellVA = BatteryCellsVoltage[iCell];

    if (iCell > 0)
    {
      tempCellVB = BatteryCellsVoltage[iCell - 1];
      tempCellV = tempCellVA - tempCellVB;
    }
    else
    {
      tempCellV = tempCellVA;
    }

    // setting the highest cell value
    if (tempCellV > HighestCellVoltage)
    {
      HighestCellVoltage = tempCellV;
    }

    // setting the lowest cell value
    if ((tempCellV < LowestCellVoltage) || (LowestCellVoltage == 0))
    {
      LowestCellVoltage = tempCellV;
    }
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
    checksumBmv += cBmv;

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

        byte result = checksumBmv % 256;

        // checksum OK
        if (result == 0)
        {
          SOC = SOCTemp;
          SOCUpdatedTime = millis();

#if IS_SERIAL
          Serial.println(F("SOC-UPD"));
#endif
        }

        // begin new serie
        checksumBmv = 0;
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

#if IS_SERIAL
  Serial.println(F("SOC NO VALID"));
#endif
  return false;
}

#if IS_SDCARD
void readSDCard()
{
  int data;

#if IS_RS485
  digitalWrite(RS485PinDE, HIGH);
#endif

#ifdef O_RDONLY
#else
#define O_RDONLY 0X00
#endif
  if (!logFile.open(DataLogFile, O_RDONLY))
  {

#if IS_RS485
    RS485.println(F("$E;NOSDFILE*"));
#endif

#if IS_SERIAL
    Serial.println(F("NOSDFILE"));
#endif
  }

#if IS_RS485
  // RS485.println(F("SDCARD"));
#endif

#if IS_SERIAL
// Serial.println(F("SDCARD"));
#endif

  while ((data = logFile.read()) >= 0)
  {
#if IS_SERIAL
    Serial.write(data);
#endif

#if IS_RS485
    RS485.write(data);
#endif

    delay(2);
  }

#if IS_RS485
  digitalWrite(RS485PinDE, LOW);
#endif

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

#ifdef O_WRONLY
#else
#define O_WRONLY 0X01
#endif

  if (!logFile.open(DataLogFile, O_WRONLY))
  {
    // sd.errorHalt("opening test.txt for write failed");
  }
  else
  {

    logFile.println(messageDate);
    logFile.close();
  }

#endif

#if IS_SERIAL
  Serial.println(message);
  Serial.println(F("-"));
#endif
}

String getDateTime()
{
#if IS_RTC
  // Serial.println("getDateTime");
  char heure[19];
  if (RTC.read(tm))
  {

    // Affiche l'heure courante retournee par le module RTC
    // Note : le %02d permet d'afficher les chiffres sur 2 digits (01, 02, ....)
    //ex : "2021/12/08 18:12:20"
    sprintf(heure, "%4d/%02d/%02d %02d:%02d:%02d", tmYearToCalendar(tm.Year), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second);
  }
  // Serial.println(heure);
  return heure;
#else
  return "";
#endif
}

#if IS_RS485
void handleRs485()
{
  while (RS485.available() > 0)
  {
    char incomingCharacter = RS485.read();

    switch (incomingCharacter)
    {
    case '1':
      printStatus();
      break;

    case '2':
#if IS_SDCARD
      readSDCard();
#endif
      break;

    case '3':
      printParams();
      break;

    case '8':
#if IS_SDCARD
      logData(F("T"));
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

#if IS_SERIAL
void handleSerial()
{
  while (Serial.available() > 0)
  {
    char incomingCharacter = Serial.read();
    Serial.println(incomingCharacter);
    switch (incomingCharacter)
    {
    case '1':
      printStatus();
      break;
    case '2':

#if IS_SDCARD
      readSDCard();
#endif
      break;

    case '8':

#if IS_SDCARD
      logData(F("Test"));
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

#if IS_SERIAL

#endif

  return false;
}

uint16_t getLowestCellVoltage()
{
  return LowestCellVoltage;
}

uint16_t getHighestCellVoltage()
{
  return HighestCellVoltage;
}

/**
   Calculate max cell difference between all cells
   Return value in mV
*/
unsigned int getMaxCellVoltageDifference()
{
  return HighestCellVoltage - LowestCellVoltage;
}

void printStatus()
{

  char messageStatusBuffer[64];
  sprintf(messageStatusBuffer, ("$S;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d*"),
          getBatteryVoltage(),
          getBatteryTemperature(),
          getBatterySOC(),
          // 
          getAdsCellVoltage(0),
          getAdsCellVoltage(1),
          getAdsCellVoltage(2),
          getAdsCellVoltage(3),
          // 
          ChargeRelay.getState(),
          LoadRelay.getState(),
          // 
          SOCChargeCycling,
          SOCDischargeCycling,
          // 
          isEnabledBMVSerialInfos(),
          isUseBMVSerialInfos(),
          // 
          LowVoltageDetected,
          HighVoltageDetected,
          CellVoltageMinDetected,
          CellVoltageMaxDetected,
          LowBatteryTemperatureDetected

  );

  // checksum calculation
  uint8_t checksum = 0;
  unsigned int sentenceIndex = 1; // Skip $ character at beginning
  const unsigned int sentencelength = strlen(messageStatusBuffer);
  for (; (sentenceIndex < sentencelength); ++sentenceIndex)
  {
    const char c = messageStatusBuffer[sentenceIndex];
    if (c == '*')
      break;
    checksum = checksum ^ c;
  }

  // deleting * char (last char)
  messageStatusBuffer[sentencelength - 1] = ';';

  // concat checksum to the sentence
  char messageEnd[6];
  sprintf(messageEnd, "%d*", checksum);
  strcat(messageStatusBuffer, messageEnd);

// Serial print
#if IS_SERIAL
  Serial.println(messageStatusBuffer);
#endif

// RS485 print
#if IS_RS485
  digitalWrite(RS485PinDE, HIGH);
  RS485.println(messageStatusBuffer);
  digitalWrite(RS485PinDE, LOW);
#endif
}

void printMemory()
{
  char messageEnd[10];
  sprintf(messageEnd, "$M;%d*", freeMemory());

// Serial
#if IS_SERIAL
  Serial.println(messageEnd);
#endif
// RS485
#if IS_RS485
  digitalWrite(RS485PinDE, HIGH);
  RS485.println(messageEnd);
  digitalWrite(RS485PinDE, LOW);
#endif
}

void printParams()
{
  char messageEnd[64];
  sprintf(messageEnd, "$P;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d*",
          SOCMax,
          SOCMaxReset,
          SOCMin,
          SOCMinReset,
          SOCMaxTimeValid,
          BatteryVoltageMax,
          BatteryVoltageMaxReset,
          BatteryVoltageMin,
          BatteryVoltageMinReset,
          CellVoltageMin,
          CellVoltageMinReset,
          CellVoltageMax,
          CellVoltageMaxReset,
          delayBeforeChargeOpening);
  // Serial communication
#if IS_SERIAL
  Serial.println(messageEnd);
#endif

// RS485
#if IS_RS485
  digitalWrite(RS485PinDE, HIGH);
  RS485.println(messageEnd);
  digitalWrite(RS485PinDE, LOW);
#endif
}

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

#if IS_SERIAL
  handleSerial();
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

#if IS_RS485
#if IS_PRINT_MEMORY
  printMemory();
#endif
  printStatus();
#endif

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

        // #18 Charge relay closing, routine without SOC
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

        // #20 Loading relay closing, routine without SOC
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
  // if not currently opening
  if (ChargeRelay.waitingForOpening != true)
  {
    if ((SOCDischargeCycling == true) || (HighVoltageDetected == true))
    {
      if ((ChargeRelay.getState() != ChargeRelay.RELAY_OPEN))
      {
        ChargeRelay.setReadyToOpen();

        // #23 Wrong Charging Relay state, opening
        MessageTemp = (String)ChargeRelay.getState() + F("!=") + (String)ChargeRelay.RELAY_OPEN;
        logDataMessNum(23, MessageTemp);
      }
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
      // if not currently opening
      if (ChargeRelay.waitingForOpening != true)
      {
        // Open Charge Relay
        SOCDischargeCycling = true;
        ChargeRelay.setReadyToOpen();

        // #25 SOC Current > Max : current/max"
        MessageTemp = (String)SOCCurrent + ValuesSpacer + (String)SOCMax;
        logDataMessNum(25, MessageTemp);
      }
    }
    // SOC Min detection when no Charging Cycle
    // => init Charging Cycling
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

    // if not currently opening
    if (ChargeRelay.waitingForOpening != true)
    {

      // NEW high voltage detection
      if ((CurrentBatteryVoltage >= BatteryVoltageMax) && (HighVoltageDetected == false))
      {
        // Open Charge Relay
        HighVoltageDetected = true;

        // set ready to open instead of forceOpen to let delay opening starts
        ChargeRelay.setReadyToOpen();

        MessageTemp = (String)(CurrentBatteryVoltage);
        logDataMessNum(2, MessageTemp, 0);
      }
      // HIGH VOLTAGE already detected
      else if (HighVoltageDetected == true)
      {

        // if not currently opening
        if (ChargeRelay.waitingForOpening != true)
        {
          // anormal state, force to open
          if (ChargeRelay.getState() == ChargeRelay.RELAY_CLOSE)
          {
            ChargeRelay.forceToOpen();
            logDataMessNum(3);
          }
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
  }

  // Reset MAX Individual Cell Voltage
  if (CellVoltageMaxDetected == true)
  {
    // if voltage cell low enough
    if (HighestCellVoltage <= CellVoltageMaxReset)
    {
      CellVoltageMaxDetected = false;
      ChargeRelay.setReadyToClose();

      // #27 RST OK : high V cell good
      MessageTemp = (String)cellVoltage;
      logDataMessNum(27, MessageTemp, 1);
    }
    else
    {
      // by security, force close each loop
      ChargeRelay.forceToOpen();
    }
  }

  // Reset LOW Individual Cell Voltage
  if (CellVoltageMinDetected == true)
  {
    // if voltage cell high enough
    if (LowestCellVoltage >= CellVoltageMinReset)
    {
      CellVoltageMinDetected = false;
      LoadRelay.setReadyToClose();

      // #29 RST Cell Voltage
      MessageTemp = (String)LowestCellVoltage;
      logDataMessNum(29, MessageTemp, 0);
    }
    else
    {
      // by security, force close on each loop
      LoadRelay.forceToOpen();
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