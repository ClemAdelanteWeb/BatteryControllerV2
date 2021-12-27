#include <Arduino.h>
#include <Wire.h>
#include <ADS1X15.h>

// ADS1115 on I2C0x48 adress
ADS1115 ADS(0x48);

#define NUMSAMPLES 10

// Number of cells in the battery
const int cellsNumber = 4;

// ADS1115 Calibration at 10v *1000
// Ex: 10 / 18107 = 0,000552273
// cell 1, 2, 3, 4 etc
const float adc_calibration[cellsNumber] = {
  0.11947112,
  0.22990510,
  0.32871799,
  0.42420728
};


uint16_t BatteryCellsVoltage[cellsNumber];

uint16_t BatteryVoltage;
unsigned long BatteryVoltageUpdatedTime;

int CellsDifferenceMax;

void checkCellsVoltage();
void printStatus();
int getMaxCellVoltageDifference();
uint16_t getAdsCellVoltage(int cellNumber);


void setup(void) 
{
  Serial.begin(38400);
  Serial.println("Hello!");
  
  ADS.begin();
  ADS.setGain(1);       //4V volt
  ADS.setMode(1);       // mesures à la demande
  ADS.setDataRate(4);   // vitesse de mesure de 1 à 7
  ADS.readADC(0);       // Et on fait une lecture à vide, pour envoyer tous ces paramètres
}

void loop(void) 
{
  Serial.println("Loop");
  checkCellsVoltage();
  //printStatus();
  delay(5000);
}


void printStatus() {


  

  int iCell;
  for (iCell = 0; iCell <= cellsNumber; iCell++) {

      Serial.print("Cell "); Serial.print(iCell); Serial.print(" "); 
      Serial.println(getAdsCellVoltage(iCell));
  }

      Serial.print("Cell max diff : "); Serial.println(getMaxCellVoltageDifference());


}



void checkCellsVoltage()
{
  unsigned long averageCell;
  int iCell, iTemp2;
  uint16_t vTemp;
  uint16_t PastCell;

PastCell = 0;
  for (iCell = 0; iCell < cellsNumber; iCell++)
  {
    averageCell = 0;

    // take N samples in a row, with a slight delay
    for (iTemp2 = 0; iTemp2 < NUMSAMPLES; iTemp2++)
    {
      averageCell += ADS.readADC(iCell);
      // Serial.println(averageCell);
      delay(10);
    }

    // moyenne des échantillons
    averageCell = averageCell / NUMSAMPLES;

  

    vTemp = (int)(averageCell * adc_calibration[iCell]);

          Serial.print("Average : "); Serial.print(iCell+1); Serial.print(" ");
       Serial.print(averageCell); Serial.print(" / ");
       Serial.print(vTemp);Serial.print(" / ");
Serial.println((vTemp-PastCell));
    if (vTemp < 0)
    {
      vTemp = 0;
    }
PastCell = vTemp;

    BatteryCellsVoltage[iCell] = vTemp;
  }
  
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
  int iTemp4;

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


/**
  Return cell Voltage
*/
uint16_t getAdsCellVoltage(int cellNumber) {

  //  for (int element : BatteryCellsVoltage) // for each element in the array
  //    Serial.println(element);

  return BatteryCellsVoltage[cellNumber];

}

