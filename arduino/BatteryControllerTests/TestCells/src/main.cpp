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
  0.1780487,
  0.2134769,
  0.4977307,
  0.5890515
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
  Serial.begin(19200);
  Serial.println("Hello!");
  
  ADS.begin();
  ADS.setGain(0);       //4V volt
  ADS.setMode(1);       // mesures à la demande
  ADS.setDataRate(6);   // vitesse de mesure de 1 à 7
  ADS.readADC(0);       // Et on fait une lecture à vide, pour envoyer tous ces paramètres
}

void loop(void) 
{
  Serial.println("Loop");
  // checkCellsVoltage();
  // printStatus();
  delay(3000);
}


void printStatus() {
  char output[60];

  // affichage des données actuelles
  //
  //  V Batt ; T° Batt ;;
  //  V Cell1 ; V Cell2 ; V Cell3 ; V Cell4 ; V Cell Max Diff ;;
  //  Ch Relay Status ; Load Relay Status ;;
  //  SOC Charge Cycling ; SOC Discharge Cycling;;
  //  Use SOC data ; Using SOC data ? ; SOC Value ;;
  //  Low Voltage detected ; High voltage detected
  //$%d;%s;%d;;%d;%d;%d;%d;%d;;%d;%d;;%d;%d;;%d;%d;;%d;%d#
   sprintf(output, "Cell 1 : $%d V \n Cell 2 : %d mV \n Cell 3 : %d mV \n Cell 4 : %d mV \n Max Diff : %d mV",
           getAdsCellVoltage(0),
           getAdsCellVoltage(1),
           getAdsCellVoltage(2),
           getAdsCellVoltage(3),
           getMaxCellVoltageDifference()
    );

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


/**
  Return cell Voltage
*/
uint16_t getAdsCellVoltage(int cellNumber) {

  //  for (int element : BatteryCellsVoltage) // for each element in the array
  //    Serial.println(element);

  return BatteryCellsVoltage[cellNumber];

}

