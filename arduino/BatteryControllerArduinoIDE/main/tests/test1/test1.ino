/*=================================================================
Ce sketch est un scanner I2C: il essaye de communiquer avec toutes 
les adresses I2C possibles et affiche celle(s) qui réponde(nt).
  
                      BRANCHEMENT
* Pin SCD du moule à scanner  ----------->  SCD de l’Arduino
* Pin SDA du moule à scanner  ----------->  SDA de l’Arduino
================================================================ */
// SDA = A4
// SCL = A5
#include <Wire.h>
void setup()
{
  Wire.begin();
  Serial.begin(19200);
  Serial.println("INIT");
}

void loop()
{
        
}



/**
   Calculate max cell difference between all cells
   Return value in mV
*/
 int getMaxCellVoltageDifference() {

  // Cells voltages
   int  cellsVoltage[(cellsNumber - 1)];
   int maxDiffCells = 0;
   int  minValue = 0;
   int  maxValue = 0;
  int iTemp4;

  for (iTemp4 = (cellsNumber - 1); iTemp4 >= 0; iTemp4--) {
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


  // return 815;


Serial.print("max value ");
Serial.println(maxValue);

Serial.print("min value ");
Serial.println(minValue);
return minValue;

  maxDiffCells = maxValue-minValue;
Serial.print("diff value ");
Serial.println(maxDiffCells);
  return maxDiffCells;
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
 int  tempVal = getMaxCellVoltageDifference();
//  Serial.print(getBatteryVoltage());Serial.print(";");
//  Serial.print(TempS);Serial.print(";");
//  Serial.print(getBatterySOC());Serial.print(";");
//  Serial.print(getAdsCellVoltage(0));Serial.print(";");
//  Serial.print(getAdsCellVoltage(1));Serial.print(";");
//  Serial.print(getAdsCellVoltage(2));Serial.print(";");
//  Serial.print(getAdsCellVoltage(3));Serial.print(";");
  Serial.print(660);Serial.print(";");
  
}
