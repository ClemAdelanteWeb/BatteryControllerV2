#1 SOC min reached : now / min 
    values : SOC actual/ SOC Min param
#2 High Voltage detected
    values : Voltage
#3 High voltage detected, Force Opening Charge Relay 
#4  Voltage Max Reset reached, charge relay closing
#5 Low voltage detected
#6 Charge Relay Closing
    value : SOC, Battery voltage
#7 Low voltage detected, Load relay force closing
#8 Charge relay force closing. 2nd attempt
#9 Voltage min Reset reached. Load Relay closing
#10 No Voltage detected Load Relay Force Open
    value : temps écoulé depuis dernière valeur en ms
#11 Cells Voltage diff high, Load relay Force open
    value : mV cell diff 
#12 Reset OK Cells Voltage diff < Rst. Closing Load Relay
#13 Cells diff upd > 10s
#14 ADS 3 cell voltage not available
#15 SOC Current > SOC Min
    => normal routine detection
#16 Load relay closing, routine without SOC
#17 SOC Current < SOC Max Charge relay closing
    => can happen when forcing open Charge Relay manually 
#18 Charge relay closing, routine without SOC
#19 SOC min reset reached
    values : SOCCurrent/SOCMinReset
#20 Loading relay closing, routine without SOC    
#21 Cancelling Discharge Cycling
    values : SOCCurrent/SOCMaxReset
#22 Cancelling Discharge Cycling without SOC
#23 Wrong Charging relay state, opening 
    => Can append when manually closing Charge Relay
    => Can also append when HighVoltage detected is going to be RESET AND SOC is just reconnected, normal routine
#24 Wrong Loading relay state, opening
#25 SOC Current > Max : current/max
#26 High individual cell voltage
    value : num / mV
#27 RST OK : high V cell good
    value : Highest cell voltage in mV
#28 Low cell voltage
    value : num / mV
#29 RST Cell Voltage
    value : Lowest cell voltage in mV
#30 Low T°
    value : T°
#31 Low T°
    value : T°
#32 T° min reset, closing charging relay

               MessageTemp = (String)SOCCurrent + F(",") + TxtSpacer + (String)SOCMin;
               logDataMessNum(15);



sprintf(messageStatusBuffer, ("$%d;%d;%d;;%d;%d;%d;%d;%d;;%d;%d;;%d;%d;%d;%d;;%d;%d;%d;%d;%d;%d*"),
          getBatteryVoltage(),
          getBatteryTemperature(),
          getBatterySOC(),
          // vide
          getAdsCellVoltage(0),
          getAdsCellVoltage(1),
          getAdsCellVoltage(2),
          getAdsCellVoltage(3),
          getMaxCellVoltageDifference(),
          // vide
          ChargeRelay.getState(),
          LoadRelay.getState(),
          // vide
          SOCChargeCycling,
          SOCDischargeCycling,
          isEnabledBMVSerialInfos(),
          isUseBMVSerialInfos(),
          // vide
          LowVoltageDetected,
           ,
          CellsDifferenceDetected,
          CellVoltageMinDetected,
          CellVoltageMaxDetected,
          LowBatteryTemperatureDetected

  );