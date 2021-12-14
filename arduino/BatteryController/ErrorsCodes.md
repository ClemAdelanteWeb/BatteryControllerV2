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
#16 Load relay closing, routine without SOC
#17 SOC Current < SOC Max Charge relay closing
#18 SOC Charge relay closing, routine without SOC
#19 SOC min reset reached
    values : SOCCurrent/SOCMinReset
#20 Loading relay closing without SOC    
#21 Cancelling Discharge Cycling
    values : SOCCurrent/SOCMaxReset
#22 Cancelling Discharge Cycling without SOC
#23 Wrong Charging relay state, opening
#24 Wrong Loading relay state, opening
#25 SOC Current > Max : current/max
#26 High individual cell voltage
    value : num / mV
#27 RST OK : high V cell good
    value : num / mV
#28 Low cell voltage
    value : num / mV
#29 RST Cell Voltage
    value : num / mV
#30 Low T°
    value : T°
#31 Low T°
    value : T°
#32 T° min reset, closing charging relay

               MessageTemp = (String)SOCCurrent + F(",") + TxtSpacer + (String)SOCMin;
               logDataMessNum(15);