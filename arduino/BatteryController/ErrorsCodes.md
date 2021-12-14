#1 SOC min reached : now / min 
    values : SOC actual, SOC Min param
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