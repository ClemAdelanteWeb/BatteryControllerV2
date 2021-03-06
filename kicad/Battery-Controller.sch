EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "Battery Controller Almarita Boat"
Date "2021-11-25"
Rev "V2"
Comp ""
Comment1 "Clément Lambert"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Notes Line
	700  2850 5350 2850
Wire Notes Line
	5350 2850 5350 700 
Wire Notes Line
	5350 700  700  700 
Wire Notes Line
	700  700  700  2850
$Comp
L Battery-Controller-rescue:Arduino_Nano_v3.x-MCU_Module A2
U 1 1 5FF622F3
P 6700 4300
F 0 "A2" H 6700 3211 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 6700 3120 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 6850 3350 50  0001 L CNN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 6700 3300 50  0001 C CNN
	1    6700 4300
	1    0    0    -1  
$EndComp
$Comp
L Analog_ADC:ADS1115IDGS U3
U 1 1 5FF655F5
P 4600 4250
F 0 "U3" H 4600 4931 50  0000 C CNN
F 1 "ADS1115IDGS" H 4600 4840 50  0000 C CNN
F 2 "Charleslabs_Parts:ADC1115_ADC_Module" H 4600 3750 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/ads1113.pdf" H 4550 3350 50  0001 C CNN
	1    4600 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 3750 4600 3400
Wire Wire Line
	4600 3400 3900 3400
$Comp
L Device:CP C3
U 1 1 5FF67E8D
P 3900 3550
F 0 "C3" H 4018 3596 50  0000 L CNN
F 1 "0.1uF" H 4018 3505 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D8.0mm_W5.0mm_P7.50mm" H 3938 3400 50  0001 C CNN
F 3 "~" H 3900 3550 50  0001 C CNN
	1    3900 3550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 61AD507C
P 2250 3900
F 0 "R3" V 2043 3900 50  0000 C CNN
F 1 "21.6K" V 2134 3900 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2180 3900 50  0001 C CNN
F 3 "~" H 2250 3900 50  0001 C CNN
	1    2250 3900
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 61AD5529
P 2550 3900
F 0 "R6" V 2757 3900 50  0000 C CNN
F 1 "32K" V 2666 3900 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2480 3900 50  0001 C CNN
F 3 "~" H 2550 3900 50  0001 C CNN
	1    2550 3900
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R4
U 1 1 61AF8771
P 2250 4250
F 0 "R4" V 2043 4250 50  0000 C CNN
F 1 "474" V 2134 4250 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2180 4250 50  0001 C CNN
F 3 "~" H 2250 4250 50  0001 C CNN
	1    2250 4250
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 61AF877B
P 2550 4250
F 0 "R7" V 2757 4250 50  0000 C CNN
F 1 "330" V 2666 4250 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2480 4250 50  0001 C CNN
F 3 "~" H 2550 4250 50  0001 C CNN
	1    2550 4250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2400 3900 2400 4000
Connection ~ 2400 3900
Wire Wire Line
	4200 4450 3700 4450
Connection ~ 2400 4600
Wire Wire Line
	2400 4600 2400 4700
$Comp
L Device:R R5
U 1 1 61AFDF3A
P 2250 4600
F 0 "R5" V 2043 4600 50  0000 C CNN
F 1 "5.1K" V 2134 4600 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2180 4600 50  0001 C CNN
F 3 "~" H 2250 4600 50  0001 C CNN
	1    2250 4600
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 61AFDF44
P 2550 4600
F 0 "R8" V 2757 4600 50  0000 C CNN
F 1 "2.2K" V 2666 4600 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2480 4600 50  0001 C CNN
F 3 "~" H 2550 4600 50  0001 C CNN
	1    2550 4600
	0    -1   -1   0   
$EndComp
Text GLabel 5000 4350 2    50   Input ~ 0
UP-SDA
Text GLabel 5000 4250 2    50   Input ~ 0
UP-SCL
Text GLabel 7200 4700 2    50   Input ~ 0
SDA
Text GLabel 7200 4800 2    50   Input ~ 0
SCL
Text GLabel 6200 4400 0    50   Input ~ 0
D7
$Comp
L Transistor_BJT:TIP120 Q1
U 1 1 61B48AE2
P 6750 6300
F 0 "Q1" H 6941 6346 50  0000 L CNN
F 1 "TIP120" H 6941 6255 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 6950 6400 50  0001 C CNN
F 3 "~" H 6750 6300 50  0001 C CNN
	1    6750 6300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 61B49F2F
P 6400 6300
F 0 "R10" V 6193 6300 50  0000 C CNN
F 1 "1K" V 6284 6300 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 6330 6300 50  0001 C CNN
F 3 "~" H 6400 6300 50  0001 C CNN
	1    6400 6300
	0    1    1    0   
$EndComp
Wire Notes Line
	5900 6950 7750 6950
Wire Notes Line
	7750 6950 7750 5650
Wire Notes Line
	7750 5650 5900 5650
Wire Notes Line
	5900 5650 5900 6950
Text Notes 6400 6900 0    50   ~ 0
Indication sortie charge possible\nLOW SIDE (GND output)\n
$Comp
L Device:Fuse F1
U 1 1 61B5B3F4
P 6850 5950
F 0 "F1" H 6910 5996 50  0000 L CNN
F 1 "Fuse" H 6910 5905 50  0000 L CNN
F 2 "Fuse:Fuseholder_Blade_ATO_Littelfuse_Pudenz_2_Pin" V 6780 5950 50  0001 C CNN
F 3 "~" H 6850 5950 50  0001 C CNN
	1    6850 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 6500 6850 6650
Text GLabel 6850 5750 2    50   Input ~ 0
CHARGING-STATUS
Text Notes 11350 8800 0    50   ~ 0
LOAD relay\n
Text Notes 11350 6400 0    50   ~ 0
CHARGE relay\n
$Comp
L Timer_RTC:DS1307+ U2
U 1 1 61B7704D
P 3750 6050
F 0 "U2" H 4294 6096 50  0000 L CNN
F 1 "DS1307+" H 4294 6005 50  0000 L CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 3750 5550 50  0001 C CNN
F 3 "https://datasheets.maximintegrated.com/en/ds/DS1307.pdf" H 3750 5850 50  0001 C CNN
	1    3750 6050
	1    0    0    -1  
$EndComp
Text GLabel 3250 5850 0    50   Input ~ 0
UP-SCL
Text GLabel 3250 5950 0    50   Input ~ 0
UP-SDA
Text GLabel 6900 3300 1    50   Input ~ 0
5V
$Comp
L power:GND #PWR0121
U 1 1 61B8176E
P 6800 5300
F 0 "#PWR0121" H 6800 5050 50  0001 C CNN
F 1 "GND" H 6805 5127 50  0000 C CNN
F 2 "" H 6800 5300 50  0001 C CNN
F 3 "" H 6800 5300 50  0001 C CNN
	1    6800 5300
	1    0    0    -1  
$EndComp
Wire Notes Line
	2900 6900 4700 6900
Wire Notes Line
	4700 6900 4700 5400
Wire Notes Line
	4700 5400 2900 5400
Wire Notes Line
	2900 5400 2900 6900
Text Notes 4200 6850 0    50   ~ 0
Horloge RTC\nI2C\n
Text GLabel 6800 3300 1    50   Input ~ 0
3.3V
$Comp
L Charleslabs_Parts:SD_Card_Module A1
U 1 1 61B94D6F
P 1850 5950
F 0 "A1" H 2180 5996 50  0000 L CNN
F 1 "SD_Card_Module" H 2180 5905 50  0000 L CNN
F 2 "Charleslabs_Parts:SD_Card_Module" H 2800 6000 50  0001 C CNN
F 3 "" H 1550 6300 50  0001 C CNN
	1    1850 5950
	1    0    0    -1  
$EndComp
Text GLabel 1450 6100 0    50   Input ~ 0
UP-MISO
Text GLabel 1450 5900 0    50   Input ~ 0
UP-MOSI
Text GLabel 1450 6000 0    50   Input ~ 0
UP-CLK
Text GLabel 1450 5800 0    50   Input ~ 0
UP-CS
Text GLabel 6200 4700 0    50   Input ~ 0
D10
Text GLabel 6200 4800 0    50   Input ~ 0
D11
Text GLabel 6200 4900 0    50   Input ~ 0
D12
Text GLabel 6200 5000 0    50   Input ~ 0
D13
Wire Notes Line
	2750 6850 2750 5300
Wire Notes Line
	2750 5300 1050 5300
Wire Notes Line
	1050 5300 1050 6850
Wire Notes Line
	1050 6850 2750 6850
Text Notes 2100 6800 0    50   ~ 0
SD Card reader\nSPI
Wire Notes Line
	5300 3200 1100 3200
Wire Notes Line
	5300 5200 1100 5200
Wire Notes Line
	5300 3200 5300 5200
Wire Notes Line
	1100 3200 1100 5200
Text Notes 4400 5150 0    50   ~ 0
ADS1115 ADC Voltage\nI2C\n
$Comp
L Device:R R1
U 1 1 61BBE7E5
P 2100 7400
F 0 "R1" V 2307 7400 50  0000 C CNN
F 1 "10K" V 2216 7400 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2030 7400 50  0001 C CNN
F 3 "~" H 2100 7400 50  0001 C CNN
	1    2100 7400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1750 7400 1900 7400
Connection ~ 1900 7400
Wire Wire Line
	1900 7400 1950 7400
Text GLabel 2000 7550 2    50   Input ~ 0
A0
Text GLabel 7200 4900 2    50   Input ~ 0
A6
Wire Wire Line
	1900 7550 2000 7550
Wire Wire Line
	1900 7400 1900 7550
Text Notes 1750 8050 0    50   ~ 0
Sonde T° batterie\n
$Comp
L power:GND #PWR0110
U 1 1 61B2F8D3
P 9300 5200
F 0 "#PWR0110" H 9300 4950 50  0001 C CNN
F 1 "GND" H 9305 5027 50  0000 C CNN
F 2 "" H 9300 5200 50  0001 C CNN
F 3 "" H 9300 5200 50  0001 C CNN
	1    9300 5200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R16
U 1 1 61B400D0
P 9300 4600
F 0 "R16" H 9370 4646 50  0000 L CNN
F 1 "1K" H 9370 4555 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9230 4600 50  0001 C CNN
F 3 "~" H 9300 4600 50  0001 C CNN
F 4 "Pull down in case pin hi-impedence" H 9300 4600 50  0001 C CNN "Field4"
	1    9300 4600
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N5822 D6
U 1 1 61AA4E76
P 10700 4800
F 0 "D6" V 10654 4880 50  0000 L CNN
F 1 "1N5822" V 10745 4880 50  0000 L CNN
F 2 "Diode_THT:D_5W_P5.08mm_Vertical_AnodeUp" H 10700 4625 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88526/1n5820.pdf" H 10700 4800 50  0001 C CNN
	1    10700 4800
	0    1    1    0   
$EndComp
Text GLabel 6200 4000 0    50   Input ~ 0
D3
Text GLabel 6200 4100 0    50   Input ~ 0
D4
Text GLabel 7200 4300 2    50   Input ~ 0
A0
Text GLabel 6200 3900 0    50   Input ~ 0
D2
Text GLabel 8700 5000 0    50   Input ~ 0
A1
Wire Notes Line
	8450 8900 11900 8900
Text GLabel 7200 5000 2    50   Input ~ 0
A7
Text GLabel 6900 1150 3    50   Input ~ 0
A7
Text Notes 7100 1800 0    50   ~ 0
A6 : Charge relay status\nA7 : Load relay status
$Comp
L Device:R R14
U 1 1 61C498FA
P 7350 1000
F 0 "R14" V 7143 1000 50  0000 C CNN
F 1 "10K" V 7234 1000 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7280 1000 50  0001 C CNN
F 3 "~" H 7350 1000 50  0001 C CNN
	1    7350 1000
	0    1    1    0   
$EndComp
Wire Wire Line
	7200 1000 6900 1000
Wire Notes Line
	8150 700  8150 1900
Text GLabel 6200 4200 0    50   Input ~ 0
D5
Text GLabel 6200 4300 0    50   Input ~ 0
D6
Text GLabel 6200 6300 0    50   Input ~ 0
D6
Wire Wire Line
	6250 6300 6200 6300
$Comp
L Connector:Screw_Terminal_01x04 J1
U 1 1 61C9A0B6
P 10100 900
F 0 "J1" H 10180 892 50  0000 L CNN
F 1 "Screw_Terminal_01x04 - RELAY" H 10180 801 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-4-5.08_1x04_P5.08mm_Horizontal" H 10100 900 50  0001 C CNN
F 3 "~" H 10100 900 50  0001 C CNN
	1    10100 900 
	1    0    0    -1  
$EndComp
Text GLabel 9900 2350 0    50   Input ~ 0
CHARGING-STATUS
$Comp
L Connector:Screw_Terminal_01x04 J2
U 1 1 61CA82FD
P 10100 1550
F 0 "J2" H 10180 1542 50  0000 L CNN
F 1 "Screw_Terminal_01x04 - COM" H 10180 1451 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-4-5.08_1x04_P5.08mm_Horizontal" H 10100 1550 50  0001 C CNN
F 3 "~" H 10100 1550 50  0001 C CNN
	1    10100 1550
	1    0    0    -1  
$EndComp
Text GLabel 9900 1450 0    50   Input ~ 0
VICTRON-IN-1
Text GLabel 9900 1550 0    50   Input ~ 0
VICTRON-IN-2
Text GLabel 10950 7550 2    50   Input ~ 0
CHARGE-RELAY-ON
Text GLabel 10950 8600 2    50   Input ~ 0
CHARGE-RELAY-OFF
Text GLabel 10950 5150 2    50   Input ~ 0
LOAD-RELAY-ON
Text GLabel 10950 6200 2    50   Input ~ 0
LOAD-RELAY-OFF
Text GLabel 9900 1000 0    50   Input ~ 0
LOAD-RELAY-ON
Text GLabel 9900 1100 0    50   Input ~ 0
LOAD-RELAY-OFF
Text GLabel 9900 800  0    50   Input ~ 0
CHARGE-RELAY-ON
Text GLabel 9900 900  0    50   Input ~ 0
CHARGE-RELAY-OFF
$Comp
L Interface_UART:SN75176AP U4
U 1 1 61CD3635
P 3600 7750
F 0 "U4" H 3600 8331 50  0000 C CNN
F 1 "SN75176AP" H 3600 8240 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 3600 7250 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn75176a.pdf" H 5200 7550 50  0001 C CNN
	1    3600 7750
	1    0    0    -1  
$EndComp
Text GLabel 3300 7950 0    50   Input ~ 0
UP-D3
Text GLabel 3300 7650 0    50   Input ~ 0
UP-D2
Wire Wire Line
	3600 7400 3600 7350
Wire Wire Line
	4200 7350 4050 7350
Connection ~ 3600 7350
Text GLabel 3900 7850 2    50   Input ~ 0
RS-485-TX
Text GLabel 3900 7950 2    50   Input ~ 0
RS-485-RX
Text GLabel 9900 1650 0    50   Input ~ 0
RS-485-RX
Text GLabel 9900 1750 0    50   Input ~ 0
RS-485-TX
Text Notes 4100 8450 0    50   ~ 0
RS-485 BUS
Wire Notes Line
	2900 7100 4700 7100
Wire Notes Line
	4700 7100 4700 8550
Wire Notes Line
	4700 8550 2900 8550
Wire Notes Line
	2900 8550 2900 7100
Text GLabel 7200 4400 2    50   Input ~ 0
A1
Text GLabel 7200 4500 2    50   Input ~ 0
A2
Text GLabel 1750 7400 0    50   Input ~ 0
THERMOCOUPLE-IN-P
$Comp
L Connector:Screw_Terminal_01x08 J3
U 1 1 61D0860E
P 10100 2250
F 0 "J3" H 10180 2242 50  0000 L CNN
F 1 "Screw_Terminal_01x08" H 10180 2151 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-8-5.08_1x08_P5.08mm_Horizontal" H 10100 2250 50  0001 C CNN
F 3 "~" H 10100 2250 50  0001 C CNN
	1    10100 2250
	1    0    0    -1  
$EndComp
Text GLabel 9900 2150 0    50   Input ~ 0
THERMOCOUPLE-IN-N
Text GLabel 1750 7700 0    50   Input ~ 0
THERMOCOUPLE-IN-N
$Comp
L power:GND #PWR0131
U 1 1 61D0ADB1
P 1900 7700
F 0 "#PWR0131" H 1900 7450 50  0001 C CNN
F 1 "GND" H 1905 7527 50  0000 C CNN
F 2 "" H 1900 7700 50  0001 C CNN
F 3 "" H 1900 7700 50  0001 C CNN
	1    1900 7700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 7700 1750 7700
Wire Notes Line
	750  7050 750  8100
$Comp
L Connector:Screw_Terminal_01x02 J4
U 1 1 61D1C628
P 10100 2900
F 0 "J4" H 10180 2892 50  0000 L CNN
F 1 "Screw_Terminal_01x02 - Battery IN" H 10180 2801 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 10100 2900 50  0001 C CNN
F 3 "~" H 10100 2900 50  0001 C CNN
	1    10100 2900
	1    0    0    -1  
$EndComp
Text GLabel 9900 2900 0    50   Input ~ 0
BAT-12V
Text GLabel 9900 3000 0    50   Input ~ 0
BAT-GND
Text GLabel 1400 2600 0    50   Input ~ 0
BAT-12V
$Comp
L power:+12V #PWR0133
U 1 1 61D277CD
P 1550 2600
F 0 "#PWR0133" H 1550 2450 50  0001 C CNN
F 1 "+12V" H 1565 2773 50  0000 C CNN
F 2 "" H 1550 2600 50  0001 C CNN
F 3 "" H 1550 2600 50  0001 C CNN
	1    1550 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 2600 1400 2600
Text GLabel 2250 2600 0    50   Input ~ 0
BAT-GND
$Comp
L power:GND #PWR0134
U 1 1 61D2BCE6
P 2400 2600
F 0 "#PWR0134" H 2400 2350 50  0001 C CNN
F 1 "GND" H 2405 2427 50  0000 C CNN
F 2 "" H 2400 2600 50  0001 C CNN
F 3 "" H 2400 2600 50  0001 C CNN
	1    2400 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 2600 2250 2600
Text GLabel 13450 2250 0    50   Input ~ 0
VICTRON-IN-1
Text GLabel 13450 2350 0    50   Input ~ 0
VICTRON-IN-2
Text GLabel 9900 2250 0    50   Input ~ 0
THERMOCOUPLE-IN-P
Text GLabel 6500 1000 0    50   Input ~ 0
CHARGE-RELAY-STATUS
Text GLabel 6400 1100 0    50   Input ~ 0
LOAD-RELAY-STATUS
Wire Notes Line
	5450 1900 5450 700 
Wire Notes Line
	5450 1900 8150 1900
Wire Notes Line
	5450 700  8150 700 
Text GLabel 9900 1950 0    50   Input ~ 0
CHARGE-RELAY-STATUS
Text GLabel 9900 2050 0    50   Input ~ 0
LOAD-RELAY-STATUS
Text GLabel 9900 2550 0    50   Input ~ 0
SOC-ENABLE
Text GLabel 1350 8400 0    50   Input ~ 0
SOC-ENABLE
Text GLabel 7200 4600 2    50   Input ~ 0
A3
Text GLabel 2100 8400 2    50   Input ~ 0
A3
$Comp
L Device:R R2
U 1 1 61D77E00
P 1700 8550
F 0 "R2" H 1770 8596 50  0000 L CNN
F 1 "4.7K" H 1770 8505 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1630 8550 50  0001 C CNN
F 3 "~" H 1700 8550 50  0001 C CNN
	1    1700 8550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 8400 1700 8400
Connection ~ 1700 8400
Wire Wire Line
	1700 8400 2100 8400
Wire Wire Line
	1700 8700 1700 8750
Wire Notes Line
	750  8250 2500 8250
Wire Notes Line
	2500 8250 2500 9300
Wire Notes Line
	2500 9300 750  9300
Wire Notes Line
	750  9300 750  8250
Text Notes 1650 9200 0    50   ~ 0
SOC ON/OFF Bouton
$Comp
L Connector:Screw_Terminal_01x04 J6
U 1 1 61DEB6EA
P 10100 3300
F 0 "J6" H 10180 3292 50  0000 L CNN
F 1 "Screw_Terminal_01x04 - Cells " H 10180 3201 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-4-5.08_1x04_P5.08mm_Horizontal" H 10100 3300 50  0001 C CNN
F 3 "~" H 10100 3300 50  0001 C CNN
	1    10100 3300
	1    0    0    -1  
$EndComp
Text GLabel 9900 3300 0    50   Input ~ 0
CELL-2
Text GLabel 9900 3400 0    50   Input ~ 0
CELL-3
Text GLabel 9900 3500 0    50   Input ~ 0
CELL-4
Text GLabel 1750 3550 0    50   Input ~ 0
CELL-1
Text GLabel 1750 3900 0    50   Input ~ 0
CELL-2
Text GLabel 1750 4250 0    50   Input ~ 0
CELL-3
Text GLabel 1750 4600 0    50   Input ~ 0
CELL-4
Wire Wire Line
	2100 3900 1750 3900
Wire Wire Line
	2100 4250 1750 4250
Wire Wire Line
	2700 4250 3000 4250
Connection ~ 3000 4250
Wire Wire Line
	2700 3900 3000 3900
Connection ~ 3000 3550
Wire Wire Line
	3600 4150 3600 3550
Wire Wire Line
	3000 3550 3600 3550
Connection ~ 3000 3900
Wire Wire Line
	3450 4000 2400 4000
Wire Wire Line
	3450 4000 3450 4250
Wire Wire Line
	3000 3900 3000 4250
Wire Wire Line
	1750 4600 2100 4600
Wire Wire Line
	4600 3400 4900 3400
Connection ~ 4600 3400
Wire Wire Line
	3700 4450 3700 4550
Wire Wire Line
	3000 4250 3000 4600
Wire Wire Line
	3100 4700 2400 4700
Wire Wire Line
	3100 4550 3100 4700
Wire Wire Line
	2700 4600 3000 4600
Connection ~ 3000 4600
Wire Wire Line
	3000 4600 3000 4900
Wire Wire Line
	2400 4350 2400 4250
Connection ~ 2400 4250
Wire Wire Line
	2800 4900 3000 4900
$Comp
L Connector:Conn_01x15_Male J7
U 1 1 620EA2CE
P 13650 1750
F 0 "J7" H 13622 1728 50  0000 R CNN
F 1 "Conn_01x15_Male" H 13622 1633 50  0001 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x15_P2.54mm_Vertical" H 13650 1750 50  0001 C CNN
F 3 "~" H 13650 1750 50  0001 C CNN
	1    13650 1750
	-1   0    0    -1  
$EndComp
Text GLabel 13450 1350 0    50   Input ~ 0
UP-SDA
Text GLabel 13450 1450 0    50   Input ~ 0
UP-SCL
Text GLabel 13450 1550 0    50   Input ~ 0
UP-CLK
Text GLabel 13450 1650 0    50   Input ~ 0
UP-CS
Text GLabel 13450 1750 0    50   Input ~ 0
UP-MISO
Text Notes 13100 2850 0    50   ~ 0
Connecteur Male \nPCB supérieur
Text Notes 13950 2850 0    50   ~ 0
Connecteur Femelle \nPCB Inférieur\n
Text GLabel 14000 1750 2    50   Input ~ 0
D12
Text GLabel 14000 1650 2    50   Input ~ 0
D10
Text GLabel 14000 1550 2    50   Input ~ 0
D13
Text GLabel 14000 1450 2    50   Input ~ 0
SCL
Text GLabel 14000 1350 2    50   Input ~ 0
SDA
Text GLabel 14000 1250 2    50   Input ~ 0
3.3V
$Comp
L Connector:Conn_01x15_Female J8
U 1 1 620F9710
P 13800 1750
F 0 "J8" H 13692 2543 50  0000 C CNN
F 1 "Conn_01x15_Female" H 13692 2244 50  0001 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x15_P2.54mm_Vertical" H 13800 1750 50  0001 C CNN
F 3 "~" H 13800 1750 50  0001 C CNN
	1    13800 1750
	-1   0    0    -1  
$EndComp
$Comp
L power:GND2 #PWR0106
U 1 1 62138146
P 4600 4650
F 0 "#PWR0106" H 4600 4400 50  0001 C CNN
F 1 "GND2" H 4605 4477 50  0000 C CNN
F 2 "" H 4600 4650 50  0001 C CNN
F 3 "" H 4600 4650 50  0001 C CNN
	1    4600 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND2 #PWR0122
U 1 1 62148D17
P 3900 3700
F 0 "#PWR0122" H 3900 3450 50  0001 C CNN
F 1 "GND2" H 3905 3527 50  0000 C CNN
F 2 "" H 3900 3700 50  0001 C CNN
F 3 "" H 3900 3700 50  0001 C CNN
	1    3900 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND2 #PWR0123
U 1 1 62149169
P 2800 4900
F 0 "#PWR0123" H 2800 4650 50  0001 C CNN
F 1 "GND2" H 2805 4727 50  0000 C CNN
F 2 "" H 2800 4900 50  0001 C CNN
F 3 "" H 2800 4900 50  0001 C CNN
	1    2800 4900
	1    0    0    -1  
$EndComp
$Comp
L power:GND2 #PWR0136
U 1 1 621804D7
P 1850 6350
F 0 "#PWR0136" H 1850 6100 50  0001 C CNN
F 1 "GND2" H 1855 6177 50  0000 C CNN
F 2 "" H 1850 6350 50  0001 C CNN
F 3 "" H 1850 6350 50  0001 C CNN
	1    1850 6350
	1    0    0    -1  
$EndComp
$Comp
L power:GND2 #PWR0141
U 1 1 621A2E7B
P 3750 6450
F 0 "#PWR0141" H 3750 6200 50  0001 C CNN
F 1 "GND2" H 3755 6277 50  0000 C CNN
F 2 "" H 3750 6450 50  0001 C CNN
F 3 "" H 3750 6450 50  0001 C CNN
	1    3750 6450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0142
U 1 1 621CA432
P 1700 8750
F 0 "#PWR0142" H 1700 8500 50  0001 C CNN
F 1 "GND" H 1705 8577 50  0000 C CNN
F 2 "" H 1700 8750 50  0001 C CNN
F 3 "" H 1700 8750 50  0001 C CNN
	1    1700 8750
	1    0    0    -1  
$EndComp
Text GLabel 13450 1850 0    50   Input ~ 0
UP-MOSI
$Comp
L power:GND #PWR0146
U 1 1 6227B302
P 15050 1050
F 0 "#PWR0146" H 15050 800 50  0001 C CNN
F 1 "GND" H 15055 877 50  0000 C CNN
F 2 "" H 15050 1050 50  0001 C CNN
F 3 "" H 15050 1050 50  0001 C CNN
	1    15050 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	15050 1050 14000 1050
Text GLabel 14000 1850 2    50   Input ~ 0
D11
Text GLabel 13450 1950 0    50   Input ~ 0
UP-D2
Text GLabel 13450 2050 0    50   Input ~ 0
UP-D3
Text GLabel 14000 1950 2    50   Input ~ 0
D2
Text GLabel 14000 2050 2    50   Input ~ 0
D3
Text GLabel 6200 4500 0    50   Input ~ 0
D8
Text GLabel 6200 4600 0    50   Input ~ 0
D9
Text GLabel 14000 2250 2    50   Input ~ 0
D8
Text GLabel 14000 2350 2    50   Input ~ 0
D9
Text GLabel 12500 5250 0    50   Input ~ 0
BAT-12V
Text GLabel 13350 4700 3    50   Input ~ 0
BAT-GND
Text GLabel 13850 4300 2    50   Input ~ 0
5V
$Comp
L power:GND #PWR0101
U 1 1 625D1024
P 13550 4700
F 0 "#PWR0101" H 13550 4450 50  0001 C CNN
F 1 "GND" H 13555 4527 50  0000 C CNN
F 2 "" H 13550 4700 50  0001 C CNN
F 3 "" H 13550 4700 50  0001 C CNN
	1    13550 4700
	1    0    0    -1  
$EndComp
Text GLabel 6850 6650 2    50   Input ~ 0
BAT-GND
Wire Wire Line
	6850 5750 6850 5800
$Comp
L yaaj_dcdc_stepdown_lm2596:YAAJ_DCDC_StepDown_LM2596 U1
U 1 1 626DB2CE
P 13450 4400
F 0 "U1" H 13450 4765 50  0000 C CNN
F 1 "YAAJ_DCDC_StepDown_LM2596" H 13450 4674 50  0000 C CNN
F 2 "DC_DC_LM2596:YAAJ_DCDC_StepDown_LM2596" H 13400 4400 50  0001 C CNN
F 3 "" H 13400 4400 50  0001 C CNN
	1    13450 4400
	1    0    0    -1  
$EndComp
Text GLabel 13450 1250 0    50   Input ~ 0
UP-3.3V
$Comp
L power:GND2 #PWR0143
U 1 1 621D0DCA
P 12750 1050
F 0 "#PWR0143" H 12750 800 50  0001 C CNN
F 1 "GND2" H 12755 877 50  0000 C CNN
F 2 "" H 12750 1050 50  0001 C CNN
F 3 "" H 12750 1050 50  0001 C CNN
	1    12750 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	12750 1050 13450 1050
Text GLabel 13450 1150 0    50   Input ~ 0
UP-5V
Text GLabel 14000 1150 2    50   Input ~ 0
5V
Text GLabel 14000 2150 2    50   Input ~ 0
D7
Text GLabel 13450 2150 0    50   Input ~ 0
UP-D7
Text Notes 13700 5200 0    50   ~ 0
DC-DC Converter\nFrom 12V to 5V
Text GLabel 4200 7350 2    50   Input ~ 0
UP-5V
$Comp
L power:GND2 #PWR0103
U 1 1 627CC670
P 3600 8150
F 0 "#PWR0103" H 3600 7900 50  0001 C CNN
F 1 "GND2" H 3605 7977 50  0000 C CNN
F 2 "" H 3600 8150 50  0001 C CNN
F 3 "" H 3600 8150 50  0001 C CNN
	1    3600 8150
	1    0    0    -1  
$EndComp
Text GLabel 3750 5650 2    50   Input ~ 0
UP-5V
Text GLabel 1850 5550 0    50   Input ~ 0
UP-3.3V
Text GLabel 4900 3400 2    50   Input ~ 0
UP-5V
Text GLabel 10950 4450 2    50   Input ~ 0
BAT-12V
Wire Wire Line
	12900 4300 13050 4300
$Comp
L Diode:1N5822 D1
U 1 1 628FFF25
P 12750 4300
F 0 "D1" H 12750 4083 50  0000 C CNN
F 1 "1N5822" H 12750 4174 50  0000 C CNN
F 2 "Diode_THT:D_DO-201AD_P15.24mm_Horizontal" H 12750 4125 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88526/1n5820.pdf" H 12750 4300 50  0001 C CNN
	1    12750 4300
	-1   0    0    1   
$EndComp
Text Notes 11250 8850 0    50   ~ 0
Low side relay
Text Notes 11300 6450 0    50   ~ 0
HIGH side relay
Wire Wire Line
	3100 4550 3700 4550
Wire Wire Line
	2400 4350 4200 4350
Wire Wire Line
	3450 4250 4200 4250
Wire Wire Line
	3600 4150 4200 4150
Wire Wire Line
	3000 3850 3000 3900
Wire Wire Line
	3000 3600 3000 3550
$Comp
L Device:R R11
U 1 1 61E832B6
P 3000 3700
F 0 "R11" H 2930 3654 50  0000 R CNN
F 1 "10K" H 2930 3745 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2930 3700 50  0001 C CNN
F 3 "~" H 3000 3700 50  0001 C CNN
	1    3000 3700
	-1   0    0    1   
$EndComp
Wire Wire Line
	1750 3550 3000 3550
Text Notes 1800 3550 0    50   ~ 0
3.65V
Text Notes 1800 3900 0    50   ~ 0
7.3V
Text Notes 1800 4250 0    50   ~ 0
10.95V
Text Notes 1800 4600 0    50   ~ 0
14.6V
Text GLabel 9900 3200 0    50   Input ~ 0
CELL-1
Text GLabel 9900 2450 0    50   Input ~ 0
5V
Text GLabel 2350 7400 2    50   Input ~ 0
3.3V
Text GLabel 2350 7550 2    50   Input ~ 0
HREF
Wire Wire Line
	2200 7400 2250 7400
Wire Wire Line
	2250 7400 2350 7400
Connection ~ 2250 7400
Wire Wire Line
	2350 7550 2250 7550
Wire Wire Line
	2250 7550 2250 7400
Wire Notes Line
	2700 7050 2700 8100
Wire Notes Line
	750  7050 2700 7050
Wire Notes Line
	750  8100 2700 8100
Wire Wire Line
	7500 1000 7700 1000
Text GLabel 6900 950  1    50   Input ~ 0
A6
Text GLabel 7700 1000 2    50   Input ~ 0
5V
$Comp
L Device:R R9
U 1 1 61ADEF58
P 7350 1100
F 0 "R9" V 7143 1100 50  0000 C CNN
F 1 "10K" V 7234 1100 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7280 1100 50  0001 C CNN
F 3 "~" H 7350 1100 50  0001 C CNN
	1    7350 1100
	0    1    1    0   
$EndComp
Text GLabel 7700 1100 2    50   Input ~ 0
5V
Wire Wire Line
	7700 1100 7500 1100
Wire Wire Line
	6400 1100 6900 1100
Wire Wire Line
	6900 1150 6900 1100
Connection ~ 6900 1100
Wire Wire Line
	6900 1100 7200 1100
Wire Wire Line
	6900 950  6900 1000
Connection ~ 6900 1000
Wire Wire Line
	6900 1000 6500 1000
$Comp
L Transistor_BJT:TIP42C Q6
U 1 1 61B22AA6
P 9900 4800
F 0 "Q6" H 10091 4846 50  0000 L CNN
F 1 "TIP42C" H 10091 4755 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 10150 4725 50  0001 L CIN
F 3 "https://www.centralsemi.com/get_document.php?cmp=1&mergetype=pd&mergepath=pd&pdf_id=TIP42.PDF" H 9900 4800 50  0001 L CNN
	1    9900 4800
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:TIP122 Q2
U 1 1 61B38CF1
P 9200 5000
F 0 "Q2" H 9407 5046 50  0000 L CNN
F 1 "TIP122" H 9407 4955 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 9400 4925 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/TIP120-D.PDF" H 9200 5000 50  0001 L CNN
	1    9200 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 4750 9300 4800
Connection ~ 9300 4800
Wire Wire Line
	9300 4450 10000 4450
Wire Wire Line
	10950 5150 10700 5150
Wire Wire Line
	10700 5150 10700 4950
Wire Wire Line
	10700 4650 10700 4450
Wire Wire Line
	10000 4600 10000 4450
Connection ~ 10000 4450
Wire Wire Line
	10000 4450 10700 4450
Wire Wire Line
	10000 5000 10000 5150
Wire Wire Line
	10000 5150 10700 5150
Connection ~ 10700 5150
Wire Wire Line
	10950 4450 10700 4450
Connection ~ 10700 4450
$Comp
L Device:R R12
U 1 1 61BD3D2C
P 8850 6050
F 0 "R12" V 8643 6050 50  0000 C CNN
F 1 "1K" V 8734 6050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8780 6050 50  0001 C CNN
F 3 "~" H 8850 6050 50  0001 C CNN
F 4 "Limit gate current" V 8850 6050 50  0001 C CNN "Field4"
	1    8850 6050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 61BD3D32
P 9300 6250
F 0 "#PWR0104" H 9300 6000 50  0001 C CNN
F 1 "GND" H 9305 6077 50  0000 C CNN
F 2 "" H 9300 6250 50  0001 C CNN
F 3 "" H 9300 6250 50  0001 C CNN
	1    9300 6250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R17
U 1 1 61BD3D39
P 9300 5650
F 0 "R17" H 9370 5696 50  0000 L CNN
F 1 "1K" H 9370 5605 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9230 5650 50  0001 C CNN
F 3 "~" H 9300 5650 50  0001 C CNN
F 4 "Pull down in case pin hi-impedence" H 9300 5650 50  0001 C CNN "Field4"
	1    9300 5650
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N5822 D2
U 1 1 61BD3D3F
P 10700 5850
F 0 "D2" V 10654 5930 50  0000 L CNN
F 1 "1N5822" V 10745 5930 50  0000 L CNN
F 2 "Diode_THT:D_5W_P5.08mm_Vertical_AnodeUp" H 10700 5675 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88526/1n5820.pdf" H 10700 5850 50  0001 C CNN
	1    10700 5850
	0    1    1    0   
$EndComp
Text GLabel 8700 6050 0    50   Input ~ 0
A2
Text GLabel 10950 5500 2    50   Input ~ 0
BAT-12V
$Comp
L Transistor_BJT:TIP42C Q7
U 1 1 61BD3D48
P 9900 5850
F 0 "Q7" H 10091 5896 50  0000 L CNN
F 1 "TIP42C" H 10091 5805 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 10150 5775 50  0001 L CIN
F 3 "https://www.centralsemi.com/get_document.php?cmp=1&mergetype=pd&mergepath=pd&pdf_id=TIP42.PDF" H 9900 5850 50  0001 L CNN
	1    9900 5850
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:TIP122 Q3
U 1 1 61BD3D4E
P 9200 6050
F 0 "Q3" H 9407 6096 50  0000 L CNN
F 1 "TIP122" H 9407 6005 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 9400 5975 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/TIP120-D.PDF" H 9200 6050 50  0001 L CNN
	1    9200 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 5800 9300 5850
Connection ~ 9300 5850
Wire Wire Line
	9300 5500 10000 5500
Wire Wire Line
	10950 6200 10700 6200
Wire Wire Line
	10700 6200 10700 6000
Wire Wire Line
	10700 5700 10700 5500
Wire Wire Line
	10000 5650 10000 5500
Connection ~ 10000 5500
Wire Wire Line
	10000 5500 10700 5500
Wire Wire Line
	10000 6050 10000 6200
Wire Wire Line
	10000 6200 10700 6200
Connection ~ 10700 6200
Wire Wire Line
	10950 5500 10700 5500
Connection ~ 10700 5500
$Comp
L Device:R R20
U 1 1 61A6CA6D
P 8850 5000
F 0 "R20" V 8643 5000 50  0000 C CNN
F 1 "1K" V 8734 5000 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8780 5000 50  0001 C CNN
F 3 "~" H 8850 5000 50  0001 C CNN
F 4 "Limit gate current" V 8850 5000 50  0001 C CNN "Field4"
	1    8850 5000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 61C0D74A
P 9300 7600
F 0 "#PWR0105" H 9300 7350 50  0001 C CNN
F 1 "GND" H 9305 7427 50  0000 C CNN
F 2 "" H 9300 7600 50  0001 C CNN
F 3 "" H 9300 7600 50  0001 C CNN
	1    9300 7600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R18
U 1 1 61C0D751
P 9300 7000
F 0 "R18" H 9370 7046 50  0000 L CNN
F 1 "1K" H 9370 6955 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9230 7000 50  0001 C CNN
F 3 "~" H 9300 7000 50  0001 C CNN
F 4 "Pull down in case pin hi-impedence" H 9300 7000 50  0001 C CNN "Field4"
	1    9300 7000
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N5822 D3
U 1 1 61C0D757
P 10700 7200
F 0 "D3" V 10654 7280 50  0000 L CNN
F 1 "1N5822" V 10745 7280 50  0000 L CNN
F 2 "Diode_THT:D_5W_P5.08mm_Vertical_AnodeUp" H 10700 7025 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88526/1n5820.pdf" H 10700 7200 50  0001 C CNN
	1    10700 7200
	0    1    1    0   
$EndComp
Text GLabel 8700 7400 0    50   Input ~ 0
D4
Text GLabel 10950 6850 2    50   Input ~ 0
BAT-12V
$Comp
L Transistor_BJT:TIP42C Q8
U 1 1 61C0D761
P 9900 7200
F 0 "Q8" H 10091 7246 50  0000 L CNN
F 1 "TIP42C" H 10091 7155 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 10150 7125 50  0001 L CIN
F 3 "https://www.centralsemi.com/get_document.php?cmp=1&mergetype=pd&mergepath=pd&pdf_id=TIP42.PDF" H 9900 7200 50  0001 L CNN
	1    9900 7200
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:TIP122 Q4
U 1 1 61C0D767
P 9200 7400
F 0 "Q4" H 9407 7446 50  0000 L CNN
F 1 "TIP122" H 9407 7355 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 9400 7325 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/TIP120-D.PDF" H 9200 7400 50  0001 L CNN
	1    9200 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 7150 9300 7200
Connection ~ 9300 7200
Wire Wire Line
	9300 6850 10000 6850
Wire Wire Line
	10950 7550 10700 7550
Wire Wire Line
	10700 7550 10700 7350
Wire Wire Line
	10700 7050 10700 6850
Wire Wire Line
	10000 7000 10000 6850
Connection ~ 10000 6850
Wire Wire Line
	10000 6850 10700 6850
Wire Wire Line
	10000 7400 10000 7550
Wire Wire Line
	10000 7550 10700 7550
Connection ~ 10700 7550
Wire Wire Line
	10950 6850 10700 6850
Connection ~ 10700 6850
$Comp
L Device:R R15
U 1 1 61C0D77D
P 8850 8450
F 0 "R15" V 8643 8450 50  0000 C CNN
F 1 "1K" V 8734 8450 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8780 8450 50  0001 C CNN
F 3 "~" H 8850 8450 50  0001 C CNN
F 4 "Limit gate current" V 8850 8450 50  0001 C CNN "Field4"
	1    8850 8450
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 61C0D783
P 9300 8650
F 0 "#PWR0107" H 9300 8400 50  0001 C CNN
F 1 "GND" H 9305 8477 50  0000 C CNN
F 2 "" H 9300 8650 50  0001 C CNN
F 3 "" H 9300 8650 50  0001 C CNN
	1    9300 8650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R19
U 1 1 61C0D78A
P 9300 8050
F 0 "R19" H 9370 8096 50  0000 L CNN
F 1 "1K" H 9370 8005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9230 8050 50  0001 C CNN
F 3 "~" H 9300 8050 50  0001 C CNN
F 4 "Pull down in case pin hi-impedence" H 9300 8050 50  0001 C CNN "Field4"
	1    9300 8050
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N5822 D4
U 1 1 61C0D790
P 10700 8250
F 0 "D4" V 10654 8330 50  0000 L CNN
F 1 "1N5822" V 10745 8330 50  0000 L CNN
F 2 "Diode_THT:D_5W_P5.08mm_Vertical_AnodeUp" H 10700 8075 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88526/1n5820.pdf" H 10700 8250 50  0001 C CNN
	1    10700 8250
	0    1    1    0   
$EndComp
Text GLabel 8700 8450 0    50   Input ~ 0
D5
Text GLabel 10950 7900 2    50   Input ~ 0
BAT-12V
$Comp
L Transistor_BJT:TIP42C Q9
U 1 1 61C0D798
P 9900 8250
F 0 "Q9" H 10091 8296 50  0000 L CNN
F 1 "TIP42C" H 10091 8205 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 10150 8175 50  0001 L CIN
F 3 "https://www.centralsemi.com/get_document.php?cmp=1&mergetype=pd&mergepath=pd&pdf_id=TIP42.PDF" H 9900 8250 50  0001 L CNN
	1    9900 8250
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:TIP122 Q5
U 1 1 61C0D79E
P 9200 8450
F 0 "Q5" H 9407 8496 50  0000 L CNN
F 1 "TIP122" H 9407 8405 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 9400 8375 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/TIP120-D.PDF" H 9200 8450 50  0001 L CNN
	1    9200 8450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 8200 9300 8250
Connection ~ 9300 8250
Wire Wire Line
	9300 7900 10000 7900
Wire Wire Line
	10950 8600 10700 8600
Wire Wire Line
	10700 8600 10700 8400
Wire Wire Line
	10700 8100 10700 7900
Wire Wire Line
	10000 8050 10000 7900
Connection ~ 10000 7900
Wire Wire Line
	10000 7900 10700 7900
Wire Wire Line
	10000 8450 10000 8600
Wire Wire Line
	10000 8600 10700 8600
Connection ~ 10700 8600
Wire Wire Line
	10950 7900 10700 7900
Connection ~ 10700 7900
$Comp
L Device:R R13
U 1 1 61C0D7B4
P 8850 7400
F 0 "R13" V 8643 7400 50  0000 C CNN
F 1 "1K" V 8734 7400 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8780 7400 50  0001 C CNN
F 3 "~" H 8850 7400 50  0001 C CNN
F 4 "Limit gate current" V 8850 7400 50  0001 C CNN "Field4"
	1    8850 7400
	0    1    1    0   
$EndComp
Wire Notes Line
	11900 6550 8450 6550
Wire Notes Line
	8450 4100 11900 4100
Wire Notes Line
	8450 4100 8450 8900
Wire Notes Line
	11900 4100 11900 8900
$Comp
L Device:R R22
U 1 1 61CE2F3C
P 9550 5850
F 0 "R22" V 9343 5850 50  0000 C CNN
F 1 "1K" V 9434 5850 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 5850 50  0001 C CNN
F 3 "~" H 9550 5850 50  0001 C CNN
F 4 "Limit gate current" V 9550 5850 50  0001 C CNN "Field4"
	1    9550 5850
	0    1    1    0   
$EndComp
Wire Wire Line
	9400 5850 9300 5850
$Comp
L Device:R R21
U 1 1 61CE31E1
P 9550 4800
F 0 "R21" V 9343 4800 50  0000 C CNN
F 1 "1K" V 9434 4800 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 4800 50  0001 C CNN
F 3 "~" H 9550 4800 50  0001 C CNN
F 4 "Limit gate current" V 9550 4800 50  0001 C CNN "Field4"
	1    9550 4800
	0    1    1    0   
$EndComp
Wire Wire Line
	9400 4800 9300 4800
$Comp
L Device:R R23
U 1 1 61CE360E
P 9550 7200
F 0 "R23" V 9343 7200 50  0000 C CNN
F 1 "1K" V 9434 7200 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 7200 50  0001 C CNN
F 3 "~" H 9550 7200 50  0001 C CNN
F 4 "Limit gate current" V 9550 7200 50  0001 C CNN "Field4"
	1    9550 7200
	0    1    1    0   
$EndComp
Wire Wire Line
	9400 7200 9300 7200
$Comp
L Device:R R24
U 1 1 61CE3AED
P 9550 8250
F 0 "R24" V 9343 8250 50  0000 C CNN
F 1 "1K" V 9434 8250 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9480 8250 50  0001 C CNN
F 3 "~" H 9550 8250 50  0001 C CNN
F 4 "Limit gate current" V 9550 8250 50  0001 C CNN "Field4"
	1    9550 8250
	0    1    1    0   
$EndComp
Wire Wire Line
	9400 8250 9300 8250
Text GLabel 7200 4100 2    50   Input ~ 0
3.3V
$Comp
L Battery-Controller-rescue:Arduino_Nano_v3.x-MCU_Module Arduino2
U 1 1 61D0DBA8
P 14950 6750
F 0 "Arduino2" H 14950 5661 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 14950 5570 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 15100 5800 50  0001 L CNN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 14950 5750 50  0001 C CNN
	1    14950 6750
	1    0    0    -1  
$EndComp
Connection ~ 4050 7350
Wire Wire Line
	4050 7350 3600 7350
$Comp
L Device:CP C?
U 1 1 61D104DE
P 4050 7500
F 0 "C?" H 4168 7546 50  0000 L CNN
F 1 "1.5uF" H 4168 7455 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D8.0mm_W5.0mm_P7.50mm" H 4088 7350 50  0001 C CNN
F 3 "~" H 4050 7500 50  0001 C CNN
	1    4050 7500
	1    0    0    -1  
$EndComp
$Comp
L power:GND2 #PWR?
U 1 1 61D14DB0
P 4550 7650
F 0 "#PWR?" H 4550 7400 50  0001 C CNN
F 1 "GND2" H 4555 7477 50  0000 C CNN
F 2 "" H 4550 7650 50  0001 C CNN
F 3 "" H 4550 7650 50  0001 C CNN
	1    4550 7650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 7650 4050 7650
Text GLabel 3300 7550 0    50   Input ~ 0
UP-D7
$Comp
L power:GND2 #PWR?
U 1 1 61D19D1C
P 3150 8150
F 0 "#PWR?" H 3150 7900 50  0001 C CNN
F 1 "GND2" H 3155 7977 50  0000 C CNN
F 2 "" H 3150 8150 50  0001 C CNN
F 3 "" H 3150 8150 50  0001 C CNN
	1    3150 8150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 8150 3150 7850
Wire Wire Line
	3150 7850 3300 7850
$EndSCHEMATC
