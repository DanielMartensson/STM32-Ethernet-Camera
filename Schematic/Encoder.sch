EESchema Schematic File Version 4
LIBS:ETH OPAMP DCMI FSMC CAN ENCODER CS1000-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 7
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:Screw_Terminal_01x03 J8
U 1 1 6020554F
P 3850 3350
F 0 "J8" V 3723 3530 50  0000 L CNN
F 1 "Screw_Terminal_01x03" V 3814 3530 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-3_1x03_P5.00mm_Horizontal" H 3850 3350 50  0001 C CNN
F 3 "651-1935174" H 3850 3350 50  0001 C CNN
	1    3850 3350
	0    1    1    0   
$EndComp
Wire Wire Line
	3850 3150 3850 2800
Wire Wire Line
	3750 3150 3750 2950
Wire Wire Line
	3750 2950 2750 2950
Wire Wire Line
	2750 2950 2750 2800
Wire Wire Line
	3950 3150 3950 2950
Wire Wire Line
	3950 2950 4950 2950
Wire Wire Line
	4950 2950 4950 2800
Wire Wire Line
	4950 2500 4950 2350
Text HLabel 2750 2200 1    50   Input ~ 0
ENCODER0_REVERSE
Text HLabel 3850 2200 1    50   Input ~ 0
ENCODER1_REVERSE
Text HLabel 4950 2200 1    50   Input ~ 0
ENCODER2_REVERSE
Text HLabel 2650 4100 1    50   Input ~ 0
ENCODER0_P
Text HLabel 2750 4100 1    50   Input ~ 0
ENCODER0_M
Text HLabel 4850 4100 1    50   Input ~ 0
ENCODER1_P
Text HLabel 4950 4100 1    50   Input ~ 0
ENCODER1_M
$Comp
L Device:R R26
U 1 1 6022C9BB
P 4850 4650
F 0 "R26" H 4920 4696 50  0000 L CNN
F 1 "10K" H 4920 4605 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4780 4650 50  0001 C CNN
F 3 "71-CRCW060310K0JNEAC" H 4850 4650 50  0001 C CNN
	1    4850 4650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R27
U 1 1 6022C9C2
P 4950 4950
F 0 "R27" H 5020 4996 50  0000 L CNN
F 1 "10K" H 5020 4905 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4880 4950 50  0001 C CNN
F 3 "71-CRCW060310K0JNEAC" H 4950 4950 50  0001 C CNN
	1    4950 4950
	1    0    0    -1  
$EndComp
$Comp
L Diode:BZX84Cxx D4
U 1 1 6022C9C9
P 4500 4300
F 0 "D4" H 4500 4084 50  0000 C CNN
F 1 "BZX84Cxx" H 4500 4175 50  0000 C CNN
F 2 "Diode_SMD:D_SOT-23_ANK" H 4500 4125 50  0001 C CNN
F 3 "863-BZX84C3V6LT1G" H 4500 4300 50  0001 C CNN
	1    4500 4300
	-1   0    0    1   
$EndComp
$Comp
L Diode:BZX84Cxx D5
U 1 1 6022C9D0
P 5300 4300
F 0 "D5" H 5300 4084 50  0000 C CNN
F 1 "BZX84Cxx" H 5300 4175 50  0000 C CNN
F 2 "Diode_SMD:D_SOT-23_ANK" H 5300 4125 50  0001 C CNN
F 3 "863-BZX84C3V6LT1G" H 5300 4300 50  0001 C CNN
	1    5300 4300
	1    0    0    1   
$EndComp
Wire Wire Line
	4850 4500 4850 4300
Wire Wire Line
	4650 4300 4850 4300
$Comp
L power:GND #PWR044
U 1 1 6022C9E1
P 4350 4450
F 0 "#PWR044" H 4350 4200 50  0001 C CNN
F 1 "GND" H 4355 4277 50  0000 C CNN
F 2 "" H 4350 4450 50  0001 C CNN
F 3 "" H 4350 4450 50  0001 C CNN
	1    4350 4450
	1    0    0    -1  
$EndComp
Text HLabel 7100 4050 1    50   Input ~ 0
ENCODER2_P
Text HLabel 7200 4050 1    50   Input ~ 0
ENCODER2_M
$Comp
L Connector:Screw_Terminal_01x06 J9
U 1 1 60240501
P 4950 6000
F 0 "J9" V 4823 6280 50  0000 L CNN
F 1 "Screw_Terminal_01x06" V 4914 6280 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-6_1x06_P5.00mm_Horizontal" H 4950 6000 50  0001 C CNN
F 3 "651-1935200" H 4950 6000 50  0001 C CNN
	1    4950 6000
	0    1    1    0   
$EndComp
Wire Wire Line
	4650 5800 4650 5700
Wire Wire Line
	4650 5700 2650 5700
Wire Wire Line
	2750 5600 4750 5600
Wire Wire Line
	4750 5600 4750 5800
Wire Wire Line
	5150 5800 5150 5700
Wire Wire Line
	5150 5700 7200 5700
Wire Wire Line
	7100 5600 5050 5600
Wire Wire Line
	5050 5600 5050 5800
Connection ~ 4850 4300
Wire Wire Line
	4850 4300 4850 4100
Wire Wire Line
	4850 4800 4850 5800
Wire Wire Line
	4950 5100 4950 5800
Wire Wire Line
	4950 4100 4950 4300
Wire Wire Line
	4350 4450 4350 4300
$Comp
L power:GND #PWR046
U 1 1 6025270B
P 5450 4450
F 0 "#PWR046" H 5450 4200 50  0001 C CNN
F 1 "GND" H 5455 4277 50  0000 C CNN
F 2 "" H 5450 4450 50  0001 C CNN
F 3 "" H 5450 4450 50  0001 C CNN
	1    5450 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 4450 5450 4300
Wire Wire Line
	5150 4300 4950 4300
Connection ~ 4950 4300
Wire Wire Line
	4950 4300 4950 4800
$Comp
L Device:R R28
U 1 1 60257BFB
P 7100 4600
F 0 "R28" H 7170 4646 50  0000 L CNN
F 1 "10K" H 7170 4555 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7030 4600 50  0001 C CNN
F 3 "71-CRCW060310K0JNEAC" H 7100 4600 50  0001 C CNN
	1    7100 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R29
U 1 1 60257C02
P 7200 4900
F 0 "R29" H 7270 4946 50  0000 L CNN
F 1 "10K" H 7270 4855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7130 4900 50  0001 C CNN
F 3 "71-CRCW060310K0JNEAC" H 7200 4900 50  0001 C CNN
	1    7200 4900
	1    0    0    -1  
$EndComp
$Comp
L Diode:BZX84Cxx D6
U 1 1 60257C09
P 6750 4250
F 0 "D6" H 6750 4034 50  0000 C CNN
F 1 "BZX84Cxx" H 6750 4125 50  0000 C CNN
F 2 "Diode_SMD:D_SOT-23_ANK" H 6750 4075 50  0001 C CNN
F 3 "863-BZX84C3V6LT1G" H 6750 4250 50  0001 C CNN
	1    6750 4250
	-1   0    0    1   
$EndComp
$Comp
L Diode:BZX84Cxx D7
U 1 1 60257C10
P 7550 4250
F 0 "D7" H 7550 4034 50  0000 C CNN
F 1 "BZX84Cxx" H 7550 4125 50  0000 C CNN
F 2 "Diode_SMD:D_SOT-23_ANK" H 7550 4075 50  0001 C CNN
F 3 "863-BZX84C3V6LT1G" H 7550 4250 50  0001 C CNN
	1    7550 4250
	1    0    0    1   
$EndComp
Wire Wire Line
	7100 4450 7100 4250
Wire Wire Line
	6900 4250 7100 4250
$Comp
L power:GND #PWR047
U 1 1 60257C19
P 6600 4400
F 0 "#PWR047" H 6600 4150 50  0001 C CNN
F 1 "GND" H 6605 4227 50  0000 C CNN
F 2 "" H 6600 4400 50  0001 C CNN
F 3 "" H 6600 4400 50  0001 C CNN
	1    6600 4400
	1    0    0    -1  
$EndComp
Connection ~ 7100 4250
Wire Wire Line
	7100 4250 7100 4050
Wire Wire Line
	7200 4050 7200 4250
Wire Wire Line
	6600 4400 6600 4250
$Comp
L power:GND #PWR048
U 1 1 60257C25
P 7700 4400
F 0 "#PWR048" H 7700 4150 50  0001 C CNN
F 1 "GND" H 7705 4227 50  0000 C CNN
F 2 "" H 7700 4400 50  0001 C CNN
F 3 "" H 7700 4400 50  0001 C CNN
	1    7700 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 4400 7700 4250
Wire Wire Line
	7400 4250 7200 4250
Connection ~ 7200 4250
Wire Wire Line
	7200 4250 7200 4750
Wire Wire Line
	7100 4750 7100 5600
Wire Wire Line
	7200 5700 7200 5050
$Comp
L Device:R R22
U 1 1 6025B5A2
P 2650 4650
F 0 "R22" H 2720 4696 50  0000 L CNN
F 1 "10K" H 2720 4605 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2580 4650 50  0001 C CNN
F 3 "71-CRCW060310K0JNEAC" H 2650 4650 50  0001 C CNN
	1    2650 4650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R23
U 1 1 6025B5A9
P 2750 4950
F 0 "R23" H 2820 4996 50  0000 L CNN
F 1 "10K" H 2820 4905 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2680 4950 50  0001 C CNN
F 3 "71-CRCW060310K0JNEAC" H 2750 4950 50  0001 C CNN
	1    2750 4950
	1    0    0    -1  
$EndComp
$Comp
L Diode:BZX84Cxx D2
U 1 1 6025B5B0
P 2300 4300
F 0 "D2" H 2300 4084 50  0000 C CNN
F 1 "BZX84Cxx" H 2300 4175 50  0000 C CNN
F 2 "Diode_SMD:D_SOT-23_ANK" H 2300 4125 50  0001 C CNN
F 3 "863-BZX84C3V6LT1G" H 2300 4300 50  0001 C CNN
	1    2300 4300
	-1   0    0    1   
$EndComp
$Comp
L Diode:BZX84Cxx D3
U 1 1 6025B5B7
P 3100 4300
F 0 "D3" H 3100 4084 50  0000 C CNN
F 1 "BZX84Cxx" H 3100 4175 50  0000 C CNN
F 2 "Diode_SMD:D_SOT-23_ANK" H 3100 4125 50  0001 C CNN
F 3 "863-BZX84C3V6LT1G" H 3100 4300 50  0001 C CNN
	1    3100 4300
	1    0    0    1   
$EndComp
Wire Wire Line
	2650 4500 2650 4300
Wire Wire Line
	2450 4300 2650 4300
$Comp
L power:GND #PWR040
U 1 1 6025B5C0
P 2150 4450
F 0 "#PWR040" H 2150 4200 50  0001 C CNN
F 1 "GND" H 2155 4277 50  0000 C CNN
F 2 "" H 2150 4450 50  0001 C CNN
F 3 "" H 2150 4450 50  0001 C CNN
	1    2150 4450
	1    0    0    -1  
$EndComp
Connection ~ 2650 4300
Wire Wire Line
	2650 4300 2650 4100
Wire Wire Line
	2150 4450 2150 4300
$Comp
L power:GND #PWR042
U 1 1 6025B5CC
P 3250 4450
F 0 "#PWR042" H 3250 4200 50  0001 C CNN
F 1 "GND" H 3255 4277 50  0000 C CNN
F 2 "" H 3250 4450 50  0001 C CNN
F 3 "" H 3250 4450 50  0001 C CNN
	1    3250 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 4450 3250 4300
Wire Wire Line
	2950 4300 2750 4300
Connection ~ 2750 4300
Wire Wire Line
	2750 4300 2750 4800
Wire Wire Line
	2750 4100 2750 4300
Wire Wire Line
	2650 4800 2650 5700
Wire Wire Line
	2750 5600 2750 5100
$Comp
L Diode:BZX84Cxx D16
U 1 1 602458F4
P 5400 2500
F 0 "D16" H 5400 2284 50  0000 C CNN
F 1 "BZX84Cxx" H 5400 2375 50  0000 C CNN
F 2 "Diode_SMD:D_SOT-23_ANK" H 5400 2325 50  0001 C CNN
F 3 "863-BZX84C3V6LT1G" H 5400 2500 50  0001 C CNN
	1    5400 2500
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR045
U 1 1 602458FB
P 5400 2700
F 0 "#PWR045" H 5400 2450 50  0001 C CNN
F 1 "GND" H 5405 2527 50  0000 C CNN
F 2 "" H 5400 2700 50  0001 C CNN
F 3 "" H 5400 2700 50  0001 C CNN
	1    5400 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 2350 4950 2350
Connection ~ 4950 2350
Wire Wire Line
	4950 2350 4950 2200
Wire Wire Line
	3850 2500 3850 2350
$Comp
L Diode:BZX84Cxx D15
U 1 1 6024DEC2
P 4300 2500
F 0 "D15" H 4300 2284 50  0000 C CNN
F 1 "BZX84Cxx" H 4300 2375 50  0000 C CNN
F 2 "Diode_SMD:D_SOT-23_ANK" H 4300 2325 50  0001 C CNN
F 3 "863-BZX84C3V6LT1G" H 4300 2500 50  0001 C CNN
	1    4300 2500
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR043
U 1 1 6024DEC9
P 4300 2700
F 0 "#PWR043" H 4300 2450 50  0001 C CNN
F 1 "GND" H 4305 2527 50  0000 C CNN
F 2 "" H 4300 2700 50  0001 C CNN
F 3 "" H 4300 2700 50  0001 C CNN
	1    4300 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 2350 3850 2350
Connection ~ 3850 2350
Wire Wire Line
	3850 2350 3850 2200
Wire Wire Line
	2750 2500 2750 2350
$Comp
L Diode:BZX84Cxx D14
U 1 1 6025163C
P 3200 2500
F 0 "D14" H 3200 2284 50  0000 C CNN
F 1 "BZX84Cxx" H 3200 2375 50  0000 C CNN
F 2 "Diode_SMD:D_SOT-23_ANK" H 3200 2325 50  0001 C CNN
F 3 "863-BZX84C3V6LT1G" H 3200 2500 50  0001 C CNN
	1    3200 2500
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR041
U 1 1 60251643
P 3200 2650
F 0 "#PWR041" H 3200 2400 50  0001 C CNN
F 1 "GND" H 3205 2477 50  0000 C CNN
F 2 "" H 3200 2650 50  0001 C CNN
F 3 "" H 3200 2650 50  0001 C CNN
	1    3200 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 2350 2750 2350
Connection ~ 2750 2350
Wire Wire Line
	2750 2350 2750 2200
$Comp
L Device:D_Schottky D?
U 1 1 601D1596
P 4950 2650
AR Path="/602C59FF/601D1596" Ref="D?"  Part="1" 
AR Path="/60200981/601D1596" Ref="D28"  Part="1" 
F 0 "D28" H 4950 2434 50  0000 C CNN
F 1 "D_Schottky" H 4950 2525 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123" H 4950 2650 50  0001 C CNN
F 3 "863-BAT54T1G" H 4950 2650 50  0001 C CNN
	1    4950 2650
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Schottky D?
U 1 1 601D620C
P 3850 2650
AR Path="/602C59FF/601D620C" Ref="D?"  Part="1" 
AR Path="/60200981/601D620C" Ref="D27"  Part="1" 
F 0 "D27" H 3850 2434 50  0000 C CNN
F 1 "D_Schottky" H 3850 2525 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123" H 3850 2650 50  0001 C CNN
F 3 "863-BAT54T1G" H 3850 2650 50  0001 C CNN
	1    3850 2650
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Schottky D?
U 1 1 601D7158
P 2750 2650
AR Path="/602C59FF/601D7158" Ref="D?"  Part="1" 
AR Path="/60200981/601D7158" Ref="D26"  Part="1" 
F 0 "D26" H 2750 2434 50  0000 C CNN
F 1 "D_Schottky" H 2750 2525 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123" H 2750 2650 50  0001 C CNN
F 3 "863-BAT54T1G" H 2750 2650 50  0001 C CNN
	1    2750 2650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4300 2650 4300 2700
Wire Wire Line
	5400 2700 5400 2650
$EndSCHEMATC
