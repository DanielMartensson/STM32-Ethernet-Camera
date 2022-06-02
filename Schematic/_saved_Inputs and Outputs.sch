EESchema Schematic File Version 4
LIBS:ETH OPAMP DCMI FSMC CAN ENCODER CS1000-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 2950 2200 1    50   Input ~ 0
INPUT0
$Comp
L Device:R R?
U 1 1 602ACD09
P 2950 2750
AR Path="/60200981/602ACD09" Ref="R?"  Part="1" 
AR Path="/602ACA79/602ACD09" Ref="R?"  Part="1" 
F 0 "R?" H 3020 2796 50  0000 L CNN
F 1 "10K" H 3020 2705 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2880 2750 50  0001 C CNN
F 3 "71-CRCW060310K0JNEAC" H 2950 2750 50  0001 C CNN
	1    2950 2750
	1    0    0    -1  
$EndComp
$Comp
L Diode:BZX84Cxx D?
U 1 1 602ACD17
P 2600 2400
AR Path="/60200981/602ACD17" Ref="D?"  Part="1" 
AR Path="/602ACA79/602ACD17" Ref="D?"  Part="1" 
F 0 "D?" H 2600 2184 50  0000 C CNN
F 1 "BZX84Cxx" H 2600 2275 50  0000 C CNN
F 2 "Diode_SMD:D_SOT-23_ANK" H 2600 2225 50  0001 C CNN
F 3 "https://diotec.com/tl_files/diotec/files/pdf/datasheets/bzx84c2v4.pdf" H 2600 2400 50  0001 C CNN
	1    2600 2400
	-1   0    0    1   
$EndComp
Wire Wire Line
	2950 2600 2950 2400
Wire Wire Line
	2750 2400 2950 2400
$Comp
L power:GND #PWR?
U 1 1 602ACD27
P 2450 2550
AR Path="/60200981/602ACD27" Ref="#PWR?"  Part="1" 
AR Path="/602ACA79/602ACD27" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 2450 2300 50  0001 C CNN
F 1 "GND" H 2455 2377 50  0000 C CNN
F 2 "" H 2450 2550 50  0001 C CNN
F 3 "" H 2450 2550 50  0001 C CNN
	1    2450 2550
	1    0    0    -1  
$EndComp
Connection ~ 2950 2400
Wire Wire Line
	2950 2400 2950 2200
Wire Wire Line
	2450 2550 2450 2400
Wire Wire Line
	2950 2900 2950 3750
Text HLabel 3800 2200 1    50   Input ~ 0
INPUT1
$Comp
L Device:R R?
U 1 1 602ACE5E
P 3800 2750
AR Path="/60200981/602ACE5E" Ref="R?"  Part="1" 
AR Path="/602ACA79/602ACE5E" Ref="R?"  Part="1" 
F 0 "R?" H 3870 2796 50  0000 L CNN
F 1 "10K" H 3870 2705 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3730 2750 50  0001 C CNN
F 3 "71-CRCW060310K0JNEAC" H 3800 2750 50  0001 C CNN
	1    3800 2750
	1    0    0    -1  
$EndComp
$Comp
L Diode:BZX84Cxx D?
U 1 1 602ACE65
P 3450 2400
AR Path="/60200981/602ACE65" Ref="D?"  Part="1" 
AR Path="/602ACA79/602ACE65" Ref="D?"  Part="1" 
F 0 "D?" H 3450 2184 50  0000 C CNN
F 1 "BZX84Cxx" H 3450 2275 50  0000 C CNN
F 2 "Diode_SMD:D_SOT-23_ANK" H 3450 2225 50  0001 C CNN
F 3 "https://diotec.com/tl_files/diotec/files/pdf/datasheets/bzx84c2v4.pdf" H 3450 2400 50  0001 C CNN
	1    3450 2400
	-1   0    0    1   
$EndComp
Wire Wire Line
	3800 2600 3800 2400
Wire Wire Line
	3600 2400 3800 2400
$Comp
L power:GND #PWR?
U 1 1 602ACE6E
P 3300 2550
AR Path="/60200981/602ACE6E" Ref="#PWR?"  Part="1" 
AR Path="/602ACA79/602ACE6E" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 3300 2300 50  0001 C CNN
F 1 "GND" H 3305 2377 50  0000 C CNN
F 2 "" H 3300 2550 50  0001 C CNN
F 3 "" H 3300 2550 50  0001 C CNN
	1    3300 2550
	1    0    0    -1  
$EndComp
Connection ~ 3800 2400
Wire Wire Line
	3800 2400 3800 2200
Wire Wire Line
	3300 2550 3300 2400
Text HLabel 4650 2200 1    50   Input ~ 0
INPUT2
$Comp
L Device:R R?
U 1 1 602ACF16
P 4650 2750
AR Path="/60200981/602ACF16" Ref="R?"  Part="1" 
AR Path="/602ACA79/602ACF16" Ref="R?"  Part="1" 
F 0 "R?" H 4720 2796 50  0000 L CNN
F 1 "10K" H 4720 2705 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4580 2750 50  0001 C CNN
F 3 "71-CRCW060310K0JNEAC" H 4650 2750 50  0001 C CNN
	1    4650 2750
	1    0    0    -1  
$EndComp
$Comp
L Diode:BZX84Cxx D?
U 1 1 602ACF1D
P 4300 2400
AR Path="/60200981/602ACF1D" Ref="D?"  Part="1" 
AR Path="/602ACA79/602ACF1D" Ref="D?"  Part="1" 
F 0 "D?" H 4300 2184 50  0000 C CNN
F 1 "BZX84Cxx" H 4300 2275 50  0000 C CNN
F 2 "Diode_SMD:D_SOT-23_ANK" H 4300 2225 50  0001 C CNN
F 3 "https://diotec.com/tl_files/diotec/files/pdf/datasheets/bzx84c2v4.pdf" H 4300 2400 50  0001 C CNN
	1    4300 2400
	-1   0    0    1   
$EndComp
Wire Wire Line
	4650 2600 4650 2400
Wire Wire Line
	4450 2400 4650 2400
$Comp
L power:GND #PWR?
U 1 1 602ACF26
P 4150 2550
AR Path="/60200981/602ACF26" Ref="#PWR?"  Part="1" 
AR Path="/602ACA79/602ACF26" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4150 2300 50  0001 C CNN
F 1 "GND" H 4155 2377 50  0000 C CNN
F 2 "" H 4150 2550 50  0001 C CNN
F 3 "" H 4150 2550 50  0001 C CNN
	1    4150 2550
	1    0    0    -1  
$EndComp
Connection ~ 4650 2400
Wire Wire Line
	4650 2400 4650 2200
Wire Wire Line
	4150 2550 4150 2400
Wire Wire Line
	4650 2900 4650 3750
$Comp
L Connector:Screw_Terminal_01x03 J?
U 1 1 602AD0F5
P 3800 4200
F 0 "J?" V 3673 4380 50  0000 L CNN
F 1 "Screw_Terminal_01x03" V 3764 4380 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-3_1x03_P5.00mm_Horizontal" H 3800 4200 50  0001 C CNN
F 3 "651-1935174" H 3800 4200 50  0001 C CNN
	1    3800 4200
	0    1    1    0   
$EndComp
Wire Wire Line
	3800 2900 3800 4000
Wire Wire Line
	3700 4000 3700 3750
Wire Wire Line
	3700 3750 2950 3750
Wire Wire Line
	3900 4000 3900 3750
Wire Wire Line
	3900 3750 4650 3750
$EndSCHEMATC
