EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:L298
LIBS:lm35
LIBS:ControlBoard-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 3
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
L Conn_01x02 J6
U 1 1 614A30B2
P 3700 3000
F 0 "J6" H 3700 3100 50  0000 C CNN
F 1 "Ultrasonic Transmitter" H 3700 2800 50  0000 C CNN
F 2 "Connectors_Terminal_Blocks:TerminalBlock_bornier-2_P5.08mm" H 3700 3000 50  0001 C CNN
F 3 "" H 3700 3000 50  0001 C CNN
	1    3700 3000
	1    0    0    -1  
$EndComp
Text GLabel 1350 2200 0    60   Input ~ 0
ULTRATRANS
$Comp
L Conn_01x02 J7
U 1 1 614A32EF
P 1200 4350
F 0 "J7" H 1200 4450 50  0000 C CNN
F 1 "Ultrasonic Receiver" H 1400 4150 50  0000 C CNN
F 2 "Connectors_Terminal_Blocks:TerminalBlock_bornier-2_P5.08mm" H 1200 4350 50  0001 C CNN
F 3 "" H 1200 4350 50  0001 C CNN
	1    1200 4350
	-1   0    0    1   
$EndComp
$Comp
L LM358 U3
U 2 1 614A3340
P 2450 4150
F 0 "U3" H 2450 4350 50  0000 L CNN
F 1 "LM358" H 2450 3950 50  0000 L CNN
F 2 "Housings_DIP:DIP-8_W7.62mm_Socket" H 2450 4150 50  0001 C CNN
F 3 "" H 2450 4150 50  0001 C CNN
	2    2450 4150
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR081
U 1 1 614A336E
P 2350 3750
F 0 "#PWR081" H 2350 3600 50  0001 C CNN
F 1 "+5V" H 2350 3890 50  0000 C CNN
F 2 "" H 2350 3750 50  0001 C CNN
F 3 "" H 2350 3750 50  0001 C CNN
	1    2350 3750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR082
U 1 1 614A3393
P 2350 4800
F 0 "#PWR082" H 2350 4550 50  0001 C CNN
F 1 "GND" H 2350 4650 50  0000 C CNN
F 2 "" H 2350 4800 50  0001 C CNN
F 3 "" H 2350 4800 50  0001 C CNN
	1    2350 4800
	1    0    0    -1  
$EndComp
$Comp
L C_Small C24
U 1 1 614A33BA
P 5800 1800
F 0 "C24" H 5810 1870 50  0000 L CNN
F 1 "100 nF" H 5810 1720 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 5800 1800 50  0001 C CNN
F 3 "" H 5800 1800 50  0001 C CNN
	1    5800 1800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR083
U 1 1 614A352A
P 1400 4450
F 0 "#PWR083" H 1400 4200 50  0001 C CNN
F 1 "GND" H 1400 4300 50  0000 C CNN
F 2 "" H 1400 4450 50  0001 C CNN
F 3 "" H 1400 4450 50  0001 C CNN
	1    1400 4450
	1    0    0    -1  
$EndComp
$Comp
L C_Small C26
U 1 1 614A356A
P 1550 4250
F 0 "C26" V 1700 4200 50  0000 L CNN
F 1 "1 nF" V 1400 4150 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 1550 4250 50  0001 C CNN
F 3 "" H 1550 4250 50  0001 C CNN
	1    1550 4250
	0    1    1    0   
$EndComp
$Comp
L R R50
U 1 1 614A3905
P 1900 4250
F 0 "R50" V 1980 4250 50  0000 C CNN
F 1 "10k" V 1900 4250 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1830 4250 50  0001 C CNN
F 3 "" H 1900 4250 50  0001 C CNN
	1    1900 4250
	0    1    1    0   
$EndComp
$Comp
L R R51
U 1 1 614A39E2
P 2600 4500
F 0 "R51" V 2680 4500 50  0000 C CNN
F 1 "220k" V 2600 4500 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2530 4500 50  0001 C CNN
F 3 "" H 2600 4500 50  0001 C CNN
	1    2600 4500
	0    1    1    0   
$EndComp
$Comp
L C_Small C25
U 1 1 614A3AD8
P 6200 1800
F 0 "C25" H 6210 1870 50  0000 L CNN
F 1 "100 nF" H 6210 1720 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 6200 1800 50  0001 C CNN
F 3 "" H 6200 1800 50  0001 C CNN
	1    6200 1800
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR084
U 1 1 614A3B3B
P 6000 1550
F 0 "#PWR084" H 6000 1400 50  0001 C CNN
F 1 "+5V" H 6000 1690 50  0000 C CNN
F 2 "" H 6000 1550 50  0001 C CNN
F 3 "" H 6000 1550 50  0001 C CNN
	1    6000 1550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR085
U 1 1 614A3B80
P 6000 2100
F 0 "#PWR085" H 6000 1850 50  0001 C CNN
F 1 "GND" H 6000 1950 50  0000 C CNN
F 2 "" H 6000 2100 50  0001 C CNN
F 3 "" H 6000 2100 50  0001 C CNN
	1    6000 2100
	1    0    0    -1  
$EndComp
Text GLabel 2000 4050 0    60   Input ~ 0
2V5
$Comp
L LM358 U3
U 1 1 614A40D2
P 4000 4050
F 0 "U3" H 4000 4250 50  0000 L CNN
F 1 "LM358" H 4000 3850 50  0000 L CNN
F 2 "Housings_DIP:DIP-8_W7.62mm_Socket" H 4000 4050 50  0001 C CNN
F 3 "" H 4000 4050 50  0001 C CNN
	1    4000 4050
	1    0    0    -1  
$EndComp
$Comp
L R R52
U 1 1 614A41F6
P 2900 4300
F 0 "R52" V 2980 4300 50  0000 C CNN
F 1 "10k" V 2900 4300 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2830 4300 50  0001 C CNN
F 3 "" H 2900 4300 50  0001 C CNN
	1    2900 4300
	-1   0    0    1   
$EndComp
$Comp
L R R53
U 1 1 614A43D7
P 3150 4150
F 0 "R53" V 3230 4150 50  0000 C CNN
F 1 "10k" V 3150 4150 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3080 4150 50  0001 C CNN
F 3 "" H 3150 4150 50  0001 C CNN
	1    3150 4150
	0    1    1    0   
$EndComp
Text GLabel 3600 3950 0    60   Input ~ 0
2V5
$Comp
L +5V #PWR086
U 1 1 614A4DD3
P 3900 3650
F 0 "#PWR086" H 3900 3500 50  0001 C CNN
F 1 "+5V" H 3900 3790 50  0000 C CNN
F 2 "" H 3900 3650 50  0001 C CNN
F 3 "" H 3900 3650 50  0001 C CNN
	1    3900 3650
	1    0    0    -1  
$EndComp
$Comp
L R R54
U 1 1 614A4E45
P 4050 4450
F 0 "R54" V 4130 4450 50  0000 C CNN
F 1 "220k" V 4050 4450 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3980 4450 50  0001 C CNN
F 3 "" H 4050 4450 50  0001 C CNN
	1    4050 4450
	0    1    1    0   
$EndComp
$Comp
L LM358 U4
U 1 1 614A5029
P 5550 3950
F 0 "U4" H 5550 4150 50  0000 L CNN
F 1 "LM358" H 5550 3750 50  0000 L CNN
F 2 "Housings_DIP:DIP-8_W7.62mm_Socket" H 5550 3950 50  0001 C CNN
F 3 "" H 5550 3950 50  0001 C CNN
	1    5550 3950
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR087
U 1 1 614A515C
P 5450 3550
F 0 "#PWR087" H 5450 3400 50  0001 C CNN
F 1 "+5V" H 5450 3690 50  0000 C CNN
F 2 "" H 5450 3550 50  0001 C CNN
F 3 "" H 5450 3550 50  0001 C CNN
	1    5450 3550
	1    0    0    -1  
$EndComp
$Comp
L C_Small C27
U 1 1 614A4340
P 3500 4150
F 0 "C27" V 3650 4100 50  0000 L CNN
F 1 "1 nF" V 3400 4050 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 3500 4150 50  0001 C CNN
F 3 "" H 3500 4150 50  0001 C CNN
	1    3500 4150
	0    1    1    0   
$EndComp
$Comp
L C_Small C28
U 1 1 614B0CD2
P 4950 4050
F 0 "C28" V 5100 4000 50  0000 L CNN
F 1 "1 nF" V 4850 3950 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 4950 4050 50  0001 C CNN
F 3 "" H 4950 4050 50  0001 C CNN
	1    4950 4050
	0    1    1    0   
$EndComp
$Comp
L R R56
U 1 1 614B0D64
P 4650 4050
F 0 "R56" V 4730 4050 50  0000 C CNN
F 1 "10k" V 4650 4050 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4580 4050 50  0001 C CNN
F 3 "" H 4650 4050 50  0001 C CNN
	1    4650 4050
	0    1    1    0   
$EndComp
Text GLabel 5100 3850 0    60   Input ~ 0
2V5
$Comp
L POT RV3
U 1 1 614B0EF7
P 5750 4400
F 0 "RV3" V 5575 4400 50  0000 C CNN
F 1 "200k" V 5650 4400 50  0000 C CNN
F 2 "Potentiometers:Potentiometer_Trimmer_Bourns_3296W" H 5750 4400 50  0001 C CNN
F 3 "" H 5750 4400 50  0001 C CNN
	1    5750 4400
	0    1    1    0   
$EndComp
$Comp
L GND #PWR088
U 1 1 614B0FF4
P 5450 4550
F 0 "#PWR088" H 5450 4300 50  0001 C CNN
F 1 "GND" H 5450 4400 50  0000 C CNN
F 2 "" H 5450 4550 50  0001 C CNN
F 3 "" H 5450 4550 50  0001 C CNN
	1    5450 4550
	1    0    0    -1  
$EndComp
$Comp
L R R55
U 1 1 614B1258
P 4450 4250
F 0 "R55" V 4530 4250 50  0000 C CNN
F 1 "10k" V 4450 4250 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4380 4250 50  0001 C CNN
F 3 "" H 4450 4250 50  0001 C CNN
	1    4450 4250
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR089
U 1 1 614B13F0
P 4450 4500
F 0 "#PWR089" H 4450 4250 50  0001 C CNN
F 1 "GND" H 4450 4350 50  0000 C CNN
F 2 "" H 4450 4500 50  0001 C CNN
F 3 "" H 4450 4500 50  0001 C CNN
	1    4450 4500
	1    0    0    -1  
$EndComp
$Comp
L R R58
U 1 1 614B147F
P 6150 4100
F 0 "R58" V 6230 4100 50  0000 C CNN
F 1 "10k" V 6150 4100 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6080 4100 50  0001 C CNN
F 3 "" H 6150 4100 50  0001 C CNN
	1    6150 4100
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR090
U 1 1 614B1531
P 6150 4350
F 0 "#PWR090" H 6150 4100 50  0001 C CNN
F 1 "GND" H 6150 4200 50  0000 C CNN
F 2 "" H 6150 4350 50  0001 C CNN
F 3 "" H 6150 4350 50  0001 C CNN
	1    6150 4350
	1    0    0    -1  
$EndComp
$Comp
L C_Small C29
U 1 1 614B15DA
P 6850 3950
F 0 "C29" H 6860 4020 50  0000 L CNN
F 1 "100 nF" H 6860 3870 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 6850 3950 50  0001 C CNN
F 3 "" H 6850 3950 50  0001 C CNN
	1    6850 3950
	0    -1   -1   0   
$EndComp
$Comp
L D D16
U 1 1 614B16F6
P 7150 4150
F 0 "D16" H 7150 4250 50  0000 C CNN
F 1 "1N4148" H 7150 4050 50  0000 C CNN
F 2 "Diodes_THT:D_DO-41_SOD81_P7.62mm_Horizontal" H 7150 4150 50  0001 C CNN
F 3 "" H 7150 4150 50  0001 C CNN
	1    7150 4150
	0    1    1    0   
$EndComp
$Comp
L GND #PWR091
U 1 1 614B1897
P 7150 4350
F 0 "#PWR091" H 7150 4100 50  0001 C CNN
F 1 "GND" H 7150 4200 50  0000 C CNN
F 2 "" H 7150 4350 50  0001 C CNN
F 3 "" H 7150 4350 50  0001 C CNN
	1    7150 4350
	1    0    0    -1  
$EndComp
$Comp
L D D17
U 1 1 614B1989
P 7400 3950
F 0 "D17" H 7400 4050 50  0000 C CNN
F 1 "1N4148" H 7400 3850 50  0000 C CNN
F 2 "Diodes_THT:D_DO-41_SOD81_P7.62mm_Horizontal" H 7400 3950 50  0001 C CNN
F 3 "" H 7400 3950 50  0001 C CNN
	1    7400 3950
	-1   0    0    1   
$EndComp
$Comp
L C_Small C30
U 1 1 614B1ADF
P 7650 4150
F 0 "C30" V 7800 4100 50  0000 L CNN
F 1 "1 nF" V 7500 4050 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 7650 4150 50  0001 C CNN
F 3 "" H 7650 4150 50  0001 C CNN
	1    7650 4150
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR092
U 1 1 614B1BD3
P 7650 4350
F 0 "#PWR092" H 7650 4100 50  0001 C CNN
F 1 "GND" H 7650 4200 50  0000 C CNN
F 2 "" H 7650 4350 50  0001 C CNN
F 3 "" H 7650 4350 50  0001 C CNN
	1    7650 4350
	1    0    0    -1  
$EndComp
$Comp
L R R59
U 1 1 614B1E0F
P 8050 4150
F 0 "R59" V 8130 4150 50  0000 C CNN
F 1 "100k" V 8050 4150 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7980 4150 50  0001 C CNN
F 3 "" H 8050 4150 50  0001 C CNN
	1    8050 4150
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR093
U 1 1 614B1EB6
P 8050 4800
F 0 "#PWR093" H 8050 4550 50  0001 C CNN
F 1 "GND" H 8050 4650 50  0000 C CNN
F 2 "" H 8050 4800 50  0001 C CNN
F 3 "" H 8050 4800 50  0001 C CNN
	1    8050 4800
	1    0    0    -1  
$EndComp
Text GLabel 7800 3550 0    60   Input ~ 0
ULTRASOUNDOUT
$Comp
L LM358 U4
U 2 1 614B24A8
P 8900 4050
F 0 "U4" H 8900 4250 50  0000 L CNN
F 1 "LM358" H 8900 3850 50  0000 L CNN
F 2 "Housings_DIP:DIP-8_W7.62mm_Socket" H 8900 4050 50  0001 C CNN
F 3 "" H 8900 4050 50  0001 C CNN
	2    8900 4050
	1    0    0    -1  
$EndComp
$Comp
L R R57
U 1 1 614B4B9B
P 5750 4800
F 0 "R57" V 5830 4800 50  0000 C CNN
F 1 "NC" V 5750 4800 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5680 4800 50  0001 C CNN
F 3 "" H 5750 4800 50  0001 C CNN
	1    5750 4800
	0    -1   -1   0   
$EndComp
$Comp
L R R61
U 1 1 614B612A
P 8400 4650
F 0 "R61" V 8480 4650 50  0000 C CNN
F 1 "10k" V 8400 4650 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 8330 4650 50  0001 C CNN
F 3 "" H 8400 4650 50  0001 C CNN
	1    8400 4650
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR094
U 1 1 614B61A2
P 8250 4800
F 0 "#PWR094" H 8250 4550 50  0001 C CNN
F 1 "GND" H 8250 4650 50  0000 C CNN
F 2 "" H 8250 4800 50  0001 C CNN
F 3 "" H 8250 4800 50  0001 C CNN
	1    8250 4800
	1    0    0    -1  
$EndComp
$Comp
L POT RV4
U 1 1 614B631A
P 8950 4650
F 0 "RV4" V 8775 4650 50  0000 C CNN
F 1 "100k" V 8850 4650 50  0000 C CNN
F 2 "Potentiometers:Potentiometer_Trimmer_Bourns_3296W" H 8950 4650 50  0001 C CNN
F 3 "" H 8950 4650 50  0001 C CNN
	1    8950 4650
	0    1    1    0   
$EndComp
$Comp
L 2N3904 Q2
U 1 1 614B6631
P 9950 4050
F 0 "Q2" H 10150 4125 50  0000 L CNN
F 1 "2N3904" H 10150 4050 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Wide" H 10150 3975 50  0001 L CIN
F 3 "" H 9950 4050 50  0001 L CNN
	1    9950 4050
	1    0    0    -1  
$EndComp
$Comp
L R R63
U 1 1 614B673A
P 9550 4050
F 0 "R63" V 9630 4050 50  0000 C CNN
F 1 "1k" V 9550 4050 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 9480 4050 50  0001 C CNN
F 3 "" H 9550 4050 50  0001 C CNN
	1    9550 4050
	0    1    1    0   
$EndComp
$Comp
L GND #PWR095
U 1 1 614B698E
P 10050 4350
F 0 "#PWR095" H 10050 4100 50  0001 C CNN
F 1 "GND" H 10050 4200 50  0000 C CNN
F 2 "" H 10050 4350 50  0001 C CNN
F 3 "" H 10050 4350 50  0001 C CNN
	1    10050 4350
	1    0    0    -1  
$EndComp
$Comp
L R R64
U 1 1 614B6A82
P 10050 3600
F 0 "R64" V 10130 3600 50  0000 C CNN
F 1 "10k" V 10050 3600 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 9980 3600 50  0001 C CNN
F 3 "" H 10050 3600 50  0001 C CNN
	1    10050 3600
	-1   0    0    1   
$EndComp
$Comp
L +5V #PWR096
U 1 1 614B6BC6
P 10050 3350
F 0 "#PWR096" H 10050 3200 50  0001 C CNN
F 1 "+5V" H 10050 3490 50  0000 C CNN
F 2 "" H 10050 3350 50  0001 C CNN
F 3 "" H 10050 3350 50  0001 C CNN
	1    10050 3350
	1    0    0    -1  
$EndComp
$Comp
L R R62
U 1 1 614B6CCE
P 8900 5000
F 0 "R62" V 8980 5000 50  0000 C CNN
F 1 "NC" V 8900 5000 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 8830 5000 50  0001 C CNN
F 3 "" H 8900 5000 50  0001 C CNN
	1    8900 5000
	0    -1   -1   0   
$EndComp
Text GLabel 9700 3750 1    60   Input ~ 0
ULTRASOUNDTHR
$Comp
L R R60
U 1 1 614B7BB8
P 8050 4550
F 0 "R60" V 8130 4550 50  0000 C CNN
F 1 "100k" V 8050 4550 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7980 4550 50  0001 C CNN
F 3 "" H 8050 4550 50  0001 C CNN
	1    8050 4550
	-1   0    0    1   
$EndComp
Text GLabel 6600 1550 0    60   Input ~ 0
2V5
$Comp
L C_Small C37
U 1 1 61490D31
P 6800 1750
F 0 "C37" H 6810 1820 50  0000 L CNN
F 1 "100 nF" H 6810 1670 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 6800 1750 50  0001 C CNN
F 3 "" H 6800 1750 50  0001 C CNN
	1    6800 1750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR097
U 1 1 61490DC0
P 6800 2000
F 0 "#PWR097" H 6800 1750 50  0001 C CNN
F 1 "GND" H 6800 1850 50  0000 C CNN
F 2 "" H 6800 2000 50  0001 C CNN
F 3 "" H 6800 2000 50  0001 C CNN
	1    6800 2000
	1    0    0    -1  
$EndComp
$Comp
L R R27
U 1 1 614939AB
P 6500 3950
F 0 "R27" V 6580 3950 50  0000 C CNN
F 1 "100R" V 6500 3950 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6430 3950 50  0001 C CNN
F 3 "" H 6500 3950 50  0001 C CNN
	1    6500 3950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2350 4450 2350 4800
Wire Wire Line
	1450 4250 1400 4250
Wire Wire Line
	2350 3850 2350 3750
Wire Wire Line
	2050 4250 2150 4250
Wire Wire Line
	1650 4250 1750 4250
Wire Wire Line
	2750 4150 2750 4500
Wire Wire Line
	2450 4500 2100 4500
Wire Wire Line
	2100 4500 2100 4250
Connection ~ 2100 4250
Wire Wire Line
	5800 1900 5800 2000
Wire Wire Line
	5450 2000 6200 2000
Wire Wire Line
	6200 2000 6200 1900
Wire Wire Line
	6200 1650 6200 1700
Wire Wire Line
	5450 1650 6200 1650
Wire Wire Line
	5800 1650 5800 1700
Wire Wire Line
	6000 1550 6000 1650
Connection ~ 6000 1650
Wire Wire Line
	6000 2100 6000 2000
Connection ~ 6000 2000
Wire Wire Line
	2750 4150 3000 4150
Wire Wire Line
	2900 4450 2900 4650
Wire Wire Line
	2900 4650 2350 4650
Connection ~ 2350 4650
Connection ~ 2900 4150
Wire Wire Line
	3600 4150 3700 4150
Wire Wire Line
	3900 3650 3900 3750
Wire Wire Line
	4300 4050 4300 4450
Wire Wire Line
	4300 4450 4200 4450
Wire Wire Line
	3650 4150 3650 4450
Wire Wire Line
	3650 4450 3900 4450
Connection ~ 3650 4150
Wire Wire Line
	5450 3550 5450 3650
Wire Wire Line
	2000 4050 2150 4050
Wire Wire Line
	1400 4450 1400 4350
Wire Wire Line
	3600 3950 3700 3950
Wire Wire Line
	4300 4050 4500 4050
Wire Wire Line
	5050 4050 5250 4050
Wire Wire Line
	5250 3850 5100 3850
Wire Wire Line
	5450 4250 5450 4550
Wire Wire Line
	5850 3950 6350 3950
Wire Wire Line
	5950 3950 5950 4800
Wire Wire Line
	5950 4400 5900 4400
Wire Wire Line
	5750 4550 5600 4550
Wire Wire Line
	5600 4550 5600 4400
Wire Wire Line
	5600 4400 5150 4400
Wire Wire Line
	5150 4050 5150 4800
Connection ~ 5150 4050
Wire Wire Line
	4450 4100 4450 4050
Connection ~ 4450 4050
Wire Wire Line
	4450 4500 4450 4400
Connection ~ 5950 3950
Wire Wire Line
	6150 4350 6150 4250
Connection ~ 6150 3950
Wire Wire Line
	7150 4350 7150 4300
Wire Wire Line
	6950 3950 7250 3950
Wire Wire Line
	7150 3950 7150 4000
Connection ~ 7150 3950
Wire Wire Line
	7650 4350 7650 4250
Wire Wire Line
	7650 3950 7650 4050
Wire Wire Line
	8050 3550 8050 4000
Connection ~ 7650 3950
Wire Wire Line
	7800 3550 8050 3550
Connection ~ 8050 3950
Wire Wire Line
	5950 4800 5900 4800
Connection ~ 5950 4400
Wire Wire Line
	5150 4800 5600 4800
Connection ~ 5150 4400
Wire Wire Line
	8250 4800 8250 4650
Wire Wire Line
	8550 4650 8800 4650
Wire Wire Line
	8600 4150 8600 5000
Connection ~ 8600 4650
Wire Wire Line
	9200 4050 9200 5000
Wire Wire Line
	9200 4650 9100 4650
Wire Wire Line
	8950 4800 8750 4800
Wire Wire Line
	8750 4800 8750 4650
Connection ~ 8750 4650
Wire Wire Line
	9750 4050 9700 4050
Wire Wire Line
	9400 4050 9200 4050
Wire Wire Line
	10050 4350 10050 4250
Wire Wire Line
	10050 3750 10050 3850
Wire Wire Line
	10050 3350 10050 3450
Wire Wire Line
	9200 5000 9050 5000
Connection ~ 9200 4650
Wire Wire Line
	8600 5000 8750 5000
Wire Wire Line
	10050 3800 9700 3800
Wire Wire Line
	9700 3800 9700 3750
Connection ~ 10050 3800
Wire Wire Line
	7550 3950 8050 3950
Wire Wire Line
	8050 4700 8050 4800
Wire Wire Line
	8050 4300 8050 4400
Wire Wire Line
	8050 4350 8300 4350
Wire Wire Line
	8300 4350 8300 3950
Wire Wire Line
	8300 3950 8600 3950
Connection ~ 8050 4350
Wire Wire Line
	6800 2000 6800 1850
Wire Wire Line
	6800 1650 6800 1550
Wire Wire Line
	6800 1550 6600 1550
Wire Wire Line
	3300 4150 3400 4150
Wire Wire Line
	4850 4050 4800 4050
Wire Wire Line
	6750 3950 6650 3950
$Comp
L +5V #PWR098
U 1 1 614B24F3
P 1900 1800
F 0 "#PWR098" H 1900 1650 50  0001 C CNN
F 1 "+5V" H 1900 1940 50  0000 C CNN
F 2 "" H 1900 1800 50  0001 C CNN
F 3 "" H 1900 1800 50  0001 C CNN
	1    1900 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 1800 1900 1950
$Comp
L C_Small C42
U 1 1 614B26B0
P 5450 1800
F 0 "C42" H 5460 1870 50  0000 L CNN
F 1 "100 nF" H 5460 1720 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 5450 1800 50  0001 C CNN
F 3 "" H 5450 1800 50  0001 C CNN
	1    5450 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 1700 5450 1650
Connection ~ 5800 1650
Wire Wire Line
	5450 1900 5450 2000
Connection ~ 5800 2000
$Comp
L GND #PWR099
U 1 1 614B2A2C
P 1900 2250
F 0 "#PWR099" H 1900 2000 50  0001 C CNN
F 1 "GND" H 1900 2100 50  0000 C CNN
F 2 "" H 1900 2250 50  0001 C CNN
F 3 "" H 1900 2250 50  0001 C CNN
	1    1900 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 2150 1900 2250
Wire Wire Line
	1500 2200 1350 2200
Wire Wire Line
	2400 2050 2550 2050
Wire Wire Line
	2500 2600 2600 2600
$Comp
L C_Small C41
U 1 1 614B34BE
P 2650 3100
F 0 "C41" H 2660 3170 50  0000 L CNN
F 1 "100 nF" H 2660 3020 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 2650 3100 50  0001 C CNN
F 3 "" H 2650 3100 50  0001 C CNN
	1    2650 3100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3500 2050 3500 3000
Wire Wire Line
	3500 2050 3450 2050
Connection ~ 1500 2200
Wire Wire Line
	1500 2050 1500 3100
Connection ~ 1500 2600
Wire Wire Line
	2500 2600 2500 2050
Connection ~ 2500 2050
$Comp
L 74HC14 U14
U 6 1 614D1C36
P 2650 950
F 0 "U14" H 2800 1050 50  0000 C CNN
F 1 "74HC14" H 2850 850 50  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket" H 2650 950 50  0001 C CNN
F 3 "" H 2650 950 50  0001 C CNN
	6    2650 950 
	1    0    0    -1  
$EndComp
$Comp
L 74HC14 U14
U 5 1 614D24B3
P 1950 2050
F 0 "U14" H 2100 2150 50  0000 C CNN
F 1 "74HC14" H 2150 1950 50  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket" H 1950 2050 50  0001 C CNN
F 3 "" H 1950 2050 50  0001 C CNN
	5    1950 2050
	1    0    0    -1  
$EndComp
$Comp
L 74HC14 U14
U 4 1 614D2D27
P 3000 2050
F 0 "U14" H 3150 2150 50  0000 C CNN
F 1 "74HC14" H 3200 1950 50  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket" H 3000 2050 50  0001 C CNN
F 3 "" H 3000 2050 50  0001 C CNN
	4    3000 2050
	1    0    0    -1  
$EndComp
$Comp
L 74HC14 U14
U 1 1 614D2E64
P 1950 2600
F 0 "U14" H 2100 2700 50  0000 C CNN
F 1 "74HC14" H 2150 2500 50  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket" H 1950 2600 50  0001 C CNN
F 3 "" H 1950 2600 50  0001 C CNN
	1    1950 2600
	1    0    0    -1  
$EndComp
$Comp
L 74HC14 U14
U 3 1 614D2F99
P 3050 2600
F 0 "U14" H 3200 2700 50  0000 C CNN
F 1 "74HC14" H 3250 2500 50  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket" H 3050 2600 50  0001 C CNN
F 3 "" H 3050 2600 50  0001 C CNN
	3    3050 2600
	1    0    0    -1  
$EndComp
$Comp
L 74HC14 U14
U 2 1 614D30D2
P 1950 3100
F 0 "U14" H 2100 3200 50  0000 C CNN
F 1 "74HC14" H 2150 3000 50  0000 C CNN
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket" H 1950 3100 50  0001 C CNN
F 3 "" H 1950 3100 50  0001 C CNN
	2    1950 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 3100 2550 3100
Wire Wire Line
	2400 3100 2400 2600
Connection ~ 3500 2600
Wire Wire Line
	3500 3100 2750 3100
$Comp
L GND #PWR0100
U 1 1 614D3C7D
P 2100 1450
F 0 "#PWR0100" H 2100 1200 50  0001 C CNN
F 1 "GND" H 2100 1300 50  0000 C CNN
F 2 "" H 2100 1450 50  0001 C CNN
F 3 "" H 2100 1450 50  0001 C CNN
	1    2100 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 1450 2100 1400
Wire Wire Line
	2100 1200 2100 950 
Wire Wire Line
	2100 950  2200 950 
$Comp
L R R33
U 1 1 614D404F
P 2600 1200
F 0 "R33" V 2680 1200 50  0000 C CNN
F 1 "10k" V 2600 1200 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2530 1200 50  0001 C CNN
F 3 "" H 2600 1200 50  0001 C CNN
	1    2600 1200
	0    1    1    0   
$EndComp
Wire Wire Line
	2450 1200 2100 1200
Wire Wire Line
	3100 950  3100 1200
Wire Wire Line
	3100 1200 2750 1200
Text GLabel 3350 950  2    60   Input ~ 0
PHOTSIG
Wire Wire Line
	3350 950  3100 950 
$Comp
L C_Small C43
U 1 1 614D4DC2
P 2100 1300
F 0 "C43" H 2110 1370 50  0000 L CNN
F 1 "100 nF" H 2110 1220 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 2100 1300 50  0001 C CNN
F 3 "" H 2100 1300 50  0001 C CNN
	1    2100 1300
	1    0    0    -1  
$EndComp
$EndSCHEMATC
