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
Sheet 2 3
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
L 4051 U12
U 1 1 6147F841
P 4850 3550
F 0 "U12" H 4950 3550 50  0000 C CNN
F 1 "4051" H 4950 3350 50  0000 C CNN
F 2 "Housings_DIP:DIP-16_W7.62mm_Socket" H 4850 3550 60  0001 C CNN
F 3 "" H 4850 3550 60  0001 C CNN
	1    4850 3550
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR77
U 1 1 6147F8AB
P 4850 2850
F 0 "#PWR77" H 4850 2700 50  0001 C CNN
F 1 "+5V" H 4850 2990 50  0000 C CNN
F 2 "" H 4850 2850 50  0001 C CNN
F 3 "" H 4850 2850 50  0001 C CNN
	1    4850 2850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR78
U 1 1 6147F8C9
P 4850 4350
F 0 "#PWR78" H 4850 4100 50  0001 C CNN
F 1 "GND" H 4850 4200 50  0000 C CNN
F 2 "" H 4850 4350 50  0001 C CNN
F 3 "" H 4850 4350 50  0001 C CNN
	1    4850 4350
	1    0    0    -1  
$EndComp
Text GLabel 3950 4150 3    60   Input ~ 0
PHOTSELA
Text GLabel 4050 4150 3    60   Input ~ 0
PHOTSELB
Text GLabel 4150 4150 3    60   Input ~ 0
PHOTSELC
Text GLabel 3500 1050 0    60   Input ~ 0
PHOTSELA
Text GLabel 3500 1150 0    60   Input ~ 0
PHOTSELB
Text GLabel 3500 1250 0    60   Input ~ 0
PHOTSELC
$Comp
L +5V #PWR73
U 1 1 6147FAC5
P 4100 800
F 0 "#PWR73" H 4100 650 50  0001 C CNN
F 1 "+5V" H 4100 940 50  0000 C CNN
F 2 "" H 4100 800 50  0001 C CNN
F 3 "" H 4100 800 50  0001 C CNN
	1    4100 800 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR74
U 1 1 6147FAE8
P 4100 2100
F 0 "#PWR74" H 4100 1850 50  0001 C CNN
F 1 "GND" H 4100 1950 50  0000 C CNN
F 2 "" H 4100 2100 50  0001 C CNN
F 3 "" H 4100 2100 50  0001 C CNN
	1    4100 2100
	1    0    0    -1  
$EndComp
$Comp
L LED ID1
U 1 1 6147FD6A
P 6250 1100
F 0 "ID1" H 6250 1200 50  0000 C CNN
F 1 "940nm" H 6250 1000 50  0000 C CNN
F 2 "LEDs:LED_D5.0mm" H 6250 1100 50  0001 C CNN
F 3 "" H 6250 1100 50  0001 C CNN
	1    6250 1100
	1    0    0    -1  
$EndComp
$Comp
L R R38
U 1 1 6147FDFF
P 5850 1100
F 0 "R38" V 5930 1100 50  0000 C CNN
F 1 "220R" V 5850 1100 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5780 1100 50  0001 C CNN
F 3 "" H 5850 1100 50  0001 C CNN
	1    5850 1100
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR84
U 1 1 6147FF6D
P 6650 1100
F 0 "#PWR84" H 6650 950 50  0001 C CNN
F 1 "+5V" H 6650 1240 50  0000 C CNN
F 2 "" H 6650 1100 50  0001 C CNN
F 3 "" H 6650 1100 50  0001 C CNN
	1    6650 1100
	1    0    0    -1  
$EndComp
$Comp
L LED ID2
U 1 1 614800DE
P 6250 1400
F 0 "ID2" H 6250 1500 50  0000 C CNN
F 1 "940nm" H 6250 1300 50  0000 C CNN
F 2 "LEDs:LED_D5.0mm" H 6250 1400 50  0001 C CNN
F 3 "" H 6250 1400 50  0001 C CNN
	1    6250 1400
	1    0    0    -1  
$EndComp
$Comp
L R R39
U 1 1 61480123
P 5850 1400
F 0 "R39" V 5930 1400 50  0000 C CNN
F 1 "220R" V 5850 1400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5780 1400 50  0001 C CNN
F 3 "" H 5850 1400 50  0001 C CNN
	1    5850 1400
	0    1    1    0   
$EndComp
$Comp
L R R40
U 1 1 614801EB
P 5850 1700
F 0 "R40" V 5930 1700 50  0000 C CNN
F 1 "220R" V 5850 1700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5780 1700 50  0001 C CNN
F 3 "" H 5850 1700 50  0001 C CNN
	1    5850 1700
	0    1    1    0   
$EndComp
$Comp
L LED ID3
U 1 1 614802C8
P 6250 1700
F 0 "ID3" H 6250 1800 50  0000 C CNN
F 1 "940nm" H 6250 1600 50  0000 C CNN
F 2 "LEDs:LED_D5.0mm" H 6250 1700 50  0001 C CNN
F 3 "" H 6250 1700 50  0001 C CNN
	1    6250 1700
	1    0    0    -1  
$EndComp
$Comp
L R R41
U 1 1 61480351
P 5850 2000
F 0 "R41" V 5930 2000 50  0000 C CNN
F 1 "220R" V 5850 2000 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5780 2000 50  0001 C CNN
F 3 "" H 5850 2000 50  0001 C CNN
	1    5850 2000
	0    1    1    0   
$EndComp
$Comp
L LED ID4
U 1 1 614803A9
P 6250 2000
F 0 "ID4" H 6250 2100 50  0000 C CNN
F 1 "940nm" H 6250 1900 50  0000 C CNN
F 2 "LEDs:LED_D5.0mm" H 6250 2000 50  0001 C CNN
F 3 "" H 6250 2000 50  0001 C CNN
	1    6250 2000
	1    0    0    -1  
$EndComp
Text GLabel 3500 1650 0    60   Input ~ 0
PHOTSIG
$Comp
L R R28
U 1 1 61480E30
P 1750 2750
F 0 "R28" V 1830 2750 50  0000 C CNN
F 1 "10k" V 1750 2750 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1680 2750 50  0001 C CNN
F 3 "" H 1750 2750 50  0001 C CNN
	1    1750 2750
	-1   0    0    1   
$EndComp
$Comp
L R R26
U 1 1 61480EB9
P 1550 2750
F 0 "R26" V 1630 2750 50  0000 C CNN
F 1 "10k" V 1550 2750 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1480 2750 50  0001 C CNN
F 3 "" H 1550 2750 50  0001 C CNN
	1    1550 2750
	-1   0    0    1   
$EndComp
$Comp
L R R25
U 1 1 61480F4E
P 1350 2750
F 0 "R25" V 1430 2750 50  0000 C CNN
F 1 "10k" V 1350 2750 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1280 2750 50  0001 C CNN
F 3 "" H 1350 2750 50  0001 C CNN
	1    1350 2750
	-1   0    0    1   
$EndComp
$Comp
L R R23
U 1 1 61480FE7
P 1150 2750
F 0 "R23" V 1230 2750 50  0000 C CNN
F 1 "10k" V 1150 2750 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1080 2750 50  0001 C CNN
F 3 "" H 1150 2750 50  0001 C CNN
	1    1150 2750
	-1   0    0    1   
$EndComp
$Comp
L +5V #PWR58
U 1 1 614812EF
P 1750 2400
F 0 "#PWR58" H 1750 2250 50  0001 C CNN
F 1 "+5V" H 1750 2540 50  0000 C CNN
F 2 "" H 1750 2400 50  0001 C CNN
F 3 "" H 1750 2400 50  0001 C CNN
	1    1750 2400
	1    0    0    -1  
$EndComp
$Comp
L C_Small C18
U 1 1 614813DB
P 2050 900
F 0 "C18" H 2060 970 50  0000 L CNN
F 1 "100 nF" H 2060 820 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 2050 900 50  0001 C CNN
F 3 "" H 2050 900 50  0001 C CNN
	1    2050 900 
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR62
U 1 1 61481486
P 2050 700
F 0 "#PWR62" H 2050 550 50  0001 C CNN
F 1 "+5V" H 2050 840 50  0000 C CNN
F 2 "" H 2050 700 50  0001 C CNN
F 3 "" H 2050 700 50  0001 C CNN
	1    2050 700 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR63
U 1 1 614814CA
P 2050 1150
F 0 "#PWR63" H 2050 900 50  0001 C CNN
F 1 "GND" H 2050 1000 50  0000 C CNN
F 2 "" H 2050 1150 50  0001 C CNN
F 3 "" H 2050 1150 50  0001 C CNN
	1    2050 1150
	1    0    0    -1  
$EndComp
$Comp
L C_Small C19
U 1 1 614815D5
P 2400 900
F 0 "C19" H 2410 970 50  0000 L CNN
F 1 "100 nF" H 2410 820 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 2400 900 50  0001 C CNN
F 3 "" H 2400 900 50  0001 C CNN
	1    2400 900 
	1    0    0    -1  
$EndComp
$Comp
L C_Small C17
U 1 1 6148175D
P 1700 900
F 0 "C17" H 1710 970 50  0000 L CNN
F 1 "100 nF" H 1710 820 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 1700 900 50  0001 C CNN
F 3 "" H 1700 900 50  0001 C CNN
	1    1700 900 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR72
U 1 1 61481D45
P 3800 4350
F 0 "#PWR72" H 3800 4100 50  0001 C CNN
F 1 "GND" H 3800 4200 50  0000 C CNN
F 2 "" H 3800 4350 50  0001 C CNN
F 3 "" H 3800 4350 50  0001 C CNN
	1    3800 4350
	1    0    0    -1  
$EndComp
$Comp
L Q_Photo_NPN_EC PT1
U 1 1 614827A1
P 2100 3950
F 0 "PT1" H 2050 4150 50  0000 L CNN
F 1 "PT333" V 2350 3700 50  0000 L CNN
F 2 "LEDs:LED_D5.0mm" H 2300 4050 50  0001 C CNN
F 3 "" H 2100 3950 50  0001 C CNN
	1    2100 3950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR64
U 1 1 61482BB1
P 2200 4250
F 0 "#PWR64" H 2200 4000 50  0001 C CNN
F 1 "GND" H 2200 4100 50  0000 C CNN
F 2 "" H 2200 4250 50  0001 C CNN
F 3 "" H 2200 4250 50  0001 C CNN
	1    2200 4250
	1    0    0    -1  
$EndComp
$Comp
L Q_Photo_NPN_EC PT2
U 1 1 61482F81
P 1750 3950
F 0 "PT2" H 1700 4150 50  0000 L CNN
F 1 "PT333" V 2000 3700 50  0000 L CNN
F 2 "LEDs:LED_D5.0mm" H 1950 4050 50  0001 C CNN
F 3 "" H 1750 3950 50  0001 C CNN
	1    1750 3950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR59
U 1 1 61482FE8
P 1850 4250
F 0 "#PWR59" H 1850 4000 50  0001 C CNN
F 1 "GND" H 1850 4100 50  0000 C CNN
F 2 "" H 1850 4250 50  0001 C CNN
F 3 "" H 1850 4250 50  0001 C CNN
	1    1850 4250
	1    0    0    -1  
$EndComp
$Comp
L Q_Photo_NPN_EC PT3
U 1 1 61483159
P 1400 3950
F 0 "PT3" H 1350 4150 50  0000 L CNN
F 1 "PT333" V 1650 3700 50  0000 L CNN
F 2 "LEDs:LED_D5.0mm" H 1600 4050 50  0001 C CNN
F 3 "" H 1400 3950 50  0001 C CNN
	1    1400 3950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR55
U 1 1 614831F0
P 1500 4250
F 0 "#PWR55" H 1500 4000 50  0001 C CNN
F 1 "GND" H 1500 4100 50  0000 C CNN
F 2 "" H 1500 4250 50  0001 C CNN
F 3 "" H 1500 4250 50  0001 C CNN
	1    1500 4250
	1    0    0    -1  
$EndComp
$Comp
L Q_Photo_NPN_EC PT4
U 1 1 61483339
P 1050 3950
F 0 "PT4" H 1000 4150 50  0000 L CNN
F 1 "PT333" V 1300 3600 50  0000 L CNN
F 2 "LEDs:LED_D5.0mm" H 1250 4050 50  0001 C CNN
F 3 "" H 1050 3950 50  0001 C CNN
	1    1050 3950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR54
U 1 1 614834E7
P 1150 4250
F 0 "#PWR54" H 1150 4000 50  0001 C CNN
F 1 "GND" H 1150 4100 50  0000 C CNN
F 2 "" H 1150 4250 50  0001 C CNN
F 3 "" H 1150 4250 50  0001 C CNN
	1    1150 4250
	1    0    0    -1  
$EndComp
$Comp
L LM358 U2
U 2 1 61484141
P 6500 2850
F 0 "U2" H 6500 3050 50  0000 L CNN
F 1 "LM358" H 6500 2650 50  0000 L CNN
F 2 "Housings_DIP:DIP-8_W7.62mm_Socket" H 6500 2850 50  0001 C CNN
F 3 "" H 6500 2850 50  0001 C CNN
	2    6500 2850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR83
U 1 1 614843BC
P 6400 3350
F 0 "#PWR83" H 6400 3100 50  0001 C CNN
F 1 "GND" H 6400 3200 50  0000 C CNN
F 2 "" H 6400 3350 50  0001 C CNN
F 3 "" H 6400 3350 50  0001 C CNN
	1    6400 3350
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR82
U 1 1 614844BC
P 6400 2500
F 0 "#PWR82" H 6400 2350 50  0001 C CNN
F 1 "+5V" H 6400 2640 50  0000 C CNN
F 2 "" H 6400 2500 50  0001 C CNN
F 3 "" H 6400 2500 50  0001 C CNN
	1    6400 2500
	1    0    0    -1  
$EndComp
$Comp
L LM358 U2
U 1 1 61484625
P 8100 2750
F 0 "U2" H 8100 2950 50  0000 L CNN
F 1 "LM358" H 8100 2550 50  0000 L CNN
F 2 "Housings_DIP:DIP-8_W7.62mm_Socket" H 8100 2750 50  0001 C CNN
F 3 "" H 8100 2750 50  0001 C CNN
	1    8100 2750
	1    0    0    -1  
$EndComp
$Comp
L R R45
U 1 1 61484D65
P 7300 2850
F 0 "R45" V 7380 2850 50  0000 C CNN
F 1 "1k" V 7300 2850 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7230 2850 50  0001 C CNN
F 3 "" H 7300 2850 50  0001 C CNN
	1    7300 2850
	0    -1   -1   0   
$EndComp
$Comp
L R R47
U 1 1 6148841F
P 8600 3000
F 0 "R47" V 8680 3000 50  0000 C CNN
F 1 "1k" V 8600 3000 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 8530 3000 50  0001 C CNN
F 3 "" H 8600 3000 50  0001 C CNN
	1    8600 3000
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR85
U 1 1 614885D2
P 8600 3200
F 0 "#PWR85" H 8600 2950 50  0001 C CNN
F 1 "GND" H 8600 3050 50  0000 C CNN
F 2 "" H 8600 3200 50  0001 C CNN
F 3 "" H 8600 3200 50  0001 C CNN
	1    8600 3200
	1    0    0    -1  
$EndComp
$Comp
L R R44
U 1 1 614887D5
P 7050 3000
F 0 "R44" V 7130 3000 50  0000 C CNN
F 1 "1k" V 7050 3000 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6980 3000 50  0001 C CNN
F 3 "" H 7050 3000 50  0001 C CNN
	1    7050 3000
	-1   0    0    1   
$EndComp
$Comp
L D D14
U 1 1 6148932D
P 9450 2950
F 0 "D14" H 9450 3050 50  0000 C CNN
F 1 "1N4148" H 9450 2850 50  0000 C CNN
F 2 "Diodes_THT:D_DO-41_SOD81_P7.62mm_Horizontal" H 9450 2950 50  0001 C CNN
F 3 "" H 9450 2950 50  0001 C CNN
	1    9450 2950
	0    1    1    0   
$EndComp
$Comp
L GND #PWR86
U 1 1 61489586
P 9450 3200
F 0 "#PWR86" H 9450 2950 50  0001 C CNN
F 1 "GND" H 9450 3050 50  0000 C CNN
F 2 "" H 9450 3200 50  0001 C CNN
F 3 "" H 9450 3200 50  0001 C CNN
	1    9450 3200
	1    0    0    -1  
$EndComp
$Comp
L D D15
U 1 1 614898CB
P 9700 2750
F 0 "D15" H 9700 2850 50  0000 C CNN
F 1 "1N4148" H 9700 2650 50  0000 C CNN
F 2 "Diodes_THT:D_DO-41_SOD81_P7.62mm_Horizontal" H 9700 2750 50  0001 C CNN
F 3 "" H 9700 2750 50  0001 C CNN
	1    9700 2750
	-1   0    0    1   
$EndComp
$Comp
L C_Small C23
U 1 1 61489AE6
P 9950 3000
F 0 "C23" H 9960 3070 50  0000 L CNN
F 1 "10 nF" V 9850 2850 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 9950 3000 50  0001 C CNN
F 3 "" H 9950 3000 50  0001 C CNN
	1    9950 3000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR87
U 1 1 61489C2F
P 9950 3200
F 0 "#PWR87" H 9950 2950 50  0001 C CNN
F 1 "GND" H 9950 3050 50  0000 C CNN
F 2 "" H 9950 3200 50  0001 C CNN
F 3 "" H 9950 3200 50  0001 C CNN
	1    9950 3200
	1    0    0    -1  
$EndComp
$Comp
L R R48
U 1 1 61489FA9
P 10250 3000
F 0 "R48" V 10330 3000 50  0000 C CNN
F 1 "1M" V 10250 3000 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 10180 3000 50  0001 C CNN
F 3 "" H 10250 3000 50  0001 C CNN
	1    10250 3000
	-1   0    0    1   
$EndComp
$Comp
L C_Small C22
U 1 1 6148A13B
P 9200 2750
F 0 "C22" H 9210 2820 50  0000 L CNN
F 1 "100 nF" H 9210 2670 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 9200 2750 50  0001 C CNN
F 3 "" H 9200 2750 50  0001 C CNN
	1    9200 2750
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR88
U 1 1 6148A4C2
P 10250 3200
F 0 "#PWR88" H 10250 2950 50  0001 C CNN
F 1 "GND" H 10250 3050 50  0000 C CNN
F 2 "" H 10250 3200 50  0001 C CNN
F 3 "" H 10250 3200 50  0001 C CNN
	1    10250 3200
	1    0    0    -1  
$EndComp
Text GLabel 10500 2750 3    60   Input ~ 0
PHOTLEVELOUT
$Comp
L POT RV2
U 1 1 6148B18C
P 8200 3200
F 0 "RV2" V 8300 3300 50  0000 C CNN
F 1 "200k" V 8100 3200 50  0000 C CNN
F 2 "Potentiometers:Potentiometer_Trimmer_Bourns_3296W" H 8200 3200 50  0001 C CNN
F 3 "" H 8200 3200 50  0001 C CNN
	1    8200 3200
	0    1    1    0   
$EndComp
$Comp
L 74LS138 U11
U 1 1 6147F95C
P 4100 1400
F 0 "U11" H 4200 1900 50  0000 C CNN
F 1 "74HC138" H 4250 851 50  0000 C CNN
F 2 "Housings_DIP:DIP-16_W7.62mm_Socket" H 4100 1400 50  0001 C CNN
F 3 "" H 4100 1400 50  0001 C CNN
	1    4100 1400
	1    0    0    -1  
$EndComp
Text GLabel 3500 1550 0    60   Input ~ 0
PHOTSELON
Text GLabel 7650 2650 0    60   Input ~ 0
2V5
$Comp
L CP_Small C21
U 1 1 614ADB50
P 7600 2850
F 0 "C21" H 7610 2920 50  0000 L CNN
F 1 "1 uF" H 7610 2770 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.50mm" H 7600 2850 50  0001 C CNN
F 3 "" H 7600 2850 50  0001 C CNN
	1    7600 2850
	0    -1   -1   0   
$EndComp
Text GLabel 6050 2750 0    60   Input ~ 0
2V5
$Comp
L R R42
U 1 1 614AF930
P 6000 2950
F 0 "R42" V 6080 2950 50  0000 C CNN
F 1 "22k" V 6000 2950 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5930 2950 50  0001 C CNN
F 3 "" H 6000 2950 50  0001 C CNN
	1    6000 2950
	0    -1   -1   0   
$EndComp
$Comp
L R R43
U 1 1 614AFF60
P 6650 3200
F 0 "R43" V 6730 3200 50  0000 C CNN
F 1 "100k" V 6650 3200 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6580 3200 50  0001 C CNN
F 3 "" H 6650 3200 50  0001 C CNN
	1    6650 3200
	0    -1   -1   0   
$EndComp
$Comp
L R R46
U 1 1 614B4537
P 8100 3500
F 0 "R46" V 8180 3500 50  0000 C CNN
F 1 "NC" V 8100 3500 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 8030 3500 50  0001 C CNN
F 3 "" H 8100 3500 50  0001 C CNN
	1    8100 3500
	0    -1   -1   0   
$EndComp
$Comp
L 4051 U13
U 1 1 614BCEB9
P 5150 6550
F 0 "U13" H 5250 6550 50  0000 C CNN
F 1 "4051" H 5250 6350 50  0000 C CNN
F 2 "Housings_DIP:DIP-16_W7.62mm_Socket" H 5150 6550 60  0001 C CNN
F 3 "" H 5150 6550 60  0001 C CNN
	1    5150 6550
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR79
U 1 1 614BCEBF
P 5150 5850
F 0 "#PWR79" H 5150 5700 50  0001 C CNN
F 1 "+5V" H 5150 5990 50  0000 C CNN
F 2 "" H 5150 5850 50  0001 C CNN
F 3 "" H 5150 5850 50  0001 C CNN
	1    5150 5850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR80
U 1 1 614BCEC5
P 5150 7350
F 0 "#PWR80" H 5150 7100 50  0001 C CNN
F 1 "GND" H 5150 7200 50  0000 C CNN
F 2 "" H 5150 7350 50  0001 C CNN
F 3 "" H 5150 7350 50  0001 C CNN
	1    5150 7350
	1    0    0    -1  
$EndComp
Text GLabel 4250 7150 3    60   Input ~ 0
LIGHTSELA
Text GLabel 4350 7150 3    60   Input ~ 0
LIGHTSELB
Text GLabel 4450 7150 3    60   Input ~ 0
LIGHTSELC
$Comp
L R R37
U 1 1 614BCED4
P 3000 6850
F 0 "R37" V 3080 6850 50  0000 C CNN
F 1 "100k" V 3000 6850 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2930 6850 50  0001 C CNN
F 3 "" H 3000 6850 50  0001 C CNN
	1    3000 6850
	-1   0    0    1   
$EndComp
$Comp
L R R36
U 1 1 614BCEDA
P 2650 6850
F 0 "R36" V 2730 6850 50  0000 C CNN
F 1 "100k" V 2650 6850 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2580 6850 50  0001 C CNN
F 3 "" H 2650 6850 50  0001 C CNN
	1    2650 6850
	-1   0    0    1   
$EndComp
$Comp
L R R35
U 1 1 614BCEE0
P 2300 6850
F 0 "R35" V 2380 6850 50  0000 C CNN
F 1 "100k" V 2300 6850 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2230 6850 50  0001 C CNN
F 3 "" H 2300 6850 50  0001 C CNN
	1    2300 6850
	-1   0    0    1   
$EndComp
$Comp
L R R32
U 1 1 614BCEE6
P 1950 6850
F 0 "R32" V 2030 6850 50  0000 C CNN
F 1 "100k" V 1950 6850 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1880 6850 50  0001 C CNN
F 3 "" H 1950 6850 50  0001 C CNN
	1    1950 6850
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR75
U 1 1 614BCF0A
P 4100 7350
F 0 "#PWR75" H 4100 7100 50  0001 C CNN
F 1 "GND" H 4100 7200 50  0000 C CNN
F 2 "" H 4100 7350 50  0001 C CNN
F 3 "" H 4100 7350 50  0001 C CNN
	1    4100 7350
	1    0    0    -1  
$EndComp
$Comp
L Q_Photo_NPN_EC PDT4
U 1 1 614BCF10
P 1850 5300
F 0 "PDT4" H 1750 5500 50  0000 L CNN
F 1 "PT333" V 2100 5050 50  0000 L CNN
F 2 "LEDs:LED_D5.0mm" H 2050 5400 50  0001 C CNN
F 3 "" H 1850 5300 50  0001 C CNN
	1    1850 5300
	1    0    0    -1  
$EndComp
$Comp
L Q_Photo_NPN_EC PDT3
U 1 1 614BCF1C
P 2200 5300
F 0 "PDT3" H 2100 5500 50  0000 L CNN
F 1 "PT333" V 2450 5050 50  0000 L CNN
F 2 "LEDs:LED_D5.0mm" H 2400 5400 50  0001 C CNN
F 3 "" H 2200 5300 50  0001 C CNN
	1    2200 5300
	1    0    0    -1  
$EndComp
$Comp
L Q_Photo_NPN_EC PDT1
U 1 1 614BCF28
P 2900 5300
F 0 "PDT1" H 2800 5500 50  0000 L CNN
F 1 "PT333" V 3150 5050 50  0000 L CNN
F 2 "LEDs:LED_D5.0mm" H 3100 5400 50  0001 C CNN
F 3 "" H 2900 5300 50  0001 C CNN
	1    2900 5300
	1    0    0    -1  
$EndComp
$Comp
L Q_Photo_NPN_EC PDT2
U 1 1 614BCF34
P 2550 5300
F 0 "PDT2" H 2450 5500 50  0000 L CNN
F 1 "PT333" V 2800 4950 50  0000 L CNN
F 2 "LEDs:LED_D5.0mm" H 2750 5400 50  0001 C CNN
F 3 "" H 2550 5300 50  0001 C CNN
	1    2550 5300
	1    0    0    -1  
$EndComp
Text GLabel 6000 5950 2    60   Input ~ 0
LIGHTLEVEL
$Comp
L +5V #PWR60
U 1 1 614C0527
P 1950 4950
F 0 "#PWR60" H 1950 4800 50  0001 C CNN
F 1 "+5V" H 1950 5090 50  0000 C CNN
F 2 "" H 1950 4950 50  0001 C CNN
F 3 "" H 1950 4950 50  0001 C CNN
	1    1950 4950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR68
U 1 1 614C2B73
P 3000 7100
F 0 "#PWR68" H 3000 6850 50  0001 C CNN
F 1 "GND" H 3000 6950 50  0000 C CNN
F 2 "" H 3000 7100 50  0001 C CNN
F 3 "" H 3000 7100 50  0001 C CNN
	1    3000 7100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR66
U 1 1 614C2DE0
P 2650 7100
F 0 "#PWR66" H 2650 6850 50  0001 C CNN
F 1 "GND" H 2650 6950 50  0000 C CNN
F 2 "" H 2650 7100 50  0001 C CNN
F 3 "" H 2650 7100 50  0001 C CNN
	1    2650 7100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR65
U 1 1 614C2FCF
P 2300 7100
F 0 "#PWR65" H 2300 6850 50  0001 C CNN
F 1 "GND" H 2300 6950 50  0000 C CNN
F 2 "" H 2300 7100 50  0001 C CNN
F 3 "" H 2300 7100 50  0001 C CNN
	1    2300 7100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR61
U 1 1 614C31BF
P 1950 7100
F 0 "#PWR61" H 1950 6850 50  0001 C CNN
F 1 "GND" H 1950 6950 50  0000 C CNN
F 2 "" H 1950 7100 50  0001 C CNN
F 3 "" H 1950 7100 50  0001 C CNN
	1    1950 7100
	1    0    0    -1  
$EndComp
$Comp
L LM35 U10
U 1 1 614CA1DF
P 900 6000
F 0 "U10" H 700 5900 60  0000 C CNN
F 1 "LMT85" H 650 6050 60  0000 C CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Wide" H 900 6000 60  0001 C CNN
F 3 "" H 900 6000 60  0001 C CNN
	1    900  6000
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR57
U 1 1 614CA69F
P 1500 6700
F 0 "#PWR57" H 1500 6450 50  0001 C CNN
F 1 "GND" H 1500 6550 50  0000 C CNN
F 2 "" H 1500 6700 50  0001 C CNN
F 3 "" H 1500 6700 50  0001 C CNN
	1    1500 6700
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR56
U 1 1 614CA9A1
P 1500 5900
F 0 "#PWR56" H 1500 5750 50  0001 C CNN
F 1 "+5V" H 1500 6040 50  0000 C CNN
F 2 "" H 1500 5900 50  0001 C CNN
F 3 "" H 1500 5900 50  0001 C CNN
	1    1500 5900
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J12
U 1 1 61486C0E
P 7850 4650
F 0 "J12" H 7850 4750 50  0000 C CNN
F 1 "Conn_01x01" H 7850 4550 50  0000 C CNN
F 2 "ControlBoard:IRPair" H 7850 4650 50  0001 C CNN
F 3 "" H 7850 4650 50  0001 C CNN
	1    7850 4650
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J14
U 1 1 61486E35
P 8500 4650
F 0 "J14" H 8500 4750 50  0000 C CNN
F 1 "Conn_01x01" H 8500 4550 50  0000 C CNN
F 2 "ControlBoard:IRPair" H 8500 4650 50  0001 C CNN
F 3 "" H 8500 4650 50  0001 C CNN
	1    8500 4650
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J15
U 1 1 61486F04
P 8500 5100
F 0 "J15" H 8500 5200 50  0000 C CNN
F 1 "Conn_01x01" H 8500 5000 50  0000 C CNN
F 2 "ControlBoard:IRPair" H 8500 5100 50  0001 C CNN
F 3 "" H 8500 5100 50  0001 C CNN
	1    8500 5100
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J13
U 1 1 61486FD0
P 7850 5100
F 0 "J13" H 7850 5200 50  0000 C CNN
F 1 "Conn_01x01" H 7850 5000 50  0000 C CNN
F 2 "ControlBoard:IRPair" H 7850 5100 50  0001 C CNN
F 3 "" H 7850 5100 50  0001 C CNN
	1    7850 5100
	1    0    0    -1  
$EndComp
$Comp
L C_Small C33
U 1 1 6148A68B
P 1400 900
F 0 "C33" H 1410 970 50  0000 L CNN
F 1 "100 nF" H 1410 820 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 1400 900 50  0001 C CNN
F 3 "" H 1400 900 50  0001 C CNN
	1    1400 900 
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR76
U 1 1 6148B87D
P 4800 850
F 0 "#PWR76" H 4800 700 50  0001 C CNN
F 1 "+5V" H 4800 990 50  0000 C CNN
F 2 "" H 4800 850 50  0001 C CNN
F 3 "" H 4800 850 50  0001 C CNN
	1    4800 850 
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x06 J16
U 1 1 6148CF2D
P 2750 4300
F 0 "J16" H 2750 4600 50  0000 C CNN
F 1 "Conn_01x06" H 2750 3900 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 2750 4300 50  0001 C CNN
F 3 "" H 2750 4300 50  0001 C CNN
	1    2750 4300
	0    -1   1    0   
$EndComp
$Comp
L GND #PWR70
U 1 1 6148D946
P 3450 4450
F 0 "#PWR70" H 3450 4200 50  0001 C CNN
F 1 "GND" H 3450 4300 50  0000 C CNN
F 2 "" H 3450 4450 50  0001 C CNN
F 3 "" H 3450 4450 50  0001 C CNN
	1    3450 4450
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR67
U 1 1 6148DEB8
P 2950 3850
F 0 "#PWR67" H 2950 3700 50  0001 C CNN
F 1 "+5V" H 2950 3990 50  0000 C CNN
F 2 "" H 2950 3850 50  0001 C CNN
F 3 "" H 2950 3850 50  0001 C CNN
	1    2950 3850
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x06 J17
U 1 1 614829A2
P 5150 1150
F 0 "J17" H 5150 1450 50  0000 C CNN
F 1 "Conn_01x06" H 5150 750 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 5150 1150 50  0001 C CNN
F 3 "" H 5150 1150 50  0001 C CNN
	1    5150 1150
	1    0    0    1   
$EndComp
$Comp
L GND #PWR81
U 1 1 61482B42
P 5500 700
F 0 "#PWR81" H 5500 450 50  0001 C CNN
F 1 "GND" H 5500 550 50  0000 C CNN
F 2 "" H 5500 700 50  0001 C CNN
F 3 "" H 5500 700 50  0001 C CNN
	1    5500 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 2850 4850 2900
Wire Wire Line
	4850 4200 4850 4350
Wire Wire Line
	3500 1050 3500 1050
Wire Wire Line
	3500 1150 3500 1150
Wire Wire Line
	3500 1250 3500 1250
Wire Wire Line
	4100 800  4100 950 
Wire Wire Line
	4100 1850 4100 2100
Wire Wire Line
	3500 1550 3500 1550
Wire Wire Line
	5550 4050 5550 4300
Wire Wire Line
	5550 4300 4850 4300
Connection ~ 4850 4300
Wire Wire Line
	6650 1100 6400 1100
Wire Wire Line
	6100 1100 6000 1100
Wire Wire Line
	6650 1100 6650 2000
Wire Wire Line
	6650 1400 6400 1400
Wire Wire Line
	6100 1400 6000 1400
Wire Wire Line
	6650 1700 6400 1700
Connection ~ 6650 1400
Wire Wire Line
	6100 1700 6000 1700
Wire Wire Line
	6650 2000 6400 2000
Connection ~ 6650 1700
Wire Wire Line
	6100 2000 6000 2000
Wire Wire Line
	5550 1100 5700 1100
Wire Wire Line
	4700 1650 5700 1650
Wire Wire Line
	5700 1650 5700 1700
Wire Wire Line
	4700 1750 5650 1750
Wire Wire Line
	5650 2000 5700 2000
Wire Wire Line
	3500 1750 3500 2000
Wire Wire Line
	3500 2000 4100 2000
Connection ~ 4100 2000
Wire Wire Line
	4700 1550 5600 1550
Wire Wire Line
	2850 2950 4150 2950
Wire Wire Line
	2750 3050 4150 3050
Wire Wire Line
	2650 3150 4150 3150
Wire Wire Line
	2550 3250 4150 3250
Wire Wire Line
	1750 3350 4150 3350
Wire Wire Line
	1750 3350 1750 2900
Wire Wire Line
	1550 3450 4150 3450
Wire Wire Line
	1550 3450 1550 2900
Wire Wire Line
	1350 3550 4150 3550
Wire Wire Line
	1350 3550 1350 2900
Wire Wire Line
	1150 3650 4150 3650
Wire Wire Line
	1150 2900 1150 3750
Wire Wire Line
	1150 2500 1750 2500
Wire Wire Line
	1150 2500 1150 2600
Wire Wire Line
	1350 2600 1350 2500
Connection ~ 1350 2500
Wire Wire Line
	1550 2600 1550 2500
Connection ~ 1550 2500
Wire Wire Line
	1750 2400 1750 2600
Connection ~ 1750 2500
Wire Wire Line
	1750 2500 1750 2550
Connection ~ 1750 2550
Wire Wire Line
	2050 1000 2050 1150
Wire Wire Line
	2050 700  2050 800 
Wire Wire Line
	550  750  2400 750 
Connection ~ 2050 750 
Wire Wire Line
	550  1100 2400 1100
Connection ~ 2050 1100
Wire Wire Line
	1700 800  1700 750 
Wire Wire Line
	1700 1000 1700 1100
Wire Wire Line
	4150 4150 4150 4150
Wire Wire Line
	4050 4150 4050 4050
Wire Wire Line
	3950 4150 3950 3950
Wire Wire Line
	4050 4050 4150 4050
Wire Wire Line
	3950 3950 4150 3950
Wire Wire Line
	3800 4350 3800 3850
Wire Wire Line
	3800 3850 4150 3850
Wire Wire Line
	2200 4250 2200 4150
Wire Wire Line
	1850 4250 1850 4150
Wire Wire Line
	1500 4250 1500 4150
Wire Wire Line
	1150 4250 1150 4150
Wire Wire Line
	2400 1100 2400 1000
Wire Wire Line
	2400 750  2400 800 
Wire Wire Line
	8600 2850 8600 2750
Connection ~ 8600 2750
Wire Wire Line
	8600 3200 8600 3150
Wire Wire Line
	5650 1750 5650 2000
Wire Wire Line
	9450 2800 9450 2750
Wire Wire Line
	9300 2750 9550 2750
Wire Wire Line
	9450 3200 9450 3100
Connection ~ 9450 2750
Wire Wire Line
	9950 3200 9950 3100
Wire Wire Line
	9950 2900 9950 2750
Wire Wire Line
	9850 2750 10500 2750
Wire Wire Line
	10250 3200 10250 3150
Wire Wire Line
	10250 2750 10250 2850
Connection ~ 9950 2750
Connection ~ 10250 2750
Wire Wire Line
	6850 2850 6850 3750
Wire Wire Line
	6800 2850 7150 2850
Wire Wire Line
	6400 2500 6400 2550
Wire Wire Line
	6400 3150 6400 3350
Connection ~ 6850 2850
Wire Wire Line
	7050 3300 7050 3150
Connection ~ 7050 2850
Wire Wire Line
	7700 2850 7800 2850
Wire Wire Line
	8400 2750 8400 3500
Wire Wire Line
	8400 3200 8350 3200
Wire Wire Line
	8050 3200 7750 3200
Wire Wire Line
	7750 2850 7750 3500
Connection ~ 7750 2850
Wire Wire Line
	8400 3350 8200 3350
Connection ~ 8400 3200
Wire Wire Line
	8400 2750 8700 2750
Wire Wire Line
	7800 2650 7650 2650
Wire Wire Line
	6050 2750 6200 2750
Wire Wire Line
	5600 2950 5550 2950
Wire Wire Line
	6200 2950 6150 2950
Wire Wire Line
	5850 2950 5800 2950
Wire Wire Line
	6850 3200 6800 3200
Wire Wire Line
	6500 3200 6150 3200
Wire Wire Line
	6150 2950 6150 3750
Wire Wire Line
	6400 3300 7050 3300
Connection ~ 6400 3300
Wire Wire Line
	8400 3500 8250 3500
Connection ~ 8400 3350
Wire Wire Line
	7750 3500 7950 3500
Connection ~ 7750 3200
Wire Wire Line
	5150 5850 5150 5900
Wire Wire Line
	5150 7200 5150 7350
Wire Wire Line
	5850 7050 5850 7300
Wire Wire Line
	5850 7300 5150 7300
Connection ~ 5150 7300
Wire Wire Line
	4450 7150 4450 7150
Wire Wire Line
	4350 7150 4350 7050
Wire Wire Line
	4250 7150 4250 6950
Wire Wire Line
	4350 7050 4450 7050
Wire Wire Line
	4250 6950 4450 6950
Wire Wire Line
	4100 7350 4100 6850
Wire Wire Line
	4100 6850 4450 6850
Wire Wire Line
	3000 5050 3000 5100
Wire Wire Line
	1950 5050 3000 5050
Wire Wire Line
	1950 4950 1950 5100
Wire Wire Line
	2300 5100 2300 5050
Connection ~ 2300 5050
Wire Wire Line
	2650 5100 2650 5050
Connection ~ 2650 5050
Wire Wire Line
	1950 5500 1950 6700
Wire Wire Line
	2300 5500 2300 6700
Wire Wire Line
	2650 5500 2650 6700
Wire Wire Line
	3000 5500 3000 6700
Wire Wire Line
	3000 7100 3000 7000
Wire Wire Line
	2650 7100 2650 7000
Wire Wire Line
	2300 7100 2300 7000
Wire Wire Line
	1950 7100 1950 7000
Wire Wire Line
	1500 6250 4450 6250
Wire Wire Line
	6000 5950 5850 5950
Wire Wire Line
	1500 6400 1500 6700
Wire Wire Line
	1500 5900 1500 6100
Connection ~ 1950 5050
Wire Wire Line
	5600 1550 5600 1400
Wire Wire Line
	5600 1400 5700 1400
Wire Wire Line
	4700 1450 5550 1450
Wire Wire Line
	5550 1450 5550 1100
Wire Wire Line
	2550 3250 2550 4100
Wire Wire Line
	2200 3750 2200 3350
Connection ~ 2200 3350
Wire Wire Line
	1850 3750 1850 3450
Connection ~ 1850 3450
Wire Wire Line
	1500 3750 1500 3550
Connection ~ 1500 3550
Connection ~ 1150 3650
Wire Wire Line
	7500 2850 7450 2850
Wire Wire Line
	1400 800  1400 750 
Connection ~ 1700 750 
Wire Wire Line
	1400 1000 1400 1100
Connection ~ 1700 1100
Wire Wire Line
	4950 1350 4700 1350
Wire Wire Line
	4950 1250 4700 1250
Wire Wire Line
	4950 1150 4700 1150
Wire Wire Line
	4950 1050 4700 1050
Wire Wire Line
	4800 850  4800 950 
Wire Wire Line
	4800 950  4950 950 
Wire Wire Line
	2650 3150 2650 4100
Wire Wire Line
	2750 3050 2750 4100
Wire Wire Line
	2850 2950 2850 4100
Wire Wire Line
	3450 4450 3450 4100
Wire Wire Line
	3450 4100 3050 4100
Wire Wire Line
	2950 3850 2950 4100
Wire Wire Line
	4950 850  4950 600 
Wire Wire Line
	4950 600  5500 600 
Wire Wire Line
	5500 600  5500 700 
Wire Wire Line
	4450 6650 1950 6650
Connection ~ 1950 6650
Wire Wire Line
	4450 6550 2300 6550
Connection ~ 2300 6550
Wire Wire Line
	2650 6450 4450 6450
Connection ~ 2650 6450
Wire Wire Line
	4450 6350 3000 6350
Connection ~ 3000 6350
$Comp
L Conn_01x06 J18
U 1 1 61484592
P 3950 5200
F 0 "J18" H 3950 5500 50  0000 C CNN
F 1 "Conn_01x06" H 3950 4800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 3950 5200 50  0001 C CNN
F 3 "" H 3950 5200 50  0001 C CNN
	1    3950 5200
	0    1    -1   0   
$EndComp
Wire Wire Line
	4150 5400 4150 5950
Wire Wire Line
	4150 5950 4450 5950
Wire Wire Line
	4050 5400 4050 6050
Wire Wire Line
	4050 6050 4450 6050
Wire Wire Line
	3950 5400 3950 6150
Wire Wire Line
	3950 6150 4450 6150
Wire Wire Line
	3850 6250 3850 5400
$Comp
L GND #PWR71
U 1 1 61484CD9
P 3650 5800
F 0 "#PWR71" H 3650 5550 50  0001 C CNN
F 1 "GND" H 3650 5650 50  0000 C CNN
F 2 "" H 3650 5800 50  0001 C CNN
F 3 "" H 3650 5800 50  0001 C CNN
	1    3650 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 5800 3650 5400
$Comp
L +5V #PWR69
U 1 1 61484F3C
P 3400 5100
F 0 "#PWR69" H 3400 4950 50  0001 C CNN
F 1 "+5V" H 3400 5240 50  0000 C CNN
F 2 "" H 3400 5100 50  0001 C CNN
F 3 "" H 3400 5100 50  0001 C CNN
	1    3400 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 5400 3750 5500
Wire Wire Line
	3750 5500 3400 5500
Wire Wire Line
	3400 5500 3400 5100
Connection ~ 3850 6250
$Comp
L C_Small C35
U 1 1 6148E685
P 1100 900
F 0 "C35" H 1110 970 50  0000 L CNN
F 1 "100 nF" H 1110 820 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 1100 900 50  0001 C CNN
F 3 "" H 1100 900 50  0001 C CNN
	1    1100 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 750  1100 800 
Connection ~ 1400 750 
Wire Wire Line
	1100 1000 1100 1100
Connection ~ 1400 1100
$Comp
L C_Small C34
U 1 1 6148E954
P 800 900
F 0 "C34" H 810 970 50  0000 L CNN
F 1 "100 nF" H 810 820 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 800 900 50  0001 C CNN
F 3 "" H 800 900 50  0001 C CNN
	1    800  900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	800  750  800  800 
Connection ~ 1100 750 
Wire Wire Line
	800  1000 800  1100
Connection ~ 1100 1100
$Comp
L C_Small C36
U 1 1 6148EF50
P 550 900
F 0 "C36" H 560 970 50  0000 L CNN
F 1 "100 nF" H 560 820 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 550 900 50  0001 C CNN
F 3 "" H 550 900 50  0001 C CNN
	1    550  900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	550  800  550  750 
Connection ~ 800  750 
Wire Wire Line
	550  1000 550  1100
Connection ~ 800  1100
$Comp
L R R24
U 1 1 61494289
P 8850 2750
F 0 "R24" V 8930 2750 50  0000 C CNN
F 1 "100R" V 8850 2750 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 8780 2750 50  0001 C CNN
F 3 "" H 8850 2750 50  0001 C CNN
	1    8850 2750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9100 2750 9000 2750
$Comp
L C_Small C20
U 1 1 61721EF4
P 5700 2950
F 0 "C20" H 5710 3020 50  0000 L CNN
F 1 "100 nF" H 5710 2870 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 5700 2950 50  0001 C CNN
F 3 "" H 5700 2950 50  0001 C CNN
	1    5700 2950
	0    1    1    0   
$EndComp
$Comp
L C_Small C48
U 1 1 6197E1B3
P 6500 3750
F 0 "C48" V 6600 3650 50  0000 L CNN
F 1 "470 pF" V 6400 3600 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 6500 3750 50  0001 C CNN
F 3 "" H 6500 3750 50  0001 C CNN
	1    6500 3750
	0    1    1    0   
$EndComp
Wire Wire Line
	6850 3750 6600 3750
Connection ~ 6850 3200
Wire Wire Line
	6150 3750 6400 3750
Connection ~ 6150 3200
$EndSCHEMATC
