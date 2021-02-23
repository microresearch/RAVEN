EESchema Schematic File Version 4
LIBS:raven_kicad-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 6 6
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 1460 900  0    50   ~ 0
INS\n
Text GLabel 980  1745 0    50   Input ~ 0
IO_INL
Text Notes 4365 625  0    50   ~ 0
OUTS\n
Wire Wire Line
	1925 1645 1950 1645
Wire Wire Line
	1925 2220 1925 1645
Wire Wire Line
	1600 2220 1925 2220
Wire Wire Line
	1255 1745 1295 1745
Wire Wire Line
	1255 2220 1400 2220
Wire Wire Line
	1255 1745 1255 2220
Connection ~ 1925 1645
Wire Wire Line
	1895 1645 1925 1645
Wire Wire Line
	1495 1330 1495 1395
Wire Wire Line
	1260 1545 1295 1545
Connection ~ 1255 1745
Wire Wire Line
	1220 1745 1255 1745
$Comp
L power:GND #PWR0100
U 1 1 603BD83A
P 1260 1545
F 0 "#PWR0100" H 1260 1295 50  0001 C CNN
F 1 "GND" H 1110 1620 50  0000 C CNN
F 2 "" H 1260 1545 50  0001 C CNN
F 3 "" H 1260 1545 50  0001 C CNN
	1    1260 1545
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R35
U 1 1 603BD633
P 2050 1645
F 0 "R35" V 2215 1665 50  0000 C CNN
F 1 "5.6K" V 1980 1640 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 2050 1645 50  0001 C CNN
F 3 "~" H 2050 1645 50  0001 C CNN
	1    2050 1645
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R33
U 1 1 603BD3F7
P 1500 2220
F 0 "R33" V 1665 2240 50  0000 C CNN
F 1 "33K" V 1575 2225 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 1500 2220 50  0001 C CNN
F 3 "~" H 1500 2220 50  0001 C CNN
	1    1500 2220
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R31
U 1 1 603BD2D0
P 1120 1745
F 0 "R31" V 924 1745 50  0000 C CNN
F 1 "27K" V 1015 1745 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 1120 1745 50  0001 C CNN
F 3 "~" H 1120 1745 50  0001 C CNN
	1    1120 1745
	0    1    1    0   
$EndComp
$Comp
L power:-12V #PWR0102
U 1 1 603BD280
P 1495 1955
F 0 "#PWR0102" H 1495 2055 50  0001 C CNN
F 1 "-12V" H 1510 2128 50  0000 C CNN
F 2 "" H 1495 1955 50  0001 C CNN
F 3 "" H 1495 1955 50  0001 C CNN
	1    1495 1955
	-1   0    0    1   
$EndComp
$Comp
L power:+12V #PWR0101
U 1 1 603BD1E3
P 1495 1330
F 0 "#PWR0101" H 1495 1180 50  0001 C CNN
F 1 "+12V" H 1510 1503 50  0000 C CNN
F 2 "" H 1495 1330 50  0001 C CNN
F 3 "" H 1495 1330 50  0001 C CNN
	1    1495 1330
	1    0    0    -1  
$EndComp
$Comp
L 4ms-ic:TL082 U8
U 2 1 603BCA2A
P 1585 3245
F 0 "U8" H 1815 3385 50  0000 L CNN
F 1 "LME49720" H 1575 3070 50  0000 L CNN
F 2 "SMD_Packages:SOIC-8-N" H 1585 3245 50  0001 C CNN
F 3 "" H 1585 3245 50  0001 C CNN
	2    1585 3245
	1    0    0    -1  
$EndComp
$Comp
L 4ms-ic:TL082 U9
U 1 1 603BDEFA
P 4875 1510
F 0 "U9" H 5105 1650 50  0000 L CNN
F 1 "LME49720" H 4865 1335 50  0000 L CNN
F 2 "SMD_Packages:SOIC-8-N" H 4875 1510 50  0001 C CNN
F 3 "" H 4875 1510 50  0001 C CNN
	1    4875 1510
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0109
U 1 1 603BDF01
P 4775 1195
F 0 "#PWR0109" H 4775 1045 50  0001 C CNN
F 1 "+12V" H 4790 1368 50  0000 C CNN
F 2 "" H 4775 1195 50  0001 C CNN
F 3 "" H 4775 1195 50  0001 C CNN
	1    4775 1195
	1    0    0    -1  
$EndComp
$Comp
L power:-12V #PWR0110
U 1 1 603BDF07
P 4775 1820
F 0 "#PWR0110" H 4775 1920 50  0001 C CNN
F 1 "-12V" H 4790 1993 50  0000 C CNN
F 2 "" H 4775 1820 50  0001 C CNN
F 3 "" H 4775 1820 50  0001 C CNN
	1    4775 1820
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R40
U 1 1 603BDF0D
P 4400 1610
F 0 "R40" V 4204 1610 50  0000 C CNN
F 1 "5.6K" V 4295 1610 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 4400 1610 50  0001 C CNN
F 3 "~" H 4400 1610 50  0001 C CNN
	1    4400 1610
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R42
U 1 1 603BDF14
P 4780 2085
F 0 "R42" V 4775 2095 50  0000 C CNN
F 1 "39K" V 4855 2090 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 4780 2085 50  0001 C CNN
F 3 "~" H 4780 2085 50  0001 C CNN
	1    4780 2085
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R44
U 1 1 603BDF1B
P 5330 1510
F 0 "R44" V 5495 1530 50  0000 C CNN
F 1 "1K" V 5405 1515 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 5330 1510 50  0001 C CNN
F 3 "~" H 5330 1510 50  0001 C CNN
	1    5330 1510
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 603BDF22
P 4540 1410
F 0 "#PWR0107" H 4540 1160 50  0001 C CNN
F 1 "GND" H 4390 1485 50  0000 C CNN
F 2 "" H 4540 1410 50  0001 C CNN
F 3 "" H 4540 1410 50  0001 C CNN
	1    4540 1410
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 1610 4535 1610
Wire Wire Line
	4540 1410 4575 1410
Wire Wire Line
	4775 1195 4775 1260
Wire Wire Line
	5175 1510 5205 1510
Wire Wire Line
	4535 1610 4535 2085
Wire Wire Line
	4535 2085 4680 2085
Connection ~ 4535 1610
Wire Wire Line
	4535 1610 4575 1610
Wire Wire Line
	4880 2085 5205 2085
Wire Wire Line
	5205 2085 5205 1510
Connection ~ 5205 1510
Wire Wire Line
	5205 1510 5230 1510
Wire Wire Line
	980  1745 1020 1745
$Comp
L Device:R_Small R37
U 1 1 603BE40B
P 2195 1745
F 0 "R37" V 2115 1735 50  0000 C CNN
F 1 "5.6K" V 2115 1860 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 2195 1745 50  0001 C CNN
F 3 "~" H 2195 1745 50  0001 C CNN
	1    2195 1745
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 603BE5ED
P 2195 1880
F 0 "#PWR0104" H 2195 1630 50  0001 C CNN
F 1 "GND" H 2045 1955 50  0000 C CNN
F 2 "" H 2195 1880 50  0001 C CNN
F 3 "" H 2195 1880 50  0001 C CNN
	1    2195 1880
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 1645 2195 1645
Wire Wire Line
	2195 1845 2195 1880
Wire Wire Line
	1495 1895 1495 1955
Wire Wire Line
	4775 1760 4775 1820
$Comp
L Device:C_Small C52
U 1 1 603BFBA5
P 2410 1745
F 0 "C52" H 2502 1791 50  0000 L CNN
F 1 "220pF" H 2502 1700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 2410 1745 50  0001 C CNN
F 3 "~" H 2410 1745 50  0001 C CNN
	1    2410 1745
	1    0    0    -1  
$EndComp
$Comp
L Device:CP_Small C54
U 1 1 603BFCE5
P 2570 1645
F 0 "C54" V 2345 1645 50  0000 C CNN
F 1 "10uF" V 2436 1645 50  0000 C CNN
F 2 "SMD_Packages:SMD-1206_Pol" H 2570 1645 50  0001 C CNN
F 3 "~" H 2570 1645 50  0001 C CNN
	1    2570 1645
	0    1    1    0   
$EndComp
Text GLabel 2700 1645 2    50   Input ~ 0
LIN
Wire Wire Line
	2195 1645 2410 1645
Connection ~ 2195 1645
Wire Wire Line
	2410 1645 2470 1645
Connection ~ 2410 1645
Wire Wire Line
	2410 1845 2195 1845
Connection ~ 2195 1845
Wire Wire Line
	2670 1645 2700 1645
Text GLabel 970  3345 0    50   Input ~ 0
IO_INR
Wire Wire Line
	1915 3245 1940 3245
Wire Wire Line
	1915 3820 1915 3245
Wire Wire Line
	1590 3820 1915 3820
Wire Wire Line
	1245 3345 1285 3345
Wire Wire Line
	1245 3820 1390 3820
Wire Wire Line
	1245 3345 1245 3820
Connection ~ 1915 3245
Wire Wire Line
	1250 3145 1285 3145
Connection ~ 1245 3345
Wire Wire Line
	1210 3345 1245 3345
$Comp
L power:GND #PWR099
U 1 1 603C2D30
P 1250 3145
F 0 "#PWR099" H 1250 2895 50  0001 C CNN
F 1 "GND" H 1100 3220 50  0000 C CNN
F 2 "" H 1250 3145 50  0001 C CNN
F 3 "" H 1250 3145 50  0001 C CNN
	1    1250 3145
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R34
U 1 1 603C2D36
P 2040 3245
F 0 "R34" V 2205 3265 50  0000 C CNN
F 1 "5.6K" V 1970 3240 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 2040 3245 50  0001 C CNN
F 3 "~" H 2040 3245 50  0001 C CNN
	1    2040 3245
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R32
U 1 1 603C2D3D
P 1490 3820
F 0 "R32" V 1655 3840 50  0000 C CNN
F 1 "33K" V 1565 3825 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 1490 3820 50  0001 C CNN
F 3 "~" H 1490 3820 50  0001 C CNN
	1    1490 3820
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R30
U 1 1 603C2D44
P 1110 3345
F 0 "R30" V 914 3345 50  0000 C CNN
F 1 "27K" V 1005 3345 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 1110 3345 50  0001 C CNN
F 3 "~" H 1110 3345 50  0001 C CNN
	1    1110 3345
	0    1    1    0   
$EndComp
Wire Wire Line
	970  3345 1010 3345
$Comp
L Device:R_Small R36
U 1 1 603C2D5F
P 2185 3345
F 0 "R36" V 2105 3335 50  0000 C CNN
F 1 "5.6K" V 2105 3460 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 2185 3345 50  0001 C CNN
F 3 "~" H 2185 3345 50  0001 C CNN
	1    2185 3345
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 603C2D66
P 2185 3480
F 0 "#PWR0103" H 2185 3230 50  0001 C CNN
F 1 "GND" H 2035 3555 50  0000 C CNN
F 2 "" H 2185 3480 50  0001 C CNN
F 3 "" H 2185 3480 50  0001 C CNN
	1    2185 3480
	1    0    0    -1  
$EndComp
Wire Wire Line
	2140 3245 2185 3245
Wire Wire Line
	2185 3445 2185 3480
$Comp
L Device:C_Small C51
U 1 1 603C2D6F
P 2400 3345
F 0 "C51" H 2492 3391 50  0000 L CNN
F 1 "220pF" H 2492 3300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 2400 3345 50  0001 C CNN
F 3 "~" H 2400 3345 50  0001 C CNN
	1    2400 3345
	1    0    0    -1  
$EndComp
$Comp
L Device:CP_Small C53
U 1 1 603C2D76
P 2560 3245
F 0 "C53" V 2335 3245 50  0000 C CNN
F 1 "10uF" V 2426 3245 50  0000 C CNN
F 2 "SMD_Packages:SMD-1206_Pol" H 2560 3245 50  0001 C CNN
F 3 "~" H 2560 3245 50  0001 C CNN
	1    2560 3245
	0    1    1    0   
$EndComp
Text GLabel 2690 3245 2    50   Input ~ 0
RIN
Wire Wire Line
	2185 3245 2400 3245
Connection ~ 2185 3245
Wire Wire Line
	2400 3245 2460 3245
Connection ~ 2400 3245
Wire Wire Line
	2400 3445 2185 3445
Connection ~ 2185 3445
Wire Wire Line
	2660 3245 2690 3245
$Comp
L 4ms-ic:TL082 U8
U 1 1 603C398A
P 1595 1645
F 0 "U8" H 1595 2012 50  0000 C CNN
F 1 "LME49720" H 1865 1875 50  0000 C CNN
F 2 "SMD_Packages:SOIC-8-N" H 1595 1645 50  0001 C CNN
F 3 "" H 1595 1645 50  0001 C CNN
	1    1595 1645
	1    0    0    -1  
$EndComp
$Comp
L Device:CP_Small C55
U 1 1 603C597C
P 4135 1610
F 0 "C55" V 4360 1610 50  0000 C CNN
F 1 "10uF" V 4269 1610 50  0000 C CNN
F 2 "SMD_Packages:SMD-1206_Pol" H 4135 1610 50  0001 C CNN
F 3 "~" H 4135 1610 50  0001 C CNN
	1    4135 1610
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R38
U 1 1 603C73F9
P 4265 1710
F 0 "R38" H 4206 1664 50  0000 R CNN
F 1 "39K" H 4206 1755 50  0000 R CNN
F 2 "Resistors_SMD:R_0805" H 4265 1710 50  0001 C CNN
F 3 "~" H 4265 1710 50  0001 C CNN
	1    4265 1710
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 603C768C
P 4265 1840
F 0 "#PWR0105" H 4265 1590 50  0001 C CNN
F 1 "GND" H 4115 1915 50  0000 C CNN
F 2 "" H 4265 1840 50  0001 C CNN
F 3 "" H 4265 1840 50  0001 C CNN
	1    4265 1840
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C57
U 1 1 603C79F8
P 4785 2410
F 0 "C57" V 4905 2410 50  0000 C CNN
F 1 "22pF" V 4675 2410 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 4785 2410 50  0001 C CNN
F 3 "~" H 4785 2410 50  0001 C CNN
	1    4785 2410
	0    1    1    0   
$EndComp
Wire Wire Line
	4535 2085 4535 2410
Wire Wire Line
	4535 2410 4685 2410
Connection ~ 4535 2085
Wire Wire Line
	5205 2410 5205 2085
Connection ~ 5205 2085
Wire Wire Line
	4885 2410 5205 2410
Text GLabel 3995 1610 0    50   Input ~ 0
LOUT
Text GLabel 5465 1510 2    50   Input ~ 0
IO_OUTL
Wire Wire Line
	5430 1510 5465 1510
Wire Wire Line
	4235 1610 4265 1610
Wire Wire Line
	4265 1610 4300 1610
Connection ~ 4265 1610
Wire Wire Line
	4265 1810 4265 1840
Wire Wire Line
	3995 1610 4035 1610
$Comp
L Device:R_Small R41
U 1 1 603D02E4
P 4400 3435
F 0 "R41" V 4204 3435 50  0000 C CNN
F 1 "5.6K" V 4295 3435 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 4400 3435 50  0001 C CNN
F 3 "~" H 4400 3435 50  0001 C CNN
	1    4400 3435
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R43
U 1 1 603D02EB
P 4780 3910
F 0 "R43" V 4775 3920 50  0000 C CNN
F 1 "39K" V 4855 3915 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 4780 3910 50  0001 C CNN
F 3 "~" H 4780 3910 50  0001 C CNN
	1    4780 3910
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R45
U 1 1 603D02F2
P 5330 3335
F 0 "R45" V 5495 3355 50  0000 C CNN
F 1 "1K" V 5405 3340 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 5330 3335 50  0001 C CNN
F 3 "~" H 5330 3335 50  0001 C CNN
	1    5330 3335
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 603D02F9
P 4540 3235
F 0 "#PWR0108" H 4540 2985 50  0001 C CNN
F 1 "GND" H 4390 3310 50  0000 C CNN
F 2 "" H 4540 3235 50  0001 C CNN
F 3 "" H 4540 3235 50  0001 C CNN
	1    4540 3235
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 3435 4535 3435
Wire Wire Line
	4540 3235 4575 3235
Wire Wire Line
	5175 3335 5205 3335
Wire Wire Line
	4535 3435 4535 3910
Wire Wire Line
	4535 3910 4680 3910
Connection ~ 4535 3435
Wire Wire Line
	4535 3435 4575 3435
Wire Wire Line
	4880 3910 5205 3910
Wire Wire Line
	5205 3910 5205 3335
Connection ~ 5205 3335
Wire Wire Line
	5205 3335 5230 3335
$Comp
L Device:CP_Small C56
U 1 1 603D030C
P 4135 3435
F 0 "C56" V 4360 3435 50  0000 C CNN
F 1 "10uF" V 4269 3435 50  0000 C CNN
F 2 "SMD_Packages:SMD-1206_Pol" H 4135 3435 50  0001 C CNN
F 3 "~" H 4135 3435 50  0001 C CNN
	1    4135 3435
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R39
U 1 1 603D0313
P 4265 3535
F 0 "R39" H 4206 3489 50  0000 R CNN
F 1 "39K" H 4206 3580 50  0000 R CNN
F 2 "Resistors_SMD:R_0805" H 4265 3535 50  0001 C CNN
F 3 "~" H 4265 3535 50  0001 C CNN
	1    4265 3535
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 603D031A
P 4265 3665
F 0 "#PWR0106" H 4265 3415 50  0001 C CNN
F 1 "GND" H 4115 3740 50  0000 C CNN
F 2 "" H 4265 3665 50  0001 C CNN
F 3 "" H 4265 3665 50  0001 C CNN
	1    4265 3665
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C58
U 1 1 603D0320
P 4785 4235
F 0 "C58" V 4905 4235 50  0000 C CNN
F 1 "22pF" V 4675 4235 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 4785 4235 50  0001 C CNN
F 3 "~" H 4785 4235 50  0001 C CNN
	1    4785 4235
	0    1    1    0   
$EndComp
Wire Wire Line
	4535 3910 4535 4235
Wire Wire Line
	4535 4235 4685 4235
Connection ~ 4535 3910
Wire Wire Line
	5205 4235 5205 3910
Connection ~ 5205 3910
Wire Wire Line
	4885 4235 5205 4235
Text GLabel 3995 3435 0    50   Input ~ 0
ROUT
Text GLabel 5465 3335 2    50   Input ~ 0
IO_OUTR
Wire Wire Line
	5430 3335 5465 3335
Wire Wire Line
	4235 3435 4265 3435
Wire Wire Line
	4265 3435 4300 3435
Connection ~ 4265 3435
Wire Wire Line
	4265 3635 4265 3665
Wire Wire Line
	3995 3435 4035 3435
$Comp
L 4ms-ic:TL082 U9
U 2 1 602DE93A
P 4875 3335
F 0 "U9" H 5105 3475 50  0000 L CNN
F 1 "LME49720" H 4865 3160 50  0000 L CNN
F 2 "SMD_Packages:SOIC-8-N" H 4875 3335 50  0001 C CNN
F 3 "" H 4875 3335 50  0001 C CNN
	2    4875 3335
	1    0    0    -1  
$EndComp
Wire Wire Line
	1885 3245 1915 3245
Text Notes 6520 1635 0    50   ~ 0
add bypass caps
$Comp
L power:+12V #PWR0117
U 1 1 603F6630
P 6815 2095
F 0 "#PWR0117" H 6815 1945 50  0001 C CNN
F 1 "+12V" H 6830 2268 50  0000 C CNN
F 2 "" H 6815 2095 50  0001 C CNN
F 3 "" H 6815 2095 50  0001 C CNN
	1    6815 2095
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 603F6673
P 6815 2375
F 0 "#PWR0118" H 6815 2125 50  0001 C CNN
F 1 "GND" H 6665 2450 50  0000 C CNN
F 2 "" H 6815 2375 50  0001 C CNN
F 3 "" H 6815 2375 50  0001 C CNN
	1    6815 2375
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C63
U 1 1 603F68DF
P 6815 2235
F 0 "C63" V 6935 2235 50  0000 C CNN
F 1 "100nF" V 6705 2235 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 6815 2235 50  0001 C CNN
F 3 "~" H 6815 2235 50  0001 C CNN
	1    6815 2235
	-1   0    0    1   
$EndComp
Wire Wire Line
	6815 2095 6815 2135
Wire Wire Line
	6815 2335 6815 2375
$Comp
L power:+12V #PWR0119
U 1 1 603F9E8E
P 7425 2090
F 0 "#PWR0119" H 7425 1940 50  0001 C CNN
F 1 "+12V" H 7440 2263 50  0000 C CNN
F 2 "" H 7425 2090 50  0001 C CNN
F 3 "" H 7425 2090 50  0001 C CNN
	1    7425 2090
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 603F9E94
P 7425 2370
F 0 "#PWR0120" H 7425 2120 50  0001 C CNN
F 1 "GND" H 7275 2445 50  0000 C CNN
F 2 "" H 7425 2370 50  0001 C CNN
F 3 "" H 7425 2370 50  0001 C CNN
	1    7425 2370
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C65
U 1 1 603F9E9A
P 7425 2230
F 0 "C65" V 7545 2230 50  0000 C CNN
F 1 "100nF" V 7315 2230 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 7425 2230 50  0001 C CNN
F 3 "~" H 7425 2230 50  0001 C CNN
	1    7425 2230
	-1   0    0    1   
$EndComp
Wire Wire Line
	7425 2090 7425 2130
Wire Wire Line
	7425 2330 7425 2370
$Comp
L power:GND #PWR0121
U 1 1 603FD3C7
P 6800 3285
F 0 "#PWR0121" H 6800 3035 50  0001 C CNN
F 1 "GND" H 6650 3360 50  0000 C CNN
F 2 "" H 6800 3285 50  0001 C CNN
F 3 "" H 6800 3285 50  0001 C CNN
	1    6800 3285
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C62
U 1 1 603FD3CD
P 6800 3145
F 0 "C62" V 6920 3145 50  0000 C CNN
F 1 "100nF" V 6690 3145 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 6800 3145 50  0001 C CNN
F 3 "~" H 6800 3145 50  0001 C CNN
	1    6800 3145
	-1   0    0    1   
$EndComp
Wire Wire Line
	6800 3005 6800 3045
Wire Wire Line
	6800 3245 6800 3285
$Comp
L power:GND #PWR0122
U 1 1 603FD3DC
P 7410 3280
F 0 "#PWR0122" H 7410 3030 50  0001 C CNN
F 1 "GND" H 7260 3355 50  0000 C CNN
F 2 "" H 7410 3280 50  0001 C CNN
F 3 "" H 7410 3280 50  0001 C CNN
	1    7410 3280
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C64
U 1 1 603FD3E2
P 7410 3140
F 0 "C64" V 7530 3140 50  0000 C CNN
F 1 "100nF" V 7300 3140 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 7410 3140 50  0001 C CNN
F 3 "~" H 7410 3140 50  0001 C CNN
	1    7410 3140
	-1   0    0    1   
$EndComp
Wire Wire Line
	7410 3000 7410 3040
Wire Wire Line
	7410 3240 7410 3280
$Comp
L power:-12V #PWR0123
U 1 1 603FEE7A
P 6800 3005
F 0 "#PWR0123" H 6800 3105 50  0001 C CNN
F 1 "-12V" H 6815 3178 50  0000 C CNN
F 2 "" H 6800 3005 50  0001 C CNN
F 3 "" H 6800 3005 50  0001 C CNN
	1    6800 3005
	1    0    0    -1  
$EndComp
$Comp
L power:-12V #PWR0124
U 1 1 603FEF85
P 7410 3000
F 0 "#PWR0124" H 7410 3100 50  0001 C CNN
F 1 "-12V" H 7425 3173 50  0000 C CNN
F 2 "" H 7410 3000 50  0001 C CNN
F 3 "" H 7410 3000 50  0001 C CNN
	1    7410 3000
	1    0    0    -1  
$EndComp
$EndSCHEMATC
