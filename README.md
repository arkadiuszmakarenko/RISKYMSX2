
# RISKYMSX2

The RISKYMSX2 cartridge PROTOTYPE by Arek Makarenko is a compact MSX flash ROM cartridge based on the inexpensive CH32V467VCT6 RISC-V MCU which can be easily programmed from a MSX computer using a FAT formatted USB pen drive containing ROM files.
This chip is 200Mhz with 8MB PSRAM onboard, USB HS, and I will be figting to meet all needed MSX timings. Wish me luck!

DO NOT BUILD AT THIS STAGE unless you can create own firmware!


<<<<<<< HEAD
Piout ChatGPT
U1 CONNECTIONS
KiCad 10 schematic: RISKYMSX2.kicad_sch
Component: CH32V467VET6

Pin	MCU pin	Connection
----	-------	----------
1	PE2(FT)	WR
2	PE3(FT)	WAIT
3	PE4(FT)	~{RESET}
4	PE5(FT)	MREQ
5	PE6(FT)	M1
6	VBAT	unlabelled connection
7	PC13	NC
8	PC14	NC
9	PC15	NC
10	VSS	unlabelled connection
11	VDD	unlabelled connection
12	OSC_IN	X1 (crystal connection)
13	OSC_OUT	X1 (crystal connection)
14	~{NRST}	NRST
15	PC0	NC
16	PC1	NC
17	PC2	NC
18	PC3	NC
19	VSSA	unlabelled connection
20	VREF-	unlabelled connection
21	VREF+	unlabelled connection
22	VDDA	unlabelled connection
23	PA0	LEDFLASH
24	PA1	NC
25	PA2	NC
26	PA3	NC
27	VSS	unlabelled connection
28	VDD	unlabelled connection
29	PA4(HS)	SCC_OUT
30	PA5(HS)	NC
31	PA6(HS)	NC
32	PA7(HS)	NC
33	PC4	NC
34	PC5	NC
35	PB0	NC
36	PB1	NC
37	BOOT1(FT)	GND
38	PE8(FT)	IORQ
39	PE9(FT)	BDIR
40	VDDK	unlabelled connection
41	VSS	unlabelled connection
42	MDIRN	NC
43	MDIRP	NC
44	MDITN	NC
45	MDITP	NC
46	VDD	unlabelled connection
47	VSS	unlabelled connection
48	VDD18	unlabelled connection
49	PB10(FT)	D2
50	PB11(FT)	D3
51	PD8(FT)	A8
52	PB12(FT)	D4
53	PB13(FT)	D5
54	PB14(FT)	D6
55	PB15(FT)	D7
56	PD9(FT)	A9
57	PD10(FT)	A10
58	PD11(FT)	A11
59	PD12(FT)	A12
60	PD13(FT)	A13
61	PD14(FT)	A14
62	PD15(FT)	A15
63	PC6(FT)	CS2
64	PC7(FT)	RFSH
65	PC8(FT)	SLOTINT
66	PC9(FT)	SLOTCLK
67	PA8(FT)	NC
68	PA9(FT)	USART_TX
69	PA10(FT)	USART_RX
70	PA11	USB1_DM
71	PA12	USB1_DP
72	SWDIO(FT)	SWDIO
73	PE10(FT)	CS1
74	PE11(FT)	CS12
75	VDD	unlabelled connection
76	SWCLK(FT)	SWCLK
77	PA15(FT)	NC
78	PC10(FT)	NC
79	PC11(FT)	NC
80	PC12(FT)	NC
81	PD0(FT)	A0
82	PD1(FT)	A1
83	PD2(FT)	A2
84	PD3(FT)	A3
85	PD4(FT)	A4
86	PD5(FT)	A5
87	PD6(FT)	A6
88	PD7(FT)	A7
89	PB3(FT)	NC
90	PB4(FT)	NC
91	PB5(FT)	NC
92	PB6	NC
93	PB7	NC
94	BOOT0(FT)	BOOT0
95	PB8(FT)	D0
96	PB9(FT)	D1
97	PE0(FT)	~{EXSLTSL}
98	PE1(FT)	RD
99	VSS	unlabelled connection
100	VDD	unlabelled connection

Notes:
- NC = explicit KiCad no-connect marker.
- 'unlabelled connection' = the pin is wired, but no named net label was found on that net.
- X1 = the crystal connected to U1 pins 12 and 13.

Important: In this schematic U1 pin 34 (PC5) is NC; A0 is U1 pin 81 (PD0).
=======
<img width="1167" height="826" alt="Screenshot from 2026-08-22 15-25-02" src="https://github.com/user-attachments/assets/4b711d65-d632-4d83-b889-175446aa83a0" />
>>>>>>> 920e6b2be6f260394af7e6f48916e51129308a6d
