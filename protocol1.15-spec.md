1.  Interface  RS485
 
		baud rate 19200  
		startbits 1 
		databits 8 
		stoppbits 1 
		parity even 
 
 2. Communication
		- The master (PLC) sends the data packets in cycles of 150ms to the slave (ACS).
		- The slave must answer with his data packet before the master changes his interface back to the sending state, less then 150ms
		- The master checks, if the slave answers correctly (checksum) 
		- Also the slave must check, if there is a lack of information. In this case the slave has to set the system in an secure state
		
3.	Packet times 
		1 Byte = 11 transmitted bit  (1 start bit, 8 data bit, 1 parity, 1 stop bit) 
		Packet ACS  PLC  : 38 (+max. 34) Byte 
		Packet PLC  ACS  : 54 (+max. 50) Byte
		
4. 	Protocol rules
		- Each data frame starts with the sequence 10h, 02h (DLE, STX) and ends with the sequence 10h, 03h(DLE, ETX). 
		  To transmit the character “10h” in the user data, it is necessary to extend this character by the value “41h” (see example).
		- The calculation of the checksum (CRC) and the length of packet (SIZE) have to be done before the data stream is extended by the value “41h” !  
		
		Data packet (PLC -> ACS):
			Format Inhalt Daten Anmerkung 
    
			Byte DLE rrh DLE : 10h 
			Byte STX rrh STX : 02h 
			Byte SIZE rrh Size of data packet  = count of byte between SIZE and CHECKSUM 
			Byte data-ID rrh Identification of the data packet :  100d = ACS-data (PLC  ACS) 
			Byte version-ID rrh Version of the packet configuration : 0d = Version 1.00 
			Byte dummy rrh free 
			DWord packet number rrrrrrrrh Actual packet number, to differentiate each packet 
			Word plc_system_word1 rrrrh PLC-System-Word 1 : 
															Bit0  = 1  :  free 
															Bit1  = 1  :  control system ON 
															Bit2  = 1  :  crane is free slewing 
															Bit3  = 1  :  JS slewing gear right 
															Bit4  = 1  :  JS slewing gear left 
															Bit5  = 1  :  JS trolley forward /  
																		  JS boom down (luffing crane) 
															Bit6  = 1  :  JS trolley backward /  
																		  JS boom up (luffing crane) 
															Bit7  = 1  :  JS travel gear forward 
															Bit8  = 1  :  JS travel gear backward 
															Bit9  = 1  :  JS hoisting gear up 
															Bit10 = 1  :  JS hoisting gear down 
															Bit11 = 1  :  Dead-man switch triggered in  
																		  left JS (control lever is 
																		  encompassed by hand).  
																		  Notice : In remote control 
																		  this bit is always set to 1, also 
																		  if the handheld is switched off.     
															Bit12 = 1  :  Dead-man switch triggered in  
																		  right JS (control lever is 
																		  encompassed by hand).  
																		  Notice : In remote control 
																		  this bit is always set to 1, also 
																		  if the handheld is switched off.    
															Bit13 = 1  :  not used 
															Bit14 = 1  :  not used 
															Bit15 = 1  :  position data valid (JS = Joystick)  
			Word plc_system_word2 rrrrh PLC-System-Word 2 : 
															Bit0  : slewing mode info bit0*  
															Bit1  : slewing mode info bit1* 
															Bit2  : remote control active (1 = active) 
															Bit3  : single/double trolley used (1 = double) 
															Bit4  : free 
															Bit5  : free 
															Bit6  : free 
															Bit7  : free 
															Bit8  : free 
															Bit9  : free 
															Bit10 : free 
															Bit11 : free 
															Bit12 : free 
															Bit13 : free 
															Bit14 : free 
															Bit15 : free  
			Word plc_system_word3 rrrrh PLC-System-Word 3 : 
															Bit0  : free 
															Bit1  : free 
															Bit2  : free 
															Bit3  : free 
															Bit4  : free 
															Bit5  : free 
															Bit6  : free 
															Bit7  : free 
															Bit8  : free 
															Bit9  : free 
															Bit10 : free 
															Bit11 : free 
															Bit12 : free 
															Bit13 : free 
															Bit14 : free 
															Bit15 : free  
			Word dummy rrrrh free 
			Word trolley sensor rrrrh trolley position in 0.1m  
			Word hoist sensor rrrrh lifting height in 0.1m     
			Word slewing sensor rrrrh slewing angle in 0.1°    
			Word wind sensor rrrrh wind speed in 0.1 m/s  
			Word travel gear sensor rrrrh travel gear position in 0.1m 
			Word load sensor rrrrh load in 0.01t 
			Word slewing sensor rrrrh slewing speed in 0.001/min 
			Word luffing gear sensor rrrrh luffing angle in 0.1° (only luffing cranes) 
			Byte Actual speed set value trolley fc rrh Actual speed set value to the trolley frequency converter (0 - 100 = 0 - 100%) 
			Byte Actual speed set value hoisting fc rrh Actual speed set value to the frequency converter of the hoisting unit  (0 - 100 = 0 - 100%) 
			Word Max. radius of trolley rrrrh Max. radius of the trolley in 0.1m  
			Word dummy rrrrh free 
			Word dummy rrrrh free 
			Word dummy rrrrh free 
			Word dummy rrrrh free 
			DWord CHECKSUM (CRC32) rrrrrrrrh CRC32-checksum over all data bytes = Begin  SIZE, End  Value before CHECKSUM 
			Byte DLE rrh DLE : 10h 
			Byte ETX rrh ETX : 03h 
			 
			* - explanation to Bit0 and Bit1 in plc_system_word2: 
			 
			Bit0  |  Bit1  |   Actual slewing-Mode (FC slewing gear)  
			  0   |   0    |   speed regulation (automatic decelaration)   
			  1   |   0    |   torque controlled (no automatic deceleration) 
			  0   |   1    |   combination of speed regulation and torque controlled 
			  1   |   1    |   undefined 

5. Data format 
	- The actual values of the drives are transferred as binary word values.  The formats of the word values are shown in the next chart. 
	
	 
		Description 		Possible values (binary) 		Possible values (dec.) 		Format 
			
		slewing angle 		0 – 07FFEh 						0 – 32766 					359.9 ° -> 3599 
		slewing speed 		0 – 07FFEh 						0 – 32766 					0.1 1/min -> 100 
		trolley distance 	0 – 07FFEh 						0 – 32766 					99.9 m -> 999 
		luffing angle 		0 – 07FFEh 						0 – 32766 					90.0 ° -> 900 
		lifting height 		0 – 07FFEh 						0 – 32766 					999.9 m -> 9999 
		load 				0 – 07FFEh 						0 – 32766  					2.13 t -> 213 
		wind speed 			0 – 07FFEh 						0 – 32766  					10.4 m/s -> 104 
		travel gearposition 0 – 07FFEh 						0 – 32766  					99.9 m -> 999 
		Sensor-ERROR 		7FFFh 							32767 						--- 
 
 
	- If a sensor error occurs the PLC sets the value for this sensor to 32767. This is the information for the ACS system for a sensor error.

6. Data packet (ACS -> PLC)

	Format Inhalt Daten Anmerkung 
    
	Byte DLE rrh DLE : 10h 
	Byte STX rrh STX : 02h 
	Byte SIZE rrh Size of data packet = count of byte between SIZE and CHECKSUM 
	Byte data-ID rrh Identification of the data packet : 200d = ACS-data (ACS  PLC) 
	Byte version-ID rrh Version of the packet configuration : 0d = Version 1.00 
	Byte dummy rrh free 
	DWord packet number rrrrrrrrh Actual packet number, to differentiate each packet 
	Word acs_system_word1 rrrrh ACS-System-Word 1 : 
													Bit0  = 1 :  STOP slewing right 
													Bit1  = 1 :  STOP slewing left 
													Bit2  = 1 :  slewing direction left 
													Bit3  = 1 :  slewing direction right 
													Bit4  = 1 :  close the slewing break 
													Bit5  = 1 :  slewing with minimum speed in the 
																 given direction. Limit set value  
																 for slewing is used (*).  
													Bit6  = 1 :  free 
													Bit7  = 1 :  free 
													Bit8  = 1 :  STOP trolley forward / STOP boom down (LUFF) (FC-RAMP) 
													Bit9  = 1 :  STOP trolley backward /  STOP boom up (LUFF) (FC-RAMP) 
													Bit10 = 1 :  STOP trolley forward /  STOP boom down (LUFF) (FAST STOP) 
													Bit11 = 1 :  STOP trolley backward /  STOP boom up (LUFF) (FAST STOP) 
													Bit12 = 1 :  limit forward trolleying / limit boom down speed (LUFF) 
													Bit13 = 1 :  limit backward trolleying / limit boom up speed (LUFF)  
													Bit14 = 1 :  acs-system is overridden (by key) 
													Bit15 = 1 :  STOP all movements -> set all breaks ! (LUFF = luffing cranes)  
	Word acs_system_word2 rrrrh ACS-System-Word 2 : 
													Bit0  = 1 : STOP forward travelling 
													Bit1  = 1 : STOP backward travelling 
													Bit2  = 1 : limit forward travelling speed 
													Bit3  = 1 : limit backward travelling speed 
													Bit4  = 1 : STOP hook raising (FAST STOP) 
													Bit5  = 1 : STOP hook lowering (FAST STOP) 
													Bit6  = 1 : limit hook raising speed 
													Bit7  = 1 : limit hook lowering speed 
													Bit8  = 1 : STOP hook raising (FC-RAMP) 
													Bit9  = 1 : STOP hook lowering (FC-RAMP) 
													Bit10 : free 
													Bit11 : free 
													Bit12 : free 
													Bit13 : free 
													Bit14 : free 
													Bit15  = 1 : Torque control is activated in the  acs-system acs-system will counter to reduce the slewing speed)  
	Word dummy rrrrh free 
	Byte Slewing counter set value rrh Counter set value for slewing gear (0 - 100 = 0 - 100%) 
	Byte Max. speed pre limit area for trolley / luffing rrh Max. speed for the trolley / luffing in pre limit area (0 - 100 = 0 - 100%).  Attention : The value is taken over, 
															 when the corresponding pre-limit bit is set by ACS. 
	Byte Max. speed pre limit area for hoisting gear rrh Max. speed for the hoisting gear in pre-limit area (0 - 100 = 0 - 100%). 
														 Attention : The value is taken over, when the corresponding pre-limit bit is set by ACS. 
	Byte Limit set value for Slewing gear (*) rrh Limit set value for slewing for approaching to  obstacle (10 - 100 = 10 -100%). 
												  It is used if Bit 5 in acs_system_word1 is set. 
	Word dummy rrrrh Free 
	Word dummy rrrrh Free 
	Word dummy rrrrh free 
	Word dummy rrrrh free 
	Word dummy rrrrh free 
	DWord CHECKSUM (CRC32) rrrrrrrrh CRC32-checksum over all data bytes = Begin  SIZE, End  Value before CHECKSUM 
	Byte DLE rrh DLE : 10h 
	Byte ETX rrh ETX : 03h
	
	
7. Consecutive numbering 
	- Each packet is market with a consecutive number. So the receiver of the packet is able to check, if there is something wrong in the transmit unit of the sender
	- The consecutive number is protected via a 32Bit-CRC-Checksum together with all other data. 
	  Both the master (PLC of the crane) and the slave (ACS) have to implement this in their send packets

8. Identification for each packet format 
	- There is only one packet for receiving and sending. Each packet is market with an unique identification code. 
	  Additional to this there is only one sender and one receiver. 
	- The identification code is protected via a 32Bit-CRC-Checksum together with all other data
	
9. Watchdog monitoring 
	-  the slave (ACS) a watchdog monitoring has to be installed. This monitoring function  has to check if two of three expected consecutive packets are wrong or missing. 

10. Protection of the data integrity by CRC32 checksum 
	- The CRC32-Checksum is calculated by using a lockup table
	- The checksum is calculated over all data bytes of the packet, that means from the data-ID until the last byte before the checksum datum in the packet.
	Basics of the CRC32 checksum calculation : 
			Generator polynom for the lockup-table : (IEEE802.3, used for Ethernet) 
			x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x + 1 
			Minimum Haming-Distanz (MHD) : 	4 
			Maximum length for MHD :   		2^32 – 1 Bits 
			CRC-Format :    				DWord 
			Checksum start value :   		FFFFFFFFh (standard initial value)
			
	Calculation : 
			1. XOR–operation of the input byte with the low-order byte of the checksum, to generate the index for the lockup table. 
			2. Shift the DWord checksum value 8Bits to the right. 
			3. XOR-operation of the checksum DWord value with the indicated table value. 
			4. Repeat step 1.-3. until the checksum is calculated over all data bytes.  
			
			Lookup – Table :  pic is added!!
			
11. Error response
	- Both systems (PLC and ACS) have to react in a suitable way if they detect a failure in the bus connection
	-  if two of three consecutive data frames are wrong or missing. the PLC will stop all drives immediately, that means that all breaks 
	   will be closed.(very importent!!)
	- The ACS must send the information of the disturbance to all cranes that have an overlapping working area to the crane with this problem. 
	  In this case the ACS must lock the whole overlapping working area.(this will be implemented in the step of connection the protocol to the hole system, not now!!)
	  
12. Data order in memory 
	- Data are stored in the memory of the PLC (AC500S) in Motorola byte order (Big-Endian)
	****It is very important to store the data in the same way, before the CRC32 checksum will be calculated. 
	    The fact that the calculation of the checksum based directly on the data order in the memory makes it necessary that the slave (ACS) 
		have the same data order before calculation of the checksum. 

 