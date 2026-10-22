1. General description 
	- The communication between PLC and ACS is implemented on a RS485 interface.
	- The PLC isn’t directly connected with the ACS. A modul (CSM485 modul) converts 
	  the data from the CS31 bus (special bus from ABB) to the following RS485 bus protocol. 

2. Interface
	Parameter  Adjustment 
  
	baud rate  19200  
	startbits  1 
	databits   8 
	stoppbits  1 
	parity 	   even 

3. Data exchange
	- The master (CSM485modul) sends the data packets in cycles of 130ms to the slave (ACS).
	- The slave has to answer with his data packet before the master changes his interface back to the sending state(less then 130ms)
	- If there will be three wrong consecutive packets, the master will detect on a communication error and all drives will be stopped.
	- Additional the slave has to check, if there is a lack of information. In this case the slave has to set the system in a secure state. 
	
4. Packet times
	1 Byte = 11 transmitted bit  (1 start bit, 8 data bit, 1 parity, 1 stopp bit) 
		packet ACS -> PLC  : 8 Byte 
		packet PLC -> ACS  : 28 Byte 
		
5. Data packet (PLC -> ACS):

	ByteNum Bit Possible_values Description Bit_activ Function 
      
	0 	0-7 C0h STX  -- Start of Text 
      
	1 	 0 0/1 1.State / 0 1 override signal from PLC  
		 1 0/1 1.State / 1 1 control system ON 
		 2 0/1 1.State / 2 1 crane is in free slewing 
		 3 0/1 1.State / 3 1 slewing gear right 
		 4 0/1 1.State / 4 1 slewing gear left 
		 5 0/1 1.State / 5 1 trolley direction forw. / jib dir. outside (HCL) 
		 6 0/1 1.State / 6 1 trolley direction backw. / jib dir. inside (HCL) 
		 7 0 ---   
      
	2 	 0 0/1 2.State / 0 1 travel gear forward 
		 1 0/1 2.State / 1 1 travel gear backward 
		 2 0/1 2.State / 2 1 hook direction upward 
		 3 0/1 2.State / 3 1 hook direction downward 
		 4 0/1 2.State / 4 -- free 
		 5 0/1 2.State / 5 -- free 
		 6 0/1 2.State / 6 -- free 
		 7 0 ---   
      
	3 	 0 0/1 1.Res / 0 -- free 
		 1 0/1 1.Res / 1 -- free 
		 2 0/1 1.Res / 2 -- free 
		 3 0/1 1.Res / 3 -- free 
		 4 0/1 1.Res / 4 -- free 
		 5 0/1 1.Res / 5 -- free 
		 6 0/1 1.Res / 6 -- free 
		 7 0 ---   
      
	4 	 0 0/1 2.Res / 0 -- free 
		 1 0/1 2.Res / 1 -- free 
		 2 0/1 2.Res / 2 -- free 
		 3 0/1 2.Res / 3 -- free 
		 4 0/1 2.Res / 4 -- free 
		 5 0/1 2.Res / 5 1 position data valid 
		 6 0/1 2.Res / 6 1 bus connection between plc and CSM485 modul is o. k.  
		 7 0 ---   
      
	5 	 0-6 0 – 7Fh Value 00 (DB1-7) -- slewing angle (Bit1-7) * 
		 7 0 ---   
      
	6 	 0-6 0 – 7Fh Value 00 (DB9-15) -- slewing angle (Bit9-15) * 
		 7 0 ---   
		  
	7 	0-6 0 – 7Fh Value 01 (DB1-7) -- slewing speed (Bit1-7) 
		7 0 ---   
		  
	8 	0-6 0 – 7Fh Value 01 (DB9-15) -- slewing speed (Bit9-15) 
		7 0 ---   
		  
	9 	0-6 0 – 7Fh Value 02 (DB1-7) -- trolley distance (Bit1-7) 
		7 0 ---   
      
	10 	0-6 0 – 7Fh Value 02 (DB9-15) -- trolley distance (Bit9-15) 
		7 0 ---   
		  
	11 	0-6 0 – 7Fh Value 03 (DB1-7) -- luffing angle (Bit1-7) (only luffing jib cranes) 
		7 0 ---   
		  
	12 	0-6 0 – 7Fh Value 03 (DB9-15) -- luffing angle (Bit9-15) (only luffing jib cranes) 
		7 0 ---   
		  
	13 	0-6 0 – 7Fh Value 04 (DB1-7) -- lifting height (Bit1-7) 2 
		7 0 ---   
		  
	14 	0-6 0 – 7Fh Value 04 (DB9-15) -- lifting height (Bit9-15) 
		7 0 ---   
		  
	15 	0-6 0 – 7Fh Value 05 (DB1-7) -- load (Bit1-7) 
		7 0 ---   
		  
	16 	0-6 0 – 7Fh Value 05 (DB9-15) -- load (Bit9-15) 
		7 0 ---   
		  
	17 	0-6 0 – 7Fh Value 06 (DB1-7) -- wind speed (Bit1-7)  
		7 0 ---   
		  
	18 	0-6 0 – 7Fh Value 06 (DB9-15) -- wind speed (Bit9-15)  
		7 0 ---   
		  
	19 	0-6 0 – 7Fh Value 07 (DB1-7) -- travel gear position (Bit1-7) 
		7 0 ---   
		  
	20 	0-6 0 – 7Fh Value 07 (DB9-15) -- travel gear position (Bit9-15) 
		7 0 ---   
		  
	21 	0-6 0 – 7Fh Value 08 (DB1-7) -- free 
		7 0 ---   
		  
	22 	0-6 0 – 7Fh Value 08 (DB9-15) -- free 
		7 0 ---   
		  
	23 	 0 0/1 Value 00 (DB0) -- data bit 0 of Value 00 
		 1 0/1 Value 00 (DB8) -- data bit 8 of Value 00 
		 2 0/1 Value 01 (DB0) -- data bit 0 of Value 01 
		 3 0/1 Value 01 (DB8) -- data bit 8 of Value 01 
		 4 0/1 Value 02 (DB0) -- data bit 0 of Value 02 
		 5 0/1 Value 02 (DB8) -- data bit 8 of Value 02 
		 6 0/1 --- -- free 
		 7 0 ---   
		  
	24 	 0 0/1 Value 03 (DB0) -- data bit 0 of Value 03 
		 1 0/1 Value 03 (DB8) -- data bit 8 of Value 03 
		 2 0/1 Value 04 (DB0) -- data bit 0 of Value 04 
		 3 0/1 Value 04 (DB8) -- data bit 8 of Value 04 
		 4 0/1 Value 05 (DB0) -- data bit 0 of Value 05 
		 5 0/1 Value 05 (DB8) -- data bit 8 of Value 05 
		 6 0/1 --- -- free 
		 7 0 ---   
On luffing-cranes (HCL) lifting-height 0.0m with vertical positioned jib and hook up 
 
	25 	 0 0/1 Value 06 (DB0) -- data bit 0 of Value 06 
		 1 0/1 Value 06 (DB8) -- data bit 8 of Value 06 
		 2 0/1 Value 07 (DB0) -- data bit 0 of Value 07 
		 3 0/1 Value 07 (DB8) -- data bit 8 of Value 07 
		 4 0/1 Value 08 (DB0) -- data bit 0 of Value 08 
		 5 0/1 Value 08 (DB8) -- data bit 8 of Value 08 
		 6 0/1 --- -- free 
		 7 0 ---   
      
	26 	0-6 0 – 7Fh XOR (Byte1 – 25) -- Checksum 
		7 0 ---   
		  
	27 	0-7 80h ETX -- End of Text
	
	
6. Data format  
	- The actual values of the drives are transferred as binary word values.
	- The data word is transferred in 2 separated bytes
	- These two bytes content the bit values from bit1 to bit7 and the bit values from bit9 to bit15
	-  The MSB of each byte is 0, to detect the STX and ETX correctly! The values of bit0 and bit8 are transferred in an additional byte.
	
	Description    Possible_values(binary) 		Possible_values (dec.) 		Format 
    
	slewing angle 		0 – 07FFEh 					0 – 32766 				359.9 ° -> 3599 
	slewing speed 		0 – 07FFEh 					0 – 32766 				0.1 1/min -> 100 
	trolley distance 	0 – 07FFEh 					0 – 32766 				99.9 m -> 999 
	luffing angle 		0 – 07FFEh 					0 – 32766 				90.0 ° -> 900 
	lifting height 		0 – 07FFEh 					0 – 32766 				999.9 m -> 9999 
	load 				0 – 07FFEh 					0 – 32766  				2.13 t -> 213 
	wind speed 			0 – 07FFEh 					0 – 32766  				10.4 m/s -> 104 
	travel gear_position 0 – 07FFEh 				0 – 32766  				99.9 m -> 999 
	Sensor-ERROR 		7FFFh 						32767 					--- 
	
7. Data packet (ACS -> PLC): 
	
	Byte No.	 Bit 	Possible_values 	Description 	Bit_value  Function 
      
	0 			 0-7 C0h STX -- Start of Text 
		  
	1 			 0 0/1 1.State / 0 1 Stopp slewing right 
				 1 0/1 1.State / 1 1 Stopp slewing left 
				 2 0/1 1.State / 2 1 Slewing left in first step 
				 3 0/1 1.State / 3 1 Slewing right in first step 
				 4 0/1 1.State / 4 1 Slew in second step 
				 5 0/1 1.State / 5 1 Slew in third step 
				 6 0/1 1.State / 6 1 Slew in fourth step 
				 7 0 ---   
		  
	2 			 0 0/1 2.State / 0 1 Slew in fifth step 
				 1 0/1 2.State / 1 1 Close the slewing brake  
				 2 0/1 2.State / 2 1 Stopp forward trolleying  / Stopp outside luffing 
				 3 0/1 2.State / 3 1 Stopp backward trolleying  / Stopp inside luffing 
				 4 0/1 2.State / 4 1 limit forward trolleying / jib outside luffing speed 
				 5 0/1 2.State / 5 1 limit backward trolleying / jib inside luffing speed 
				 6 0/1 2.State / 6 1 Stopp forward travelling 
				 7 0 ---   
		  
	3 			 0 0/1 3.State / 0 1 Stopp backward travelling 
				 1 0/1 3.State / 1 1 limit forward travelling speed 
				 2 0/1 3.State / 2 1 limit backward travelling speed 
				 3 0/1 3.State / 3 1 Stopp hook raising 
				 4 0/1 3.State / 4 1 Stopp hook lowering 
				 5 0/1 3.State / 5 1 limit hook raising speed 
				 6 0/1 3.State / 6 1 limit hook lowering speed 
				 7 0 ---   
		  
	4 			 0 0/1 4.State / 0 1 free 
				 1 0/1 4.State / 1 1 free 
				 2 0/1 4.State / 2 1 free 
				 3 0/1 4.State / 3 1 free 
				 4 0/1 4.State / 4 -- Limit the set value of the slewing gear to slew slowly to border (default 10%) 
				 5 0/1 4.State / 5 1 anti-collision system is overridden (by key) 
				 6 0/1 4.State / 6 1 Stopp all motions and activate the brakes 
				 7 0 ---   
		  
	5 			 0 0/1 5.State / 0 -- Info bit0 to limit slewing speed * 
				 1 0/1 5.State / 1 -- Info bit1 to limit slewing speed * 
				 2 0/1 5.State / 2 -- Info bit for max. speed in the pre-limitswitch sector of the trolley or luffing gear 0 = 50%, 1 = 25% 
				 3 0/1 5.State / 3 -- free 
				 4 0/1 5.State / 4 -- free 
				 5 0/1 5.State / 5 -- free 
				 6 0/1 5.State / 6 -- free 
				 7 0 ---   
		  
	6 			 0-6 0 – 7Fh XOR (Byte1–5) -- Checksum 
				 7 0 ---   
		  
	7 			 0-7 80h ETX -- End of Text

*- Comment to Bit0 and Bit1 in Byte5 : 
 
Bit0  |  Bit1  |  Limit for set value (Bit4/Byte4 active) 
  0   |   0    |            10% 
  1   |   0    |            15% 
  0   |   1    |            20% 
  1   |   1    |            25%
  
  
8. Start identification of the data packet 
	- Every packet starts with the start identification STX (C0h).
	- The fact that bit7 is 0 in every data byte of the packet provides that the STX can be detected correctly.  

9. Checksum 
	- The checksum is an XOR – combination of all data bytes and is transferred in a separate byte (Bit7 of the checksum is also 0 !).
	
10. End identification of the data packet 
	- Every packet ends with the end identification STX (80h). The fact that bit7 is 0 in every data 
	  byte of the packet provides that the ETX can be detected correctly.