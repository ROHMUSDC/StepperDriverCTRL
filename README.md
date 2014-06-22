StepperDriverCTRL
=================

Stepper Motor Driver Control Firmware (Q102)

This is the release version...but I do have plans to make future improvements...such as improving Continuous mode.

Evaluation Board Objective
To allow customers and applications support the ability to evaluate any of Rohm’s BD637xx High Performance and BD638xx Micro Stepping Motor Drivers in a quick and easy environment.  This document will describe the operation and functionality of the evaluation board.
Functionality
The evaluation board has two drivers installed, the BD63720 Clock-in and the BD63876 Parallel input type.  The BD63720 clock-driven High Performance Stepper Motor Driver is connected to an on-board LAPIS micro (ML610Q102) for demonstration purposes.
Each circuit can also be connected to a user’s external MCU to control the driver and operate the attached stepper motor.  Additionally, the BD63720 circuit can be controlled manually in a single step or continuous mode.  The BD63720 and BD63876 are the 2A versions of each type.  Users of lower current devices will be able to use these for evaluation as well.

Getting to know your Stepper Motor Eval Board
•	The RED Blocks in the Picture above highlight the key portions of the ROHM BD63720 Clock-in Stepper Motor Driver LSI (U5).  The green terminal Block (J2) is where you will add Power for the Stepper Motor (i.e.: 24Vdc & GND), as well as where you will connect the 4 legs of the Bi Polar Stepper Motor windings if you want this IC to drive the Motor. This motor driver LSI is designed to be controlled via on-board pre-programmed Lapis ML610Q102 mcu (Blue Box above) (U3), or able to control via external mcu provided by the user (J1)
•	The Green Blocks in the Picture above highlight the key portions of the ROHM BD63876 Parallel-in Stepper Motor Driver LSI (U6).  The green terminal Block (J7) is where you will add Power for the Stepper Motor (i.e.: 24Vdc & GND), as well as where you will connect the 4 legs of the Bi Polar Stepper Motor windings if you want this IC to drive the Motor. This motor driver LSI is designed to be controlled only via external mcu provided by the user (J6)
•	The ROHM (SML-21 Series) LED’s in the lower corner of the board are controlled via a ROHM BD8377 12-ch. LED Driver (U4), and are used to indicate which Machine State the eval board in currently in in (adjustable via the push button encoder knob (E1)): 
o	4 possible Step Modes (Full, Half-A, Half-B or Quarter Step) 
o	2 possible Driver States (Disabled or Enabled)
o	2 Possible Operation Modes (Single-Step or Continuous Mode)
o	2 Possible Pass-Thru States (Disabled or Enabled)
•	Suggested Motor
o	Portescap 23H118D10B Stepper Motor (1.8 Deg Step Angle; BiPolar)
o	Availible at DigiKey: http://www.digikey.com/product-detail/en/23H118D10B/403-1039-ND/1894182

•	Key Potentiometer & Switch Settings
o	MTH 	=> Current decay mode setting terminal (SLOW, MIXED or FAST Decay)
o	VREF 	=> Output current value setting terminal
o	CR	=> Terminal to set the Chopping Frequency of output
o	PS	=> Power Save terminal - can put circuit in standby mode state and allow motor _	     to operate 
o	J4 & J5	=> Both should jump pins 3 & 4 together to allow powering control circuitry (3.3V) on the eval board from the power provided in the USB connector (5V) which is regulated down to 3.3V via the ROHM BD3570HFP LDO Voltage Regulator (U2).
Evaluation Board Objective
To allow customers and applications support the ability to evaluate any of Rohm’s BD63720 & BD63876 High Performance Stepping Motor Drivers in a quick and easy environment. 
Functionality
The evaluation board has two drivers installed, the BD63720 Clock-in and the BD63876 Parallel input type. Each circuit can be connected to a user’s MCU to control the driver and operate the attached stepper motor. Additionally, the BD63720 circuit can be controlled manually in a single step or continuous mode. 
LED INDICATORS
•	Step Mode 	 	Indicates that the step modes can be selected by rotating the knob. 
o	Full Step 	 	Indicates that the Full Step Mode has been selected for manual mode 
o	Half Step A		Indicates that the Half Step A Mode has been selected for manual mode. 
o	Half Step B  	Indicates that the Half Step B Mode has been selected for manual mode. 
o	Quarter Step 	Indicates that the Quarter Step Mode has been selected for manual mode.
•	Driver Enabled - RED 	Indicates that the Manual mode is ready but currently disabled
•	Driver Enabled - GREEN	Indicates that the driver is now enabled for Manual mode 
•	Single Step Mode	 	Indicates that the board is prepared to operate in single step mode. 
•	Continuous 		 	Indicates that the board is prepared to operate in continuous mode. 
•	Pass Through - RED		Indicates that the Pass-Through mode has been selected but disabled
•	Pass Through - GREEN	Indicates that the Pass-Through mode is now enabled 
KEY Components
BD63720EFV - 36V Extra High-performance & High Reliability Stepper Motor Driver
The BD63720EFV is a bipolar low-consumption driver that is driven by PWM current. Rated power supply voltage of the device is 36 V, and rated output current is 2.0A. CLK-IN driving mode is adopted for input interface, and excitation mode is corresponding to FULL STEP mode, HALF STEP mode (2 types) and QUARTER STEP mode via a built-in DAC. In terms of current decay, the FAST DECAY/SLOW DECAY ratio may be set without any limitation, and all available modes may be controlled in the most appropriate way. In addition, the power supply may be driven by one single system, which simplifies the design.