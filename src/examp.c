//*****************************************************************************
// Program:	 Stepper Motor Driver IC Evaluation Board Controller (ML610Q102)
//		
// Author:	 C. Schell
//		 ROHM Semiconductor USA, LLC
//		 US Design Center
// Started:  June 19th, 2013
// Purpose:	 Demonstration Code for use with Stepper Motor Driver Board 
//
//
// Last Updated:	NOVEMBER 4th, 2013 
// Code Status:	Released Version - Working Well
//
// Tasks to Do:	Change Continuous mode to go from CCW-to-0-to-CW
//
// Known Bugs:	Only known bug...Once clock pulses faster 
//			than 390uS, then can only make narrower, not wider
//
//
//*****************************************************************************

//*****************************************************************************
// Q102 Microcontroller's Hardwired I/O Pins on the Stepper Motor Driver Board
//
// ================================ ML610Q102 ================================= 
//
// Pin-01 => RESET_N					Pin-16 => "Encoder_Input_CH_A"	- PA0
// Pin-02 => TEST						Pin-15 => "LED_Driver_CLK" 		- PB7
// Pin-03 => "ClkIn_Stepper_CLK" 	- PB0/PWMC	Pin-14 => "+3.3V_C"	 		- VDD
// Pin-04 => "ClkIn_Stepper_CW_CCW"  - PB1	Pin-13 => "GND"		 		- VSS
// Pin-05 => "ClkIn_Stepper_MODE_0"	- PB2		Pin-12 => "LED_Driver_LATCH" 		- PB6
// Pin-06 => "ClkIn_Stepper_MODE_1"	- PB3		Pin-11 => "LED_Driver_SERIN" 		- PB5
// Pin-07 => "Encoder_BUTTON" 	- PA2		Pin-10 => "ClkIn_Stepper_ENABLE"	- PB4 
// Pin-08 => VPP						Pin-09 => "Encoder_Input_CH_B" 	- PA1
//	
// ============================================================================
//
//*****************************************************************************

/*
=== Evaluation Board Objective ===
To allow customers and applications support the ability to evaluate any of Rohm’s BD63720 & BD63876 High 
Performance Stepping Motor Drivers in a quick and easy environment. 

=== Functionality ===
The evaluation board has two drivers installed, the BD63720 Clock-in and the BD63876 Parallel input type. Each
circuit can be connected to a user’s MCU to control the driver and operate the attached stepper motor. Additionally,
the BD63720 circuit can be controlled manually in a single step or continuous mode. 

=== Inputs and Outputs ===
Except for the Circuit Ground, the two circuits on the PCB will be completely separate so as not to back feed or cause
erroneous inputs to the operating circuit. There are four I/O types on the board: Terminal Block (TB), Test Point (TP),
Micro USB (MU) and Header (HDR). Terminal blocks are used to connect to power, ground and the motor. Test Points
are used on all of the I/O signals to allow for easy monitoring with an oscilloscope or meter or to allow quick connection
with a test lead. Header connections are used on the control inputs to allow an easy cable connection to an MCU.

Step Mode 		 Indicates that the step modes can be selected by rotating the knob. 
	Full Step 	 Indicates that the Full Step Mode has been selected for manual mode 
	Half Step A  Indicates that the Half Step A Mode has been selected for manual mode. 
	Half Step B  Indicates that the Half Step B Mode has been selected for manual mode. 
	Quarter Step Indicates that the Quarter Step Mode has been selected for manual mode.

Driver Enabled 	 Indicates that the Manual mode is ready and can be enabled 
(GREEN)		 Indicates that the driver is enabled for Manual mode 

Single Step Mode	 Indicates that the board is prepared to operate in single step mode. 
Continuous 		 Indicates that the board is prepared to operate in continuous mode. 

Pass Through	 Indicates that the Pass-Through mode has been selected 
(GREEN)		 Indicates that the Pass-Through mode is enabled 
*/


/*
BD63720EFV - 36V Extra High-performance & High Reliability Stepper Motor Driver

The BD63720EFV is a bipolar low-consumption driver that
driven by PWM current. Rated power supply voltage of
the device is 36 V, and rated output current is 2.0A.
CLK-IN driving mode is adopted for input interface, and
excitation mode is corresponding to FULL STEP mode,
HALF STEP mode (2 types) and QUARTER STEP
mode via a built-in DAC. In terms of current decay, the
FAST DECAY/SLOW DECAY ratio may be set without
any limitation, and all available modes may be
controlled in the most appropriate way. In addition, the
power supply may be driven by one single system,
which simplifies the design.

MTH 	=> Current decay mode setting terminal (SLOW, MIXED or FAST Decay)
VREF 	=> Output current value setting terminal
PS	=> Power Save terminal - can make circuit standby state and make motor output OPEN (when PS=LOW)
CR	=> Terminal to set the Chopping Frequency of output


*/


//***** START INCLUDE FILES **************************************************
//===========================================================================
// INCLUDED FILES...
//===========================================================================
// Include Path: common;main;irq;timer;clock;tbc;pwm;uart;

	#include	<ML610102.H>	// Lapis Micro ML610Q102 
	#include	<stdlib.h>		// General-purpose utilities
	#include 	<common.h>		// Common Definitions
	#include 	<irq.h>		// IRQ Definitions
	//#include 	<mcu.h>		// MCU Definition
	//#include 	<uart.h>		// UART Function Prototypes
	//#include	<i2c.h>		// I2C Definition
	//#include 	<clock.h>		// Set System Clock API
	//#include 	<tbc.h>		// Set TBC (Timer Based Clock) API
	//#include 	<timer.h>		// Timer Macros & APIs
	//#include 	<main.h>		// Clear WDT API

	#include	<ctype.h>		// Character classification and conversion 
	#include	<errno.h>		// Error identifiers Library
	#include	<float.h>		// Numerical limits for floating-point numbers
	#include	<limits.h>		// Numerical limits for integers
	#include	<math.h>		// Mathematical functions
	#include	<muldivu8.h>	// Multiplication and Division accelerator
	#include	<setjmp.h>		// Global jump (longjmp)
	#include	<signal.h>		// Signal handling functions
	#include	<stdarg.h>		// Variable numbers of arguments
	#include	<stddef.h>		// Standard types and macros 
	#include	<stdio.h>		// I/O-related processing
	#include	<string.h>		// Character string manipulation routines
	#include	<yfuns.h>		// 
	#include	<yvals.h>		// Called for by most Header Files
//===========================================================================
//***** END INCLUDE FILES ****************************************************


//***** START I/O PIN DATA ALIASES *******************************************
//===========================================================================
// I/O PIN DATA ALIASES...
//===========================================================================
	// Connections for Q102 to Control the Stepper Motor IC - ROHM P/N: BD63720EFV  
		#define ClkIn_Stepper_CLK    PB0D	// (pwm_output)
		#define ClkIn_Stepper_CW_CCW PB1D 
		#define ClkIn_Stepper_MODE_0 PB2D 
		#define ClkIn_Stepper_MODE_1 PB3D 
		#define ClkIn_Stepper_ENABLE PB4D

	// Connections for Q102 to Control the LED Driver - ROHM P/N: BD8377FV-M
		#define LED_Driver_SERIN   PB5D 
		#define LED_Driver_LATCH   PB6D
		#define LED_Driver_CLK 	   PB7D

	// Connections for Q102 to Read the Quadrature Encode & Push-Button: 
		#define Encoder_Input_CH_A PA0D 
		#define Encoder_Input_CH_B PA1D 
		#define Encoder_BUTTON	   PA2D

//===========================================================================
//***** END I/O PIN DATA ALIASES *********************************************


//***** START FUNCTION PROTOTYPES ********************************************
//===========================================================================
//   FUNCTION PROTOTYPES: 
//	Establishes the name and return type of a function and may specify the 
// 	types, formal parameter names and number of arguments to the function                                 
//===========================================================================
void main_clrWDT( void );			// no return value and no arguments
void Initialization( void );			// no return value and no arguments
void SetOSC( void );				// no return value and no arguments
void PortA_Low( void );				// no return value and no arguments
void PortB_Low( void );				// no return value and no arguments
void PortC_Low( void );				// no return value and no arguments
void PortA_Digital_Inputs( void );		// no return value and no arguments
void PinB0_PWM( void ); 			// no return value and no arguments

void NOPxxx( void );				// no return value and no arguments
void NOPyyy( void );				// no return value and no arguments
void NOP_ClkStep( void );			// no return value and no arguments
void NOP10uS( void );				// no return value and no arguments

void NOP_Long( void );				// no return value and no arguments
void Set_LEDs( void ); 				// no return value and no arguments
void ClockingPulse( void ); 			// no return value and no arguments
void ALL_LEDs_ON( void );			// no return value and no arguments
void ALL_LEDs_OFF( void );			// no return value and no arguments
void myClockingPulse ( void );

void LEDStateA  ( void ); 			// no return value and no arguments
void LEDStateA1 ( void ); 			// no return value and no arguments
void LEDStateA2 ( void ); 			// no return value and no arguments
void LEDStateA3 ( void ); 			// no return value and no arguments
void LEDStateA4 ( void ); 			// no return value and no arguments
void LEDStateB  ( void ); 			// no return value and no arguments
void LEDStateC  ( void ); 			// no return value and no arguments
void LEDStateC1 ( void ); 			// no return value and no arguments
void LEDStateC1A( void ); 			// no return value and no arguments
void LEDStateC2 ( void ); 			// no return value and no arguments
void LEDStateC2A( void ); 			// no return value and no arguments
void LEDStateD  ( void ); 			// no return value and no arguments
void LEDStateE  ( void ); 			// no return value and no arguments

void DisableController( void ); 		// no return value and no arguments
void EnableController( void ); 		// no return value and no arguments
void ButtonKnobCheck ( void ); 		// no return value and no arguments
void EncoderPostion   ( void ); 		// no return value and no arguments
void EncoderDirection ( void );		// no return value and no arguments
void MachineStateAction ( void );		// no return value and no arguments
void ZeroFrequency ( void );			// no return value and no arguments
void GetMode ( void );				// no return value and no arguments

void StepCW (void);				// no return value and no arguments
void StepCCW (void);				// no return value and no arguments
void ContinuousMode(void);			// no return value and no arguments
void MotorRampUp  (void);			// no return value and no arguments	


//===========================================================================
//***** END FUNCTION PROTOTYPES ***********************************************

//***** START GLOBAL VARIABLE DECLARATIONS ************************************
//===========================================================================
//GLOBAL VARIABLE DECLARATIONS......
//===========================================================================
//char char_a;		// -128 to 127
//unsigned char uchar;	// 0-255
//int inta, table [100];	// -32,768 to 32767
//unsigned int uint;	// 0 to 65,535
//long long_a,delay;	// -2,147,483,648 to 2,147,483,647
//float float_a;		// 1.17549435e-38 to 3.40282347e+38
//double double_a;		// 2.2250738585072014e-308 to 1.7976931348623157e+308

//Values for LED Controller Buffer
char SINGLE_LED,CONTINUOUS_LED,STEP_MODE_LED,PASS_THRU_RED_LED,PASS_THRU_GREEN_LED,DRIVER_EN_RED_LED;
char DRIVER_EN_GREEN_LED,FULL_STEP_LED, HALF_STEP_A_LED,HALF_STEP_B_LED,QUARTER_STEP_LED,BUFFER_OE_PIN; 
unsigned int mode;									// 0 to 65,535 	
unsigned int EncoderPosition, previous_encoder_position, button_flag;   // 0 to 65,535
unsigned int AbsolutePosition, OffsetPosition ; 	    			// 0 to 65,535
unsigned int direction, previousDirection; //  0=NO CHANGE, 1=CW, 2=CCW	   0 to 65,535

unsigned int Frequency; //For CONTINUOUS MODE					   0 to 65,535

unsigned char StartPosition;	// 0-255 
//===========================================================================
//***** END GLOBAL VARIABLE DECLARATIONS **************************************

	

//***** START MAIN FUNCTION **************************************************
//===========================================================================
//  	Start of MAIN FUNCTION
//===========================================================================
int main(void) {
							

PowerOnReset:
	// Set Q102 micro to desired initial state...
	Initialization(); 	//Oscillator, Ports, UART, Timers, Comparators, LEDs, etc.
	DisableController(); 	//Disable Controller at PowerOn or RESET

Start:
	EncoderPostion();    	//EncoderPosition: 	1-4 Quadrature Encoder position	
	previous_encoder_position = AbsolutePosition; 
	

MainLoop:
	ButtonKnobCheck();  	//Check for BUTTION Press & KNOB Rotation	
	MachineStateAction();	//Act based on Machine State...

goto MainLoop;

}  //END main Function
//===========================================================================
//***** END MAIN FUNCTION *****************************************************



//*****************************************************************************
//===========================================================================
//	Initialize Q102 Micro to Desired State...
//===========================================================================
static void Initialization(void){

	//Initialize Peripherals	
		//BLKCON2 Control Bits...Manually Set 
			DUA0  = 1; // 0=> Enables the operation of UART0 (initial value).
	
	
		BLKCON4 = 0x00; // 0=> Enables SA type ADC
		BLKCON6 = 0x00; // (1=disables; 0=enables) the operation of Timers 8, 9, A, E, F.
		BLKCON7 = 0x01; // (1=disables; 0=enables) the operation of PWM (PWMC)

	// Port Initialize
		PortA_Low();	//Initialize all 3 Ports of Port A to GPIO-Low
		PortB_Low();	//Initialize all 8 Ports of Port B to GPIO-Low

	//Setup Quadrature Encoder Inputs on A0, A1 & A2
		PortA_Digital_Inputs();

	// PWM...	
		//PinB0_PWM();	// Set up PWM Pin on B.0...(PWMC)


	// Comparator...	
		//analog_comparator();

	// Set Oscillator Rate...Must Have.
     		SetOSC();

    
	// TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT
	// TIMER SETUP...

	Setup_Timer_8:
	// Reset TIMER DATA REGISTER...
		TM8D    = 0;	//Timer 8 DATA Register

	// Reset TIMER CLOCK REGISTER...
		TM8C    = 0;	//Timer 8 CLOCK Register

	// TIMER-8 Control...
	//   CONTROL-0 Register:
		// Operation Clock for Timer...
		T8C1 = 0;	// 01 = HTBCLK  
		T8C0 = 1;
		// Count Mode...
		T89M16 = 0;	// 0=8-Bit Mode; 1=16bit Mode...
		//One-Shot or Normal Mode...
		T8OST = 0;	// 0=Normal; 1=One-Shot...
	//   CONTROL-1 Register:
		// RUN Mode...
		T8RUN = 0;	//0=STOP; 1=START...

	// TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT


	
	// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
	// INTERRUPT SETUP...
		//irq_di();	// Disable Interrupts
		//irq_wdtint();	// Initialize Interrupts (All Off and NO Requests)

		// INTERRUPT ENABLE REGISTERS...
		//  IE0 = VOLTAGE LEVEL SUPERVISOR Int.
		//  IE1 = EXTERNAL Ints on B1, B0, A2, A1, & A0
		//  IE2 = SUCCESSIVE APPROXIMATION Int.
		//  IE3 = TIMERS 8 & 9 Ints.
		//  IE4 = UART & COMPARATOR Ints.
		//  IE5 = TIMERS A, B, E & F Ints.
		//  IE6 = PWMC &  128Hz & 32Hz TBC Ints.
		//  IE7 = 16Hz & 2Hz TBC Ints.
		IE0 = IE1 = IE2 = IE3 = IE4 = IE5 = IE6 = IE7 = 0;

		// INTERRUPT REQUEST REGISTERS...
		//  IRQ0 = WDT & VLS Int Requests
		//  IRQ1 = EXTERNAL Int Requests
		//  IRQ2 = SUCCESSIVE APPROXIMATION Int Requests
		//  IRQ3 = TIMERS 8 & 9 Int Requests 
		//  IRQ4 = UART & COMPARATOR Int Requests 
		//  IRQ5 = TIMERS A, B, E & F Int Requests 
		//  IRQ6 = PWMC &  128Hz & 32Hz TBC Int Requests 
		//  IRQ7 = 16Hz & 2Hz TBC Int Requests 
		IRQ0 = IRQ1 = IRQ2 = IRQ3 = IRQ4 = IRQ5 = IRQ6 = IRQ7 = 0;


		E2H = 0; 	// E2H is the Enable flag for 2Hz TBC Interrupt (1=ENABLED)
		
		
		//(void)irq_setHdr( (unsigned char)IRQ_NO_I2C0INT, _intI2c );
		//(void)irq_setHdr( (unsigned char)IRQ_NO_UA1INT, _intUart );
		//(void)irq_setHdr( (unsigned char)IRQ_NO_UA0INT, _intUart );

		//EUA0 = 1; // EUA0 is the enable flag for the UART0 interrupt (1=ENABLED)
		//irq_ei(); // Enable Interrupts
	// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII


	//LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
	//SET LED's & BUFFER_OE to INITIAL STATE...
	//12-bits from output of LED Driver BD8377FV-M (D0-D11):
		STEP_MODE_LED 		= 1; // Bit D2  => "STEP MODE" LED 		(Orange)
	  	FULL_STEP_LED 		= 0; // Bit D7  => "FULL STEP" LED		(YELLOW)
	  	HALF_STEP_A_LED 		= 0; // Bit D8  => "HALF STEP A" LED	(YELLOW)
	  	HALF_STEP_B_LED 		= 0; // Bit D9  => "HALF STEP B" LED	(YELLOW)
	  	QUARTER_STEP_LED 		= 0; // Bit D10 => "QUARTER STEP" LED	(YELLOW)

		DRIVER_EN_RED_LED 	= 0; // Bit D6  => "DRIVER ENABLED" LED  	(RED)
		DRIVER_EN_GREEN_LED	= 0; // Bit D5  => "DRIVER ENABLED" LED 	(GREEN)

	  	SINGLE_LED 			= 0; // Bit D0  => "SINGLE" LED 		(Orange)
	  	CONTINUOUS_LED 		= 0; // Bit D1  => "CONTINUOUS" LED 	(Orange)

		PASS_THRU_GREEN_LED 	= 0; // Bit D3  => "PASS THRU" LED		(GREEN)
		PASS_THRU_RED_LED 	= 0; // Bit D4  => "PASS THRU" LED		(RED)

		BUFFER_OE_PIN 		= 0; // Bit D11 => Octal Buffer_OE

		Set_LEDs();		// Send 12-Bit Data to BD8377 Driver, Clock & Latch
	//LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL


	mode = 1; 					//Initialize "mode" to Step-MODE
	direction = 0;				//Initialize "direction" to 0
	AbsolutePosition = 0;			//Initialize "AbsolutePosition " to 0
	OffsetPosition = 0;			//Initialize "OffsetPosition " to 0
	button_flag = 0;				//Initialize "button_flag" to 0
	Frequency = 0;				//Initialize "Frequency " to 0

	// WDT...
	WDTMOD = 0x03; 	// 0x03=overflow 8sec...
	main_clrWDT(); 	// Clear WDT
	
} //END Initialization Function
//*****************************************************************************


//*****************************************************************************
//===========================================================================
// 	Clear the Watch-Dog-Timer
//===========================================================================
void main_clrWDT( void )
{
	//How to clear the Watch Dog Timer:
	// => Write alternately 0x5A and 0xA5 into WDTCON register
	do {
		WDTCON = 0x5Au;
	} while (WDP != 1);
	WDTCON = 0xA5u;
} //END main_clrWDT Function
//*****************************************************************************


//*****************************************************************************
//===========================================================================
//	OSC set
//===========================================================================
static void SetOSC(void){

	//FCON0: 			// xMHz PLL (3=1MHz; 2=2MHz; 1=4MHz; 0=8MHz)...
	SYSC0  = 0;			// Used to select the frequency of the HSCLK => 00=8.192MHz.
	SYSC1  = 0;

	OSCM1  = 1;			// 10 => Built-in PLL oscillation mode
	OSCM0  = 0;
   	
	ENOSC  = 1;			// 1=Enable High Speed Oscillator...MUST ENABLE before setting SYSTEM CLOCK!
	SYSCLK = 1;			// 1=HSCLK; 0=LSCLK (MUST set ENOSC = 1 first) 

	LPLL   = 1;			// 1=Enables the use of PLL oscillation - ADDED 4/30/2013

	__EI();			// INT enable
} //END SetOSC Function
//*****************************************************************************


//*****************************************************************************
//===========================================================================
//	Clear All 3 Bits of Port A
//===========================================================================
void PortA_Low(void){

//Carl's Notes...

//Step 1: Set Pin Direction...
//Step 2: Set Pin I/O Type...
//Step 3: Set Pin Purpose...
//Step 4: Set Pin Data...

	//Direction...	
	PA0DIR = 0;		// PortA Bit0 set to Output Mode...
	PA1DIR = 0;		// PortA Bit1 set to Output Mode...
	PA2DIR = 0;		// PortA Bit2 set to Output Mode...

	//I/O Type...
	PA0C1  = 1;		// PortA Bit0 set to CMOS Output...
	PA0C0  = 1;		
	PA1C1  = 1;		// PortA Bit1 set to CMOS Output...
	PA1C0  = 1;	
	PA2C1  = 1;		// PortA Bit2 set to CMOS Output...
	PA2C0  = 1;	

	//Purpose...
	PA0MD1  = 0;	// PortA Bit0 set to General Purpose Output...
	PA0MD0  = 0;	
	PA1MD1  = 0;	// PortA Bit1 set to General Purpose Output...
	PA1MD0  = 0;	
	PA2MD1  = 0;	// PortA Bit2 set to General Purpose Output...
	PA2MD0  = 0;	

	//Data...
	PA0D = 0;		// A.0 Output OFF....
	PA1D = 0;		// A.1 Output OFF....
	PA2D = 0;		// A.2 Output OFF....

}  //END PortA_Low Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Clear All 8 Bits of Port B
//===========================================================================
void PortB_Low(void){

//Carl's Notes...

//Step 1: Set Pin Direction...
//Step 2: Set Pin I/O Type...
//Step 3: Set Pin Purpose...
//Step 4: Set Pin Data...

	//Direction...	
	PB0DIR = 0;		// PortB Bit0 set to Output Mode...
	PB1DIR = 0;		// PortB Bit1 set to Output Mode...
	PB2DIR = 0;		// PortB Bit2 set to Output Mode...
	PB3DIR = 0;		// PortB Bit3 set to Output Mode...
	PB4DIR = 0;		// PortB Bit4 set to Output Mode...
	PB5DIR = 0;		// PortB Bit5 set to Output Mode...
	PB6DIR = 0;		// PortB Bit6 set to Output Mode...
	PB7DIR = 0;		// PortB Bit7 set to Output Mode...

	//I/O Type...
	PB0C1  = 1;		// PortB Bit0 set to CMOS Output...
	PB0C0  = 1;		
	PB1C1  = 1;		// PortB Bit1 set to CMOS Output...
	PB1C0  = 1;	
	PB2C1  = 1;		// PortB Bit2 set to CMOS Output...
	PB2C0  = 1;	
	PB3C1  = 1;		// PortB Bit3 set to CMOS Output...
	PB3C0  = 1;		
	PB4C1  = 1;		// PortB Bit4 set to CMOS Output...
	PB4C0  = 1;	
	PB5C1  = 1;		// PortB Bit5 set to CMOS Output...
	PB5C0  = 1;	
	PB6C1  = 1;		// PortB Bit6 set to CMOS Output...
	PB6C0  = 1;	
	PB7C1  = 1;		// PortB Bit7 set to CMOS Output...
	PB7C0  = 1;	

	//Purpose...
	PB0MD1  = 0;	// PortB Bit0 set to General Purpose Output...
	PB0MD0  = 0;	
	PB1MD1  = 0;	// PortB Bit1 set to General Purpose Output...
	PB1MD0  = 0;	
	PB2MD1  = 0;	// PortB Bit2 set to General Purpose Output...
	PB2MD0  = 0;	
	PB3MD1  = 0;	// PortB Bit3 set to General Purpose Output...
	PB3MD0  = 0;	
	PB4MD1  = 0;	// PortB Bit4 set to General Purpose Output...
	PB4MD0  = 0;	
	PB5MD1  = 0;	// PortB Bit5 set to General Purpose Output...
	PB5MD0  = 0;
	PB6MD1  = 0;	// PortB Bit6 set to General Purpose Output...
	PB6MD0  = 0;	
	PB7MD1  = 0;	// PortB Bit7 set to General Purpose Output...
	PB7MD0  = 0;

	//Data...
	PB0D = 0;		// B.0 Output OFF....
	PB1D = 0;		// B.1 Output OFF....
	PB2D = 0;		// B.2 Output OFF....
	PB3D = 0;		// B.3 Output OFF....
	PB4D = 0;		// B.4 Output OFF....
	PB5D = 0;		// B.5 Output OFF....
	PB6D = 0;		// B.6 Output OFF....
	PB7D = 0;		// B.7 Output OFF....

}  //END PortB_Low Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Set All 3 Bits of Port A as Digital Input Pins
//===========================================================================
void PortA_Digital_Inputs(void){

//Carl's Notes...

//Step 1: Set Pin Direction...
//Step 2: Set Pin I/O Type...
//Step 3: Set Pin Purpose...
//Step 4: Set Pin Data...

	//Direction...	
	PA0DIR = 1;		// PortA Bit0 set to Input Mode...
	PA1DIR = 1;		// PortA Bit1 set to Input Mode...
	PA2DIR = 1;		// PortA Bit2 set to Input Mode...


	//I/O Type...
	PA0C1  = 1;		// PortA Bit0 set to Input with Pull-Up Resistor...
	PA0C0  = 0;		
	PA1C1  = 1;		// PortA Bit1 set to Input with Pull-Up Resistor...
	PA1C0  = 0;	
	PA2C1  = 1;		// PortA Bit2 set to Input with Pull-Up Resistor...
	PA2C0  = 0;	

	//Purpose...
	PA0MD1  = 0;	// PortA Bit0 set to General Purpose I/O...
	PA0MD0  = 0;	
	PA1MD1  = 0;	// PortA Bit1 set to General Purpose I/O...
	PA1MD0  = 0;	
	PA2MD1  = 0;	// PortA Bit2 set to General Purpose I/O...
	PA2MD0  = 0;	


}  //END PortA_Digital_Inputs Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	PWM Output on Port B - Pin 0 (PWMC)
//===========================================================================
void PinB0_PWM(void){

//Carl's Notes...

//Step 1: Set Pin Direction...
//Step 2: Set Pin I/O Type...
//Step 3: Set Pin Purpose...
//Step 4: Select the Clock Mode...
//Step 5: Set the Duty Cycle...
//Step 5: Start the PWM Counter...

//The PWM signals with the periods of approximately 122 ns (@PLLCLK=16.384MHz) to 2s (@LSCLK=32.768kHz)
//  can be generated and output outside of this micro!

	//Direction...	
	PB0DIR = 0;		// PortB Bit0 set to Output Mode...

	//I/O Type...
	PB0C1  = 1;		// PortB Bit0 set to CMOS Output...
	PB0C0  = 1;		

	//Purpose...
	PB0MD1  = 0;	// PortB Bit0 set to PWM Output (0,1)...
	PB0MD0  = 1;	

	//Select the Clock Mode...
	PCCS1 = 0;	//00= LS; 01=HS; 10=PLL
	PCCS0 = 1;

	//SET THE PERIOD...(Added June 4th, 2013)
	PWCP = 4250;		// Init Period to (1=255kHz; 10=46kHz; 50=10kHz; 200=2.5kH; ; 3185 = 160Hz; 3400=150Hz; 4250=120Hz; 5000=102Hz)

	//SET THE DUTY CYCLE...(Added June 15th, 2013)
	//PWCD =    10;		//10    ~  0.2  % duty cycle @ 120Hz
	//PWCD =   100;		//100   ~  2.4  % duty cycle @ 120Hz
	//PWCD =  1000;		//1000  ~ 23.5  % duty cycle @ 120Hz
	//PWCD =  4000;		//4000  ~ 94.0  % duty cycle @ 120Hz
	//PWCD =  4150;		//4150  ~ 99.0  % duty cycle @ 120Hz
	//PWCD =    20;		//20    ~  0.4  % duty cycle @ 120Hz	
	PWCD =    12;		//12    ~  0.25 % duty cycle @ 160Hz

	PCRUN = 0;		// OFF to start

}  //END PinB0_PWM Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Analog Comparator setup
//===========================================================================
void analog_comparator(void){

//Carl's Notes...

//Step 1: Select the Interrupt Mode
// 	a.) Interrupt Disabled      => CMPxE1 = 0; CMPxE0 = 0;	  
// 	b.) Falling-Edge Int. Mode  => CMPxE1 = 0; CMPxE0 = 1;
// 	c.) Rising-Edge Int. Mode   => CMPxE1 = 1; CMPxE0 = 0;
// 	d.) Both-Edge Int. Mode     => CMPxE1 = 1; CMPxE0 = 1;


//Step 2: Enable the Comparator                       => CMPxEN = 1;	

//Step 3: Wait 3ms to allow Comparator to stabilize

//Step 4: Read the comparison result			=> CMPxD: 0= +<-; 1= +>-

//Step 5: Disable the Comparator				=> CMPxEN = 0;	


   //Comparator 0...
	CMP0EN  = 0x01; 	// Comparator ON...
	CMP0E1  = 0x00; 	// No Interupt...
	CMP0E0  = 0x00;
	CMP0SM1 = 0x00; 	// Detect without Sampling... 
	CMP0RFS = 0x01; 	// Differential Input on B5

   //Comparator 0 OFF
	CMP0EN  = 0x00;


} //END analog_comparator Function
//*****************************************************************************


//*****************************************************************************
//===========================================================================
//	Turn ALL LED's ON for Testing via ROHM BD8377 Driver...
//===========================================================================
void ALL_LEDs_ON(void){

	//12-bits from output of LED Driver BD8377FV-M (D0-D11):
	STEP_MODE_LED 		= 1; // Bit D2  => "STEP MODE" LED 		(Orange)
	  FULL_STEP_LED 		= 1; // Bit D7  => "FULL STEP" LED		(YELLOW)
	  HALF_STEP_A_LED 	= 1; // Bit D8  => "HALF STEP A" LED	(YELLOW)
	  HALF_STEP_B_LED 	= 1; // Bit D9  => "HALF STEP B" LED	(YELLOW)
	  QUARTER_STEP_LED 	= 1; // Bit D10 => "QUARTER STEP" LED	(YELLOW)

	DRIVER_EN_RED_LED 	= 1; // Bit D6  => "DRIVER ENABLED" LED  	(RED)
	DRIVER_EN_GREEN_LED	= 1; // Bit D5  => "DRIVER ENABLED" LED 	(GREEN)

	  SINGLE_LED 		= 1; // Bit D0  => "SINGLE" LED 		(Orange)
	  CONTINUOUS_LED 		= 1; // Bit D1  => "CONTINUOUS" LED 	(Orange)

	PASS_THRU_GREEN_LED 	= 1; // Bit D3  => "PASS THRU" LED		(GREEN)
	PASS_THRU_RED_LED 	= 1; // Bit D4  => "PASS THRU" LED		(RED)

	BUFFER_OE_PIN 		= 0; // Bit D11 => Octal Buffer_OE

	Set_LEDs();	//Send 12-Bit Data to BD8377 Driver, Clock & Latch

} //END ALL_LEDs_ON Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Turn ALL LED's OFF for Testing via ROHM BD8377 Driver...
//===========================================================================
void ALL_LEDs_OFF(void){

	//12-bits from output of LED Driver BD8377FV-M (D0-D11):
	//STEP_MODE_LED 		= 0; // Bit D2  => "STEP MODE" LED 		(Orange)
	  //FULL_STEP_LED 		= 0; // Bit D7  => "FULL STEP" LED		(YELLOW)
	  //HALF_STEP_A_LED 	= 0; // Bit D8  => "HALF STEP A" LED	(YELLOW)
	  //HALF_STEP_B_LED 	= 0; // Bit D9  => "HALF STEP B" LED	(YELLOW)
	  //QUARTER_STEP_LED 	= 0; // Bit D10 => "QUARTER STEP" LED	(YELLOW)

	DRIVER_EN_RED_LED 	= 0; // Bit D6  => "DRIVER ENABLED" LED  	(RED)
	DRIVER_EN_GREEN_LED	= 0; // Bit D5  => "DRIVER ENABLED" LED 	(GREEN)

	  SINGLE_LED 		= 0; // Bit D0  => "SINGLE" LED 		(Orange)
	  CONTINUOUS_LED 		= 0; // Bit D1  => "CONTINUOUS" LED 	(Orange)

	PASS_THRU_GREEN_LED 	= 0; // Bit D3  => "PASS THRU" LED		(GREEN)
	PASS_THRU_RED_LED 	= 0; // Bit D4  => "PASS THRU" LED		(RED)

	BUFFER_OE_PIN 		= 0; // Bit D11 => Octal Buffer_OE

	Set_LEDs();	//Send 12-Bit Data to BD8377 Driver, Clock & Latch

} //END ALL_LEDs_OFF Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Simple Delay - xxx NOPs...
//===========================================================================
void NOPxxx( void ){

unsigned int ONCNT = 0;

	while(ONCNT < 4000) {	// NOP for xxx Cycles ~700ns
		ONCNT++;
	}
	ONCNT = 0;			// Reset Counter 
}  //END NOPxxx Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Simple Delay - yyy NOPs...
//===========================================================================
void NOPyyy( void ){

unsigned int ONCNT = 0;

	while(ONCNT < 10) {	// NOP for yyy Cycles ~XXXns
		ONCNT++;
	}
	ONCNT = 0;			// Reset Counter 
}  //END NOPxxx Function
//*****************************************************************************


//*****************************************************************************
//===========================================================================
//	Simple Delay - xxx NOPs...
//===========================================================================
void NOP_ClkStep( void ){

unsigned int ONCNT = 0;

	while(ONCNT < 35000) {	// NOP for xxx Cycles ~x ns
		ONCNT++;
	}
	ONCNT = 0;			// Reset Counter 
}  //END NOP_ClkStep Function
//*****************************************************************************


//*****************************************************************************
//===========================================================================
//	Simple Delay - xxx NOPs...
//===========================================================================
void NOP10uS( void ){
unsigned int i;

	for (i = 0; i < 40; i++)		//was 4
	{
		NOPxxx();
	}

}  //END NOP10uSFunction
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Longer Delay - 5,000 NOPs...Used for LED Latch
//===========================================================================
void NOP_Long( void )
{
unsigned int ONCNT = 0;

	while(ONCNT < 5000) {	// NOP for 5,000 Cycles
		ONCNT++;
	}
	ONCNT = 0;			// Reset Counter 
}  //END NOPxxx Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Turn SET the 11-LEDs & 1-Buffer Enable Bit via ROHM BD8377 Driver...
//===========================================================================
void Set_LEDs(void){

//Carl's Notes...

	//PB5 LED_Driver_SERIN
	//PB6 LED_Driver_LATCH
	//PB7 LED_Driver_CLK 

//========= START OF TRANSMISSION... ==========================================================
	LED_Driver_LATCH = 0;					// Set LATCH LOW...

			LED_Driver_SERIN = BUFFER_OE_PIN;		// Set Data 1 or 0 (ON/OFF)...Bit 11
		myClockingPulse();
			LED_Driver_SERIN = QUARTER_STEP_LED;	// Set Data 1 or 0 (ON/OFF)...Bit 10
		myClockingPulse();
			LED_Driver_SERIN = HALF_STEP_B_LED;		// Set Data 1 or 0 (ON/OFF)...Bit 9	
		myClockingPulse();
			LED_Driver_SERIN = HALF_STEP_A_LED;		// Set Data 1 or 0 (ON/OFF)...Bit 8
		myClockingPulse();
			LED_Driver_SERIN = FULL_STEP_LED;		// Set Data 1 or 0 (ON/OFF)...Bit 7	
		myClockingPulse();
			LED_Driver_SERIN = DRIVER_EN_RED_LED;	// Set Data 1 or 0 (ON/OFF)...Bit 6
		myClockingPulse();
			LED_Driver_SERIN = DRIVER_EN_GREEN_LED;	// Set Data 1 or 0 (ON/OFF)...Bit 5
		myClockingPulse();
			LED_Driver_SERIN = PASS_THRU_RED_LED;	// Set Data 1 or 0 (ON/OFF)...Bit 4	
		myClockingPulse();
			LED_Driver_SERIN = PASS_THRU_GREEN_LED;	// Set Data 1 or 0 (ON/OFF)...Bit 3	
		myClockingPulse();
			LED_Driver_SERIN = STEP_MODE_LED;		// Set Data 1 or 0 (ON/OFF)...Bit 2	
		myClockingPulse();
			LED_Driver_SERIN = CONTINUOUS_LED;		// Set Data 1 or 0 (ON/OFF)...Bit 1	
		myClockingPulse();
			LED_Driver_SERIN = SINGLE_LED;		// Set Data 1 or 0 (ON/OFF)...Bit 0	
		myClockingPulse();

	LED_Driver_LATCH = 1;					// Set LATCH High to end transmission...
			NOP_Long();
	main_clrWDT(); 	// Clear WDT
//========= END OF TRANSMISSION... ============================================================

} //END Set_LEDs Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Turn Clocking Pulse to send Data via ROHM BD8377 Driver...
//===========================================================================
void myClockingPulse(void){

	//PB5 LED_Driver_SERIN
	//PB6 LED_Driver_LATCH
	//PB7 LED_Driver_CLK
		//NOPxxx();
	LED_Driver_CLK = 0;		// Set CLK Low to Start...
		NOPyyy();
	LED_Driver_CLK = 1;		// Cycle Clk HIGH...
		NOPyyy();
} //END ClockingPulse Function
//*****************************************************************************


//*****************************************************************************
//===========================================================================
//	Set LEDs to "State A" via ROHM BD8377 Driver...
//===========================================================================
void LEDStateA (void){

	//12-bits from output of LED Driver BD8377FV-M (D0-D11):
	STEP_MODE_LED 		= 1; // Bit D2  => "STEP MODE" LED 		(Orange)
	  FULL_STEP_LED 		= 0; // Bit D7  => "FULL STEP" LED		(YELLOW)
	  HALF_STEP_A_LED 	= 0; // Bit D8  => "HALF STEP A" LED	(YELLOW)
	  HALF_STEP_B_LED 	= 0; // Bit D9  => "HALF STEP B" LED	(YELLOW)
	  QUARTER_STEP_LED 	= 0; // Bit D10 => "QUARTER STEP" LED	(YELLOW)

	DRIVER_EN_RED_LED 	= 0; // Bit D6  => "DRIVER ENABLED" LED  	(RED)
	DRIVER_EN_GREEN_LED	= 0; // Bit D5  => "DRIVER ENABLED" LED 	(GREEN)

	  SINGLE_LED 		= 0; // Bit D0  => "SINGLE" LED 		(Orange)
	  CONTINUOUS_LED 		= 0; // Bit D1  => "CONTINUOUS" LED 	(Orange)

	PASS_THRU_GREEN_LED 	= 0; // Bit D3  => "PASS THRU" LED		(GREEN)
	PASS_THRU_RED_LED 	= 0; // Bit D4  => "PASS THRU" LED		(RED)

	BUFFER_OE_PIN 		= 0; // Bit D11 => Octal Buffer_OE

  Set_LEDs();	// Send 12-Bit Data to BD8377 Driver, Clock & Latch

} //END LEDStateA Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Set LEDs to "State A1" via ROHM BD8377 Driver...
//===========================================================================
void LEDStateA1 (void){

	//12-bits from output of LED Driver BD8377FV-M (D0-D11):
	STEP_MODE_LED 		= 1; // Bit D2  => "STEP MODE" LED 		(Orange)
	  FULL_STEP_LED 		= 1; // Bit D7  => "FULL STEP" LED		(YELLOW)
	  HALF_STEP_A_LED 	= 0; // Bit D8  => "HALF STEP A" LED	(YELLOW)
	  HALF_STEP_B_LED 	= 0; // Bit D9  => "HALF STEP B" LED	(YELLOW)
	  QUARTER_STEP_LED 	= 0; // Bit D10 => "QUARTER STEP" LED	(YELLOW)

	DRIVER_EN_RED_LED 	= 0; // Bit D6  => "DRIVER ENABLED" LED  	(RED)
	DRIVER_EN_GREEN_LED	= 0; // Bit D5  => "DRIVER ENABLED" LED 	(GREEN)

	  SINGLE_LED 		= 0; // Bit D0  => "SINGLE" LED 		(Orange)
	  CONTINUOUS_LED 		= 0; // Bit D1  => "CONTINUOUS" LED 	(Orange)

	PASS_THRU_GREEN_LED 	= 0; // Bit D3  => "PASS THRU" LED		(GREEN)
	PASS_THRU_RED_LED 	= 0; // Bit D4  => "PASS THRU" LED		(RED)

	BUFFER_OE_PIN 		= 0; // Bit D11 => Octal Buffer_OE

  Set_LEDs();	// Send 12-Bit Data to BD8377 Driver, Clock & Latch

} //END LEDStateA1 Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Set LEDs to "State A2" via ROHM BD8377 Driver...
//===========================================================================
void LEDStateA2 (void){

	//12-bits from output of LED Driver BD8377FV-M (D0-D11):
	STEP_MODE_LED 		= 1; // Bit D2  => "STEP MODE" LED 		(Orange)
	  FULL_STEP_LED 		= 0; // Bit D7  => "FULL STEP" LED		(YELLOW)
	  HALF_STEP_A_LED 	= 1; // Bit D8  => "HALF STEP A" LED	(YELLOW)
	  HALF_STEP_B_LED 	= 0; // Bit D9  => "HALF STEP B" LED	(YELLOW)
	  QUARTER_STEP_LED 	= 0; // Bit D10 => "QUARTER STEP" LED	(YELLOW)

	DRIVER_EN_RED_LED 	= 0; // Bit D6  => "DRIVER ENABLED" LED  	(RED)
	DRIVER_EN_GREEN_LED	= 0; // Bit D5  => "DRIVER ENABLED" LED 	(GREEN)

	  SINGLE_LED 		= 0; // Bit D0  => "SINGLE" LED 		(Orange)
	  CONTINUOUS_LED 		= 0; // Bit D1  => "CONTINUOUS" LED 	(Orange)

	PASS_THRU_GREEN_LED 	= 0; // Bit D3  => "PASS THRU" LED		(GREEN)
	PASS_THRU_RED_LED 	= 0; // Bit D4  => "PASS THRU" LED		(RED)

	BUFFER_OE_PIN 		= 0; // Bit D11 => Octal Buffer_OE

  Set_LEDs();	// Send 12-Bit Data to BD8377 Driver, Clock & Latch

} //END LEDStateA2 Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Set LEDs to "State A3" via ROHM BD8377 Driver...
//===========================================================================
void LEDStateA3 (void){

	//12-bits from output of LED Driver BD8377FV-M (D0-D11):
	STEP_MODE_LED 		= 1; // Bit D2  => "STEP MODE" LED 		(Orange)
	  FULL_STEP_LED 		= 0; // Bit D7  => "FULL STEP" LED		(YELLOW)
	  HALF_STEP_A_LED 	= 0; // Bit D8  => "HALF STEP A" LED	(YELLOW)
	  HALF_STEP_B_LED 	= 1; // Bit D9  => "HALF STEP B" LED	(YELLOW)
	  QUARTER_STEP_LED 	= 0; // Bit D10 => "QUARTER STEP" LED	(YELLOW)

	DRIVER_EN_RED_LED 	= 0; // Bit D6  => "DRIVER ENABLED" LED  	(RED)
	DRIVER_EN_GREEN_LED	= 0; // Bit D5  => "DRIVER ENABLED" LED 	(GREEN)

	  SINGLE_LED 		= 0; // Bit D0  => "SINGLE" LED 		(Orange)
	  CONTINUOUS_LED 		= 0; // Bit D1  => "CONTINUOUS" LED 	(Orange)

	PASS_THRU_GREEN_LED 	= 0; // Bit D3  => "PASS THRU" LED		(GREEN)
	PASS_THRU_RED_LED 	= 0; // Bit D4  => "PASS THRU" LED		(RED)

	BUFFER_OE_PIN 		= 0; // Bit D11 => Octal Buffer_OE

  Set_LEDs();	// Send 12-Bit Data to BD8377 Driver, Clock & Latch

} //END LEDStateA3 Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Set LEDs to "State A4" via ROHM BD8377 Driver...
//===========================================================================
void LEDStateA4 (void){

	//12-bits from output of LED Driver BD8377FV-M (D0-D11):
	STEP_MODE_LED 		= 1; // Bit D2  => "STEP MODE" LED 		(Orange)
	  FULL_STEP_LED 		= 0; // Bit D7  => "FULL STEP" LED		(YELLOW)
	  HALF_STEP_A_LED 	= 0; // Bit D8  => "HALF STEP A" LED	(YELLOW)
	  HALF_STEP_B_LED 	= 0; // Bit D9  => "HALF STEP B" LED	(YELLOW)
	  QUARTER_STEP_LED 	= 1; // Bit D10 => "QUARTER STEP" LED	(YELLOW)

	DRIVER_EN_RED_LED 	= 0; // Bit D6  => "DRIVER ENABLED" LED  	(RED)
	DRIVER_EN_GREEN_LED	= 0; // Bit D5  => "DRIVER ENABLED" LED 	(GREEN)

	  SINGLE_LED 		= 0; // Bit D0  => "SINGLE" LED 		(Orange)
	  CONTINUOUS_LED 		= 0; // Bit D1  => "CONTINUOUS" LED 	(Orange)

	PASS_THRU_GREEN_LED 	= 0; // Bit D3  => "PASS THRU" LED		(GREEN)
	PASS_THRU_RED_LED 	= 0; // Bit D4  => "PASS THRU" LED		(RED)

	BUFFER_OE_PIN 		= 0; // Bit D11 => Octal Buffer_OE

  Set_LEDs();	// Send 12-Bit Data to BD8377 Driver, Clock & Latch

} //END LEDStateA4 Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Set LEDs to "State B" via ROHM BD8377 Driver...
//===========================================================================
void LEDStateB (void){

	//12-bits from output of LED Driver BD8377FV-M (D0-D11):
	//STEP_MODE_LED 		= 0; // Bit D2  => "STEP MODE" LED 		(Orange)
	  //FULL_STEP_LED 	= 0; // Bit D7  => "FULL STEP" LED		(YELLOW)
	  //HALF_STEP_A_LED 	= 0; // Bit D8  => "HALF STEP A" LED	(YELLOW)
	  //HALF_STEP_B_LED 	= 0; // Bit D9  => "HALF STEP B" LED	(YELLOW)
	  //QUARTER_STEP_LED 	= 0; // Bit D10 => "QUARTER STEP" LED	(YELLOW) 	

	DRIVER_EN_RED_LED 	= 1; // Bit D6  => "DRIVER ENABLED" LED  	(RED)
	DRIVER_EN_GREEN_LED	= 0; // Bit D5  => "DRIVER ENABLED" LED 	(GREEN)

	  SINGLE_LED 		= 0; // Bit D0  => "SINGLE" LED 		(Orange)
	  CONTINUOUS_LED 		= 0; // Bit D1  => "CONTINUOUS" LED 	(Orange)

	PASS_THRU_GREEN_LED 	= 0; // Bit D3  => "PASS THRU" LED		(GREEN)
	PASS_THRU_RED_LED 	= 0; // Bit D4  => "PASS THRU" LED		(RED)

	BUFFER_OE_PIN 		= 0; // Bit D11 => Octal Buffer_OE

  Set_LEDs();	// Send 12-Bit Data to BD8377 Driver, Clock & Latch

} //END LEDStateB Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Set LEDs to "State C" via ROHM BD8377 Driver...
//===========================================================================
void LEDStateC (void){

	//12-bits from output of LED Driver BD8377FV-M (D0-D11):
	//STEP_MODE_LED 		= 0; // Bit D2  => "STEP MODE" LED 		(Orange)
	  //FULL_STEP_LED 	= 0; // Bit D7  => "FULL STEP" LED		(YELLOW)
	  //HALF_STEP_A_LED 	= 0; // Bit D8  => "HALF STEP A" LED	(YELLOW)
	  //HALF_STEP_B_LED 	= 0; // Bit D9  => "HALF STEP B" LED	(YELLOW)
	  //QUARTER_STEP_LED 	= 0; // Bit D10 => "QUARTER STEP" LED	(YELLOW)

	DRIVER_EN_RED_LED 	= 0; // Bit D6  => "DRIVER ENABLED" LED  	(RED)
	DRIVER_EN_GREEN_LED	= 1; // Bit D5  => "DRIVER ENABLED" LED 	(GREEN)

	  SINGLE_LED 		= 0; // Bit D0  => "SINGLE" LED 		(Orange)
	  CONTINUOUS_LED 		= 0; // Bit D1  => "CONTINUOUS" LED 	(Orange)

	PASS_THRU_GREEN_LED 	= 0; // Bit D3  => "PASS THRU" LED		(GREEN)
	PASS_THRU_RED_LED 	= 0; // Bit D4  => "PASS THRU" LED		(RED)

	BUFFER_OE_PIN 		= 0; // Bit D11 => Octal Buffer_OE

  Set_LEDs();	// Send 12-Bit Data to BD8377 Driver, Clock & Latch

} //END LEDStateC Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Set LEDs to "State C1" via ROHM BD8377 Driver...
//===========================================================================
void LEDStateC1 (void){

	//12-bits from output of LED Driver BD8377FV-M (D0-D11):
	//STEP_MODE_LED 		= 0; // Bit D2  => "STEP MODE" LED 		(Orange)
	  //FULL_STEP_LED 	= 0; // Bit D7  => "FULL STEP" LED		(YELLOW)
	  //HALF_STEP_A_LED 	= 0; // Bit D8  => "HALF STEP A" LED	(YELLOW)
	  //HALF_STEP_B_LED 	= 0; // Bit D9  => "HALF STEP B" LED	(YELLOW)
	  //QUARTER_STEP_LED 	= 0; // Bit D10 => "QUARTER STEP" LED	(YELLOW)

	DRIVER_EN_RED_LED 	= 0; // Bit D6  => "DRIVER ENABLED" LED  	(RED)
	DRIVER_EN_GREEN_LED	= 1; // Bit D5  => "DRIVER ENABLED" LED 	(GREEN)

	  SINGLE_LED 		= 1; // Bit D0  => "SINGLE" LED 		(Orange)
	  CONTINUOUS_LED 		= 0; // Bit D1  => "CONTINUOUS" LED 	(Orange)

	PASS_THRU_GREEN_LED 	= 0; // Bit D3  => "PASS THRU" LED		(GREEN)
	PASS_THRU_RED_LED 	= 0; // Bit D4  => "PASS THRU" LED		(RED)

	BUFFER_OE_PIN 		= 0; // Bit D11 => Octal Buffer_OE

  Set_LEDs();	// Send 12-Bit Data to BD8377 Driver, Clock & Latch

} //END LEDStateC1 Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Set LEDs to "State C1A" via ROHM BD8377 Driver...
//===========================================================================
void LEDStateC1A (void){

	//12-bits from output of LED Driver BD8377FV-M (D0-D11):
	//STEP_MODE_LED 		= 0; // Bit D2  => "STEP MODE" LED 		(Orange)
	  //FULL_STEP_LED 	= 0; // Bit D7  => "FULL STEP" LED		(YELLOW)
	  //HALF_STEP_A_LED 	= 0; // Bit D8  => "HALF STEP A" LED	(YELLOW)
	  //HALF_STEP_B_LED 	= 0; // Bit D9  => "HALF STEP B" LED	(YELLOW)
	  //QUARTER_STEP_LED 	= 0; // Bit D10 => "QUARTER STEP" LED	(YELLOW)

	DRIVER_EN_RED_LED 	= 0; // Bit D6  => "DRIVER ENABLED" LED  	(RED)
	DRIVER_EN_GREEN_LED	= 1; // Bit D5  => "DRIVER ENABLED" LED 	(GREEN)

	  //TURN OFF "SINGLE_LED" for Flashing Effect...
	  SINGLE_LED 		= 0; // Bit D0  => "SINGLE" LED 		(Orange) 
	  CONTINUOUS_LED 		= 0; // Bit D1  => "CONTINUOUS" LED 	(Orange)

	PASS_THRU_GREEN_LED 	= 0; // Bit D3  => "PASS THRU" LED		(GREEN)
	PASS_THRU_RED_LED 	= 0; // Bit D4  => "PASS THRU" LED		(RED)

	BUFFER_OE_PIN 		= 0; // Bit D11 => Octal Buffer_OE

  Set_LEDs();	// Send 12-Bit Data to BD8377 Driver, Clock & Latch

} //END LEDStateC1A Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Set LEDs to "State C2" via ROHM BD8377 Driver...
//===========================================================================
void LEDStateC2 (void){

	//12-bits from output of LED Driver BD8377FV-M (D0-D11):
	//STEP_MODE_LED 		= 0; // Bit D2  => "STEP MODE" LED 		(Orange)
	  //FULL_STEP_LED 	= 0; // Bit D7  => "FULL STEP" LED		(YELLOW)
	  //HALF_STEP_A_LED 	= 0; // Bit D8  => "HALF STEP A" LED	(YELLOW)
	  //HALF_STEP_B_LED 	= 0; // Bit D9  => "HALF STEP B" LED	(YELLOW)
	  //QUARTER_STEP_LED 	= 0; // Bit D10 => "QUARTER STEP" LED	(YELLOW)

	DRIVER_EN_RED_LED 	= 0; // Bit D6  => "DRIVER ENABLED" LED  	(RED)
	DRIVER_EN_GREEN_LED	= 1; // Bit D5  => "DRIVER ENABLED" LED 	(GREEN)

	  SINGLE_LED 		= 0; // Bit D0  => "SINGLE" LED 		(Orange)
	  CONTINUOUS_LED 		= 1; // Bit D1  => "CONTINUOUS" LED 	(Orange)

	PASS_THRU_GREEN_LED 	= 0; // Bit D3  => "PASS THRU" LED		(GREEN)
	PASS_THRU_RED_LED 	= 0; // Bit D4  => "PASS THRU" LED		(RED)

	BUFFER_OE_PIN 		= 0; // Bit D11 => Octal Buffer_OE

  Set_LEDs();	// Send 12-Bit Data to BD8377 Driver, Clock & Latch

} //END LEDStateC2 Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Set LEDs to "State C2A" via ROHM BD8377 Driver...
//===========================================================================
void LEDStateC2A (void){

	//12-bits from output of LED Driver BD8377FV-M (D0-D11):
	//STEP_MODE_LED 		= 0; // Bit D2  => "STEP MODE" LED 		(Orange)
	  //FULL_STEP_LED 	= 0; // Bit D7  => "FULL STEP" LED		(YELLOW)
	  //HALF_STEP_A_LED 	= 0; // Bit D8  => "HALF STEP A" LED	(YELLOW)
	  //HALF_STEP_B_LED 	= 0; // Bit D9  => "HALF STEP B" LED	(YELLOW)
	  //QUARTER_STEP_LED 	= 0; // Bit D10 => "QUARTER STEP" LED	(YELLOW)

	DRIVER_EN_RED_LED 	= 0; // Bit D6  => "DRIVER ENABLED" LED  	(RED)
	DRIVER_EN_GREEN_LED	= 1; // Bit D5  => "DRIVER ENABLED" LED 	(GREEN)

	  //TURN OFF "CONTINUOUS_LED" for Flashing Effect...
	  SINGLE_LED 		= 0; // Bit D0  => "SINGLE" LED 		(Orange)
	  CONTINUOUS_LED 		= 0; // Bit D1  => "CONTINUOUS" LED 	(Orange)

	PASS_THRU_GREEN_LED 	= 0; // Bit D3  => "PASS THRU" LED		(GREEN)
	PASS_THRU_RED_LED 	= 0; // Bit D4  => "PASS THRU" LED		(RED)

	BUFFER_OE_PIN 		= 0; // Bit D11 => Octal Buffer_OE

  Set_LEDs();	// Send 12-Bit Data to BD8377 Driver, Clock & Latch

} //END LEDStateC2A Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Set LEDs to "State D" via ROHM BD8377 Driver...
//===========================================================================
void LEDStateD (void){

	//12-bits from output of LED Driver BD8377FV-M (D0-D11):
	STEP_MODE_LED 		= 0; // Bit D2  => "STEP MODE" LED 		(Orange)
	  FULL_STEP_LED 		= 0; // Bit D7  => "FULL STEP" LED		(YELLOW)
	  HALF_STEP_A_LED 	= 0; // Bit D8  => "HALF STEP A" LED	(YELLOW)
	  HALF_STEP_B_LED 	= 0; // Bit D9  => "HALF STEP B" LED	(YELLOW)
	  QUARTER_STEP_LED 	= 0; // Bit D10 => "QUARTER STEP" LED	(YELLOW)

	DRIVER_EN_RED_LED 	= 0; // Bit D6  => "DRIVER ENABLED" LED  	(RED)
	DRIVER_EN_GREEN_LED	= 0; // Bit D5  => "DRIVER ENABLED" LED 	(GREEN)

	  SINGLE_LED 		= 0; // Bit D0  => "SINGLE" LED 		(Orange)
	  CONTINUOUS_LED 		= 0; // Bit D1  => "CONTINUOUS" LED 	(Orange)

	PASS_THRU_GREEN_LED 	= 0; // Bit D3  => "PASS THRU" LED		(GREEN)
	PASS_THRU_RED_LED 	= 1; // Bit D4  => "PASS THRU" LED		(RED)

	BUFFER_OE_PIN 		= 0; // Bit D11 => Octal Buffer_OE

  Set_LEDs();	// Send 12-Bit Data to BD8377 Driver, Clock & Latch

} //END LEDStateD Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Set LEDs to "State E" via ROHM BD8377 Driver...
//===========================================================================
void LEDStateE (void){

	//12-bits from output of LED Driver BD8377FV-M (D0-D11):
	STEP_MODE_LED 		= 0; // Bit D2  => "STEP MODE" LED 		(Orange)
	  FULL_STEP_LED 		= 0; // Bit D7  => "FULL STEP" LED		(YELLOW)
	  HALF_STEP_A_LED 	= 0; // Bit D8  => "HALF STEP A" LED	(YELLOW)
	  HALF_STEP_B_LED 	= 0; // Bit D9  => "HALF STEP B" LED	(YELLOW)
	  QUARTER_STEP_LED 	= 0; // Bit D10 => "QUARTER STEP" LED	(YELLOW)

	DRIVER_EN_RED_LED 	= 0; // Bit D6  => "DRIVER ENABLED" LED  	(RED)
	DRIVER_EN_GREEN_LED	= 0; // Bit D5  => "DRIVER ENABLED" LED 	(GREEN)

	  SINGLE_LED 		= 0; // Bit D0  => "SINGLE" LED 		(Orange)
	  CONTINUOUS_LED 		= 0; // Bit D1  => "CONTINUOUS" LED 	(Orange)

	PASS_THRU_GREEN_LED 	= 1; // Bit D3  => "PASS THRU" LED		(GREEN)
	PASS_THRU_RED_LED 	= 0; // Bit D4  => "PASS THRU" LED		(RED)

	BUFFER_OE_PIN 		= 0; // Bit D11 => Octal Buffer_OE

  Set_LEDs();	// Send 12-Bit Data to BD8377 Driver, Clock & Latch

} //END LEDStateE Function
//*****************************************************************************


//*****************************************************************************
//===========================================================================
//	Disable Controller at Power ON or RESET...
//===========================================================================
void DisableController(void){
	// Pin 20 of the BD63720 Stepper Motor Driver IC is the ENABLE pin
	// ...setting this pin LOW, disables IC
	// 
	// When ENABLE=L, input to CLK is blocked, and phase advance operation 
	// ...of internal translator circuit is stopped
	//// When ENABLE=H, IC is ACTIVE

	  
	//Clear STEP LED's at Disable
	FULL_STEP_LED 	= 0; // Bit D7  => "FULL STEP" LED		(YELLOW)
	HALF_STEP_A_LED 	= 0; // Bit D8  => "HALF STEP A" LED	(YELLOW)
	HALF_STEP_B_LED 	= 0; // Bit D9  => "HALF STEP B" LED	(YELLOW)
	QUARTER_STEP_LED 	= 0; // Bit D10 => "QUARTER STEP" LED	(YELLOW)

	ClkIn_Stepper_ENABLE = 0;	//Set PIN LOW to DISABLE.

} //END DisableController Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	Enable Controller After Button Press while in Driver Enabled Mode...
//===========================================================================
void EnableController(void){
	// Pin 20 of the BD63720 Stepper Motor Driver IC is the ENABLE pin
	// ...setting this pin LOW, disables IC
	// 
	// When ENABLE=L, input to CLK is blocked, and phase advance operation 
	// ...of internal translator circuit is stopped
	//// When ENABLE=H, IC is ACTIVE

	ClkIn_Stepper_ENABLE = 1;	//Set PIN HIGH to ENABLE.

} //END EnableController Function
//*****************************************************************************

//*****************************************************************************
//===========================================================================
//	ZERO FREQUENCY in CONTINUOUS Mode...
//===========================================================================
void ZeroFrequency(void){

//Place holder routine for now...

	Frequency = 0;	//Set Frequency back to zero.

} //END ZeroFrequency Function
//*****************************************************************************







//***** START Encoder Button AND/OR KNOB Check *******************************
//===========================================================================
//	Check Encoder for BUTTION Press & KNOB Rotation...
//===========================================================================
void ButtonKnobCheck (void){	
//int myLoop,i;
	//--------------------------
	// Now Check Check Quadrature Encoder Button State...
	//  Default state is High...
	//  If Low, Button is Pressed...
	if (Encoder_BUTTON == 0) 		// Encoder Button Pressed?			
	{
		button_flag = 1;	//Set Flag to indicate Button was pressed

		//delay at least 50ms...and more if needed - short button debounce...
		do 
		{
			ALL_LEDs_OFF();		// Turns LEDs off at Startup / RESET
			NOPxxx(); //Extra Debounce time...
		}while (Encoder_BUTTON == 0);
		
	}//endif "Encoder Button Pressed?" 

	
	//Actions based on Button Press...
	if (button_flag == 1)
	{
		button_flag = 0; //First, Clear Flag!

		//If button is pressed...do this...based on MODE
		switch (mode) //...and if previous position was ONE, then direction is xxx  
		{
		case 0:
			DisableController(); //DISABLE & 
			mode = 1;		   //Default to STEP MODE (#1)	
			break;
		case 1:
			mode = 14;		//Jump down to PASS THRU RED (#14)
			break;
		case 2:
			mode = 6;		//Move on to Driver ENABLE-RED (#6)
			break;
		case 3:
			mode = 6;		//Move on to Driver ENABLE-RED (#6)		
			break;
		case 4:
			mode = 6;		//Move on to Driver ENABLE-RED (#6)
			break;
		case 5:
			mode = 6;		//Move on to Driver ENABLE-RED (#6)
			break;
		case 6:
			DisableController(); //DISABLE & 
			mode = 14;		   //Move to PASS THRU - RED (#14)
			break;
		case 7:	
			mode = 8;		//Move to SINGLE STEP MODE (#8)
			break;
		case 8:
			mode = 11;		//Go onto CONTINUOUS Mode (#11)
			break;
		case 9:
			mode = 11;		//Go onto CONTINUOUS Mode (#11)
			break;
		case 10:
			mode = 11;		//Go onto CONTINUOUS Mode (#11)
			break;
		case 11:
			mode = 7;		//Return to DRIVER-ENABLE-GREEN (#7)
			break;
		case 12:
 			DisableController(); //DISABLE & 
			ZeroFrequency();	   //Zero the Frequency & Return to Continuous Mode (#11)
			mode = 11;
			break;
		case 13:
			DisableController(); //DISABLE & 
			ZeroFrequency();	   //Zero the Frequency & Return to Continuous Mode (#11)
			mode = 11;	
			break;
		case 14:
			mode = 1;		//Return to STEP MODE (#1)
			break;
		case 15:
			mode = 14;		//Return to PASS THRU RED (#14)
			break;
		default:	
			mode = 1;		//Default to STEP MODE (#1)
			break;
		}//End "mode" Switch
		
	}//EndIf button_flag = 0


	//GET KNOB DIRECTION...
	EncoderDirection();	//direction; //0=NO CHANGE, 1=CW, 2=CCW, 3=Unknown

	//Actions based on Button Press...
	if (direction != 0)
	{
		//If KNOB IS TURNED...do this...based on MODE
		switch (mode) //...and if previous position was ONE, then direction is xxx  
		{
		case 0:			//No Action at "0"
			break;
		case 1:
			switch (direction ) 
			{
			case 1:		//CW Rotation...
				mode++;
			if (mode > 5)
			{
				mode = 2;
			}
			if (mode < 2)
			{
			 	mode = 5;
			}
				break;
			case 2:		//CCW Rotation...
				mode--;
			if (mode > 5)
			{
				mode = 2;
			}
			if (mode < 2)
			{
			 	mode = 5;
			}
			break;
			}//End "direction" Switch
			break;

		case 2:
			switch (direction ) 
			{
			case 1:		//CW Rotation...
				mode++;
			if (mode > 5)
			{
				mode = 2;
			}
			if (mode < 2)
			{
			 	mode = 5;
			}
				break;
			case 2:		//CCW Rotation...
				mode--;
			if (mode > 5)
			{
				mode = 2;
			}
			if (mode < 2)
			{
			 	mode = 5;
			}
			break;
			}//End "direction" Switch		
			break;

		case 3:						//CAN SIMPLIFY this particular case...
			switch (direction ) 
			{
			case 1:		//CW Rotation...
				mode++;
			if (mode > 5)
			{
				mode = 2;
			}
			if (mode < 2)
			{
			 	mode = 5;
			}
				break;
			case 2:		//CCW Rotation...
				mode--;
			if (mode > 5)
			{
				mode = 2;
			}
			if (mode < 2)
			{
			 	mode = 5;
			}
			break;
			}//End "direction" Switch			
			break;

		case 4:						//CAN SIMPLIFY this particular case...
			switch (direction ) 
			{
			case 1:		//CW Rotation...
				mode++;
			if (mode > 5)
			{
				mode = 2;
			}
			if (mode < 2)
			{
			 	mode = 5;
			}
				break;
			case 2:		//CCW Rotation...
				mode--;
			if (mode > 5)
			{
				mode = 2;
			}
			if (mode < 2)
			{
			 	mode = 5;
			}
			break;
			}//End "direction" Switch			
			break;

		case 5:
			switch (direction ) 
			{
			case 1:		//CW Rotation...
				mode++;
			if (mode > 5)
			{
				mode = 2;
			}
			if (mode < 2)
			{
			 	mode = 5;
			}
				break;
			case 2:		//CCW Rotation...
				mode--;
			if (mode > 5)
			{
				mode = 2;
			}
			if (mode < 2)
			{
			 	mode = 5;
			}
			break;
			}//End "direction" Switch			
			break;

		case 6:
			mode = 7;	//Rotation in either direction moves to Driver-Enable-GREEN (#7)
			break;

		case 7:	
			mode = 6;	//Rotation in either direction moves back to Driver-Enable-RED (#6)
			break;

		case 8:				//SINGLE-STEP MODE...

			switch (direction ) 
			{
			case 1:			//CW Rotation...
				//FlashCW();		//Flash LEDs for User Feedback for CW Step
		
				//for (myLoop=0; myLoop<49; myLoop++){
				//EncoderPostion();
				StepCW();		//Step CW 1-Step
				//}

				//mode = 8;		//Return to Mode 8
				break;
			case 2:			//CCW Rotation...
				//FlashCCW();		//Flash LEDs for User Feedback for CCW Step

				//for (myLoop=0; myLoop<49; myLoop++){
				//EncoderPostion();
				StepCCW();		//Step CCW 1-Step
				//}

				//mode = 8;		//Return to Mode 8 
				break;

			default:
				//for (i=0; i<3; i++){
				//	FlashCW();		//multiple flashes to indicate ERROR (Moved encoder too fast!)
				//}
				break;
				
			}//End "direction" Switch

		
			break;

		case 9: 	//CASE 9 & 10 REDUNDANT - NEVER GETS HERE...
			switch (direction ) 
			{
			case 1:		//CW Rotation...
				//mode = 10;
				break;
			case 2:		//CCW Rotation...
				//mode = 8;
				break;
			}//End "direction" Switch			
			break;

		case 10:	//CASE 9 & 10 REDUNDANT - NEVER GETS HERE...
			switch (direction ) 
			{
			case 1:		//CW Rotation...
				//mode = 10;
				break;
			case 2:		//CCW Rotation...
				//mode = 8;
				break;
			}//End "direction" Switch			
			break;

		case 11:
			switch (direction ) 
			{
			case 1:		//CW Rotation...
				//FlashContCW();		//Flash LEDs for User Feedback for CW Step
				ContinuousMode();
				//mode = 12;
				break;
			case 2:		//CCW Rotation...
				//FlashContCCW();		//Flash LEDs for User Feedback for CCW Step
				ContinuousMode();
				//mode = 13;
				break;
			}//End "direction" Switch			
			break;


		case 12:	//Never gets here...
			switch (direction ) 
			{
			case 1:		//CW Rotation...increments Frequency
				mode = 12;
				break;
			case 2:		//CCW Rotation...decrements Frequency
				mode = 13;
				break;
			}//End "direction" Switch			
			break;

		case 13:	//Never gets here...
			switch (direction ) 
			{
			case 1:		//CW Rotation...increments Frequency
				mode = 12;
				break;
			case 2:		//CCW Rotation...decrements Frequency
				mode = 13;
				break;
			}//End "direction" Switch			
			break;

		case 14:
			switch (direction ) 
			{
			case 1:		//CW Rotation...move to PASS-THRU-GREEN (#15)
				mode = 15;
				break;
			case 2:		//CCW Rotation...move to PASS-THRU-RED (#14)
				mode = 14;
				break;
			}//End "direction" Switch			
			break;

		case 15:
			switch (direction ) 
			{
			case 1:		//CW Rotation...keeps at PASS-THRU-GREEN (#15)
				mode = 15;
				break;
			case 2:		//CCW Rotation...move back to PASS-THRU-RED (#14)
				mode = 14;
				break;
			}//End "direction" Switch			
			break;

		default:	
			mode =1; //Return to 1 when lost...
			break;
		}//End "mode" then "directiion" Switch	
	}

 
} //END ButtonKnobCheck  Function
//===========================================================================
//***** END Encoder Button AND/OR KNOB Check *********************************


//***** START Encoder Position ***********************************************
//===========================================================================
//	Read and Monitor Encoder Postion 
//===========================================================================
void EncoderPostion (void) {

//This development board uses a Quadrature Encoder (with button press feature)
// from Grayhill (Part#: 62P22-L6) to Control the Stepper Motor Driver Eval Board.

	// Decode position (1-4) via the encoder inputs (Quadrature Encoder Inputs A&B)...
	if (Encoder_Input_CH_A == 0) 			// Encoder Input "CH-A" on A0; "CH-B" on A1			
	{
		if (Encoder_Input_CH_B == 0)		// STEP #1 => If Ch.A=0 & Ch.B=0...
		{
			AbsolutePosition = 1;
		}
		else						// STEP #4 => Ch.A=1 & Ch.B=0...
		{
			AbsolutePosition = 4;
		}	

	} //endif 
	else					
	{
		if (Encoder_Input_CH_B == 1)		// STEP #3 => Ch.A=1 & Ch.B=1...
		{
			AbsolutePosition = 3;
		}
		else						// STEP #2 => If Ch.A=0 & Ch.B=1... 
		{
			AbsolutePosition = 2;
		}
	} //end else

	//EncoderPosition = (AbsolutePosition - OffsetPosition);

	//if (EncoderPosition > 4) {EncoderPosition = 4;} //Ensure Encoder value is always in range...   	
	//if (EncoderPosition < 1) {EncoderPosition = 1;} 


} //END EncoderPostion Function
//===========================================================================
//***** END Encoder Position *************************************************



//***** START Encoder Direction  *********************************************
//===========================================================================
//	Read and Monitor Encoder Direction  
//===========================================================================
void EncoderDirection(void){

	EncoderPostion();

	//direction; //0=NO CHANGE, 1=CW, 2=CCW, 3=Unknown
	//direction = 0;	//Assume no change...

	if (AbsolutePosition == previous_encoder_position)
	{
		direction = 0;	//No change...
		goto NoChange;
	} 

	//Store Previous Direction to help decode mis-steps...
	previousDirection = direction;
 	

	switch (AbsolutePosition) 
	{
	case 1:	//If Current EncoderPosition = 1
		switch (previous_encoder_position ) //...and if previous position was xxx, then direction is xxx  
		{
		case 2:
			direction =  2; 	// 2=CCW
			break;
		//case 3:
			//direction =  3; 	// 3=Unknown
			//break;
		case 4:
			direction =  1; 	// 1=CW
			break;
		//default:	
			//direction =  3; 	// 3=Unknown
			//break;
		}
		break;

	case 2:	//Current EncoderPosition = 2
		switch (previous_encoder_position ) //...and if previous position was xxx, then direction is xxx  
		{
		case 1:
			direction =  1; 	// 1=CW
			break;
		case 3:
			direction =  2; 	// 2=CCW 
			break;
		//case 4:
		//	direction =  3; 	// 3=Unknown
		//	break; 
		//default:	
		//	direction =  3; 	// 3=Unknown
		//	break;
		//break; !!!
		}
		break;
		
	case 3:	//Current EncoderPosition = 3
		switch (previous_encoder_position ) //...and if previous position was xxx, then direction is xxx  
		{
		//case 1:
		//	direction =  3; 	// 3=Unknown
		//	break; 
		case 2:
			direction =  1; 	// 1=CW
			break; 
		case 4:
			direction =  2; 	// 2=CCW
			break;  
		//default:	
		//	direction =  3; 	// 3=Unknown
		//	break;  
		}
		break;

	case 4:	//Current EncoderPosition = 4
		switch (previous_encoder_position ) //...and if previous position was xxx, then direction is xxx  
		{
		case 1:
			direction =  2; 	// 2=CCW
			break;
		//case 2:
		//	direction =  3; 	// 3=Unknown
		//	break;
		case 3:
			direction =  1; 	// 1=CW
			break;
		//default:	
		//	direction =  3; 	// 3=Unknown
		//	break;
		}
		break;

	//default:	
		//direction =  3; 		// 3=Unknown
		//break;
	}//

	
	//Assume direction is the same as the last known direction, if direction is unknown...
	if (direction ==  3){ 		// 3=Unknown => 1=CW
		direction = previousDirection;
	}

NoChange:

	previous_encoder_position = AbsolutePosition; //STORE CURRENT POSITION

return;

} //END EncoderDirection
//===========================================================================
//***** START Encoder Direction  *********************************************









//***** START Step CW ********************************************************
//===========================================================================
//	STEP 1 CW Step
//===========================================================================
void StepCW (void){		//Working 12 September 2013, 7pm...

unsigned int j, ClkHighCounter, ClkLowCounter, HowManyClockCycles, CWClkPlseWidth;	// 0 to 65,535

CWClkPlseWidth = 2000;; // <=========CHANGE THIS VALUE TO OPTIMIZE for particular motor!!! was 1000

	main_clrWDT();

	//-------------------------------------------------------------------------------------------------
	//Set Direction
		ClkIn_Stepper_CW_CCW = 0;				// PB1D - 0=CW; 1=CCW

	// Start with Clock LOW & Enable LOW & then Clear Timers, etc...
		ClkIn_Stepper_CLK    	= 0;				// Start with Clock Low! 
		ClkIn_Stepper_ENABLE 	= 0;				// PB4D - 1=>ENABLED; 0=>DISABLED
		ClkHighCounter 		= 0;				// Zero the counter...
		ClkLowCounter 		= 0;				// Zero the counter...

	//-------------------------------------------------------------------------------------------------

	//Get Mode to count needed Clock Cycles...
		if (ClkIn_Stepper_MODE_0 == 0 & ClkIn_Stepper_MODE_1 == 0)
		{
			HowManyClockCycles = 1; 	// FULL STEP 	=> 4 CLK Cycles = Electrical Angle 360
		}	
	
		if (ClkIn_Stepper_MODE_0 == 1 & ClkIn_Stepper_MODE_1 == 0)
		{
			HowManyClockCycles = 1; 	// HALF STEP-A 	=> 8 CLK Cycles = Electrical Angle 360
		}

		if (ClkIn_Stepper_MODE_0 == 0 & ClkIn_Stepper_MODE_1 == 1)
		{
			HowManyClockCycles = 1; 	// HALF STEP-B 	=> 8 CLK Cycles = Electrical Angle 360
		}

		if (ClkIn_Stepper_MODE_0 == 1 & ClkIn_Stepper_MODE_1 == 1)
		{
			HowManyClockCycles = 1; 	// QUARTER STEP 	=> 16 CLK Cycles = Electrical Angle 360
		}


		//NOW ENABLE CONTROLLER!
			ClkIn_Stepper_ENABLE = 1;			// PB4D - 1=>ENABLED; 0=>DISABLED			
	
		//And Send Clock Pulse(s)...	
		for (j=0; j<HowManyClockCycles ; j++)		// Desired number of Clock pulses...
		{
		   
			ClkIn_Stepper_CLK   = 1;			// PB0D - The Electrical angle advances by one for each CLK input

				while(ClkHighCounter < CWClkPlseWidth ) {		// NOP for 250 Cycles => 400uS
				ClkHighCounter++;
				}
				ClkHighCounter= 0;


			ClkIn_Stepper_CLK = 0;				// PB0D - The Electrical angle advances by one for each CLK input

				while(ClkLowCounter < CWClkPlseWidth) {		// NOP for 250 Cycles => 400uS
				ClkLowCounter++;
				}
				ClkLowCounter= 0;
	
		}
	
		//NOW DISABLE CONTROLLER!
		   	ClkIn_Stepper_ENABLE = 0;			// PB4D - 1=>ENABLED; 0=>DISABLED

} //END StepCW  Function
//===========================================================================
//***** END Step CW **********************************************************

//***** START Step CCW *******************************************************
//===========================================================================
//	STEP 1 CCW Step
//===========================================================================
void StepCCW (void){		//Working 12 September 2013...

unsigned int j,ClkHighCounter, ClkLowCounter, HowManyClockCycles, CCWClkPlseWidth ;	// 0 to 65,535

CCWClkPlseWidth = 2000; // <=========CHANGE THIS VALUE TO OPTIMIZE!!!  was 1000

	main_clrWDT();

	//-------------------------------------------------------------------------------------------------
	//Set Direction
		ClkIn_Stepper_CW_CCW = 1;				// PB1D - 0=CW; 1=CCW

	// Start with Clock LOW & Enable LOW & then Clear Timers, etc...
		ClkIn_Stepper_CLK    	= 0;				// Start with Clock Low! 
		ClkIn_Stepper_ENABLE 	= 0;				// PB4D - 1=>ENABLED; 0=>DISABLED
		ClkHighCounter 		= 0;				// Zero the counter...
		ClkLowCounter 		= 0;				// Zero the counter...

	//-------------------------------------------------------------------------------------------------

	//Get Mode to count needed Clock Cycles...
		if (ClkIn_Stepper_MODE_0 == 0 & ClkIn_Stepper_MODE_1 == 0)
		{
			HowManyClockCycles = 1; 	// FULL STEP 	=> 4 CLK Cycles = Electrical Angle 360
		}	
	
		if (ClkIn_Stepper_MODE_0 == 1 & ClkIn_Stepper_MODE_1 == 0)
		{
			HowManyClockCycles = 1; 	// HALF STEP-A 	=> 8 CLK Cycles = Electrical Angle 360
		}

		if (ClkIn_Stepper_MODE_0 == 0 & ClkIn_Stepper_MODE_1 == 1)
		{
			HowManyClockCycles = 1; 	// HALF STEP-B 	=> 8 CLK Cycles = Electrical Angle 360
		}

		if (ClkIn_Stepper_MODE_0 == 1 & ClkIn_Stepper_MODE_1 == 1)
		{
			HowManyClockCycles = 1; 	// QUARTER STEP 	=> 16 CLK Cycles = Electrical Angle 360
		}


		//NOW ENABLE CONTROLLER!
			ClkIn_Stepper_ENABLE = 1;			// PB4D - 1=>ENABLED; 0=>DISABLED			
	
		//And Send Clock Pulse(s)...	
		for (j=0; j<HowManyClockCycles ; j++)		// Desired number of Clock pulses...
		{
		   
			ClkIn_Stepper_CLK   = 1;			// PB0D - The Electrical angle advances by one for each CLK input

				while(ClkHighCounter < CCWClkPlseWidth ) {	// NOP for 2000 Cycles => 3164uS
				ClkHighCounter++;
				}
				ClkHighCounter= 0;


			ClkIn_Stepper_CLK = 0;				// PB0D - The Electrical angle advances by one for each CLK input

				while(ClkLowCounter < CCWClkPlseWidth) {		// NOP for 2000 Cycles => 3164uS
				ClkLowCounter++;
				}
				ClkLowCounter= 0;
	
		}
	
		//NOW DISABLE CONTROLLER!
		   	ClkIn_Stepper_ENABLE = 0;			// PB4D - 1=>ENABLED; 0=>DISABLED

} //END StepCCW  Function
//===========================================================================
//***** END Step CCW *********************************************************



//***** START Continuous Mode ************************************************
//===========================================================================
//	Adjust Continuous Mode Frequency in CW & CCW Directions...
//===========================================================================
void ContinuousMode(void){

unsigned int counter, ClockingCount;		// 0 to 65,535
long LongCounter; 

	main_clrWDT();

		//MOTOR DIRECTION IS CARRIED THROUGH FROM SINGLE STEP MODE...

		ClkIn_Stepper_ENABLE = 0;			// PB4D - 1=>ENABLED; 0=>DISABLED
		ClockingCount = 0;				// Initial ClockingCount value (was 375...)


	//CONTINUOUS MODE
	while(Encoder_BUTTON == 1 ){  	//While Not Pushed... 
ContModeHolding:
			EncoderDirection();		//direction; //0=NO CHANGE, 1=CW, 2=CCW, 3=Unknown

		//ADJUST CLOCK PULSE WIDTH if CW...

			if (direction == 1) 		//If CW...Increase PulseWidth
			{
				ClockingCount = (ClockingCount - (ClockingCount * 0.005)); //Increase Speed by Decreasing Clock Pulse width by say 5% - C. Schell 10/23/2013

				if (ClockingCount < 2)
				{
					ClockingCount = 2;
				}
				
			}//endif... 

		//ADJUST CLOCK PULSE WIDTH if CCW...
			else if (direction == 2) 		//If CCW...Decrease Pulse Width
			{
				ClockingCount = (ClockingCount + (ClockingCount * 0.005));//Decrease Speed by Increasing Clock Pulse width by say 5% - C. Schell 10/23/2013			

				if (ClockingCount > 2000)
				{
					LongCounter = 0;
					ClkIn_Stepper_ENABLE = 0;				// PB4D - 1=>ENABLED; 0=>DISABLED
					//Brief Delay between changing directions...
					while (LongCounter < 300000 ) {			// NOP for 316,055 Cycles => 500ms
						LongCounter ++;
						//ClockingCount = 0;				// Zero Frequency...
						ClockingCount = 2000; 				// Minimum Continuous Speed = 2000
					}

					ClkIn_Stepper_CW_CCW = ~ClkIn_Stepper_CW_CCW;	// Reverse Direction
					ClkIn_Stepper_ENABLE = 1;				// PB4D - 1=>ENABLED; 0=>DISABLED
					goto ContModeHolding;					// Go Back to beginning of Continuous Mode Funct...
				}
				
			}//endif...

			else  //For example, no rotation of Encoder...
			{
				if (ClockingCount == 0)		//If ClockCount is currently zerroed, then wait here for Encoder Rotation...
				{
					EncoderDirection();	//direction; //0=NO CHANGE, 1=CW, 2=CCW 
					if (direction == 1) 	//CW?...
					{
						ClkIn_Stepper_CW_CCW = 0;	// PB1D - 0=CW; 1=CCW
						ClockingCount = 375;
		  				//ENABLE & RAMP UP...
		  				ClkIn_Stepper_ENABLE = 1;			// PB4D - 1=>ENABLED; 0=>DISABLED
		  				MotorRampUp();
						goto StartCont;
						 
					}
					else if (direction == 2) //CCW?...
					{
						ClkIn_Stepper_CW_CCW = 1;	// PB1D - 0=CW; 1=CCW
						ClockingCount = 375;
		  				//ENABLE & RAMP UP...
		  				ClkIn_Stepper_ENABLE = 1;			// PB4D - 1=>ENABLED; 0=>DISABLED
		  				MotorRampUp();
						goto StartCont; 
					}

					goto ContModeHolding;	//Hold here until Knob button is pressed, OR Turned either CW or CCW
				}
			}

StartCont:


			
		//NOW SEND CLOCK PULSE!
			//HIGH PULSE...
			ClkIn_Stepper_CLK    = 1;	// PB0D - The Electrical angle advances by one for each CLK input

			while(counter < ClockingCount ) {	// NOP for 2000 Cycles => 3164uS
				counter++;
			}
			counter = 0;


			//LOW PULSE...
			ClkIn_Stepper_CLK    = 0;	// PB0D - The Electrical angle advances by one for each CLK input

			while(counter < ClockingCount ) {	// NOP for 2000 Cycles => 3164uS
				counter++;
			}
			counter = 0;


	}//wend


	//Ensure Disabled & Clock left in Low State
	ClkIn_Stepper_ENABLE = 0;			// PB4D - 1=>ENABLED; 0=>DISABLED
	ClockingCount 	   = 0;			// Zero ClockingCount value when button pressed
	ClkIn_Stepper_CLK    = 0;			// PB0D - The Electrical angle advances by one for each CLK input

} //END Continuous Mode  Function
//===========================================================================
//***** END Continuous Mode **************************************************




//***** START Machine State Action *******************************************
//===========================================================================
//	Get Machine State Action based on Mode & Encoder Postion 
//===========================================================================
void MachineStateAction (void){

//There are 16 possible Machine States from the microcontroller driven application 

INPUTS:
	//mode: 			0=PowerOn/RESET; 1=STEP MODE 
	//EncoderPosition:      1-4 Quadrature Encoder position with Offset factored in	
	//direction: 		0=NO CHANGE, 1=CW, 2=CCW, 3=Unknown

	switch(mode) 
	{
	case 0:			// 0=PowerOn/RESET
		ALL_LEDs_OFF();
		break;

	case 1: 			// 1=STEP MODE-No LEDs
		LEDStateA();
		break;

	case 2: 			// 2=STEP MODE-FULL STEP
		LEDStateA1();
			//ClkIn_Stepper_CW_CCW = 0;		// PB1D - 0=CW; 1=CCW 
			ClkIn_Stepper_MODE_0 = 0;		// PB2D - 00=>FULL STEP; 10=>Half_A; 01=>Half_B; 11=>Quarter Step
			ClkIn_Stepper_MODE_1 = 0;		// PB3D - ...  
		break;

	case 3: 			// 3=STEP MODE-HALF STEP A
		LEDStateA2();
			//ClkIn_Stepper_CW_CCW = 0;		// PB1D - 0=CW; 1=CCW 
			ClkIn_Stepper_MODE_0 = 1;		// PB2D - 00=>FULL STEP; 10=>Half_A; 01=>Half_B; 11=>Quarter Step
			ClkIn_Stepper_MODE_1 = 0;		// PB3D - ...  
		break;

	case 4:			// 4=STEP MODE-HALF STEP B
		LEDStateA3();
			//ClkIn_Stepper_CW_CCW = 0;		// PB1D - 0=CW; 1=CCW 
			ClkIn_Stepper_MODE_0 = 0;		// PB2D - 00=>FULL STEP; 10=>Half_A; 01=>Half_B; 11=>Quarter Step
			ClkIn_Stepper_MODE_1 = 1;		// PB3D - ...  
		break;	

	case 5:			// 5=STEP MODE-QUARTER STEP
		LEDStateA4();
			//ClkIn_Stepper_CW_CCW = 0;		// PB1D - 0=CW; 1=CCW 
			ClkIn_Stepper_MODE_0 = 1;		// PB2D - 00=>FULL STEP; 10=>Half_A; 01=>Half_B; 11=>Quarter Step
			ClkIn_Stepper_MODE_1 = 1;		// PB3D - ...  
		break;	

	case 6:			// 6=DRIVER_EN RED
		LEDStateB();
		ClkIn_Stepper_ENABLE = 0;		// PB4D - 0=>DISABLED Controller
		break;	

	case 7:			// 7=DRIVER_EN GREEN
		LEDStateC();
		ClkIn_Stepper_ENABLE = 0;		// PB4D - 0=>DISABLED Controller
		break;	

	case 8:			// 8=SINGLE STEP LED ON				
		LEDStateC1();
		break;	

	case 9:			// 9=SINGLE-STEP-CW
		LEDStateC1();
		StepCW();
		break;	

	case 10:			// 10=SINGLE-STEP-CCW
		LEDStateC1();
		StepCCW(); 
		break;	

	case 11:			// 11=CONTINUOUS LED ON
		ClkIn_Stepper_ENABLE = 0;		// PB4D - 0=>DISABLED Controller
		LEDStateC2();
		break;	

	case 12:			// 12=CONTINUOUS-INC FREQ CW
		LEDStateC2();
		break;	

	case 13:			// 13=CONTINUOUS-INC FREQ CCW 
		LEDStateC2();
		break;

	case 14:			// 14=PASS THRU RED
		LEDStateD();
		break;	

	case 15:			// 15=PASS THRU GREEN
		LEDStateE();
		break;

	default:			// ERROR
		ALL_LEDs_OFF();	
		
		break;
	}//End Select...


} //END MachineStateAction Function
//===========================================================================
//***** END Machine State Action *********************************************



//===========================================================================
//***** Start Motor Ramp Up Function *****************************************
void MotorRampUp (void) {
//Motor Ramp Up...From 10ms pulses down to 1ms pulses in ~150ms
unsigned int k, WarmUp, counter;

	for (k=0; k<4625; k++) // 5000-375=4625
	{
 
		WarmUp = (5000 - k);
			
		//HIGH PULSE...
		ClkIn_Stepper_CLK    = 1;	// PB0D - The Electrical angle advances by one for each CLK input

		while(counter < WarmUp ) {	// NOP for ~375 Cycles
			counter++;
		}
		counter = 0;


		//LOW PULSE...
		ClkIn_Stepper_CLK    = 0;	// PB0D - The Electrical angle advances by one for each CLK input

		while(counter < WarmUp ) {	// NOP for ~375 Cycles
			counter++;
		}
		counter = 0;
			
		k = (k+250);
	}//next k 
}
//===========================================================================
//***** End Motor Ramp Up Function *******************************************
