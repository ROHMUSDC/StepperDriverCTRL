/*
* ----------------------------------------------------------------
*	(C) COPYRIGHT 2012 LAPIS SEMICONDUCTOR Co., Ltd.
*		ALL RIGHTS RESERVED
* ----------------------------------------------------------------
*/

#include	<ML610102.H>
	int XXX;			//PORTB
	
static void int_wdtint(void);
static void int_dummy(void);
static void int_2Hz(void);

		           
/*############################################################################*/
/*#                          interrupt pragma                                #*/
/*############################################################################*/

	#if !defined(_WIN32)
	#pragma interrupt int_wdtint			0x08 1		/* WDT		*/
	#pragma interrupt int_dummy			0x0C 1		/* VLS		*/
	#pragma interrupt int_dummy			0x10 1		/* PA0		*/
	#pragma interrupt int_dummy			0x12 1		/* PA1		*/
	#pragma interrupt int_dummy			0x14 1		/* PA2		*/
	#pragma interrupt int_dummy			0x18 1		/* PB0          */
	#pragma interrupt int_dummy			0x1A 1		/* PB1 	      */
	#pragma interrupt int_dummy			0x24 1		/* A/D	*/
	#pragma interrupt int_dummy			0x34 1		/* TMR 8		*/
	#pragma interrupt int_dummy			0x36 1		/* TMR 9		*/
      #pragma interrupt int_dummy			0x40 1		/* UART0		*/
	#pragma interrupt int_dummy			0x4C 1		/* CMP0	 	*/
      #pragma interrupt int_dummy			0x4E 1		/* CMP1	 	*/	
	#pragma interrupt int_dummy			0x58 1		/* TMR E		*/
	#pragma interrupt int_dummy			0x5A 1		/* TMR F		*/
	#pragma interrupt int_dummy			0x5C 1		/* TMR A		*/
	#pragma interrupt int_dummy			0x5E 1		/* TMR B		*/
	#pragma interrupt int_dummy			0x60 1		/* PWMC		*/
	#pragma interrupt int_dummy			0x6A 1		/* TBC128Hz		*/
	#pragma interrupt int_dummy			0x6E 1		/* TBC32Hz		*/
	#pragma interrupt int_dummy			0x70 1		/* TBC16Hz		*/
	#pragma interrupt int_2Hz			0x76 1		/* TBC2Hz	*/
	#endif

static void int_wdtint(void){
	__DI();
	do
	{
	WDTCON = 0x5a;    /* WDP: 0 -> 1 */
	}while(WDP != 1);
	WDTCON = 0xa5;        /* WDP: 1 -> 0 */
}
	
static void int_dummy(void){
	#pragma asm
	mov r0,	#33h
	st	r0,	FCON0		
	brk
	#pragma endasm
}

//===========================================================================
// TBC 2Hz
//===========================================================================
static void int_2Hz(void){
//	int XXX;
	XXX = XXX + 1;			//inc
	PBD = XXX;
}
