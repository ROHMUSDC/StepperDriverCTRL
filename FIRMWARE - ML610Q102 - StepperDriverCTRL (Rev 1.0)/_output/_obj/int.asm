;; Compile Options : /TML610102 /MS /near /Icommon /Imain /Iirq /Itimer /Iclock /Itbc /SS 64 /SD /Oa /Ot /W 3 /Wc /Fa_output\_obj\ /Zs 
;; Version Number  : Ver.3.41.8
;; File Name       : int.c

	type (ML610102) 
	model small, near
	$$int_2Hz$int segment code 2h #0h
	$$int_dummy$int segment code 2h #0h
	$$int_wdtint$int segment code 2h #0h
CVERSION 3.41.8
CSGLOBAL 03H 0000H "int_dummy" 08H 02H 01H 00H 90H 00H 00H 00H 07H
CSGLOBAL 03H 0000H "int_wdtint" 08H 02H 00H 00H 90H 02H 00H 00H 07H
CSGLOBAL 03H 0000H "int_2Hz" 08H 02H 02H 00H 90H 02H 00H 00H 07H
CSTRUCTTAG 0000H 0000H 0000H 0008H 00000001H "_Notag"
CSTRUCTMEM 52H 00000001H 00000000H "b0" 02H 00H 00H
CSTRUCTMEM 52H 00000001H 00000001H "b1" 02H 00H 00H
CSTRUCTMEM 52H 00000001H 00000002H "b2" 02H 00H 00H
CSTRUCTMEM 52H 00000001H 00000003H "b3" 02H 00H 00H
CSTRUCTMEM 52H 00000001H 00000004H "b4" 02H 00H 00H
CSTRUCTMEM 52H 00000001H 00000005H "b5" 02H 00H 00H
CSTRUCTMEM 52H 00000001H 00000006H "b6" 02H 00H 00H
CSTRUCTMEM 52H 00000001H 00000007H "b7" 02H 00H 00H
CTYPEDEF 0000H 0000H 43H "_BYTE_FIELD" 04H 00H 05H 00H 00H
CGLOBAL 00H 43H 0002H "XXX" 02H 00H 01H
CFILE 0001H 000003F4H "C:\\Users\\cschell\\U8DEV~1\\Inc\\ML610102.H"
CFILE 0000H 00000046H "src\\int.c"

	rseg $$int_wdtint$int
CFUNCTION 0

_int_wdtint	:
CBLOCK 0 1 45

;;static void int_wdtint(void){
CLINEA 0000H 0001H 002DH 0001H 001DH
	push	er0
CBLOCK 0 2 45

;;	__DI();
CLINEA 0000H 0001H 002EH 0002H 0008H
	di

;;	do
CLINEA 0000H 0001H 002FH 0002H 0003H
_$L3 :
CBLOCK 0 3 48

;;	WDTCON = 0x5a;    /* WDP: 0 -> 1 */
CLINEA 0000H 0001H 0031H 0002H 0024H
	mov	r0,	#05ah
	st	r0,	0f00eh
CBLOCKEND 0 3 50

;;	}while(WDP != 1);
CLINEA 0000H 0000H 0032H 0002H 0012H
	tb	0f00eh.0
	beq	_$L3

;;	WDTCON = 0xa5;        /* WDP: 1 -> 0 */
CLINEA 0000H 0001H 0033H 0002H 0028H
	mov	r0,	#0a5h
	st	r0,	0f00eh
CBLOCKEND 0 2 52

;;}
CLINEA 0000H 0001H 0034H 0001H 0001H
	pop	er0
	rti
CBLOCKEND 0 1 52
CFUNCTIONEND 0


	rseg $$int_dummy$int
CFUNCTION 1

_int_dummy	:
CBLOCK 1 1 54

;;static void int_dummy(void){
CLINEA 0000H 0001H 0036H 0001H 001CH
CBLOCK 1 2 54

;;	#pragma asm
CLINEA 0000H 0001H 0037H 0002H 000CH
	mov r0,	#33h
	st	r0,	FCON0		
	brk
CBLOCKEND 1 2 60

;;}
CLINEA 0000H 0001H 003CH 0001H 0001H
	rti
CBLOCKEND 1 1 60
CFUNCTIONEND 1


	rseg $$int_2Hz$int
CFUNCTION 2

_int_2Hz	:
CBLOCK 2 1 65

;;static void int_2Hz(void){
CLINEA 0000H 0001H 0041H 0001H 001AH
	push	er0
CBLOCK 2 2 65

;;	XXX = XXX + 1;			//inc
CLINEA 0000H 0001H 0043H 0002H 0017H
	l	er0,	NEAR _XXX
	add	er0,	#1 
	st	er0,	NEAR _XXX

;;	PBD = XXX;
CLINEA 0000H 0001H 0044H 0002H 000BH
	l	r0,	NEAR _XXX
	st	r0,	0f258h
CBLOCKEND 2 2 69

;;}
CLINEA 0000H 0001H 0045H 0001H 0001H
	pop	er0
	rti
CBLOCKEND 2 1 69
CFUNCTIONEND 2

	_XXX comm data 02h #00h
	extrn code near : _main

	cseg #00h at 08h
	dw	_int_wdtint

	cseg #00h at 0Ch
	dw	_int_dummy

	cseg #00h at 010h
	dw	_int_dummy
	dw	_int_dummy
	dw	_int_dummy

	cseg #00h at 018h
	dw	_int_dummy
	dw	_int_dummy

	cseg #00h at 024h
	dw	_int_dummy

	cseg #00h at 034h
	dw	_int_dummy
	dw	_int_dummy

	cseg #00h at 040h
	dw	_int_dummy

	cseg #00h at 04Ch
	dw	_int_dummy
	dw	_int_dummy

	cseg #00h at 058h
	dw	_int_dummy
	dw	_int_dummy
	dw	_int_dummy
	dw	_int_dummy
	dw	_int_dummy

	cseg #00h at 06Ah
	dw	_int_dummy

	cseg #00h at 06Eh
	dw	_int_dummy
	dw	_int_dummy

	cseg #00h at 076h
	dw	_int_2Hz

	end
