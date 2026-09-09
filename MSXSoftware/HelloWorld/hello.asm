;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 4.2.0 #13081 (Linux)
;--------------------------------------------------------
	.module hello
	.optsdcc -mz80
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _print
	.globl _wait_key
	.globl _putchar_msx
	.globl _set_screen
;--------------------------------------------------------
; special function registers
;--------------------------------------------------------
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area _DATA
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area _INITIALIZED
;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area _DABS (ABS)
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area _HOME
	.area _GSINIT
	.area _GSFINAL
	.area _GSINIT
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area _HOME
	.area _HOME
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area _CODE
;hello.c:21: void set_screen(uint8_t mode) __z88dk_fastcall
;	---------------------------------
; Function set_screen
; ---------------------------------
_set_screen::
;hello.c:35: __endasm;
	push	ix
	push	iy
	ex	af, af'
	ld	a, l
	ld	iy, #0xFCC0 ; EXPTBL & 0xFF00: main-ROM slot
	ld	ix, #0x005F ; CHGMOD
	call	0x001C ; CALSLT
	ex	af, af'
	pop	iy
	pop	ix
;hello.c:36: }
	ret
;hello.c:39: void putchar_msx(char c) __z88dk_fastcall
;	---------------------------------
; Function putchar_msx
; ---------------------------------
_putchar_msx::
;hello.c:53: __endasm;
	push	ix
	push	iy
	ex	af, af'
	ld	a, l
	ld	iy, #0xFCC0
	ld	ix, #0x00A2 ; CHPUT
	call	0x001C ; CALSLT
	ex	af, af'
	pop	iy
	pop	ix
;hello.c:54: }
	ret
;hello.c:57: char wait_key(void) __naked
;	---------------------------------
; Function wait_key
; ---------------------------------
_wait_key::
;hello.c:66: __endasm;
	ld	iy, #0xFCC0
	ld	ix, #0x009F ; CHGET
	call	0x001C ; CALSLT
	ld	l, a ; CHGET returns keycode in A;
;	put it in L as C return value
	ret
;hello.c:67: }
;hello.c:70: void print(const char *s)
;	---------------------------------
; Function print
; ---------------------------------
_print::
	ex	de, hl
;hello.c:72: while (*s)
00101$:
	ld	a, (de)
	or	a, a
	ret	Z
;hello.c:73: putchar_msx(*s++);
	ld	l, a
;	spillPairReg hl
;	spillPairReg hl
	inc	de
	push	de
	call	_putchar_msx
	pop	de
;hello.c:74: }
	jr	00101$
;hello.c:80: int main(void)
;	---------------------------------
; Function main
; ---------------------------------
_main::
;hello.c:82: set_screen(SCREEN1_WIDTH == 32 ? 1 : 0);
	ld	l, #0x01
;	spillPairReg hl
;	spillPairReg hl
	call	_set_screen
;hello.c:84: print("HELLO WORLD!\r\n\r\n");
	ld	hl, #___str_0
	call	_print
;hello.c:85: print("FROM RISKYMSX2 CARTRIDGE\r\n\r\n");
	ld	hl, #___str_1
	call	_print
;hello.c:86: print("PRESS ANY KEY TO EXIT...");
	ld	hl, #___str_2
	call	_print
;hello.c:88: wait_key();
	call	_wait_key
;hello.c:90: set_screen(0);          /* back to SCREEN 0 for BASIC */
	ld	l, #0x00
;	spillPairReg hl
;	spillPairReg hl
	call	_set_screen
;hello.c:92: return 0;               /* RET to BIOS, boot continues into BASIC */
	ld	de, #0x0000
;hello.c:93: }
	ret
___str_0:
	.ascii "HELLO WORLD!"
	.db 0x0d
	.db 0x0a
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_1:
	.ascii "FROM RISKYMSX2 CARTRIDGE"
	.db 0x0d
	.db 0x0a
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_2:
	.ascii "PRESS ANY KEY TO EXIT..."
	.db 0x00
	.area _CODE
	.area _INITIALIZER
	.area _CABS (ABS)
