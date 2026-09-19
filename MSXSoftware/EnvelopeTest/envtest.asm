;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 4.2.0 #13081 (Linux)
;--------------------------------------------------------
	.module envtest
	.optsdcc -mz80
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _wait_key
	.globl _putchar_msx
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
;envtest.c:29: void putchar_msx(char c) __z88dk_fastcall
;	---------------------------------
; Function putchar_msx
; ---------------------------------
_putchar_msx::
;envtest.c:43: __endasm;
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
;envtest.c:44: }
	ret
;envtest.c:46: char wait_key(void) __naked
;	---------------------------------
; Function wait_key
; ---------------------------------
_wait_key::
;envtest.c:54: __endasm;
	ld	iy, #0xFCC0
	ld	ix, #0x009F ; CHGET
	call	0x001C ; CALSLT
	ld	l, a
	ret
;envtest.c:55: }
;envtest.c:57: static void print(const char *s)
;	---------------------------------
; Function print
; ---------------------------------
_print:
	ex	de, hl
;envtest.c:59: while (*s)
00101$:
	ld	a, (de)
	or	a, a
	ret	Z
;envtest.c:60: putchar_msx(*s++);
	ld	l, a
;	spillPairReg hl
;	spillPairReg hl
	inc	de
	push	de
	call	_putchar_msx
	pop	de
;envtest.c:61: }
	jr	00101$
;envtest.c:63: static void print_hex(uint8_t v)
;	---------------------------------
; Function print_hex
; ---------------------------------
_print_hex:
	call	___sdcc_enter_ix
	ld	hl, #-17
	add	hl, sp
	ld	sp, hl
	ld	c, a
;envtest.c:65: const char hex[] = "0123456789ABCDEF";
	ld	hl, #0
	add	hl, sp
	ex	de, hl
	ld	a, #0x30
	ld	(de), a
	ld	-16 (ix), #0x31
	ld	-15 (ix), #0x32
	ld	-14 (ix), #0x33
	ld	-13 (ix), #0x34
	ld	-12 (ix), #0x35
	ld	-11 (ix), #0x36
	ld	-10 (ix), #0x37
	ld	-9 (ix), #0x38
	ld	-8 (ix), #0x39
	ld	-7 (ix), #0x41
	ld	-6 (ix), #0x42
	ld	-5 (ix), #0x43
	ld	-4 (ix), #0x44
	ld	-3 (ix), #0x45
	ld	-2 (ix), #0x46
	ld	-1 (ix), #0x00
;envtest.c:66: putchar_msx(hex[(v >> 4) & 0x0FU]);
	ld	a, c
	rlca
	rlca
	rlca
	rlca
;	spillPairReg hl
;	spillPairReg hl
;	spillPairReg hl
;	spillPairReg hl
	and	a, #0xf
	ld	l, a
;	spillPairReg hl
;	spillPairReg hl
	ld	h, #0x00
;	spillPairReg hl
;	spillPairReg hl
	add	hl, de
	ld	l, (hl)
;	spillPairReg hl
	push	bc
	push	de
	call	_putchar_msx
	pop	de
	pop	bc
;envtest.c:67: putchar_msx(hex[v & 0x0FU]);
	ld	a, c
	and	a, #0x0f
	ld	l, a
;	spillPairReg hl
;	spillPairReg hl
	ld	h, #0x00
;	spillPairReg hl
;	spillPairReg hl
	add	hl, de
	ld	l, (hl)
;	spillPairReg hl
	call	_putchar_msx
;envtest.c:68: }
	ld	sp, ix
	pop	ix
	ret
;envtest.c:70: int main(void)
;	---------------------------------
; Function main
; ---------------------------------
_main::
	call	___sdcc_enter_ix
	push	af
	dec	sp
;envtest.c:72: print("ENVELOPE TEST\r\n");
	ld	hl, #___str_1
	call	_print
;envtest.c:73: print("press a key to send 0xFE (undefined cmd)\r\n");
	ld	hl, #___str_2
	call	_print
;envtest.c:74: print("to the firmware mailbox at 0x7FF0.\r\n\r\n");
	ld	hl, #___str_3
	call	_print
;envtest.c:83: uint8_t hdr = *((volatile uint8_t *)0x4000U);
	ld	hl, #0x4000
	ld	c, (hl)
;envtest.c:84: print("cart hdr @4000=");
	push	bc
	ld	hl, #___str_4
	call	_print
	pop	bc
;envtest.c:85: print_hex(hdr);
	ld	a, c
	call	_print_hex
;envtest.c:86: print("\r\n");
	ld	hl, #___str_5
	call	_print
;envtest.c:89: uint8_t hdr2 = *((volatile uint8_t *)0x4001U);
	ld	a, (#0x4001)
	ld	-1 (ix), a
;envtest.c:90: print("cart hdr @4001=");
	ld	hl, #___str_6
	call	_print
;envtest.c:91: print_hex(hdr2);
	ld	a, -1 (ix)
	call	_print_hex
;envtest.c:92: print("\r\n");
	ld	hl, #___str_5
	call	_print
;envtest.c:95: uint8_t hdr3 = *((volatile uint8_t *)0x4010U);
	ld	hl, #0x4010
	ld	c, (hl)
;envtest.c:96: print("cart @4010=");
	push	bc
	ld	hl, #___str_7
	call	_print
	pop	bc
;envtest.c:97: print_hex(hdr3);
	ld	a, c
	call	_print_hex
;envtest.c:98: print("\r\n");
	ld	hl, #___str_5
	call	_print
00106$:
;envtest.c:103: uint8_t st = *MBOX_STATUS;
	ld	a, (#0x7ff0)
	ld	-3 (ix), a
;envtest.c:104: print("status=");
	ld	hl, #___str_8
	call	_print
;envtest.c:105: print_hex(st);
	ld	a, -3 (ix)
	call	_print_hex
;envtest.c:106: print("  ready=");
	ld	hl, #___str_9
	call	_print
;envtest.c:107: print((st & ST_READY) ? "Y" : "N");
	bit	7, -3 (ix)
	jr	Z, 00110$
	ld	hl, #___str_10+0
	ld	-2 (ix), l
	ld	-1 (ix), h
	jr	00111$
00110$:
	ld	hl, #___str_11+0
	ld	-2 (ix), l
	ld	-1 (ix), h
00111$:
	ld	l, -2 (ix)
;	spillPairReg hl
;	spillPairReg hl
	ld	h, -1 (ix)
;	spillPairReg hl
;	spillPairReg hl
	call	_print
;envtest.c:108: print("  done=");
	ld	hl, #___str_12
	call	_print
;envtest.c:109: print((st & ST_DONE) ? "Y" : "N");
	bit	6, -3 (ix)
	jr	Z, 00112$
	ld	hl, #___str_10+0
	jr	00113$
00112$:
	ld	hl, #___str_11+0
00113$:
	call	_print
;envtest.c:110: print("\r\n");
	ld	hl, #___str_5
	call	_print
;envtest.c:112: print("press any key...\r\n");
	ld	hl, #___str_13
	call	_print
;envtest.c:113: wait_key();
	call	_wait_key
;envtest.c:119: *MBOX_CMD = 0xFEU;
	ld	hl, #0x7ff0
	ld	(hl), #0xfe
;envtest.c:120: print("wrote 0xFE\r\n");
	ld	hl, #___str_14
	call	_print
;envtest.c:127: for (uint8_t i = 0; i < 5; i++) {
	ld	c, #0x00
00104$:
	ld	a, c
	sub	a, #0x05
	jr	NC, 00101$
;envtest.c:128: uint8_t s = *MBOX_STATUS;
	ld	hl, #0x7ff0
	ld	b, (hl)
;envtest.c:129: print("post ");
	push	bc
	ld	hl, #___str_15
	call	_print
	pop	bc
;envtest.c:130: print_hex(i);
	push	bc
	ld	a, c
	call	_print_hex
	ld	hl, #___str_16
	call	_print
	pop	bc
;envtest.c:132: print_hex(s);
	push	bc
	ld	a, b
	call	_print_hex
	ld	hl, #___str_5
	call	_print
	pop	bc
;envtest.c:127: for (uint8_t i = 0; i < 5; i++) {
	inc	c
	jr	00104$
00101$:
;envtest.c:135: print("\r\n");
	ld	hl, #___str_5
	call	_print
;envtest.c:137: }
	jp	00106$
___str_1:
	.ascii "ENVELOPE TEST"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_2:
	.ascii "press a key to send 0xFE (undefined cmd)"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_3:
	.ascii "to the firmware mailbox at 0x7FF0."
	.db 0x0d
	.db 0x0a
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_4:
	.ascii "cart hdr @4000="
	.db 0x00
___str_5:
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_6:
	.ascii "cart hdr @4001="
	.db 0x00
___str_7:
	.ascii "cart @4010="
	.db 0x00
___str_8:
	.ascii "status="
	.db 0x00
___str_9:
	.ascii "  ready="
	.db 0x00
___str_10:
	.ascii "Y"
	.db 0x00
___str_11:
	.ascii "N"
	.db 0x00
___str_12:
	.ascii "  done="
	.db 0x00
___str_13:
	.ascii "press any key..."
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_14:
	.ascii "wrote 0xFE"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_15:
	.ascii "post "
	.db 0x00
___str_16:
	.ascii " status="
	.db 0x00
	.area _CODE
	.area _INITIALIZER
	.area _CABS (ABS)
