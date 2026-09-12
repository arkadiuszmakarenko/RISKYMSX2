;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 4.2.0 #13081 (Linux)
;--------------------------------------------------------
	.module psramdiag
	.optsdcc -mz80
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
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
;psramdiag.c:70: static void chgmod(uint8_t mode) __z88dk_fastcall
;	---------------------------------
; Function chgmod
; ---------------------------------
_chgmod:
;psramdiag.c:84: __endasm;
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
;psramdiag.c:85: }
	ret
;psramdiag.c:88: static void chput(char c) __z88dk_fastcall
;	---------------------------------
; Function chput
; ---------------------------------
_chput:
;psramdiag.c:102: __endasm;
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
;psramdiag.c:103: }
	ret
;psramdiag.c:116: static uint8_t key_scan_all(void) __naked
;	---------------------------------
; Function key_scan_all
; ---------------------------------
_key_scan_all:
;psramdiag.c:145: __endasm;
	push	ix
	push	iy
	di	; no ISR interference while we
;	hold the PSG address latch
	ld	b, #10 ; rows 0..9
	ld	c, #0 ; current row
	scan_row:
	push	bc
	ld	a, c ; row number
	ld	iy, #0xFCC0
	ld	ix, #0x0141 ; SNSMAT
	call	0x001C ; CALSLT -> A = row matrix
	pop	bc
	inc	a ; 0xFF (row idle) -> 0 sets Z
	jr	z, row_idle
	pop	iy
	pop	ix
	ld	l, #1 ; some key is down
	ret
	row_idle:
	inc	c
	djnz	scan_row
	pop	iy
	pop	ix
	ld	l, #0 ; nothing down
	ret
;psramdiag.c:146: }
;psramdiag.c:154: static void delay_ms(uint16_t ms) __z88dk_fastcall __naked
;	---------------------------------
; Function delay_ms
; ---------------------------------
_delay_ms:
;psramdiag.c:169: __endasm;
	dly_outer:
	ld	bc, #138
	dly_inner:
	dec	bc
	ld	a, b
	or	c
	jr	nz, dly_inner
	dec	hl
	ld	a, h
	or	l
	jr	nz, dly_outer
	ret
;psramdiag.c:170: }
;psramdiag.c:177: static void wait_key(void)
;	---------------------------------
; Function wait_key
; ---------------------------------
_wait_key:
00107$:
;psramdiag.c:180: if (key_scan_all()) {
	call	_key_scan_all
	or	a, a
	jr	Z, 00105$
;psramdiag.c:181: delay_ms(30);		/* debounce */
	ld	hl, #0x001e
	call	_delay_ms
;psramdiag.c:182: while (key_scan_all())	/* wait for release */
00101$:
	call	_key_scan_all
	or	a, a
	ret	Z
;psramdiag.c:183: delay_ms(20);
	ld	hl, #0x0014
	call	_delay_ms
;psramdiag.c:184: return;
	jr	00101$
00105$:
;psramdiag.c:186: delay_ms(20);
	ld	hl, #0x0014
	call	_delay_ms
;psramdiag.c:188: }
	jr	00107$
;psramdiag.c:192: static void gtstck(uint8_t stick) __z88dk_fastcall
;	---------------------------------
; Function gtstck
; ---------------------------------
_gtstck:
;psramdiag.c:206: __endasm;
	push	ix
	push	iy
	ex	af, af'
	ld	a, l
	ld	iy, #0xFCC0
	ld	ix, #0x00D5 ; GTSTCK
	call	0x001C ; CALSLT
	ex	af, af'
	pop	iy
	pop	ix
;psramdiag.c:207: }
	ret
;psramdiag.c:209: static void print(const char *s)
;	---------------------------------
; Function print
; ---------------------------------
_print:
	ex	de, hl
;psramdiag.c:211: while (*s)
00101$:
	ld	a, (de)
	or	a, a
	ret	Z
;psramdiag.c:212: chput(*s++);
	ld	l, a
;	spillPairReg hl
;	spillPairReg hl
	inc	de
	push	de
	call	_chput
	pop	de
;psramdiag.c:213: }
	jr	00101$
;psramdiag.c:215: static void print_hex(uint8_t b)
;	---------------------------------
; Function print_hex
; ---------------------------------
_print_hex:
	ld	e, a
;psramdiag.c:218: chput(hex[(b >> 4) & 0x0F]);
	ld	bc, #_print_hex_hex_65536_19+0
	ld	a, e
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
	add	hl, bc
	ld	l, (hl)
;	spillPairReg hl
	push	bc
	push	de
	call	_chput
	pop	de
	pop	bc
;psramdiag.c:219: chput(hex[b & 0x0F]);
	ld	d, #0x00
	ld	a, e
	and	a, #0x0f
	ld	l, a
;	spillPairReg hl
;	spillPairReg hl
	ld	h, #0x00
;	spillPairReg hl
;	spillPairReg hl
	add	hl, bc
	ld	l, (hl)
;	spillPairReg hl
;psramdiag.c:220: }
	jp	_chput
_print_hex_hex_65536_19:
	.ascii "0123456789ABCDEF"
	.db 0x00
;psramdiag.c:231: static void phase1_fixed_reads(void) __naked
;	---------------------------------
; Function phase1_fixed_reads
; ---------------------------------
_phase1_fixed_reads:
;psramdiag.c:243: __endasm;
	ld	bc, #0x0FA0 ; 4000U = 4000
	ld	hl, #0x5000 ; 0x5000U
	ph1_loop:
	ld	a, (hl) ; the cart data read
	dec	bc
	ld	a, b
	or	c
	jp	nz, ph1_loop
	ret
;psramdiag.c:244: }
;psramdiag.c:255: static void phase2_ldir_copy(void) __naked
;	---------------------------------
; Function phase2_ldir_copy
; ---------------------------------
_phase2_ldir_copy:
;psramdiag.c:263: __endasm;
	ld	hl, #0x5000 ; src: walking pattern in cart
	ld	de, #0xD000 ; dst: MSX RAM
	ld	bc, #0x0100 ; 256U
	ldir
	ret
;psramdiag.c:264: }
;psramdiag.c:279: static void phase3_isolated_reads(void) __naked
;	---------------------------------
; Function phase3_isolated_reads
; ---------------------------------
_phase3_isolated_reads:
;psramdiag.c:300: __endasm;
;	build the stub at 0xD100 (ld a,(hl) / djnz -3 / ret)
	ld	hl, #0xD100
	ld	(hl), #0x7E
	inc	hl
	ld	(hl), #0x10
	inc	hl
	ld	(hl), #0xFD
	inc	hl
	ld	(hl), #0xC9
;	8 passes x 256 isolated reads @ 0x5040
	ld	c, #0x08 ; 8U
	ph3_outer:
	ld	b, #0x00 ; djnz counts 256 reads
	ld	hl, #0x5040 ; 0x5040U
	call	0xD100 ; run the RAM stub
	dec	c
	jr	nz, ph3_outer
	ret
;psramdiag.c:301: }
;psramdiag.c:310: static void phase4_integrity(void)
;	---------------------------------
; Function phase4_integrity
; ---------------------------------
_phase4_integrity:
	call	___sdcc_enter_ix
	dec	sp
;psramdiag.c:313: uint8_t  bad = 0;
	ld	-1 (ix), #0x00
;psramdiag.c:315: print("PH4 integrity 0x5000-0x50FF ... ");
	ld	hl, #___str_1
	call	_print
;psramdiag.c:316: for (off = 0; off < PATTERN_SIZE; off++) {
	ld	bc, #0x0000
00106$:
;psramdiag.c:317: uint8_t exp = (uint8_t)(off ^ 0xA5U);
	ld	a, c
	xor	a, #0xa5
	ld	e, a
;psramdiag.c:318: uint8_t got = *(volatile uint8_t *)(uint16_t)(PATTERN_ADDR + off);
	ld	l, c
;	spillPairReg hl
;	spillPairReg hl
;	spillPairReg hl
;	spillPairReg hl
	ld	a, b
	add	a, #0x50
	ld	h, a
;	spillPairReg hl
;	spillPairReg hl
	ld	d, (hl)
;psramdiag.c:319: if (got != exp) {
	ld	a, e
	sub	a, d
	jr	Z, 00107$
;psramdiag.c:320: bad = 1;
	ld	-1 (ix), #0x01
;psramdiag.c:321: print("\r\nBAD@");
	push	bc
	push	de
	ld	hl, #___str_2
	call	_print
	pop	de
	pop	bc
;psramdiag.c:322: print_hex((uint8_t)(off >> 8));
	ld	l, b
;	spillPairReg hl
;	spillPairReg hl
	push	bc
	push	de
	ld	a, l
	call	_print_hex
	pop	de
	pop	bc
;psramdiag.c:323: print_hex((uint8_t)off);
	ld	l, c
;	spillPairReg hl
;	spillPairReg hl
	push	bc
	push	de
	ld	a, l
	call	_print_hex
	ld	hl, #___str_3
	call	_print
	pop	de
	push	de
	ld	a, d
	call	_print_hex
	ld	hl, #___str_4
	call	_print
	pop	de
	ld	a, e
	call	_print_hex
	ld	hl, #___str_5
	call	_print
	pop	bc
00107$:
;psramdiag.c:316: for (off = 0; off < PATTERN_SIZE; off++) {
	inc	bc
	ld	a, b
	sub	a, #0x01
	jr	C, 00106$
;psramdiag.c:331: if (!bad)
	ld	a, -1 (ix)
	or	a, a
	jr	NZ, 00108$
;psramdiag.c:332: print("OK\r\n");
	ld	hl, #___str_6
	call	_print
00108$:
;psramdiag.c:333: }
	inc	sp
	pop	ix
	ret
___str_1:
	.ascii "PH4 integrity 0x5000-0x50FF ... "
	.db 0x00
___str_2:
	.db 0x0d
	.db 0x0a
	.ascii "BAD@"
	.db 0x00
___str_3:
	.ascii " got="
	.db 0x00
___str_4:
	.ascii " exp="
	.db 0x00
___str_5:
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_6:
	.ascii "OK"
	.db 0x0d
	.db 0x0a
	.db 0x00
;psramdiag.c:345: static void phase5_turnaround(void)
;	---------------------------------
; Function phase5_turnaround
; ---------------------------------
_phase5_turnaround:
;psramdiag.c:350: print("PH5 BIOS/rd @0x5080 x512 ... ");
	ld	hl, #___str_7
	call	_print
;psramdiag.c:351: for (i = 0; i < PH5_ITER; i++) {
	ld	bc, #0x0000
00102$:
;psramdiag.c:352: gtstck(2);	/* main-ROM slot cycle (SLTSL high)   */
	push	bc
	ld	l, #0x02
;	spillPairReg hl
;	spillPairReg hl
	call	_gtstck
	pop	bc
;psramdiag.c:353: dummy = *(volatile uint8_t *)(uint16_t)PH5_ADDR;
	ld	a, (#0x5080)
;psramdiag.c:351: for (i = 0; i < PH5_ITER; i++) {
	inc	bc
	ld	e, c
	ld	d, b
	ld	a, d
	sub	a, #0x02
	jr	C, 00102$
;psramdiag.c:356: print("done\r\n");
	ld	hl, #___str_8
;psramdiag.c:357: }
	jp	_print
___str_7:
	.ascii "PH5 BIOS/rd @0x5080 x512 ... "
	.db 0x00
___str_8:
	.ascii "done"
	.db 0x0d
	.db 0x0a
	.db 0x00
;psramdiag.c:363: int main(void)
;	---------------------------------
; Function main
; ---------------------------------
_main::
	call	___sdcc_enter_ix
	dec	sp
;psramdiag.c:367: chgmod(0);	/* SCREEN 0, 40 columns */
	ld	l, #0x00
;	spillPairReg hl
;	spillPairReg hl
	call	_chgmod
;psramdiag.c:369: print("\r\nPSRAMDIAG v2 - RISKYMSX2\r\n\r\n");
	ld	hl, #___str_9
	call	_print
;psramdiag.c:370: print("ANY key = run next phase.\r\n");
	ld	hl, #___str_10
	call	_print
;psramdiag.c:371: print("(interrupt-free key scan via SNSMAT,\r\n");
	ld	hl, #___str_11
	call	_print
;psramdiag.c:372: print(" works even if the BIOS keyboard ISR\r\n");
	ld	hl, #___str_12
	call	_print
;psramdiag.c:373: print(" never runs - deliberately robust\r\n");
	ld	hl, #___str_13
	call	_print
;psramdiag.c:374: print(" on the emulated-cart bring-up)\r\n\r\n");
	ld	hl, #___str_14
	call	_print
;psramdiag.c:375: print("PH1 fixed rd 0x5000 x4000\r\n");
	ld	hl, #___str_15
	call	_print
;psramdiag.c:376: print("PH2 LDIR 0x5000->0xD000\r\n");
	ld	hl, #___str_16
	call	_print
;psramdiag.c:377: print("PH3 isolated rd 0x5040 x2048\r\n");
	ld	hl, #___str_17
	call	_print
;psramdiag.c:378: print("PH4 pattern integrity 0x5000-0x50FF\r\n");
	ld	hl, #___str_18
	call	_print
;psramdiag.c:379: print("PH5 BIOS+rd 0x5080 x512\r\n\r\n");
	ld	hl, #___str_19
	call	_print
;psramdiag.c:380: print("Set SRC SRAM/PSRAM on UART, reset,\r\n");
	ld	hl, #___str_20
	call	_print
;psramdiag.c:381: print("arm the 1660C, then press a key.\r\n\r\n");
	ld	hl, #___str_21
	call	_print
;psramdiag.c:383: for (phase = 1; ; phase++) {
	ld	-1 (ix), #0x01
00112$:
;psramdiag.c:384: wait_key();	/* blocks until any key is pressed+released */
	call	_wait_key
;psramdiag.c:386: switch (phase) {
	ld	a, -1 (ix)
	dec	a
	jr	Z, 00101$
	ld	a, -1 (ix)
	sub	a, #0x02
	jr	Z, 00102$
	ld	a, -1 (ix)
	sub	a, #0x03
	jr	Z, 00103$
	ld	a, -1 (ix)
	sub	a, #0x04
	jr	Z, 00104$
	ld	a, -1 (ix)
	sub	a, #0x05
	jr	Z, 00105$
	jr	00106$
;psramdiag.c:387: case 1:
00101$:
;psramdiag.c:388: print("PH1 fixed rd @0x5000 x4000\r\n");
	ld	hl, #___str_22
	call	_print
;psramdiag.c:389: phase1_fixed_reads();
	call	_phase1_fixed_reads
;psramdiag.c:390: print("PH1 done\r\n");
	ld	hl, #___str_23
	call	_print
;psramdiag.c:391: break;
	jr	00113$
;psramdiag.c:392: case 2:
00102$:
;psramdiag.c:393: print("PH2 LDIR 0x5000->0xD000\r\n");
	ld	hl, #___str_16
	call	_print
;psramdiag.c:394: phase2_ldir_copy();
	call	_phase2_ldir_copy
;psramdiag.c:395: print("PH2 done\r\n");
	ld	hl, #___str_24
	call	_print
;psramdiag.c:396: break;
	jr	00113$
;psramdiag.c:397: case 3:
00103$:
;psramdiag.c:398: print("PH3 isolated rd @0x5040\r\n");
	ld	hl, #___str_25
	call	_print
;psramdiag.c:399: phase3_isolated_reads();
	call	_phase3_isolated_reads
;psramdiag.c:400: print("PH3 done\r\n");
	ld	hl, #___str_26
	call	_print
;psramdiag.c:401: break;
	jr	00113$
;psramdiag.c:402: case 4:
00104$:
;psramdiag.c:403: phase4_integrity();
	call	_phase4_integrity
;psramdiag.c:404: break;
	jr	00113$
;psramdiag.c:405: case 5:
00105$:
;psramdiag.c:406: phase5_turnaround();
	call	_phase5_turnaround
;psramdiag.c:407: break;
	jr	00113$
;psramdiag.c:408: default:
00106$:
;psramdiag.c:411: print("\r\nALL DONE - PARKED in PH1 loop\r\n");
	ld	hl, #___str_27
	call	_print
;psramdiag.c:412: print("(LA: trigger on A=0x5000, M1=0)\r\n\r\n");
	ld	hl, #___str_28
	call	_print
00110$:
;psramdiag.c:414: phase1_fixed_reads();
	call	_phase1_fixed_reads
	jr	00110$
;psramdiag.c:418: }
00113$:
;psramdiag.c:383: for (phase = 1; ; phase++) {
	inc	-1 (ix)
	jr	00112$
;psramdiag.c:421: return 0;
;psramdiag.c:422: }
	inc	sp
	pop	ix
	ret
___str_9:
	.db 0x0d
	.db 0x0a
	.ascii "PSRAMDIAG v2 - RISKYMSX2"
	.db 0x0d
	.db 0x0a
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_10:
	.ascii "ANY key = run next phase."
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_11:
	.ascii "(interrupt-free key scan via SNSMAT,"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_12:
	.ascii " works even if the BIOS keyboard ISR"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_13:
	.ascii " never runs - deliberately robust"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_14:
	.ascii " on the emulated-cart bring-up)"
	.db 0x0d
	.db 0x0a
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_15:
	.ascii "PH1 fixed rd 0x5000 x4000"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_16:
	.ascii "PH2 LDIR 0x5000->0xD000"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_17:
	.ascii "PH3 isolated rd 0x5040 x2048"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_18:
	.ascii "PH4 pattern integrity 0x5000-0x50FF"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_19:
	.ascii "PH5 BIOS+rd 0x5080 x512"
	.db 0x0d
	.db 0x0a
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_20:
	.ascii "Set SRC SRAM/PSRAM on UART, reset,"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_21:
	.ascii "arm the 1660C, then press a key."
	.db 0x0d
	.db 0x0a
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_22:
	.ascii "PH1 fixed rd @0x5000 x4000"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_23:
	.ascii "PH1 done"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_24:
	.ascii "PH2 done"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_25:
	.ascii "PH3 isolated rd @0x5040"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_26:
	.ascii "PH3 done"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_27:
	.db 0x0d
	.db 0x0a
	.ascii "ALL DONE - PARKED in PH1 loop"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_28:
	.ascii "(LA: trigger on A=0x5000, M1=0)"
	.db 0x0d
	.db 0x0a
	.db 0x0d
	.db 0x0a
	.db 0x00
	.area _CODE
	.area _INITIALIZER
	.area _CABS (ABS)
