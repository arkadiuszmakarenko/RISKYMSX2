;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 4.2.0 #13081 (Linux)
;--------------------------------------------------------
	.module maptest
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
;maptest.c:47: void putchar_msx(char c) __z88dk_fastcall
;	---------------------------------
; Function putchar_msx
; ---------------------------------
_putchar_msx::
;maptest.c:61: __endasm;
	push	ix
	push	iy
	ex	af, af'
	ld	a, l
	ld	iy, #0xFCC0
	ld	ix, #0x00A2
	call	0x001C
	ex	af, af'
	pop	iy
	pop	ix
;maptest.c:62: }
	ret
_mapper_names:
	.dw __str_0
	.dw __str_1
	.dw __str_2
	.dw __str_3
	.dw __str_4
	.dw __str_5
	.dw __str_6
	.dw __str_7
	.dw __str_8
	.dw __str_9
	.dw __str_10
__str_0:
	.ascii "ROM16K"
	.db 0x00
__str_1:
	.ascii "ROM32K"
	.db 0x00
__str_2:
	.ascii "ROM48K"
	.db 0x00
__str_3:
	.ascii "KONAMI"
	.db 0x00
__str_4:
	.ascii "KONAMINOSCC"
	.db 0x00
__str_5:
	.ascii "ASCII8K"
	.db 0x00
__str_6:
	.ascii "ASCII16K"
	.db 0x00
__str_7:
	.ascii "NEO8"
	.db 0x00
__str_8:
	.ascii "NEO16"
	.db 0x00
__str_9:
	.ascii "KONAMISCC"
	.db 0x00
__str_10:
	.ascii "FLASH"
	.db 0x00
;maptest.c:64: char wait_key(void) __naked
;	---------------------------------
; Function wait_key
; ---------------------------------
_wait_key::
;maptest.c:72: __endasm;
	ld	iy, #0xFCC0
	ld	ix, #0x009F
	call	0x001C
	ld	l, a
	ret
;maptest.c:73: }
;maptest.c:75: static void print(const char *s)
;	---------------------------------
; Function print
; ---------------------------------
_print:
	ex	de, hl
;maptest.c:77: while (*s)
00101$:
	ld	a, (de)
	or	a, a
	ret	Z
;maptest.c:78: putchar_msx(*s++);
	ld	l, a
;	spillPairReg hl
;	spillPairReg hl
	inc	de
	push	de
	call	_putchar_msx
	pop	de
;maptest.c:79: }
	jr	00101$
;maptest.c:81: static void print_hex(uint8_t v)
;	---------------------------------
; Function print_hex
; ---------------------------------
_print_hex:
	call	___sdcc_enter_ix
	ld	hl, #-17
	add	hl, sp
	ld	sp, hl
	ld	c, a
;maptest.c:83: const char hex[] = "0123456789ABCDEF";
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
;maptest.c:84: putchar_msx(hex[(v >> 4) & 0x0FU]);
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
;maptest.c:85: putchar_msx(hex[v & 0x0FU]);
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
;maptest.c:86: }
	ld	sp, ix
	pop	ix
	ret
;maptest.c:88: static void mbox_wait_done(void)
;	---------------------------------
; Function mbox_wait_done
; ---------------------------------
_mbox_wait_done:
;maptest.c:90: while ((*MBOX_STATUS & ST_DONE) == 0U) { }
00101$:
	ld	a, (#0x7ff0)
	bit	6, a
	jr	Z, 00101$
;maptest.c:91: }
	ret
;maptest.c:93: static uint32_t mbox_load_rom(const uint8_t *name83)
;	---------------------------------
; Function mbox_load_rom
; ---------------------------------
_mbox_load_rom:
	push	ix
	ld	ix,#0
	add	ix,sp
	push	af
	push	af
	ex	de, hl
;maptest.c:95: *MBOX_CMD = CMD_LOAD_ROM;
	ld	hl, #0x7ff0
	ld	(hl), #0x03
;maptest.c:96: for (uint8_t i = 0; i < 11; i++) {
	ld	c, #0x00
00104$:
	ld	a, c
	sub	a, #0x0b
	jr	NC, 00101$
;maptest.c:97: *MBOX_CMD = name83[i];
	ld	l, c
	ld	h, #0x00
	add	hl, de
	ld	a, (hl)
	ld	(#0x7ff0),a
;maptest.c:96: for (uint8_t i = 0; i < 11; i++) {
	inc	c
	jr	00104$
00101$:
;maptest.c:99: mbox_wait_done();
	call	_mbox_wait_done
;maptest.c:100: uint32_t total = 0;
	ld	bc, #0x0000
	ld	hl, #0x0000
;maptest.c:101: for (uint8_t i = 0; i < 4; i++) {
	xor	a, a
00107$:
	cp	a, #0x04
	jr	NC, 00102$
;maptest.c:102: total = (total << 8) | (uint32_t)*MBOX_DATA;
	ld	-3 (ix), c
	ld	-2 (ix), b
	ld	-1 (ix), l
	ld	-4 (ix), #0x00
	ld	hl, #0x7ff1
	ld	c, (hl)
	ld	b, #0x00
	ld	de, #0x0000
	push	af
	ld	a, -4 (ix)
	or	a, c
	ld	c, a
	ld	a, -3 (ix)
	or	a, b
	ld	b, a
	ld	a, -2 (ix)
	or	a, e
	ld	l, a
;	spillPairReg hl
;	spillPairReg hl
	ld	a, -1 (ix)
	or	a, d
	ld	h, a
;	spillPairReg hl
;	spillPairReg hl
	pop	af
;maptest.c:101: for (uint8_t i = 0; i < 4; i++) {
	inc	a
	jr	00107$
00102$:
;maptest.c:104: return total;
	ld	e, c
	ld	d, b
;maptest.c:105: }
	ld	sp, ix
	pop	ix
	ret
;maptest.c:107: static void mbox_cmd_set_mapper(uint8_t idx)
;	---------------------------------
; Function mbox_cmd_set_mapper
; ---------------------------------
_mbox_cmd_set_mapper:
	ld	c, a
;maptest.c:109: *MBOX_CMD = CMD_SET_MAPPER;
	ld	hl, #0x7ff0
	ld	(hl), #0x04
;maptest.c:110: *MBOX_CMD = idx;
	ld	(hl), c
;maptest.c:111: mbox_wait_done();
;maptest.c:112: }
	jp	_mbox_wait_done
;maptest.c:116: static uint16_t mbox_dir_list(uint8_t *out_names, uint16_t max_out)
;	---------------------------------
; Function mbox_dir_list
; ---------------------------------
_mbox_dir_list:
	push	ix
	ld	ix,#0
	add	ix,sp
	push	af
	push	af
	push	af
	dec	sp
	ld	-5 (ix), l
	ld	-4 (ix), h
	inc	sp
	inc	sp
	push	de
;maptest.c:118: *MBOX_CMD = CMD_DIR_OPEN;
	ld	hl, #0x7ff0
	ld	(hl), #0x00
;maptest.c:119: mbox_wait_done();
	call	_mbox_wait_done
;maptest.c:120: uint16_t count = (uint16_t)((uint16_t)*MBOX_DATA
	ld	a, (#0x7ff1)
	ld	c, a
	ld	b, #0x00
	ld	a, (#0x7ff1)
	ld	d, a
	xor	a, a
	or	a, c
	ld	e, a
	ld	a, d
	or	a, b
	ld	d, a
;maptest.c:122: for (uint16_t i = 0; i < count && i < max_out; i++) {
	xor	a, a
	ld	-3 (ix), a
	ld	-2 (ix), a
00108$:
	ld	a, -3 (ix)
	sub	a, e
	ld	a, -2 (ix)
	sbc	a, d
	jr	NC, 00102$
	ld	a, -3 (ix)
	sub	a, -7 (ix)
	ld	a, -2 (ix)
	sbc	a, -6 (ix)
	jr	NC, 00102$
;maptest.c:123: *MBOX_CMD = CMD_DIR_READ;
	ld	hl, #0x7ff0
	ld	(hl), #0x01
;maptest.c:124: mbox_wait_done();
	push	de
	call	_mbox_wait_done
	pop	de
;maptest.c:125: for (uint8_t j = 0; j < 11; j++) {
	ld	-1 (ix), #0x00
00104$:
	ld	a, -1 (ix)
	sub	a, #0x0b
	jr	NC, 00109$
;maptest.c:126: out_names[i * 11 + j] = *MBOX_DATA;
	ld	c, -3 (ix)
	ld	b, -2 (ix)
	ld	l, c
	ld	h, b
	add	hl, hl
	add	hl, hl
	add	hl, bc
	add	hl, hl
	add	hl, bc
	ld	c, -1 (ix)
	ld	b, #0x00
	add	hl, bc
	ld	a, l
	add	a, -5 (ix)
	ld	c, a
	ld	a, h
	adc	a, -4 (ix)
	ld	b, a
	ld	a, (#0x7ff1)
	ld	(bc), a
;maptest.c:125: for (uint8_t j = 0; j < 11; j++) {
	inc	-1 (ix)
	jr	00104$
00109$:
;maptest.c:122: for (uint16_t i = 0; i < count && i < max_out; i++) {
	inc	-3 (ix)
	jr	NZ, 00108$
	inc	-2 (ix)
	jr	00108$
00102$:
;maptest.c:129: return count;
;maptest.c:130: }
	ld	sp, ix
	pop	ix
	ret
;maptest.c:132: static void print_name(const uint8_t *name83)
;	---------------------------------
; Function print_name
; ---------------------------------
_print_name:
	ex	de, hl
;maptest.c:134: for (uint8_t i = 0; i < 8 && name83[i] != ' '; i++) {
	ld	c, #0x00
00107$:
	ld	a, c
	sub	a, #0x08
	jr	NC, 00101$
	ld	l, c
	ld	h, #0x00
	add	hl, de
	ld	l, (hl)
;	spillPairReg hl
	ld	a, l
	sub	a, #0x20
	jr	Z, 00101$
;maptest.c:135: putchar_msx((char)name83[i]);
	push	bc
	push	de
	call	_putchar_msx
	pop	de
	pop	bc
;maptest.c:134: for (uint8_t i = 0; i < 8 && name83[i] != ' '; i++) {
	inc	c
	jr	00107$
00101$:
;maptest.c:137: if (name83[8] != ' ') {
	ld	c, e
	ld	b, d
	ld	hl, #8
	add	hl, bc
	ld	a, (hl)
	sub	a, #0x20
	ret	Z
;maptest.c:138: putchar_msx('.');
	push	de
	ld	l, #0x2e
;	spillPairReg hl
;	spillPairReg hl
	call	_putchar_msx
	pop	de
;maptest.c:139: for (uint8_t i = 8; i < 11 && name83[i] != ' '; i++) {
	ld	c, #0x08
00111$:
	ld	a, c
	sub	a, #0x0b
	ret	NC
	ld	l, c
	ld	h, #0x00
	add	hl, de
	ld	l, (hl)
;	spillPairReg hl
	ld	a, l
	sub	a, #0x20
	ret	Z
;maptest.c:140: putchar_msx((char)name83[i]);
	push	bc
	push	de
	call	_putchar_msx
	pop	de
	pop	bc
;maptest.c:139: for (uint8_t i = 8; i < 11 && name83[i] != ' '; i++) {
	inc	c
;maptest.c:143: }
	jr	00111$
;maptest.c:145: int main(void)
;	---------------------------------
; Function main
; ---------------------------------
_main::
	call	___sdcc_enter_ix
	ld	hl, #-113
	add	hl, sp
	ld	sp, hl
;maptest.c:147: print("USB .ROM LOADER\r\n\r\n");
	ld	hl, #___str_12
	call	_print
;maptest.c:148: print("scanning USB...\r\n");
	ld	hl, #___str_13
	call	_print
;maptest.c:151: uint16_t count = mbox_dir_list(names, MAX_DISPLAY);
	ld	de, #0x000a
	ld	hl, #0
	add	hl, sp
	call	_mbox_dir_list
	ld	-3 (ix), e
	ld	-2 (ix), d
;maptest.c:152: print("found ");
	ld	hl, #___str_14
	call	_print
;maptest.c:153: if (count >= 100) putchar_msx('0' + (char)(count / 100));
	ld	c, -3 (ix)
	ld	b, -2 (ix)
	ld	a, c
	sub	a, #0x64
	ld	a, b
	sbc	a, #0x00
	jr	C, 00102$
	push	bc
	ld	de, #0x0064
	ld	l, c
;	spillPairReg hl
;	spillPairReg hl
	ld	h, b
;	spillPairReg hl
;	spillPairReg hl
	call	__divuint
	pop	bc
	ld	a, e
	add	a, #0x30
	ld	l, a
;	spillPairReg hl
;	spillPairReg hl
	push	bc
	call	_putchar_msx
	pop	bc
00102$:
;maptest.c:154: if (count >= 10)  putchar_msx('0' + (char)((count / 10) % 10));
	ld	a, c
	sub	a, #0x0a
	ld	a, b
	sbc	a, #0x00
	jr	C, 00104$
	push	bc
	ld	de, #0x000a
	ld	l, c
;	spillPairReg hl
;	spillPairReg hl
	ld	h, b
;	spillPairReg hl
;	spillPairReg hl
	call	__divuint
	ex	de, hl
	ld	de, #0x000a
	call	__moduint
	pop	bc
	ld	a, e
	add	a, #0x30
	ld	l, a
;	spillPairReg hl
;	spillPairReg hl
	push	bc
	call	_putchar_msx
	pop	bc
00104$:
;maptest.c:155: putchar_msx('0' + (char)(count % 10));
	push	bc
	ld	de, #0x000a
	ld	l, c
;	spillPairReg hl
;	spillPairReg hl
	ld	h, b
;	spillPairReg hl
;	spillPairReg hl
	call	__moduint
	pop	bc
	ld	a, e
	add	a, #0x30
	ld	l, a
;	spillPairReg hl
;	spillPairReg hl
	push	bc
	call	_putchar_msx
	ld	hl, #___str_15
	call	_print
	pop	bc
	ld	a, #0x0a
	cp	a, c
	ld	a, #0x00
	sbc	a, b
	ld	a, #0x00
	rla
	ld	-1 (ix), a
00140$:
;maptest.c:159: print("0 = FLASH menu\r\n");
	ld	hl, #___str_16
	call	_print
;maptest.c:160: for (uint16_t i = 0; i < count && i < MAX_DISPLAY; i++) {
	ld	bc, #0x0000
00134$:
	ld	a, c
	sub	a, -3 (ix)
	ld	a, b
	sbc	a, -2 (ix)
	jr	NC, 00105$
	ld	e, c
	ld	d, b
	ld	a, e
	sub	a, #0x0a
	ld	a, d
	sbc	a, #0x00
	jr	NC, 00105$
;maptest.c:161: putchar_msx('1' + (char)i);
	ld	a, c
	add	a, #0x31
	ld	l, a
;	spillPairReg hl
;	spillPairReg hl
	push	bc
	push	de
	call	_putchar_msx
	ld	hl, #___str_17
	call	_print
	pop	de
	pop	bc
;maptest.c:163: print_name(names + i * 11);
	ld	l, e
	ld	h, d
	add	hl, hl
	add	hl, hl
	add	hl, de
	add	hl, hl
	add	hl, de
	ex	de, hl
	ld	hl, #0
	add	hl, sp
	add	hl, de
	push	bc
	call	_print_name
	ld	hl, #___str_18
	call	_print
	pop	bc
;maptest.c:160: for (uint16_t i = 0; i < count && i < MAX_DISPLAY; i++) {
	inc	bc
	jr	00134$
00105$:
;maptest.c:166: if (count > MAX_DISPLAY) {
	ld	a, -1 (ix)
	or	a, a
	jr	Z, 00107$
;maptest.c:167: print("(more on USB)\r\n");
	ld	hl, #___str_19
	call	_print
00107$:
;maptest.c:169: print("\r\npick a number: ");
	ld	hl, #___str_20
	call	_print
;maptest.c:171: char k = wait_key();
	call	_wait_key
	ld	l, a
;	spillPairReg hl
;	spillPairReg hl
;maptest.c:172: putchar_msx(k);
	push	hl
	call	_putchar_msx
	ld	hl, #___str_18
	call	_print
	pop	hl
;maptest.c:175: if (k == '0') {
	ld	a, l
	sub	a, #0x30
	jr	NZ, 00110$
;maptest.c:176: mbox_cmd_set_mapper(11);
	ld	a, #0x0b
	call	_mbox_cmd_set_mapper
;maptest.c:177: *MBOX_CMD = CMD_RESET;
	ld	hl, #0x7ff0
	ld	(hl), #0x05
00137$:
	jr	00137$
00110$:
;maptest.c:182: if (k >= '1' && k <= '9') file_idx = (uint8_t)(k - '1');
	ld	a, l
	sub	a, #0x31
	jr	C, 00112$
	ld	a, #0x39
	sub	a, l
	jr	C, 00112$
	ld	a, l
	add	a, #0xcf
	ld	c, a
	jr	00113$
00112$:
;maptest.c:183: else { print(" ?\r\n"); continue; }
	ld	hl, #___str_21
	call	_print
	jp	00140$
00113$:
;maptest.c:184: if (file_idx >= count) { print(" out of range\r\n"); continue; }
	ld	b, c
	ld	e, #0x00
	ld	a, b
	sub	a, -3 (ix)
	ld	a, e
	sbc	a, -2 (ix)
	jr	C, 00116$
	ld	hl, #___str_22
	call	_print
	jp	00140$
00116$:
;maptest.c:186: print("LOAD_ROM ");
	push	bc
	ld	hl, #___str_23
	call	_print
	pop	bc
;maptest.c:187: print_name(names + file_idx * 11);
	ld	b, #0x00
	ld	l, c
	ld	h, b
	add	hl, hl
	add	hl, hl
	add	hl, bc
	add	hl, hl
	add	hl, bc
	ex	de, hl
	ld	hl, #0
	add	hl, sp
	add	hl, de
	ld	e, l
	ld	d, h
	ex	de, hl
	push	de
	call	_print_name
	ld	hl, #___str_24
	call	_print
	pop	hl
;maptest.c:190: uint32_t total = mbox_load_rom(names + file_idx * 11);
	call	_mbox_load_rom
	ld	c, e
	ld	b, d
;maptest.c:191: print("loaded ");
	push	hl
	push	bc
	ld	hl, #___str_25
	call	_print
	pop	bc
	pop	hl
;maptest.c:192: if (total == 0) {
	ld	a, h
	or	a, l
	or	a, b
	or	a, c
	jr	NZ, 00118$
;maptest.c:193: print("0 bytes (USB error)\r\n");
	ld	hl, #___str_26
	call	_print
;maptest.c:194: continue;
	jp	00140$
00118$:
;maptest.c:196: print_hex((uint8_t)(total >> 24));
	ld	e, h
	push	hl
	push	bc
	ld	a, e
	call	_print_hex
	pop	bc
	pop	hl
;maptest.c:197: print_hex((uint8_t)(total >> 16));
	ld	e, l
	push	hl
	push	bc
	ld	a, e
	call	_print_hex
	pop	bc
	pop	hl
;maptest.c:198: print_hex((uint8_t)(total >> 8));
	ld	e, b
	push	hl
	push	bc
	ld	a, e
	call	_print_hex
	pop	bc
	pop	hl
;maptest.c:199: print_hex((uint8_t)total);
	ld	a, c
	call	_print_hex
;maptest.c:200: print(" bytes\r\n\r\n");
	ld	hl, #___str_27
	call	_print
;maptest.c:202: print("pick a mapper (default 2=ROM32K for plain .ROM):\r\n");
	ld	hl, #___str_28
	call	_print
;maptest.c:203: print(" 1=ROM16K 2=ROM32K 3=ROM48K\r\n");
	ld	hl, #___str_29
	call	_print
;maptest.c:204: print(" 4=KONAMI 5=KONAMINOSCC 6=ASCII8K\r\n");
	ld	hl, #___str_30
	call	_print
;maptest.c:205: print(" 7=ASCII16K 8=NEO8 9=NEO16 0=FLASH\r\n");
	ld	hl, #___str_31
	call	_print
;maptest.c:206: print("mapper (ENTER=2): ");
	ld	hl, #___str_32
	call	_print
;maptest.c:208: char m = wait_key();
	call	_wait_key
;	spillPairReg hl
;	spillPairReg hl
;maptest.c:209: if (m == '\r') {
	ld	l, a
	sub	a, #0x0d
	jr	NZ, 00120$
;maptest.c:210: m = '2';
	ld	l, #0x32
;	spillPairReg hl
;	spillPairReg hl
;maptest.c:211: print("(default)\r\n");
	push	hl
	ld	hl, #___str_33
	call	_print
	pop	hl
	jr	00121$
00120$:
;maptest.c:213: putchar_msx(m);
	push	hl
	call	_putchar_msx
	ld	hl, #___str_18
	call	_print
	pop	hl
00121$:
;maptest.c:218: if (m >= '1' && m <= '9') midx = (uint8_t)(m - '0');
	ld	a, l
	sub	a, #0x31
	jr	C, 00126$
	ld	a, #0x39
	sub	a, l
	jr	C, 00126$
	ld	a, l
	add	a, #0xd0
	ld	c, a
	jr	00127$
00126$:
;maptest.c:219: else if (m == '0') midx = 11;
	ld	a, l
	sub	a, #0x30
	jr	NZ, 00123$
	ld	c, #0x0b
	jr	00127$
00123$:
;maptest.c:220: else { print(" ?\r\n"); continue; }
	ld	hl, #___str_21
	call	_print
	jp	00140$
00127$:
;maptest.c:222: print("SET_MAPPER ");
	push	bc
	ld	hl, #___str_34
	call	_print
	pop	bc
;maptest.c:223: print(mapper_names[midx - 1]);
	ld	a, c
	dec	a
	ld	l, a
;	spillPairReg hl
;	spillPairReg hl
	rlca
	sbc	a, a
	ld	h, a
	add	hl, hl
	ld	de, #_mapper_names
	add	hl, de
	ld	e, (hl)
	inc	hl
	ld	d, (hl)
	push	bc
	ex	de, hl
	call	_print
	ld	hl, #___str_35
	call	_print
	pop	bc
;maptest.c:226: mbox_cmd_set_mapper(midx);
	ld	a, c
	call	_mbox_cmd_set_mapper
;maptest.c:227: *MBOX_CMD = CMD_RESET;
	ld	hl, #0x7ff0
	ld	(hl), #0x05
00139$:
;maptest.c:231: }
	jr	00139$
___str_12:
	.ascii "USB .ROM LOADER"
	.db 0x0d
	.db 0x0a
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_13:
	.ascii "scanning USB..."
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_14:
	.ascii "found "
	.db 0x00
___str_15:
	.ascii " .ROM files"
	.db 0x0d
	.db 0x0a
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_16:
	.ascii "0 = FLASH menu"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_17:
	.ascii ": "
	.db 0x00
___str_18:
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_19:
	.ascii "(more on USB)"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_20:
	.db 0x0d
	.db 0x0a
	.ascii "pick a number: "
	.db 0x00
___str_21:
	.ascii " ?"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_22:
	.ascii " out of range"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_23:
	.ascii "LOAD_ROM "
	.db 0x00
___str_24:
	.ascii " ..."
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_25:
	.ascii "loaded "
	.db 0x00
___str_26:
	.ascii "0 bytes (USB error)"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_27:
	.ascii " bytes"
	.db 0x0d
	.db 0x0a
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_28:
	.ascii "pick a mapper (default 2=ROM32K for plain .ROM):"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_29:
	.ascii " 1=ROM16K 2=ROM32K 3=ROM48K"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_30:
	.ascii " 4=KONAMI 5=KONAMINOSCC 6=ASCII8K"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_31:
	.ascii " 7=ASCII16K 8=NEO8 9=NEO16 0=FLASH"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_32:
	.ascii "mapper (ENTER=2): "
	.db 0x00
___str_33:
	.ascii "(default)"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_34:
	.ascii "SET_MAPPER "
	.db 0x00
___str_35:
	.db 0x0d
	.db 0x0a
	.ascii "RESET..."
	.db 0x0d
	.db 0x0a
	.db 0x00
	.area _CODE
	.area _INITIALIZER
	.area _CABS (ABS)
