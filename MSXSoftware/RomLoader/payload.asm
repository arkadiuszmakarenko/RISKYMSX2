;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 4.2.0 #13081 (Linux)
;--------------------------------------------------------
	.module romloader
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
_s_files:
	.ds 1536
_s_count:
	.ds 2
_main_file_ptrs_65537_55:
	.ds 256
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
;romloader.c:98: static uint8_t snsmat(uint8_t row) __z88dk_fastcall __naked
;	---------------------------------
; Function snsmat
; ---------------------------------
_snsmat:
;romloader.c:112: __endasm;
	push	ix
	push	iy
	ld	a, l
	ld	iy, #0xFCC0
	ld	ix, #0x0141 ; SNSMAT
	call	0x001C ; CALSLT
	ld	l, a ; return value in L
	pop	iy
	pop	ix
	ret
;romloader.c:113: }
;romloader.c:116: static void chput(char c) __z88dk_fastcall
;	---------------------------------
; Function chput
; ---------------------------------
_chput:
;romloader.c:130: __endasm;
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
;romloader.c:131: }
	ret
;romloader.c:134: static void chgmod(uint8_t mode) __z88dk_fastcall
;	---------------------------------
; Function chgmod
; ---------------------------------
_chgmod:
;romloader.c:148: __endasm;
	push	ix
	push	iy
	ex	af, af'
	ld	a, l
	ld	iy, #0xFCC0
	ld	ix, #0x005F ; CHGMOD
	call	0x001C ; CALSLT
	ex	af, af'
	pop	iy
	pop	ix
;romloader.c:149: }
	ret
;romloader.c:156: static uint8_t mbox_status(void)     { return mbox[MBOX_STATUS]; }
;	---------------------------------
; Function mbox_status
; ---------------------------------
_mbox_status:
	ld	hl, (_mbox)
	ld	a, (hl)
	ret
_mbox:
	.dw #0x7ff0
;romloader.c:157: static uint8_t mbox_pop(void)        { return mbox[MBOX_DATA];   }
;	---------------------------------
; Function mbox_pop
; ---------------------------------
_mbox_pop:
	ld	hl, (_mbox)
	inc	hl
	ld	a, (hl)
	ret
;romloader.c:158: static void    mbox_push(uint8_t v)  { mbox[MBOX_CMD] = v;       }
;	---------------------------------
; Function mbox_push
; ---------------------------------
_mbox_push:
	ld	c, a
	ld	hl, (_mbox)
	ld	(hl), c
	ret
;romloader.c:161: static void mbox_wait_done(void)
;	---------------------------------
; Function mbox_wait_done
; ---------------------------------
_mbox_wait_done:
;romloader.c:163: while ((mbox_status() & ST_DONE) == 0U) {
00101$:
	call	_mbox_status
	bit	6, a
	jr	Z, 00101$
;romloader.c:165: }
	ret
;romloader.c:169: static void mbox_cmd(uint8_t cmd, const uint8_t *args, uint8_t argc)
;	---------------------------------
; Function mbox_cmd
; ---------------------------------
_mbox_cmd:
	call	___sdcc_enter_ix
	ld	c, a
;romloader.c:171: mbox_push(cmd);
	push	de
	ld	a, c
	call	_mbox_push
	pop	de
;romloader.c:172: for (uint8_t i = 0; i < argc; i++)
	ld	c, #0x00
00103$:
	ld	a, c
	sub	a, 4 (ix)
	jr	NC, 00101$
;romloader.c:173: mbox_push(args[i]);
	ld	l, c
	ld	h, #0x00
	add	hl, de
	ld	b, (hl)
	push	bc
	push	de
	ld	a, b
	call	_mbox_push
	pop	de
	pop	bc
;romloader.c:172: for (uint8_t i = 0; i < argc; i++)
	inc	c
	jr	00103$
00101$:
;romloader.c:174: mbox_wait_done();
	call	_mbox_wait_done
;romloader.c:175: }
	pop	ix
	pop	hl
	inc	sp
	jp	(hl)
;romloader.c:179: static void print(const char *s)
;	---------------------------------
; Function print
; ---------------------------------
_print:
	ex	de, hl
;romloader.c:181: while (*s)
00101$:
	ld	a, (de)
	or	a, a
	ret	Z
;romloader.c:182: chput(*s++);
	ld	l, a
;	spillPairReg hl
;	spillPairReg hl
	inc	de
	push	de
	call	_chput
	pop	de
;romloader.c:183: }
	jr	00101$
;romloader.c:185: static void print_dec(uint16_t v)
;	---------------------------------
; Function print_dec
; ---------------------------------
_print_dec:
	push	ix
	ld	ix,#0
	add	ix,sp
	push	af
	push	af
	push	af
	ex	de, hl
;romloader.c:189: if (v == 0) {
	ld	a, d
	or	a, e
	jr	NZ, 00113$
;romloader.c:190: chput('0');
	ld	l, #0x30
;	spillPairReg hl
;	spillPairReg hl
	call	_chput
;romloader.c:191: return;
	jr	00109$
;romloader.c:193: while (v) {
00113$:
	ld	c, #0x00
00103$:
	ld	a, d
	or	a, e
	jr	Z, 00115$
;romloader.c:194: buf[i++] = (char)('0' + (v % 10U));
	push	de
	ld	e, c
	ld	d, #0x00
	ld	hl, #2
	add	hl, sp
	add	hl, de
	pop	de
	inc	c
	push	de
	pop	iy
	push	hl
	push	bc
	push	iy
	ld	de, #0x000a
	push	iy
	pop	hl
	call	__moduint
	pop	iy
	pop	bc
	pop	hl
	ld	a, e
	add	a, #0x30
	ld	(hl), a
;romloader.c:195: v /= 10U;
	push	bc
	ld	de, #0x000a
	push	iy
	pop	hl
	call	__divuint
	pop	bc
	jr	00103$
;romloader.c:197: while (i)
00115$:
00106$:
	ld	a, c
	or	a, a
	jr	Z, 00109$
;romloader.c:198: chput(buf[--i]);
	dec	c
	ld	e, c
	ld	d, #0x00
	ld	hl, #0
	add	hl, sp
	add	hl, de
	ld	l, (hl)
;	spillPairReg hl
	push	bc
	call	_chput
	pop	bc
	jr	00106$
00109$:
;romloader.c:199: }
	ld	sp, ix
	pop	ix
	ret
;romloader.c:201: static void cls(void)
;	---------------------------------
; Function cls
; ---------------------------------
_cls:
;romloader.c:203: chput(0x0C);		/* CHPUT ctrl-L = clear screen */
	ld	l, #0x0c
;	spillPairReg hl
;	spillPairReg hl
;romloader.c:204: }
	jp	_chput
;romloader.c:209: static uint8_t any_key_down(void)
;	---------------------------------
; Function any_key_down
; ---------------------------------
_any_key_down:
;romloader.c:211: for (uint8_t row = 0; row < 10; row++) {
	ld	c, #0x00
00105$:
	ld	a, c
	sub	a, #0x0a
	jr	NC, 00103$
;romloader.c:212: if (snsmat(row) != 0xFFU)
	push	bc
	ld	l, c
;	spillPairReg hl
;	spillPairReg hl
	call	_snsmat
	pop	bc
	inc	l
	jr	Z, 00106$
;romloader.c:213: return 1;
	ld	a, #0x01
	ret
00106$:
;romloader.c:211: for (uint8_t row = 0; row < 10; row++) {
	inc	c
	jr	00105$
00103$:
;romloader.c:215: return 0;
	xor	a, a
;romloader.c:216: }
	ret
;romloader.c:219: static void delay_ms(uint16_t ms)
;	---------------------------------
; Function delay_ms
; ---------------------------------
_delay_ms:
;romloader.c:221: while (ms--)
00101$:
	ld	a, l
	ld	c, h
	dec	hl
	or	a, c
	ret	Z
;romloader.c:222: (void)any_key_down();
	push	hl
	call	_any_key_down
	pop	hl
;romloader.c:223: }
	jr	00101$
;romloader.c:233: static uint8_t wait_key_event(void)
;	---------------------------------
; Function wait_key_event
; ---------------------------------
_wait_key_event:
;romloader.c:236: while (!any_key_down()) { }
00101$:
	call	_any_key_down
	or	a, a
	jr	Z, 00101$
;romloader.c:240: while (any_key_down()) {
	ld	bc, #0x0000
00106$:
	push	bc
	call	_any_key_down
	pop	bc
	or	a, a
	jr	Z, 00108$
;romloader.c:241: held++;
	inc	bc
;romloader.c:242: if (held >= HOLD_MS)
	ld	e, c
	ld	d, b
	ld	a, e
	sub	a, #0x90
	ld	a, d
	sbc	a, #0x01
	jr	C, 00106$
;romloader.c:243: return 1U;		/* HOLD */
	ld	a, #0x01
	ret
00108$:
;romloader.c:246: delay_ms(30);
	ld	hl, #0x001e
	call	_delay_ms
;romloader.c:247: return 0U;
	xor	a, a
;romloader.c:248: }
	ret
;romloader.c:258: static void fetch_file_list(void)
;	---------------------------------
; Function fetch_file_list
; ---------------------------------
_fetch_file_list:
	call	___sdcc_enter_ix
	push	af
	dec	sp
;romloader.c:260: mbox_cmd(CMD_DIR_OPEN, 0, 0);
	xor	a, a
	push	af
	inc	sp
	ld	de, #0x0000
	xor	a, a
	call	_mbox_cmd
;romloader.c:261: uint8_t cnt_lo = mbox_pop();
	call	_mbox_pop
	ld	c, a
;romloader.c:262: uint8_t cnt_hi = mbox_pop();
	push	bc
	call	_mbox_pop
	pop	bc
;romloader.c:263: s_count = (uint16_t)cnt_lo | ((uint16_t)cnt_hi << 8);
	ld	b, #0x00
	ld	e, a
	xor	a, a
	or	a, c
	ld	(_s_count+0), a
	ld	a, e
	or	a, b
	ld	(_s_count+1), a
;romloader.c:264: if (s_count > MAX_FILES)
	ld	hl, (_s_count)
	ld	a, #0x80
	cp	a, l
	ld	a, #0x00
	sbc	a, h
	jr	NC, 00117$
;romloader.c:265: s_count = MAX_FILES;
	ld	hl, #0x0080
	ld	(_s_count), hl
;romloader.c:267: for (uint16_t i = 0; i < s_count; i++) {
00117$:
	ld	bc, #0x0000
00109$:
	ld	hl, #_s_count
	ld	a, c
	sub	a, (hl)
	inc	hl
	ld	a, b
	sbc	a, (hl)
	jr	NC, 00104$
;romloader.c:268: mbox_cmd(CMD_DIR_READ, 0, 0);
	push	bc
	xor	a, a
	push	af
	inc	sp
	ld	de, #0x0000
	ld	a, #0x01
	call	_mbox_cmd
	pop	bc
;romloader.c:269: for (uint8_t j = 0; j < 11; j++)
	ld	l, c
	ld	h, b
	add	hl, hl
	add	hl, bc
	add	hl, hl
	add	hl, hl
	ex	de, hl
	ld	hl, #_s_files
	add	hl, de
	ex	(sp), hl
	ld	-1 (ix), #0x00
00106$:
	ld	a, -1 (ix)
	sub	a, #0x0b
	jr	NC, 00103$
;romloader.c:270: s_files[i][j] = (char)mbox_pop();
	ld	a, -3 (ix)
	add	a, -1 (ix)
	ld	l, a
;	spillPairReg hl
;	spillPairReg hl
	ld	a, -2 (ix)
	adc	a, #0x00
	ld	h, a
;	spillPairReg hl
;	spillPairReg hl
	push	hl
	push	bc
	push	de
	call	_mbox_pop
	pop	de
	pop	bc
	pop	hl
	ld	(hl), a
;romloader.c:269: for (uint8_t j = 0; j < 11; j++)
	inc	-1 (ix)
	jr	00106$
00103$:
;romloader.c:271: s_files[i][11] = '\0';
	ld	hl, #_s_files
	add	hl, de
	ld	de, #0x000b
	add	hl, de
	ld	(hl), #0x00
;romloader.c:267: for (uint16_t i = 0; i < s_count; i++) {
	inc	bc
	jr	00109$
00104$:
;romloader.c:274: mbox_cmd(CMD_DIR_CLOSE, 0, 0);
	xor	a, a
	push	af
	inc	sp
	ld	de, #0x0000
	ld	a, #0x02
	call	_mbox_cmd
;romloader.c:275: }
	ld	sp, ix
	pop	ix
	ret
;romloader.c:280: static uint16_t choose(const char *title, const char *const *names,
;	---------------------------------
; Function choose
; ---------------------------------
_choose:
	push	ix
	ld	ix,#0
	add	ix,sp
	ld	iy, #-8
	add	iy, sp
	ld	sp, iy
	ld	-4 (ix), l
	ld	-3 (ix), h
;romloader.c:283: uint16_t sel = start < count ? start : 0;
	ld	a, 8 (ix)
	sub	a, 6 (ix)
	ld	a, 9 (ix)
	sbc	a, 7 (ix)
	jr	NC, 00116$
	ld	a, 8 (ix)
	ld	c, 9 (ix)
	jr	00117$
00116$:
	xor	a, a
	ld	c, a
00117$:
	ld	-2 (ix), a
	ld	-1 (ix), c
00112$:
;romloader.c:286: cls();
	call	_cls
;romloader.c:287: print(title);
	ld	l, -4 (ix)
;	spillPairReg hl
;	spillPairReg hl
	ld	h, -3 (ix)
;	spillPairReg hl
;	spillPairReg hl
	call	_print
;romloader.c:288: print("\r\n\r\n");
	ld	hl, #___str_0
	call	_print
;romloader.c:291: uint16_t top = sel >= 5U ? (uint16_t)(sel - 5U) : 0U;
	ld	a, -2 (ix)
	ld	-8 (ix), a
	ld	a, -1 (ix)
	ld	-7 (ix), a
	ld	a, -8 (ix)
	sub	a, #0x05
	ld	a, -7 (ix)
	sbc	a, #0x00
	jr	C, 00118$
	ld	a, -8 (ix)
	add	a, #0xfb
	ld	-6 (ix), a
	ld	a, -7 (ix)
	adc	a, #0xff
	ld	-5 (ix), a
	jr	00119$
00118$:
	xor	a, a
	ld	-6 (ix), a
	ld	-5 (ix), a
00119$:
	pop	hl
	pop	bc
	push	bc
	push	hl
;romloader.c:292: uint16_t bot = top + 10U;
	ld	e, c
	ld	d, b
	ld	hl, #0x000a
	add	hl, de
	ld	-6 (ix), l
	ld	-5 (ix), h
;romloader.c:293: if (bot > count)
	ld	a, 6 (ix)
	sub	a, -6 (ix)
	ld	a, 7 (ix)
	sbc	a, -5 (ix)
	jr	NC, 00127$
;romloader.c:294: bot = count;
	ld	a, 6 (ix)
	ld	-6 (ix), a
	ld	a, 7 (ix)
	ld	-5 (ix), a
;romloader.c:295: for (uint16_t i = top; i < bot; i++) {
00127$:
00110$:
	ld	a, c
	sub	a, -6 (ix)
	ld	a, b
	sbc	a, -5 (ix)
	jr	NC, 00103$
;romloader.c:296: print(i == sel ? ">" : " ");
	ld	l, -2 (ix)
	ld	h, -1 (ix)
	cp	a, a
	sbc	hl, bc
	jr	NZ, 00120$
	ld	hl, #___str_1
	jr	00121$
00120$:
	ld	hl, #___str_2
00121$:
	push	bc
	call	_print
	ld	hl, #___str_2
	call	_print
	pop	bc
;romloader.c:298: print(names[i]);
	ld	l, c
;	spillPairReg hl
;	spillPairReg hl
	ld	h, b
;	spillPairReg hl
;	spillPairReg hl
	add	hl, hl
	ex	de,hl
	ld	l, 4 (ix)
	ld	h, 5 (ix)
	add	hl, de
	ld	e, (hl)
	inc	hl
	ld	d, (hl)
	push	bc
	ex	de, hl
	call	_print
	ld	hl, #___str_3
	call	_print
	pop	bc
;romloader.c:295: for (uint16_t i = top; i < bot; i++) {
	inc	bc
	jr	00110$
00103$:
;romloader.c:301: print("\r\nTAP=NEXT  HOLD=SELECT\r\n");
	ld	hl, #___str_4
	call	_print
;romloader.c:303: if (wait_key_event() == 1U)
	call	_wait_key_event
	dec	a
	jr	NZ, 00105$
;romloader.c:304: return sel;		/* HOLD on the highlighted line */
	ld	e, -2 (ix)
	ld	d, -1 (ix)
	jr	00114$
00105$:
;romloader.c:305: sel++;
	inc	-2 (ix)
	jr	NZ, 00172$
	inc	-1 (ix)
00172$:
;romloader.c:306: if (sel >= count)
	ld	a, -2 (ix)
	sub	a, 6 (ix)
	ld	a, -1 (ix)
	sbc	a, 7 (ix)
	jp	C, 00112$
;romloader.c:307: sel = 0;
	xor	a, a
	ld	-2 (ix), a
	ld	-1 (ix), a
	jp	00112$
00114$:
;romloader.c:309: }
	ld	sp, ix
	pop	ix
	pop	hl
	pop	af
	pop	af
	pop	af
	jp	(hl)
___str_0:
	.db 0x0d
	.db 0x0a
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_1:
	.ascii ">"
	.db 0x00
___str_2:
	.ascii " "
	.db 0x00
___str_3:
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_4:
	.db 0x0d
	.db 0x0a
	.ascii "TAP=NEXT  HOLD=SELECT"
	.db 0x0d
	.db 0x0a
	.db 0x00
;romloader.c:313: int main(void)
;	---------------------------------
; Function main
; ---------------------------------
_main::
	call	___sdcc_enter_ix
	ld	hl, #-23
	add	hl, sp
	ld	sp, hl
;romloader.c:315: chgmod(0);		/* SCREEN 0, 40 columns */
	ld	l, #0x00
;	spillPairReg hl
;	spillPairReg hl
	call	_chgmod
;romloader.c:317: cls();
	call	_cls
;romloader.c:318: print("RISKYMSX2 ROM LOADER\r\n\r\n");
	ld	hl, #___str_17
	call	_print
;romloader.c:319: print("Reading USB directory...\r\n");
	ld	hl, #___str_18
	call	_print
;romloader.c:321: fetch_file_list();
	call	_fetch_file_list
;romloader.c:323: if (s_count == 0U) {
	ld	a, (_s_count+1)
	ld	hl, #_s_count
	or	a, (hl)
	jr	NZ, 00131$
;romloader.c:324: cls();
	call	_cls
;romloader.c:325: print("NO .ROM FILES ON USB STICK\r\n\r\n");
	ld	hl, #___str_19
	call	_print
;romloader.c:326: print("Plug a stick with .ROM files,\r\n");
	ld	hl, #___str_20
	call	_print
;romloader.c:327: print("power-cycle the MSX.\r\n");
	ld	hl, #___str_21
	call	_print
00111$:
;romloader.c:329: (void)wait_key_event();
	call	_wait_key_event
	jr	00111$
;romloader.c:334: for (uint16_t i = 0; i < s_count; i++)
00131$:
	ld	bc, #0x0000
00114$:
	ld	hl, #_s_count
	ld	a, c
	sub	a, (hl)
	inc	hl
	ld	a, b
	sbc	a, (hl)
	jr	NC, 00104$
;romloader.c:335: file_ptrs[i] = s_files[i];
	ld	l, c
;	spillPairReg hl
;	spillPairReg hl
	ld	h, b
;	spillPairReg hl
;	spillPairReg hl
	add	hl, hl
	ld	de, #_main_file_ptrs_65537_55
	add	hl, de
	ld	-2 (ix), l
	ld	-1 (ix), h
	ld	l, c
	ld	h, b
	add	hl, hl
	add	hl, bc
	add	hl, hl
	add	hl, hl
	ld	de, #_s_files
	add	hl, de
	ex	de, hl
	ld	l, -2 (ix)
	ld	h, -1 (ix)
	ld	(hl), e
	inc	hl
	ld	(hl), d
;romloader.c:334: for (uint16_t i = 0; i < s_count; i++)
	inc	bc
	jr	00114$
00104$:
;romloader.c:336: uint16_t sel = choose("SELECT ROM FILE:", file_ptrs, s_count, 0);
	ld	hl, #0x0000
	push	hl
	ld	hl, (_s_count)
	push	hl
	ld	hl, #_main_file_ptrs_65537_55
	push	hl
	ld	hl, #___str_5
	call	_choose
;romloader.c:339: cls();
	push	de
	call	_cls
	ld	hl, #___str_22
	call	_print
	pop	de
;romloader.c:341: print(s_files[sel]);
	ld	l, e
	ld	h, d
	add	hl, hl
	add	hl, de
	add	hl, hl
	add	hl, hl
	ld	de, #_s_files
	add	hl, de
;	spillPairReg hl
;	spillPairReg hl
;	spillPairReg hl
;	spillPairReg hl
	push	hl
	call	_print
	ld	hl, #___str_23
	call	_print
	pop	bc
;romloader.c:345: for (uint8_t i = 0; i < 11; i++)
	ld	hl, #0
	add	hl, sp
	ex	de, hl
	ld	-1 (ix), #0x00
00117$:
	ld	a, -1 (ix)
	sub	a, #0x0b
	jr	NC, 00105$
;romloader.c:346: args[i] = (uint8_t)s_files[sel][i];
	ld	l, -1 (ix)
	ld	h, #0x00
	add	hl, de
	push	iy
	ex	(sp), hl
	ld	l, -1 (ix)
;	spillPairReg hl
;	spillPairReg hl
	ex	(sp), hl
	ex	(sp), hl
	ld	h, #0x00
;	spillPairReg hl
;	spillPairReg hl
	ex	(sp), hl
	pop	iy
	add	iy, bc
	ld	a, 0 (iy)
	ld	(hl), a
;romloader.c:345: for (uint8_t i = 0; i < 11; i++)
	inc	-1 (ix)
	jr	00117$
00105$:
;romloader.c:347: mbox_cmd(CMD_LOAD_ROM, args, 11);
	ld	a, #0x0b
	push	af
	inc	sp
	ld	a, #0x03
	call	_mbox_cmd
;romloader.c:350: for (uint8_t i = 0; i < 4; i++)
	ld	c, #0x00
00120$:
	ld	a, c
	sub	a, #0x04
	jr	NC, 00106$
;romloader.c:351: len[i] = mbox_pop();
	ld	e, c
	ld	d, #0x00
	ld	hl, #11
	add	hl, sp
	add	hl, de
	push	hl
	push	bc
	call	_mbox_pop
	pop	bc
	pop	hl
	ld	(hl), a
;romloader.c:350: for (uint8_t i = 0; i < 4; i++)
	inc	c
	jr	00120$
00106$:
;romloader.c:352: uint32_t total = ((uint32_t)len[0] << 24)
	ld	c, -12 (ix)
	ld	-5 (ix), c
	xor	a, a
	ld	-8 (ix), a
	ld	-7 (ix), a
	ld	-6 (ix), a
	ld	c, -11 (ix)
	ld	b, #0x00
	ld	de, #0x0000
	ld	a, -8 (ix)
	or	a, e
	ld	-4 (ix), a
	ld	a, -7 (ix)
	or	a, d
	ld	-3 (ix), a
	ld	a, -6 (ix)
	or	a, c
	ld	-2 (ix), a
	ld	a, -5 (ix)
	or	a, b
	ld	-1 (ix), a
	ld	c, -10 (ix)
	xor	a, a
	ld	b, a
	ld	d, c
	ld	l, b
;	spillPairReg hl
;	spillPairReg hl
	ld	h, a
;	spillPairReg hl
;	spillPairReg hl
	ld	e, #0x00
	ld	a, -4 (ix)
	or	a, e
	ld	c, a
	ld	a, -3 (ix)
	or	a, d
	ld	b, a
	ld	a, -2 (ix)
	or	a, l
	ld	e, a
	ld	a, -1 (ix)
	or	a, h
	ld	d, a
	ld	a, -9 (ix)
	ld	-4 (ix), a
	xor	a, a
	ld	-3 (ix), a
	ld	-2 (ix), a
	ld	-1 (ix), a
	ld	a, c
	or	a, -4 (ix)
	ld	c, a
	ld	a, b
	or	a, -3 (ix)
	ld	b, a
	ld	a, e
	or	a, -2 (ix)
	ld	e, a
	ld	a, d
	or	a, -1 (ix)
	ld	d, a
	ld	-4 (ix), c
	ld	-3 (ix), b
	ld	-2 (ix), e
	ld	-1 (ix), d
;romloader.c:356: print("LOADED ");
	ld	hl, #___str_24
	call	_print
;romloader.c:357: print_dec((uint16_t)((total + 1023U) / 1024U));
	ld	a, -4 (ix)
	add	a, #0xff
	ld	a, -3 (ix)
	adc	a, #0x03
	ld	b, a
	ld	a, -2 (ix)
	adc	a, #0x00
	ld	e, a
	ld	a, -1 (ix)
	adc	a, #0x00
	ld	d, a
	ld	l, b
;	spillPairReg hl
;	spillPairReg hl
	ld	h, e
;	spillPairReg hl
;	spillPairReg hl
	ld	e, d
	ld	b, #0x02
00192$:
	srl	e
	rr	h
	rr	l
	djnz	00192$
	call	_print_dec
;romloader.c:358: print(" KiB\r\n\r\n");
	ld	hl, #___str_25
	call	_print
;romloader.c:375: uint8_t chosen = (uint8_t)choose("SELECT MAPPER:",
	ld	hl, #0x0001
	push	hl
	ld	l, #0x0a
	push	hl
	ld	hl, #_main_mapper_names_65539_63
	push	hl
	ld	hl, #___str_16
	call	_choose
	ld	-1 (ix), e
;romloader.c:379: cls();
	call	_cls
;romloader.c:380: print("MAPPER: ");
	ld	hl, #___str_26
	call	_print
;romloader.c:381: print(mapper_names[chosen]);
	ld	l, -1 (ix)
;	spillPairReg hl
;	spillPairReg hl
	ld	h, #0x00
;	spillPairReg hl
;	spillPairReg hl
	add	hl, hl
	ld	de, #_main_mapper_names_65539_63
	add	hl, de
	ld	e, (hl)
	inc	hl
	ld	d, (hl)
	ex	de, hl
	call	_print
;romloader.c:382: print("\r\n");
	ld	hl, #___str_27
	call	_print
;romloader.c:385: args[0] = (uint8_t)(chosen + 1U);	/* Cart_Mapper index */
	ld	hl, #14
	add	hl, sp
	ex	de, hl
	ld	a, -1 (ix)
	inc	a
	ld	(de), a
;romloader.c:386: mbox_cmd(CMD_SET_MAPPER, args, 1);
	ld	a, #0x01
	push	af
	inc	sp
	ld	a, #0x04
	call	_mbox_cmd
;romloader.c:387: uint8_t rc = mbox_pop();
	call	_mbox_pop
;romloader.c:388: if (rc != 0U) {
	or	a, a
	jr	Z, 00109$
;romloader.c:389: print("REFUSED! (PSRAM not ready?)\r\n");
	ld	hl, #___str_28
	call	_print
00122$:
;romloader.c:391: (void)wait_key_event();
	call	_wait_key_event
	jr	00122$
00109$:
;romloader.c:396: print("RESET...");
	ld	hl, #___str_29
	call	_print
;romloader.c:397: mbox_push(CMD_RESET);
	ld	a, #0x05
	call	_mbox_push
00125$:
;romloader.c:402: }
	jr	00125$
___str_5:
	.ascii "SELECT ROM FILE:"
	.db 0x00
_main_mapper_names_65539_63:
	.dw ___str_6
	.dw ___str_7
	.dw ___str_8
	.dw ___str_9
	.dw ___str_10
	.dw ___str_11
	.dw ___str_12
	.dw ___str_13
	.dw ___str_14
	.dw ___str_15
___str_6:
	.ascii "ROM16K"
	.db 0x00
___str_7:
	.ascii "ROM32K"
	.db 0x00
___str_8:
	.ascii "ROM48K"
	.db 0x00
___str_9:
	.ascii "KONAMI"
	.db 0x00
___str_10:
	.ascii "KONAMINOSCC"
	.db 0x00
___str_11:
	.ascii "ASCII8K"
	.db 0x00
___str_12:
	.ascii "ASCII16K"
	.db 0x00
___str_13:
	.ascii "NEO8"
	.db 0x00
___str_14:
	.ascii "NEO16"
	.db 0x00
___str_15:
	.ascii "KONAMISCC"
	.db 0x00
___str_16:
	.ascii "SELECT MAPPER:"
	.db 0x00
___str_17:
	.ascii "RISKYMSX2 ROM LOADER"
	.db 0x0d
	.db 0x0a
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_18:
	.ascii "Reading USB directory..."
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_19:
	.ascii "NO .ROM FILES ON USB STICK"
	.db 0x0d
	.db 0x0a
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_20:
	.ascii "Plug a stick with .ROM files,"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_21:
	.ascii "power-cycle the MSX."
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_22:
	.ascii "LOADING "
	.db 0x00
___str_23:
	.ascii "..."
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_24:
	.ascii "LOADED "
	.db 0x00
___str_25:
	.ascii " KiB"
	.db 0x0d
	.db 0x0a
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_26:
	.ascii "MAPPER: "
	.db 0x00
___str_27:
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_28:
	.ascii "REFUSED! (PSRAM not ready?)"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_29:
	.ascii "RESET..."
	.db 0x00
	.area _CODE
	.area _INITIALIZER
	.area _CABS (ABS)
