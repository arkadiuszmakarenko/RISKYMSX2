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
	.ds 1664
_s_count:
	.ds 2
_main_file_ptrs_65537_68:
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
;romloader.c:102: static uint8_t snsmat(uint8_t row) __z88dk_fastcall __naked
;	---------------------------------
; Function snsmat
; ---------------------------------
_snsmat:
;romloader.c:116: __endasm;
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
;romloader.c:117: }
;romloader.c:120: static void chput(char c) __z88dk_fastcall
;	---------------------------------
; Function chput
; ---------------------------------
_chput:
;romloader.c:134: __endasm;
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
;romloader.c:135: }
	ret
;romloader.c:138: static void chgmod(uint8_t mode) __z88dk_fastcall
;	---------------------------------
; Function chgmod
; ---------------------------------
_chgmod:
;romloader.c:152: __endasm;
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
;romloader.c:153: }
	ret
;romloader.c:160: static uint8_t mbox_status(void)     { return mbox[MBOX_STATUS]; }
;	---------------------------------
; Function mbox_status
; ---------------------------------
_mbox_status:
	ld	hl, (_mbox)
	ld	a, (hl)
	ret
_mbox:
	.dw #0x7ff0
;romloader.c:161: static uint8_t mbox_pop(void)        { return mbox[MBOX_DATA];   }
;	---------------------------------
; Function mbox_pop
; ---------------------------------
_mbox_pop:
	ld	hl, (_mbox)
	inc	hl
	ld	a, (hl)
	ret
;romloader.c:162: static void    mbox_push(uint8_t v)  { mbox[MBOX_CMD] = v;       }
;	---------------------------------
; Function mbox_push
; ---------------------------------
_mbox_push:
	ld	c, a
	ld	hl, (_mbox)
	ld	(hl), c
	ret
;romloader.c:165: static void mbox_wait_done(void)
;	---------------------------------
; Function mbox_wait_done
; ---------------------------------
_mbox_wait_done:
;romloader.c:167: while ((mbox_status() & ST_DONE) == 0U) {
00101$:
	call	_mbox_status
	bit	6, a
	jr	Z, 00101$
;romloader.c:169: }
	ret
;romloader.c:173: static void mbox_cmd(uint8_t cmd, const uint8_t *args, uint8_t argc)
;	---------------------------------
; Function mbox_cmd
; ---------------------------------
_mbox_cmd:
	call	___sdcc_enter_ix
	ld	c, a
;romloader.c:175: mbox_push(cmd);
	push	de
	ld	a, c
	call	_mbox_push
	pop	de
;romloader.c:176: for (uint8_t i = 0; i < argc; i++)
	ld	c, #0x00
00103$:
	ld	a, c
	sub	a, 4 (ix)
	jr	NC, 00101$
;romloader.c:177: mbox_push(args[i]);
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
;romloader.c:176: for (uint8_t i = 0; i < argc; i++)
	inc	c
	jr	00103$
00101$:
;romloader.c:178: mbox_wait_done();
	call	_mbox_wait_done
;romloader.c:179: }
	pop	ix
	pop	hl
	inc	sp
	jp	(hl)
;romloader.c:183: static void print(const char *s)
;	---------------------------------
; Function print
; ---------------------------------
_print:
	ex	de, hl
;romloader.c:185: while (*s)
00101$:
	ld	a, (de)
	or	a, a
	ret	Z
;romloader.c:186: chput(*s++);
	ld	l, a
;	spillPairReg hl
;	spillPairReg hl
	inc	de
	push	de
	call	_chput
	pop	de
;romloader.c:187: }
	jr	00101$
;romloader.c:189: static void print_dec(uint16_t v)
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
;romloader.c:193: if (v == 0) {
	ld	a, d
	or	a, e
	jr	NZ, 00113$
;romloader.c:194: chput('0');
	ld	l, #0x30
;	spillPairReg hl
;	spillPairReg hl
	call	_chput
;romloader.c:195: return;
	jr	00109$
;romloader.c:197: while (v) {
00113$:
	ld	c, #0x00
00103$:
	ld	a, d
	or	a, e
	jr	Z, 00115$
;romloader.c:198: buf[i++] = (char)('0' + (v % 10U));
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
;romloader.c:199: v /= 10U;
	push	bc
	ld	de, #0x000a
	push	iy
	pop	hl
	call	__divuint
	pop	bc
	jr	00103$
;romloader.c:201: while (i)
00115$:
00106$:
	ld	a, c
	or	a, a
	jr	Z, 00109$
;romloader.c:202: chput(buf[--i]);
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
;romloader.c:203: }
	ld	sp, ix
	pop	ix
	ret
;romloader.c:205: static void cls(void)
;	---------------------------------
; Function cls
; ---------------------------------
_cls:
;romloader.c:207: chput(0x0C);		/* CHPUT ctrl-L = clear screen + home cursor */
	ld	l, #0x0c
;	spillPairReg hl
;	spillPairReg hl
;romloader.c:208: }
	jp	_chput
;romloader.c:245: static void delay_ms(uint16_t ms)
;	---------------------------------
; Function delay_ms
; ---------------------------------
_delay_ms:
;romloader.c:247: while (ms--)
00101$:
	ld	a, l
	ld	c, h
	dec	hl
	or	a, c
	ret	Z
;romloader.c:248: (void)snsmat(0);
	push	hl
	ld	l, #0x00
;	spillPairReg hl
;	spillPairReg hl
	call	_snsmat
	pop	hl
;romloader.c:249: }
	jr	00101$
;romloader.c:253: static uint8_t wait_arrow(void)
;	---------------------------------
; Function wait_arrow
; ---------------------------------
_wait_arrow:
	call	___sdcc_enter_ix
	dec	sp
;romloader.c:256: while ((snsmat(8) & (R8_DOWN | R8_UP | R8_SPACE)) ==
00101$:
	ld	l, #0x08
;	spillPairReg hl
;	spillPairReg hl
	call	_snsmat
	ld	a, l
	and	a, #0x61
	ld	c, a
	ld	b, #0x00
	ld	a, c
	sub	a, #0x61
	or	a, b
	jr	Z, 00101$
;romloader.c:260: delay_ms(10);
	ld	hl, #0x000a
	call	_delay_ms
;romloader.c:261: uint8_t row = snsmat(8);
	ld	l, #0x08
;	spillPairReg hl
;	spillPairReg hl
	call	_snsmat
;romloader.c:262: if ((row & (R8_DOWN | R8_UP | R8_SPACE)) ==
	ld	a, l
	and	a, #0x61
	ld	c, a
	ld	b, #0x00
	ld	a, c
	sub	a, #0x61
;romloader.c:264: return KEY_NONE;
	or	a,b
	jr	Z, 00121$
;romloader.c:267: uint8_t key = KEY_NONE;
	ld	-1 (ix), #0x00
;romloader.c:268: if ((row & R8_SPACE) == 0U) key = KEY_SELECT;
	bit	0, l
	jr	NZ, 00116$
	ld	-1 (ix), #0x03
	jr	00118$
00116$:
;romloader.c:269: else if ((row & R8_DOWN) == 0U && (row & R8_UP) == 0U) key = KEY_DOWN;
	ld	a, l
	and	a, #0x40
	ld	e, a
	ld	d, #0x00
	ld	a, l
	and	a, #0x20
	ld	c, a
	ld	b, #0x00
	ld	a, d
	or	a, e
	jr	NZ, 00112$
	ld	a, b
	or	a, c
	jr	NZ, 00112$
	ld	-1 (ix), #0x01
	jr	00118$
00112$:
;romloader.c:270: else if ((row & R8_DOWN) == 0U) key = KEY_DOWN;
	ld	a, d
	or	a, e
	jr	NZ, 00109$
	ld	-1 (ix), #0x01
	jr	00118$
00109$:
;romloader.c:271: else if ((row & R8_UP)   == 0U) key = KEY_UP;
	ld	a, b
	or	a, c
	jr	NZ, 00118$
	ld	-1 (ix), #0x02
;romloader.c:274: while ((snsmat(8) & (R8_DOWN | R8_UP | R8_SPACE)) !=
00118$:
	ld	l, #0x08
;	spillPairReg hl
;	spillPairReg hl
	call	_snsmat
	ld	a, l
	and	a, #0x61
	ld	c, a
	ld	b, #0x00
	ld	a, c
	sub	a, #0x61
	or	a, b
	jr	NZ, 00118$
;romloader.c:276: delay_ms(10);
	ld	hl, #0x000a
	call	_delay_ms
;romloader.c:277: return key;
	ld	a, -1 (ix)
00121$:
;romloader.c:278: }
	inc	sp
	pop	ix
	ret
;romloader.c:281: static void wait_any_key(void)
;	---------------------------------
; Function wait_any_key
; ---------------------------------
_wait_any_key:
;romloader.c:284: for (uint8_t row = 0; row < 10; row++) {
00115$:
	ld	c, #0x00
00108$:
	ld	a, c
	sub	a, #0x0a
	jr	NC, 00115$
;romloader.c:285: if (snsmat(row) != 0xFFU) return;
	push	bc
	ld	l, c
;	spillPairReg hl
;	spillPairReg hl
	call	_snsmat
	pop	bc
	inc	l
	ret	NZ
;romloader.c:284: for (uint8_t row = 0; row < 10; row++) {
	inc	c
;romloader.c:288: }
	jr	00108$
;romloader.c:298: static void fetch_file_list(void)
;	---------------------------------
; Function fetch_file_list
; ---------------------------------
_fetch_file_list:
	call	___sdcc_enter_ix
	push	af
	dec	sp
;romloader.c:300: mbox_cmd(CMD_DIR_OPEN, 0, 0);
	xor	a, a
	push	af
	inc	sp
	ld	de, #0x0000
	xor	a, a
	call	_mbox_cmd
;romloader.c:301: uint8_t cnt_lo = mbox_pop();
	call	_mbox_pop
	ld	c, a
;romloader.c:302: uint8_t cnt_hi = mbox_pop();
	push	bc
	call	_mbox_pop
	pop	bc
;romloader.c:303: s_count = (uint16_t)cnt_lo | ((uint16_t)cnt_hi << 8);
	ld	b, #0x00
	ld	e, a
	xor	a, a
	or	a, c
	ld	(_s_count+0), a
	ld	a, e
	or	a, b
	ld	(_s_count+1), a
;romloader.c:304: if (s_count > MAX_FILES)
	ld	hl, (_s_count)
	ld	a, #0x80
	cp	a, l
	ld	a, #0x00
	sbc	a, h
	jr	NC, 00117$
;romloader.c:305: s_count = MAX_FILES;
	ld	hl, #0x0080
	ld	(_s_count), hl
;romloader.c:307: for (uint16_t i = 0; i < s_count; i++) {
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
;romloader.c:308: mbox_cmd(CMD_DIR_READ, 0, 0);
	push	bc
	xor	a, a
	push	af
	inc	sp
	ld	de, #0x0000
	ld	a, #0x01
	call	_mbox_cmd
	pop	bc
;romloader.c:309: for (uint8_t j = 0; j < LOADER_NAME_LEN; j++)
	ld	l, c
	ld	h, b
	add	hl, hl
	add	hl, bc
	add	hl, hl
	add	hl, hl
	add	hl, bc
	ex	de, hl
	ld	hl, #_s_files
	add	hl, de
	ex	(sp), hl
	ld	-1 (ix), #0x00
00106$:
	ld	a, -1 (ix)
	sub	a, #0x0c
	jr	NC, 00103$
;romloader.c:310: s_files[i][j] = (char)mbox_pop();
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
;romloader.c:309: for (uint8_t j = 0; j < LOADER_NAME_LEN; j++)
	inc	-1 (ix)
	jr	00106$
00103$:
;romloader.c:311: s_files[i][LOADER_NAME_LEN] = '\0';
	ld	hl, #_s_files
	add	hl, de
	ld	de, #0x000c
	add	hl, de
	ld	(hl), #0x00
;romloader.c:307: for (uint16_t i = 0; i < s_count; i++) {
	inc	bc
	jr	00109$
00104$:
;romloader.c:314: mbox_cmd(CMD_DIR_CLOSE, 0, 0);
	xor	a, a
	push	af
	inc	sp
	ld	de, #0x0000
	ld	a, #0x02
	call	_mbox_cmd
;romloader.c:315: }
	ld	sp, ix
	pop	ix
	ret
;romloader.c:320: static uint16_t choose(const char *title, const char *const *names,
;	---------------------------------
; Function choose
; ---------------------------------
_choose:
	push	ix
	ld	ix,#0
	add	ix,sp
	ld	iy, #-10
	add	iy, sp
	ld	sp, iy
	ld	-6 (ix), l
	ld	-5 (ix), h
;romloader.c:323: if (count == 0U) return 0U;
	ld	a, 7 (ix)
	or	a, 6 (ix)
	jr	NZ, 00102$
	ld	de, #0x0000
	jp	00134$
00102$:
;romloader.c:324: uint16_t sel = start < count ? start : 0;
	ld	a, 8 (ix)
	sub	a, 6 (ix)
	ld	a, 9 (ix)
	sbc	a, 7 (ix)
	jr	NC, 00136$
	ld	a, 8 (ix)
	ld	c, 9 (ix)
	jr	00137$
00136$:
	xor	a, a
	ld	c, a
00137$:
	ld	-4 (ix), a
	ld	-3 (ix), c
00132$:
;romloader.c:327: cls();
	call	_cls
;romloader.c:328: print(title);
	ld	l, -6 (ix)
;	spillPairReg hl
;	spillPairReg hl
	ld	h, -5 (ix)
;	spillPairReg hl
;	spillPairReg hl
	call	_print
;romloader.c:329: print("\r\n\r\n");
	ld	hl, #___str_0
	call	_print
;romloader.c:339: if (count <= 10U) {
	ld	a, 6 (ix)
	ld	-8 (ix), a
	ld	a, 7 (ix)
	ld	-7 (ix), a
;romloader.c:341: bot = count;
	ld	a, 6 (ix)
	ld	-10 (ix), a
	ld	a, 7 (ix)
	ld	-9 (ix), a
;romloader.c:339: if (count <= 10U) {
	ld	a, #0x0a
	cp	a, -8 (ix)
	ld	a, #0x00
	sbc	a, -7 (ix)
	jr	C, 00110$
;romloader.c:340: top = 0U;
	ld	bc, #0x0000
;romloader.c:341: bot = count;
	pop	de
	push	de
	jr	00147$
00110$:
;romloader.c:342: } else if (sel < 5U) {
	ld	a, -4 (ix)
	ld	-2 (ix), a
	ld	a, -3 (ix)
	ld	-1 (ix), a
	ld	a, -2 (ix)
	sub	a, #0x05
	ld	a, -1 (ix)
	sbc	a, #0x00
	jr	NC, 00107$
;romloader.c:343: top = 0U;
	ld	bc, #0x0000
;romloader.c:344: bot = 10U;
	ld	de, #0x000a
	jr	00147$
00107$:
;romloader.c:345: } else if (sel >= (uint16_t)(count - 5U)) {
	ld	a, -8 (ix)
	add	a, #0xfb
	ld	c, a
	ld	a, -7 (ix)
	adc	a, #0xff
	ld	b, a
	ld	a, -4 (ix)
	sub	a, c
	ld	a, -3 (ix)
	sbc	a, b
	jr	C, 00104$
;romloader.c:346: top = (uint16_t)(count - 10U);
	ld	a, -8 (ix)
	add	a, #0xf6
	ld	c, a
	ld	a, -7 (ix)
	adc	a, #0xff
	ld	b, a
;romloader.c:347: bot = count;
	pop	de
	push	de
	jr	00147$
00104$:
;romloader.c:349: top = (uint16_t)(sel - 5U);
	ld	a, -2 (ix)
	add	a, #0xfb
	ld	c, a
	ld	a, -1 (ix)
	adc	a, #0xff
	ld	b, a
;romloader.c:350: bot = (uint16_t)(sel + 5U);
	ld	l, -2 (ix)
	ld	h, -1 (ix)
	ld	de, #0x0005
	add	hl, de
	ex	de, hl
;romloader.c:352: for (uint16_t i = top; i < bot; i++) {
00147$:
	ld	-2 (ix), c
	ld	-1 (ix), b
00130$:
	ld	a, -2 (ix)
	sub	a, e
	ld	a, -1 (ix)
	sbc	a, d
	jr	NC, 00112$
;romloader.c:353: print(i == sel ? "> " : "  ");
	ld	a, -4 (ix)
	sub	a, -2 (ix)
	jr	NZ, 00138$
	ld	a, -3 (ix)
	sub	a, -1 (ix)
	jr	NZ, 00138$
	ld	hl, #___str_1
	jr	00139$
00138$:
	ld	hl, #___str_2
00139$:
	push	de
	call	_print
	pop	de
;romloader.c:354: print(names[i]);
	ld	c, -2 (ix)
	ld	b, -1 (ix)
	sla	c
	rl	b
	ld	l, 4 (ix)
	ld	h, 5 (ix)
	add	hl, bc
	ld	c, (hl)
	inc	hl
	ld	h, (hl)
;	spillPairReg hl
	push	de
	ld	l, c
;	spillPairReg hl
;	spillPairReg hl
	call	_print
	ld	hl, #___str_3
	call	_print
	pop	de
;romloader.c:352: for (uint16_t i = top; i < bot; i++) {
	inc	-2 (ix)
	jr	NZ, 00130$
	inc	-1 (ix)
	jr	00130$
00112$:
;romloader.c:357: print("\r\nUP/DOWN=navigate  SPACE=select\r\n");
	ld	hl, #___str_4
	call	_print
;romloader.c:360: do {
00113$:
;romloader.c:361: k = wait_arrow();
	call	_wait_arrow
	ld	-1 (ix), a
;romloader.c:362: } while (k == KEY_NONE);
	or	a, a
	jr	Z, 00113$
;romloader.c:364: if (k == KEY_SELECT) {
	cp	a, #0x03
	jr	NZ, 00117$
;romloader.c:365: return sel;
	ld	e, -4 (ix)
	ld	d, -3 (ix)
	jr	00134$
00117$:
;romloader.c:368: if (k == KEY_DOWN) {
	cp	a, #0x01
	jr	NZ, 00126$
;romloader.c:369: sel++;
	inc	-4 (ix)
	jr	NZ, 00229$
	inc	-3 (ix)
00229$:
;romloader.c:370: if (sel >= count) sel = 0U;
	ld	a, -4 (ix)
	sub	a, 6 (ix)
	ld	a, -3 (ix)
	sbc	a, 7 (ix)
	jp	C, 00132$
	xor	a, a
	ld	-4 (ix), a
	ld	-3 (ix), a
	jp	00132$
00126$:
;romloader.c:371: } else if (k == KEY_UP) {
	sub	a, #0x02
	jp	NZ,00132$
;romloader.c:372: if (sel == 0U) sel = (uint16_t)(count - 1U);
	ld	a, -3 (ix)
	or	a, -4 (ix)
	jr	NZ, 00121$
	ld	c, -8 (ix)
	ld	b, -7 (ix)
	dec	bc
	ld	-4 (ix), c
	ld	-3 (ix), b
	jp	00132$
00121$:
;romloader.c:373: else sel--;
	ld	l, -4 (ix)
	ld	h, -3 (ix)
	dec	hl
	ld	-4 (ix), l
	ld	-3 (ix), h
	jp	00132$
00134$:
;romloader.c:376: }
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
	.ascii "> "
	.db 0x00
___str_2:
	.ascii "  "
	.db 0x00
___str_3:
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_4:
	.db 0x0d
	.db 0x0a
	.ascii "UP/DOWN=navigate  SPACE=select"
	.db 0x0d
	.db 0x0a
	.db 0x00
;romloader.c:380: int main(void)
;	---------------------------------
; Function main
; ---------------------------------
_main::
	call	___sdcc_enter_ix
	ld	hl, #-24
	add	hl, sp
	ld	sp, hl
;romloader.c:382: chgmod(0);		/* SCREEN 0, 40 columns */
	ld	l, #0x00
;	spillPairReg hl
;	spillPairReg hl
	call	_chgmod
;romloader.c:384: cls();
	call	_cls
;romloader.c:385: print("RISKYMSX2 ROM LOADER\r\n\r\n");
	ld	hl, #___str_17
	call	_print
;romloader.c:386: print("Reading USB directory...\r\n");
	ld	hl, #___str_18
	call	_print
;romloader.c:388: fetch_file_list();
	call	_fetch_file_list
;romloader.c:390: if (s_count == 0U) {
	ld	a, (_s_count+1)
	ld	hl, #_s_count
	or	a, (hl)
	jr	NZ, 00131$
;romloader.c:391: cls();
	call	_cls
;romloader.c:392: print("NO .ROM FILES ON USB STICK\r\n\r\n");
	ld	hl, #___str_19
	call	_print
;romloader.c:393: print("Plug a stick with .ROM files,\r\n");
	ld	hl, #___str_20
	call	_print
;romloader.c:394: print("power-cycle the MSX.\r\n");
	ld	hl, #___str_21
	call	_print
00111$:
;romloader.c:396: wait_any_key();
	call	_wait_any_key
	jr	00111$
;romloader.c:401: for (uint16_t i = 0; i < s_count; i++)
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
;romloader.c:402: file_ptrs[i] = s_files[i];
	ld	l, c
;	spillPairReg hl
;	spillPairReg hl
	ld	h, b
;	spillPairReg hl
;	spillPairReg hl
	add	hl, hl
	ld	de, #_main_file_ptrs_65537_68
	add	hl, de
	ld	-2 (ix), l
	ld	-1 (ix), h
	ld	l, c
	ld	h, b
	add	hl, hl
	add	hl, bc
	add	hl, hl
	add	hl, hl
	add	hl, bc
	ld	de, #_s_files
	add	hl, de
	ex	de, hl
	ld	l, -2 (ix)
	ld	h, -1 (ix)
	ld	(hl), e
	inc	hl
	ld	(hl), d
;romloader.c:401: for (uint16_t i = 0; i < s_count; i++)
	inc	bc
	jr	00114$
00104$:
;romloader.c:403: uint16_t sel = choose("SELECT ROM FILE:", file_ptrs, s_count, 0);
	ld	hl, #0x0000
	push	hl
	ld	hl, (_s_count)
	push	hl
	ld	hl, #_main_file_ptrs_65537_68
	push	hl
	ld	hl, #___str_5
	call	_choose
;romloader.c:406: cls();
	push	de
	call	_cls
	ld	hl, #___str_22
	call	_print
	pop	de
;romloader.c:408: print(s_files[sel]);
	ld	l, e
	ld	h, d
	add	hl, hl
	add	hl, de
	add	hl, hl
	add	hl, hl
	add	hl, de
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
;romloader.c:412: for (uint8_t i = 0; i < LOADER_NAME_LEN; i++)
	ld	hl, #0
	add	hl, sp
	ex	de, hl
	ld	-1 (ix), #0x00
00117$:
	ld	a, -1 (ix)
	sub	a, #0x0c
	jr	NC, 00105$
;romloader.c:413: args[i] = (uint8_t)s_files[sel][i];
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
;romloader.c:412: for (uint8_t i = 0; i < LOADER_NAME_LEN; i++)
	inc	-1 (ix)
	jr	00117$
00105$:
;romloader.c:414: mbox_cmd(CMD_LOAD_ROM, args, LOADER_NAME_LEN);
	ld	a, #0x0c
	push	af
	inc	sp
	ld	a, #0x03
	call	_mbox_cmd
;romloader.c:417: for (uint8_t i = 0; i < 4; i++)
	ld	c, #0x00
00120$:
	ld	a, c
	sub	a, #0x04
	jr	NC, 00106$
;romloader.c:418: len[i] = mbox_pop();
	ld	e, c
	ld	d, #0x00
	ld	hl, #12
	add	hl, sp
	add	hl, de
	push	hl
	push	bc
	call	_mbox_pop
	pop	bc
	pop	hl
	ld	(hl), a
;romloader.c:417: for (uint8_t i = 0; i < 4; i++)
	inc	c
	jr	00120$
00106$:
;romloader.c:419: uint32_t total = ((uint32_t)len[0] << 24)
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
;romloader.c:423: print("LOADED ");
	ld	hl, #___str_24
	call	_print
;romloader.c:424: print_dec((uint16_t)((total + 1023U) / 1024U));
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
;romloader.c:425: print(" KiB\r\n\r\n");
	ld	hl, #___str_25
	call	_print
;romloader.c:442: uint8_t chosen = (uint8_t)choose("SELECT MAPPER:",
	ld	hl, #0x0001
	push	hl
	ld	l, #0x0a
	push	hl
	ld	hl, #_main_mapper_names_65539_76
	push	hl
	ld	hl, #___str_16
	call	_choose
	ld	-1 (ix), e
;romloader.c:446: cls();
	call	_cls
;romloader.c:447: print("MAPPER: ");
	ld	hl, #___str_26
	call	_print
;romloader.c:448: print(mapper_names[chosen]);
	ld	l, -1 (ix)
;	spillPairReg hl
;	spillPairReg hl
	ld	h, #0x00
;	spillPairReg hl
;	spillPairReg hl
	add	hl, hl
	ld	de, #_main_mapper_names_65539_76
	add	hl, de
	ld	e, (hl)
	inc	hl
	ld	d, (hl)
	ex	de, hl
	call	_print
;romloader.c:449: print("\r\n");
	ld	hl, #___str_27
	call	_print
;romloader.c:452: args[0] = (uint8_t)(chosen + 1U);	/* Cart_Mapper index */
	ld	hl, #15
	add	hl, sp
	ex	de, hl
	ld	a, -1 (ix)
	inc	a
	ld	(de), a
;romloader.c:453: mbox_cmd(CMD_SET_MAPPER, args, 1);
	ld	a, #0x01
	push	af
	inc	sp
	ld	a, #0x04
	call	_mbox_cmd
;romloader.c:454: uint8_t rc = mbox_pop();
	call	_mbox_pop
;romloader.c:455: if (rc != 0U) {
	or	a, a
	jr	Z, 00109$
;romloader.c:456: print("REFUSED! (PSRAM not ready?)\r\n");
	ld	hl, #___str_28
	call	_print
00122$:
;romloader.c:458: wait_any_key();
	call	_wait_any_key
	jr	00122$
00109$:
;romloader.c:463: print("RESET...");
	ld	hl, #___str_29
	call	_print
;romloader.c:464: mbox_push(CMD_RESET);
	ld	a, #0x05
	call	_mbox_push
00125$:
;romloader.c:469: }
	jr	00125$
___str_5:
	.ascii "SELECT ROM FILE:"
	.db 0x00
_main_mapper_names_65539_76:
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
