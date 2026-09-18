;-------------------------------------------------------------------------------
; crt0.s - C runtime startup for the RISKYMSX2 ROM LOADER cartridge.
;
; ROM layout (see Makefile for how the image is assembled):
;   0x4000  "AB" MSX ROM header, INIT -> init
;   0x4010  payload_len (word, patched by the Makefile after linking)
;   0x4020  copy stub: LDIRs the payload to 0xC000, jumps to 0xC000.
;   0x4040  PAYLOAD: the C loader, linked to run at 0xC000 in MSX RAM.
;
; The payload is position-DEPENDENT (sdcc emits absolute jp/call), so
; the stub MUST copy it to its link address before jumping.
;
; Note: the SDCC z80 port merges all globals (zero-initialised ones
; included) into the _DATA area, which the linker places directly
; after _CODE.  makebin zero-fills the uninitialised tail, so copying
; the whole span 0xC000..end covers data+bss - no separate BSS step.
;
; Entry (INIT, via CALSLT, IF=0):
;   1. DI (keep interrupts off; the loader polls keys via SNSMAT so
;      it never needs the BIOS keyboard ISR).
;   2. LDIR payload -> 0xC000.
;   3. JP 0xC000 (_main is the first symbol of _CODE).
;-------------------------------------------------------------------------------

	.module crt0

	.area	_HEADER (ABS)

	.org	0x4000

	;; --- MSX ROM header (16 bytes) ---
	.db	0x41, 0x42		; 'A' 'B' cartridge ID
	.dw	init			; INIT: auto-start entry point
	.dw	0x0000			; STATEMENT
	.dw	0x0000			; DEVICE
	.dw	0x0000			; TEXT
	.dw	0x0000, 0x0000, 0x0000	; reserved

	;; --- patched-by-Makefile constant (file offset 0x10) ---
	.org	0x4010
payload_len:	.dw	0		; bytes to copy (payload span)
	.dw	0, 0, 0		; spare (aligns stub to 0x4020)

	;; --- copy stub (0x4020) ---
	.org	0x4020
init:
	di

	ld	hl, #0x4040		; source: payload in ROM
	ld	de, #0xC000		; destination: MSX RAM
	ld	bc, (payload_len)	; length (patched by Makefile)
	ldir

	jp	0xC000			; -> RAM-resident _main

	;; _main never returns; if it somehow did, park.
	di
	halt