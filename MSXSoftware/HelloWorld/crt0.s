;-------------------------------------------------------------------------------
; crt0.s - C runtime startup for an MSX ROM cartridge
;
; Placed at 0x4000 (page 1). The "AB" header tells the MSX BIOS that a ROM
; cartridge is present; the INIT vector makes it auto-start at power-on
; (called via CALSLT on the main-ROM interrupt, with interrupts disabled
; and the stack near the top of RAM).
;-------------------------------------------------------------------------------

	.module crt0
	.globl	_main

	.area	_HEADER (ABS)

	.org	0x4000

	;; --- MSX ROM header (16 bytes) ---
	.db	0x41, 0x42		; 'A' 'B' cartridge ID
	.dw	init			; INIT: auto-start entry point
	.dw	0x0000			; STATEMENT: no new BASIC statements
	.dw	0x0000			; DEVICE: no new devices
	.dw	0x0000			; TEXT: no BASIC program in ROM
	.dw	0x0000, 0x0000, 0x0000	; reserved

	;; --- Entry point ---
init:
	;; Interrupts stay DISABLED (INIT is entered via CALSLT with
	;; interrupts off). The menu reads keys via SNSMAT (PSG matrix
	;; scan), which works without the BIOS keyboard ISR - and keeping
	;; DI avoids any interaction between the BIOS interrupt handler
	;; and the emulated cart during measurement.
	call	_main
	;; _main does not return if the program ends in an endless loop,
	;; but if it does, clean up: disable screen interrupts and RET to BIOS
	di
	halt
	ret
