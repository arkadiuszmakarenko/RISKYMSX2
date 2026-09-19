;-------------------------------------------------------------------------------
; crt0.s - envelope-test cartridge startup.
;
; Layout (mirrors the working hello.bin / fixed selector ROM):
;   0x4000  "AB" header, INIT -> 0x4010
;   0x4010  DI / CALL _main / DI / HALT / RET
;   0x4020  C code starts here
;
; The entry stub is intentionally NOT at 0x4020: SDCC's --code-loc 0x4020
; starts _CODE at 0x4020, so any stub there gets overwritten by the first
; C function when makebin packs overlapping IHX records.  hello.bin's
; stub at 0x4010 survives because it lives between the header and _CODE.
;-------------------------------------------------------------------------------

	.module crt0
	.globl	_main

	.area	_HEADER (ABS)

	.org	0x4000

	;; --- MSX ROM header (16 bytes) ---
	.db	0x41, 0x42		; 'A' 'B' cartridge ID
	.dw	init			; INIT -> 0x4010 (resolved at link time)
	.dw	0x0000			; STATEMENT
	.dw	0x0000			; DEVICE
	.dw	0x0000			; TEXT
	.dw	0x0000, 0x0000, 0x0000	; reserved

	;; --- direct entry stub at 0x4010 (between header and _CODE) ---
	.org	0x4010
init:
	di
	call	_main
	di
	halt
	ret
