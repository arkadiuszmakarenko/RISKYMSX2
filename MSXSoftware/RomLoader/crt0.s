;-------------------------------------------------------------------------------
; crt0.s - C runtime startup for the RISKYMSX2 ROM LOADER cartridge.
;
; The loader now starts like a normal MSX ROM cartridge, like
; MSXSoftware/HelloWorld/hello.bin:
;   0x4000  "AB" MSX ROM header, INIT -> init
;   0x4020  direct entry stub: call _main
;
; The ROM runs directly from flash and uses the cart mailbox window
; (0x7FF0..0x7FFF) as the shared envelope for USB directory, file and
; mapper data. No copy-to-RAM bootstrap is needed.
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

	;; --- direct entry stub (0x4010) ---
	;; The 16-byte MSX ROM header lives at 0x4000..0x400F. The C code
	;; area (_CODE) starts at 0x4020, so the entry stub must NOT be
	;; at 0x4020 or it will be overwritten by _set_screen (the first
	;; C function) when makebin packs the IHX. Place it at 0x4010
	;; (right after the header, before _CODE at 0x4020) and update
	;; the header's INIT vector to match. SDCC resolves the .dw
	;; reference at link time, so we just need a label here.
	.org	0x4010
init:
	di
	call	_main
	di
	halt
	ret