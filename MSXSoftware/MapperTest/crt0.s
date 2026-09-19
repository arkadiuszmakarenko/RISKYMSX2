
	.module crt0
	.globl	_main

	.area	_HEADER (ABS)

	.org	0x4000

	;; --- MSX ROM header (16 bytes) ---
	.db	0x41, 0x42
	.dw	init
	.dw	0x0000
	.dw	0x0000
	.dw	0x0000
	.dw	0x0000, 0x0000, 0x0000

	;; --- direct entry stub at 0x4010 ---
	.org	0x4010
init:
	di
	call	_main
	di
	halt
	ret
