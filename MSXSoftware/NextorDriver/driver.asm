;*****************************************************************************
; driver.asm - Nextor 3.0 disk driver for the RISKYMSX2 cartridge.
;
; The Nextor kernel executes this driver from the cartridge's driver bank
; (paged in at page 1 = 0x4000-0x7FFF) through the standard Nextor 3
; query/READ_WRITE contract (see docs/NEXTOR_PLAN.md section 4 and the
; official Nextor 3.0 Driver Development Guide).
;
; All hardware access goes through the cart-bus mailbox window at
; 0x7FF0..0x7FF7, decoded BY ADDRESS by the firmware's
; Cart_EXTI0_NEXTOR_Handler in every paged-in bank:
;
;   0x7FF0  read  STATUS   bit7 READY (fw alive) / bit6 DONE /
;                          bit5 ERR / bit4 RX_AVAIL
;          write CMD      starts a command (firmware clears DONE)
;   0x7FF1  DATA           read: pop result byte / write: push arg byte
;   0x7FF2  RX_COUNT_LO    result bytes available to drain (informational)
;   0x7FF3  RX_COUNT_HI
;   0x7FF4  ERR            0 none / 1 no media / 2 USB+SCSI error / 3 tmo
;   0x7FF5  FW version
;
; Commands (write CMD, then args as DATA writes; results are DATA reads
; once DONE appears in STATUS):
;
;   0x00 HANDSHAKE   -> 5 bytes: "RNX2" + fw version byte
;   0x01 CAPACITY    <- dev -> 8 bytes: block count LE(4) + block size LE(4)
;   0x02 STATUS      <- dev -> 1 byte: 0 no image / 1 ready / 2 image changed
;                                 (CONSUMES the firmware's change latch)
;   0x03 READ        <- dev + LBA LE(4) -> 512 bytes sector data
;   0x04 WRITE       <- dev + LBA LE(4) + 512 bytes -> nothing back
;   0x05 ABORT       -> nothing
;   0x06 STAPEEK     <- dev -> 1 byte: like STATUS but does NOT consume the
;                                 change latch (device availability query)
;
; THE DEVICE BYTE. Every command except HANDSHAKE and ABORT takes the
; device number as its FIRST argument. That is what lets two devices share
; one command set and one driver entry point: the medium is chosen on the
; wire, per request, instead of being fixed by which routine the kernel
; happened to call. MB_STDEV pushes it for the one-byte commands and the
; two sector paths inline it; the only place the device number is ever
; used as a constant is picking the ROM byte to send (MB_DEV1 / MB_DEV2)
; and the per-device answer in DQP_TAIL.
;
; HANDSHAKE and ABORT carry no device because neither touches a medium:
; the first is about the mailbox itself, the second is a mailbox-level
; operation. This also means the firmware can answer a handshake before it
; knows whether its device count agrees with the driver's - which is the
; point of the version byte below.
;
; Taking results off the DATA port:
;   MB_RESULT_IS  compare N expected bytes as they stream past (no RAM)
;   MB_RD4        load 4 result bytes into L/H/E/D
;   MB_GETRES     store N result bytes at (HL) - (HL) MUST be RAM
;   MB_STDEV      1 result byte, straight back in A (command in A, dev in B)
;
; There is a trap here worth stating once, because it cost a long debug
; session: this is a ROM driver, so it has NO writable variables. A `ds`
; buffer in this file is part of the cartridge image, the Z80 discards
; `ld (hl),a` into it, and the buffer reads back as the zeros the
; assembler put in the file. Anything that stores a mailbox result into
; the driver's own data will silently see zeros. Results therefore have
; to be consumed into registers (MB_RESULT_IS / MB_RD4) or into a buffer
; the kernel supplies in RAM. This is also why the device byte is sent
; from two read-only ROM bytes rather than from a computed one: MB_SEND
; copies its arguments out of memory, and the only memory this driver can
; name as an argument source is ROM.
;
; Device model (docs/NEXTOR_PLAN.md D4): TWO devices, each one file on
; the USB stick, served sector by sector.
;
;   device 1   NEXTOR.DSK - a fixed, READ-ONLY 720K floppy image:
;              1440 x 512-byte sectors, no partition table, flagged to
;              the kernel as a floppy disk drive and as removable. The
;              floppy flag is what tells the kernel to map a drive onto
;              sector 0 rather than scan for partitions (see DQP_TAIL).
;
;   device 2   NEXTOR.IMG - a READ-WRITE disk image whose length IS its
;              capacity, carrying an MBR and a FAT16 volume. Flagged
;              removable and NOT floppy, so the kernel runs its ordinary
;              partition scan and finds the MBR. The image is created and
;              deleted from the firmware terminal's Nextor menu, not by
;              the driver.
;
; Z80 rules respected:
;   - documented opcodes only (pairs with a .NO_UNDOC. kernel variant);
;   - no use of C000h-C400h as scratch;
;   - no RAM work area requested: mailbox results are consumed into
;     registers (see the note above), so the driver needs none; the
;     media-change latch lives in the firmware (CMD_STAPEEK reads it
;     without consuming), so query 3 returns B=0 flags and HL=0 area.
;
; Build: see Makefile (N80 -> mknexrom).
;*****************************************************************************

	INCLUDE asm/macros/undoc.inc		;documented-only expansion helpers
	INCLUDE asm/constants/dos_errors.inc
	INCLUDE asm/constants/driver_result_codes.inc
	INCLUDE asm/constants/rom_bank_header.inc

	module DRIVER_QUERY
	INCLUDE asm/constants/driver_driver_queries.inc
	endmod

	module DEVICE_QUERY
	INCLUDE asm/constants/driver_device_queries.inc
	endmod

;-----------------------------------------------------------------------------
; Mailbox window (cart bus; decoded by address in every bank)
;-----------------------------------------------------------------------------

MBOX_CMD	equ	7FF0h	;write = command byte
MBOX_STAT	equ	7FF0h	;read = STATUS bits
MBOX_DATA	equ	7FF1h	;data pop/push port
MBOX_RXCL	equ	7FF2h	;result bytes available, low
MBOX_RXCH	equ	7FF3h	;result bytes available, high
MBOX_ERR	equ	7FF4h	;last error code
MBOX_VER	equ	7FF5h	;firmware version byte

;STATUS bits
MBST_READY	equ	80h
MBST_DONE	equ	40h
MBST_ERR	equ	20h
MBST_RXAVL	equ	10h

;commands
MB_HANDSHAKE	equ	00h
MB_CAPACITY	equ	01h
MB_STATUS	equ	02h
MB_READ	equ	03h
MB_WRITE	equ	04h
MB_ABORT	equ	05h
MB_STAPEEK	equ	06h

;device numbers (the wire form of nextor.h's NEXTOR_DEV_*)
MB_DEV_FLANKY	equ	1	;NEXTOR.DSK, read-only 720K floppy
MB_DEV_DISK	equ	2	;NEXTOR.IMG, read-write disk
MB_DEVICE_COUNT	equ	2	;driver query 5's answer

;mailbox ERR port values
MBERR_NONE	equ	00h
MBERR_NOMEDIA	equ	01h
MBERR_SCSI	equ	02h
MBERR_TMO	equ	03h

;firmware handshake magic
MB_MAGIC0	equ	"R"
MB_MAGIC1	equ	"N"
MB_MAGIC2	equ	"X"
MB_MAGIC3	equ	"2"

;protocol revision this driver is written against. The firmware answers
;the handshake with ITS revision byte and MB_EXPECT compares it, so a
;driver built against one revision talking to a firmware built against
;another is rejected at boot - loudly, with the kernel skipping this
;driver - instead of answering every command for the wrong device.
;Must equal NEXTOR_VERSION in firmware/User/nextor.h.
MB_PROTOCOL_V2	equ	02h

;-----------------------------------------------------------------------------
; ROM driver boilerplate
;-----------------------------------------------------------------------------

; mknexrom expects the driver file to start with 256 dummy bytes (it
; overwrites them with the kernel's common bank header code), so the
; actual driver code starts at 4100h.

	org	4000h
	ds	4100h-$,0

DRIVER_START:

	;Driver signature

	db	"NEXTORv3_DRIVER",0

	;Jump table

	jp	TIMER_INT
	jp	OEMSTAT
	jp	BASDEV
	jp	EXTBIO
	jp	DRIVER_QUERY
	jp	DEVICE_QUERY
	jp	CUSTOM_DRIVER_QUERY
	jp	CUSTOM_DEVICE_QUERY
	jp	READ_WRITE
	jp	RESERVED_0
	jp	RESERVED_1
	jp	RESERVED_2
	jp	DIRECT_0
	jp	DIRECT_1
	jp	DIRECT_2
	jp	DIRECT_3
	jp	DIRECT_4

	;--- Timer interrupt routine: not hooked (query 3 returns B=0).

TIMER_INT:
	ret
	ret
	ret

	;--- BASIC expanded statement ("CALL") handler: none.

OEMSTAT:
	scf
	ret
	ret

	;--- BASIC expanded devices: none.

BASDEV:
	scf
	ret
	ret

	;--- Extended BIOS hook: not hooked (query 3 returns B=0).

EXTBIO:
	ret
	ret
	ret

	;* Jump table entries reserved for future use.

RESERVED_0:
RESERVED_1:
RESERVED_2:
	ret

	;* Direct call entry points: none.

DIRECT_0:
DIRECT_1:
DIRECT_2:
DIRECT_3:
DIRECT_4:
	ret

;-----------------------------------------------------------------------------
; Mailbox primitives
;-----------------------------------------------------------------------------

;--- MB_SEND: send a command byte plus B argument bytes copied from (HL).
;    B = 0 is legal (command with no arguments).
;    The DI/EI wrap is belt and braces: the command+args burst must not
;    be split by anything else touching the window (nothing in the BIOS
;    interrupt path does, but the cost is a few microseconds).
;    In:  A = command, B = arg count, HL = arg source
;    Out: HL advanced past the args - so HL is an OUTPUT even when B = 0.
;         Trashes AF, B.
;
;    That last point is the trap. `ld hl,MB_NOARGS / ld b,0 / call MB_SEND`
;    reads like "send a command with no arguments" and also silently
;    overwrites whatever the caller had in HL. DO_DEVQ_GET_PARAMS kept its
;    buffer address there, filled the 12-byte parameter block at MB_NOARGS
;    - inside this image - and handed the kernel an untouched buffer.
;    Any caller holding something in HL across this must save it.

MB_SEND:
	di
	ld	(MBOX_CMD),a
	ld	a,b
	or	a
	jr	z,MB_SEND_Z
MB_SEND_L:
	ld	a,(hl)
	inc	hl
	ld	(MBOX_DATA),a
	djnz	MB_SEND_L
MB_SEND_E:
	ei
	ret
MB_SEND_Z:
	ei
	ret

;--- MB_POLL: wait for the firmware to set the DONE bit.
;    Bounded: 3 passes of a 16-bit poll counter (~0.8 s total at 3.58 MHz).
;    Out: on DONE: A = STATUS bits, Cy=0.  On timeout: Cy=1.
;    Trashes AF, BC. Uses B' as the pass counter (re-armed every call);
;    the persistent shadow state (DE' = LBA pointer) is untouched.

MB_POLL:
	exx
	ld	b,3
	exx
MB_POLL_P:
	ld	bc,0FFFFh
MB_POLL_L:
	ld	a,(MBOX_STAT)
	and	MBST_DONE
	jr	nz,MB_POLL_OK
	dec	bc
	ld	a,b
	or	c
	jr	nz,MB_POLL_L
	exx
	djnz	MB_POLL_GO	;inner pass done, next pass
	exx
	scf			;passes exhausted: timeout
	ret
MB_POLL_GO:
	exx
	jr	MB_POLL_P
MB_POLL_OK:
	or	a		;clear Cy (A = STATUS bits, DONE set)
	ret

;--- MB_RESULT_IS: consume A result bytes from the DATA port, comparing each
;    one against the matching byte at (HL) as it arrives.
;
;    In:  A = result byte count, HL = pointer to the A expected bytes
;    Out: Cy=0 = every byte matched.
;         Cy=1 = mismatch: B = its 0-based index, C = the byte that was
;               read, D = the byte that was expected.
;    Trashes: AF, BC, DE, HL.  E is left as the original count.
;
;    Deliberately storage-free, and that is the whole point. The obvious
;    shape - pop the bytes into a buffer, then compare - cannot work in
;    this driver: a ROM driver has no writable variables, so the buffer
;    would be a `ds` inside the cartridge image, and the Z80 discards
;    `ld (hl),a` into ROM. The buffer would keep the zeros baked into the
;    ROM file and the comparison would run against those (see the note
;    on MB_GETRES - that is exactly the bug that made the handshake
;    report "bad magic" forever). Comparing as the bytes stream past
;    needs no storage at all, so it is correct at driver-query time,
;    when the driver has no RAM of its own yet.
;
;    The expected bytes may live in ROM: they are only ever read.
;
;    Bailing out on the first mismatch leaves the rest of the result
;    FIFO unread. That is safe: the firmware's res_begin() resets the
;    FIFO on the next command byte, so the stale bytes are dropped
;    rather than delivered to the following request.

MB_RESULT_IS:
	ld	e,a		;E = count
	ld	b,0		;B = index of the byte under test
MB_RI_L:
	ld	a,(MBOX_DATA)
	cp	(hl)
	jr	nz,MB_RI_NO
	inc	hl
	inc	b
	dec	e
	jr	nz,MB_RI_L
	xor	a		;Cy=0: all bytes matched
	ret
MB_RI_NO:
	ld	c,a		;C = the byte we read
	ld	d,(hl)		;D = the byte we wanted
	scf			;Cy=1: mismatch
	ret

;--- MB_GETRES: pop A result bytes from the DATA port into (HL).
;
;    *** (HL) MUST BE WRITABLE RAM. ***
;    This routine is NOT safe to point at a `ds` buffer in this driver:
;    the driver is a ROM image, so `ld (hl),a` into it is discarded by
;    the Z80 and the destination keeps whatever the ROM file contains
;    (zeros, for a `ds`). Use MB_RESULT_IS when the result only has to
;    be checked, and hand MB_GETRES a kernel-supplied RAM pointer when
;    the bytes have to be kept.
;    Trashes AF, C.

MB_GETRES:
	ld	c,a
MB_GETRES_L:
	ld	a,(MBOX_DATA)
	ld	(hl),a
	inc	hl
	dec	c
	jr	nz,MB_GETRES_L
	ret

;--- MB_STDEV: send command A for device B, poll, pop its 1-byte result.
;    Used for STATUS / STAPEEK.
;    In:  A = command byte, B = device number
;    Out: A = result byte, Cy=0.  Cy=1 on timeout.
;    Trashes AF, BC, HL.
;
;    Why the device number cannot simply be pushed as an argument value:
;    MB_SEND copies its argument bytes out of memory addressed by HL, and
;    the only memory this ROM driver can name as an argument source is ROM
;    (see the note at the top of the file - a `ds` buffer cannot be
;    written to). So the device byte is one of two READ-ONLY bytes,
;    MB_DEV1 / MB_DEV2, selected by a compare. That costs 2 bytes of ROM
;    and is the only place the driver names a device as a constant; a
;    third device is a third byte and one more compare here.
;
;    The command byte is stacked across the select because A is needed for
;    both jobs and there is only one of it. `pop af` is safe: the compare's
;    flags have already been consumed by the jr.

MB_STDEV:
	push	af		;the command byte
	ld	a,b		;A = device number
	cp	MB_DEV_DISK
	ld	hl,MB_DEV2
	jr	z,MB_STDEV_GO
	ld	hl,MB_DEV1	;device 1, and anything else the callers
				;did not already reject
MB_STDEV_GO:
	ld	b,1		;one argument byte
	pop	af		;A = command byte again
	call	MB_SEND
	call	MB_POLL
	ret	c
	ld	a,(MBOX_DATA)
	or	a
	ret

;--- MB_RD4: pop 4 result bytes from the DATA port into L, H, E and D, in
;    stream order (first byte -> L, fourth -> D).
;
;    The register-load counterpart to MB_RESULT_IS: use that when the
;    result only has to be checked, this when it has to be kept. Four
;    bytes is the most that fits without spilling, which covers the LBA
;    and the sector count - the only multi-byte results this driver has.
;    Trashes AF, DE, HL.
;
;    Trashes HL, and that is the whole reason it has no caller: L and H
;    are the result, so a routine that is holding a pointer in HL cannot
;    call this and go on using that pointer. DO_DEVQ_GET_PARAMS wanted
;    exactly this and had to stream into the destination instead - see
;    the note there, which is the same trap as MB_RESULT_IS destroying
;    the DE print callback.

MB_RD4:
	ld	a,(MBOX_DATA)
	ld	l,a
	ld	a,(MBOX_DATA)
	ld	h,a
	ld	a,(MBOX_DATA)
	ld	e,a
	ld	a,(MBOX_DATA)
	ld	d,a
	ret

;--- MB_HANDSHAKE_CHK: probe the firmware mailbox.
;    Out: Cy=0 = firmware answered with the "RNX2" magic;
;         Cy=1 = no firmware / foreign cart / wrong magic.
;    On failure, for MB_HS_NO only (MB_RESULT_IS left them intact):
;         B = 0-based index of the first byte that differed
;         C = the byte that was actually read
;    The expected byte is NOT returned: it is MB_EXPECT[B], still in ROM
;    and just as readable from the report code, while D is unavailable
;    because DE is where the caller keeps the print callback.
;    Trashes AF, BC, DE, HL.
;    Consequently every caller must save its DE across this call before
;    doing anything else with it - see DO_DRVQ_GET_INIT_PARAMS.

MB_HANDSHAKE_CHK:
	ld	a,MB_HANDSHAKE
	ld	hl,MB_NOARGS
	ld	b,0
	call	MB_SEND
	call	MB_POLL
	jr	c,MB_HS_TO
	ld	hl,MB_EXPECT		;ROM is fine here: read-only
	ld	a,MB_HSLEN
	call	MB_RESULT_IS
	jr	c,MB_HS_NO
	xor	a		;Cy=0: magic OK
	ret
;--- On failure Cy=1 and A carries the reason:
;    A=0 -> MB_POLL gave up: DONE never became visible to the Z80.
;    A=1 -> the firmware answered, but the bytes read back are wrong.
;    These point at different faults (the status path vs the data path),
;    and collapsing both into one silent RESULT_INIT_ERROR is what made
;    a bus-level fault indistinguishable from "the driver never ran".

MB_HS_TO:
	xor	a		;A=0: DONE never appeared
	scf
	ret
MB_HS_NO:
	ld	a,1		;A=1: answered, wrong bytes
	scf			;B/C from MB_RESULT_IS are left alone: they
				;are the whole diagnosis
	ret

;--- Zero-arg source byte (MB_SEND arg pointer for no-arg commands: the
;    handshake and abort). The byte's VALUE is never sent (B = 0 means no
;    argument burst at all); it exists so HL has something legal to point
;    at, and so it cannot accidentally read as a real argument if a caller
;    ever gets the count wrong.

MB_NOARGS:	db	0

;--- The device byte to send, as a function of the device number.
;
;    Read-only, like everything else in this file. MB_STDEV selects
;    between these two; the two sector paths inline `ld a,(iy+0)` and
;    write it straight to MBOX_DATA instead, so these are for the one-byte
;    commands only.

MB_DEV1:	db	MB_DEV_FLANKY
MB_DEV2:	db	MB_DEV_DISK

;--- The handshake answer the firmware is required to send: "RNX2" plus the
;    firmware version byte. In ROM on purpose - MB_RESULT_IS only ever
;    READS the expected bytes, so this is the one kind of table a ROM
;    driver can have.
;
;    There is deliberately no matching result buffer. The two `ds 5` /
;    `ds 8` buffers that used to sit here were the bug: they are part of
;    the cartridge image, the Z80 cannot write to them, and MB_GETRES
;    `ld (hl),a` into them was discarded - so the handshake compared the
;    firmware's correct "RNX2" reply against the 00 00 00 00 00 baked
;    into the ROM file and always reported "bad magic".
;
;    The fifth byte is the protocol revision, and it is a REAL check now:
;    MB_EXPECT is compared byte by byte, so a firmware still speaking v1
;    (one device, no device byte) reports "bad magic at byte 4" and the
;    kernel skips this driver. That is the intended outcome of a half
;    flashed update - much better than letting it run and answering every
;    CAPACITY for device 1 while the kernel thinks it asked about device 2.

MB_HSLEN	equ	5
MB_EXPECT:	db	MB_MAGIC0,MB_MAGIC1,MB_MAGIC2,MB_MAGIC3
		db	MB_PROTOCOL_V2	;== NEXTOR_VERSION in nextor.h

;--- INC32: increment the 32-bit little-endian value at (HL) in place.
;    Preserves HL. Trashes AF. (inc (hl) sets Z only when the byte
;    wrapped to zero, which is exactly the carry-propagation condition.)

INC32:
	push	hl		;return HL unchanged even after wraps
	inc	(hl)
	jr	nz,INC32_D
	inc	hl
	inc	(hl)
	jr	nz,INC32_D
	inc	hl
	inc	(hl)
	jr	nz,INC32_D
	inc	hl
	inc	(hl)
INC32_D:
	pop	hl
	ret

;-----------------------------------------------------------------------------
; Driver queries
;-----------------------------------------------------------------------------

DRIVER_QUERY:
	dec	a
	jp	z,DO_DRVQ_GET_VERSION
	dec	a
	jp	z,DO_DRVQ_GET_STRING
	dec	a
	jp	z,DO_DRVQ_GET_INIT_PARAMS
	dec	a
	jp	z,DO_DRVQ_INIT
	dec	a
	jp	z,DO_DRVQ_GET_MAX_DEVICE
	ld	a,RESULT_NOT_IMPLEMENTED
	ret


; Driver query 1: Get driver version number
; Out: A = RESULT_OK, version in B.C.D = 1.0.0

DO_DRVQ_GET_VERSION:
	ld	bc,0000h
	ld	d,1
	xor	a
	ret


; Driver query 2: Get driver information string
; In:  B = string index (1 = driver name), D = buffer size, HL = buffer
; Out: A = RESULT_OK / RESULT_TRUNCATED_STRING / RESULT_NOT_IMPLEMENTED

DO_DRVQ_GET_STRING:
	ld	a,b
	ld	b,d
	ex	de,hl
	dec	a
	ld	hl,MSG_DRIVER_NAME
	jp	z,OUTPUT_STRING
	ld	a,RESULT_NOT_IMPLEMENTED
	ret


; Driver query 3: Get driver initialization parameters
;
; In:  HL = available work area size, B = available drives, C = flags,
;      DE = print-character routine
; Out: A = RESULT_OK / RESULT_INIT_ERROR, B = flags (0 = no hooks),
;      HL = page-3 space required (0)
;
; The mailbox handshake IS the hardware detection: on a foreign cart
; (openMSX generic ASCII16, another flash cart, no firmware) the CMD
; write lands on ordinary ROM data, nothing answers, MB_POLL times out
; and we return RESULT_INIT_ERROR - the kernel then skips this driver
; gracefully instead of wedging the boot.

DO_DRVQ_GET_INIT_PARAMS:
	;DE is the print callback, and MB_HANDSHAKE_CHK destroys DE while
	;handing back its diagnosis in B/C. Save the callback across the call:
	;the success path pops it here, the failure path pops it in
	;DO_DRVQ_INIT_FAIL, so the stack is balanced on both exits.
	push	de
	call	MB_HANDSHAKE_CHK
	jr	c,DO_DRVQ_INIT_FAIL
	pop	de
	xor	a		;RESULT_OK
	ld	b,0		;no TIMER_INT / EXTBIO hooks
	ld	hl,0		;no page-3 work area
	ret

;--- Reached from query 3 AND query 4, and DE holds the print callback in
;    both, so this is the one place that can report "the driver ran but
;    the mailbox did not answer", whichever query got that far.
;
;    A on entry is the MB_HANDSHAKE_CHK failure reason (0 = no DONE,
;    1 = wrong bytes); the two get distinct messages because they are
;    different faults and the fix differs. For reason 1, MB_HANDSHAKE_CHK
;    also left the first mismatching byte's index in B and the byte it
;    read in C, both of which survive the print helpers (they touch only
;    AF, HL and IX and restore DE around their stack trampoline). The
;    byte that was expected there is MB_EXPECT[B], read back out of ROM
;    here rather than carried in D, because D is half of DE and DE is
;    the callback this whole routine prints through.
;
;    The callback itself is on the stack: both callers push DE before the
;    handshake and this entry pops it, so the kernel's stack is left
;    exactly as it was found on either exit.
;
;    Note this also means a failing query 3 keeps query 4 from ever
;    running: the kernel prints its own "Driver initialization failed!"
;    and jumps to NO_DRIVE, which is why a bad handshake shows up as no
;    banner and no wait at all.

DO_DRVQ_INIT_FAIL:
	pop	de		;the callback the caller saved before the handshake
	or	a		;Cy=1: A was 0 (no DONE) or 1 (bad bytes)
	jr	nz,DO_DRVQ_INIT_BADMAGIC
	ld	hl,MSG_NO_ANSWER
	call	PRINT_WITH_DE
	jr	DO_DRVQ_INIT_RETURN
DO_DRVQ_INIT_BADMAGIC:
	;Report index / got / want. Three bytes say as much as a dump of
	;all five and, unlike a dump, they survive: the driver has nowhere
	;to put a dump, because every buffer it could use is in ROM.
	ld	hl,MSG_BAD_MAGIC
	call	PRINT_WITH_DE
	ld	a,b
	call	PRINT_HEXA_DE		;index of the first differing byte
	ld	hl,MSG_GOT
	call	PRINT_WITH_DE
	ld	a,c
	call	PRINT_HEXA_DE		;the byte that came back
	ld	hl,MSG_WANT
	call	PRINT_WITH_DE
	ld	hl,MB_EXPECT		;the byte that should have: still in ROM
	ld	a,b		;index (B is spent, so rebuild it in BC)
	ld	b,0
	ld	c,a
	add	hl,bc
	ld	a,(hl)
	call	PRINT_HEXA_DE
DO_DRVQ_INIT_RETURN:
	;Every print above is immediately preceded by its own ld hl, so HL is
	;always a known-good string pointer here; PRINT_WITH_DE walks HL and
	;would otherwise run off the end of the message.
	ld	hl,MSG_CRLF
	call	PRINT_WITH_DE
	ld	a,RESULT_INIT_ERROR
	ld	b,0
	ld	hl,0
	ret


; Driver query 4: Initialize driver
;
; Prints the banner BEFORE the handshake gate, deliberately. The banner
; is the proof that the kernel reached this driver at all; the handshake
; verdict that follows is the proof that the firmware mailbox answered.
; Printing after the gate collapses those two facts into one, which is
; exactly the ambiguity this ordering removes:
;
;   banner + OK + 5 s stall  -> driver runs, mailbox answers, all good
;   banner + "no mailbox"    -> driver runs, firmware not answering
;   nothing at all           -> driver not called, or the print path
;                              itself is broken
;
; The wait is a deliberate boot-visible stall proving the driver got as
; far as running code. It happens after the banner, so the text is on
; screen while the machine is held.

DO_DRVQ_INIT:
	ld	hl,INIT_MSG
	call	PRINT_WITH_DE		;unconditional
	push	de			;same reason as in query 3 above
	call	MB_HANDSHAKE_CHK	;re-verify (queries 3 and 4 run back to back)
	jr	c,DO_DRVQ_INIT_FAIL
	pop	de
	xor	a		;RESULT_OK
	ret


; Driver query 5: Get maximum supported device number
;
; The kernel counts UP from 1 to this answer, so it is a count AND a
; maximum: device numbers are 1..MB_DEVICE_COUNT with no gaps. Keep this,
; DEVICE_QUERY's own range test and the DEVICE NAMES below in step - the
; firmware has its own copy of the count in NEXTOR_DEVICE_COUNT, and the
; handshake is what keeps the two from drifting.

DO_DRVQ_GET_MAX_DEVICE:
	ld	a,RESULT_OK
	ld	b,MB_DEVICE_COUNT	;devices 1 and 2
	ret


;-----------------------------------------------------------------------------
; Device queries
;-----------------------------------------------------------------------------

;--- DEVICE_QUERY dispatcher.
;    In:  A = query index, C = device number.
;    The device number is validated BEFORE the index is consumed
;    (.IDEVN for unknown devices regardless of the query index), and C is
;    left intact so each device query still knows which device it is for.

DEVICE_QUERY:
	push	af		;save the query index
	ld	a,c
	cp	MB_DEV_FLANKY	;C < 1: device numbers start at 1
	jr	c,DQ_DEVN
	cp	MB_DEVICE_COUNT+1
	jr	nc,DQ_DEVN	;C > 2
	pop	af		;recover the query index
	dec	a
	jp	z,DO_DEVQ_GET_STRING
	dec	a
	jp	z,DO_DEVQ_GET_PARAMS
	dec	a
	jp	z,DO_DEVQ_GET_STATUS
	dec	a
	jp	z,DO_DEVQ_GET_AVAILABILITY
	dec	a
	jp	z,DO_DEVQ_GET_FORMAT_CHOICES
	dec	a
	jp	z,DO_DEVQ_DO_FORMAT
	dec	a
	jp	z,DO_DEVQ_STOP_MOTOR
	ld	a,RESULT_NOT_IMPLEMENTED
	ret
DQ_DEVN:
	pop	af		;discard (keep the stack clean)
	ld	a,RESULT_INVALID_DEVICE
	ret


; Device query 1: Get device information string
; In:  B = string index (4 = device name), D = buffer size, HL = buffer,
;      C = device number (still intact from the dispatcher)
; Out: A = RESULT_OK / RESULT_TRUNCATED_STRING / RESULT_NOT_IMPLEMENTED
;
; Two devices, two names - NEXTOR's own device-parameter screen shows
; this string as the device's description, and "720K disk image" on the
; read-write disk is not a useful thing to read.
;
; C survives OUTPUT_STRING (it modifies AF, B, DE and HL only), so the
; device number is still available after the copy and does not have to be
; stacked across it. The B it DOES destroy is why the buffer size is
; moved into B first.

DO_DEVQ_GET_STRING:
	ld	a,b		;A = string index
	ld	b,d		;B = buffer size (OUTPUT_STRING wants it in B)
	ex	de,hl		;HL = buffer, DE = string pointer
	dec	a
	jr	nz,DO_DEVQ_STR_NI
	ld	a,c		;A = device number
	cp	MB_DEV_DISK
	ld	hl,MSG_DEVICE_NAME2
	jr	z,DO_DEVQ_STR_OUT
	ld	hl,MSG_DEVICE_NAME	;device 1; the dispatcher has already
					;rejected anything above 2
DO_DEVQ_STR_OUT:
	call	OUTPUT_STRING
	xor	a		;RESULT_OK
	ret
DO_DEVQ_STR_NI:
	ld	a,RESULT_NOT_IMPLEMENTED
	ret


; Device query 2: Get device parameters
;
; In:  HL = buffer address, or 0 for "validate only" (must be supported),
;      C = device number (still intact from the dispatcher)
; Out: A = RESULT_OK / RESULT_NOT_IMPLEMENTED
;
; Buffer layout (12 bytes):
;   +0 (1)  device type: 0 = block device
;   +1 (2)  sector size LE: always 512, medium or not, device or not
;   +3 (4)  total sectors LE (from firmware CMD_CAPACITY over the
;           image file; 0 when there is no image / no capacity yet)
;   +7 (1)  flags: bit0 removable, bit2 floppy disk drive
;   +8..11  cylinders(2)/heads(1)/sectors-per-track(1)
;
; Bytes +0..+2 are written by the shared DQP_HDR and +7..+11 by the
; shared DQP_TAIL, so the medium and no-medium answers cannot drift
; apart and device 1 and device 2 cannot drift apart. They must not: the
; kernel reads this block at two different points with two different
; expectations, and it can see one without the other.
;
; +8..+11 is NOT the same for the two devices, and the difference is the
; whole point of device 2. See DQP_TAIL.

DO_DEVQ_GET_PARAMS:
	ld	a,h
	or	l
	jr	nz,DQP_FILL
	xor	a		;HL=0: validate only -> RESULT_OK
	ret
DQP_FILL:
	;MB_SEND takes its argument pointer in HL and leaves HL advanced past
	;the arguments, so it eats HL - and HL is this buffer address. Without
	;the save the whole 12-byte fill below ran at MB_DEV2, inside the
	;driver's own image: the kernel got an untouched parameter block and
	;the Z80 wrote over its own code. MB_SEND documents HL as an output
	;(see its header); it is easy to read the `ld hl,MB_DEV2` as merely
	;picking a source and miss that it also overwrites the caller's HL.
	push	hl		;the buffer address

	;CAPACITY takes the device byte as its only argument. D also carries
	;the device number to DQP_TAIL - it cannot stay in C, because MB_POLL
	;does `ld bc,0FFFFh` and destroys C, and it cannot go on the stack
	;either because DQP_TAIL is reached from two places. D is safe across
	;both: MB_SEND uses A/B/HL, and MB_POLL's exx protects DE'.
	ld	a,c
	cp	MB_DEV_DISK
	ld	a,MB_DEV_DISK
	ld	hl,MB_DEV2
	jr	z,DQP_SEND
	ld	a,MB_DEV_FLANKY
	ld	hl,MB_DEV1
DQP_SEND:
	ld	d,a		;device number, for DQP_TAIL
	ld	a,MB_CAPACITY
	ld	b,1		;one argument byte
	call	MB_SEND
	pop	hl
	call	MB_POLL
	jr	c,DQP_TMO
	;DONE is not the same as success. The firmware sets ERR together
	;with DONE whenever the command failed - which is what it does
	;when the image file is not there - and an empty result FIFO pops
	;0xFF, not 0.
	;So without this test the four pops below return FF FF FF FF and
	;the driver hands the kernel a 4294967295-sector disk built out of
	;one's-complement noise. (The sector count is the one field the
	;kernel cannot sanity-check: a device that big looks plausible.)
	ld	a,(MBOX_STAT)
	bit	5,a		;MBST_ERR
	jp	nz,DQP_NOMEDIA
	;Write the fixed fields first so that HL - the only thing holding the
	;buffer address - arrives at the sector-count field exactly when the
	;count starts coming out of the FIFO.
	call	DQP_HDR
	;Stream the 4 count bytes straight into +3..+6. Both the FIFO and the
	;buffer field are little-endian, so stream order is field order and no
	;byte swapping is needed.
	;
	;This deliberately does NOT use MB_RD4. MB_RD4 hands the count back in
	;L and H, and L and H are this buffer pointer, so a `call MB_RD4` here
	;would move the fill to wherever the count says. Consuming into the
	;destination needs no registers at all, so it is both correct and
	;shorter than staging the count in registers and copying it back.
	;
	;The block-size half of CAPACITY's 8 result bytes is left unread: the
	;firmware clears the result FIFO when the next command byte lands, and
	;512 is this driver's only block size for either device.
	ld	a,(MBOX_DATA)
	ld	(hl),a
	inc	hl
	ld	a,(MBOX_DATA)
	ld	(hl),a
	inc	hl
	ld	a,(MBOX_DATA)
	ld	(hl),a
	inc	hl
	ld	a,(MBOX_DATA)
	ld	(hl),a
	inc	hl
	jr	DQP_TAIL
DQP_TMO:
	ld	a,RESULT_NOT_IMPLEMENTED
	ret

;--- The three fixed header bytes: what this device IS, as opposed to what
;    it currently contains. Both the medium and no-medium answers come
;    through here on purpose, because the one thing that must never vary
;    with the medium is the device's own geometry.
DQP_HDR:
	ld	(hl),0		;+0 type: block device
	inc	hl
	ld	(hl),00h	;+1 sector size LE low
	inc	hl
	ld	(hl),02h	;+2 sector size LE high
	inc	hl		;the last inc hl, and the reason this routine
				;needs a test of its own: without it DQP_HDR
				;returns with HL on +2, every caller then
				;writes one byte low, and the whole block
				;comes out shifted left by one - a sector
				;size of 256 and the count in the wrong
				;place, with no error anywhere to notice.
	ret		;HL = buffer+3, the sector count field

;--- No medium: the same device, reporting zero available sectors.
;
;    The sector SIZE must still be 512, and this is not a formality:
;    both of the kernel's automap paths reject anything else outright.
;    bank4/partit.mac:1032-1039 (boot-time assign) and
;    bank6/idrvauto.mac:878-893 (driver loaded into RAM) both read
;    +1/+2 and require low byte 0 and high byte 2, and both bail to
;    "try the next device" otherwise. A sector size of 0 here - which
;    the SDK's "0 if not available" wording seems to invite - means the
;    device is never automapped, ever. An earlier version of this code
;    did exactly that, and the result was a drive the kernel counted
;    but never assigned: no A: at all.
;
;    RESULT_NOT_IMPLEMENTED is NOT used here: the kernel reads it as
;    "block device, 512 byte sectors, unknown total, flags 0" (device
;    query 2 in the SDK template driver), and that substituted default
;    cannot set the floppy bit - so a missing NEXTOR.DSK would silently
;    turn this into a partition-scanning device and lose the drive
;    instead of keeping it.
DQP_NOMEDIA:
	call	DQP_HDR
	xor	a
	ld	b,4		;+3..6 total sectors = 0
DQP_NCLEAR:
	ld	(hl),a
	inc	hl
	djnz	DQP_NCLEAR
	jr	DQP_TAIL

;--- Shared tail: the flags and geometry bytes, then RESULT_OK. Reached
;    with HL = buffer+7 and D = device number.
;
;    +7 FLAGS differ per device, and the difference is not cosmetic:
;
;      bit 2 (floppy disk drive) - SET on device 1, CLEAR on device 2.
;
;        Device 1 MUST have it. With it clear the kernel's automapper does
;        a full partition scan; there is none on a flat .dsk file, and
;        the resulting "no partition" outcome means the drive is never
;        assigned. bank4/partit.mac:1054-1092 routes the floppy bit
;        straight to AA_DO_ASSIGN with first-sector = 0.
;
;        Device 2 MUST NOT have it. NEXTOR.IMG is a hard-disk-shaped
;        image - its sector 0 is an MBR and its length is its capacity -
;        and the floppy flag would make the kernel map a filesystem onto
;        sector 0, i.e. read the MBR as a boot sector. Clearing the bit
;        sends it down the ordinary partition-scan path instead, which is
;        what finds the partition entry and produces a real drive letter.
;
;      bit 0 (removable) - SET on both. The medium is a file on the USB
;        stick, which can come and go. The removable fallback path in
;        partit.mac:1128-1136 is the right one for "image went away
;        between probes" - without it the kernel thinks the drive is
;        always-on, which makes the eject-on-removal case (stick pulled)
;        look like a media change that isn't followed by a re-probe.
;
;      bit 1 (read only) - deliberately CLEAR on both. The kernel honours
;        it by refusing writes itself; setting it here too just makes the
;        drive look "frozen" in a way that confuses partition tools (they
;        ask the device, get RO, skip). The firmware answers device 1's
;        writes with NEXTOR_ERR_READONLY, which the kernel turns into
;        .WPROT - and this driver refuses a write to device 1 locally,
;        before any command byte is sent. See RW_ONE_WRITE.
;
;      bit 3 (no automapping) stays clear on both so the devices are still
;        automapped at boot.
;
;    +8..+11 CHS GEOMETRY. Device 2 reports zeros - its capacity is a
;    FILE LENGTH, not the product of three numbers, and there is no
;    honest CHS triple to report. Zeros is the documented "not available"
;    answer (guide 4.6.2), and for a non-floppy HDD the kernel's
;    partition-scan path ignores +8..+11.
;
;    Device 1 (the floppy) is different. The floppy shortcut in partit.mac
;    routes the device straight to A: with first-sector 0, and that path
;    REQUIRES a non-zero, internally-consistent CHS to recognise the
;    device as a floppy at all. Zeros aborts the shortcut and the kernel
;    never assigns A: (it then asks for "drive B", the next empty drive
;    letter). A hardcoded 80/2/9 = 1440 works for the standard 720K .dsk
;    but contradicts any other total (e.g. an 896KB image with tot=1792),
;    and the contradiction aborts the shortcut the same way.
;
;    The chosen compromise: 80/2/9 for the 720K case (exact match,
;    unchanged behaviour), 255/2/9 = 4590 for anything else. The 255/2/9
;    CHS capacity is always >= total, so the kernel takes min(total, CHS)
;    = total for the drive size and the floppy shortcut fires. Inventing
;    a "correct" CHS for an arbitrary size would need a division by the
;    host's heads*spt, and the kernel doesn't actually USE the CHS for
;    boot (it reads sector 0 via LBA and parses the BPB) - it only uses
;    CHS to recognise the floppy shortcut, so "big enough to not look
;    broken" is the right answer. See DQP_TAIL_FDK for the 4-byte compare
;    that selects between the two.
DQP_TAIL:
	ld	a,d
	cp	MB_DEV_FLANKY
	jr	z,DQP_TAIL_FDK

	;Device 2: removable only, no floppy bit, no CHS geometry.

	ld	(hl),01h	;+7 flags: bit0 removable
	inc	hl
	xor	a
	ld	b,4		;+8..11 geometry: not available
DQP_GEOM0:
	ld	(hl),a
	inc	hl
	djnz	DQP_GEOM0
	jr	DQP_DONE

DQP_TAIL_FDK:
	ld	(hl),05h	;+7 flags: bit0 removable + bit2 floppy
	;
	;CHS geometry must be NON-ZERO and consistent with the capacity
	;for the floppy shortcut to fire and map this device to A:.
	;Zeros caused Nextor to skip the floppy path entirely and leave
	;A: empty (the kernel then asks "insert disk in drive B" - the
	;next drive letter, also empty). A hardcoded 80/2/9 = 1440 is
	;correct for the standard 720K .dsk but contradicts any other
	;total (e.g. the 896KB image with tot=1792), and the
	;contradiction aborts the shortcut the same way.
	;
	;Solution: 80/2/9 for the 720K case (exact match, unchanged
	;behaviour), 255/2/9 = 4590 for everything else. The 255/2/9
	;CHS capacity is always >= total, so the kernel takes
	;min(total, CHS) = total for the drive size and the floppy
	;shortcut fires. No division: a 4-byte compare against the
	;720K signature (A0 05 00 00 LE) is enough.
	;
	;IX is used here as the base for negative-displacement reads -
	;HL has no indexed addressing mode, only IX/IY do.
	push	hl
	pop	ix		;IX = buffer+7 (flags byte, just written)
	ld	a,(ix-4)	;buffer+3 (total LSB)
	cp	0A0h
	jr	nz,DQP_FDK_MAX
	ld	a,(ix-3)	;buffer+4
	cp	05h
	jr	nz,DQP_FDK_MAX
	ld	a,(ix-2)	;buffer+5
	or	a
	jr	nz,DQP_FDK_MAX
	ld	a,(ix-1)	;buffer+6 (total MSB)
	or	a
	jr	nz,DQP_FDK_MAX
	;Total == 1440 -> 80/2/9 (exact). IX is still buffer+7 here;
	;(ix+1) writes buffer+8, etc.
	ld	(ix+1),50h	;+8 cylinders low = 80
	ld	(ix+2),0	;+9 cylinders high
	ld	(ix+3),2	;+10 heads
	ld	(ix+4),9	;+11 sectors per track
	jr	DQP_DONE
DQP_FDK_MAX:
	;Any other total -> 255/2/9 (always >= total)
	ld	(ix+1),0FFh	;+8 cylinders low = 255
	ld	(ix+2),0	;+9 cylinders high
	ld	(ix+3),2	;+10 heads
	ld	(ix+4),9	;+11 sectors per track
DQP_DONE:
	xor	a		;RESULT_OK
	ret


; Device query 3: Get device status
; In:  C = device number
; Out: A = RESULT_OK, B = 0 no media / 1 ready / 2 media changed.
; Consumes the firmware's change latch (this query is what the
; "changed once, then ready" tracking is built on).
;
; This used to answer `ld b,1 / xor a` without going near the mailbox:
; correct only while there was one device whose medium was whatever it was
; at boot. With two devices it is wrong in both directions - a missing
; NEXTOR.IMG reported as "ready" makes the kernel mount a drive that is
; not there, and a stick that was plugged in late reported as "no media"
; is never noticed at all. The mailbox already knows the answer, because
; CMD_STATUS is the same call the sector paths make.

DO_DEVQ_GET_STATUS:
	ld	b,c		;B = device number
	ld	a,MB_STATUS	;A = command byte
	call	MB_STDEV
	jr	c,DQ_STAT_TMO
	ld	b,a		;0 no media / 1 ready / 2 changed
	ld	a,RESULT_OK
	ret


DQ_STAT_TMO:
	ld	a,RESULT_NOT_IMPLEMENTED
	ld	b,1
	ret


; Device query 4: Get device availability
; In:  C = device number
; Out: A = RESULT_OK, B = 0 not available / 1 available.
; Must NOT consume the change latch -> uses CMD_STAPEEK; a "changed"
; result still counts as "available".

DO_DEVQ_GET_AVAILABILITY:
	ld	b,c		;B = device number
	ld	a,MB_STAPEEK	;A = command byte
	call	MB_STDEV
	jr	c,DQ_STAT_TMO
	ld	b,a
	or	a		;0 no media / 1 ready / 2 changed
	jr	z,DQ_AVAIL_NO
	ld	b,1		;ready or just-changed: available
	jr	DQ_AVAIL_DONE
DQ_AVAIL_NO:
	xor	a		;B = 0: no medium
DQ_AVAIL_DONE:
	ld	a,RESULT_OK
	ret


; Device queries 5-7: the format-related ones.
;
; Device 1 is flagged as a floppy (DQP_TAIL bit 2) but is read-only, so
; there is nothing to offer. Device 2 is NOT flagged as a floppy, but
; formatting it means throwing away a disk image that lives in a file on
; a removable stick - and the honest way to do that is to delete the file
; from the firmware terminal's Nextor menu and let the user create a new
; one, which is a choice with a size in front of it. Formatting here
; would be a silent wipe of whatever was on the medium, which is not
; something a driver should do behind the user's back.
;
; RESULT_NOT_IMPLEMENTED is the documented answer for "this device is not
; formattable" (guide 4.6.5/4.6.6: "for floppy disks if the driver
; doesn't support formatting it should always return
; RESULT_NOT_IMPLEMENTED"), and it is what all three queries are required
; to answer together - query 5 and 6 are a pair, so leaving 5
; unimplemented and answering 6 would be the inconsistent one.
;
; Consequence: CALL FORMAT (BASIC) and COMMAND3.COM's FORMAT still list the
; drive with the kernel's default single/double side choices, and the
; format itself then fails on query 6. The user-visible result is an
; error rather than a silent "formatted", which is the honest outcome.
DO_DEVQ_GET_FORMAT_CHOICES:
DO_DEVQ_DO_FORMAT:
DO_DEVQ_STOP_MOTOR:
	ld	a,RESULT_NOT_IMPLEMENTED
	ret


;--- Custom driver/device queries: none.

CUSTOM_DRIVER_QUERY:
CUSTOM_DEVICE_QUERY:
	ld	a,RESULT_NOT_IMPLEMENTED
	ret


;-----------------------------------------------------------------------------
; READ_WRITE
;-----------------------------------------------------------------------------

;--- READ_WRITE: read or write logical sectors.
;
;    In:  Cy = 0 read / 1 write
;         A  = device number (1 or 2)
;         B  = sector count
;         C  = media descriptor (ignored)
;         HL = buffer (non page-1, direct accessible)
;         DE = address of the 4-byte LBA (LE), non page-1
;    Out: A = DOS error code (0 = ok), B = sectors transferred
;
;    Persistent register plan (across the per-sector loop):
;      IX  = buffer pointer (advanced 512 per sector)
;      B   = sectors remaining (helpers preserve B)
;      DE' = address of the 4-byte LBA value in kernel memory
;      IY  = stack frame (helpers never touch IY)
;
;    Per sector: media check -> CMD_READ/CMD_WRITE + dev + LBA(4) ->
;    (read: drain 512 from DATA / write: push 512 to DATA) -> poll DONE
;    -> map errors -> increment the 32-bit LBA at (DE').
;    On error: B = sectors transferred = original count - remaining.

READ_WRITE:
	;Frame layout once the pushes are done (IY = SP, bytes in Z80 order):
	;
	;   (iy+0)  A on entry = device number / A on exit = DOS error code
	;   (iy+1)  F on entry; bit 0 is the read/write selector
	;   (iy+10) C on entry = media descriptor (unused)
	;   (iy+11) B on entry = sector count / sectors transferred on exit
	;
	;AF is pushed LAST, which is what puts the device number at (iy+0).
	;That offset is not arbitrary: it is the one byte the per-sector
	;helpers read on every sector AND the one slot the returned error code
	;is written to, so both the device number and the exit code survive in
	;the same place and both exits can share RW_EXIT. It is also the only
	;offset none of the helpers can move - they all save and restore BC,
	;DE and HL around themselves and none of them touches IY at all.
	;
	;Pushed bottom-up BC, DE, HL, IX, IY, AF, so the pops are the reverse:
	push	bc
	push	de
	push	hl
	push	ix
	push	iy
	push	af
	ld	iy,0
	add	iy,sp

	;Device number must be 1 or 2 (.IDEVN otherwise). Same range the
	;device-query dispatcher accepts, and the same one the firmware's
	;device table has rows for.

	ld	a,(iy+0)
	cp	MB_DEV_FLANKY	;below 1: device numbers start at 1
	jr	c,RW_IDEVN
	cp	MB_DEVICE_COUNT+1
	jr	nc,RW_IDEVN

	;Zero sectors: immediate success.

	ld	a,b
	or	a
	jr	z,RW_OK

	;Persistent state: IX = buffer, DE' = LBA pointer,
	;B (main set) = remaining count (entry value, intact after pushes).

	push	hl
	pop	ix		;IX = buffer pointer
	push	de
	exx
	pop	de		;DE' = LBA pointer
	exx

	;Dispatch on Cy (bit 0 of the saved F).

	ld	a,(iy+1)	;saved F
	rra			;bit0 (Cy) -> carry
	jr	c,RW_W_LOOP

	;--- READ loop ---

RW_R_LOOP:
	call	RW_ONE_READ
	or	a
	jp	nz,RW_ERROR
	djnz	RW_R_LOOP
	jp	RW_OK

	;--- WRITE loop ---

RW_W_LOOP:
	call	RW_ONE_WRITE
	or	a
	jp	nz,RW_ERROR
	djnz	RW_W_LOOP

RW_OK:
	xor	a		;A = 0 (ok)
	ld	(iy+0),a	;becomes the returned A at exit
	ld	b,(iy+11)	;B = all sectors transferred
	jr	RW_EXIT

	;--- Error exit: A = error code (from the helper), B = remaining
	;    count (the djnz has not run yet for the failed sector, so
	;    remaining is intact). Done = original count - remaining.
	;    The error code is written into the saved-A slot so it
	;    survives the register restore below.

RW_ERROR:
	push	af		;stash the error code
	ld	a,(iy+11)	;original B = total requested
	sub	b		;done = total - remaining
	ld	b,a
	pop	af		;recover the error code
	ld	(iy+0),a	;it becomes the returned A
	jr	RW_EXIT

RW_EXIT:
	pop	af		;A = error code, F = saved flags
	pop	iy
	pop	ix
	pop	hl
	pop	de
	pop	bc		;B = sectors transferred
	ret

;--- Unknown device number.
;
;    .IDEVN is loaded here rather than left as the device number in A, and
;    that is a fix: the check above ends on a compare, so by the time
;    control reaches this label A holds the device number, and storing that
;    as the DOS error code returns the DEVICE NUMBER as the error. For
;    device 2 that is error code 2 - which is a real DOS error meaning
;    something else entirely, so the kernel reported a nonsensical disk
;    failure instead of "invalid device number".

RW_IDEVN:
	ld	a,.IDEVN
	ld	(iy+0),a
	xor	a
	ld	b,a		;B = 0 sectors transferred
	jr	RW_EXIT


;--- RW_ONE_READ: transfer one sector (read).
;    IX = buffer, DE' = LBA pointer, (iy+0) = device number,
;    B = remaining (preserved).
;    Returns A = 0 or a DOS error code. Trashes AF, BC, DE, HL.
;
;    The device number is read out of the frame rather than passed in a
;    register: A holds the command byte here, B has to stay the remaining
;    count across the call, and the frame is the only place the value
;    already lives. (iy) is set up once by READ_WRITE and never disturbed
;    by anything below.

RW_ONE_READ:
	push	bc		;preserve the remaining-count register

	;Media check (consumes the change latch; that is fine - the
	;kernel calls status query 3 before any access anyway).

	ld	b,(iy+0)	;device number
	ld	a,MB_STATUS
	call	MB_STDEV
	jp	c,RW1_TMO
	or	a
	jp	z,RW1_NRDY

	;Send CMD_READ + the device byte + the 4 LBA bytes (copied
	;from (DE') via HL').

	exx
	push	de
	pop	hl		;HL' = copy of the LBA pointer
	ld	a,MB_READ
	ld	(MBOX_CMD),a	;the command byte comes FIRST...
	ld	a,(iy+0)
	ld	(MBOX_DATA),a	;...then the device, then the LBA
	ld	a,(hl)
	ld	(MBOX_DATA),a
	inc	hl
	ld	a,(hl)
	ld	(MBOX_DATA),a
	inc	hl
	ld	a,(hl)
	ld	(MBOX_DATA),a
	inc	hl
	ld	a,(hl)
	ld	(MBOX_DATA),a
	ex	de,hl		;HL' = LBA pointer (for INC32)
	call	INC32		;increment the 32-bit LBA in kernel memory
	ex	de,hl		;back: DE' = LBA pointer, HL' = ptr+4 (free)
	exx

	;Wait for the firmware to fill the sector buffer.

	call	MB_POLL
	jp	c,RW1_TMO
	ld	a,(MBOX_STAT)
	bit	5,a		;ERR bit
	jp	nz,RW1_ERR

	;Drain 512 bytes from DATA to (IX).

	ld	bc,512
RW1_RDR:
	ld	a,(MBOX_DATA)
	ld	(ix+0),a
	inc	ix
	dec	bc
	ld	a,b
	or	c
	jr	nz,RW1_RDR
	pop	bc		;restore remaining count
	xor	a
	ret

;--- RW_ONE_WRITE: transfer one sector (write).
;    Same register contract as RW_ONE_READ, and the same frame slot for
;    the device number.
;
;    Device 1 is the read-only floppy image and returns .WPROT without a
;    media check and without a command byte. That is not just an
;    optimisation: the previous version of this routine issued MB_STATUS,
;    pushed the device byte, the LBA and 512 data bytes at MBOX_DATA, and
;    only found out at the end that the firmware had refused all of it.
;    That cost 517 cart-bus cycles per sector, each one an EXTI0
;    interrupt on the firmware side, to be told the answer the device's
;    own flags already give the kernel. It also leaked the sector: the
;    command byte and its arguments went out before the refusal was
;    known, leaving a half-finished request in the mailbox collector.
;
;    .WPROT is the right code rather than .NRDY or .DISK: the guide
;    (4.9.4) says to return the DOS error that describes the failure,
;    and the kernel already uses .WPROT to report "Write protected disk"
;    for exactly this case.

RW_ONE_WRITE:
	push	bc		;preserve the remaining-count register

	;Only device 2 (the read-write .img) has anything to write to.

	ld	a,(iy+0)
	cp	MB_DEV_DISK
	jr	c,RW1_WPROT

	;Media check, exactly as in the read path.

	ld	b,(iy+0)	;device number
	ld	a,MB_STATUS
	call	MB_STDEV
	jp	c,RW1_TMO
	or	a
	jp	z,RW1_NRDY

	;Send CMD_WRITE + the device byte + the 4 LBA bytes (copied
	;from (DE') via HL').

	exx
	push	de
	pop	hl		;HL' = copy of the LBA pointer
	ld	a,MB_WRITE
	ld	(MBOX_CMD),a
	ld	a,(iy+0)
	ld	(MBOX_DATA),a	;device, then LBA
	ld	a,(hl)
	ld	(MBOX_DATA),a
	inc	hl
	ld	a,(hl)
	ld	(MBOX_DATA),a
	inc	hl
	ld	a,(hl)
	ld	(MBOX_DATA),a
	inc	hl
	ld	a,(hl)
	ld	(MBOX_DATA),a
	ex	de,hl		;HL' = LBA pointer (for INC32)
	call	INC32
	ex	de,hl
	exx

	;Push the sector: 512 bytes from (IX) to DATA. BC is the loop
	;counter here, which is safe precisely because the remaining count
	;was pushed on entry and is popped back at the end.

	ld	bc,512
RW1_WDR:
	ld	a,(ix+0)
	ld	(MBOX_DATA),a
	inc	ix
	dec	bc
	ld	a,b
	or	c
	jr	nz,RW1_WDR

	;Wait for the firmware to acknowledge. A successful write produces
	;no result bytes at all - DONE with an empty FIFO is the success
	;signal - so there is nothing to drain afterwards.

	call	MB_POLL
	jp	c,RW1_TMO
	ld	a,(MBOX_STAT)
	bit	5,a		;ERR bit
	jp	nz,RW1_ERR
	pop	bc		;restore remaining count
	xor	a
	ret

;--- Shared one-sector error tails (BC was pushed by the helper).

RW1_TMO:
	pop	bc
	ld	a,.DISK
	ret

RW1_NRDY:
	pop	bc
	ld	a,.NRDY
	ret

RW1_ERR:
	pop	bc
	ld	a,(MBOX_ERR)
	cp	MBERR_NOMEDIA
	jr	z,RW1_NRDY2
	ld	a,.DISK
	ret
RW1_NRDY2:
	ld	a,.NRDY
	ret

RW1_WPROT:
	pop	bc
	ld	a,.WPROT
	ret

;-----------------------------------------------------------------------------
; Strings and messages
;-----------------------------------------------------------------------------

	.stresc on

MSG_DRIVER_NAME:	db	"RISKY MSX 2",0
MSG_DEVICE_NAME:	db	"720K disk image",0
MSG_DEVICE_NAME2:	db	"read-write disk image",0

INIT_MSG:		db	"\r\nRISKY MSX 2 driver\r\n"
			db	"720K floppy image + read/write disk",0

;--- Init-phase verdicts. The handshake already prints enough on
;    success; the FAIL prints below cover what can still go wrong.

MSG_NO_ANSWER:	db	"RISKY MSX 2: no DONE (status never reached Z80)",0
MSG_BAD_MAGIC:	db	"RISKY MSX 2: bad magic at byte ",0
MSG_GOT:	db	" got ",0
MSG_WANT:	db	" want ",0
MSG_CRLF:	db	13,10,0

;-----------------------------------------------------------------------------
; SDK helpers
;-----------------------------------------------------------------------------

	INCLUDE asm/code/output_string.asm

	;--- Print a zero-terminated string via a character output routine
	;    Input: HL = string, DE = character output routine address
	;    Trashes: AF, HL, IX

PRINT_WITH_DE:
	push	de
	ld	de,0C300h	;JP opcode + 00
	push	de
	ld	ix,1
	add	ix,sp		;IX -> the "JP <charout>" trampoline on the stack
	call	PRINT_HL
	pop	de
	pop	de
	ret

;--- How the trampoline works (do not "fix" ix,1 to ix,2 - it is correct)
;
;    After the two pushes the stack holds, at increasing addresses:
;
;        SP+0  00     (low  byte of 0xC300)
;        SP+1  C3     (high byte of 0xC300)
;        SP+2  cb_lo  \
;        SP+3  cb_hi  /  the callback address
;
;    so the three bytes at SP+1 are the instruction `JP cb`, and the
;    job of the caller is to make the CPU EXECUTE them.
;
;    `JP (IX)` (opcode DD E9, the IX-relative form of `JP (HL)`) does
;    NOT read a word from memory at IX - it jumps to the address HELD
;    IN the register IX. The parentheses denote the register, not an
;    indirect read. So with ix,1, IX = SP+1 and `jp (ix)` makes the
;    CPU start executing at SP+1, i.e. at the `C3` opcode, which then
;    jumps to cb. The whole 0xC300 push exists solely to hold that
;    opcode; the `00` is never executed.
;
;    With ix,2, IX = SP+2 and the CPU would start executing at cb_lo
;    instead - for CHPUT that is 0xA2, which is not a JP, so the
;    machine runs off into garbage and hangs. (This was tried, and
;    it produced exactly that freeze.)

PRINT_HL:
	ld	a,(hl)
	or	a
	ret	z
	call	JP_IX
	inc	hl
	jr	PRINT_HL

JP_IX:	jp	(ix)

;--- PRINT_HEXA_DE: print the byte in A as two upper-case hex digits.
;    In: A = byte, DE = char output callback.
;    Trashes AF, IX. Preserves BC, DE, HL, IY - which is what lets the
;    diagnostic in DO_DRVQ_INIT_FAIL hold the index in B, the byte read
;    in C and the byte wanted in D across a whole report.
;
;    Needed because PRINT_HEXBUF_DE needs a buffer to read from and this
;    driver has none (see the note on MB_GETRES).

PRINT_HEXA_DE:
	push	de
	ld	de,0C300h
	push	de
	ld	ix,1
	add	ix,sp
	push	af		;A is the byte; PRINT_HEXNIBBLE clobbers it
	rrca
	rrca
	rrca
	rrca			;high nibble now in bits 0..3
	call	PRINT_HEXNIBBLE
	pop	af
	call	PRINT_HEXNIBBLE	;and 0Fh leaves the low nibble
	pop	de
	pop	de
	ret

;--- PRINT_HEXBUF_DE: print B bytes at (HL) as hex pairs, upper case.
;    In:  HL = buffer, B = count, DE = char output callback
;    Out: -      Trashes AF, BC, DE, HL, IX
;
;    Same trampoline as PRINT_WITH_DE: ix,1 and `jp (ix)` - see the
;    note there for why ix,1 is the correct offset.

PRINT_HEXBUF_DE:
	push	de
	ld	de,0C300h
	push	de
	ld	ix,1
	add	ix,sp
	call	PRINT_HEXBUF_L
	pop	de
	pop	de
	ret

PRINT_HEXBUF_L:
	ld	a,b
	or	a
	ret	z
	push	hl
	ld	a,(hl)
	call	PRINT_HEXBYTE		;needs the trampoline IX
	pop	hl
	inc	hl
	dec	b
	jr	PRINT_HEXBUF_L

;--- Print A as two hex digits, upper case, via the trampoline IX.
;    Trashes AF. Relies on the callback clobbering AF only.

PRINT_HEXBYTE:
	push	af
	rrca
	rrca
	rrca
	rrca			;A = high nibble in bits 0..3
	call	PRINT_HEXNIBBLE
	pop	af
PRINT_HEXNIBBLE:
	and	0Fh
	add	a,'0'
	cp	'9'+1
	jr	c,PRINT_HEX_EMIT
	add	a,7			;':' - 10
PRINT_HEX_EMIT:
	call	JP_IX
	ret

	;Pad the ROM driver up to the bank switching code area at 7FD0h
	;(assembly fails if the driver outgrows the bank).

	ds	7FD0h-$,0FFh

	end
