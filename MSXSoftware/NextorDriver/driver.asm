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
;   0x01 CAPACITY    -> 8 bytes: block count LE(4) + block size LE(4)
;   0x02 STATUS      -> 1 byte: 0 no media / 1 ready / 2 media changed
;                               (CONSUMES the firmware's change latch)
;   0x03 READ        <- LBA LE(4)  -> 512 bytes sector data
;   0x04 WRITE       <- LBA LE(4) + 512 bytes -> DONE after SCSI WRITE(10)
;   0x05 ABORT       -> nothing
;   0x06 STAPEEK     -> 1 byte: like STATUS but does NOT consume the
;                               change latch (device availability query)
;
; Device model (docs/NEXTOR_PLAN.md D4): device 1 = the USB stick as one
; block device (raw sectors, MBR included - the kernel owns partitioning).
; 512-byte sectors, removable medium.
;
; Z80 rules respected:
;   - documented opcodes only (pairs with a .NO_UNDOC. kernel variant);
;   - no use of C000h-C400h as scratch;
;   - no RAM work area requested: the media-change latch lives in the
;     firmware (CMD_STAPEEK reads it without consuming), so query 3
;     returns B=0 flags and HL=0 work area.
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
;    Out: HL advanced past the args. Trashes AF, B.

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

;--- MB_GETRES: pop A result bytes from the DATA port into (HL).
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

;--- MB_STCMD: send zero-arg command A, poll, pop its 1-byte result.
;    Used for STATUS / STAPEEK.
;    Out: A = result byte, Cy=0.  Cy=1 on timeout.
;    Trashes AF, BC, HL.

MB_STCMD:
	ld	hl,MB_NOARGS
	ld	b,0
	call	MB_SEND
	call	MB_POLL
	ret	c
	ld	a,(MBOX_DATA)
	or	a
	ret

;--- MB_STCMD5: send zero-arg command A, pop 8 result bytes into
;    MB_CAPBUF (CAPACITY).
;    Out: Cy=0 on success.  Trashes AF, BC, HL.

MB_STCMD5:
	ld	hl,MB_NOARGS
	ld	b,0
	call	MB_SEND
	call	MB_POLL
	ret	c
	ld	hl,MB_CAPBUF
	ld	a,8
	call	MB_GETRES
	or	a
	ret

;--- MB_HANDSHAKE_CHK: probe the firmware mailbox.
;    Out: Cy=0 = firmware answered with the "RNX2" magic;
;         Cy=1 = no firmware / foreign cart / wrong magic.
;    Trashes AF, BC, DE, HL.

MB_HANDSHAKE_CHK:
	ld	a,MB_HANDSHAKE
	ld	hl,MB_NOARGS
	ld	b,0
	call	MB_SEND
	call	MB_POLL
	jr	c,MB_HS_TO
	ld	hl,MB_HSBUF
	ld	a,5
	call	MB_GETRES
	ld	a,(MB_HSBUF+0)
	cp	MB_MAGIC0
	jr	nz,MB_HS_NO
	ld	a,(MB_HSBUF+1)
	cp	MB_MAGIC1
	jr	nz,MB_HS_NO
	ld	a,(MB_HSBUF+2)
	cp	MB_MAGIC2
	jr	nz,MB_HS_NO
	ld	a,(MB_HSBUF+3)
	cp	MB_MAGIC3
	jr	nz,MB_HS_NO
	or	a		;Cy=0: magic OK
	ret
MB_HS_TO:
	scf
	ret
MB_HS_NO:
	or	1
	scf
	ret

;--- Zero-arg source byte (MB_SEND arg pointer for no-arg commands).

MB_NOARGS:	db	0

;--- Handshake result buffer (5 bytes) + capacity buffer (8 bytes).

MB_HSBUF:	ds	5
MB_CAPBUF:	ds	8

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
	ld	bc,0100h
	ld	d,0
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
	call	MB_HANDSHAKE_CHK
	jp	c,DO_DRVQ_INIT_FAIL
	xor	a		;RESULT_OK
	ld	b,0		;no TIMER_INT / EXTBIO hooks
	ld	hl,0		;no page-3 work area
	ret

DO_DRVQ_INIT_FAIL:
	ld	a,RESULT_INIT_ERROR
	ld	b,0
	ld	hl,0
	ret


; Driver query 4: Initialize driver
;
; Re-verifies the handshake (cheap insurance; the kernel calls queries 3
; and 4 back to back) and prints the banner via the DE print-character
; callback (CHPUT semantics; never the direct BIOS CHPUT).

DO_DRVQ_INIT:
	call	MB_HANDSHAKE_CHK
	jp	c,DO_DRVQ_INIT_FAIL
	ld	hl,INIT_MSG
	call	PRINT_WITH_DE
	xor	a		;RESULT_OK
	ret


; Driver query 5: Get maximum supported device number

DO_DRVQ_GET_MAX_DEVICE:
	ld	a,RESULT_OK
	ld	b,1		;device 1 = the USB stick
	ret


;-----------------------------------------------------------------------------
; Device queries
;-----------------------------------------------------------------------------

;--- DEVICE_QUERY dispatcher.
;    In:  A = query index, C = device number.
;    The device number is validated BEFORE the index is consumed
;    (.IDEVN for unknown devices regardless of the query index).

DEVICE_QUERY:
	push	af		;save the query index
	ld	a,c
	cp	1
	jr	z,DQ_DEV_OK
	pop	af		;discard (keep the stack clean)
	ld	a,RESULT_INVALID_DEVICE
	ret
DQ_DEV_OK:
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


; Device query 1: Get device information string
; In:  B = string index (4 = device name), D = buffer size, HL = buffer
; Out: A = RESULT_OK / RESULT_TRUNCATED_STRING / RESULT_NOT_IMPLEMENTED

DO_DEVQ_GET_STRING:
	ld	a,b
	ld	b,d
	ex	de,hl
	dec	a
	jr	nz,DO_DEVQ_STR_NI
	ld	hl,MSG_DEVICE_NAME
	call	OUTPUT_STRING
	xor	a		;RESULT_OK
	ret
DO_DEVQ_STR_NI:
	ld	a,RESULT_NOT_IMPLEMENTED
	ret


; Device query 2: Get device parameters
;
; In:  HL = buffer address, or 0 for "validate only" (must be supported)
; Out: A = RESULT_OK / RESULT_NOT_IMPLEMENTED
;
; Buffer layout (12 bytes):
;   +0 (1)  device type: 0 = block device
;   +1 (2)  sector size LE: 512
;   +3 (4)  total sectors LE (from firmware SCSI READ CAPACITY;
;           0 when the firmware has no medium / no capacity yet)
;   +7 (1)  flags: bit0 = removable; bit3 = 0 (automapping allowed)
;   +8..11  cylinders(2)/heads(1)/sectors-per-track(1) = 0

DO_DEVQ_GET_PARAMS:
	ld	a,h
	or	l
	jr	nz,DQP_FILL
	xor	a		;HL=0: validate only -> RESULT_OK
	ret
DQP_FILL:
	push	hl		;save the caller's buffer pointer
	ld	a,MB_CAPACITY
	call	MB_STCMD5
	pop	hl
	jr	c,DQP_TMO
	;Fill the 12-byte parameter buffer.
	ld	(hl),0		;type: block device
	inc	hl
	ld	(hl),00h	;512 LE low
	inc	hl
	ld	(hl),02h	;512 LE high
	inc	hl
	ld	a,(MB_CAPBUF+0)
	ld	(hl),a
	inc	hl
	ld	a,(MB_CAPBUF+1)
	ld	(hl),a
	inc	hl
	ld	a,(MB_CAPBUF+2)
	ld	(hl),a
	inc	hl
	ld	a,(MB_CAPBUF+3)
	ld	(hl),a
	inc	hl
	ld	(hl),01h	;flags: removable
	inc	hl
	xor	a
	ld	b,4		;cylinders(2)+heads(1)+sectors-per-track(1)=0
DQP_ZERO:
	ld	(hl),a
	inc	hl
	djnz	DQP_ZERO
	xor	a		;RESULT_OK
	ret
DQP_TMO:
	ld	a,RESULT_NOT_IMPLEMENTED
	ret


; Device query 3: Get device status
; Out: A = RESULT_OK, B = 0 no media / 1 ready / 2 media changed.
; Consumes the firmware's change latch (this query is what the
; "changed once, then ready" tracking is built on).

DO_DEVQ_GET_STATUS:
	ld	a,MB_STATUS
	call	MB_STCMD
	jr	c,DQ_STAT_TMO
	ld	b,a
	ld	a,RESULT_OK
	ret
DQ_STAT_TMO:
	ld	a,RESULT_NOT_IMPLEMENTED
	ld	b,1
	ret


; Device query 4: Get device availability
; Out: A = RESULT_OK, B = 0 not available / 1 available.
; Must NOT consume the change latch -> uses CMD_STAPEEK; a "changed"
; result still counts as "available".

DO_DEVQ_GET_AVAILABILITY:
	ld	a,MB_STAPEEK
	call	MB_STCMD
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


; Device queries 5-7: floppy-only; we are not a floppy.

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
;         A  = device number (must be 1)
;         B  = sector count
;         C  = media descriptor (ignored: not a floppy)
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
;    Per sector: media check -> CMD_READ/CMD_WRITE + LBA(4) ->
;    (read: drain 512 from DATA / write: push 512 to DATA) -> poll DONE
;    -> map errors -> increment the 32-bit LBA at (DE').
;    On error: B = sectors transferred = original count - remaining.

READ_WRITE:
	push	af
	push	bc
	push	de
	push	hl
	push	ix
	push	iy
	ld	iy,0
	add	iy,sp		;frame: (iy+8..9)=BC, (iy+10..11)=AF

	;Device number must be 1 (.IDEVN otherwise).

	ld	a,(iy+11)	;original A = device number
	cp	1
	jr	nz,RW_IDEVN

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

	ld	a,(iy+10)	;saved F
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
	ld	(iy+11),a	;becomes the returned A at exit
	ld	b,(iy+9)	;B = all sectors transferred
	jr	RW_EXIT

	;--- Error exit: A = error code (from the helper), B = remaining
	;    count (the djnz has not run yet for the failed sector, so
	;    remaining is intact). Done = original count - remaining.
	;    The error code is written into the saved-A slot so it
	;    survives the register restore below.

RW_ERROR:
	push	af		;stash the error code
	ld	a,(iy+9)	;original B = total requested
	sub	b		;done = total - remaining
	ld	b,a
	pop	af		;recover the error code
	ld	(iy+11),a	;it becomes the returned A
	jr	RW_EXIT

RW_EXIT:
	pop	iy
	pop	ix
	pop	hl
	pop	de
	pop	bc
	pop	af		;A = error code, B = sectors transferred
	ret

RW_IDEVN:
	ld	(iy+11),a	;.IDEVN into the saved-A slot
	xor	a
	ld	b,a		;B = 0 sectors transferred
	jr	RW_EXIT


;--- RW_ONE_READ: transfer one sector (read).
;    IX = buffer, DE' = LBA pointer, B = remaining (preserved).
;    Returns A = 0 or a DOS error code. Trashes AF, BC, DE, HL.

RW_ONE_READ:
	push	bc		;preserve the remaining-count register

	;Media check (consumes the change latch; that is fine - the
	;kernel calls status query 3 before any access anyway).

	ld	a,MB_STATUS
	call	MB_STCMD
	jp	c,RW1_TMO
	or	a
	jp	z,RW1_NRDY

	;Send CMD_READ + the 4 LBA bytes (copied from (DE') via HL').

	exx
	push	de
	pop	hl		;HL' = copy of the LBA pointer
	ld	a,MB_READ
	ld	(MBOX_CMD),a
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
;    Same register contract as RW_ONE_READ.

RW_ONE_WRITE:
	push	bc		;preserve the remaining-count register

	ld	a,MB_STATUS
	call	MB_STCMD
	jp	c,RW1_TMO
	or	a
	jp	z,RW1_NRDY

	;Send CMD_WRITE + the 4 LBA bytes.

	exx
	push	de
	pop	hl
	ld	a,MB_WRITE
	ld	(MBOX_CMD),a
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
	exx

	;Push 512 bytes from (IX) to DATA.

	ld	bc,512
RW1_WDR:
	ld	a,(ix+0)
	inc	ix
	ld	(MBOX_DATA),a
	dec	bc
	ld	a,b
	or	c
	jr	nz,RW1_WDR

	;Wait for the firmware to complete the SCSI write.

	call	MB_POLL
	jp	c,RW1_TMO
	ld	a,(MBOX_STAT)
	bit	5,a
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

;-----------------------------------------------------------------------------
; Strings and messages
;-----------------------------------------------------------------------------

	.stresc on

MSG_DRIVER_NAME:	db	"RISKYMSX2 USB",0
MSG_DEVICE_NAME:	db	"USB storage",0

INIT_MSG:		db	"\r\nRISKYMSX2 USB driver\r\n"
			db	"USB stick as block device\r\n",0

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
	add	ix,sp		;IX -> JP <charout> trampoline on stack
	call	PRINT_HL
	pop	de
	pop	de
	ret

PRINT_HL:
	ld	a,(hl)
	or	a
	ret	z
	call	JP_IX
	inc	hl
	jr	PRINT_HL

JP_IX:	jp	(ix)

	;Pad the ROM driver up to the bank switching code area at 7FD0h
	;(assembly fails if the driver outgrows the bank).

	ds	7FD0h-$,0FFh

	end
