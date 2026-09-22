;-------------------------------------------------------------------------------
; terminal.asm - MSX-side terminal ROM for RISKYMSX2 (CH32V407).
;
; Ported from v303firmwaer/MSXTerminal/msxterminal.asm (sjasmplus syntax) to
; sdasz80 syntax so it builds with the existing MSXSoftware/RomLoader
; toolchain (sdcc + sdasz80 + makebin).
;
; Layout, mailbox semantics, and screen I/O are byte-for-byte identical to
; v303 (the firmware side expects the same protocol - see cart.c
; Cart_EXTI0_Terminal_Handler).
;
; Mailbox at the cart edge:
;
;   0x7FFD  WRITE   - keyboard byte (firmware pops the byte, decides key)
;   0x7FFE  WRITE   - control byte
;                        0x00 = "continue / terminal menu"
;                        0x04 = "user held GRPH - skip the terminal, boot
;                                the cart image straight away"
;   0x7FFF  READ    - firmware output FIFO (CHAR by CHAR; 0 = empty, the
;                     byte itself is the printable character; 0x03 = "user
;                     asked to abort / soft-reset the MSX into ROM");
;                     0x04 = "move the menu cursor" (followed by reading
;                     X-then-Y bytes).
;
; For RISKYMSX2 the firmware also recognises a few extra control codes that
; the v303 firmware did not need:
;
;   0x7FF0..0x7FF7  READ  - "USB stick presence" status (1 byte = "stick
;                           present", 0 = "no stick yet")
;
; Boot flow:
;   1. INIT moves the post-init body to RAM at 0xE000. The body length is
;      a fixed constant (BODY_SIZE = 213 bytes, matching the v303 ROM).
;   2. INIT probes GRPH-key. If GRPH is held, send 0x04 on 0x7FFE -> the
;      firmware swaps to the user cart and reboots the MSX into the game.
;   3. Otherwise, set 32-column screen, palette, sprite cursor, then loop:
;      read 0x7FFF, print whatever the firmware puts there, poll keyboard
;      and forward each keystroke on 0x7FFD.
;-------------------------------------------------------------------------------

        .module terminal

;-------------------------------------------------------------------------------
; Empty _DATA area. sdcc's linker emits a spurious
; "No definition of area _DATA" warning (which it returns exit code 1
; for) when a standalone asm binary has no .data section. Declaring an
; empty area here silences both the warning and the exit code without
; affecting the binary output.
;-------------------------------------------------------------------------------
        .area _DATA
        .ds 1

;-------------------------------------------------------------------------------
; ROM header - "AB" + INIT pointer, then ID/version strings.
;-------------------------------------------------------------------------------
        .area   _HEADER (ABS)
        .org    0x4000

        ;; --- MSX ROM header (16 bytes) ---
        .db     0x41, 0x42              ; 'A' 'B' cartridge ID
        .dw     init                    ; INIT: auto-start entry point
        .dw     0x0000                  ; STATEMENT
        .dw     0x0000                  ; DEVICE
        .dw     0x0000                  ; TEXT
        .dw     0x0000, 0x0000, 0x0000  ; reserved

        ;; --- INIT body at 0x4010 ---
        ;; The MSX BIOS calls init() during ROM probing. init() copies
        ;; a fixed-size body (BODY_SIZE bytes) from ROM to MSX RAM at
        ;; 0xE000, then JP into RAM. Same trick as v303 - the cart may
        ;; swap itself out for the user's cart image before we are done
        ;; printing, so we run from RAM.
        .org    0x4010
init:
        ;; Probe GRPH key. If held, return to the BIOS so the MSX
        ;; drops to BASIC / old loader (no terminal menu visible).
        ;; If NOT held, copy the body to MSX RAM and enter the
        ;; terminal menu.
        ld      a, #6
        call    0x0141                  ; GTSIZE - row 6 of matrix
        and     #0x04                   ; GRPH = bit 2 of row 6
        ret     nz                       ; held -> BIOS continues slot probe
        ;; GRPH not held: continue with body copy + run
        ld      hl, #body              ; source in ROM
        ld      de, #0xE000            ; destination in MSX RAM
        ld      bc, #BODY_SIZE         ; byte count
        ldir
        jp      0xE000                 ; jump into the relocated copy

;-------------------------------------------------------------------------------
; Body to copy to RAM. Same shape as v303: GRPH-probe, screen setup,
; poll-FIFO + forward-keyboard loop, ROM_START path.
;
; IMPORTANT: do NOT use .org here - let the assembler place body right
; after the init code. The .org 0x401D in the previous version collided
; with the trailing byte of the JP 0xE000 instruction (3 bytes ending
; at 0x401D), so body actually got placed at 0x401E while LDIR still
; pointed at 0x401D - that one-byte offset meant the RAM copy started
; with the trailing E0 of the JP, which Z80 decodes as RET PO. RET PO
; sometimes returns to a random stack address, which is exactly the
; "rubbish on screen / doesn't drop to BASIC" symptom. Letting the
; assembler place body sequentially keeps the LDIR source correct.
;-------------------------------------------------------------------------------
body:
        ;; GRPH-key probe is DISABLED: the v303 CALL #141 (GTSIZE)
        ;; behaviour is model-dependent (Panasonic returns GRPH bit,
        ;; some clones always return 0x04). When the firmware treated
        ;; that as "boot the OLD loader", the terminal would silently
        ;; disappear on affected hardware with no recovery path. The
        ;; terminal menu is now always shown; the user picks the OLD
        ;; loader from the menu by pressing F. Just initialise the
        ;; screen + cursor and enter the main loop.

        ;; Relocate the 5-byte launch patch to a FIXED MSX RAM address
        ;; (0xE100). The BIOS executes it later - after the firmware
        ;; has swapped the cart mapper away from this ROM - so a
        ;; ROM-resident patch would fetch game bytes as code. The
        ;; v303 original solved this with PHASE #E000 (all body labels
        ;; resolved to RAM); the sdasz80 port keeps ROM addresses, so
        ;; the copy is explicit. Body RAM base = 0xE000, body ROM
        ;; base = 0x4026, so 0xE100 is well clear of the copy.
        ld      hl, #ugly_patch
        ld      de, #0xE100
        ld      bc, #5
        ldir

        ;; Set 32-column screen, palette, sprite cursor.
        ld      a, #32
        ld      (0xF3AF), a
        ld      hl, #0x000F
        ld      (0xF3EA), hl
        ld      (0xF3E9), hl
        ld      a, #1
        call    0x005F
        ld      hl, #arrow
        ld      de, #0x3800
        ld      bc, #8
        call    0x005C
        ld      a, #8
        ld      hl, #0x1B03
        call    0x004D
        ;; Initialise sprite 0 properly. v303 only wrote 8 to 0x1B03
        ;; (the unused attribute byte on MSX1) and relied on the
        ;; pattern index at 0x1B02 being zero - which is only true
        ;; on cold boot. After other carts or sprites use 0x1B02 the
        ;; arrow shows whatever pattern sits at the stale index.
        ;; Force pattern index = 0 (arrow lives at VRAM 0x3800) and
        ;; start with the sprite hidden (Y = 0) so a stale VRAM Y
        ;; doesn't show junk before the firmware's first 0x04 push.
        ld      a, #0                   ; pattern index 0 = arrow @ 0x3800
        ld      hl, #0x1B02
        call    0x004D
        ld      a, #0                   ; Y = 0 = sprite hidden
        ld      hl, #0x1B00
        call    0x004D
        ld      a, #1                   ; X = 1 (off-screen) - hidden by Y first
        ld      hl, #0x1B01
        call    0x004D

loop:
        ;; Poll firmware FIFO at 0x7FFF.
        ld      a, (0x7FFF)
        cp      #3
        jr      z, rom_start            ; "boot the cart"
        cp      #4
        jr      z, sprite               ; "move the cursor"
        and     a
        call    z, wait                 ; empty -> wait one refresh
        call    nz, 0x00A2              ; non-empty -> CHPUT (BIOS print)
        ;; Poll keyboard and forward on 0x7FFD.
        call    0x009C                  ; CHSNS - any key?
        jr      z, loop
        call    0x009F                  ; CHGET - read key
        ld      (0x7FFD), a
        jr      loop

sprite:
        ld      a, (0x7FFF)             ; X
        ld      hl, #0x1B01
        call    0x004D
        ld      a, (0x7FFF)             ; Y
        dec     a
        dec     l
        call    0x004D
        jr      loop

rom_start:
        ;; >= ~35 ms for the firmware to settle: wait two JIFFY ticks.
        ;; INLINED and executed from MSX RAM - no cart access from
        ;; here on (the firmware swaps the mapper during this window;
        ;; a `call wait` would fetch the swapped-in cart's bytes).
        ei
        ld      hl, #0xFC9E             ; JIFFY
        ld      a, (hl)
rs_w1:
        cp      (hl)
        jr      z, rs_w1                ; jiffy change #1
        ld      a, (hl)
rs_w2:
        cp      (hl)
        jr      z, rs_w2                ; ... and #2
        ld      hl, (0xF6B1)
        ld      de, #-12
        add     hl, de
        push    hl
        ld      e, (hl)
        inc     hl
        ld      d, (hl)
        ld      hl, #0x7DA3
        rst     #0x20                   ; COMPARE HL, DE (BIOS helper)
        pop     hl
        jr      nz, unknown
        ;; Patch the BIOS INIT-return frame to 0xE100 (little-endian
        ;; 00 E1): the launch patch executes from MSX RAM, safe under
        ;; the mapper swap.
        ld      (hl), #0x00
        inc     hl
        ld      (hl), #0xE1
        ret

ugly_patch:
        pop     bc
        pop     hl
        jp      0x7D84

unknown:
        ld      e, #'R'
        call    0xFFB1
        rst     0

wait:
        push    af
        ei
        ld      hl, #0xFC9E             ; JIFFY
        ld      a, (hl)
wait_loop:
        cp      (hl)
        jr      z, wait_loop
        pop     af
        ret
body_end:

BODY_SIZE = body_end - body

;-------------------------------------------------------------------------------
; Cursor arrow sprite pattern - sits just past the relocated body.
;-------------------------------------------------------------------------------
        .org    0x40F0
arrow:
        .db     0b00100000
        .db     0b00110000
        .db     0b00111000
        .db     0b00111100
        .db     0b00111000
        .db     0b00110000
        .db     0b00100000
        .db     0b00000000