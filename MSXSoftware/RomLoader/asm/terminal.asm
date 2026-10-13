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
;                        0x00 = "boot gate: GRAPH held -> terminal menu"
;                        0x01 = "boot gate: GRAPH not held -> boot Nextor"
;                        0x04 = legacy v303 "user held GRPH" (ignored)
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
;   1. INIT probes the GRAPH key (SNSMAT row 6, bit 2) and tells the
;      firmware which path to take on 0x7FFE BEFORE it starts polling:
;        - GRAPH held     -> 0x00: full terminal menu.
;        - GRAPH not held -> 0x01: boot Nextor directly.
;      The firmware gates its first menu render on this byte, so a
;      no-GRAPH power-on goes straight to Nextor with no menu flash.
;   2. INIT then copies the post-init body to RAM at 0xE000. The body
;      length is a fixed constant (BODY_SIZE bytes).
;   3. Set 32-column screen, palette, sprite cursor, then loop:
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
        ;; Boot gate. Probe the GRAPH key via the BIOS SNSMAT matrix
        ;; reader (entry 0x0141 - it is SNSMAT, not GTSIZE). Row 6,
        ;; bit 2, active-low: 0 = held. Tell the firmware the decision
        ;; on 0x7FFE BEFORE copying/running the body, so the firmware
        ;; main loop can skip the menu render and go straight to
        ;; Nextor on a no-GRAPH power-on.
        ld      a, #6
        call    0x0141                  ; SNSMAT - A = row, returns matrix
        and     #0x04                   ; GRAPH = row 6 bit 2, 0 = held
        jr      z, bg_terminal          ; held -> terminal menu
        ;; GRAPH not held -> ask the firmware to boot Nextor.
        ld      a, #0x01
        ld      (0x7FFE), a
        jr      bg_run
bg_terminal:
        ;; GRAPH held -> full terminal menu.
        ld      a, #0x00
        ld      (0x7FFE), a
bg_run:
        ld      hl, #body              ; source in ROM
        ld      de, #0xE000            ; destination in MSX RAM
        ld      bc, #BODY_SIZE         ; byte count
        ldir
        jp      0xE000                 ; jump into the relocated copy

;-------------------------------------------------------------------------------
; Body to copy to RAM. Same shape as v303: screen setup,
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
        ;; The GRAPH boot gate ran in init() (see above); by the time
        ;; control reaches here the firmware already knows whether to
        ;; render the menu or boot Nextor. Just initialise the screen +
        ;; cursor and enter the main loop.

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

        ;; Disable the BIOS function-key display (equivalent to
        ;; BASIC's `KEY OFF`). In text screen modes the BIOS draws
        ;; the default F1-F10 macros ("color", "auto", "goto",
        ;; "list", "run", ...) on the bottom screen line by default,
        ;; and redraws them on screen events - that line lands under
        ;; the terminal footer, so the macros keep reappearing over
        ;; the menu. CNSDFG (0xF3DE) = 0 means "hidden" (see
        ;; Nextor's msx_workarea.h); FNKSB (0x00C9) erases the
        ;; currently displayed line. Must run AFTER CHGMOD, which
        ;; is what draws it in the first place.
        ld      a, #0
        ld      (0xF3DE), a
        call    0x00C9                  ; FNKSB - erase the key display

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
        ;; The firmware pushes 0x04, X then Y as three separate FIFO
        ;; writes. An EXTI0 can therefore hand us the 0x04 before X/Y
        ;; are queued; reading 0x7FFF while empty returns 0x00, which
        ;; would be taken as a real coordinate and shift the cursor off
        ;; the text stream (stray cursor jumps / partial screens).
        ;; Both coordinates are never 0 (col*8+1 and row*8+1), so spin
        ;; until a nonzero byte arrives before consuming it.
spr_x:
        ld      a, (0x7FFF)             ; X
        and     a
        jr      z, spr_x
        ld      hl, #0x1B01
        call    0x004D
spr_y:
        ld      a, (0x7FFF)             ; Y
        and     a
        jr      z, spr_y
        dec     a
        dec     l
        call    0x004D
        jr      loop

rom_start:
        ;; >= ~35 ms for the firmware to settle: wait two JIFFY ticks.
        ;; FULLY INLINED and executed from MSX RAM - no cart access from
        ;; here on. The firmware swaps the mapper ~20 ms after pushing
        ;; the 0x03, i.e. DURING this wait; a `call wait` would fetch the
        ;; swapped-in cart's bytes as code (wait lives at a cart-window
        ;; address, and the swap lands mid-wait-loop - the Z80 then
        ;; executes garbage and the MSX hangs: exactly the bug the inline
        ;; exists to prevent).
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
        ;; Reboot: plain BIOS-standard RST 0 cold start. The target
        ;; mapper is already installed (the swap landed during the wait
        ;; above), so the BIOS slot probe finds the new ROM's 'AB'.
        ;; This replaces the v303 BIOS stack-layout patch (compare the
        ;; INIT return against 0x7DA3, patch the frame to the 0xE100
        ;; stub, jp 0x7D84) - those are BIOS-version-specific addresses,
        ;; and RST 0 is equivalent because the hard-reset test proved a
        ;; plain cold boot works on this machine.
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
; Cursor arrow sprite pattern - emitted immediately after the body. It used to
; sit at a hard-coded .org 0x40F0; when the body grew past that address the
; assembler overwrote the trailing `ret` of `wait`, hanging the body (no keys,
; no boot). Letting it follow body_end keeps the two from ever colliding; it
; still lands inside the 256-byte trimmed cartridge window.
;-------------------------------------------------------------------------------
arrow:
        .db     0b00100000
        .db     0b00110000
        .db     0b00111000
        .db     0b00111100
        .db     0b00111000
        .db     0b00110000
        .db     0b00100000
        .db     0b00000000
