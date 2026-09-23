# RISKYMSX2 — Nextor Sunrise IDE user guide (after the kernel boots)

## You're already there

If the MSX rebooted into MSX-BASIC after you typed `CALL SYSTEM`,
that's expected Nextor behavior, not a bug. `CALL SYSTEM` is the
documented command that takes you back to MSX-BASIC. The kernel is
still in flash; rebooting just drops back to BASIC while keeping the
cart slot mapped to the kernel ROM (it'll reappear the next time the
MSX probes that slot).

If instead the MSX restarted into the kernel banner (the
`Master device: ... / initialization OK / ...` sequence), then
`CALL SYSTEM` returned to the MSX-DOS prompt — also normal.

Both outcomes mean the boot worked.

## How to access the USB drive from MSX-BASIC

From the MSX-BASIC prompt:

```
CALL SYSTEM
```

This re-enters MSX-DOS (or the COMMAND.COM / MSX-DOS prompt if a
shell is loaded from the USB stick).

From the MSX-DOS prompt:

```
A:
DIR
```

`A:` selects the first drive. The kernel binds the USB stick as `A:`.
`DIR` lists the root directory of the USB stick.

If the stick isn't listed, type:

```
MAP
```

to print the slot-to-drive mapping. The USB stick should show up as
`A:` (slot 3 primary, drive 0).

## How to get back to the terminal menu from MSX-DOS

From the MSX-DOS prompt:

```
SET SLTNOEXP(0)=3
```

or just press `CTRL-STOP` repeatedly — the BIOS returns to BASIC.
Then reboot (F1 if you have a keyboard buffer, otherwise power cycle).
The terminal menu stays armed across reboots because the boot-time
mapper is CART_MAP_TERMINAL.

## How to go straight to MSX-DOS after power-on

If you don't want the terminal menu, just hold down a key during the
MSX boot — the menu forwards keystrokes to the kernel if the user
picks `N` from the file list. Without any input the menu keeps
showing the file list until you type something.

## Common commands cheat sheet (MSX-DOS 2 / Nextor)

| Command  | Effect                                    |
|----------|-------------------------------------------|
| `DIR`    | List files                                |
| `DIR /W` | List files in wide columns                |
| `CD x:`  | Change current drive (e.g. `CD A:`)       |
| `TYPE f` | Print file contents                       |
| `COPY a b` | Copy file                              |
| `FORMAT A:` | Format the USB stick (FAT12/16 only)   |
| `CHDIR \SUBDIR` | Enter a subdirectory                |
| `BASIC`  | Drop back to MSX-BASIC                    |
| `CALL SYSTEM` | Soft-reboot back to MSX-BASIC       |

## What if the USB stick doesn't show up

1. Type `MAP` and check that `A:` is mapped to a slot.
2. Type `MAP /U` for the unmapped view, or `MAP /R` to refresh.
3. If `A:` is missing, the USB stick wasn't enumerated by the time
   the kernel probed. Power-cycle the MSX with the stick already
   inserted, or insert the stick and type `MAP /R` after a few
   seconds.
4. If the stick is "changed" but not "ready", try `MAP /R` again or
   pull and re-insert.

## What if `CALL SYSTEM` really did break the boot

If after `CALL SYSTEM` the MSX hangs (no prompt, no banner), then the
`Sunrise_IDE_Service` block did not catch up after the kernel
soft-reset. The fix is to:

1. Hold `F1` (or power-cycle) — the BIOS re-probes the cart, which
   re-arms the Sunrise IDE handler.
2. If the hang persists, check `USBH_PreDeal` is being called. The
   USB host state machine is gated on the SUNRIDE mapper; if it was
   never installed, no USB work happens.
3. Watch the firmware USART log for `SUNRIDE:` lines. If no log line
   appears within 5 seconds of boot, the USB host stack failed to
   enumerate.