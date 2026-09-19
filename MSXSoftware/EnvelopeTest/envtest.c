/*
 * envtest.c - mailbox envelope self-test cartridge.
 *
 * Boots, prints a header, then polls the firmware mailbox at
 * cart 0x7FF0.  On every keypress it writes 0xFE (an undefined
 * command) to the mailbox and waits for the firmware to ack with
 * ST_DONE.  We deliberately avoid CMD_RESET (0x05) because that
 * triggers a firmware-side MSX reset pulse, which would reboot us
 * before we can observe the ack.
 *
 * Purpose: prove the shared envelope round-trips between the MSX
 * loader and the firmware before we turn on USB or bank switching.
 *
 * Boot contract matches hello.bin and the fixed selector ROM:
 *   0x4000  "AB" header, INIT -> 0x4010
 *   0x4010  DI / CALL _main / DI / HALT / RET
 *   0x4020  C code starts here
 */

#include <stdint.h>

#define MBOX_CMD    ((volatile uint8_t *)0x7FF0U)
#define MBOX_STATUS ((volatile uint8_t *)0x7FF0U)
#define MBOX_DATA   ((volatile uint8_t *)0x7FF1U)

#define ST_READY 0x80U
#define ST_DONE  0x40U

void putchar_msx(char c) __z88dk_fastcall
{
	c; /* passed in L */
	__asm
		push	ix
		push	iy
		ex	af, af'
		ld	a, l
		ld	iy, #0xFCC0
		ld	ix, #0x00A2		; CHPUT
		call	0x001C			; CALSLT
		ex	af, af'
		pop	iy
		pop	ix
	__endasm;
}

char wait_key(void) __naked
{
	__asm
		ld	iy, #0xFCC0
		ld	ix, #0x009F		; CHGET
		call	0x001C			; CALSLT
		ld	l, a
		ret
	__endasm;
}

static void print(const char *s)
{
	while (*s)
		putchar_msx(*s++);
}

static void print_hex(uint8_t v)
{
	const char hex[] = "0123456789ABCDEF";
	putchar_msx(hex[(v >> 4) & 0x0FU]);
	putchar_msx(hex[v & 0x0FU]);
}

int main(void)
{
	print("ENVELOPE TEST\r\n");
	print("press a key to send 0xFE (undefined cmd)\r\n");
	print("to the firmware mailbox at 0x7FF0.\r\n\r\n");

	/* Diagnostic: read the cart header byte at 0x4000 to confirm
	 * the cart slot is still expanded in page 1 when _main runs.
	 * If this reads 0x41 ('A'), the slot is expanded. If it reads
	 * something else, the BIOS un-expanded the slot before calling
	 * INIT, which would explain why 0x7FF0 reads from BIOS ROM
	 * instead of the cart mailbox. */
	{
		uint8_t hdr = *((volatile uint8_t *)0x4000U);
		print("cart hdr @4000=");
		print_hex(hdr);
		print("\r\n");
	}
	{
		uint8_t hdr2 = *((volatile uint8_t *)0x4001U);
		print("cart hdr @4001=");
		print_hex(hdr2);
		print("\r\n");
	}
	{
		uint8_t hdr3 = *((volatile uint8_t *)0x4010U);
		print("cart @4010=");
		print_hex(hdr3);
		print("\r\n");
	}

	for (;;) {
		/* Show the current mailbox status. */
		uint8_t st = *MBOX_STATUS;
		print("status=");
		print_hex(st);
		print("  ready=");
		print((st & ST_READY) ? "Y" : "N");
		print("  done=");
		print((st & ST_DONE) ? "Y" : "N");
		print("\r\n");

		print("press any key...\r\n");
		wait_key();

		/* Send an undefined command. The firmware's loader handler
		 * latches it into g_loader_mbox.cmd; Loader_Service hits
		 * the default case in the switch and just sets ST_DONE.
		 * No MSX reset, so we get to observe the ack. */
		*MBOX_CMD = 0xFEU;
		print("wrote 0xFE\r\n");

		/* Simple poll: read status a few times and print each one.
		 * If the envelope round-trips, DONE will toggle on then off
		 * then on again as the handler clears it and Loader_Service
		 * sets it. Even a single read after the write should show
		 * READY (bit 7) set. */
		for (uint8_t i = 0; i < 5; i++) {
			uint8_t s = *MBOX_STATUS;
			print("post ");
			print_hex(i);
			print(" status=");
			print_hex(s);
			print("\r\n");
		}
		print("\r\n");
	}
}
