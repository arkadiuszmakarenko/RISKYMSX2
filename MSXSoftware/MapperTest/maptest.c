/*
 * maptest.c - USB .ROM listing + mapper selection boot menu.
 *
 * On boot, asks the firmware for the .ROM file list on the USB stick
 * (CMD_DIR_OPEN + CMD_DIR_READ) and shows the first 10 entries. The
 * user picks one by number; the MSX sends CMD_LOAD_ROM with that
 * filename, then picks a mapper (1..9, or 0 for FLASH menu).
 *
 * Boot contract matches hello.bin / fixed selector ROM:
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

#define CMD_SET_MAPPER 0x04U
#define CMD_RESET      0x05U
#define CMD_LOAD_ROM   0x03U
#define CMD_DIR_OPEN   0x00U
#define CMD_DIR_READ   0x01U

#define MAX_DISPLAY 10U

/* Mapper indices — MUST match Cart_Mapper enum in firmware/User/cart.h */
static const char *const mapper_names[] = {
    "ROM16K",      /* 1 */
    "ROM32K",      /* 2 */
    "ROM48K",      /* 3 */
    "KONAMI",      /* 4 */
    "KONAMINOSCC", /* 5 */
    "ASCII8K",     /* 6 */
    "ASCII16K",    /* 7 */
    "NEO8",        /* 8 */
    "NEO16",       /* 9 */
    "KONAMISCC",   /* 10 */
    "FLASH",       /* 11 - boot cart image from flash */
};

void putchar_msx(char c) __z88dk_fastcall
{
	c;
	__asm
		push	ix
		push	iy
		ex	af, af'
		ld	a, l
		ld	iy, #0xFCC0
		ld	ix, #0x00A2
		call	0x001C
		ex	af, af'
		pop	iy
		pop	ix
	__endasm;
}

char wait_key(void) __naked
{
	__asm
		ld	iy, #0xFCC0
		ld	ix, #0x009F
		call	0x001C
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

static void mbox_wait_done(void)
{
	while ((*MBOX_STATUS & ST_DONE) == 0U) { }
}

static uint32_t mbox_load_rom(const uint8_t *name83)
{
	*MBOX_CMD = CMD_LOAD_ROM;
	for (uint8_t i = 0; i < 11; i++) {
		*MBOX_CMD = name83[i];
	}
	mbox_wait_done();
	uint32_t total = 0;
	for (uint8_t i = 0; i < 4; i++) {
		total = (total << 8) | (uint32_t)*MBOX_DATA;
	}
	return total;
}

static void mbox_cmd_set_mapper(uint8_t idx)
{
	*MBOX_CMD = CMD_SET_MAPPER;
	*MBOX_CMD = idx;
	mbox_wait_done();
}

/* Ask the firmware for the .ROM file list. Returns the count.
 * Fills out_names (up to max_out entries) with 11-byte 8.3 names. */
static uint16_t mbox_dir_list(uint8_t *out_names, uint16_t max_out)
{
	*MBOX_CMD = CMD_DIR_OPEN;
	mbox_wait_done();
	uint16_t count = (uint16_t)((uint16_t)*MBOX_DATA
	                          | ((uint16_t)*MBOX_DATA << 8));
	for (uint16_t i = 0; i < count && i < max_out; i++) {
		*MBOX_CMD = CMD_DIR_READ;
		mbox_wait_done();
		for (uint8_t j = 0; j < 11; j++) {
			out_names[i * 11 + j] = *MBOX_DATA;
		}
	}
	return count;
}

static void print_name(const uint8_t *name83)
{
	for (uint8_t i = 0; i < 8 && name83[i] != ' '; i++) {
		putchar_msx((char)name83[i]);
	}
	if (name83[8] != ' ') {
		putchar_msx('.');
		for (uint8_t i = 8; i < 11 && name83[i] != ' '; i++) {
			putchar_msx((char)name83[i]);
		}
	}
}

int main(void)
{
	print("USB .ROM LOADER\r\n\r\n");
	print("scanning USB...\r\n");

	uint8_t names[MAX_DISPLAY * 11];
	uint16_t count = mbox_dir_list(names, MAX_DISPLAY);
	print("found ");
	if (count >= 100) putchar_msx('0' + (char)(count / 100));
	if (count >= 10)  putchar_msx('0' + (char)((count / 10) % 10));
	putchar_msx('0' + (char)(count % 10));
	print(" .ROM files\r\n\r\n");

	for (;;) {
		print("0 = FLASH menu\r\n");
		for (uint16_t i = 0; i < count && i < MAX_DISPLAY; i++) {
			putchar_msx('1' + (char)i);
			print(": ");
			print_name(names + i * 11);
			print("\r\n");
		}
		if (count > MAX_DISPLAY) {
			print("(more on USB)\r\n");
		}
		print("\r\npick a number: ");

		char k = wait_key();
		putchar_msx(k);
		print("\r\n");

		if (k == '0') {
			mbox_cmd_set_mapper(11);
			*MBOX_CMD = CMD_RESET;
			for (;;) { }
		}

		uint8_t file_idx;
		if (k >= '1' && k <= '9') file_idx = (uint8_t)(k - '1');
		else { print(" ?\r\n"); continue; }
		if (file_idx >= count) { print(" out of range\r\n"); continue; }

		print("LOAD_ROM ");
		print_name(names + file_idx * 11);
		print(" ...\r\n");

		uint32_t total = mbox_load_rom(names + file_idx * 11);
		print("loaded ");
		if (total == 0) {
			print("0 bytes (USB error)\r\n");
			continue;
		}
		print_hex((uint8_t)(total >> 24));
		print_hex((uint8_t)(total >> 16));
		print_hex((uint8_t)(total >> 8));
		print_hex((uint8_t)total);
		print(" bytes\r\n\r\n");

		print("pick a mapper (default 2=ROM32K for plain .ROM):\r\n");
		print(" 1=ROM16K 2=ROM32K 3=ROM48K\r\n");
		print(" 4=KONAMI 5=KONAMINOSCC 6=ASCII8K\r\n");
		print(" 7=ASCII16K 8=NEO8 9=NEO16 0=FLASH\r\n");
		print("mapper (ENTER=2): ");

		char m = wait_key();
		if (m == '\r') {
			m = '2';
			print("(default)\r\n");
		} else {
			putchar_msx(m);
			print("\r\n");
		}

		uint8_t midx = 0;
		if (m >= '1' && m <= '9') midx = (uint8_t)(m - '0');
		else if (m == '0') midx = 11;
		else { print(" ?\r\n"); continue; }

		print("SET_MAPPER ");
		print(mapper_names[midx - 1]);
		print("\r\nRESET...\r\n");

		mbox_cmd_set_mapper(midx);
		*MBOX_CMD = CMD_RESET;

		for (;;) { }
	}
}
