/********************************** (C) COPYRIGHT *******************************
 * File Name          : loader.h
 * Description        : ROM-loader command engine for the RISKYMSX2.
 *
 *                      The MSX-side loader (MSXSoftware/RomLoader)
 *                      runs from MSX RAM and talks to the firmware
 *                      through the mailbox protocol decoded by
 *                      Cart_EXTI0_Loader_Handler (cart.c).  This
 *                      module implements the COMMAND side: directory
 *                      listing, ROM load into PSRAM, mapper select and
 *                      MSX reset.  Loader_Service() must be called
 *                      from the main loop so pending mailbox commands
 *                      drain without blocking the cart IRQ.
 *********************************************************************************/

#ifndef __LOADER_H
#define __LOADER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Mailbox command bytes (MUST match romloader.c). */
#define LOADER_CMD_DIR_OPEN    0x00U
#define LOADER_CMD_DIR_READ   0x01U
#define LOADER_CMD_DIR_CLOSE  0x02U
#define LOADER_CMD_LOAD_ROM   0x03U
#define LOADER_CMD_SET_MAPPER 0x04U
/* 0x05 was LOADER_CMD_RESET (cart-driven ~RESET pulse). Removed:
 * driving ~RESET can damage MSXs whose cart-edge reset line is not
 * designed to be sinked by an external device. Boot path now uses
 * LOADER_CMD_SOFTRESET exclusively. The slot is intentionally left
 * empty rather than reused, so old firmware images on a stick don't
 * silently invoke a half-defined command. */
#define LOADER_CMD_BOOT_NEXTOR 0x06U  /* reserved (NEXTOR_PLAN §D2b) */
#define LOADER_CMD_SOFTRESET  0x07U  /* RAM-resident slingshot reboot (no
                                      * ~RESET drive; safe on MSXs whose
                                      * reset line is read-only). */

/* Mailbox status bits (MUST match romloader.c). */
#define LOADER_ST_READY       0x80U
#define LOADER_ST_DONE        0x40U

/* Mailbox FIFO depth.  The longest command is CMD_LOAD_ROM with an
 * 11-byte filename; the longest result is also 11 bytes (a filename).
 * 16 is comfortable headroom. */
#define LOADER_FIFO_DEPTH     16U

/* Max .ROM files the loader will list.  The MSX UI shows a scrolling
 * window; this only bounds the directory scan. */
#define LOADER_MAX_FILES     128U

/* Filename storage per file. The MSX-side loader pushes exactly this
 * many bytes as LOAD_ROM arguments and expects to receive this many
 * bytes per file from DIR_READ. The on-disk 8.3 SFN includes a
 * "~N" tilde-tail when the original LFN was longer than 8 chars (e.g.
 * "Knightmare.rom" -> "KNIGHT~1.ROM", 12 chars), so 12 is the minimum
 * wire size that fits every FAT volume's short name. */
#define LOADER_NAME_LEN       12U

/* Interrupt-side mailbox state (written by the cart IRQ handler in
 * cart.c, drained by Loader_Service in the main loop). */
typedef struct {
    volatile uint8_t  cmd;               /* last command byte        */
    volatile uint8_t  arg_n;             /* arg bytes still expected */
    volatile uint8_t  args[LOADER_FIFO_DEPTH];
    volatile uint8_t  have_cmd;           /* a full command is ready */
    volatile uint8_t  res_n;              /* result bytes pending    */
    volatile uint8_t  res_head;          /* pop index               */
    volatile uint8_t  res_tail;          /* push index              */
    volatile uint8_t  res[LOADER_FIFO_DEPTH];
    volatile uint8_t  status;            /* READY|DONE bits         */
} Loader_Mailbox;

/* The single mailbox instance.  cart.c's loader handler writes it from
 * EXTI0 context; loader.c drains it from the main loop. */
extern Loader_Mailbox g_loader_mbox;

/* Reset the mailbox state (called when the LOADER mapper installs). */
void Loader_Reset (void);

/* Main-loop service: if the mailbox has a complete command, execute
 * it (directory ops, file load, mapper switch, reset).  Never blocks
 * longer than one sector transfer; call it every loop pass. */
void Loader_Service (void);

#ifdef __cplusplus
}
#endif

#endif /* __LOADER_H */