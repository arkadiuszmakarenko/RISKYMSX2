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
#define LOADER_CMD_RESET      0x05U

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

/* Filename storage per file: 11 bytes of 8.3 name. */
#define LOADER_NAME_LEN       11U

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