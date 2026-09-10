ENTRY( _start )

__stack_size = 2048;

PROVIDE( _stack_size = __stack_size );


MEMORY
{
	/*
	FLASH + RAM supports the following configuration
	For specific choices, please refer :CH32V407RM.PDF\Table 32-4
	FLASH-576K + RAM-136K
	FLASH-512K + RAM-200K
	*/
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 576K
	RAM (xrw) : ORIGIN = 0x20000000+1024, LENGTH = 136K-1K
/* Dedicated 32 KiB SRAM mirror of hello_rom[]. Lives at the very top
	 * of RAM (above .stack). cartpnt points here at boot.
	 *
	 * Placement strategy:
	 *   - RAM goes from 0x20000400 to 0x20000400 + 0x22000 = 0x20022400.
	 *   - CARTROM occupies the top 32 KiB: 0x20014400 to 0x2001C400.
	 *     Wait, that overlaps RAM. Recompute:
	 *   - RAM end = 0x20022400.
	 *   - CARTROM = top 32 KiB = 0x20014400 .. 0x2001C400, but that's
	 *     OUTSIDE RAM. So CARTROM must be a sub-region of RAM.
	 *   - CARTROM = [0x20014400, 0x2001C400), 32 KiB.
	 *     Wait, that's also outside RAM (0x2001C400 > 0x20022400? No,
	 *     0x2001C400 < 0x20022400 - 0x800 stack).
	 *   - Final: CARTROM = 0x20014400 .. 0x2001C400 (32 KiB),
	 *     .stack lives at 0x20021C00..0x20022400 (top 2 KiB of RAM),
	 *     so there is no overlap. CARTROM sits between .bss (which
	 *     ends somewhere below 0x20014000) and .stack.
	 */
	CARTROM (xrw) : ORIGIN = 0x20014400, LENGTH = 32K
}


SECTIONS
{

	.init :
	{
		_sinit = .;
		. = ALIGN(4);
		KEEP(*(SORT_NONE(.init)))
		. = ALIGN(4);
		_einit = .;
	} >FLASH AT>FLASH

	.text :
	{
		. = ALIGN(4);
		*(.text)
		*(.text.*)
		*(.rodata)
		*(.rodata*)
		*(.gnu.linkonce.t.*)
		. = ALIGN(4);
	} >FLASH AT>FLASH 

	.fini :
	{
		KEEP(*(SORT_NONE(.fini)))
		. = ALIGN(4);
	} >FLASH AT>FLASH

	PROVIDE( _etext = . );
	PROVIDE( _eitcm = . );	

	.preinit_array  :
	{
	  PROVIDE_HIDDEN (__preinit_array_start = .);
	  KEEP (*(.preinit_array))
	  PROVIDE_HIDDEN (__preinit_array_end = .);
	} >FLASH AT>FLASH 
	
	.init_array     :
	{
	  PROVIDE_HIDDEN (__init_array_start = .);
	  KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
	  KEEP (*(.init_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
	  PROVIDE_HIDDEN (__init_array_end = .);
	} >FLASH AT>FLASH 
	
	.fini_array     :
	{
	  PROVIDE_HIDDEN (__fini_array_start = .);
	  KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
	  KEEP (*(.fini_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
	  PROVIDE_HIDDEN (__fini_array_end = .);
	} >FLASH AT>FLASH 
	
	.ctors          :
	{
	  /* gcc uses crtbegin.o to find the start of
	     the constructors, so we make sure it is
	     first.  Because this is a wildcard, it
	     doesn't matter if the user does not
	     actually link against crtbegin.o; the
	     linker won't look for a file to match a
	     wildcard.  The wildcard also means that it
	     doesn't matter which directory crtbegin.o
	     is in.  */
	  KEEP (*crtbegin.o(.ctors))
	  KEEP (*crtbegin?.o(.ctors))
	  /* We don't want to include the .ctor section from
	     the crtend.o file until after the sorted ctors.
	     The .ctor section from the crtend file contains the
	     end of ctors marker and it must be last */
	  KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .ctors))
	  KEEP (*(SORT(.ctors.*)))
	  KEEP (*(.ctors))
	} >FLASH AT>FLASH 
	
	.dtors          :
	{
	  KEEP (*crtbegin.o(.dtors))
	  KEEP (*crtbegin?.o(.dtors))
	  KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
	  KEEP (*(SORT(.dtors.*)))
	  KEEP (*(.dtors))
	} >FLASH AT>FLASH 

	.dalign :
	{
		. = ALIGN(4);
		PROVIDE(_data_vma = .);
	} >RAM AT>FLASH	

	.dlalign :
	{
		. = ALIGN(4); 
		PROVIDE(_data_lma = .);
	} >FLASH AT>FLASH

	.data :
	{
    	*(.gnu.linkonce.r.*)
    	*(.data .data.*)
    	*(.gnu.linkonce.d.*)
		/* Code that must execute from zero-wait-state SRAM.
		 * The startup .data copy loop (_data_lma.._edata) copies this
		 * from flash to RAM automatically at reset. */
		. = ALIGN(4);
		*(.ramfunc)
		*(.ramfunc.*)
		. = ALIGN(8);
    	PROVIDE( __global_pointer$ = . + 0x800 );
    	*(.sdata .sdata.*)
		*(.sdata2.*)
    	*(.gnu.linkonce.s.*)
    	. = ALIGN(8);
    	*(.srodata.cst16)
    	*(.srodata.cst8)
    	*(.srodata.cst4)
    	*(.srodata.cst2)
    	*(.srodata .srodata.*)
    	. = ALIGN(4);
		PROVIDE( _edata = .);
	} >RAM AT>FLASH

	.bss :
	{
		. = ALIGN(4);
		PROVIDE( _sbss = .);
  	    *(.sbss*)
        *(.gnu.linkonce.sb.*)
		*(.bss*)
     	*(.gnu.linkonce.b.*)
		*(COMMON*)
		. = ALIGN(4);
		PROVIDE( _ebss = .);
	} >RAM AT>FLASH

	PROVIDE( _end = _ebss);
	PROVIDE( end = . );

	/* Cart ROM mirror: a 32 KiB SRAM region (VMA = CARTROM SRAM) where the
	 * MSX cart image is served from. cartpnt points here at boot.
	 *
	 * hello_rom[] in firmware/User/hello_rom.c is declared as a 1-byte
	 * placeholder so the linker keeps the .cartrom section alive and
	 * emits _cartrom_vma / _cartrom_lma / _cartrom_end for the startup
	 * copy loop. The actual ROM image is uploaded at runtime via the
	 * CLI's XLOAD/LOAD commands - main() calls ROM_Clear() before
	 * Init_Cart() so the SRAM mirror starts as 0xFF (open bus), and
	 * the XLOAD command writes the user-supplied .bin file into it.
	 *
	 * KEEP() is required because --gc-sections is enabled at link time
	 * and the section is referenced only by the _cartrom_* PROVIDE
	 * symbols, not by any C code (cart.c reads through the cartpnt
	 * pointer, which is a runtime constant).
	 *
	 * Symbols exported for the startup copy loop:
	 *   _cartrom_vma  - destination in SRAM (0x20014400)
	 *   _cartrom_lma  - source in flash (4 bytes - just the placeholder)
	 *   _cartrom_end  - end of both (~4 bytes worth of placeholder) */
	.cartrom :
	{
		. = ALIGN(4);
		PROVIDE(_cartrom_vma = .);
		KEEP(*(.cartrom))
		KEEP(*(.cartrom.*))
		. = ALIGN(4);
		PROVIDE(_cartrom_end = .);
	} >CARTROM AT>FLASH

	PROVIDE(_cartrom_lma = LOADADDR(.cartrom));

    .stack ORIGIN(RAM) + LENGTH(RAM) - __stack_size :
    {
        PROVIDE( _heap_end = . );    
        . = ALIGN(4);
        PROVIDE(_susrstack = . );
        . = . + __stack_size;
        PROVIDE( _eusrstack = .);
    } >RAM 

}



