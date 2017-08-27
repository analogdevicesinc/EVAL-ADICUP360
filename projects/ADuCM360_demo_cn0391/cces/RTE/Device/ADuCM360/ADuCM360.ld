/* 
 * Portions Copyright (c) 2017 Analog Devices, Inc.
 *
 * Based on Device/ARM/ARMCM3/Source/GCC/gcc_arm.ld file in
 * ARM.CMSIS.4.5.0.pack.
 */

/*
 * Placeholder to list other libraries required by the application.
 
GROUP(
)

 */

 /*
 * Memory Spaces Definitions.
 *
 * Need modifying for a specific board. 
 *   FLASH.ORIGIN: starting address of flash
 *   FLASH.LENGTH: length of flash
 *   RAM.ORIGIN: starting address of RAM bank 0
 *   RAM.LENGTH: length of RAM bank 0
 *
 * The values below can be addressed in further linker scripts
 * using functions like 'ORIGIN(RAM)' or 'LENGTH(RAM)'.
 */

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 128K
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 8K

  /*
   * Optional sections; define the origin and length to match
   * the the specific requirements of your hardware. The zero
   * length prevents inadvertent allocation.
   */
  CCMRAM (xrw) : ORIGIN = 0x10000000, LENGTH = 0
  FLASHB1 (rx) : ORIGIN = 0x00000000, LENGTH = 0
  EXTMEMB0 (rx) : ORIGIN = 0x00000000, LENGTH = 0
  EXTMEMB1 (rx) : ORIGIN = 0x00000000, LENGTH = 0
  EXTMEMB2 (rx) : ORIGIN = 0x00000000, LENGTH = 0
  EXTMEMB3 (rx) : ORIGIN = 0x00000000, LENGTH = 0
}
/*
 * The '__stack' definition is required by crt0, do not remove it.
 */
__stack = ORIGIN(RAM) + LENGTH(RAM);

__StackTop = __stack; 	/* STM specific definition */

/*
 * Default stack sizes.
 * These are used by the startup in order to allocate stacks 
 * for the different modes.
 */

__Main_Stack_Size = 1024 ;

PROVIDE ( _Main_Stack_Size = __Main_Stack_Size ) ;

__Main_Stack_Limit = __stack  - __Main_Stack_Size ;

/* "PROVIDE" allows to easily override these values from an 
 * object file or the command line. */
PROVIDE ( _Main_Stack_Limit = __Main_Stack_Limit ) ;

/*
 * There will be a link error if there is not this amount of 
 * RAM free at the end. 
 */
_Minimum_Stack_Size = 256 ;

/*
 * Default heap definitions.
 * The heap start immediately after the last statically allocated 
 * .sbss/.noinit section, and extends up to the main stack limit.
 */
PROVIDE ( _Heap_Begin = _end_noinit ) ;
PROVIDE ( _Heap_Limit = __stack - __Main_Stack_Size ) ;
/*
 * For external ram use something like:
 *  RAM (xrw) : ORIGIN = 0x64000000, LENGTH = 2048K
 *
 * For special RAM areas use something like:
 *  MEMORY_ARRAY (xrw)  : ORIGIN = 0x20002000, LENGTH = 32
 */
 
/* 
 * The entry point is informative, for debuggers and simulators,
 * since the Cortex-M vector points to it anyway.
 */
ENTRY(Reset_Handler)


/* Sections Definitions */

SECTIONS
{
    /*
     * For Cortex-M devices, the beginning of the startup code is stored in
     * the .vectors section, which goes to FLASH 
     */
    .vectors :
    {
        . = ALIGN(4);
        KEEP(*(.vectors))
        __Vectors_End = .;
        __Vectors_Size = __Vectors_End - __Vectors;

        /* 
         * This section is here for convenience, to store the
         * startup code at the beginning of the flash area, hoping that
         * this will increase the readability of the listing.
         */
        *(.after_vectors .after_vectors.*)  /* Startup code and ISR */

        __end__ = .;
        . = ALIGN(4);
    } > FLASH

    /*
     * The program code is stored in the .text section, 
     * which goes to FLASH.
     */
    .text :
    {
        . = ALIGN(4);
        *(.text*)

        KEEP(*(.init))
        KEEP(*(.fini))

        /* .ctors */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)

        /* .dtors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        *(.rodata*)
        *(vtable)

        KEEP(*(.eh_frame*))

    } >FLASH

    /* ARM magic sections */
    .ARM.extab :
    {
       *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH
    
    __exidx_start = .;      
    .ARM.exidx :
    {
       *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH
    __exidx_end = .;
    
    /* To copy multiple ROM to RAM sections,
     * uncomment .copy.table section and,
     * define __STARTUP_COPY_MULTIPLE in startup_ARMCMx.c */
    /*
    .copy.table :
    {
      . = ALIGN(4);
      __copy_table_start__ = .;
      LONG (__etext)
      LONG (__data_start__)
      LONG (__data_end__ - __data_start__)
      LONG (__etext2)
      LONG (__data2_start__)
      LONG (__data2_end__ - __data2_start__)
      __copy_table_end__ = .;
    } > FLASH
    */

    /* To clear multiple BSS sections,
     * uncomment .zero.table section and,
     * define __STARTUP_CLEAR_BSS_MULTIPLE in startup_ARMCMx.c */
    /*
    .zero.table :
    {
      . = ALIGN(4);
      __zero_table_start__ = .;
      LONG (__bss_start__)
      LONG (__bss_end__ - __bss_start__)
      LONG (__bss2_start__)
      LONG (__bss2_end__ - __bss2_start__)
      __zero_table_end__ = .;
    } > FLASH
    */

    /*
     * The initialised data section.
     * The program executes knowing that the data is in the RAM
     * but the loader puts the initial values in the FLASH (inidata).
     * It is one task of the startup to copy the initial values from 
     * FLASH to RAM.
     */

    __etext = .;

    .data  : AT ( __etext )
    {
        . = ALIGN(4);

        __data_start__ = .;
        *(.data*)

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);


        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        KEEP(*(.jcr*))
        . = ALIGN(4);
        /* All data end */
        __data_end__ = .;

    } >RAM 
          

    /* For external memory assign */
    .bss_CCMRAM : ALIGN(4)
    {
        *(.bss.CCMRAM .bss.CCMRAM.*)
    } > CCMRAM
    /*
     * The uninitialised data section. NOLOAD is used to avoid
     * the "section `.bss' type changed to PROGBITS" warning
     */
    .bss (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM

    
    .noinit (NOLOAD) : ALIGN(4)
    {
        _noinit = .;
        
        *(.noinit .noinit.*) 
        
         . = ALIGN(4) ;
        _end_noinit = .;   
    } > RAM
    /* Mandatory to be word aligned, _sbrk assumes this */
    PROVIDE ( end = _end_noinit ); /* was _ebss */
    PROVIDE ( _end = _end_noinit );
    PROVIDE ( __end = _end_noinit );
    PROVIDE ( __end__ = _end_noinit );
    
    /*
     * Used for validation only, do not allocate anything here!
     *
     * This is just to check that there is enough RAM left for the Main
     * stack. It should generate an error if it's full.
     */
    ._check_stack : ALIGN(4)
    {
        . = . + _Minimum_Stack_Size ;
    } >RAM
    

   
    /*
     * The FLASH Bank1.
     * The C or assembly source must explicitly place the code 
     * or data there using the "section" attribute.
     */
    .b1text :
    {
        *(.b1text)                   /* remaining code */
        *(.b1rodata)                 /* read-only data (constants) */
        *(.b1rodata.*)
    } >FLASHB1
    
    /*
     * The EXTMEM.
     * The C or assembly source must explicitly place the code or data there
     * using the "section" attribute.
     */

    /* EXTMEM Bank0 */
    .eb0text :
    {
        *(.eb0text)                   /* remaining code */
        *(.eb0rodata)                 /* read-only data (constants) */
        *(.eb0rodata.*)
    } >EXTMEMB0
    
    /* EXTMEM Bank1 */
    .eb1text :
    {
        *(.eb1text)                   /* remaining code */
        *(.eb1rodata)                 /* read-only data (constants) */
        *(.eb1rodata.*)
    } >EXTMEMB1
    
    /* EXTMEM Bank2 */
    .eb2text :
    {
        *(.eb2text)                   /* remaining code */
        *(.eb2rodata)                 /* read-only data (constants) */
        *(.eb2rodata.*)
    } >EXTMEMB2
    
    /* EXTMEM Bank0 */
    .eb3text :
    {
        *(.eb3text)                   /* remaining code */
        *(.eb3rodata)                 /* read-only data (constants) */
        *(.eb3rodata.*)
    } >EXTMEMB3
   

    /* After that there are only debugging sections. */
    
    /* This can remove the debugging information from the standard libraries */    
    /* 
    DISCARD :
    {
     libc.a ( * )
     libm.a ( * )
     libgcc.a ( * )
     }
     */
  
}
 