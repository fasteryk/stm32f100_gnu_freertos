ENTRY(reset_handler)

MEMORY
{
	flash (rx) : ORIGIN = 0, LENGTH = 64K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
}

SECTIONS
{   
	.text : ALIGN(4) {
    	KEEP(*(.isr_vector))  
       	*(.text .text.*)
        *(.rodata .rodata.* .constdata .constdata.*)
		_end_of_rodata = .;
    } >flash
    
    .data : ALIGN(4) {
		_begin_of_data = .;
		*(.data .data.*)
		_end_of_data = .;
	} >ram AT >flash

    .bss : ALIGN(4) {
		_begin_of_bss = .;
		*(.bss .bss.*)
		_end_of_bss = .;
	} >ram

	.stack : ALIGN(4) {
		KEEP(*(.stack))
	} >ram
}

