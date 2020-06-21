/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2020 Lubomir Rintel <lkundrak@v3.sk>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * This is a generic linker script for GD32V targets using libopencm3.
 *
 * Memory regions MUST be defined in the ld script which includes this one!
 * Example:

MEMORY
{
	romalias (rx)	: ORIGIN = 0x00000000, LENGTH = 4K
	rom (rx)	: ORIGIN = 0x08000000, LENGTH = 256K
	ram (rwx)	: ORIGIN = 0x20000000, LENGTH = 16K
}

INCLUDE gd32v-generic.ld

*/

/* Enforce emission of the vector table. */
EXTERN (vector_table)

/* Define the entry point of the output file. */
ENTRY (reset_handler)

/* Define sections. */
SECTIONS
{
	.reset : {
		*(.reset)
		. = 0x000000c;
		*(.vectors)	/* Vector table */
	} >romalias AT >rom

	.text : {
		*(.text*)	/* Program code */
		. = ALIGN(4);
		*(.rodata*)	/* Read-only data */
		*(.srodata*)
		. = ALIGN(4);
	} >rom

	/* C++ Static constructors/destructors, also used for __attribute__
	 * ((constructor)) and the likes */
	.preinit_array : {
		. = ALIGN(4);
		__preinit_array_start = .;
		KEEP (*(.preinit_array))
		__preinit_array_end = .;
	} >rom
	.init_array : {
		. = ALIGN(4);
		__init_array_start = .;
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array))
		__init_array_end = .;
	} >rom
	.fini_array : {
		. = ALIGN(4);
		__fini_array_start = .;
		KEEP (*(.fini_array))
		KEEP (*(SORT(.fini_array.*)))
		__fini_array_end = .;
	} >rom

	. = ALIGN(4);
	_etext = .;

	.data : {
		_data = .;
		*(.data*)	/* Read-write initialized data */
		*(.sdata*)
	} >ram AT >rom
	_data_loadaddr = LOADADDR(.data);

	. = ALIGN(4);
	_edata = .;

	.bss : {
		*(.bss*)	/* Read-write zero initialized data */
		*(.sbss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
	} >ram

	/*
	 * The .eh_frame section appears to be used for C++ exception handling.
	 * You may need to fix this if you're using C++.
	 */
	/DISCARD/ : { *(.eh_frame) }

	. = ALIGN(4);
	end = .;
}

PROVIDE(_stack = ORIGIN(ram) + LENGTH(ram));
