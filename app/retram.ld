/*
 * libmicroros's code and constants in RETRAM. Included before Zephyr's
 * text section (PRE_TEXT_SECTIONS, patches/zephyr/0004): ld puts each
 * input section into the first output section that matches it, so these
 * patterns win over the generic *(.text.*) and *(.rodata.*). Its data
 * and bss stay in SRAM2.
 *
 * Linked in place: remoteproc loads the segment to its address, nothing
 * is copied at boot. Calls between here and SRAM1 are too far for a BL
 * and go through linker veneers.
 */
SECTION_PROLOGUE(.retram_text,,)
{
	*libmicroros.a:*(.text .text.* .rodata .rodata.*)
} GROUP_LINK_IN(RETRAM)
