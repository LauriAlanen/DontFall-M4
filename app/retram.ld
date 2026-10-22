/*
 * libmicroros's runtime code in RETRAM: XRCE client, rcl, rclc, rmw,
 * micro-CDR. It has a fixed size and only grows with new kinds of
 * entities. In SRAM1 instead: its constants (.rodata) and the generated
 * code of the message types (objects named *rosidl*), which grows with
 * every new type, so adding a message type never fills RETRAM.
 *
 * Included before Zephyr's text section (PRE_TEXT_SECTIONS,
 * patches/zephyr/0004): ld puts each input section into the first output
 * section that matches it, so these patterns win over the generic
 * *(.text.*). Its data and bss stay in SRAM2.
 *
 * Linked in place: remoteproc loads the segment to its address, nothing
 * is copied at boot. Calls between here and SRAM1 are too far for a BL
 * and go through linker veneers.
 */
SECTION_PROLOGUE(.retram_text,,)
{
	*libmicroros.a:*(EXCLUDE_FILE(*libmicroros.a:*rosidl*) .text
	                  EXCLUDE_FILE(*libmicroros.a:*rosidl*) .text.*)
} GROUP_LINK_IN(RETRAM)
