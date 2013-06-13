MEMORY
{
	/* rom is really the shadow region that points to SPI flash or elsewhere */
	rom (rx)  : ORIGIN = 0x00000000, LENGTH =  1M
	ram_local1 (rwx) : ORIGIN = 0x10000000, LENGTH =  128K
	ram_local2 (rwx) : ORIGIN = 0x10080000, LENGTH =  72K
	/* there are some additional RAM regions */
	ram_ahb1   (rwx) : ORIGIN = 0x20000000, LENGTH =  16K
	/* Removed 32K of AHB SRAM for USB buffer. Straddles two blocks of RAM
	 * to get performance benefit of having two USB buffers addressable
	 * simultaneously (on two different buses of the AHB multilayer matrix)
	 */
	ram_ahb2   (rwx) : ORIGIN = 0x2000C000, LENGTH =  16K
}

SECTIONS
{
        .dma_data :
        {
                *(.dma_data)
        } >ram_local1
}

/* Include the common ld script. */
INCLUDE libopencm3_lpc43xx.ld
