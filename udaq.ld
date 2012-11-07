MEMORY
{
	rom (rx) : ORIGIN = 0x00000000, LENGTH = 512K
	ram (rwx) : ORIGIN = 0x10000000, LENGTH = 32K

	iram1 (rwx) : ORIGIN = 0x20000000, LENGTH = 112K
	iram2 (rwx) : ORIGIN = 0x2001c000, LENGTH =  16K
}

/* Include the common ld script. */
INCLUDE libopencm3_stm32f4.ld
