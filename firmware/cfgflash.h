// Config SPI-flash NV driver (S7SPIFlash / SPIMaster via STARTUPE2).
// Stores cs= + CRF stream binding in a sector at the TOP of the boot flash,
// far above the bitstream. Phase 1 = read-only (JEDEC + read) for verification.
#ifndef CFGFLASH_H
#define CFGFLASH_H

#include <stdint.h>

// SPIMaster internal-loopback self-test (no flash needed). 1 = driver OK.
int cfgflash_selftest(void);

// JEDEC ID (cmd 0x9F): [23:16]=manufacturer, [15:8]=mem type, [7:0]=capacity
// (capacity byte N => density 2^N bytes; e.g. 0x16=4MB, 0x18=16MB).
uint32_t cfgflash_jedec(void);

// Read n bytes from flash byte-address addr into buf (per-byte auto-CS).
void cfgflash_read(uint32_t addr, uint8_t *buf, uint32_t n);

// Erase the 4 KB sector containing addr (cmd 0x20). Blocks until done.
void cfgflash_erase_4k(uint32_t addr);

// Page-program n (<=256) bytes at addr (cmd 0x02). Sector must be pre-erased
// and the write must not cross a 256 B page boundary. Blocks until done.
void cfgflash_program(uint32_t addr, const uint8_t *buf, uint32_t n);

// Config sector: top 4 KB of the 16 MB flash, far above the ~2.2 MB bitstream.
#define CFG_FLASH_ADDR   0xFFF000u

#endif // CFGFLASH_H
