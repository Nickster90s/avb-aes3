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

// Read n bytes from flash byte-address addr into buf (cmd 0x03, CS held).
void cfgflash_read(uint32_t addr, uint8_t *buf, uint32_t n);

#endif // CFGFLASH_H
