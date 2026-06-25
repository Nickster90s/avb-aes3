// Config SPI-flash NV driver — S7SPIFlash (SPIMaster, 40-bit) via STARTUPE2.
// CSRs: cfgflash_spi_{control,status,mosi,miso,cs}. Raw mode: MOSI is sent from
// bit 39 downward (so a command goes in the TOP bits); MISO accumulates in the
// LOW bits (last `bits` received). _cs: bit0=sel(assert), bit1=mode(1=hold CS).
#include "cfgflash.h"
#include <generated/csr.h>

#define SPI_DW   40           // SPIMaster data_width

// One SPI transfer of `bits` bits. `mosi` must already be top-aligned (the bits
// to send occupy [39 : 40-bits]). Returns MISO (received bits in the low `bits`).
static uint64_t spi_xfer(uint64_t mosi, uint32_t bits)
{
    cfgflash_spi_mosi_write(mosi);
    cfgflash_spi_control_write((bits << 8) | 1);   // length=bits, start (pulse)
    while (!(cfgflash_spi_status_read() & 1))       // wait done
        ;
    return cfgflash_spi_miso_read();
}

int cfgflash_selftest(void)
{
    // Internal loopback (MOSI->MISO inside the SPIMaster, before the pins).
    // Isolates the CSR interface + bit-driver from the CCLK/flash/pin path:
    // pass => driver good, any failure is the flash side; fail => driver bug.
    cfgflash_spi_loopback_write(1);
    cfgflash_spi_cs_write(1);
    uint64_t r = spi_xfer((uint64_t)0xA5 << 32, 8);   // send 0xA5, 8 bits
    cfgflash_spi_loopback_write(0);
    return ((r & 0xFF) == 0xA5);
}

uint32_t cfgflash_jedec(void)
{
    // 0x9F in [39:32], then 24 read bits -> JEDEC ID lands in miso[23:0].
    cfgflash_spi_cs_write(1);                       // sel=1, mode=0 (auto CS for one xfer)
    uint64_t r = spi_xfer((uint64_t)0x9F << 32, 32);
    return (uint32_t)(r & 0xFFFFFFu);
}

void cfgflash_read(uint32_t addr, uint8_t *buf, uint32_t n)
{
    // cmd 0x03 + 24-bit address (32 bits, top-aligned to [39:8]), CS HELD,
    // then n byte reads (8 clocks each).
    cfgflash_spi_cs_write(0x3);                     // sel=1, mode=1 (hold CS)
    uint64_t cmd = ((uint64_t)0x03 << 24) | (addr & 0xFFFFFFu);
    spi_xfer(cmd << (SPI_DW - 32), 32);             // send cmd+addr
    for (uint32_t i = 0; i < n; i++) {
        uint64_t r = spi_xfer(0, 8);               // clock 8 bits, capture
        buf[i] = (uint8_t)(r & 0xFF);
    }
    cfgflash_spi_cs_write(0);                       // release CS
}
