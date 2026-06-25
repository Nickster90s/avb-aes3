// Config SPI-flash NV driver — S7SPIFlash (SPIMaster, 40-bit) via STARTUPE2.
// CSRs: cfgflash_spi_{control,status,mosi,miso,cs}. Raw mode: MOSI is sent from
// bit 39 downward (command in the TOP bits); MISO accumulates in the LOW bits
// (last `bits` received). _cs: bit0=sel(assert), bit1=mode(1=hold CS for multi-
// transfer ops like page-program).
#include "cfgflash.h"
#include <generated/csr.h>

#define SPI_DW   40           // SPIMaster data_width

static void cfg_write_enable(void);   // fwd-decl (used by cfgflash_unprotect)
static void cfg_wait_wip(void);

// One SPI transfer of `bits` bits. `mosi` top-aligned ([39 : 40-bits]).
static uint64_t spi_xfer(uint64_t mosi, uint32_t bits)
{
    cfgflash_spi_mosi_write(mosi);
    cfgflash_spi_control_write((bits << 8) | 1);   // length=bits, start
    while (!(cfgflash_spi_status_read() & 1))
        ;
    return cfgflash_spi_miso_read();
}

int cfgflash_selftest(void)
{
    cfgflash_spi_loopback_write(1);
    cfgflash_spi_cs_write(1);
    uint64_t r = spi_xfer((uint64_t)0xA5 << 32, 8);
    cfgflash_spi_loopback_write(0);
    return ((r & 0xFF) == 0xA5);
}

void cfgflash_warmup(void)
{
    // 7-series STARTUPE2 masks the first few USRCCLKO edges after configuration,
    // so the VERY FIRST flash transaction gets swallowed (JEDEC reads 0xFFFFFF).
    // Clock out dummy bits with CS DEASSERTED (flash ignores them) to get past
    // the masking; subsequent real transactions then work. Must be called once
    // before any flash op. (The old loopback selftest masked this by accident.)
    cfgflash_spi_cs_write(0);                       // CS high (deselected)
    for (int i = 0; i < 8; i++)
        spi_xfer(0, 32);                            // 256 dummy CCLK edges
}

uint32_t cfgflash_jedec(void)
{
    cfgflash_spi_cs_write(1);                       // auto CS
    uint64_t r = spi_xfer((uint64_t)0x9F << 32, 32);
    return (uint32_t)(r & 0xFFFFFFu);
}

// READ: one self-contained 40-bit transfer per byte (cmd 0x03 + 24-bit addr +
// 8 read bits), AUTO CS — same single-shot pattern as the proven JEDEC read, so
// it sidesteps the flaky manual-CS multi-transfer path. Slow but rock-solid; our
// config blob is tiny.
void cfgflash_read(uint32_t addr, uint8_t *buf, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        uint32_t a = (addr + i) & 0xFFFFFFu;
        cfgflash_spi_cs_write(1);                  // auto CS
        uint64_t m = ((uint64_t)0x03 << 32) | ((uint64_t)a << 8);  // [39:32]cmd [31:8]addr
        uint64_t r = spi_xfer(m, 40);
        buf[i] = (uint8_t)(r & 0xFF);
    }
}

// ---- write path -----------------------------------------------------------
uint8_t cfgflash_status(void)
{
    cfgflash_spi_cs_write(1);
    uint64_t r = spi_xfer((uint64_t)0x05 << 32, 16);   // RDSR: cmd + 8 read
    return (uint8_t)(r & 0xFF);
}
#define cfg_status cfgflash_status

void cfgflash_unprotect(void)
{
    // Clear the status-register block-protect bits (WRSR 0x00) so data writes
    // are allowed. WRSR = cmd 0x01 + one status byte (16-bit single xfer). If the
    // flash has SRWD=1 AND WP# is low this is itself blocked — which would mean
    // the WP#(P21)-vs-USB pin conflict is real.
    cfg_write_enable();
    cfgflash_spi_cs_write(1);
    spi_xfer(((uint64_t)0x01 << 32) | ((uint64_t)0x00 << 24), 16);
    cfg_wait_wip();
}

static void cfg_wait_wip(void)
{
    // poll WIP (status bit0) until clear; bounded so a fault can't hang boot.
    for (uint32_t i = 0; i < 4000000; i++)
        if (!(cfg_status() & 0x01)) return;
}

static void cfg_write_enable(void)
{
    cfgflash_spi_cs_write(1);
    spi_xfer((uint64_t)0x06 << 32, 8);                 // WREN
}

void cfgflash_erase_4k(uint32_t addr)
{
    cfg_write_enable();
    cfgflash_spi_cs_write(1);                           // SE = single xfer
    uint64_t m = ((uint64_t)0x20 << 32) | ((uint64_t)(addr & 0xFFFFFF) << 8);
    spi_xfer(m, 32);                                    // SE: cmd + 24-bit addr
    cfg_wait_wip();
}

void cfgflash_program(uint32_t addr, const uint8_t *buf, uint32_t n)
{
    // ONE byte per page-program command, each a single 40-bit AUTO-CS transfer
    // (cmd 0x02 [39:32] + 24-bit addr [31:8] + 1 data byte [7:0]). Manual-CS
    // (hold) multi-transfer does NOT work on this SPIMaster — erase (auto-CS)
    // succeeds but the old held-CS program left the sector erased. Per-byte
    // auto-CS uses only the proven path. Slow but the config blob is tiny.
    for (uint32_t i = 0; i < n; i++) {
        cfg_write_enable();
        cfgflash_spi_cs_write(1);                       // auto CS
        uint64_t m = ((uint64_t)0x02 << 32)
                   | ((uint64_t)((addr + i) & 0xFFFFFF) << 8)
                   |  (uint64_t)buf[i];
        spi_xfer(m, 40);                                // PP cmd+addr+1 byte
        cfg_wait_wip();
    }
}
