// Persistent config store on the config-flash NV (cfgflash.c).
#include "config.h"
#include "cfgflash.h"

cfg_t g_cfg;

static uint32_t cfg_crc(const cfg_t *c)
{
    // Simple rolling checksum over all bytes except the trailing crc word.
    const uint8_t *p = (const uint8_t *)c;
    uint32_t s = 0x1357BD13u;
    for (uint32_t i = 0; i < sizeof(cfg_t) - 4; i++)
        s = (s * 31u) + p[i];
    return s;
}

static void cfg_defaults(void)
{
    uint8_t *p = (uint8_t *)&g_cfg;
    for (uint32_t i = 0; i < sizeof(cfg_t); i++) p[i] = 0;
    g_cfg.magic   = CFG_MAGIC;
    g_cfg.version = CFG_VERSION;
    g_cfg.size    = sizeof(cfg_t);
    g_cfg.cs      = 0;          // default gPTP
}

int cfg_load(void)
{
    cfg_t t;
    cfgflash_read(CFG_FLASH_ADDR, (uint8_t *)&t, sizeof(t));
    if (t.magic == CFG_MAGIC && t.version == CFG_VERSION &&
        t.size == sizeof(cfg_t) && t.crc == cfg_crc(&t)) {
        g_cfg = t;
        return 1;
    }
    cfg_defaults();
    return 0;
}

void cfg_save(void)
{
    g_cfg.magic   = CFG_MAGIC;
    g_cfg.version = CFG_VERSION;
    g_cfg.size    = sizeof(cfg_t);
    g_cfg.crc     = cfg_crc(&g_cfg);
    cfgflash_erase_4k(CFG_FLASH_ADDR);
    cfgflash_program(CFG_FLASH_ADDR, (const uint8_t *)&g_cfg, sizeof(g_cfg));
}
