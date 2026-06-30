// Persistent AVDECC/system configuration, stored in the config-flash NV sector
// (see cfgflash.c). General + EXTENSIBLE: to persist a new parameter, add a field
// before `reserved`, shrink `reserved` by the same size, and bump CFG_VERSION.
// Old/blank/corrupt blobs fail validation -> defaults are used (never corrupts).
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#define CFG_MAGIC    0xCFA50701u
#define CFG_VERSION  1

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;              // sizeof(cfg_t) — guards struct-layout changes
    // ---- persisted parameters ----
    uint8_t  cs;               // media clock source (0=gPTP/INTERNAL, 1=CRF/STREAM)
    uint8_t  crf_valid;        // 1 if the crf binding below is real
    uint8_t  _pad[2];
    uint8_t  crf_stream_id[8]; // last bound CRF stream (auto-reconnect, #70)
    uint8_t  crf_dmac[6];      // its dest MAC (for the AVTP RX filter on reconnect)
    uint8_t  _pad2[2];
    uint8_t  crf_talker_eid[8];// talker entity-id — matches the re-advertise so the
                               // fast-connect path auto-binds at boot (the #70 method)
    // ---- TX/talker auto-reconnect (#69): which listener was connected to each of
    //      our 6 talker streams, so we can PROACTIVELY re-connect it on cold boot
    //      (send it CONNECT_RX_COMMAND) instead of waiting for it to re-initiate. ----
    uint8_t  tx_listener_eid[8];   // the listener entity (MOTU) to reconnect to
    uint8_t  tx_streams_mask;      // bit N set = talker stream N was connected
    uint8_t  _pad3;
    uint16_t tx_listener_uid[6];   // per-stream listener unique_id (usually == talker uid)
    // ---- room for future AVDECC params ----
    // reserved sized so sizeof(cfg_t) is UNCHANGED from v1 (carved out of the old
    // reserved[80], which v1 wrote as zeros) -> old saved cs/CRF still validates,
    // tx_* reads as zero (mask=0 -> no talker reconnect until populated). No ver bump.
    uint8_t  reserved[58];
    uint32_t crc;              // checksum over all bytes above
} cfg_t;

extern cfg_t g_cfg;            // the live config (loaded at boot, saved on change)

// Load NV into g_cfg. Returns 1 if a valid saved config was found, else 0 and
// g_cfg is filled with defaults.
int  cfg_load(void);

// Persist g_cfg to NV (erase sector + program). Call after changing any field.
void cfg_save(void);

#endif // CONFIG_H
