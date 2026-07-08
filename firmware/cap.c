#include "cap.h"
#include "gptp.h"      // gptp_uptime_ms()
#include <stdio.h>
#include <string.h>

// 150 control frames × 56 bytes ≈ 8.4 KB. Enough for boot → gPTP lock → the
// talker reconnect handshake. STOP-when-full (keep the FIRST N from boot, not a
// ring) so the boot reconnect is always captured no matter when you dump.
#define CAP_N      150
#define CAP_BYTES  56    // eth(14) + ACMPDU up to listener_unique_id @ frame[52..53]

typedef struct {
    uint32_t t_ms;
    uint8_t  dir;        // 0=RX 1=TX
    uint16_t len;
    uint8_t  data[CAP_BYTES];
} cap_entry_t;

static cap_entry_t cap_ring[CAP_N];
static uint16_t    cap_count = 0;

static uint8_t cap_eid[8];       // our entity_id (set by cap_set_eid)
static uint8_t cap_eid_set = 0;

void cap_set_eid(const uint8_t *eid) { memcpy(cap_eid, eid, 8); cap_eid_set = 1; }

// Keep ADP (discovery) + MSRP + ONLY the ACMP that concerns OUR entity (talker
// or listener entity_id == us). Skip AECP (0xFB) — it's the controllers' READ_
// DESCRIPTOR enumeration flood that buries the reconnect — and skip cross-device
// ACMP we merely snoop (ACMP is multicast, so we see the whole segment).
static int cap_is_control(const uint8_t *f, uint32_t len)
{
    if (len < 16) return 0;
    uint16_t et = ((uint16_t)f[12] << 8) | f[13];
    if (et == 0x22ea) return 1;                       // MSRP
    if (et != 0x22f0) return 0;
    if (f[14] == 0xFA) return 1;                       // ADP (discovery)
    if (f[14] == 0xFC) {                               // ACMP — only OUR streams
        if (!cap_eid_set || len < 50) return 1;        // eid not known yet: keep all
        int tk = 1, ln = 1;
        for (int i = 0; i < 8; i++) {
            if (f[34 + i] != cap_eid[i]) tk = 0;       // talker_entity_id @ PDU+20
            if (f[42 + i] != cap_eid[i]) ln = 0;       // listener_entity_id @ PDU+28
        }
        return tk || ln;
    }
    return 0;                                          // AECP (0xFB) + rest: skip
}

void cap_record(uint8_t dir, const uint8_t *frame, uint32_t len)
{
    if (cap_count >= CAP_N) return;                   // stop when full (first-N)
    if (!cap_is_control(frame, len)) return;
    cap_entry_t *e = &cap_ring[cap_count++];
    e->t_ms = gptp_uptime_ms();
    e->dir  = dir;
    e->len  = (uint16_t)len;
    uint32_t n = len < CAP_BYTES ? len : CAP_BYTES;
    memcpy(e->data, frame, n);
    if (n < CAP_BYTES) memset(e->data + n, 0, CAP_BYTES - n);
}

void cap_reset(void) { cap_count = 0; }

static const char *acmp_name(uint8_t m)
{
    switch (m) {
    case 0:  return "CONNECT_TX_CMD";    case 1:  return "CONNECT_TX_RSP";
    case 2:  return "DISCONNECT_TX_CMD"; case 3:  return "DISCONNECT_TX_RSP";
    case 4:  return "GET_TX_STATE_CMD";  case 5:  return "GET_TX_STATE_RSP";
    case 6:  return "CONNECT_RX_CMD";    case 7:  return "CONNECT_RX_RSP";
    case 8:  return "DISCONNECT_RX_CMD"; case 9:  return "DISCONNECT_RX_RSP";
    case 10: return "GET_RX_STATE_CMD";  case 11: return "GET_RX_STATE_RSP";
    default: return "ACMP?";
    }
}

void cap_dump(void)
{
    printf("\n[CAP] %u control frames from boot (RX=from wire, TX=we sent):\n", cap_count);
    printf("   t_ms  dir  type                tuid luid st  src-mac\n");
    for (uint16_t i = 0; i < cap_count; i++) {
        cap_entry_t *e = &cap_ring[i];
        const uint8_t *f = e->data;
        const char *d = e->dir ? "TX" : "RX";
        uint16_t et = ((uint16_t)f[12] << 8) | f[13];
        // src MAC (last 3 bytes) — tells us MOTU (00:0e:55) vs us (…:00:42) vs bridge
        if (et == 0x22f0 && f[14] == 0xFC) {          // ACMP: PDU starts at frame[14]
            uint8_t  msg    = f[15] & 0x0F;
            uint8_t  status = (f[16] >> 3) & 0x1F;
            uint16_t tuid   = ((uint16_t)f[14 + 36] << 8) | f[14 + 37];
            uint16_t luid   = ((uint16_t)f[14 + 38] << 8) | f[14 + 39];
            // controller_id @ PDU+12 -> frame[26..33]; flags @ PDU+50 -> frame[64..65]
            // (beyond CAP_BYTES=56, so read low bytes we DO have). stream_id @ frame[18..25].
            uint8_t sid_present = 0;
            for (int k = 18; k < 26; k++) if (f[k]) { sid_present = 1; break; }
            printf("  %6lu  %s  %-18s t%u l%u st%u src%02x:%02x:%02x ctrl%02x:%02x:%02x sid%c\n",
                   (unsigned long)e->t_ms, d, acmp_name(msg), tuid, luid, status,
                   f[6], f[7], f[8], f[31], f[32], f[33], sid_present ? 'Y' : '-');
        } else if (et == 0x22f0 && f[14] == 0xFA) {
            printf("  %6lu  %s  ADP msg=%u            -    -   -  %02x:%02x:%02x\n",
                   (unsigned long)e->t_ms, d, f[15] & 0x0F, f[6], f[7], f[8]);
        } else if (et == 0x22f0 && f[14] == 0xFB) {
            printf("  %6lu  %s  AECP                  -    -   -  %02x:%02x:%02x\n",
                   (unsigned long)e->t_ms, d, f[6], f[7], f[8]);
        } else if (et == 0x22ea) {
            printf("  %6lu  %s  MSRP len=%-3u          -    -   -  %02x:%02x:%02x\n",
                   (unsigned long)e->t_ms, d, e->len, f[6], f[7], f[8]);
        }
    }
    printf("[CAP] end. %s\n\n",
           cap_count >= CAP_N ? "(buffer FULL — increase CAP_N if reconnect not shown)"
                              : "(still capturing)");
}
