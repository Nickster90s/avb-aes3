// Minimal read-only OSC over UDP/IPv4 for the raw-MAC AVB SoC. See osc.h.
#include "osc.h"
#include "avdecc.h"          // avdecc_apply_usb_source()
#include <string.h>
#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>

#define IPPROTO_UDP  17u

uint8_t g_osc_ip[4]  = {169, 254, 9, 200};   // static default
uint8_t g_osc_prefix = 16;                   // /16

static uint8_t  osc_mac[6];
static uint32_t osc_msgs;
static uint32_t osc_arps;

// ---- byte helpers (big-endian on the wire) ----
static inline uint16_t rd16(const uint8_t *p){ return ((uint16_t)p[0] << 8) | p[1]; }
static inline uint32_t rd32(const uint8_t *p){
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static inline void wr16(uint8_t *p, uint16_t v){ p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
// memcmp/strcmp aren't in this libc — tiny inlines instead.
static inline int eq4(const uint8_t *a, const uint8_t *b){
    return a[0]==b[0] && a[1]==b[1] && a[2]==b[2] && a[3]==b[3];
}
static int streq(const char *a, const char *b){
    while (*a && *a == *b){ a++; b++; }
    return *a == *b;
}

// ---- "a.b.c.d/prefix" <-> g_osc_ip/g_osc_prefix (for the UTF8 AVDECC control) ----
static int u8str(char *b, unsigned v){
    if (v >= 100){ b[0]='0'+v/100; b[1]='0'+(v/10)%10; b[2]='0'+v%10; return 3; }
    if (v >= 10) { b[0]='0'+v/10;  b[1]='0'+v%10; return 2; }
    b[0]='0'+v; return 1;
}
int osc_ip_str(char *buf, int maxlen){
    (void)maxlen;
    int n = 0;
    for (int k = 0; k < 4; k++){ n += u8str(buf+n, g_osc_ip[k]); if (k<3) buf[n++]='.'; }
    buf[n++] = '/'; n += u8str(buf+n, g_osc_prefix); buf[n] = 0;
    return n;
}
int osc_parse_ipstr(const char *s){
    unsigned oct[4], val = 0, pfx = 0; int oi = 0, dig = 0, havep = 0;
    const char *p = s;
    for (;;){
        if (*p>='0' && *p<='9'){ val = val*10 + (*p-'0'); if (val>255) return 0; dig=1; p++; }
        else if (*p=='.'){ if (!dig || oi>=3) return 0; oct[oi++]=val; val=0; dig=0; p++; }
        else break;
    }
    if (!dig || oi != 3) return 0;
    oct[oi++] = val;
    if (*p=='/'){ p++; val=0; dig=0;
        while (*p>='0' && *p<='9'){ val=val*10+(*p-'0'); dig=1; p++; }
        if (dig){ pfx=val; havep=1; } }
    if (*p!=0 && *p!='\n' && *p!='\r' && *p!=' ') return 0;   // trailing garbage
    for (int k=0;k<4;k++) g_osc_ip[k] = (uint8_t)oct[k];
    if (havep) g_osc_prefix = (pfx >= 20) ? 24 : 16;          // snap /16 or /24
    return 1;
}

void     osc_set_mac(const uint8_t mac[6]){ memcpy(osc_mac, mac, 6); }
uint32_t osc_rx_count(void){ return osc_msgs; }
uint32_t osc_arp_count(void){ return osc_arps; }

// ---- raw Ethernet TX (own txslot; shared 2-slot reader, serialized like the
// other modules — sends only happen from the single-threaded main RX loop) ----
static uint32_t osc_txslot;
static uint8_t *osc_tx_buf(void){
    return (uint8_t *)(ETHMAC_BASE + ETHMAC_SLOT_SIZE * (ETHMAC_RX_SLOTS + osc_txslot));
}
static void osc_eth_send(uint32_t len){
    while (!ethmac_sram_reader_ready_read())
        ;
    ethmac_sram_reader_slot_write(osc_txslot);
    ethmac_sram_reader_length_write(len);
    ethmac_sram_reader_start_write(1);
    osc_txslot = (osc_txslot + 1) % ETHMAC_TX_SLOTS;
}

// ---- is this IPv4 dest addressed to us? (unicast / limited bcast / multicast
// / subnet-directed broadcast under our /16 or /24) ----
static int ip_for_us(const uint8_t d[4]){
    if (eq4(d, g_osc_ip)) return 1;
    if (d[0] == 255 && d[1] == 255 && d[2] == 255 && d[3] == 255) return 1;
    if (d[0] >= 224 && d[0] <= 239) return 1;                       // multicast
    if (g_osc_prefix == 24){
        if (d[0]==g_osc_ip[0] && d[1]==g_osc_ip[1] && d[2]==g_osc_ip[2] && d[3]==255) return 1;
    } else {                                                        // /16
        if (d[0]==g_osc_ip[0] && d[1]==g_osc_ip[1] && d[2]==255 && d[3]==255) return 1;
    }
    return 0;
}

// ---- ARP responder: reply to who-has <our-ip> ----
static void arp_handle(const uint8_t *f, uint32_t len){
    if (len < 42) return;
    const uint8_t *a = f + 14;                       // ARP payload
    if (rd16(a)!=1 || rd16(a+2)!=IPV4_ETHERTYPE || a[4]!=6 || a[5]!=4) return;
    if (rd16(a+6) != 1) return;                      // requests only
    if (!eq4(a + 24, g_osc_ip)) return;    // target proto addr != us
    const uint8_t *sha = a + 8;                      // sender hw addr
    const uint8_t *spa = a + 14;                     // sender proto addr

    uint8_t *tx = osc_tx_buf();
    memcpy(tx + 0, sha, 6);                          // dst = requester
    memcpy(tx + 6, osc_mac, 6);                      // src = us
    wr16(tx + 12, ARP_ETHERTYPE);
    uint8_t *r = tx + 14;
    wr16(r+0, 1); wr16(r+2, IPV4_ETHERTYPE); r[4]=6; r[5]=4; wr16(r+6, 2);  // op=reply
    memcpy(r + 8,  osc_mac, 6);                      // sender hw  = us
    memcpy(r + 14, g_osc_ip, 4);                     // sender proto = our ip
    memcpy(r + 18, sha, 6);                          // target hw  = requester
    memcpy(r + 24, spa, 4);                          // target proto = requester ip
    memset(tx + 42, 0, 60 - 42);                     // pad to min frame
    osc_eth_send(60);
    osc_arps++;
}

// ---- OSC parse + dispatch (one message; int/float first arg) ----
static inline uint32_t align4(uint32_t n){ return (n + 3u) & ~3u; }

static void osc_dispatch(const uint8_t *p, uint32_t plen){
    if (plen < 8) return;
    // address pattern: null-terminated, 4-byte padded
    uint32_t alen = 0; while (alen < plen && p[alen]) alen++;
    if (alen >= plen) return;
    const char *addr = (const char *)p;
    uint32_t toff = align4(alen + 1);
    if (toff >= plen || p[toff] != ',') return;       // type-tag string
    const char *types = (const char *)(p + toff);
    uint32_t tlen = 0; while (toff + tlen < plen && p[toff + tlen]) tlen++;
    uint32_t aoff = toff + align4(tlen + 1);

    int32_t v0 = 0; int have = 0;
    if (tlen >= 2 && (types[1] == 'i' || types[1] == 'f')){
        if (aoff + 4 <= plen){ v0 = (int32_t)rd32(p + aoff); have = 1; }
    }

    // ---- dispatch table (extend here) ----
    if (streq(addr, "/switchover/source") && have){
        avdecc_apply_usb_source(v0 ? 1 : 0); osc_msgs++;
    } else if (streq(addr, "/switchover/main")){
        avdecc_apply_usb_source(0); osc_msgs++;
    } else if (streq(addr, "/switchover/backup")){
        avdecc_apply_usb_source(1); osc_msgs++;
    }
}

// ---- entry: called from main RX loop for ethertype 0x0806 / 0x0800 ----
int osc_rx_frame(const uint8_t *frame, uint32_t len){
    if (len < 14) return 0;
    uint16_t et = rd16(frame + 12);
    if (et == ARP_ETHERTYPE){ arp_handle(frame, len); return 1; }
    if (et != IPV4_ETHERTYPE) return 0;
    if (len < 14 + 20 + 8) return 0;

    const uint8_t *ip = frame + 14;
    if ((ip[0] >> 4) != 4) return 0;                  // IPv4
    uint32_t ihl = (uint32_t)(ip[0] & 0x0f) * 4u;
    if (ihl < 20) return 0;
    if (ip[9] != IPPROTO_UDP) return 0;
    if (!ip_for_us(ip + 16)) return 0;

    const uint8_t *udp = ip + ihl;
    if ((uint32_t)(udp - frame) + 8u > len) return 0;
    if (rd16(udp + 2) != OSC_UDP_PORT) return 1;      // ours, but not OSC port
    uint16_t ulen = rd16(udp + 4);
    if (ulen < 8) return 1;
    uint32_t plen = (uint32_t)ulen - 8u;
    const uint8_t *payload = udp + 8;
    uint32_t avail = len - (uint32_t)(payload - frame);
    if (plen > avail) plen = avail;
    osc_dispatch(payload, plen);
    return 1;
}
