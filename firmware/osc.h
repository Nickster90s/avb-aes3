// Minimal read-only OSC (Open Sound Control) over UDP/IPv4.
//
// The SoC uses a raw LiteEthMAC (no HW IP stack), so this is a software
// IPv4/UDP/ARP shim that the main RX loop feeds for ethertype 0x0800/0x0806.
// A static IP + /16 or /24 mask (settable via AVDECC CONTROLs) identifies us;
// an ARP responder lets unicast senders resolve us. OSC messages on UDP port
// 8000 are parsed and dispatched (e.g. /switchover/source -> Main/Backup mux).
#pragma once
#include <stdint.h>

#define ARP_ETHERTYPE   0x0806u
#define IPV4_ETHERTYPE  0x0800u
#define OSC_UDP_PORT     8000u

// Our static IPv4 config (defined in osc.c). Written directly by the AVDECC
// IP-config CONTROLs; read by the RX filter + ARP + descriptor build.
extern uint8_t g_osc_ip[4];      // default 169.254.9.200
extern uint8_t g_osc_prefix;     // 16 or 24

int      osc_ip_str(char *buf, int maxlen);      // format "a.b.c.d/prefix"; returns length
int      osc_parse_ipstr(const char *s);         // parse "a.b.c.d[/16|/24]"; 1=ok, 0=bad
void     osc_set_mac(const uint8_t mac[6]);      // our Ethernet MAC (for ARP replies)
int      osc_rx_frame(const uint8_t *frame, uint32_t len);  // 1 = consumed
uint32_t osc_rx_count(void);     // OSC messages dispatched (diag)
uint32_t osc_arp_count(void);    // ARP replies sent (diag)
