// On-FPGA control-plane recorder. Captures AVDECC (ADP/AECP/ACMP) + MSRP frames
// on BOTH directions (RX from the wire, TX we send) into a RAM ring from boot,
// with NO printf in the record path (so it doesn't perturb the timing we debug).
// Dump later with the 'R' console command. AAF/CRF audio + gPTP are skipped so the
// buffer stays small and the ACMP reconnect handshake is readable.
#ifndef CAP_H
#define CAP_H
#include <stdint.h>

void cap_set_eid(const uint8_t *eid);   // our 8-byte entity_id — filters ACMP to our streams
void cap_record(uint8_t dir, const uint8_t *frame, uint32_t len);  // dir: 0=RX 1=TX
void cap_dump(void);      // decode + print the ring ('R' command)
void cap_reset(void);     // clear + re-arm

#endif
