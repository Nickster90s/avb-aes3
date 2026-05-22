// AVTP wire constants — ethertype + subtype codepoints. Shared by the
// dispatcher (main.c) and per-subtype handlers (aaf.c, mcr.c).
//
// Lifted from the now-removed firmware/avtp.c stub (which was a stale
// IEC 61883-6 talker/listener kept around as scaffolding before
// AAF/CRF/AVDECC went in). The 61883 subtype constant stays defined
// here but no handler exists — dispatcher silently drops those.

#ifndef AVTP_CONST_H
#define AVTP_CONST_H

#define AVTP_ETHERTYPE           0x22F0

// Data PDUs (cd=0 — byte 0 of AVTPDU is the subtype directly).
#define AVTP_SUBTYPE_61883_IIDC  0x00
#define AVTP_SUBTYPE_AAF         0x02
#define AVTP_SUBTYPE_CRF         0x04

// Control-PDU subtypes (ADP/AECP/ACMP, 0xFA/B/C) live in avdecc.h.

#endif // AVTP_CONST_H
