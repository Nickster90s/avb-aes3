// IEEE 802.1Qat SRP (Stream Reservation Protocol) — minimal endpoint
// MSRP only: Domain, Talker Advertise, Listener Ready
//
// Uses MRP (Multiple Registration Protocol) PDU encoding over
// EtherType 0x22EA to multicast 01:80:C2:00:00:0E.

#ifndef SRP_H
#define SRP_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

#define MSRP_ETHERTYPE          0x22EA
#define MSRP_PROTO_VERSION      0x00

// MSRP multicast destination (IEEE 802.1Q Table 10-1)
#define MSRP_MCAST_ADDR         {0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E}

// MSRP Attribute Types
#define MSRP_ATTR_TALKER_ADV    1   // AttributeLength = 25
#define MSRP_ATTR_TALKER_FAIL   2   // AttributeLength = 34
#define MSRP_ATTR_LISTENER      3   // AttributeLength = 8
#define MSRP_ATTR_DOMAIN        4   // AttributeLength = 4

// Listener declaration sub-states (FourPackedEvent encoding)
#define MSRP_LISTENER_IGNORE    0
#define MSRP_LISTENER_ASKFAILED 1
#define MSRP_LISTENER_READY     2
#define MSRP_LISTENER_READYFAIL 3

// MRP ThreePackedEvent encodings
#define MRP_EVT_NEW             0
#define MRP_EVT_JOININ          1
#define MRP_EVT_IN              2
#define MRP_EVT_JOINMT          3
#define MRP_EVT_MT              4
#define MRP_EVT_LV              5

// SR Class definitions
#define SR_CLASS_A              6
#define SR_CLASS_B              5
#define SR_CLASS_A_PRIO         3
#define SR_CLASS_B_PRIO         2
#define SR_CLASS_A_VID          2   // Default PVID for AVB

// MRP timer values (ms)
#define MRP_JOIN_PERIOD_MS      1000    // Join period (we send every 1s for simplicity)
#define MRP_LEAVEALL_PERIOD_MS  10000   // LeaveAll every 10s

// MRP PDU encoding
#define MRP_ENDMARK             0x0000
#define MRP_3PACK(a,b,c)        ((((a)*6 + (b))*6) + (c))
#define MRP_4PACK(a,b,c,d)      (((a)*64) + ((b)*16) + ((c)*4) + (d))

// Max MSRP frame size
#define MSRP_MAX_FRAME_LEN      200

// ---------------------------------------------------------------------------
// Talker Advertise parameters
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  stream_id[8];
    uint8_t  dest_addr[6];      // Stream destination multicast MAC
    uint16_t vlan_id;
    uint16_t max_frame_size;    // Max Ethernet frame size for this stream
    uint16_t max_interval_frames; // Max frames per class interval (1 for Class A)
    uint8_t  priority_and_rank; // (3-bit prio | 1-bit rank | 4 bits reserved)
    uint32_t accumulated_latency_ns;
} srp_talker_attr_t;

// Per-stream registrar entry. We track every TalkerAdvertise heard on
// the wire so AVDECC handlers (GET_STREAM_INFO, GetStreamInputInfoEx,
// GET_AVB_INFO) can report the real MSRP-accumulated-latency, vlan_id
// and dest_mac learned over SRP — not the hardcoded defaults we used
// to ship. Aged out by srp_poll when last_seen_ms exceeds the timeout.
#define SRP_MAX_REMOTE_TALKERS 8

typedef struct {
    uint8_t  valid;
    uint8_t  stream_id[8];
    uint8_t  dest_mac[6];
    uint16_t vlan_id;
    uint16_t max_frame_size;
    uint16_t max_interval_frames;
    uint8_t  priority_and_rank;
    uint32_t accumulated_latency_ns;
    uint32_t last_seen_ms;
} srp_remote_talker_t;

// One Listener attribute we declare. The FPGA is simultaneously a CRF
// listener (media-clock recovery) AND, when looped/tested, an AAF listener
// — and MSRP lets a participant declare many Listener attributes in one
// PDU, each its own stream_id + applicant state. A single listener slot
// made CRF and AAF clobber each other (CRF rebootstrap storm + AAF never
// registered). One entry per concurrently-registered listener stream.
#define SRP_MAX_LISTENER_STREAMS 4

typedef struct {
    uint8_t  enabled;
    uint8_t  stream_id[8];
    uint8_t  substate;            // MSRP_LISTENER_READY etc.
    uint8_t  new_count;           // applicant NEW-count: NEW(0) for first 2 TX
    uint8_t  talker_registered;   // a remote talker registered for THIS stream
    uint32_t talker_last_seen_ms; // last TalkerAdvertise matching THIS stream
} srp_listener_t;

// ---------------------------------------------------------------------------
// SRP state
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  src_mac[6];

    // Talker state — N_SRP_TALKERS reservations (6x8ch time-mux), each emitted
    // as its own TalkerAdvertise PDU. n_talkers = how many srp_talker_set has
    // configured. talker_enabled / talker_new_count / talker_listener_seen are
    // shared (all streams advertise together, gated by the same gPTP/CRF lock).
#define N_SRP_TALKERS 6
    uint8_t  talker_enabled;
    uint8_t  n_talkers;
    srp_talker_attr_t talkers[N_SRP_TALKERS];

    // Listener state — one entry per concurrently-registered listener
    // stream (CRF + AAF + spare). srp_listener_enable() adds/removes by
    // stream_id; srp_send_declarations() emits a Listener attribute for
    // each enabled entry.
    srp_listener_t listeners[SRP_MAX_LISTENER_STREAMS];

    // Timers (in milliseconds, tracked via gPTP uptime)
    uint32_t last_join_ms;
    uint32_t last_leaveall_ms;
    // Per-cycle randomized LeaveAll period. mrpd (mrp.c:422) uses
    // MRP_LVATIMER_VAL + random()%(VAL/2) so co-booting endpoints don't
    // synchronize their LeaveAlls into a storm. Recomputed each time a
    // LeaveAll fires; range [MRP_LEAVEALL_PERIOD_MS, 1.5×].
    uint32_t leaveall_period_ms;
    uint32_t join_count;

    // RX state
    uint8_t  domain_received;       // Have we seen a domain declaration?
    uint8_t  rx_sr_class;
    uint8_t  rx_sr_prio;
    uint16_t rx_sr_vid;
    // The SR domain MUST come from the bridge (gPTP grandmaster / switch),
    // which is authoritative for the reservation. Other talkers advertise
    // their own domain with a different SRclassID/priority; latching theirs
    // made us declare the wrong talker priority -> bridge rejects 0x13 (SR
    // Class Priority Mismatch). bridge_mac = GM MAC derived from gm_clock_id.
    uint8_t  bridge_mac[6];
    uint8_t  have_bridge_mac;
    uint32_t rx_pdu_count;
    // (talker_registered / talker_last_seen_ms / listener_new_count are now
    // per-stream, in srp_listener_t — see listeners[] above.)

    // MVRP VID registration — first 2 cycles emit NEW(0)
    // to register our port for VLAN 2 at the bridge; then JoinMt for refresh.
    uint8_t  mvrp_new_count;
    // Same idea for TalkerAdvertise + Domain. Without an initial MRPDU_NEW
    // the bridge's MSRP registrar never moves into the registered state
    // for our talker stream, so listeners get rejected even though the
    // wire-side advertise looks correct.
    uint8_t  talker_new_count;
    uint8_t  domain_new_count;
    // Set by srp_process_rx when a RECEIVED MSRP/MVRP PDU carries a LeaveAll
    // event; consumed by srp_poll to re-declare immediately with NEW (the MRP
    // applicant RLA response — GenAVB mrp.c: rLA -> VP -> re-Join). Without it,
    // a bridge/talker LeaveAll resets our registration and we don't re-register
    // until the next 1 s tick (often after the registrar leave_timeout), so the
    // talker prunes the stream -> the on-HW ~3 s CRF flap.
    uint8_t  rx_leaveall;

    // Flood guard: set when a RECEIVED MSRP Listener declaration carries OUR
    // talker stream_id — i.e. a remote listener actually wants our stream, so
    // the bridge has/will reserve a path and it's safe to transmit. main.c
    // gates the gateware AAF TX on this so the talker never blasts an
    // unreserved multicast (which the bridge floods to every port → the
    // segment-congestion / "AVB stuck" we hit). Aged out by srp_poll.
    uint8_t  talker_listener_seen;
    uint32_t talker_listener_seen_ms;
    uint8_t  talker_fail_code;        // last TalkerFailed code the bridge sent for OUR stream (0=none)
    uint32_t talker_fail_count;       // # TalkerFailed received for OUR stream

    // Per-attribute RX diagnostic counters. Bumped once per vector
    // (FirstValue) processed, NOT once per PDU. Lets the UART stats line
    // distinguish "PDU received but parser dropped attr" from "PDU never
    // arrived at all" — the missing-MOTU-TalkerAdv case under burst load.
    uint32_t rx_talker_adv_count;
    uint32_t rx_talker_failed_count;
    uint32_t rx_listener_count;
    uint32_t rx_domain_count;
    uint32_t rx_match_count;        // TalkerAdv whose stream_id matched our listener

    // Per-stream registrar table — one entry per unique TalkerAdvertise
    // stream_id observed. Updated on every matching RX, aged out by poll.
    srp_remote_talker_t remote_talkers[SRP_MAX_REMOTE_TALKERS];

    // Callback fired for every TalkerAdvertise observed on the wire,
    // regardless of whether it matches our currently-registered listener
    // stream_id. Used by main.c to learn the real stream_id for a
    // FAST_CONNECT listener bound only by dest_mac. NULL = disabled.
    void (*on_talker_advertise)(const uint8_t *stream_id,
                                const uint8_t *dest_mac);
} srp_state_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

void srp_init(srp_state_t *s, const uint8_t *mac_addr);

// Tell SRP the bridge (gPTP grandmaster / switch) MAC so it only latches the
// SR-class domain from the authoritative switch, not from peer talkers.
void srp_set_bridge_mac(srp_state_t *s, const uint8_t *bridge_mac);

// Look up a remote talker entry by stream_id. Returns NULL if not seen
// or aged out.
const srp_remote_talker_t *srp_find_talker(const srp_state_t *s,
                                            const uint8_t *stream_id);

// Configure talker advertisement for stream `idx` (0..N_SRP_TALKERS-1).
void srp_talker_set(srp_state_t *s, uint8_t idx, const uint8_t *stream_id,
                    const uint8_t *dest_mac, uint16_t max_frame_size);

// Enable/disable talker and listener SRP declarations.
void srp_talker_enable(srp_state_t *s, uint8_t enable);
// Add (enable=1) or remove (enable=0) a Listener declaration for stream_id.
// Multiple stream_ids can be active at once (CRF + AAF), up to
// SRP_MAX_LISTENER_STREAMS. Re-enabling an already-active stream_id just
// refreshes it. Returns nothing; silently no-ops if the table is full.
void srp_listener_enable(srp_state_t *s, const uint8_t *stream_id, uint8_t enable);

// 1 if any enabled listener currently has a registered remote talker.
int  srp_any_talker_registered(const srp_state_t *s);

// Process received MSRP frame (called from main RX dispatch).
void srp_process_rx(srp_state_t *s, const uint8_t *frame, uint32_t len);

// Poll — call from main loop. Sends periodic MSRP PDUs.
void srp_poll(srp_state_t *s);

#endif // SRP_H
