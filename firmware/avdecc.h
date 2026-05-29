// IEEE 1722.1 AVDECC — minimal endpoint
// ADP (discovery) + ACMP (connection management)
//
// Uses AVTP EtherType (0x22F0) with subtypes 0x7A (ADP) and 0x7C (ACMP).
// Multicast to 91:E0:F0:01:00:00.

#ifndef AVDECC_H
#define AVDECC_H

#include <stdint.h>
#include "gptp.h"   // gptp_t — surfaced via avdecc_set_gptp()
#include "mcr.h"    // mcr_state_t — surfaced via avdecc_set_mcr()

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

// AVDECC uses the AVTP EtherType
#define AVDECC_ETHERTYPE        0x22F0

// AVTP subtypes for AVDECC. Byte 0 of an AVTP control PDU is cd(1)|subtype(7);
// for ADP/AECP/ACMP the cd bit MUST be 1, so the on-the-wire byte is 0xFA/0xFB/0xFC
// (not 0x7A/0x7B/0x7C). Hive/L-Acoustics drops frames with cd=0 as malformed.
#define AVTP_SUBTYPE_ADP        0xFA
#define AVTP_SUBTYPE_AECP       0xFB
#define AVTP_SUBTYPE_ACMP       0xFC

// AVDECC multicast address (IEEE 1722.1-2013 Table 7.1)
#define AVDECC_MCAST_ADDR       {0x91, 0xE0, 0xF0, 0x01, 0x00, 0x00}

// ADP message types
#define ADP_MSG_ENTITY_AVAILABLE    0
#define ADP_MSG_ENTITY_DEPARTING    1
#define ADP_MSG_ENTITY_DISCOVER     2

// ADP valid_time: in 2-second units.  10 = 20 seconds before expiry.
#define ADP_VALID_TIME              10
#define ADP_ADVERTISE_PERIOD_MS     10000   // Send every 10s (< valid_time * 2)

// ADP Entity Capabilities (IEEE 1722.1-2013 Table 6.2, cross-checked with
// jdksavdecc_adp.h). Bit numbers in the spec are MSB-counted; these constants
// use the on-the-wire (network-byte-order) numeric values from jdksavdecc.
// AEM_SUPPORTED is mandatory for Hive/L-Acoustics controllers to issue
// READ_DESCRIPTOR — without it the entity appears discovered-but-empty.
#define ADP_CAP_EFU_MODE                        0x00000001UL
#define ADP_CAP_ADDRESS_ACCESS_SUPPORTED        0x00000002UL
#define ADP_CAP_GATEWAY_ENTITY                  0x00000004UL
#define ADP_CAP_AEM_SUPPORTED                   0x00000008UL
#define ADP_CAP_LEGACY_AVC                      0x00000010UL
#define ADP_CAP_ASSOCIATION_ID_SUPPORTED        0x00000020UL
#define ADP_CAP_ASSOCIATION_ID_VALID            0x00000040UL
#define ADP_CAP_VENDOR_UNIQUE_SUPPORTED         0x00000080UL
#define ADP_CAP_CLASS_A_SUPPORTED               0x00000100UL
#define ADP_CAP_CLASS_B_SUPPORTED               0x00000200UL
#define ADP_CAP_GPTP_SUPPORTED                  0x00000400UL
#define ADP_CAP_AEM_AUTH_SUPPORTED              0x00000800UL
#define ADP_CAP_AEM_AUTH_REQUIRED               0x00001000UL
#define ADP_CAP_AEM_PERSISTENT_ACQ_SUPPORTED    0x00002000UL
#define ADP_CAP_AEM_IDENTIFY_CTRL_INDEX_VALID   0x00004000UL
// Tells the controller that ADP's interface_index field is meaningful
// (not just left as 0). Required for the controller to associate ADP
// state with our AVB_INTERFACE descriptor. Hive flags entities without
// this even though our interface_index=0 was already correct.
#define ADP_CAP_AEM_INTERFACE_INDEX_VALID       0x00008000UL

// ADP Talker Capabilities
#define ADP_TALKER_CAP_IMPLEMENTED              (1u << 0)
#define ADP_TALKER_CAP_AUDIO_SOURCE             (1u << 14)

// ADP Listener Capabilities
#define ADP_LISTENER_CAP_IMPLEMENTED            (1u << 0)
// MEDIA_CLOCK_SINK (bit 11) is required for CRF listeners. session_mgr2
// sets only this bit (its listener is CRF-only). Without it, talkers do
// not deliver CRF streams to us — verified against la_avdecc enum
// ListenerCapability::MediaClockSink = 1u << 11.
#define ADP_LISTENER_CAP_MEDIA_CLOCK_SINK       (1u << 11)
#define ADP_LISTENER_CAP_AUDIO_SINK             (1u << 14)

// ACMP message types
#define ACMP_MSG_CONNECT_TX_COMMAND             0
#define ACMP_MSG_CONNECT_TX_RESPONSE            1
#define ACMP_MSG_DISCONNECT_TX_COMMAND          2
#define ACMP_MSG_DISCONNECT_TX_RESPONSE         3
#define ACMP_MSG_GET_TX_STATE_COMMAND           4
#define ACMP_MSG_GET_TX_STATE_RESPONSE          5
#define ACMP_MSG_CONNECT_RX_COMMAND             6
#define ACMP_MSG_CONNECT_RX_RESPONSE            7
#define ACMP_MSG_DISCONNECT_RX_COMMAND          8
#define ACMP_MSG_DISCONNECT_RX_RESPONSE         9
#define ACMP_MSG_GET_RX_STATE_COMMAND           10
#define ACMP_MSG_GET_RX_STATE_RESPONSE          11

// ACMP status codes
#define ACMP_STATUS_SUCCESS                     0
#define ACMP_STATUS_LISTENER_UNKNOWN_ID         1
#define ACMP_STATUS_TALKER_UNKNOWN_ID           2
#define ACMP_STATUS_TALKER_DEST_MAC_FAIL        3
#define ACMP_STATUS_TALKER_NO_STREAM_INDEX      4
#define ACMP_STATUS_TALKER_NO_BANDWIDTH         5
#define ACMP_STATUS_TALKER_EXCLUSIVE            6
#define ACMP_STATUS_LISTENER_TALKER_TIMEOUT     7
#define ACMP_STATUS_LISTENER_EXCLUSIVE          8
#define ACMP_STATUS_STATE_UNAVAILABLE           9
#define ACMP_STATUS_NOT_CONNECTED               10
#define ACMP_STATUS_NO_SUCH_CONNECTION          11
#define ACMP_STATUS_NOT_SUPPORTED               31

// ACMP flags
#define ACMP_FLAG_CLASS_B                       (1u << 0)
#define ACMP_FLAG_FAST_CONNECT                  (1u << 1)
#define ACMP_FLAG_SAVED_STATE                   (1u << 2)
#define ACMP_FLAG_STREAMING_WAIT                (1u << 3)
#define ACMP_FLAG_SUPPORTS_ENCRYPTED            (1u << 4)

// AECP message types
#define AECP_MSG_AEM_COMMAND                    0
#define AECP_MSG_AEM_RESPONSE                   1
#define AECP_MSG_VENDOR_UNIQUE_COMMAND          6
#define AECP_MSG_VENDOR_UNIQUE_RESPONSE         7

// Milan Vendor Unique (MVU) — Avnu OUI-36 (00:1B:C5:0A:C) + ProtocolUniqueID 0x100
// = 00:1B:C5:0A:C1:00. Hive treats us as Milan-base capable only after we
// answer MvuCommandType::GetMilanInfo via this protocol_id.
#define MVU_PROTOCOL_ID                         { 0x00, 0x1B, 0xC5, 0x0A, 0xC1, 0x00 }

// MVU command types (Milan 1.3 Clause 5.4.3.2.3)
#define MVU_CMD_GET_MILAN_INFO                  0x0000
#define MVU_CMD_SET_SYSTEM_UNIQUE_ID            0x0001
#define MVU_CMD_GET_SYSTEM_UNIQUE_ID            0x0002
#define MVU_CMD_SET_MEDIA_CLOCK_REFERENCE_INFO  0x0003
#define MVU_CMD_GET_MEDIA_CLOCK_REFERENCE_INFO  0x0004
#define MVU_CMD_BIND_STREAM                     0x0005
#define MVU_CMD_UNBIND_STREAM                   0x0006
#define MVU_CMD_GET_STREAM_INPUT_INFO_EX        0x0007

// AECP/AEM status codes (IEEE 1722.1-2013 Table 7.127)
#define AECP_STATUS_SUCCESS                     0
#define AECP_STATUS_NOT_IMPLEMENTED             1
#define AECP_STATUS_NO_SUCH_DESCRIPTOR          2
#define AECP_STATUS_ENTITY_LOCKED               3
#define AECP_STATUS_ENTITY_ACQUIRED             4
#define AECP_STATUS_BAD_ARGUMENTS               7
#define AECP_STATUS_STREAM_IS_RUNNING           10

// AEM command types (IEEE 1722.1-2013 Table 7.126)
#define AEM_CMD_ACQUIRE_ENTITY                  0x0000
#define AEM_CMD_LOCK_ENTITY                     0x0001
#define AEM_CMD_READ_DESCRIPTOR                 0x0004
#define AEM_CMD_SET_STREAM_FORMAT               0x0008
#define AEM_CMD_GET_STREAM_FORMAT               0x0009
#define AEM_CMD_GET_STREAM_INFO                 0x000F
#define AEM_CMD_GET_NAME                        0x0019
#define AEM_CMD_SET_CLOCK_SOURCE                0x0016
#define AEM_CMD_GET_CLOCK_SOURCE                0x0017
#define AEM_CMD_REGISTER_UNSOLICITED            0x0024
#define AEM_CMD_DEREGISTER_UNSOLICITED          0x0025
#define AEM_CMD_GET_AVB_INFO                    0x0027
#define AEM_CMD_GET_AS_PATH                     0x0028
#define AEM_CMD_GET_COUNTERS                    0x0029
#define AEM_CMD_GET_AUDIO_MAP                   0x002B
#define AEM_CMD_GET_MAX_TRANSIT_TIME            0x004D

// AEM_STREAM_INFO flags (IEEE 1722.1-2013 Table 7.16, la_avdecc src/protocol)
// — included in GET_STREAM_INFO responses to advertise which fields are
// populated. The "CONNECTED" bit is set on a listener when the controller
// has issued CONNECT_RX for it; STREAM_VLAN_ID_VALID + others are set on
// our talker because we always know dest_mac/stream_id/VLAN.
#define STREAM_INFO_FLAG_STREAM_FORMAT_VALID       0x80000000UL
#define STREAM_INFO_FLAG_STREAM_ID_VALID           0x40000000UL
#define STREAM_INFO_FLAG_MSRP_ACC_LATENCY_VALID    0x20000000UL
#define STREAM_INFO_FLAG_STREAM_DEST_MAC_VALID     0x10000000UL
#define STREAM_INFO_FLAG_MSRP_FAILURE_VALID        0x08000000UL
// Per la_avdecc protocolAemEnums.hpp StreamInfoFlags — values were
// previously swapped, making our talker advertise CONNECTED with
// VLAN_ID bit clear. Hive then showed "Stream Vlan ID No Value" even
// though we always emit the VLAN tag on the wire (PCP=3 VID=2).
#define STREAM_INFO_FLAG_STREAM_VLAN_ID_VALID      0x02000000UL
#define STREAM_INFO_FLAG_CONNECTED                 0x04000000UL
#define STREAM_INFO_FLAG_STREAMING_WAIT            0x00008000UL
#define STREAM_INFO_FLAG_CLASS_B                   0x00004000UL
// Per IEEE 1722.1-2013 §7.4.16.4 low byte — set by listeners on a
// CONNECT_RX so a controller restart can re-bind via FAST_CONNECT.
// Without these flags Hive treats the connection as "current session
// only" and shows the input as "running" after a Hive reload.
#define STREAM_INFO_FLAG_SRP_REGISTRATION_FAILED   0x00000040UL
#define STREAM_INFO_FLAG_SAVED_STATE               0x00000004UL
#define STREAM_INFO_FLAG_FAST_CONNECT              0x00000002UL

// AEM descriptor types (IEEE 1722.1-2013 Table 7.1, cross-checked with
// jdksavdecc-c/include/jdksavdecc_aem_descriptor.h)
#define AEM_DESC_ENTITY                         0x0000
#define AEM_DESC_CONFIGURATION                  0x0001
#define AEM_DESC_AUDIO_UNIT                     0x0002
#define AEM_DESC_STREAM_INPUT                   0x0005
#define AEM_DESC_STREAM_OUTPUT                  0x0006
#define AEM_DESC_AVB_INTERFACE                  0x0009
#define AEM_DESC_CLOCK_SOURCE                   0x000A
#define AEM_DESC_LOCALE                         0x000C
#define AEM_DESC_STRINGS                        0x000D
#define AEM_DESC_STREAM_PORT_INPUT              0x000E
#define AEM_DESC_STREAM_PORT_OUTPUT             0x000F
#define AEM_DESC_AUDIO_CLUSTER                  0x0014
#define AEM_DESC_AUDIO_MAP                      0x0017
#define AEM_DESC_CLOCK_DOMAIN                   0x0024

// Stream-format identifiers (IEEE 1722-2016 + 1722.1-2013 Annex A).
// Byte order: octet 0 is subtype.
//
// CRF audio-sample 48000 Hz, pull 1/1, 6 timestamps/PDU.
//   octet 0 = 0x04 (CRF subtype)
//   octets 1..2 = type/timestamp_interval
//   octet 3 = timestamps_per_pdu = 6 (0x06)? actually session_mgr.aemt
//   uses 0x10 here (=16). Format word taken byte-exact from
//   /home/lisp/avdecc-endpoint/models/session_mgr.aemt stream_input[0].
#define STREAM_FMT_CRF_48K \
    { 0x04, 0x10, 0x60, 0x01, 0x00, 0x00, 0xBB, 0x80 }
// AAF PCM 48000 Hz, INT_32BIT, 8 channels, 6 samples/frame (class A).
// Verified by decoding session_mgr.aemt stream_output[0].formats[1]:
//   ch_msb<<2 | ch_lsb = 0x02<<2 | 0 = 8;  samples = (0<<4)|6 = 6
#define STREAM_FMT_AAF_8CH_48K \
    { 0x02, 0x05, 0x02, 0x20, 0x02, 0x00, 0x60, 0x00 }

// AVB_INTERFACE flags (IEEE 1722.1 Table 7.84)
#define AVB_INTERFACE_FLAG_GPTP_GRANDMASTER     (1u << 0)
#define AVB_INTERFACE_FLAG_GPTP_SUPPORTED       (1u << 1)
#define AVB_INTERFACE_FLAG_SRP_SUPPORTED        (1u << 2)

// STREAM flags (IEEE 1722.1 Table 7.21)
#define STREAM_FLAG_CLOCK_SYNC_SOURCE           (1u << 0)
#define STREAM_FLAG_CLASS_A                     (1u << 1)
#define STREAM_FLAG_CLASS_B                     (1u << 2)

// STREAM_PORT flags
#define STREAM_PORT_FLAG_CLOCK_SYNC_SOURCE      (1u << 0)
#define STREAM_PORT_FLAG_ASYNC_SAMPLE_RATE_CONV (1u << 1)
#define STREAM_PORT_FLAG_SYNC_SAMPLE_RATE_CONV  (1u << 2)

// CLOCK_SOURCE type (IEEE 1722.1 Table 7.87)
#define CLOCK_SOURCE_TYPE_INTERNAL              0x0000
#define CLOCK_SOURCE_TYPE_EXTERNAL              0x0001
#define CLOCK_SOURCE_TYPE_INPUT_STREAM          0x0002

// AUDIO_CLUSTER format (IEEE 1722.1 Table 7.143)
#define AUDIO_CLUSTER_FORMAT_IEC_60958          0x00
#define AUDIO_CLUSTER_FORMAT_MBLA               0x40
#define AUDIO_CLUSTER_FORMAT_MIDI               0x80
#define AUDIO_CLUSTER_FORMAT_SMPTE              0x88

// ---------------------------------------------------------------------------
// AVDECC state
// ---------------------------------------------------------------------------

// Talker stream — we are the source. One per stream_output descriptor.
typedef struct {
    uint8_t  stream_id[8];        // our advertised stream_id
    uint8_t  dest_mac[6];         // our advertised stream_dest_mac
    // Connected listener (most recent CONNECT_TX)
    uint8_t  listener_id[8];
    uint16_t listener_uid;
    uint8_t  connected;
    uint16_t connection_count;
} avdecc_talker_stream_t;

// Listener stream — we are the sink. One per stream_input descriptor.
typedef struct {
    uint8_t  talker_id[8];        // remote talker entity (when connected)
    uint16_t talker_uid;
    uint8_t  stream_id[8];        // remote talker's stream_id
    uint8_t  dest_mac[6];         // multicast dest MAC of the stream
    uint8_t  connected;
    uint16_t connection_count;
    // MSRP-learned (filled when the talker's TalkerAdvertise has been
    // observed via SRP). Used by GET_STREAM_INFO / GetStreamInputInfoEx
    // to report wire-truth instead of hardcoded defaults. Zero when no
    // SRP advertisement has matched yet.
    uint32_t msrp_accumulated_latency_ns;
    uint16_t stream_vlan_id;
} avdecc_listener_stream_t;

#define AVDECC_MAX_TALKERS    1   // N_STREAM_OUTPUTS in avdecc.c
#define AVDECC_MAX_LISTENERS  2   // N_STREAM_INPUTS  in avdecc.c

// Slow-path resolve state (IEEE 1722.1 §8.2.2 Path B). When CONNECT_RX
// arrives with stream_id=0 + dst_mac=0, we (the listener) MUST send an
// ACMP CONNECT_TX_COMMAND to the talker to learn its real stream_id +
// dest_mac. The original controller's CONNECT_RX_RESPONSE is DEFERRED
// until we get the talker's CONNECT_TX_RESPONSE (or until timeout).
typedef struct {
    uint8_t  active;                // 1 = waiting for talker's CONNECT_TX_RESPONSE
    uint8_t  listener_uid;          // our listener that needs resolve
    uint16_t our_seq_id;            // sequence_id of OUR outgoing CONNECT_TX_COMMAND
    uint8_t  ctrl_eid_orig[8];      // original controller's entity_id
    uint16_t ctrl_seq_id_orig;      // original controller's sequence_id (echo back)
    uint8_t  talker_id[8];
    uint16_t talker_uid;
    uint32_t start_ms;              // gptp_uptime_ms() when we sent CONNECT_TX
} avdecc_resolve_t;

typedef struct {
    // Identity
    uint8_t  entity_id[8];      // Typically MAC + 0x0000 + unique_id
    uint8_t  src_mac[6];

    // ADP state
    uint32_t adp_available_index;   // Increments each advertisement
    uint32_t last_adp_ms;
    uint8_t  boot_announce_done;    // first poll: send DEPARTING then AVAILABLE

    // Per-stream connection state (sized by descriptor counts in avdecc.c)
    avdecc_talker_stream_t   talkers  [AVDECC_MAX_TALKERS];
    avdecc_listener_stream_t listeners[AVDECC_MAX_LISTENERS];

    // Clock domain — written by SET_CLOCK_SOURCE, read by build_desc_clock_domain
    // 0 = Internal oscillator, 1 = Media Clock (CRF stream input)
    uint16_t current_clock_source;

    // Callbacks (set by caller to wire AVTP/SRP). `uid` is the stream's
    // descriptor index (talker_unique_id or listener_unique_id).
    void (*on_talker_connect)   (uint16_t uid, const uint8_t *listener_entity_id);
    void (*on_talker_disconnect)(uint16_t uid);
    void (*on_listener_connect) (uint16_t uid, const uint8_t *stream_id,
                                 const uint8_t *dest_mac,
                                 const uint8_t *talker_entity_id);
    void (*on_listener_disconnect)(uint16_t uid);
    // Fired when SET_CLOCK_SOURCE picks a new source for clock_domain 0.
    // src_idx: 0 = Internal oscillator, 1 = Media Clock (CRF stream).
    void (*on_clock_source_change)(uint16_t src_idx);

    // Stats
    uint32_t adp_tx_count;
    uint32_t acmp_rx_count;
    uint32_t acmp_tx_count;
    uint32_t aecp_rx_count;
    uint32_t aecp_tx_count;

    // CLOCK_DOMAIN counters (Milan §5.3.11.2). Monotonic — increment on
    // each gPTP-lock transition. Hive shows the CLOCK_DOMAIN indicator
    // green when LOCKED > UNLOCKED, yellow when LOCKED == UNLOCKED.
    uint32_t clock_locked_count;
    uint32_t clock_unlocked_count;
    uint8_t  clock_last_locked;     // last sampled gptp.servo_locked

    // STREAM_INPUT counters (Milan §5.3.12). Per-listener MEDIA_LOCKED /
    // MEDIA_UNLOCKED transitions + FRAMES_RX. Hive reads these to flip
    // the listener indicator green and report "Media Locked N" — stays
    // 0 if we never increment, even when AVTP is actually streaming.
    uint32_t stream_media_locked  [AVDECC_MAX_LISTENERS];
    uint32_t stream_media_unlocked[AVDECC_MAX_LISTENERS];
    uint32_t stream_frames_rx     [AVDECC_MAX_LISTENERS];
    uint8_t  stream_last_locked   [AVDECC_MAX_LISTENERS];

    // ACMP slow-path resolve state. One per listener UID.
    avdecc_resolve_t resolves[AVDECC_MAX_LISTENERS];
    uint16_t next_acmp_seq;         // free-running counter for our outgoing ACMP commands

    // CRF data-flow re-bootstrap watchdog state (see avdecc_crf_flow_watchdog).
    uint32_t crf_wd_rx_snapshot;    // last observed listener rx_count
    uint32_t crf_wd_flow_ms;        // last time rx_count advanced
    uint32_t crf_wd_last_retry_ms;  // last CONNECT_TX re-issue
    uint8_t  crf_wd_recovery;       // 1 = in re-bootstrap (DISCONNECT sent)
    uint8_t  crf_wd_init;           // baseline captured since connect

    // Registered unsolicited-notification controllers (IEEE 1722.1-2013
    // §7.4.37). REGISTER_UNSOLICITED_NOTIFICATION adds a (controller_eid,
    // src_mac) tuple; on state changes (clock lock flip, listener
    // connect/disconnect) we push an AEM response with the U flag set
    // to each tuple. Without this Hive only refreshes UI on manual
    // re-discover — CLOCK_DOMAIN stays yellow until the user clicks.
    #define AVDECC_MAX_UNSOL_CTRL  4
    struct {
        uint8_t  active;
        uint8_t  controller_eid[8];
        uint8_t  mac[6];
        // Per-controller monotonic sequence_id. IEEE 1722.1-2013
        // §7.4.37: each registered controller's unsolicited stream
        // must be contiguous. A single global counter shared across
        // controllers makes Hive log "Unsolicited notification lost
        // detected" because it sees gaps from pushes to other ctrls.
        uint16_t unsol_seq_id;
    } unsol_ctrl[AVDECC_MAX_UNSOL_CTRL];
} avdecc_state_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// Initialize state and derive entity_id from MAC. Per-talker stream_id +
// dest_mac are set separately so callers can populate each stream output.
void avdecc_init(avdecc_state_t *s, const uint8_t *mac_addr);

// Configure the local talker stream `uid` (must be < AVDECC_MAX_TALKERS).
void avdecc_set_talker_stream(avdecc_state_t *s, uint16_t uid,
                              const uint8_t *stream_id,
                              const uint8_t *stream_dest_mac);

// Provide a pointer to the gPTP state so ADP and AVB_INTERFACE responses
// reflect the actual grandmaster identity learned from Announce messages.
// Pass NULL to detach (advertised GM falls back to all-zeros).
void avdecc_set_gptp(const gptp_t *g);

// Provide a pointer to the MCR state so CLOCK_DOMAIN counters report
// the right "lock" source: when current_clock_source = 1 (Media Clock),
// LOCKED tracks mcr.servo_locked; when source = 0 (Internal), it tracks
// gptp.servo_locked.
void avdecc_set_mcr(const mcr_state_t *m);

// Plumb SRP so GET_AVB_INFO can emit a matching msrp_mapping. Without
// this the listener sees Mappings_count=0, falls back to its own defaults,
// and may report failure 0x13 if they disagree with our actual MSRP TX.
#include "srp.h"
void avdecc_set_srp(const srp_state_t *s);

// Tracker called by the listener data path: notify when a stream input's
// media-lock state changes (CRF: MCR servo_locked transition; AAF: first
// frame after rebind). On transition we bump MEDIA_LOCKED/UNLOCKED and
// push an unsolicited GET_COUNTERS_RESPONSE so Hive flips green without
// manual refresh — the missing piece for Milan probing → Completed.
void avdecc_listener_lock_changed(avdecc_state_t *s, uint16_t uid, uint8_t locked);
// Notify on every received AVTP frame for FRAMES_RX bookkeeping. Cheap
// — increments only; no unsolicited push (Hive polls counter values).
void avdecc_listener_frame_rx   (avdecc_state_t *s, uint16_t uid);

// Process received AVDECC frame (called from RX dispatch for EtherType 0x22F0
// with subtypes 0x7A-0x7C).
void avdecc_process_rx(avdecc_state_t *s, const uint8_t *frame, uint32_t len);

// Poll — sends periodic ADP advertisements.
void avdecc_poll(avdecc_state_t *s);

// CRF data-flow re-bootstrap watchdog. Call once per main loop with the
// listener's running CRF rx-frame count (mcr.rx_count) and gptp_uptime_ms().
// Re-triggers a stalled-but-connected talker (Auvitran LeaveAll expiry).
void avdecc_crf_flow_watchdog(avdecc_state_t *s, uint16_t luid,
                              uint32_t rx_count, uint32_t now_ms);

// Send ADP departing message (call before shutdown).
void avdecc_depart(avdecc_state_t *s);

#endif // AVDECC_H
