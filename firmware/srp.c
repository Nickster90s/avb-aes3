// IEEE 802.1Qat SRP (Stream Reservation Protocol) — minimal endpoint
//
// Implements MSRP over MRP PDU encoding for a simple AVB endpoint:
// - Domain declaration (SR Class A, priority 3, VLAN 2)
// - Talker Advertise (when talker is enabled)
// - Listener Ready (when listener is enabled)
// - RX: parse Domain, Talker Advertise, Listener declarations
//
// This is a simplified "declare and forget" implementation: we send
// periodic JoinIn messages without the full MRP applicant/registrar
// state machine. This is sufficient for interop with standard AVB
// switches which will accept our declarations and maintain reservations.

#include "srp.h"
#include "gptp.h"

#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>
#include <string.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Byte-order helpers
// ---------------------------------------------------------------------------

static inline void srp_put_be16(uint8_t *p, uint16_t v)
{
    p[0] = (v >> 8) & 0xFF;
    p[1] = v & 0xFF;
}

static inline void srp_put_be32(uint8_t *p, uint32_t v)
{
    p[0] = (v >> 24) & 0xFF;
    p[1] = (v >> 16) & 0xFF;
    p[2] = (v >>  8) & 0xFF;
    p[3] = v & 0xFF;
}

static inline uint16_t srp_get_be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static inline uint32_t srp_get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] <<  8) | p[3];
}

// ---------------------------------------------------------------------------
// TX — shared MAC access
// ---------------------------------------------------------------------------

static uint32_t srp_txslot;

static uint8_t *srp_tx_buf(void)
{
    return (uint8_t *)(ETHMAC_BASE + ETHMAC_SLOT_SIZE * (ETHMAC_RX_SLOTS + srp_txslot));
}

static void srp_eth_send(uint32_t len)
{
    while (!ethmac_sram_reader_ready_read())
        ;
    ethmac_sram_reader_slot_write(srp_txslot);
    ethmac_sram_reader_length_write(len);
    ethmac_sram_reader_start_write(1);
    srp_txslot = (srp_txslot + 1) % ETHMAC_TX_SLOTS;
}

// ---------------------------------------------------------------------------
// MSRP PDU builder
// ---------------------------------------------------------------------------

// Start an MSRP Ethernet frame.  Returns pointer to MRPDU ProtocolVersion byte.
static uint8_t *msrp_frame_begin(uint8_t *frame, const uint8_t *src_mac)
{
    static const uint8_t msrp_mcast[] = MSRP_MCAST_ADDR;

    // Ethernet header
    memcpy(frame, msrp_mcast, 6);
    memcpy(frame + 6, src_mac, 6);
    srp_put_be16(frame + 12, MSRP_ETHERTYPE);

    // MRPDU header
    frame[14] = MSRP_PROTO_VERSION;  // ProtocolVersion = 0

    return frame + 15;  // Points to start of MessageList
}

// Write a Domain message: AttributeType=4, AttributeLength=4
// FirstValue = {SRclassID, SRclassPriority, SRclassVID(2 bytes)}
// One vector with one JoinIn event.
static uint8_t *msrp_emit_domain(uint8_t *p, uint8_t sr_class, uint8_t sr_prio,
                                  uint16_t sr_vid, int leaveall)
{
    // AttributeType
    *p++ = MSRP_ATTR_DOMAIN;
    // AttributeLength
    *p++ = 4;
    // AttributeListLength (2 bytes) — we'll fill after
    uint8_t *list_len_ptr = p;
    p += 2;

    uint8_t *vec_start = p;

    // VectorHeader: LeaveAll(1 bit) << 13 | NumberOfValues(13 bits)
    uint16_t vec_hdr = 1;  // NumberOfValues = 1
    if (leaveall)
        vec_hdr |= (1 << 13);
    srp_put_be16(p, vec_hdr);
    p += 2;

    // FirstValue (4 bytes): SRclassID, SRclassPriority, SRclassVID
    *p++ = sr_class;
    *p++ = sr_prio;
    srp_put_be16(p, sr_vid);
    p += 2;

    // ThreePackedEvents: one event (JoinIn), packed as 3pack with padding
    *p++ = MRP_3PACK(MRP_EVT_JOININ, 0, 0);

    // End of vectors for this attribute — write AttributeListLength
    uint16_t list_len = (uint16_t)(p - vec_start);
    srp_put_be16(list_len_ptr, list_len);

    // EndMark for this message
    srp_put_be16(p, 0);
    p += 2;

    return p;
}

// Write a TalkerAdvertise message: AttributeType=1, AttributeLength=25
static uint8_t *msrp_emit_talker_adv(uint8_t *p, const srp_talker_attr_t *t,
                                      int leaveall)
{
    // AttributeType
    *p++ = MSRP_ATTR_TALKER_ADV;
    // AttributeLength
    *p++ = 25;
    // AttributeListLength
    uint8_t *list_len_ptr = p;
    p += 2;

    uint8_t *vec_start = p;

    // VectorHeader
    uint16_t vec_hdr = 1;
    if (leaveall)
        vec_hdr |= (1 << 13);
    srp_put_be16(p, vec_hdr);
    p += 2;

    // FirstValue (25 bytes):
    // StreamID (8)
    memcpy(p, t->stream_id, 8);
    p += 8;
    // DataFrameParameters: DestAddr (6) + VlanID (2)
    memcpy(p, t->dest_addr, 6);
    p += 6;
    srp_put_be16(p, t->vlan_id);
    p += 2;
    // TSpec: MaxFrameSize (2) + MaxIntervalFrames (2)
    srp_put_be16(p, t->max_frame_size);
    p += 2;
    srp_put_be16(p, t->max_interval_frames);
    p += 2;
    // PriorityAndRank (1)
    *p++ = t->priority_and_rank;
    // AccumulatedLatency (4)
    srp_put_be32(p, t->accumulated_latency_ns);
    p += 4;

    // ThreePackedEvents: JoinIn
    *p++ = MRP_3PACK(MRP_EVT_JOININ, 0, 0);

    uint16_t list_len = (uint16_t)(p - vec_start);
    srp_put_be16(list_len_ptr, list_len);

    // EndMark
    srp_put_be16(p, 0);
    p += 2;

    return p;
}

// Write a Listener message: AttributeType=3, AttributeLength=8
static uint8_t *msrp_emit_listener(uint8_t *p, const uint8_t *stream_id,
                                    uint8_t substate, int leaveall)
{
    // AttributeType
    *p++ = MSRP_ATTR_LISTENER;
    // AttributeLength
    *p++ = 8;
    // AttributeListLength
    uint8_t *list_len_ptr = p;
    p += 2;

    uint8_t *vec_start = p;

    // VectorHeader
    uint16_t vec_hdr = 1;
    if (leaveall)
        vec_hdr |= (1 << 13);
    srp_put_be16(p, vec_hdr);
    p += 2;

    // FirstValue (8 bytes): StreamID
    memcpy(p, stream_id, 8);
    p += 8;

    // ThreePackedEvents: JoinIn
    *p++ = MRP_3PACK(MRP_EVT_JOININ, 0, 0);

    // FourPackedEvents: listener substate
    *p++ = MRP_4PACK(substate, 0, 0, 0);

    uint16_t list_len = (uint16_t)(p - vec_start);
    srp_put_be16(list_len_ptr, list_len);

    // EndMark
    srp_put_be16(p, 0);
    p += 2;

    return p;
}

// ---------------------------------------------------------------------------
// TX — send MSRP PDU with all our active declarations
// ---------------------------------------------------------------------------

static void srp_send_declarations(srp_state_t *s, int leaveall)
{
    uint8_t *frame = srp_tx_buf();
    uint8_t *p = msrp_frame_begin(frame, s->src_mac);

    // Always declare Domain (SR Class A)
    p = msrp_emit_domain(p, SR_CLASS_A, SR_CLASS_A_PRIO,
                         SR_CLASS_A_VID, leaveall);

    // Talker Advertise (if enabled)
    if (s->talker_enabled) {
        p = msrp_emit_talker_adv(p, &s->talker, leaveall);
    }

    // Listener Ready (if enabled)
    if (s->listener_enabled) {
        p = msrp_emit_listener(p, s->listener_stream_id,
                               s->listener_substate, leaveall);
    }

    // Final EndMark (end of MRPDU MessageList)
    srp_put_be16(p, 0);
    p += 2;

    uint32_t frame_len = (uint32_t)(p - frame);

    // Pad to minimum Ethernet frame size (64 bytes)
    if (frame_len < 64) {
        memset(p, 0, 64 - frame_len);
        frame_len = 64;
    }

    srp_eth_send(frame_len);
    s->join_count++;
}

// ---------------------------------------------------------------------------
// RX — parse incoming MSRP PDUs
// ---------------------------------------------------------------------------

void srp_process_rx(srp_state_t *s, const uint8_t *frame, uint32_t len)
{
    if (len < 14 + 2)  // Ethernet header + minimum MRPDU
        return;

    // Check ethertype
    if (srp_get_be16(frame + 12) != MSRP_ETHERTYPE)
        return;

    const uint8_t *mrpdu = frame + 14;
    uint32_t mrpdu_len = len - 14;

    // Check protocol version
    if (mrpdu[0] != MSRP_PROTO_VERSION)
        return;

    const uint8_t *p = mrpdu + 1;  // Past ProtocolVersion
    const uint8_t *end = mrpdu + mrpdu_len;

    s->rx_pdu_count++;

    // Parse message list
    while (p + 4 <= end) {
        uint8_t attr_type = p[0];
        uint8_t attr_len  = p[1];

        // EndMark check
        if (attr_type == 0 && attr_len == 0)
            break;

        uint16_t attr_list_len = srp_get_be16(p + 2);
        const uint8_t *attr_end = p + 4 + attr_list_len;

        if (attr_end > end)
            break;

        const uint8_t *vp = p + 4;  // Start of vector attributes

        // Parse vectors within this message
        while (vp + 2 <= attr_end) {
            uint16_t vec_hdr = srp_get_be16(vp);
            vp += 2;

            // EndMark within attribute list
            if (vec_hdr == 0)
                break;

            uint16_t num_values = vec_hdr & 0x1FFF;
            // int lva = (vec_hdr >> 13) & 1;

            // FirstValue
            if (vp + attr_len > attr_end)
                break;

            if (attr_type == MSRP_ATTR_DOMAIN && attr_len >= 4) {
                s->domain_received = 1;
                s->rx_sr_class = vp[0];
                s->rx_sr_prio  = vp[1];
                s->rx_sr_vid   = srp_get_be16(vp + 2);
            }

            if (attr_type == MSRP_ATTR_TALKER_ADV && attr_len >= 25) {
                // TalkerAdvertise FirstValue layout (IEEE 802.1Q-2014 §35):
                //   +0..7   stream_id (8)
                //   +8..13  stream_dest_addr (6)
                //   +14..15 vlan_id
                //   +16..17 max_frame_size
                //   +18..19 max_interval_frames
                //   +20     priority_and_rank
                //   +21..24 accumulated_latency
                //
                // (b) Diagnostic: rate-limited print of each unique
                // stream_id+dest_mac we see. Rate limit = per-stream_id
                // 5-second cache (cheap LRU of size 4). avb_session_mgr2
                // re-advertises every 200 ms × 4 streams; without the
                // cache we'd flood the UART.
                {
                    static uint8_t cache_sid[4][8];
                    static uint32_t cache_t_ms[4];
                    static int cache_n = 0;
                    uint32_t now = gptp_uptime_ms();
                    int found = -1, oldest = 0;
                    uint32_t oldest_t = 0xFFFFFFFFu;
                    for (int i = 0; i < cache_n; i++) {
                        int eq = 1;
                        for (int k = 0; k < 8; k++)
                            if (cache_sid[i][k] != vp[k]) { eq = 0; break; }
                        if (eq) { found = i; break; }
                        if (cache_t_ms[i] < oldest_t) {
                            oldest_t = cache_t_ms[i]; oldest = i;
                        }
                    }
                    int log_it = 0;
                    if (found < 0) {
                        // new stream_id — log + cache
                        int slot = (cache_n < 4) ? cache_n++ : oldest;
                        memcpy(cache_sid[slot], vp, 8);
                        cache_t_ms[slot] = now;
                        log_it = 1;
                    } else if ((now - cache_t_ms[found]) >= 5000) {
                        cache_t_ms[found] = now;
                        log_it = 1;
                    }
                    if (log_it) {
                        printf("[SRP-RX] TalkerAdv sid=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x "
                               "dest=%02x:%02x:%02x:%02x:%02x:%02x\n",
                               vp[0], vp[1], vp[2], vp[3], vp[4], vp[5], vp[6], vp[7],
                               vp[8], vp[9], vp[10], vp[11], vp[12], vp[13]);
                    }
                }

                // Fire the observer callback — main.c uses it to learn
                // stream_ids for FAST_CONNECT bindings that only know
                // the dest_mac at ACMP CONNECT_RX time.
                if (s->on_talker_advertise)
                    s->on_talker_advertise(vp, vp + 8);

                // Check if this talker's stream_id matches what our listener wants
                if (s->listener_enabled) {
                    int match = 1;
                    for (int i = 0; i < 8; i++) {
                        if (vp[i] != s->listener_stream_id[i]) {
                            match = 0;
                            break;
                        }
                    }
                    if (match) {
                        if (!s->talker_registered) {
                            printf("[SRP] Talker registered for our stream\n");
                        }
                        s->talker_registered = 1;
                        s->talker_last_seen_ms = gptp_uptime_ms();
                        s->listener_substate = MSRP_LISTENER_READY;
                    }
                }
            }

            // Skip past FirstValue + packed events
            // Each 3pack byte encodes 3 events, 4pack byte encodes 4
            vp += attr_len;
            uint32_t num_3pack = (num_values + 2) / 3;
            vp += num_3pack;

            // Listener has additional 4pack events
            if (attr_type == MSRP_ATTR_LISTENER) {
                uint32_t num_4pack = (num_values + 3) / 4;
                vp += num_4pack;
            }
        }

        p = attr_end;
    }
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void srp_init(srp_state_t *s, const uint8_t *mac_addr)
{
    memset(s, 0, sizeof(*s));
    memcpy(s->src_mac, mac_addr, 6);
    srp_txslot = 0;
    s->listener_substate = MSRP_LISTENER_ASKFAILED;
    printf("[SRP] Initialized (MSRP)\n");
}

void srp_talker_set(srp_state_t *s, const uint8_t *stream_id,
                    const uint8_t *dest_mac, uint16_t max_frame_size)
{
    memcpy(s->talker.stream_id, stream_id, 8);
    memcpy(s->talker.dest_addr, dest_mac, 6);
    s->talker.vlan_id = SR_CLASS_A_VID;
    s->talker.max_frame_size = max_frame_size;
    s->talker.max_interval_frames = 1;  // Class A: 1 frame per 125us interval
    // Priority 3 (Class A), Rank 1 (non-emergency)
    s->talker.priority_and_rank = (SR_CLASS_A_PRIO << 5) | (1 << 4);
    s->talker.accumulated_latency_ns = 0;  // We're the first hop
}

void srp_talker_enable(srp_state_t *s, uint8_t enable)
{
    s->talker_enabled = enable;
    if (enable) {
        printf("[SRP] Talker Advertise enabled\n");
    } else {
        printf("[SRP] Talker Advertise disabled\n");
    }
}

void srp_listener_enable(srp_state_t *s, const uint8_t *stream_id, uint8_t enable)
{
    s->listener_enabled = enable;
    if (enable) {
        memcpy(s->listener_stream_id, stream_id, 8);
        s->listener_substate = MSRP_LISTENER_ASKFAILED;
        s->talker_registered = 0;
        printf("[SRP] Listener enabled for stream %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
               stream_id[0], stream_id[1], stream_id[2], stream_id[3],
               stream_id[4], stream_id[5], stream_id[6], stream_id[7]);
    } else {
        printf("[SRP] Listener disabled\n");
    }
}

// ---------------------------------------------------------------------------
// Poll — periodic MSRP PDU transmission
// ---------------------------------------------------------------------------

void srp_poll(srp_state_t *s)
{
    // Get uptime in ms from gPTP time
    ptp_timestamp_t now = gptp_read_time();
    uint32_t now_ms = (uint32_t)(now.seconds & 0xFFFF) * 1000 +
                      (uint32_t)(now.nanoseconds / 1000000);

    uint32_t elapsed_join = now_ms - s->last_join_ms;
    if (elapsed_join > 2000000000)
        elapsed_join = MRP_JOIN_PERIOD_MS;  // Handle wrap

    uint32_t elapsed_lva = now_ms - s->last_leaveall_ms;
    if (elapsed_lva > 2000000000)
        elapsed_lva = MRP_LEAVEALL_PERIOD_MS;

    // LeaveAll timer
    int leaveall = 0;
    if (elapsed_lva >= MRP_LEAVEALL_PERIOD_MS) {
        leaveall = 1;
        s->last_leaveall_ms = now_ms;
    }

    // (a) Age out talker_registered. If we haven't seen a matching
    // TalkerAdvertise in 3× LeaveAll intervals (~30 s by default), the
    // talker is presumed gone — clear the flag and notify upper layers.
    // Without this, talker_registered stayed sticky forever even after
    // the talker disconnected from the wire.
    if (s->talker_registered) {
        uint32_t age = now_ms - s->talker_last_seen_ms;
        if (age > 2000000000) age = 0;          // wrap guard
        if (age >= (3u * MRP_LEAVEALL_PERIOD_MS)) {
            printf("[SRP] Talker timed out (no advertise in %u ms) — "
                   "clearing registration\n", (unsigned)age);
            s->talker_registered  = 0;
            s->listener_substate  = MSRP_LISTENER_ASKFAILED;
        }
    }

    // Join timer — send declarations periodically
    if (elapsed_join >= MRP_JOIN_PERIOD_MS) {
        srp_send_declarations(s, leaveall);
        s->last_join_ms = now_ms;

        // Periodic status
        if (s->join_count > 0 && (s->join_count % 30) == 0) {
            printf("[SRP] tx=%lu rx=%lu domain=%d talker_reg=%d\n",
                   (unsigned long)s->join_count,
                   (unsigned long)s->rx_pdu_count,
                   s->domain_received,
                   s->talker_registered);
        }
    }
}
