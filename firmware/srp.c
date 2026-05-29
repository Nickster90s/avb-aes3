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
// MVRP PDU builder — registers our port for VLAN 2 (Class A) at the bridge.
// Many AVB bridges gate forwarding of VLAN-tagged stream traffic on MVRP
// port membership: without this, the bridge proxies our MSRP attributes
// but won't actually forward stream data multicast (91:e0:f0:00:*) to our
// port. Open-AVB simple_listener does this via mrp_join_vlan() →
// "V++:I=0002" → mrpd → wire MVRP. session_mgr2.cpp:657 calls it too.
//
// MVRP wire format (IEEE 802.1Q-2018 §11.2):
//   dst MAC = 01:80:c2:00:00:21 (Customer Bridge Group)
//   ethertype = 0x88F5 (untagged)
//   ProtocolVersion=0 | AttrType=1(VID) AttrLen=2 AttrListLen vec_hdr
//   FirstValue=VID(2) 3-pack EndMark | PDU-EndMark
// ---------------------------------------------------------------------------

#define MVRP_ETHERTYPE 0x88F5
static const uint8_t MVRP_MCAST[6] = {0x01, 0x80, 0xC2, 0x00, 0x00, 0x21};

static void mvrp_send_join_vid(srp_state_t *s, uint16_t vid, uint8_t event)
{
    uint8_t *frame = srp_tx_buf();
    uint8_t *p = frame;

    // Ethernet header
    memcpy(p, MVRP_MCAST, 6);    p += 6;
    memcpy(p, s->src_mac, 6);    p += 6;
    srp_put_be16(p, MVRP_ETHERTYPE); p += 2;

    // MVRP ProtocolVersion
    *p++ = 0;

    // Message: AttrType=1 (VID), AttrLen=2
    // NOTE: MVRP does NOT include AttributeListLength here — unlike
    // MSRP/MMRP. Verified by decoding real bridge MVRP frames on the
    // wire: `00 01 02 [vec_hdr][FirstValue][3pack] ... 00 00`.
    // Open-AVB mvrp.c (mvrp_emit_vidvectors line 672) writes only
    // AttributeType+AttributeLength then VectorAttributes directly.
    // Adding a 2-byte list_len here would make the frame malformed
    // and bridges would silently drop it — no VLAN registration.
    *p++ = 1;   // AttributeType = MVRP_VID_TYPE
    *p++ = 2;   // AttributeLength = 2 bytes (VID FirstValue size)

    // VectorHeader: numValues=1, LeaveAll=0
    srp_put_be16(p, 1); p += 2;
    // FirstValue: VID (2 bytes)
    srp_put_be16(p, vid); p += 2;
    // 3-pack event
    *p++ = MRP_3PACK(event, 0, 0);

    // EndMark — terminates the VectorAttribute list, also serves as
    // the PDU EndMark since we only emit one Message per PDU.
    srp_put_be16(p, 0); p += 2;

    uint32_t len = (uint32_t)(p - frame);
    if (len < 64) { memset(p, 0, 64 - len); len = 64; }
    srp_eth_send(len);
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
                                  uint16_t sr_vid, int leaveall, uint8_t event)
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

    // ThreePackedEvents — caller passes MRP_EVT_NEW for first 2 cycles
    // after attach (drives bridge registrar VN→AN→QA), JoinMt otherwise.
    *p++ = MRP_3PACK(event, 0, 0);

    // EndMark INSIDE the AttributeList — per IEEE 802.1Q-2018 §10.8.1.5,
    // AttributeList = VectorAttributes+ + EndMark, and AttributeListLength
    // MUST include the EndMark. Bug previously: computed list_len before
    // writing the EndMark → length off by 2 → bridge parsed our PDU,
    // hit the EndMark at the "next message" position, treated it as PDU
    // EndMark, and silently dropped every subsequent attribute. Result:
    // Listener Ready never reached the bridge's MAD → stream never
    // forwarded to FPGA port. Verified via stream_mcast counter (= 0).
    srp_put_be16(p, 0);
    p += 2;

    uint16_t list_len = (uint16_t)(p - vec_start);
    srp_put_be16(list_len_ptr, list_len);

    return p;
}

// Write a TalkerAdvertise message: AttributeType=1, AttributeLength=25
static uint8_t *msrp_emit_talker_adv(uint8_t *p, const srp_talker_attr_t *t,
                                      int leaveall, uint8_t event)
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

    // ThreePackedEvents — caller passes MRP_EVT_NEW for the first 2
    // cycles after talker_enable so the bridge's registrar initialises;
    // JoinMt for steady-state refresh.
    *p++ = MRP_3PACK(event, 0, 0);

    // EndMark INSIDE AttributeList — AttrListLen must include it
    // (see [[msrp-attrlistlen-must-include-the-inner-endmark]]).
    srp_put_be16(p, 0);
    p += 2;

    uint16_t list_len = (uint16_t)(p - vec_start);
    srp_put_be16(list_len_ptr, list_len);

    return p;
}

// Write a Listener message: AttributeType=3, AttributeLength=8
static uint8_t *msrp_emit_listener(uint8_t *p, const uint8_t *stream_id,
                                    uint8_t substate, int leaveall,
                                    uint8_t event)
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

    // ThreePackedEvents: per-call MRP event. mrpd emits MRPDU_NEW (0) for
    // the first 2 transmissions after S+L (applicant in VN→AN state),
    // then settles to JoinIn/JoinMt. NEW tells the bridge's registrar
    // "fresh attribute, allocate state" — JoinMt is a refresh event that
    // assumes the registrar already knows. Without NEW, the bridge never
    // establishes our listener registration and never tells the talker
    // someone wants the stream.
    *p++ = MRP_3PACK(event, 0, 0);

    // FourPackedEvents: listener substate
    *p++ = MRP_4PACK(substate, 0, 0, 0);

    // EndMark INSIDE AttributeList. AttributeListLength MUST include this
    // EndMark — see [[msrp-attrlistlen-includes-endmark]]. Without it the
    // bridge reads our list as 2 bytes short, mistakes the inner EndMark
    // for the PDU EndMark, and never registers our Listener — Auvitran
    // stays in probing "Registering" and CRF never streams to us.
    // Domain + TalkerAdv emitters already do this; Listener was the bug.
    srp_put_be16(p, 0);
    p += 2;

    uint16_t list_len = (uint16_t)(p - vec_start);
    srp_put_be16(list_len_ptr, list_len);

    return p;
}

// ---------------------------------------------------------------------------
// TX — send MSRP PDU with all our active declarations
// ---------------------------------------------------------------------------

static void srp_send_declarations(srp_state_t *s, int leaveall)
{
    uint8_t *frame = srp_tx_buf();
    uint8_t *p = msrp_frame_begin(frame, s->src_mac);

    // Use the bridge-advertised SR class / priority / VID if we've heard
    // one (mandatory for interop — the bridge may map Class A to a non-3
    // priority on our port and reject reservations as code 0x13 "SR class
    // priority mismatch" if we keep declaring the default). Falls back to
    // 802.1Q defaults until first Domain RX arrives.
    uint8_t  dom_class = s->domain_received ? s->rx_sr_class : SR_CLASS_A;
    uint8_t  dom_prio  = s->domain_received ? s->rx_sr_prio  : SR_CLASS_A_PRIO;
    uint16_t dom_vid   = s->domain_received ? s->rx_sr_vid   : SR_CLASS_A_VID;

    // Domain — emit MRPDU_NEW(0) for the first 2 cycles (VN→AN→QA);
    // then JoinIn(1) for steady state. mrpd encodes the QA-state
    // action as JoinIn (registrar==IN); using JoinMt(3) instead tells
    // bridges "registrar is empty" and they keep the reservation in
    // half-allocated state, replying upstream with MSRP TalkerFailed.
    // See [[msrp-joinmt-vs-joinin]].
    uint8_t dom_event = (s->domain_new_count < 2) ? MRP_EVT_NEW : MRP_EVT_JOININ;
    p = msrp_emit_domain(p, dom_class, dom_prio, dom_vid, leaveall, dom_event);
    if (s->domain_new_count < 2)
        s->domain_new_count++;

    // Talker Advertise (if enabled)
    if (s->talker_enabled) {
        // Keep our talker's priority_and_rank synced with the bridge's
        // advertised priority — protects against per-port priority remap
        // that triggers "SR Class Priority Mismatch" (0x13) at downstream
        // listeners.
        s->talker.priority_and_rank = (uint8_t)((dom_prio << 5) | (1 << 4));
        s->talker.vlan_id = dom_vid;

        uint8_t tk_event = (s->talker_new_count < 2)
                               ? MRP_EVT_NEW : MRP_EVT_JOININ;
        p = msrp_emit_talker_adv(p, &s->talker, leaveall, tk_event);
        if (s->talker_new_count < 2)
            s->talker_new_count++;
    }

    // Listener Ready (if enabled). For the first 2 transmissions after
    // srp_listener_enable, emit MRPDU_NEW (event=0) so the bridge's
    // registrar establishes fresh state for our attribute. After that,
    // switch to JoinMt — mirrors mrpd's VN→AN→QA applicant transitions.
    if (s->listener_enabled) {
        uint8_t event = (s->listener_new_count < 2)
                            ? MRP_EVT_NEW
                            : MRP_EVT_JOININ;
        p = msrp_emit_listener(p, s->listener_stream_id,
                               s->listener_substate, leaveall, event);
        if (s->listener_new_count < 2)
            s->listener_new_count++;
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
                s->rx_domain_count++;
                s->domain_received = 1;
                s->rx_sr_class = vp[0];
                s->rx_sr_prio  = vp[1];
                s->rx_sr_vid   = srp_get_be16(vp + 2);
            }
            if (attr_type == MSRP_ATTR_TALKER_FAIL) {
                s->rx_talker_failed_count++;
                // TalkerFailed FirstValue is 34 bytes:
                //   +0..24  same as TalkerAdv (StreamID, DestAddr,
                //           VID, MaxFrameSize, MaxIntervalFrames,
                //           PriorityAndRank, AccumulatedLatency)
                //  +25..32  failure_bridge_id (8)
                //  +33      failure_code (1) — see IEEE 802.1Q-2018
                //           Table 35-6
                // Rate-limit print to once per unique (stream_id,code)
                // observed since boot so we don't flood UART when the
                // bridge keeps re-asserting the same failure.
                if (attr_len >= 34) {
                    static uint8_t  last_sid[8];
                    static uint8_t  last_code;
                    static uint8_t  printed_once;
                    const uint8_t *sid  = vp;
                    uint8_t code        = vp[33];
                    int same = printed_once && code == last_code;
                    if (same) {
                        for (int j = 0; j < 8; j++)
                            if (sid[j] != last_sid[j]) { same = 0; break; }
                    }
                    if (!same) {
                        printf("[SRP-RX] TalkerFailed sid=%02x:%02x:%02x:%02x:"
                               "%02x:%02x:%02x:%02x bridge=%02x:%02x:%02x:%02x:"
                               "%02x:%02x:%02x:%02x code=0x%02x\n",
                               sid[0], sid[1], sid[2], sid[3],
                               sid[4], sid[5], sid[6], sid[7],
                               vp[25], vp[26], vp[27], vp[28],
                               vp[29], vp[30], vp[31], vp[32],
                               (unsigned)code);
                        memcpy(last_sid, sid, 8);
                        last_code   = code;
                        printed_once = 1;
                    }
                }
            }
            if (attr_type == MSRP_ATTR_LISTENER)
                s->rx_listener_count++;

            if (attr_type == MSRP_ATTR_TALKER_ADV && attr_len >= 25) {
                s->rx_talker_adv_count++;
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

                // Upsert into the per-stream registrar table. Look for
                // an existing matching stream_id; fall back to the
                // oldest entry when full. Fields parsed per IEEE 802.1Q
                // §35 FirstValue layout — see comment above for offsets.
                {
                    int slot = -1, oldest = 0;
                    uint32_t oldest_t = 0xFFFFFFFFu;
                    for (int i = 0; i < SRP_MAX_REMOTE_TALKERS; i++) {
                        srp_remote_talker_t *t = &s->remote_talkers[i];
                        if (t->valid) {
                            int eq = 1;
                            for (int k = 0; k < 8; k++)
                                if (t->stream_id[k] != vp[k]) { eq = 0; break; }
                            if (eq) { slot = i; break; }
                            if (t->last_seen_ms < oldest_t) {
                                oldest_t = t->last_seen_ms; oldest = i;
                            }
                        } else if (slot < 0) {
                            slot = i;     // first free slot
                        }
                    }
                    if (slot < 0) slot = oldest;
                    srp_remote_talker_t *t = &s->remote_talkers[slot];
                    t->valid = 1;
                    memcpy(t->stream_id, vp, 8);
                    memcpy(t->dest_mac,  vp + 8, 6);
                    t->vlan_id              = srp_get_be16(vp + 14);
                    t->max_frame_size       = srp_get_be16(vp + 16);
                    t->max_interval_frames  = srp_get_be16(vp + 18);
                    t->priority_and_rank    = vp[20];
                    t->accumulated_latency_ns =
                        ((uint32_t)vp[21] << 24) | ((uint32_t)vp[22] << 16) |
                        ((uint32_t)vp[23] << 8)  |  (uint32_t)vp[24];
                    t->last_seen_ms = gptp_uptime_ms();
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
                        s->rx_match_count++;
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

const srp_remote_talker_t *srp_find_talker(const srp_state_t *s,
                                            const uint8_t *stream_id)
{
    for (int i = 0; i < SRP_MAX_REMOTE_TALKERS; i++) {
        const srp_remote_talker_t *t = &s->remote_talkers[i];
        if (!t->valid) continue;
        int eq = 1;
        for (int k = 0; k < 8; k++)
            if (t->stream_id[k] != stream_id[k]) { eq = 0; break; }
        if (eq) return t;
    }
    return NULL;
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
        // Reset MRPDU_NEW counter so the next 2 advertises emit NEW(0)
        // and force the bridge's registrar into the registered state.
        s->talker_new_count = 0;
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
        // Declare Ready immediately, mirroring avb_session_mgr2's
        // send_ready() at ACMP CONNECT_RX time. AskingFailed = "I want
        // this stream but cannot reserve" — talkers (Auvitran observed)
        // interpret it as "no listener" and hold back CRF/AAF. Ready =
        // "I want it and can receive"; this is the correct initial
        // substate once ACMP has resolved the stream.
        s->listener_substate = MSRP_LISTENER_READY;
        s->talker_registered = 0;
        // Reset last-seen so the talker-age poll grants the talker a
        // fresh 30 s window before timing out — otherwise the stamp
        // from a previous listener attachment can be ancient.
        s->talker_last_seen_ms = gptp_uptime_ms();
        // Reset the applicant counter so we emit MRPDU_NEW for the next
        // 2 transmissions, telling the bridge this is a fresh listener
        // attachment.
        s->listener_new_count = 0;
        printf("[SRP] Listener Ready for stream %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
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
    // Must match the time base used for t->last_seen_ms / talker_last_seen_ms,
    // which are stamped via gptp_uptime_ms() on TalkerAdvertise RX. Mixing the
    // (sec & 0xFFFF) * 1000 inline form with gptp_uptime_ms() underflows the
    // age calc → flaps registrations every poll. See [[gptp-time-base-consistency-when-computing-ages]].
    uint32_t now_ms = gptp_uptime_ms();

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

    // (a) Age out talker_registered as a DIAGNOSTIC ONLY. If we haven't
    // seen a matching TalkerAdvertise in 3× LeaveAll (~30 s), clear the
    // "talker seen" flag so status reflects reality — but do NOT touch
    // listener_substate.
    //
    // WHY (2026-05-29): downgrading listener_substate → ASKFAILED here
    // was self-defeating. Auvitran (Milan talker) only sources the CRF
    // stream while it sees our Listener=READY (see
    // feedback_listener_ready_immediate). When a transient advertise gap
    // tripped this age-out we declared AskingFailed → Auvitran stopped
    // advertising/sourcing the CRF (sid …:00:04) → we never re-matched →
    // permanent deadlock: Hive shows ACMP CONNECTED but the bridge prunes
    // the stream (red arrow, no black dot), MCR starved, lock lost.
    //
    // The reference stacks (GenAVB srp/msrp.c, avb_session_mgr2's
    // send_ready) keep the Listener READY for as long as the listener
    // WANTS the stream (while listener_enabled / AVDECC-connected), and
    // only drop it on explicit teardown. listener_substate is now owned
    // by srp_listener_enable() — the talker-advertise age-out no longer
    // steers it.
    if (s->talker_registered) {
        uint32_t age = now_ms - s->talker_last_seen_ms;
        if (age > 2000000000) age = 0;          // wrap guard
        if (age >= (3u * MRP_LEAVEALL_PERIOD_MS)) {
            printf("[SRP] Talker advertise gap %u ms — clearing 'seen' flag "
                   "(listener stays READY)\n", (unsigned)age);
            s->talker_registered = 0;
        }
    }

    // (a') Keepalive: while the listener is enabled (AVDECC-connected),
    // hold the substate at READY. Mirrors avb_session_mgr2's periodic
    // "force-refresh the CRF listener attachment" — keeps telling the
    // bridge/talker we want the stream so Auvitran sources it (and
    // re-sources it the moment it returns after any gap). Only
    // srp_listener_enable(enable=0) on AVDECC DISCONNECT clears it.
    if (s->listener_enabled && s->listener_substate != MSRP_LISTENER_READY) {
        s->listener_substate  = MSRP_LISTENER_READY;
        s->listener_new_count = 0;   // re-emit MRPDU_NEW on re-assert
    }

    // Age out the per-stream registrar table on the same schedule.
    for (int i = 0; i < SRP_MAX_REMOTE_TALKERS; i++) {
        srp_remote_talker_t *t = &s->remote_talkers[i];
        if (!t->valid) continue;
        uint32_t age = now_ms - t->last_seen_ms;
        if (age > 2000000000) age = 0;
        if (age >= (3u * MRP_LEAVEALL_PERIOD_MS)) {
            t->valid = 0;
        }
    }

    // Join timer — send declarations periodically
    if (elapsed_join >= MRP_JOIN_PERIOD_MS) {
        srp_send_declarations(s, leaveall);

        // MVRP TX — registers our port on VLAN 2 (Class A) at the
        // bridge. Without this, the bridge has no record that our
        // port wants VLAN-2 traffic; it accepts our Listener Ready
        // but cannot forward Auvitran's stream to us, and reports
        // back to the talker as TalkerFailed. avb_session works
        // because Linux's kernel handles MVRP automatically; the
        // FPGA must do it itself. Pattern: NEW for the first 2
        // cycles (VN→AN→QA), then JoinMt for refresh.
        uint8_t mvrp_evt = (s->mvrp_new_count < 2)
                               ? MRP_EVT_NEW : MRP_EVT_JOININ;
        mvrp_send_join_vid(s, SR_CLASS_A_VID, mvrp_evt);
        if (s->mvrp_new_count < 2) s->mvrp_new_count++;

        s->last_join_ms = now_ms;

        // Periodic status
        if (s->join_count > 0 && (s->join_count % 30) == 0) {
            printf("[SRP] tx=%lu rx=%lu domain=%d talker_reg=%d "
                   "rx_attr: tadv=%lu tfail=%lu lst=%lu dom=%lu match=%lu\n",
                   (unsigned long)s->join_count,
                   (unsigned long)s->rx_pdu_count,
                   s->domain_received,
                   s->talker_registered,
                   (unsigned long)s->rx_talker_adv_count,
                   (unsigned long)s->rx_talker_failed_count,
                   (unsigned long)s->rx_listener_count,
                   (unsigned long)s->rx_domain_count,
                   (unsigned long)s->rx_match_count);
        }
    }
}
