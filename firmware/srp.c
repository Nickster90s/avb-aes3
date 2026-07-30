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
#include "cap.h"

extern uint8_t g_verbose;   /* console verbose toggle (main.c); 0 = quiet */

/* SRP/MSRP UART logging REMOVED — the reservation path is stable now; the
 * per-frame [SRP-RX] traces + periodic [SRP] status just flooded the console
 * and starved the UART. All SRP prints route through this no-op. (Switch back
 * to printf here if SRP ever needs debugging again.) */
#define SRPLOG(...) do {} while (0)
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
// Small xorshift PRNG — used only to jitter the LeaveAll period (mrpd
// mrp.c:422 does the same with libc random()). No libc rand() in bare metal.
// ---------------------------------------------------------------------------

static uint32_t srp_rng_state = 0x12345678;

static uint32_t srp_rand(void)
{
    uint32_t x = srp_rng_state;
    if (x == 0) x = 0x12345678;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    srp_rng_state = x;
    return x;
}

// Next LeaveAll period: MRP_LEAVEALL_PERIOD_MS + random()%(period/2), per
// IEEE 802.1Q §10.7.5.22 / mrpd, so multiple endpoints don't lock-step.
static uint32_t srp_next_lva_period(void)
{
    return MRP_LEAVEALL_PERIOD_MS + (srp_rand() % (MRP_LEAVEALL_PERIOD_MS / 2));
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

// Pack ALL talker streams into ONE MSRP Message (AttributeType=1) — one
// VectorAttribute per stream, then a SINGLE EndMark — GenAVB-style. The bridge
// then processes the LeaveAll + every stream's declaration ATOMICALLY in one
// MRPDU. Sending them as separate back-to-back MRPDUs (our old way) made the
// bridge's MSRP registrar keep only the LAST one ("stream 3 reconnects, 0/1/2
// don't"). LeaveAll rides only the first VectorAttribute (applies to the type).
static uint8_t *msrp_emit_all_talkers(uint8_t *p, srp_state_t *s,
                                      int leaveall, uint8_t event)
{
    *p++ = MSRP_ATTR_TALKER_ADV;          // AttributeType
    *p++ = 25;                            // AttributeLength
    uint8_t *list_len_ptr = p; p += 2;    // AttributeListLength (backfilled)
    uint8_t *vec_start = p;

    for (uint8_t i = 0; i < s->n_talkers; i++) {
        srp_talker_attr_t *t = &s->talkers[i];
        t->priority_and_rank   = (uint8_t)((SR_CLASS_A_PRIO << 5) | (1 << 4));
        t->vlan_id             = SR_CLASS_A_VID;
        t->max_interval_frames = 1;
        // VectorHeader: NumberOfValues=1; LeaveAll only on the first vector.
        uint16_t vec_hdr = 1;
        if (leaveall && i == 0) vec_hdr |= (1 << 13);
        srp_put_be16(p, vec_hdr); p += 2;
        // FirstValue (25 bytes)
        memcpy(p, t->stream_id, 8);  p += 8;
        memcpy(p, t->dest_addr, 6);  p += 6;
        srp_put_be16(p, t->vlan_id); p += 2;
        srp_put_be16(p, t->max_frame_size); p += 2;
        srp_put_be16(p, t->max_interval_frames); p += 2;
        *p++ = t->priority_and_rank;
        srp_put_be32(p, t->accumulated_latency_ns); p += 4;
        // ThreePackedEvents for this stream
        *p++ = MRP_3PACK(event, 0, 0);
    }
    // ONE EndMark after all vectors (AttrListLen includes it).
    srp_put_be16(p, 0); p += 2;

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
// Explicit Leave (Lv) — emit a single attribute with the MRP Lv event so the
// bridge tears the registration down immediately, instead of waiting for our
// declarations to stop and the bridge's leavetimer (~ leaveall period) to
// reclaim it. mrpd does this from the applicant LA state (mrp.c:768 →
// MRP_SND_LV). We send it on AVDECC DISCONNECT (listener) / talker disable.
// ---------------------------------------------------------------------------

static void srp_send_one_pdu(uint8_t *frame, uint8_t *p)
{
    // MessageList EndMark
    srp_put_be16(p, 0);
    p += 2;
    uint32_t len = (uint32_t)(p - frame);
    if (len < 64) { memset(p, 0, 64 - len); len = 64; }
    cap_record(1, frame, len);   // record our MSRP TX into the boot ring
    srp_eth_send(len);
}

static void srp_send_talker_leave(srp_state_t *s)
{
    for (uint8_t i = 0; i < s->n_talkers; i++) {
        uint8_t *frame = srp_tx_buf();
        uint8_t *p = msrp_frame_begin(frame, s->src_mac);
        p = msrp_emit_talker_adv(p, &s->talkers[i], 0, MRP_EVT_LV);
        srp_send_one_pdu(frame, p);
    }
    SRPLOG("[SRP] Talker Leave (Lv) sent for all streams\n");
}

// Does an 8-byte stream_id match ANY of our configured talker streams?
static int srp_match_any_talker(const srp_state_t *s, const uint8_t *sid)
{
    for (uint8_t i = 0; i < s->n_talkers; i++) {
        int eq = 1;
        for (int j = 0; j < 8; j++)
            if (sid[j] != s->talkers[i].stream_id[j]) { eq = 0; break; }
        if (eq) return 1;
    }
    return 0;
}

static void srp_send_listener_leave(srp_state_t *s, const srp_listener_t *l)
{
    uint8_t *frame = srp_tx_buf();
    uint8_t *p = msrp_frame_begin(frame, s->src_mac);
    p = msrp_emit_listener(p, l->stream_id, l->substate, 0, MRP_EVT_LV);
    srp_send_one_pdu(frame, p);
    SRPLOG("[SRP] Listener Leave (Lv) sent for stream "
           "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
           l->stream_id[0], l->stream_id[1], l->stream_id[2], l->stream_id[3],
           l->stream_id[4], l->stream_id[5], l->stream_id[6], l->stream_id[7]);
}

// ---------------------------------------------------------------------------
// TX — send MSRP PDU with all our active declarations
// ---------------------------------------------------------------------------

static void srp_send_declarations(srp_state_t *s, int leaveall)
{
    // SPLIT INTO TWO SEPARATE MSRP PDUs (2026-06-08). Previously the Domain,
    // TalkerAdvertise, and Listener attributes all shared ONE PDU. Enabling the
    // talker inserted its 25-byte TalkerAdvertise AHEAD of the CRF/AAF Listener
    // attributes in the same MessageList; any parse hiccup in the talker
    // attribute made the bridge stop before the listeners -> "TX advertises (Hive
    // shows it trying) but CRF/AAF listener never registers". The user observed
    // exactly this: turning on TX broke CRF RX. Emitting the TalkerAdvertise as
    // its OWN PDU fully decouples the two — the talker can never affect how the
    // bridge parses our listener declarations.

    // ---- PDU 1: Domain + Listener declarations ("we want to receive": CRF/AAF).
    {
        uint8_t *frame = srp_tx_buf();
        uint8_t *p = msrp_frame_begin(frame, s->src_mac);

        // Domain — NEW(0) for the first 2 cycles (VN->AN->QA), then JoinIn(1).
        uint8_t dom_event = (s->domain_new_count < 2) ? MRP_EVT_NEW : MRP_EVT_JOININ;
        p = msrp_emit_domain(p, SR_CLASS_A, SR_CLASS_A_PRIO, SR_CLASS_A_VID,
                             leaveall, dom_event);
        if (s->domain_new_count < 2)
            s->domain_new_count++;

        // Listener Ready — one attribute per enabled listener stream (CRF, AAF).
        for (int li = 0; li < SRP_MAX_LISTENER_STREAMS; li++) {
            srp_listener_t *l = &s->listeners[li];
            if (!l->enabled)
                continue;
            uint8_t event = (l->new_count < 2) ? MRP_EVT_NEW : MRP_EVT_JOININ;
            p = msrp_emit_listener(p, l->stream_id, l->substate, leaveall, event);
            if (l->new_count < 2)
                l->new_count++;
        }
        srp_send_one_pdu(frame, p);   // EndMark + pad + send
    }

    // ---- PDU 2: ALL TalkerAdvertise packed into ONE MSRP PDU (GenAVB-style,
    // decoupled from the listeners above). MRP applicant event: NEW(0)x2, then
    // JoinIn(1) once a listener registered / JoinMt(3) while none. Shared across
    // the streams (they advertise together). Sending them as SEPARATE back-to-back
    // MRPDUs made the MOTU bridge keep only the last -> 0/1/2 never reserved. One
    // packed MRPDU = the bridge processes LeaveAll + all declarations atomically.
    if (s->talker_enabled) {
        uint8_t tk_event = (s->talker_new_count < 2) ? MRP_EVT_NEW
                         : (s->talker_listener_seen  ? MRP_EVT_JOININ
                                                     : MRP_EVT_JOINMT);
        uint8_t *frame = srp_tx_buf();
        uint8_t *p = msrp_frame_begin(frame, s->src_mac);
        p = msrp_emit_all_talkers(p, s, leaveall, tk_event);
        srp_send_one_pdu(frame, p);   // EndMark + pad + send — ONE frame, all streams
        if (s->talker_new_count < 2)
            s->talker_new_count++;
    }

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
            // LeaveAll event (top 3 bits): a peer asked everyone to re-register.
            // Flag it so srp_poll re-declares immediately with NEW (MRP applicant
            // RLA response). Bridges/talkers issue this periodically; ignoring it
            // = our registration ages out -> the talker prunes the CRF stream.
            if (((vec_hdr >> 13) & 0x7) == 1)   // 1 = LeaveAll (MRP_LVA_EVENT)
                s->rx_leaveall = 1;

            // FirstValue
            if (vp + attr_len > attr_end)
                break;

            if (attr_type == MSRP_ATTR_DOMAIN && attr_len >= 4) {
                s->rx_domain_count++;
                // Only latch the SR domain from the BRIDGE (gPTP GM / switch).
                // Peer talkers (e.g. the ens5 box) advertise their own domain
                // with a different SRclassID/priority; latching theirs made us
                // declare the wrong talker priority -> bridge rejects 0x13 (SR
                // Class Priority Mismatch) -> Frames RX=0. frame+6 = src MAC.
                const uint8_t *dom_src = frame + 6;
                int from_bridge = 1;
                if (s->have_bridge_mac) {
                    for (int k = 0; k < 6; k++)
                        if (dom_src[k] != s->bridge_mac[k]) { from_bridge = 0; break; }
                }
                if (from_bridge) {
                    s->domain_received = 1;
                    s->rx_sr_class = vp[0];
                    s->rx_sr_prio  = vp[1];
                    s->rx_sr_vid   = srp_get_be16(vp + 2);
                }
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
                    // Is the bridge failing OUR talker stream? THE definitive
                    // "why won't it reserve us" — print every time with the
                    // IEEE 802.1Q Table 35-6 code (0x06=no bandwidth, 0x05=dest
                    // in use, 0x16=class/priority, ...).
                    if (s->talker_enabled) {
                        int ours = srp_match_any_talker(s, sid);
                        if (ours) {
                            s->talker_fail_code = code;
                            s->talker_fail_count++;
                            SRPLOG("[SRP] *** OUR TALKER FAILED code=0x%02x "
                                   "bridge=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x ***\n",
                                   (unsigned)code, vp[25], vp[26], vp[27], vp[28],
                                   vp[29], vp[30], vp[31], vp[32]);
                        }
                    }
                    int same = printed_once && code == last_code;
                    if (same) {
                        for (int j = 0; j < 8; j++)
                            if (sid[j] != last_sid[j]) { same = 0; break; }
                    }
                    if (!same) {
                        if (g_verbose) SRPLOG("[SRP-RX] TalkerFailed sid=%02x:%02x:%02x:%02x:"
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
            if (attr_type == MSRP_ATTR_LISTENER) {
                s->rx_listener_count++;
                // Extract the FourPackedEvent substate (1=AskingFailed, 2=Ready,
                // 3=ReadyFailed) — the 4-pack byte follows the FirstValue + its
                // 3-pack event. For num_values=1: 3-pack at vp[attr_len], 4-pack
                // at vp[attr_len+1]; substate = 4pack / 64 (first value).
                uint32_t n3 = (num_values + 2) / 3;
                uint8_t  sub = 0;
                if (vp + attr_len + n3 < attr_end)
                    sub = vp[attr_len + n3] / 64;

                // Flood guard + DIAGNOSTIC: does a remote listener want OUR
                // talker stream? (vp[0..7] = Listener FirstValue = stream_id.)
                int eq = srp_match_any_talker(s, vp);

                // Log EVERY Listener declaration we receive (rate-limited 2s)
                // so we can see whether AxC ever declares a listener for our
                // stream, and in what substate. This is THE signal that the
                // bridge will forward our AAF — without it, no Frames RX.
                {
                    static uint32_t last_log_ms;
                    uint32_t now = gptp_uptime_ms();
                    // Throttle to 2 s REGARDLESS of eq: AxC re-declares OUR stream
                    // every MRP cycle, and the old `eq ||` bypass printed every
                    // single one -> flooded the 64-byte UART FIFO -> console
                    // crawled. The first eq is still announced by the [SRP] ***
                    // print below (talker_listener_seen); the 'a'/'s' counters
                    // give live state. So periodic (2 s) logging is plenty.
                    if (g_verbose && (now - last_log_ms) >= 2000) {
                        last_log_ms = now;
                        SRPLOG("[SRP-RX] Listener decl sid=%02x:%02x:%02x:%02x:"
                               "%02x:%02x:%02x:%02x substate=%u%s\n",
                               vp[0], vp[1], vp[2], vp[3],
                               vp[4], vp[5], vp[6], vp[7],
                               (unsigned)sub, eq ? "  <== OUR TALKER STREAM" : "");
                    }
                }

                if (s->talker_enabled && eq) {
                    if (!s->talker_listener_seen)
                        SRPLOG("[SRP] *** REMOTE LISTENER declared OUR stream "
                               "(substate=%u) — bridge should now forward AAF ***\n",
                               (unsigned)sub);
                    s->talker_listener_seen    = 1;
                    s->talker_listener_seen_ms = gptp_uptime_ms();
                }
            }

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
                    if (log_it && g_verbose) {
                        SRPLOG("[SRP-RX] TalkerAdv sid=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x "
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

                // Does this TalkerAdvertise match ANY of our listener streams?
                for (int li = 0; li < SRP_MAX_LISTENER_STREAMS; li++) {
                    srp_listener_t *l = &s->listeners[li];
                    if (!l->enabled)
                        continue;
                    int match = 1;
                    for (int i = 0; i < 8; i++) {
                        if (vp[i] != l->stream_id[i]) { match = 0; break; }
                    }
                    if (match) {
                        s->rx_match_count++;
                        if (!l->talker_registered)
                            SRPLOG("[SRP] Talker registered for our stream "
                                   "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
                                   l->stream_id[0], l->stream_id[1],
                                   l->stream_id[2], l->stream_id[3],
                                   l->stream_id[4], l->stream_id[5],
                                   l->stream_id[6], l->stream_id[7]);
                        l->talker_registered   = 1;
                        l->talker_last_seen_ms = gptp_uptime_ms();
                        l->substate            = MSRP_LISTENER_READY;
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
    // Seed the PRNG from the MAC so each endpoint jitters its LeaveAll
    // period to a different phase; then set the first randomized period.
    srp_rng_state = ((uint32_t)mac_addr[2] << 24) | ((uint32_t)mac_addr[3] << 16) |
                    ((uint32_t)mac_addr[4] << 8) | mac_addr[5];
    if (srp_rng_state == 0) srp_rng_state = 0x12345678;
    s->leaveall_period_ms = srp_next_lva_period();
    // listeners[] all start enabled=0 (no declarations) via the memset.
    SRPLOG("[SRP] Initialized (MSRP)\n");
}

void srp_set_bridge_mac(srp_state_t *s, const uint8_t *bridge_mac)
{
    int same = s->have_bridge_mac;
    for (int k = 0; k < 6; k++) {
        if (s->bridge_mac[k] != bridge_mac[k]) same = 0;
        s->bridge_mac[k] = bridge_mac[k];
    }
    if (!same) {
        s->have_bridge_mac = 1;
        // Force re-latching the SR domain from the (now known) bridge so a
        // peer talker's domain we may have latched earlier is overwritten.
        s->domain_received = 0;
        SRPLOG("[SRP] bridge MAC = %02x:%02x:%02x:%02x:%02x:%02x (SR domain authority)\n",
               bridge_mac[0], bridge_mac[1], bridge_mac[2],
               bridge_mac[3], bridge_mac[4], bridge_mac[5]);
    }
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

void srp_talker_set(srp_state_t *s, uint8_t idx, const uint8_t *stream_id,
                    const uint8_t *dest_mac, uint16_t max_frame_size)
{
    if (idx >= N_SRP_TALKERS) return;
    srp_talker_attr_t *tk = &s->talkers[idx];
    memcpy(tk->stream_id, stream_id, 8);
    memcpy(tk->dest_addr, dest_mac, 6);
    tk->vlan_id = SR_CLASS_A_VID;
    tk->max_frame_size = max_frame_size;
    tk->max_interval_frames = 1;  // Class A: 1 frame per 125us interval
    // Priority 3 (Class A), Rank 1 (non-emergency)
    tk->priority_and_rank = (SR_CLASS_A_PRIO << 5) | (1 << 4);
    // AccumulatedLatency: worst-case sample-reference-to-egress latency, ns.
    // 0 is a deviation — every reference talker declares non-zero (OpenAvnu
    // simple_talker 3900, GenAVB adds CFG_PORT_TC_MAX_LATENCY=500/hop, the
    // MOTU bridge declares 100095). A Milan listener uses this to size its
    // buffer; some reject 0. We batch 6 samples/packet (~125 us) so our real
    // talker latency is ~that order; declare 250 us (well under the 2 ms Class
    // A bound, matches the MOTU ecosystem magnitude). See memory
    // [[msrp-maxframesize-must-be-real-frame-size]].
    tk->accumulated_latency_ns = 500000;   // match the working reference talker (avb_session_mgr2)
    if ((uint8_t)(idx + 1) > s->n_talkers) s->n_talkers = idx + 1;
}

void srp_talker_enable(srp_state_t *s, uint8_t enable)
{
    if (enable) {
        s->talker_enabled = 1;
        // Reset MRPDU_NEW counter so the next 2 advertises emit NEW(0)
        // and force the bridge's registrar into the registered state.
        s->talker_new_count = 0;
        SRPLOG("[SRP] Talker Advertise enabled\n");
    } else {
        // Emit an explicit Lv (while talker is still marked enabled so the
        // attribute fields are valid) so the bridge frees the reservation
        // immediately rather than aging it out.
        if (s->talker_enabled)
            srp_send_talker_leave(s);
        s->talker_enabled = 0;
        SRPLOG("[SRP] Talker Advertise disabled\n");
    }
}

// Find the listener entry whose stream_id matches, else NULL.
static srp_listener_t *srp_find_listener(srp_state_t *s, const uint8_t *stream_id)
{
    for (int i = 0; i < SRP_MAX_LISTENER_STREAMS; i++) {
        srp_listener_t *l = &s->listeners[i];
        if (!l->enabled)
            continue;
        int match = 1;
        for (int j = 0; j < 8; j++) {
            if (l->stream_id[j] != stream_id[j]) { match = 0; break; }
        }
        if (match)
            return l;
    }
    return NULL;
}

int srp_any_talker_registered(const srp_state_t *s)
{
    for (int i = 0; i < SRP_MAX_LISTENER_STREAMS; i++)
        if (s->listeners[i].enabled && s->listeners[i].talker_registered)
            return 1;
    return 0;
}

void srp_listener_enable(srp_state_t *s, const uint8_t *stream_id, uint8_t enable)
{
    srp_listener_t *l = srp_find_listener(s, stream_id);

    if (enable) {
        if (!l) {
            // Allocate a free slot.
            for (int i = 0; i < SRP_MAX_LISTENER_STREAMS; i++) {
                if (!s->listeners[i].enabled) { l = &s->listeners[i]; break; }
            }
            if (!l) {
                SRPLOG("[SRP] Listener table full — cannot add stream\n");
                return;
            }
            memset(l, 0, sizeof(*l));
            l->enabled = 1;
            memcpy(l->stream_id, stream_id, 8);
        }
        // Declare Ready immediately, mirroring avb_session_mgr2's
        // send_ready() at ACMP CONNECT_RX time. AskingFailed = "I want
        // this stream but cannot reserve" — talkers (Auvitran observed)
        // interpret it as "no listener" and hold back CRF/AAF. Ready =
        // "I want it and can receive"; the correct initial substate once
        // ACMP has resolved the stream.
        l->substate = MSRP_LISTENER_READY;
        l->talker_registered = 0;
        // Fresh 30 s talker-age window (stale stamp from a prior attach
        // would otherwise time out immediately).
        l->talker_last_seen_ms = gptp_uptime_ms();
        // Re-emit MRPDU_NEW for the next 2 transmissions (fresh attach).
        l->new_count = 0;
        SRPLOG("[SRP] Listener Ready for stream %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
               stream_id[0], stream_id[1], stream_id[2], stream_id[3],
               stream_id[4], stream_id[5], stream_id[6], stream_id[7]);
    } else {
        if (l) {
            // Send an explicit Lv for this stream (while still enabled so
            // stream_id/substate are valid) so the bridge/talker drops our
            // listener registration at once and the talker stops sourcing —
            // instead of waiting for our declarations to lapse.
            srp_send_listener_leave(s, l);
            l->enabled = 0;
            SRPLOG("[SRP] Listener disabled for stream "
                   "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
                   stream_id[0], stream_id[1], stream_id[2], stream_id[3],
                   stream_id[4], stream_id[5], stream_id[6], stream_id[7]);
        }
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

    // MRP applicant RLA response (GenAVB mrp.c: received LeaveAll -> VP -> re-Join
    // on next TX). A bridge/talker LeaveAll resets every registration; we MUST
    // re-declare promptly or our Listener registration ages out and the talker
    // prunes the CRF stream (the on-HW ~3 s flap that the watchdog was papering
    // over). Re-emit NEW (not just JoinIn) so the bridge registrar re-initialises
    // [[feedback_mrp_new_event]], and force the join timer to fire THIS poll.
    if (s->rx_leaveall) {
        s->rx_leaveall        = 0;
        s->domain_new_count   = 0;
        s->talker_new_count   = 0;
        s->mvrp_new_count     = 0;
        for (int li = 0; li < SRP_MAX_LISTENER_STREAMS; li++)
            s->listeners[li].new_count = 0;
        s->last_join_ms       = now_ms - MRP_JOIN_PERIOD_MS;  // re-declare now
        // MRP LeaveAll SM (802.1Q Table 10-5 / GenAVB mrp.c:1234-1237): on a
        // RECEIVED LeaveAll (RLA) we RESTART our own LeaveAll timer instead of
        // also originating one. Only one participant per segment should source
        // LeaveAlls; double-originating doubles the segment-wide re-declaration
        // churn, and every LeaveAll forces the MOTU talker to re-declare — a slow
        // re-declare is a window where the CRF can briefly gap. Deferring ours
        // cuts that churn in half.
        s->last_leaveall_ms   = now_ms;
        s->leaveall_period_ms = srp_next_lva_period();
    }

    uint32_t elapsed_join = now_ms - s->last_join_ms;
    if (elapsed_join > 2000000000)
        elapsed_join = MRP_JOIN_PERIOD_MS;  // Handle wrap

    uint32_t elapsed_lva = now_ms - s->last_leaveall_ms;
    if (elapsed_lva > 2000000000)
        elapsed_lva = MRP_LEAVEALL_PERIOD_MS;

    // LeaveAll timer — randomized period (set per-fire) so co-booting
    // endpoints don't synchronize their LeaveAlls into a storm (mrpd
    // mrp.c:422). leaveall_period_ms ∈ [10s, 15s].
    int leaveall = 0;
    if (elapsed_lva >= s->leaveall_period_ms) {
        leaveall = 1;
        s->last_leaveall_ms = now_ms;
        s->leaveall_period_ms = srp_next_lva_period();
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
    // Per listener stream (CRF, AAF, …):
    for (int li = 0; li < SRP_MAX_LISTENER_STREAMS; li++) {
        srp_listener_t *l = &s->listeners[li];
        if (!l->enabled)
            continue;

        if (l->talker_registered) {
            uint32_t age = now_ms - l->talker_last_seen_ms;
            if (age > 2000000000) age = 0;          // wrap guard
            if (age >= (3u * MRP_LEAVEALL_PERIOD_MS)) {
                SRPLOG("[SRP] Talker advertise gap %u ms — clearing 'seen' flag "
                       "(listener stays READY)\n", (unsigned)age);
                l->talker_registered = 0;
            }
        }

        // (a') Keepalive: while the listener is enabled (AVDECC-connected),
        // hold the substate at READY. Mirrors avb_session_mgr2's periodic
        // "force-refresh the listener attachment" — keeps telling the
        // bridge/talker we want the stream so the talker sources it (and
        // re-sources it after any gap). Only srp_listener_enable(enable=0)
        // on AVDECC DISCONNECT clears it.
        if (l->substate != MSRP_LISTENER_READY) {
            l->substate  = MSRP_LISTENER_READY;
            l->new_count = 0;   // re-emit MRPDU_NEW on re-assert
        }
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

    // Flood guard age-out: if no remote listener has declared our talker
    // stream within 3× LeaveAll, clear the "safe to transmit" flag so main.c
    // stops the gateware AAF TX (don't blast an unreserved/unwanted stream).
    if (s->talker_listener_seen) {
        uint32_t age = now_ms - s->talker_listener_seen_ms;
        if (age > 2000000000) age = 0;
        if (age >= (3u * MRP_LEAVEALL_PERIOD_MS))
            s->talker_listener_seen = 0;
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
        if (g_verbose && s->join_count > 0 && (s->join_count % 30) == 0) {
            SRPLOG("[SRP] tx=%lu rx=%lu domain=%d talker_reg=%d "
                   "rx_attr: tadv=%lu tfail=%lu lst=%lu dom=%lu match=%lu\n",
                   (unsigned long)s->join_count,
                   (unsigned long)s->rx_pdu_count,
                   s->domain_received,
                   srp_any_talker_registered(s),
                   (unsigned long)s->rx_talker_adv_count,
                   (unsigned long)s->rx_talker_failed_count,
                   (unsigned long)s->rx_listener_count,
                   (unsigned long)s->rx_domain_count,
                   (unsigned long)s->rx_match_count);
        }
    }
}
