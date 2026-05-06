// IEEE 802.1AS (gPTP) — Bare-metal implementation
// Slave-only, P2P delay, Layer 2

#include "gptp.h"

#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>
#include <string.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Low-level MAC frame TX/RX
// ---------------------------------------------------------------------------

#define ETHMAC_EV_SRAM_WRITER 0x1
#define ETHMAC_EV_SRAM_READER 0x1

static uint32_t txslot;
static uint32_t rxslot;

static uint8_t *tx_buf(void)
{
    return (uint8_t *)(ETHMAC_BASE + ETHMAC_SLOT_SIZE * (ETHMAC_RX_SLOTS + txslot));
}

static void eth_send(uint32_t len)
{
    while (!ethmac_sram_reader_ready_read())
        ;
    ethmac_sram_reader_slot_write(txslot);
    ethmac_sram_reader_length_write(len);
    ethmac_sram_reader_start_write(1);
    txslot = (txslot + 1) % ETHMAC_TX_SLOTS;
}

// ---------------------------------------------------------------------------
// Byte-order helpers (network byte order = big endian)
// ---------------------------------------------------------------------------

static inline void put_be16(uint8_t *p, uint16_t v)
{
    p[0] = (v >> 8) & 0xFF;
    p[1] = v & 0xFF;
}

static inline void put_be32(uint8_t *p, uint32_t v)
{
    p[0] = (v >> 24) & 0xFF;
    p[1] = (v >> 16) & 0xFF;
    p[2] = (v >>  8) & 0xFF;
    p[3] = v & 0xFF;
}

static inline void put_be48(uint8_t *p, uint64_t v)
{
    p[0] = (v >> 40) & 0xFF;
    p[1] = (v >> 32) & 0xFF;
    p[2] = (v >> 24) & 0xFF;
    p[3] = (v >> 16) & 0xFF;
    p[4] = (v >>  8) & 0xFF;
    p[5] = v & 0xFF;
}

static inline uint16_t get_be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static inline uint32_t get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] <<  8) | p[3];
}

static inline uint64_t get_be48(const uint8_t *p)
{
    return ((uint64_t)p[0] << 40) | ((uint64_t)p[1] << 32) |
           ((uint64_t)p[2] << 24) | ((uint64_t)p[3] << 16) |
           ((uint64_t)p[4] <<  8) | p[5];
}

static inline int64_t get_be64_signed(const uint8_t *p)
{
    uint64_t v = ((uint64_t)p[0] << 56) | ((uint64_t)p[1] << 48) |
                 ((uint64_t)p[2] << 40) | ((uint64_t)p[3] << 32) |
                 ((uint64_t)p[4] << 24) | ((uint64_t)p[5] << 16) |
                 ((uint64_t)p[6] <<  8) | p[7];
    return (int64_t)v;
}

// ---------------------------------------------------------------------------
// PTP timestamp extraction from wire format (10 bytes: 6s + 4ns)
// ---------------------------------------------------------------------------

static ptp_timestamp_t get_ptp_ts(const uint8_t *p)
{
    ptp_timestamp_t ts;
    ts.seconds     = get_be48(p);
    ts.nanoseconds = get_be32(p + 6);
    return ts;
}

static void put_ptp_ts(uint8_t *p, ptp_timestamp_t ts)
{
    put_be48(p, ts.seconds);
    put_be32(p + 6, ts.nanoseconds);
}

// ---------------------------------------------------------------------------
// TSU access
// ---------------------------------------------------------------------------

ptp_timestamp_t gptp_read_time(void)
{
    ptp_timestamp_t ts;
    // Read seconds_hi first to latch all values atomically
    uint32_t shi = tsu_seconds_hi_read();
    uint32_t slo = tsu_seconds_lo_read();
    ts.nanoseconds = tsu_nanoseconds_read();
    ts.seconds = ((uint64_t)shi << 32) | slo;
    return ts;
}

ptp_timestamp_t gptp_read_rx_timestamp(void)
{
    ptp_timestamp_t ts;
    ts.seconds     = tsu_rx_ts_seconds_read();
    ts.nanoseconds = tsu_rx_ts_nsec_read();
    return ts;
}

ptp_timestamp_t gptp_read_tx_timestamp(void)
{
    ptp_timestamp_t ts;
    ts.seconds     = tsu_tx_ts_seconds_read();
    ts.nanoseconds = tsu_tx_ts_nsec_read();
    return ts;
}

void gptp_set_addend(uint32_t addend)
{
    tsu_addend_write(addend);
}

void gptp_step_time(ptp_timestamp_t t)
{
    tsu_step_seconds_write(t.seconds);
    tsu_step_nsec_write(t.nanoseconds);
    tsu_step_apply_write(1);
}

void gptp_adjust_offset(int64_t offset_ns)
{
    tsu_offset_lo_write((uint32_t)(offset_ns & 0xFFFFFFFF));
    tsu_offset_hi_write((uint32_t)((offset_ns >> 32) & 0x1FFFF));
    tsu_offset_apply_write(1);
}

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------

// Simple millisecond counter from the TSU nanoseconds

uint32_t gptp_uptime_ms(void)
{
    ptp_timestamp_t t = gptp_read_time();
    return (uint32_t)(t.seconds * 1000 + t.nanoseconds / 1000000);
}

int64_t gptp_ts_diff_ns(ptp_timestamp_t a, ptp_timestamp_t b)
{
    int64_t sec_diff = (int64_t)a.seconds - (int64_t)b.seconds;
    int64_t ns_diff  = (int64_t)a.nanoseconds - (int64_t)b.nanoseconds;
    return sec_diff * 1000000000LL + ns_diff;
}

// ---------------------------------------------------------------------------
// Build PTP common header
// ---------------------------------------------------------------------------

// Offsets within PTP header (after Ethernet header)
#define PTP_OFF_MSG_TYPE     0
#define PTP_OFF_VERSION      1
#define PTP_OFF_LENGTH       2
#define PTP_OFF_DOMAIN       4
#define PTP_OFF_FLAGS        6
#define PTP_OFF_CORRECTION   8
#define PTP_OFF_SRC_PORT_ID  20
#define PTP_OFF_SEQ_ID       30
#define PTP_OFF_CONTROL      32
#define PTP_OFF_LOG_INTERVAL 33

static void build_eth_header(uint8_t *frame, const uint8_t *src_mac)
{
    static const uint8_t gptp_mcast[] = GPTP_MCAST_ADDR;
    memcpy(frame + 0, gptp_mcast, 6);  // Destination MAC
    memcpy(frame + 6, src_mac, 6);     // Source MAC
    put_be16(frame + 12, PTP_ETHERTYPE);
}

static void build_ptp_header(uint8_t *ptp, uint8_t msg_type, uint16_t msg_len,
                              const uint8_t *clock_id, uint16_t port_num,
                              uint16_t seq_id, int8_t log_interval)
{
    memset(ptp, 0, PTP_HEADER_LEN);
    ptp[PTP_OFF_MSG_TYPE]  = msg_type & 0x0F;
    ptp[PTP_OFF_VERSION]   = 0x02;  // PTP version 2
    put_be16(ptp + PTP_OFF_LENGTH, msg_len);
    ptp[PTP_OFF_DOMAIN] = GPTP_DOMAIN;
    // Flags: two-step for event messages
    if (msg_type <= 0x3)  // Event message
        put_be16(ptp + PTP_OFF_FLAGS, PTP_FLAG_TWO_STEP);
    // Source port identity
    memcpy(ptp + PTP_OFF_SRC_PORT_ID, clock_id, CLOCK_ID_LEN);
    put_be16(ptp + PTP_OFF_SRC_PORT_ID + 8, port_num);
    put_be16(ptp + PTP_OFF_SEQ_ID, seq_id);
    ptp[PTP_OFF_LOG_INTERVAL] = (uint8_t)log_interval;

    // Control field (deprecated but set for compatibility)
    switch (msg_type) {
        case PTP_MSG_SYNC:            ptp[PTP_OFF_CONTROL] = 0x00; break;
        case PTP_MSG_PDELAY_REQ:      ptp[PTP_OFF_CONTROL] = 0x05; break;
        case PTP_MSG_PDELAY_RESP:     ptp[PTP_OFF_CONTROL] = 0x05; break;
        case PTP_MSG_FOLLOW_UP:       ptp[PTP_OFF_CONTROL] = 0x02; break;
        case PTP_MSG_PDELAY_RESP_FUP: ptp[PTP_OFF_CONTROL] = 0x05; break;
        default:                      ptp[PTP_OFF_CONTROL] = 0x05; break;
    }
}

// ---------------------------------------------------------------------------
// Send Pdelay_Req
// ---------------------------------------------------------------------------

void gptp_send_pdelay_req(gptp_t *g)
{
    uint8_t *frame = tx_buf();
    uint8_t *ptp = frame + ETH_HEADER_LEN;
    uint16_t msg_len = PTP_HEADER_LEN + 20;  // 34 + 20 = 54

    build_eth_header(frame, g->our_mac);
    build_ptp_header(ptp, PTP_MSG_PDELAY_REQ, msg_len,
                     g->clock_id, g->port_number,
                     g->pdelay_seq_id, LOG_PDELAY_REQ_INTERVAL);

    // Body: 10 bytes reserved originTimestamp + 10 bytes reserved
    memset(ptp + PTP_HEADER_LEN, 0, 20);

    g->pdelay_resp_received = 0;
    g->pdelay_fup_received  = 0;

    eth_send(ETH_HEADER_LEN + msg_len);

    // Capture TX timestamp (latched at SOF by hardware)
    g->pdelay_t1 = gptp_read_tx_timestamp();

    g->last_pdelay_time_ms = gptp_uptime_ms();
    g->pdelay_count++;
}

// ---------------------------------------------------------------------------
// Send Pdelay_Resp (we respond to Pdelay_Req from peers)
// ---------------------------------------------------------------------------

static void gptp_send_pdelay_resp(gptp_t *g, const uint8_t *req_ptp,
                                   ptp_timestamp_t rx_ts)
{
    uint8_t *frame = tx_buf();
    uint8_t *ptp = frame + ETH_HEADER_LEN;
    uint16_t msg_len = PTP_HEADER_LEN + 20;

    // Get requester's port identity and seq_id from the request
    uint16_t req_seq = get_be16(req_ptp + PTP_OFF_SEQ_ID);

    build_eth_header(frame, g->our_mac);
    build_ptp_header(ptp, PTP_MSG_PDELAY_RESP, msg_len,
                     g->clock_id, g->port_number,
                     req_seq, LOG_PDELAY_REQ_INTERVAL);

    // Body: requestReceiptTimestamp (10 bytes) + requestingPortIdentity (10 bytes)
    put_ptp_ts(ptp + PTP_HEADER_LEN, rx_ts);
    memcpy(ptp + PTP_HEADER_LEN + 10, req_ptp + PTP_OFF_SRC_PORT_ID, PORT_ID_LEN);

    eth_send(ETH_HEADER_LEN + msg_len);
}

static void gptp_send_pdelay_resp_fup(gptp_t *g, const uint8_t *req_ptp,
                                       ptp_timestamp_t resp_tx_ts)
{
    uint8_t *frame = tx_buf();
    uint8_t *ptp = frame + ETH_HEADER_LEN;
    uint16_t msg_len = PTP_HEADER_LEN + 20;

    uint16_t req_seq = get_be16(req_ptp + PTP_OFF_SEQ_ID);

    build_eth_header(frame, g->our_mac);
    build_ptp_header(ptp, PTP_MSG_PDELAY_RESP_FUP, msg_len,
                     g->clock_id, g->port_number,
                     req_seq, LOG_PDELAY_REQ_INTERVAL);

    // Body: responseOriginTimestamp (10 bytes) + requestingPortIdentity (10 bytes)
    put_ptp_ts(ptp + PTP_HEADER_LEN, resp_tx_ts);
    memcpy(ptp + PTP_HEADER_LEN + 10, req_ptp + PTP_OFF_SRC_PORT_ID, PORT_ID_LEN);

    eth_send(ETH_HEADER_LEN + msg_len);
}

// ---------------------------------------------------------------------------
// Process received PTP messages
// ---------------------------------------------------------------------------

static void process_sync(gptp_t *g, const uint8_t *ptp, uint32_t ptp_len)
{
    g->sync_seq_id    = get_be16(ptp + PTP_OFF_SEQ_ID);
    g->sync_correction = get_be64_signed(ptp + PTP_OFF_CORRECTION);
    g->sync_rx_ts     = gptp_read_rx_timestamp();
    g->sync_received  = 1;

    // Learn master identity from Sync source
    memcpy(g->master_clock_id, ptp + PTP_OFF_SRC_PORT_ID, CLOCK_ID_LEN);
    g->master_port_number = get_be16(ptp + PTP_OFF_SRC_PORT_ID + 8);
    g->last_sync_time_ms = gptp_uptime_ms();

    if (g->state == GPTP_STATE_LISTENING)
        g->state = GPTP_STATE_SLAVE;
}

static void process_follow_up(gptp_t *g, const uint8_t *ptp, uint32_t ptp_len)
{
    uint16_t seq = get_be16(ptp + PTP_OFF_SEQ_ID);

    // Must match the last Sync we received
    if (!g->sync_received || seq != g->sync_seq_id)
        return;

    // Extract preciseOriginTimestamp from Follow_Up body
    g->sync_origin_ts = get_ptp_ts(ptp + PTP_HEADER_LEN);

    // Add correction field from Follow_Up (cumulative with Sync correction)
    int64_t fup_correction = get_be64_signed(ptp + PTP_OFF_CORRECTION);
    int64_t total_correction = g->sync_correction + fup_correction;

    // Compute offset from master:
    // offset = sync_rx_ts - sync_origin_ts - correction - meanPathDelay
    int64_t rx_ns   = gptp_ts_diff_ns(g->sync_rx_ts, (ptp_timestamp_t){0, 0});
    int64_t orig_ns = gptp_ts_diff_ns(g->sync_origin_ts, (ptp_timestamp_t){0, 0});

    // Correction field is in units of 2^-16 nanoseconds
    int64_t corr_ns = total_correction >> 16;

    g->offset_from_master_ns = rx_ns - orig_ns - corr_ns - g->mean_path_delay_ns;

    g->sync_received = 0;
    g->sync_count++;

    gptp_servo_update(g);
}

static void process_pdelay_req(gptp_t *g, const uint8_t *ptp, uint32_t ptp_len)
{
    // We received a Pdelay_Req from a peer — respond
    ptp_timestamp_t rx_ts = gptp_read_rx_timestamp();

    // Send Pdelay_Resp
    gptp_send_pdelay_resp(g, ptp, rx_ts);

    // Capture the TX timestamp of our Pdelay_Resp, then send Follow_Up
    ptp_timestamp_t resp_tx_ts = gptp_read_tx_timestamp();
    gptp_send_pdelay_resp_fup(g, ptp, resp_tx_ts);
}

static void process_pdelay_resp(gptp_t *g, const uint8_t *ptp, uint32_t ptp_len)
{
    uint16_t seq = get_be16(ptp + PTP_OFF_SEQ_ID);
    if (seq != g->pdelay_seq_id)
        return;

    // Check requesting port identity matches ours
    const uint8_t *req_id = ptp + PTP_HEADER_LEN + 10;
    int id_match = 1;
    for (int i = 0; i < CLOCK_ID_LEN; i++) {
        if (req_id[i] != g->clock_id[i]) { id_match = 0; break; }
    }
    if (!id_match)
        return;

    // t4 = our RX timestamp of this Pdelay_Resp
    g->pdelay_t4 = gptp_read_rx_timestamp();
    // t2 = requestReceiptTimestamp from body
    g->pdelay_t2 = get_ptp_ts(ptp + PTP_HEADER_LEN);
    g->pdelay_resp_received = 1;
}

static void process_pdelay_resp_fup(gptp_t *g, const uint8_t *ptp, uint32_t ptp_len)
{
    uint16_t seq = get_be16(ptp + PTP_OFF_SEQ_ID);
    if (seq != g->pdelay_seq_id)
        return;
    if (!g->pdelay_resp_received)
        return;

    // t3 = responseOriginTimestamp from body
    g->pdelay_t3 = get_ptp_ts(ptp + PTP_HEADER_LEN);
    g->pdelay_fup_received = 1;

    // Compute mean path delay:
    // meanPathDelay = ((t4 - t1) - (t3 - t2)) / 2
    int64_t round_trip = gptp_ts_diff_ns(g->pdelay_t4, g->pdelay_t1);
    int64_t responder  = gptp_ts_diff_ns(g->pdelay_t3, g->pdelay_t2);
    int64_t delay = (round_trip - responder) / 2;

    // Sanity check — reject negative or unreasonably large delays
    if (delay >= 0 && delay < 1000000000LL) {
        // Simple exponential smoothing
        if (g->mean_path_delay_ns == 0)
            g->mean_path_delay_ns = delay;
        else
            g->mean_path_delay_ns = (g->mean_path_delay_ns * 7 + delay) / 8;
    }

    g->pdelay_lost_count = 0;
    g->pdelay_seq_id++;
}

// ---------------------------------------------------------------------------
// Clock servo — PI controller
// ---------------------------------------------------------------------------

void gptp_servo_update(gptp_t *g)
{
    int64_t offset = g->offset_from_master_ns;

    // First sync: step the clock directly
    if (g->servo_step_count == 0) {
        printf("[gPTP] Initial step: offset=%lld ns\n", (long long)offset);
        gptp_adjust_offset(-offset);
        g->servo_step_count = 1;
        return;
    }

    // If offset is very large (>100ms), step instead of slew
    if (offset > 100000000LL || offset < -100000000LL) {
        printf("[gPTP] Large offset step: %lld ns\n", (long long)offset);
        gptp_adjust_offset(-offset);
        g->freq_integral = 0;
        return;
    }

    // PI controller
    // Proportional: immediate frequency adjustment
    int64_t p_term = (offset * SERVO_KP) >> 16;

    // Integral: accumulate for long-term drift
    g->freq_integral += (offset * SERVO_KI) >> 16;

    // Clamp integral to prevent windup (±10 ppm worth of addend)
    int64_t max_integral = (int64_t)g->base_addend / 100;  // ~1% max
    if (g->freq_integral > max_integral)
        g->freq_integral = max_integral;
    if (g->freq_integral < -max_integral)
        g->freq_integral = -max_integral;

    // Apply: adjust addend (higher addend = faster clock)
    // Negative offset means we're ahead → slow down → decrease addend
    int64_t adjustment = p_term + g->freq_integral;
    int64_t new_addend = (int64_t)g->base_addend - adjustment;

    if (new_addend > 0 && new_addend < (int64_t)g->base_addend * 2) {
        g->current_addend = (uint32_t)new_addend;
        gptp_set_addend(g->current_addend);
    }

    g->servo_step_count++;

    // Consider locked when offset is consistently small
    if (offset > -1000 && offset < 1000)
        g->servo_locked = 1;
    else
        g->servo_locked = 0;

    // Periodic status
    if ((g->sync_count % 8) == 0) {
        printf("[gPTP] sync=%lu off=%lld ns delay=%lld ns add=%lu %s\n",
               (unsigned long)g->sync_count,
               (long long)offset,
               (long long)g->mean_path_delay_ns,
               (unsigned long)g->current_addend,
               g->servo_locked ? "LOCKED" : "");
    }
}

// ---------------------------------------------------------------------------
// Process received frame
// ---------------------------------------------------------------------------

void gptp_process_rx(gptp_t *g, const uint8_t *frame, uint32_t len)
{
    if (len < ETH_HEADER_LEN + PTP_HEADER_LEN)
        return;

    // Check ethertype
    uint16_t ethertype = get_be16(frame + 12);
    if (ethertype != PTP_ETHERTYPE)
        return;

    const uint8_t *ptp = frame + ETH_HEADER_LEN;
    uint32_t ptp_len = len - ETH_HEADER_LEN;

    uint8_t msg_type = ptp[PTP_OFF_MSG_TYPE] & 0x0F;
    uint8_t domain   = ptp[PTP_OFF_DOMAIN];

    g->rx_last_msg_type = msg_type;
    g->rx_last_domain   = domain;

    // Only process our domain
    if (domain != GPTP_DOMAIN) {
        g->rx_wrong_domain_count++;
        return;
    }

    switch (msg_type) {
        case PTP_MSG_SYNC:
            g->rx_sync_count++;
            process_sync(g, ptp, ptp_len);
            break;
        case PTP_MSG_FOLLOW_UP:
            g->rx_followup_count++;
            process_follow_up(g, ptp, ptp_len);
            break;
        case PTP_MSG_PDELAY_REQ:
            g->rx_pdelay_req_count++;
            process_pdelay_req(g, ptp, ptp_len);
            break;
        case PTP_MSG_PDELAY_RESP:
            g->rx_pdelay_resp_count++;
            process_pdelay_resp(g, ptp, ptp_len);
            break;
        case PTP_MSG_PDELAY_RESP_FUP:
            g->rx_pdelay_resp_fup_count++;
            process_pdelay_resp_fup(g, ptp, ptp_len);
            break;
        case PTP_MSG_ANNOUNCE:
            g->rx_announce_count++;
            break;
        default:
            g->rx_other_count++;
            break;
    }
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void gptp_init(gptp_t *g, const uint8_t *mac_addr)
{
    memset(g, 0, sizeof(*g));

    // Copy MAC address
    memcpy(g->our_mac, mac_addr, 6);

    // Build clock identity from MAC: insert FF:FE in the middle
    // MAC = AA:BB:CC:DD:EE:FF → Clock ID = AA:BB:CC:FF:FE:DD:EE:FF
    g->clock_id[0] = mac_addr[0];
    g->clock_id[1] = mac_addr[1];
    g->clock_id[2] = mac_addr[2];
    g->clock_id[3] = 0xFF;
    g->clock_id[4] = 0xFE;
    g->clock_id[5] = mac_addr[3];
    g->clock_id[6] = mac_addr[4];
    g->clock_id[7] = mac_addr[5];

    g->port_number = 1;
    g->state = GPTP_STATE_LISTENING;

    // Compute nominal TSU addend for this clock frequency
    // addend = (2^32 + clk_freq/2) / clk_freq
    uint32_t clk_freq = CONFIG_CLOCK_FREQUENCY;
    g->base_addend = (uint32_t)(((uint64_t)1 << 32) / clk_freq +
                                 (((uint64_t)1 << 32) % clk_freq > clk_freq/2 ? 1 : 0));
    g->current_addend = g->base_addend;
    gptp_set_addend(g->base_addend);

    // Pdelay interval: 1 second
    g->pdelay_interval_ms = 1000;

    // Init TX slot
    txslot = 0;
    rxslot = 0;

    // Clear MAC events
    ethmac_sram_reader_ev_pending_write(ETHMAC_EV_SRAM_READER);
    ethmac_sram_writer_ev_pending_write(ETHMAC_EV_SRAM_WRITER);

    printf("[gPTP] Initialized. MAC=%02x:%02x:%02x:%02x:%02x:%02x\n",
           mac_addr[0], mac_addr[1], mac_addr[2],
           mac_addr[3], mac_addr[4], mac_addr[5]);
    printf("[gPTP] Clock ID=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
           g->clock_id[0], g->clock_id[1], g->clock_id[2], g->clock_id[3],
           g->clock_id[4], g->clock_id[5], g->clock_id[6], g->clock_id[7]);
    printf("[gPTP] Base addend=%lu (clk=%lu Hz)\n",
           (unsigned long)g->base_addend, (unsigned long)clk_freq);
}

// ---------------------------------------------------------------------------
// Main poll loop — call from main()
// ---------------------------------------------------------------------------

void gptp_poll(gptp_t *g)
{
    uint32_t now = gptp_uptime_ms();

    // NOTE: RX is now handled by the central dispatcher in main.c.
    // gptp_process_rx() is called from there for PTP frames.

    // Periodic Pdelay_Req
    if (now - g->last_pdelay_time_ms >= g->pdelay_interval_ms) {
        // Check if previous pdelay completed
        if (g->pdelay_count > 0 && !g->pdelay_fup_received) {
            g->pdelay_lost_count++;
            g->pdelay_timeout_count++;
            if (g->pdelay_lost_count > PDELAY_LOST_RESPONSES_ALLOWED) {
                // Lost too many — path delay unreliable
                if (g->state == GPTP_STATE_SLAVE) {
                    printf("[gPTP] Pdelay timeout — lost lock\n");
                }
            }
        }
        gptp_send_pdelay_req(g);
    }

    // Sync timeout check
    if (g->state == GPTP_STATE_SLAVE) {
        uint32_t sync_timeout_ms = 3000;  // 3× 1-second Sync interval
        if (now - g->last_sync_time_ms > sync_timeout_ms) {
            printf("[gPTP] Sync timeout — returning to LISTENING\n");
            g->state = GPTP_STATE_LISTENING;
            g->servo_locked = 0;
        }
    }
}
