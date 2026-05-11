// Media Clock Recovery — see mcr.h.
//
// C1: bind + dispatch + per-stream packet counting.
// C2 (current): parse CRF header (subtype 0x04), extract timestamps,
//     pair the most-recent timestamp with our hardware-stamped local
//     RX time, store the (avtp, local, offset) tuple for the C3 servo.
// C3: feed pairs into the MCR PI servo / NCO.

#include "mcr.h"
#include "avtp.h"   // AVTP_SUBTYPE_CRF
#include "gptp.h"   // gptp_read_rx_timestamp()
#include <generated/csr.h>
#include <string.h>
#include <stdio.h>

static inline uint16_t be16(const uint8_t *p) {
    return ((uint16_t)p[0] << 8) | p[1];
}
static inline uint32_t be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}
static inline uint64_t be64(const uint8_t *p) {
    return ((uint64_t)be32(p) << 32) | be32(p + 4);
}

void mcr_init(mcr_state_t *m, uint32_t sys_clk_freq, uint32_t fs)
{
    memset(m, 0, sizeof(*m));
    // base_increment = fs * 2^32 / sys_clk_freq, rounded.
    // 64-bit arithmetic to avoid overflow.
    uint64_t inc = ((uint64_t)fs << 32) / sys_clk_freq;
    if ((((uint64_t)fs << 32) % sys_clk_freq) > (sys_clk_freq / 2))
        inc++;
    m->base_increment    = (uint32_t)inc;
    m->current_increment = (uint32_t)inc;
}

void mcr_bind(mcr_state_t *m, const uint8_t *stream_id)
{
    memcpy(m->stream_id, stream_id, 8);
    m->bound          = 1;
    m->rx_count       = 0;
    m->rx_other_count = 0;
    m->bad_type_count = 0;
    m->seq_errors     = 0;
    m->have_last_seq  = 0;
    m->have_latest    = 0;
    m->have_prev      = 0;
    m->servo_consumed = 1;
    m->servo_integral = 0;
    m->servo_locked   = 0;
    m->servo_step_count = 0;
    // Don't touch base_increment / current_increment — preserve servo state
    // across rebinds so the integrator's accumulated tuning isn't lost.
    printf("[MCR] bound to stream "
           "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
           stream_id[0], stream_id[1], stream_id[2], stream_id[3],
           stream_id[4], stream_id[5], stream_id[6], stream_id[7]);
}

void mcr_unbind(mcr_state_t *m)
{
    m->bound = 0;
    printf("[MCR] unbound\n");
}

// CRF AVTPDU layout (IEEE 1722-2016 §10.2)
//  +0   cd|subtype  (0x04 with cd=0)
//  +1   sv|version|mr|r|fs|tu
//  +2   sequence_num
//  +3   type
//  +4..11   stream_id (8)
// +12..15   pull(3)|base_frequency(29), big-endian
// +16..17   crf_data_length (length of timestamps array in BYTES)
// +18..19   timestamp_interval
// +20..     timestamps (8 bytes each, big-endian gPTP ns)
#define CRF_HDR_LEN 20

void mcr_process_rx(mcr_state_t *m, const uint8_t *frame, uint32_t len)
{
    if (!m->bound) return;
    if (len < 14 + CRF_HDR_LEN) return;

    const uint8_t *pdu = frame + 14;
    if (pdu[0] != AVTP_SUBTYPE_CRF) return;    // defensive

    // Stream-id match
    for (int i = 0; i < 8; i++) {
        if (pdu[4 + i] != m->stream_id[i]) {
            m->rx_other_count++;
            return;
        }
    }

    // Sequence-number tracking (CRF byte 2)
    uint8_t seq = pdu[2];
    if (m->have_last_seq && seq != (uint8_t)(m->last_seq + 1))
        m->seq_errors++;
    m->last_seq      = seq;
    m->have_last_seq = 1;

    // Type — only AUDIO_SAMPLE is supported. Track but don't reject.
    uint8_t type = pdu[3];
    if (type != CRF_TYPE_AUDIO_SAMPLE)
        m->bad_type_count++;

    // pull(3) | base_frequency(29), big-endian
    uint32_t pb = be32(pdu + 12);
    m->pull           = (pb >> 29) & 0x7;
    m->base_frequency = pb & 0x1FFFFFFFu;
    m->type           = type;

    uint16_t crf_data_len = be16(pdu + 16);
    m->timestamp_interval = be16(pdu + 18);

    // timestamps_per_pdu = crf_data_length / 8.
    // Cap at the number of 8-byte slots actually present in the frame.
    uint32_t header_end = 14 + CRF_HDR_LEN;
    uint32_t bytes_avail = (len > header_end) ? (len - header_end) : 0;
    uint32_t ts_count = crf_data_len / 8;
    if (ts_count * 8 > bytes_avail) ts_count = bytes_avail / 8;
    if (ts_count == 0) return;
    m->timestamps_per_pdu = (ts_count > 0xFF) ? 0xFF : (uint8_t)ts_count;

    // Pair the LAST timestamp in the PDU with the packet's hardware RX
    // timestamp. The last ts is most temporally close to packet arrival,
    // minimizing residual transit time spread.
    const uint8_t *ts_array = pdu + CRF_HDR_LEN;
    uint64_t avtp_ts = be64(ts_array + 8 * (ts_count - 1));

    ptp_timestamp_t rx = gptp_read_rx_timestamp();
    uint64_t local_ts = (uint64_t)rx.seconds * 1000000000ull + rx.nanoseconds;

    m->latest_avtp_ts    = avtp_ts;
    m->latest_local_ts   = local_ts;
    m->latest_offset_ns  = (int64_t)(avtp_ts - local_ts);
    m->have_latest       = 1;
    m->servo_consumed    = 0;
    m->rx_count++;
}

void mcr_servo_update(mcr_state_t *m)
{
    if (!m->bound || m->servo_consumed) return;
    m->servo_consumed = 1;

    int64_t off = m->latest_offset_ns;

    // First sample after bind — capture baseline, don't act yet.
    if (!m->have_prev) {
        m->prev_offset_ns = off;
        m->have_prev      = 1;
        return;
    }

    int64_t delta = off - m->prev_offset_ns;   // rate-error per packet (ns)
    m->prev_offset_ns = off;

    // PI on delta. Integral accumulates the rate error → equivalent to
    // absolute phase drift since bind. Anti-windup clamp.
    m->servo_integral += delta;
    if (m->servo_integral >  MCR_INTEGRAL_CLAMP) m->servo_integral =  MCR_INTEGRAL_CLAMP;
    if (m->servo_integral < -MCR_INTEGRAL_CLAMP) m->servo_integral = -MCR_INTEGRAL_CLAMP;

    int64_t p_term = (delta             * MCR_KP_NUM) / MCR_KP_DEN;
    int64_t i_term = (m->servo_integral * MCR_KI_NUM) / MCR_KI_DEN;
    int64_t correction = -(p_term + i_term);

    if (correction >  MCR_INCREMENT_MAX_DELTA) correction =  MCR_INCREMENT_MAX_DELTA;
    if (correction < -MCR_INCREMENT_MAX_DELTA) correction = -MCR_INCREMENT_MAX_DELTA;

    int64_t inc = (int64_t)m->base_increment + correction;
    if (inc < 1) inc = 1;
    if (inc > 0xFFFFFFFFLL) inc = 0xFFFFFFFFLL;
    m->current_increment = (uint32_t)inc;

    // Write to NCO CSR
    mcr_increment_write(m->current_increment);

    m->servo_locked = (delta > -200 && delta < 200) ? 1 : 0;
    m->servo_step_count++;
}
