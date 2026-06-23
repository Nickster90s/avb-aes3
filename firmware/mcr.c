// Media Clock Recovery — see mcr.h.
//
// C1: bind + dispatch + per-stream packet counting.
// C2 (current): parse CRF header (subtype 0x04), extract timestamps,
//     pair the most-recent timestamp with our hardware-stamped local
//     RX time, store the (avtp, local, offset) tuple for the C3 servo.
// C3: feed pairs into the MCR PI servo / NCO.

#include "mcr.h"
#include "avtp_const.h"   // AVTP_SUBTYPE_CRF
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

// CRF pull factor as a rational num/den (IEEE 1722-2016 Table 28). effective
// media rate = base_frequency * num / den. Mirrors phc_freq_sync apply_pull().
static inline void crf_pull_frac(uint8_t pull, uint32_t *num, uint32_t *den) {
    switch (pull) {
        case 1: *num = 1000;  *den = 1001;  break;   // base / 1.001  (e.g. 44.1k pulldown)
        case 2: *num = 1001;  *den = 1000;  break;   // base * 1.001
        case 3: *num = 24;    *den = 25;    break;   // base * 24/25
        case 4: *num = 25;    *den = 24;    break;   // base * 25/24
        case 5: *num = 10000; *den = 10001; break;   // base / 1.0001
        case 6: *num = 10001; *den = 10000; break;   // base * 1.0001
        default:*num = 1;     *den = 1;     break;   // pull 0 = 1/1
    }
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
    // Write the default to the NCO so it starts at the right rate even
    // before a CRF stream binds.
    mcr_increment_write(m->base_increment);
    m->watchdog_reset_active = 1;
    m->gptp_locked_base = m->base_increment;
}

void mcr_set_gptp(mcr_state_t *m, const gptp_t *g)
{
    m->gptp = g;
    m->gptp_locked_base = m->base_increment;
    m->pres_base_last   = 0;   // force the first pres_base CSR write
}

void mcr_set_clock_source(mcr_state_t *m, uint8_t cs)
{
    cs = (cs == 1) ? 1 : 0;
    if (cs == m->cs) return;
    m->cs = cs;
    // Re-baseline so the switch converges cleanly: snap the NCO to the gPTP base
    // immediately (safe default). cs=1 re-converges to CRF once samples arrive;
    // cs=0 holds the gPTP-disciplined base and ignores any bound CRF stream.
    m->have_prev          = 0;
    m->servo_consumed     = 1;
    m->servo_integral     = 0;
    m->servo_locked       = 0;
    m->lock_streak        = 0;
    m->crf_meas_count     = 0;   // restart avtp-spacing rate-recovery warmup
    m->crf_rate_valid     = 0;
    m->crf_ppb_filt       = 0;
    m->crf_log_idx        = 0;
    m->crf_log_count      = 0;
    m->crf_log_last_ms    = 0;
    m->current_increment  = m->gptp_locked_base;
    mcr_increment_write(m->gptp_locked_base);
    m->watchdog_reset_active = 1;
    printf("[MCR] clock source -> %s\n",
           cs == 1 ? "CRF (input stream)" : "gPTP (internal)");
}

// Compute the NCO increment that produces exactly 48000 gPTP-Hz. The nominal
// base_increment makes the NCO emit 48000 at the NOMINAL sys_clk; the actual
// crystal is off by tens of ppm. gPTP already measures that error and applies
// current_addend_full/base_addend_full to discipline the TSU. The NCO shares
// the same sys_clk, so the same fractional correction makes it emit exactly
// 48000 gPTP-Hz. Falls back to the raw nominal base until gPTP locks.
// Clamp the gPTP frequency correction to a physically-plausible crystal-error
// window: base_increment >> 9 ≈ ±0.195% ≈ ±1953 ppm — roughly 10-20x any real
// crystal+GM offset (spec'd tens of ppm), so it never clips a genuine lock but
// rejects pathological windup. Without it, a gPTP servo that winds its addend
// up with NO real grandmaster (e.g. a point-to-point link, or a transient GM
// loss) drives `corr` huge and the gPTP-disciplined NCO over-revs the media
// clock — observed as the AAF talker emitting ~112k pkt/s instead of 8000 on
// the point-to-point bring-up. This bounds the NCO to a sane rate regardless of
// gPTP state, so a gPTP unlock can never over-rev the talker.
#define MCR_GPTP_CORR_SHIFT  9

static uint32_t mcr_compute_gptp_base(const mcr_state_t *m)
{
    const gptp_t *g = m->gptp;
    if (!g || !g->servo_locked || g->base_addend_full == 0)
        return m->base_increment;
    int64_t d    = (int64_t)g->current_addend_full - (int64_t)g->base_addend_full;
    int64_t corr = ((int64_t)m->base_increment * d) / (int64_t)g->base_addend_full;
    int64_t maxc = (int64_t)m->base_increment >> MCR_GPTP_CORR_SHIFT;
    if (corr >  maxc) corr =  maxc;
    if (corr < -maxc) corr = -maxc;
    int64_t inc  = (int64_t)m->base_increment + corr;
    if (inc < 1) inc = 1;
    return (uint32_t)inc;
}

// Deadband (NCO increment units) for re-writing the gPTP-disciplined base.
// 1 unit ~ 0.6 ppm at sys_clk; a couple of units avoids CSR thrash on servo
// jitter while staying far under any audible drift.
#define MCR_GPTP_DEADBAND  2

// Stale threshold: 1000 ms. Class A CRF arrives at 8 kHz; a real talker
// stopping streaming is on the order of seconds, not ~200 ms. 200 ms was
// too twitchy: brief packet-burst losses (seq_err clusters) inside the
// window fired the stale path and toggled `servo_locked` 1→0→1, which
// then triggered unsolicited STREAM_INPUT MEDIA_LOCKED/UNLOCKED pushes
// in avdecc.c (`avdecc_listener_lock_changed`) and Hive lit up the patch
// even though there was no real disconnect (2026-05-28 bench observation).
// 1000 ms still snaps the NCO back well before any audible drift can
// accumulate (NCO retained-tuning drifts at sub-ppm; 1 s = sub-µs).
#define MCR_STALE_THRESHOLD_MS  1000

void mcr_watchdog_tick(mcr_state_t *m, uint32_t now_ms)
{
    // Keep the gPTP-locked media-clock reference current (runs every tick).
    // (REMOVED 2026-06-23: the aaf_pkt_pres_base_write mirror. The pres_base CSR
    // is deleted — it aliased into the AAF pres on openXC7 and leaked ~the NCO
    // increment into avtp_ts instead of pres_offset; the pres-ramp dilation that
    // consumed it is gone. gptp_locked_base is still tracked for the NCO.)
    uint32_t gbase = mcr_compute_gptp_base(m);
    m->gptp_locked_base = gbase;

    if (m->cs != 1 || !m->bound) {
        // Follow the gPTP-DISCIPLINED base (exactly 48000 gPTP-Hz), NOT the raw
        // nominal-crystal base. This path runs whenever CRF is NOT the active
        // media clock: clock source = gPTP/internal (cs=0, even if a CRF stream
        // is connected -- it is IGNORED), or no CRF bound. Re-apply when it moves
        // beyond the deadband so it tracks the gPTP servo. THIS is what keeps a
        // cs=0 endpoint on pure gPTP and prevents a stray CRF binding from
        // pulling the media clock off-rate (the "out of sync after a while" bug).
        uint32_t d2 = (gbase > m->current_increment) ? gbase - m->current_increment
                                                      : m->current_increment - gbase;
        if (!m->watchdog_reset_active || d2 > MCR_GPTP_DEADBAND) {
            m->current_increment    = gbase;
            mcr_increment_write(gbase);
            m->watchdog_reset_active = 1;
        }
        return;
    }
    if (m->rx_count != m->last_rx_count_snapshot) {
        // CRF arrived since last check — refresh window.
        m->last_rx_count_snapshot = m->rx_count;
        m->last_rx_check_ms       = now_ms;
        // Servo path will take over; clear the reset flag so a future
        // stale event triggers another snap-back.
        m->watchdog_reset_active  = 0;
        // During the avtp-spacing rate-recovery WARMUP (before crf_rate_valid)
        // hold the NCO at the gPTP base -- a good seed since the CRF rate is
        // within ppm of gPTP. mcr_process_rx takes over the exact recovered rate
        // once warmed up (then it owns current_increment; we don't fight it).
        if (!m->crf_rate_valid) {
            uint32_t d2 = (gbase > m->current_increment) ? gbase - m->current_increment
                                                          : m->current_increment - gbase;
            if (d2 > MCR_GPTP_DEADBAND) {
                m->current_increment = gbase;
                mcr_increment_write(gbase);
            }
        }
        return;
    }
    uint32_t age_ms = now_ms - m->last_rx_check_ms;
    if (age_ms > MCR_STALE_THRESHOLD_MS) {
        if (!m->watchdog_reset_active) {
            m->current_increment    = m->gptp_locked_base;
            mcr_increment_write(m->gptp_locked_base);
            m->servo_integral       = 0;
            m->have_prev            = 0;
            m->servo_locked         = 0;
            m->lock_streak          = 0;
            m->crf_meas_count       = 0;   // restart avtp-spacing rate-recovery warmup
            m->crf_rate_valid       = 0;
            m->watchdog_reset_active = 1;
            printf("[MCR] CRF stale %lums — increment snapped to base\n",
                   (unsigned long)age_ms);
        }
    }
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
    m->hw_rx_count    = 0;
    // phc_freq_sync-style avtp-spacing rate recovery: fresh warmup per bind.
    m->crf_meas_count = 0;
    m->crf_rate_valid = 0;
    m->crf_ppb_filt   = 0;
    // CRF convergence ring-log: fresh capture for this bind (instrument).
    m->crf_log_idx      = 0;
    m->crf_log_count    = 0;
    m->crf_log_postlock = 0;
    m->crf_log_last_ms  = 0;
    // Don't touch base_increment / current_increment — preserve servo state
    // across rebinds so the integrator's accumulated tuning isn't lost.

    // Point the gateware CRFTimestampExtractor at this stream_id and enable
    // it. From now on it snoops the RX stream and queues (avtp_ts, local_ts)
    // pairs into its FIFO regardless of MAC RX slot pressure; mcr_pump_hw()
    // drains them into the servo. stream_id is big-endian, [0]=MSB.
    crf_ts_stream_id_hi_write(
        ((uint32_t)stream_id[0] << 24) | ((uint32_t)stream_id[1] << 16) |
        ((uint32_t)stream_id[2] <<  8) |  (uint32_t)stream_id[3]);
    crf_ts_stream_id_lo_write(
        ((uint32_t)stream_id[4] << 24) | ((uint32_t)stream_id[5] << 16) |
        ((uint32_t)stream_id[6] <<  8) |  (uint32_t)stream_id[7]);
    crf_ts_enabled_write(1);

    printf("[MCR] bound to stream "
           "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
           stream_id[0], stream_id[1], stream_id[2], stream_id[3],
           stream_id[4], stream_id[5], stream_id[6], stream_id[7]);
}

void mcr_unbind(mcr_state_t *m)
{
    m->bound = 0;
    // Force lock state to 0 — without this, track_clock_lock keeps
    // seeing servo_locked=1 after CRF disconnect because the servo
    // path stops getting updates and the last "locked" decision
    // sticks. Hive then shows CLOCK_DOMAIN Media Locked never
    // transitioning to Unlocked even when the source is gone.
    m->servo_locked    = 0;
    m->lock_streak     = 0;
    m->have_latest     = 0;
    m->servo_consumed  = 1;
    // Snap the NCO back to the gPTP-locked base now that the talker reference
    // is gone. Otherwise the gateware NCO keeps ticking at the last servo-
    // tuned rate, drifting against audio_clk and audibly clicking. (gPTP-locked
    // base, not raw nominal, so the free-run rate is the real network 48 kHz.)
    m->current_increment    = m->gptp_locked_base;
    mcr_increment_write(m->gptp_locked_base);
    m->watchdog_reset_active = 1;
    crf_ts_enabled_write(0);   // stop the gateware extractor queuing pairs
    printf("[MCR] unbound — increment reset to base\n");
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

    // ---- phc_freq_sync-style CRF RATE recovery (THE media rate) ------------
    // Recover the rate from the avtp TIMESTAMP SPACING, which directly encodes
    // the talker's media clock. We measure INTRA-packet: dt = last_ts - first_ts
    // spans (ts_count-1)*timestamp_interval media events -- immune to our RX
    // latency, to network jitter, AND to packet loss (one PDU). Compare to the
    // expected gPTP-ns for those events at the nominal rate; the deviation IS the
    // CRF-vs-nominal rate error. It is an OPEN-LOOP measurement (our NCO can't
    // influence the next packet), so we smooth it with a single-pole IIR -- NOT a
    // PI integrator (which winds up on an open-loop input -- that was the ~131ppm
    // bias that dropped the audio). Ref: avdecc-endpoint/tools/phc_freq_sync.c
    // (--src=crf), GenAVB avtp/crf.c crf_measure_period.
    // Pick the timestamp spacing: prefer INTRA-packet (last_ts - first_ts, immune
    // to packet loss); fall back to INTER-packet when the PDU carries a single
    // timestamp (this AxC stream: ts/pdu=1) using the previous packet's ts, still
    // in latest_avtp_ts (updated below, after this block). events = media events
    // spanned by that gap.
    int64_t  d_avtp    = 0;
    uint64_t events    = 0;
    int      have_meas = 0;
    if (ts_count >= 2) {
        d_avtp    = (int64_t)(avtp_ts - be64(ts_array));
        events    = (uint64_t)(ts_count - 1) * m->timestamp_interval;
        have_meas = 1;
    } else if (m->have_latest) {
        d_avtp    = (int64_t)(avtp_ts - m->latest_avtp_ts);
        events    = (uint64_t)m->timestamps_per_pdu * m->timestamp_interval;
        have_meas = 1;
    }
    if (have_meas && m->base_frequency && m->timestamp_interval && events) {
        uint32_t pnum, pden;
        crf_pull_frac(m->pull, &pnum, &pden);
        int64_t  expected = (int64_t)((events * 1000000000ull * pden) /
                                      ((uint64_t)m->base_frequency * pnum));
        if (expected > 0) {
            int64_t err_ppb = ((d_avtp - expected) * 1000000000ll) / expected;
            if (err_ppb < CRF_PPB_OUTLIER && err_ppb > -CRF_PPB_OUTLIER) {
                m->crf_last_err_ppb = err_ppb;
                if (m->crf_meas_count == 0)
                    m->crf_ppb_filt = err_ppb;                       // seed (no startup transient)
                else
                    m->crf_ppb_filt += (err_ppb - m->crf_ppb_filt) >> CRF_PPB_SHIFT;
                if (m->crf_meas_count < CRF_MEAS_SAMPLES) {
                    m->crf_meas_count++;
                    if (m->crf_meas_count >= CRF_MEAS_SAMPLES) m->crf_rate_valid = 1;
                }
                // Apply the recovered rate when CRF is the selected clock (cs=1).
                // inc = gptp_locked_base * (1 - err) : start from the gPTP-
                // disciplined base (carries our crystal correction) and apply the
                // measured CRF-vs-nominal deviation on top. err>0 (ts more spaced)
                // => CRF slower => lower inc.
                if (m->cs == 1 && m->crf_rate_valid) {
                    int64_t corr = ((int64_t)m->gptp_locked_base * m->crf_ppb_filt)
                                   / 1000000000ll;
                    int64_t inc  = (int64_t)m->gptp_locked_base - corr;
                    if (inc < 1) inc = 1;
                    m->current_increment = (uint32_t)inc;
                    mcr_increment_write(m->current_increment);
                    m->servo_locked = 1;
                }
            }
        }
    }
    // ------------------------------------------------------------------------

    ptp_timestamp_t rx = gptp_read_rx_timestamp();
    uint64_t local_ts = (uint64_t)rx.seconds * 1000000000ull + rx.nanoseconds;

    m->latest_avtp_ts    = avtp_ts;
    m->latest_local_ts   = local_ts;
    m->latest_offset_ns  = (int64_t)(avtp_ts - local_ts);
    m->have_latest       = 1;
    m->servo_consumed    = 0;
    m->rx_count++;
}

void mcr_usb_lock_reset(mcr_state_t *m)
{
    m->usb_integral   = 0;
    m->usb_level_filt = 32 << 8;   // start centred (Q8); gateware primes to here
}

void mcr_usb_lock(mcr_state_t *m, int fifo_level, int center)
{
    m->usb_last_level = fifo_level;     // diag: what the servo actually sees

    // Heavily low-pass the level (Q8) so the servo reacts only to genuine drift,
    // not host jitter / the intra-µframe sawtooth (gentle). The FIFO now starts
    // CENTRED (gateware prime + always-drain), so this only trims the small
    // host-vs-NCO offset, not a full buffer.
    m->usb_level_filt += ((fifo_level << 8) - m->usb_level_filt) >> USB_FILT_SHIFT;
    int level_f = m->usb_level_filt >> 8;

    // PI on the filtered level. error>0 (above centre) ⇒ consume slightly
    // faster. Integral carries the steady offset (FIFO holds centre); the
    // proportional damps. Anti-windup on the integral.
    int error = level_f - center;
    m->usb_integral += error;
    if (m->usb_integral >  USB_INT_CLAMP) m->usb_integral =  USB_INT_CLAMP;
    if (m->usb_integral < -USB_INT_CLAMP) m->usb_integral = -USB_INT_CLAMP;

    int64_t correction = (int64_t)error * USB_KP
                       + (m->usb_integral * USB_KI_NUM) / USB_KI_DEN;
    int64_t maxd = (int64_t)m->base_increment >> USB_CORR_SHIFT;   // ±~0.78% guard
    if (correction >  maxd) correction =  maxd;
    if (correction < -maxd) correction = -maxd;

    int64_t inc = (int64_t)m->base_increment + correction;
    if (inc < 1) inc = 1;
    if (inc > 0xFFFFFFFFLL) inc = 0xFFFFFFFFLL;
    m->current_increment = (uint32_t)inc;
    mcr_increment_write(m->current_increment);
}

void mcr_servo_update(mcr_state_t *m)
{
    // cs=1 only: the CRF servo drives the NCO solely when CRF is the selected
    // clock source. At cs=0 the NCO follows gPTP (mcr_watchdog_tick) and a
    // connected CRF stream is ignored. (mcr_pump_hw still drains the FIFO.)
    if (!m->bound || m->servo_consumed || m->cs != 1) return;
    m->servo_consumed = 1;

    int64_t off = m->latest_offset_ns;

    // First sample after bind — seed baselines, don't act yet.
    if (!m->have_prev) {
        m->prev_offset_ns    = off;
        m->crf_off_filt      = off;
        m->crf_off_win_start = off;
        m->crf_win_start_ms  = gptp_uptime_ms();
        m->have_prev         = 1;
        return;
    }

    int64_t delta = off - m->prev_offset_ns;   // rate-error per packet (ns)
    m->prev_offset_ns = off;

    // OUTLIER REJECTION (the 8ch / AAF-TX-load fix). Under AAF TX the CRF RX
    // drops/reorders frames (seq_err ~0.2% on HW) and the HW extractor can
    // mispair an avtp_ts with the wrong packet RX timestamp, so `off` jumps by
    // 100s of ms -> a giant `delta`. Without rejection the PI acts on it: the
    // NCO HUNTS (strobe 48356->48697) and the integral accumulates a steady
    // bias (+0.75%) -> the media clock wobbles, the local DAC sounds bad, and
    // the listener (AxC) can't lock to a jittering presentation clock. A locked
    // 48k media clock's per-packet offset change is < ~15 us even mid-converge;
    // reject anything bigger as a glitch. Baseline (prev_offset) is kept current
    // so a transient recovers in 1-2 packets. Same idea as gPTP pdelay-outlier
    // rejection. THIS is why "bound+locked" still had no usable/synced audio.
    #define MCR_DELTA_OUTLIER_NS 100000        /* 100 us */
    if (delta > MCR_DELTA_OUTLIER_NS || delta < -MCR_DELTA_OUTLIER_NS) {
        m->servo_outlier_rejects++;
        return;
    }

    // EWMA-smooth the offset -> attenuates per-packet RX jitter on the window
    // endpoints (avg|d|~238ns, 18us spikes observed on HW). Divides by 2^SHIFT.
    m->crf_off_filt += (off - m->crf_off_filt) >> CRF_OFF_FILT_SHIFT;

    // Rolling raw-delta stats (diagnostics / 'C' instrument) — the INPUT jitter.
    int64_t abs_delta = (delta < 0) ? -delta : delta;
    if (abs_delta > m->delta_max_abs) m->delta_max_abs = abs_delta;
    m->delta_sum_abs += abs_delta;
    m->delta_window_count++;

    // FIXED-WINDOW servo (#2a): adjust the NCO only once per CRF_WINDOW_MS, on the
    // drift integrated over the window. The per-packet jitter -- which the old
    // per-packet servo turned straight into ~560ppm NCO rate hunting -- averages
    // out. Integral accumulates the FULL windowed drift (phase), so the
    // convergence rate matches the old per-packet loop; the proportional damps on
    // the per-ms drift. Same gains, same lock thresholds.
    uint32_t now_ms = gptp_uptime_ms();
    uint32_t dt_ms  = now_ms - m->crf_win_start_ms;
    if (dt_ms >= CRF_WINDOW_MS) {
        int64_t rate_err = m->crf_off_filt - m->crf_off_win_start;   // drift over window (ns)
        m->crf_off_win_start = m->crf_off_filt;
        m->crf_win_start_ms  = now_ms;
        int64_t avg_drift = rate_err / (int64_t)dt_ms;               // per-ms drift

        // The media-clock RATE and LOCK are now driven by the phc_freq_sync-style
        // avtp-timestamp-spacing recovery in mcr_process_rx. This legacy windowed
        // (avtp_ts - local_ts) offset is RATE-INSENSITIVE (both sides gPTP-locked
        // advance together) so it CANNOT recover the rate -- the old PI servo just
        // integrated startup jitter into a ~131ppm bias that dropped the audio. We
        // keep this window only to feed the 'C' instrument's offset/drift view; no
        // NCO write, no lock decision here.
        (void)avg_drift;
        m->servo_step_count++;
    }

    // CRF convergence ring-log (instrument): ~10ms-decimated snapshot of the
    // recovery (offset, rate delta, NCO correction, lock). ROLLING window (no
    // freeze): CRF locks instantly (NCO starts at the gPTP rate ~= the CRF rate),
    // so the useful view is STEADY-STATE jitter/hunting, not a convergence ramp.
    // A 'C' dump always shows the last 320 entries (~3.2 s). Observation only.
    {
        if (m->crf_log_count == 0 || (uint32_t)(now_ms - m->crf_log_last_ms) >= 10) {
            m->crf_log_last_ms = now_ms;
            int64_t o = off, d = delta;
            int64_t ic = (int64_t)m->current_increment - (int64_t)m->base_increment;
            if (o >  2000000000LL) o =  2000000000LL;
            if (o < -2000000000LL) o = -2000000000LL;
            if (d >  2000000000LL) d =  2000000000LL;
            if (d < -2000000000LL) d = -2000000000LL;
            if (ic >  2000000000LL) ic =  2000000000LL;
            if (ic < -2000000000LL) ic = -2000000000LL;
            m->crf_log[m->crf_log_idx].offset_ns = (int32_t)o;
            m->crf_log[m->crf_log_idx].delta_ns  = (int32_t)d;
            m->crf_log[m->crf_log_idx].inc_delta = (int32_t)ic;
            m->crf_log[m->crf_log_idx].locked    = m->servo_locked;
            m->crf_log_idx = (uint16_t)((m->crf_log_idx + 1) % 320);
            if (m->crf_log_count < 320) m->crf_log_count++;
        }
    }
}

// Dump the CRF convergence ring-log (console 'C'). Shows the recovery curve so
// CRF media-clock lock + per-packet jitter are measurable from one capture.
// offset_ns = avtp-local; delta_ns = per-packet rate error; inc_delta = NCO
// increment correction; index is the time axis at ~10 ms/entry.
void mcr_dump_conv_log(const mcr_state_t *m)
{
    uint16_t n     = m->crf_log_count;
    uint16_t start = (n < 320) ? 0 : m->crf_log_idx;   // oldest entry (handles wrap)
    int first_lock = -1;
    for (uint16_t k = 0; k < n; k++) {
        uint16_t i = (uint16_t)((start + k) % 320);
        if (m->crf_log[i].locked) { first_lock = (int)k; break; }
    }
    // Range of the NCO correction + offset over the logged window — the NCO
    // "hunt" magnitude is the before/after metric for the fixed-window change.
    int32_t inc_min = 0x7fffffff, inc_max = -0x7fffffff - 1;
    int32_t off_min = 0x7fffffff, off_max = -0x7fffffff - 1;
    for (uint16_t k = 0; k < n; k++) {
        uint16_t i = (uint16_t)((start + k) % 320);
        int32_t ic = m->crf_log[i].inc_delta, of = m->crf_log[i].offset_ns;
        if (ic < inc_min) inc_min = ic; if (ic > inc_max) inc_max = ic;
        if (of < off_min) off_min = of; if (of > off_max) off_max = of;
    }
    printf("\n[CRF-CONV] entries=%u (~10 ms/entry)  bound=%d locked=%d\n",
           (unsigned)n, m->bound, m->servo_locked);
    printf("  jitter(rolling win): max|d|=%ld ns  avg|d|=%ld ns  outlier_rejects=%lu seq_err=%lu\n",
           (long)m->delta_max_abs,
           (long)(m->delta_window_count ? (m->delta_sum_abs / (int64_t)m->delta_window_count) : 0),
           (unsigned long)m->servo_outlier_rejects, (unsigned long)m->seq_errors);
    if (n)
        printf("  NCO-hunt: inc_delta range [%ld..%ld] span=%ld units | offset range span=%ld ns\n",
               (long)inc_min, (long)inc_max, (long)(inc_max - inc_min),
               (long)(off_max - off_min));
    if (first_lock >= 0)
        printf("  LOCK at entry %d (~%d ms after first logged sample)\n",
               first_lock, first_lock * 10);
    else
        printf("  (not locked within the logged window)\n");
    printf("  idx   offset_ns    delta_ns   inc_delta  lk\n");
    for (uint16_t k = 0; k < n; k++) {
        uint16_t i = (uint16_t)((start + k) % 320);
        printf("  %3u  %10ld  %10ld  %10ld   %u\n", (unsigned)k,
               (long)m->crf_log[i].offset_ns, (long)m->crf_log[i].delta_ns,
               (long)m->crf_log[i].inc_delta, (unsigned)m->crf_log[i].locked);
    }
}

void mcr_pump_hw(mcr_state_t *m)
{
    if (!m->bound) return;
    // Bound the per-call drain so a backlog can't stall the main loop.
    int guard = 64;
    while (crf_ts_level_read() && guard--) {
        uint64_t avtp_ts = ((uint64_t)crf_ts_avtp_hi_read() << 32)
                         |  (uint64_t)crf_ts_avtp_lo_read();
        uint64_t sec     = crf_ts_local_sec_read();
        uint32_t ns      = crf_ts_local_ns_read();
        crf_ts_pop_write(1);                   // advance FIFO head

        uint64_t local_ts = sec * 1000000000ull + ns;
        m->latest_offset_ns = (int64_t)(avtp_ts - local_ts);
        m->have_latest      = 1;
        m->servo_consumed   = 0;
        m->hw_rx_count++;
        m->rx_count++;          // keep the stale watchdog fed from the HW source
        mcr_servo_update(m);    // run the PI servo on this sample
    }
}
