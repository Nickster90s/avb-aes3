// Media Clock Recovery — drives an audio sample clock from a CRF stream.
//
// On AVDECC CONNECT_RX of the CRF Media Clock Input, firmware calls
// mcr_bind() with the talker's stream_id. Subsequent CRF packets matching
// that stream_id feed the MCR PI servo (C2/C3).

#ifndef MCR_H
#define MCR_H

#include <stdint.h>

// CRF parameters (from IEEE 1722-2016 Table 25-27)
#define CRF_TYPE_AUDIO_SAMPLE   1
#define CRF_PULL_1_1            0

// PI servo gains for NCO frequency tuning.
//
// These are the original gains the project was built with — they were
// changed to 1/128 / 1/8192 during the unsolicited-push debugging
// (commit 0834b5a) to stop counter ringing, but that change made the
// servo 128–256× slower and stream lock effectively stopped happening.
// The counter "ringing" was a hysteresis/reporting problem, not a
// servo problem. Wide single-sample hysteresis (2 µs / 10 µs) below
// handles the counter side without crippling convergence.
#define MCR_KP_NUM            1
#define MCR_KP_DEN            1
#define MCR_KI_NUM            1
#define MCR_KI_DEN            32
#define MCR_INTEGRAL_CLAMP    1000000        // ±1 ms worth of phase
#define MCR_INCREMENT_MAX_DELTA  (1 << 24)   // ~4 Mppm guard against wild swings

typedef struct {
    uint8_t  bound;
    uint8_t  stream_id[8];

    // Stream parameters (from first valid PDU)
    uint32_t base_frequency;    // Hz, e.g. 48000
    uint8_t  pull;              // 0 = 1/1
    uint8_t  type;              // expect CRF_TYPE_AUDIO_SAMPLE
    uint16_t timestamp_interval; // Hz / packet rate (e.g. 8000 = 1ms class A)
    uint8_t  timestamps_per_pdu; // derived from crf_data_length / 8

    // Latest (avtp_ts, local_rx_ts) pair, both in ns since gPTP epoch
    uint64_t latest_avtp_ts;
    uint64_t latest_local_ts;
    int64_t  latest_offset_ns;  // avtp - local
    uint8_t  have_latest;
    uint8_t  servo_consumed;    // 0 = there's a new sample waiting

    // PI servo state
    int64_t  prev_offset_ns;
    uint8_t  have_prev;
    int64_t  servo_integral;
    uint32_t base_increment;    // Nominal NCO inc; set at init from sys_clk_freq + fs
    uint32_t current_increment; // Last value written to NCO CSR
    uint8_t  servo_locked;
    // Hysteresis state — enter LOCKED only after MCR_LOCK_STREAK
    // consecutive deltas below MCR_LOCK_ENTER_NS; exit only when a
    // delta exceeds MCR_LOCK_EXIT_NS. Single-sample threshold caused
    // 596 lock/unlock transitions per patch session because Class A
    // packet-to-packet jitter routinely hits ±200ns even when locked.
    uint8_t  lock_streak;
    uint32_t servo_step_count;
    // Diagnostic — rolling stats of |delta| over the last
    // MCR_DELTA_STAT_WINDOW samples, so we can see whether the
    // hysteresis thresholds are realistic for the network jitter
    // we're actually seeing.
    int64_t  delta_max_abs;       // max |delta| in current window
    int64_t  delta_sum_abs;       // sum of |delta| in current window
    uint32_t delta_window_count;  // samples in current window

    // Counters (debug)
    uint32_t rx_count;          // CRF packets matching our stream_id
    uint32_t hw_rx_count;       // (avtp,local) pairs drained from the gateware
                                // CRFTimestampExtractor FIFO — the flood-proof
                                // servo feed. See mcr_pump_hw().
    uint32_t rx_other_count;    // CRF packets we received but for a different stream
    uint32_t bad_type_count;    // PDUs with type != AUDIO_SAMPLE
    uint32_t seq_errors;
    uint8_t  last_seq;
    uint8_t  have_last_seq;

    // CRF stale watchdog: if no CRF arrives for > MCR_STALE_THRESHOLD_MS,
    // snap current_increment back to base_increment. Without this, the
    // last servo-tuned increment persists past CRF loss, drifting MCR's
    // strobe rate away from local audio_clk rate and causing the FIFO
    // between firmware DAC push and I2S consume to overflow/underrun.
    // last_rx_count_snapshot tracks the most-recently-observed rx_count;
    // when it stops growing for MCR_STALE_THRESHOLD_MS we declare stale.
    uint32_t last_rx_count_snapshot;
    uint32_t last_rx_check_ms;
    uint8_t  watchdog_reset_active;  // 1 while increment is held at base
} mcr_state_t;

void mcr_init  (mcr_state_t *m, uint32_t sys_clk_freq, uint32_t fs);
void mcr_bind  (mcr_state_t *m, const uint8_t *stream_id);
void mcr_unbind(mcr_state_t *m);

// Called from the AVTP RX dispatcher for any AVTPDU with subtype == CRF.
// `frame` points at the Ethernet header; `len` is total frame length.
void mcr_process_rx(mcr_state_t *m, const uint8_t *frame, uint32_t len);

// Drain the gateware CRFTimestampExtractor FIFO and feed each (avtp_ts,
// local_rx_ts) pair into the PI servo. This is the flood-proof servo input:
// the hardware extractor captures CRF timestamps directly off the RX stream,
// so the servo no longer depends on CRF frames surviving the 2-slot MAC RX.
// Call once per main-loop pass (drains all queued pairs). No-op when unbound.
void mcr_pump_hw(mcr_state_t *m);

// Called once per main loop iteration; runs the PI servo if there's a
// new sample. Safe to call when not bound (no-op).
void mcr_servo_update(mcr_state_t *m);

// Called once per main loop iteration. If CRF input has been stale for
// more than MCR_STALE_THRESHOLD_MS, snap current_increment back to
// base_increment so the NCO runs at the default fs (= local crystal rate).
// `now_ms` is monotonic ms (e.g. gptp_uptime_ms()).
void mcr_watchdog_tick(mcr_state_t *m, uint32_t now_ms);

#endif
