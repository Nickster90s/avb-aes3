// Media Clock Recovery — drives an audio sample clock from a CRF stream.
//
// On AVDECC CONNECT_RX of the CRF Media Clock Input, firmware calls
// mcr_bind() with the talker's stream_id. Subsequent CRF packets matching
// that stream_id feed the MCR PI servo (C2/C3).

#ifndef MCR_H
#define MCR_H

#include <stdint.h>
#include "gptp.h"   // gptp_t — for gPTP-disciplined NCO (cs=0 media clock)

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

// Fixed-window CRF servo (#2a, jitter smoothing). The old per-packet servo
// turned each packet's RX jitter (avg|d|~238ns, 18us spikes) directly into NCO
// rate hunting (~560ppm span measured). Instead, adjust the NCO only once per
// CRF_WINDOW_MS on the drift integrated over the window (per-packet jitter
// averages out), with the window endpoints EWMA-smoothed (÷2^FILT_SHIFT) to
// attenuate endpoint RX jitter. The integral accumulates the FULL windowed
// drift so the phase-convergence rate is unchanged vs per-packet.
#define CRF_WINDOW_MS         32   // fixed servo window
#define CRF_OFF_FILT_SHIFT     3   // EWMA on offset (÷8) for endpoint jitter

// phc_freq_sync-style CRF rate recovery (intra-packet avtp-spacing -> IIR LPF).
#define CRF_PPB_SHIFT          4        // IIR alpha = 1/16 (open-loop low-pass)
#define CRF_MEAS_SAMPLES      16        // warmup packets before crf_rate_valid
#define CRF_PPB_OUTLIER   300000        // reject |err| > 300 ppm (loss / epoch blip)

// USB-source media-clock recovery (NCO follows the USB block FIFO). Used when
// we are the USB→AVB talker + clock master and NOT locked to a CRF: servo the
// NCO so AVTP consumption exactly tracks the USB host delivery rate, keeping
// the block FIFO centred → bit-perfect regardless of whether the host honours
// async feedback (Linux snd-usb-audio does not). Called ~1 kHz. Gains tunable.
// PI servo, properly damped. The DC correction needed (~16500 increment units
// = the host's ~0.4% rate excess) is large, so pure-P would need a huge gain to
// hold near centre and would saturate/bang-bang. The INTEGRAL supplies the DC
// offset; the proportional damps. Tuned for ζ≈0.68 at ~1 kHz update on the
// FIFO-integrator plant (g≈0.01164 Hz/unit): ωn=√(g·KI·1000), ζ=KP·√g/(2·√(KI·
// 1000)). KP=400, KI=1 → ζ≈0.68, settles ~1.7 s, no saturation at the operating
// point. (The first PI used KP=200/KI=4 → ζ≈0.17, severely underdamped → hunt.)
#define USB_KP                400       // proportional (damping), per FIFO-block error
#define USB_KI_NUM            1         // integral (DC offset), per block-error per call
#define USB_KI_DEN            1
#define USB_INT_CLAMP         100000    // anti-windup (must reach the host offset)
// GENTLE-DRIFT shaping: the FIFO now starts CENTRED (gateware prime + always-
// drain), so the servo only trims the small host-vs-NCO offset (a few frames/s)
// and slow drift — NOT a full FIFO. Heavily low-pass the level (≈2^5 ≈ 32 ms)
// so the servo ignores host jitter / the intra-µframe sawtooth and reacts only
// to genuine drift. Authority capped tight (±~0.78%) — far more than the real
// offset needs, but bounds any pathological excursion.
#define USB_FILT_SHIFT        5         // level IIR time constant (~2^5 updates)
#define USB_CORR_SHIFT        6         // correction clamp = base_increment >> 6 (±1.56%)

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
    int64_t  usb_integral;      // USB-FIFO-lock servo integral (DC rate offset)
    int32_t  usb_level_filt;    // USB-FIFO-lock servo: low-passed level (Q8)
    int32_t  usb_last_level;    // diag: last FIFO level the USB servo read
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
    uint32_t servo_outlier_rejects;  // deltas rejected as glitches (dropped/mispaired CRF ts)
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

    // gPTP-disciplined media clock (cs=0). The NCO base_increment is computed
    // from the NOMINAL sys_clk, so free-running it emits 48000 ± crystal-error
    // Hz — NOT the network's 48000 gPTP-Hz, so a listener's buffer drifts.
    // gPTP already measures the sys_clk-vs-GM ratio (current_addend_full /
    // base_addend_full); we apply it to the NCO so it produces exactly 48000
    // gPTP-Hz. gptp_locked_base = that disciplined increment, recomputed each
    // watchdog tick; also pushed to the AAF packetizer's pres_base CSR so the
    // presentation-time ramp uses the SAME reference (no double correction).
    const gptp_t *gptp;
    // AVDECC clock source select (CLOCK_DOMAIN.clock_source_index): 0 = INTERNAL
    // (pure gPTP-disciplined NCO; a connected CRF stream is IGNORED), 1 = INPUT
    // STREAM (CRF servo drives the NCO when bound). Set via mcr_set_clock_source
    // from the AVDECC SET_CLOCK_SOURCE callback. Default 0.
    uint8_t  cs;
    uint32_t gptp_locked_base;       // base_increment scaled by the gPTP rate ratio
    uint32_t pres_base_last;         // last value written to aaf_pkt pres_base (deadband)

    // --- CRF convergence ring-log (instrument, mirrors the gPTP 'G' log) ----
    // While a CRF stream is bound, a ~10ms-decimated snapshot of the recovery:
    // offset (avtp-local), per-packet rate delta, NCO increment correction, and
    // lock state. Time-decimated via gptp_uptime_ms (valid: CRF recovery needs
    // gPTP locked) so a multi-second convergence fits 320 entries; freezes 8
    // entries after lock so a later 'C' dump shows the whole curve. Pure
    // observation -- touches no control path. Reset on each bind.
    struct {
        int32_t offset_ns;   // avtp_ts - local_ts at this snapshot (saturated)
        int32_t delta_ns;    // last per-packet rate delta (saturated)
        int32_t inc_delta;   // current_increment - base_increment (NCO correction)
        uint8_t locked;      // servo_locked at this snapshot
    } crf_log[320];
    uint16_t crf_log_idx;
    uint16_t crf_log_count;
    uint8_t  crf_log_postlock;     // entries logged since lock (freezes at 8)
    uint32_t crf_log_last_ms;

    // Fixed-window CRF servo state (#2a) — legacy (avtp-vs-local) diagnostics only
    int64_t  crf_off_filt;         // EWMA-smoothed offset (endpoint jitter filter)
    int64_t  crf_off_win_start;    // crf_off_filt snapshot at window start
    uint32_t crf_win_start_ms;     // window start (gptp_uptime_ms)

    // phc_freq_sync-style CRF RATE recovery (intra-packet avtp-spacing). The
    // media rate is recovered from the avtp TIMESTAMP SPACING (which encodes the
    // talker's media clock), open-loop, IIR-smoothed -- NOT a PI integrator and
    // NOT the rate-insensitive (avtp - local) offset. See mcr.c mcr_process_rx.
    int64_t  crf_ppb_filt;         // IIR-smoothed rate deviation from nominal (ppb, signed)
    int64_t  crf_last_err_ppb;     // most recent raw per-packet error (diagnostics)
    uint16_t crf_meas_count;       // warmup sample counter (-> crf_rate_valid)
    uint8_t  crf_rate_valid;       // 1 once warmed up: NCO tracks the recovered CRF rate
} mcr_state_t;

void mcr_init  (mcr_state_t *m, uint32_t sys_clk_freq, uint32_t fs);
// Give the MCR a gPTP handle so it can discipline the free-running (cs=0) NCO
// to the network media rate. Call once after mcr_init + gptp_init.
void mcr_set_gptp(mcr_state_t *m, const gptp_t *g);
// Select the media clock source (AVDECC SET_CLOCK_SOURCE). 0 = INTERNAL/gPTP
// (ignore CRF), 1 = CRF input stream. Re-baselines the servo + snaps the NCO to
// the gPTP base so the switch converges cleanly.
void mcr_set_clock_source(mcr_state_t *m, uint8_t cs);
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

// Dump the CRF convergence ring-log over UART (console 'C'): bound state +
// delta-jitter stats + per-snapshot (offset_ns, delta_ns, inc_delta, locked).
// Diagnostic only -- shows the CRF media-clock recovery curve from one capture.
void mcr_dump_conv_log(const mcr_state_t *m);

// USB-source clock recovery: drive the NCO so the USB block FIFO stays at
// `center`, i.e. AVTP consumption tracks the USB host's delivery rate. Call at
// ~1 kHz while we're the USB→AVB talker and not CRF-bound. fifo_level/center in
// block units (0..depth). Resets via mcr_usb_lock_reset().
void mcr_usb_lock(mcr_state_t *m, int fifo_level, int center);
void mcr_usb_lock_reset(mcr_state_t *m);

// Called once per main loop iteration. If CRF input has been stale for
// more than MCR_STALE_THRESHOLD_MS, snap current_increment back to
// base_increment so the NCO runs at the default fs (= local crystal rate).
// `now_ms` is monotonic ms (e.g. gptp_uptime_ms()).
void mcr_watchdog_tick(mcr_state_t *m, uint32_t now_ms);

#endif
