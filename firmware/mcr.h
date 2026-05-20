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
// Each unit of NCO increment ≈ sys_clk_freq / 2^32 ≈ 0.012 Hz (at 50 MHz),
// or about 0.24 ppm of fs=48000. PI runs on delta-offset between
// consecutive CRF packets (delta_offset = rate error in ns/packet).
// Integral of delta_offset = total accumulated phase = absolute offset
// drift, so PI-on-delta drives offset → constant in the locked state.
// PI gains. delta is rate-error per packet in ns (packet interval
// ≈ 2 ms for CRF at 48 kHz/96-intvl). NCO base_increment is
// (fs / sys_clk_freq) * 2^32 ≈ 4.12M for 48 kHz on 50 MHz. To
// correct delta ns over one 2 ms interval, the NCO needs
// delta * 2.06e-3 units of correction (≈ delta / 512). Our prior
// gain of 1/1 over-corrected by ~500×, which made the loop ring
// and breach the 500ns lock-hysteresis exit threshold ~50 times
// per second.
#define MCR_KP_NUM            1
#define MCR_KP_DEN            128
#define MCR_KI_NUM            1
#define MCR_KI_DEN            8192
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

    // Counters (debug)
    uint32_t rx_count;          // CRF packets matching our stream_id
    uint32_t rx_other_count;    // CRF packets we received but for a different stream
    uint32_t bad_type_count;    // PDUs with type != AUDIO_SAMPLE
    uint32_t seq_errors;
    uint8_t  last_seq;
    uint8_t  have_last_seq;
} mcr_state_t;

void mcr_init  (mcr_state_t *m, uint32_t sys_clk_freq, uint32_t fs);
void mcr_bind  (mcr_state_t *m, const uint8_t *stream_id);
void mcr_unbind(mcr_state_t *m);

// Called from the AVTP RX dispatcher for any AVTPDU with subtype == CRF.
// `frame` points at the Ethernet header; `len` is total frame length.
void mcr_process_rx(mcr_state_t *m, const uint8_t *frame, uint32_t len);

// Called once per main loop iteration; runs the PI servo if there's a
// new sample. Safe to call when not bound (no-op).
void mcr_servo_update(mcr_state_t *m);

#endif
