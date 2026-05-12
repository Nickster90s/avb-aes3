// IEEE 802.1AS (gPTP) — Generalized Precision Time Protocol
// Bare-metal implementation for LiteX SoC with LiteEthTSU
//
// Slave-only, peer-to-peer delay mechanism, Layer 2 transport.

#ifndef GPTP_H
#define GPTP_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

#define PTP_ETHERTYPE       0x88F7

// gPTP multicast destinations (Layer 2)
// 01:80:C2:00:00:0E — used for all gPTP messages in 802.1AS
#define GPTP_MCAST_ADDR     {0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E}

// PTP message types (low nibble of first header byte)
#define PTP_MSG_SYNC            0x0
#define PTP_MSG_PDELAY_REQ      0x2
#define PTP_MSG_PDELAY_RESP     0x3
#define PTP_MSG_FOLLOW_UP       0x8
#define PTP_MSG_PDELAY_RESP_FUP 0xA
#define PTP_MSG_ANNOUNCE        0xB
#define PTP_MSG_SIGNALING       0xC

// PTP header flags. The 16-bit flags field is at offset 6 in the PTP header
// and is transmitted big-endian: flags[0] = byte 6, flags[1] = byte 7.
// twoStepFlag lives at bit 1 of byte 6 → 0x0200 in the 16-bit word.
// ptpTimescale lives at bit 3 of byte 7 → 0x0008 in the 16-bit word.
#define PTP_FLAG_TWO_STEP       0x0200
#define PTP_FLAG_PTP_TIMESCALE  0x0008

// Header sizes
#define PTP_HEADER_LEN          34
#define PTP_TIMESTAMP_LEN       10  // 6 bytes seconds + 4 bytes nanoseconds
#define PTP_PDELAY_REQ_LEN      (PTP_HEADER_LEN + 20)
#define PTP_PDELAY_RESP_LEN     (PTP_HEADER_LEN + 20)
#define PTP_PDELAY_RESP_FUP_LEN (PTP_HEADER_LEN + 20)
#define ETH_HEADER_LEN          14

// Clock identity (8 bytes, derived from MAC address with FF:FE inserted)
#define CLOCK_ID_LEN            8
#define PORT_ID_LEN             10  // clock identity + 2-byte port number

// gPTP domain
#define GPTP_DOMAIN             0

// Intervals (as log2 values, per 802.1AS)
#define LOG_PDELAY_REQ_INTERVAL 0   // 1 second
#define LOG_SYNC_INTERVAL      -3   // 125 ms (typical gPTP)
#define LOG_ANNOUNCE_INTERVAL   0   // 1 second

// Timeouts
#define SYNC_RECEIPT_TIMEOUT_INTERVALS  3
#define ANNOUNCE_RECEIPT_TIMEOUT        3  // intervals

// Pdelay measurement
#define PDELAY_LOST_RESPONSES_ALLOWED   3

// PI servo gains for addend (frequency) control.
// Each unit of full_addend changes the TSU rate by 1e9/2^52 ≈ 2.22e-7
// ns/cycle, which at 50 MHz sysclk is ~11.1 ns/sec rate adjustment.
// Per Sync interval (~125 ms): 1 addend unit = ~1.39 ns of phase change.
//
// Kp = 72/1000 ≈ 0.072 picks ~10% phase correction per Sync (gentle).
// Ki = 72/1_000_000 = 100x slower — handles steady-state bias drift.
// Both terms multiplied by the offset (in ns) to get addend delta.
//
// The integrator absorbs the constant LiteEth capture-point bias
// (~145–290 µs depending on P&R) into a steady-state addend offset:
// no static calibration constant needed; PI does it for free.
#define SERVO_KP_NUM            72
#define SERVO_KP_DEN            1000
#define SERVO_KI_NUM            72
#define SERVO_KI_DEN            1000000

// Anti-windup: clamp |integral| at 100 ms — empirically the integrator
// needs to settle around 50–80 ms-worth of "mass" to fully cancel the
// LiteEth capture-point bias (the value depends on P&R variance). 10 ms
// is too tight (saturates and leaves ~50 µs DC residual); 1 s is overkill
// (lets the startup transient overshoot). 100 ms gives ~2x headroom over
// the worst observed steady-state requirement.
#define SERVO_INTEGRAL_CLAMP_NS 100000000LL

// ---------------------------------------------------------------------------
// PTP timestamp (80-bit: 48-bit seconds + 32-bit nanoseconds)
// ---------------------------------------------------------------------------

typedef struct {
    uint64_t seconds;       // 48-bit (upper 16 bits unused)
    uint32_t nanoseconds;
} ptp_timestamp_t;

// ---------------------------------------------------------------------------
// gPTP state
// ---------------------------------------------------------------------------

typedef enum {
    GPTP_STATE_INIT,
    GPTP_STATE_LISTENING,
    GPTP_STATE_SLAVE,
} gptp_state_t;

typedef struct {
    gptp_state_t state;

    // Our identity
    uint8_t  our_mac[6];
    uint8_t  clock_id[CLOCK_ID_LEN];
    uint16_t port_number;

    // Master identity (learned from Announce/Sync)
    uint8_t  master_clock_id[CLOCK_ID_LEN];
    uint16_t master_port_number;

    // Grandmaster info (learned from Announce) — surfaced in ADP/AVB_INTERFACE
    // so AVDECC controllers (e.g. Hive) show the actual GM ID and quality.
    uint8_t  gm_clock_id[CLOCK_ID_LEN];
    uint8_t  gm_priority1;
    uint8_t  gm_clock_class;
    uint8_t  gm_clock_accuracy;
    uint16_t gm_offset_scaled_log_variance;
    uint8_t  gm_priority2;
    uint8_t  gm_valid;                  // 1 = at least one Announce seen

    // Pdelay measurement
    uint16_t pdelay_seq_id;
    ptp_timestamp_t pdelay_t1;  // Our Pdelay_Req TX timestamp
    ptp_timestamp_t pdelay_t2;  // Responder's Pdelay_Resp RX timestamp
    ptp_timestamp_t pdelay_t3;  // Responder's Pdelay_Resp_Follow_Up TX timestamp
    ptp_timestamp_t pdelay_t4;  // Our Pdelay_Resp RX timestamp
    int64_t  mean_path_delay_ns;
    int      pdelay_lost_count;
    uint8_t  pdelay_resp_received;
    uint8_t  pdelay_fup_received;

    // Sync/Follow_Up
    uint16_t sync_seq_id;
    ptp_timestamp_t sync_rx_ts;       // Our RX timestamp of Sync
    ptp_timestamp_t sync_origin_ts;   // Master's TX timestamp (from Follow_Up)
    int64_t  sync_correction;         // Correction field from Sync (scaled ns)
    uint8_t  sync_received;

    // Clock servo
    int64_t  offset_from_master_ns;
    int64_t  freq_integral;           // Sum of past offsets (ns)
    uint64_t base_addend_full;        // Nominal 52-bit addend = (addend<<20)|frac
    uint64_t current_addend_full;
    uint8_t  servo_locked;
    uint32_t servo_step_count;

    // Timing
    uint32_t last_pdelay_time_ms;
    uint32_t last_sync_time_ms;
    uint32_t pdelay_interval_ms;      // From logMessageInterval

    // Statistics
    uint32_t sync_count;
    uint32_t pdelay_count;
    uint32_t pdelay_timeout_count;
    // RX message-type counters (debug)
    uint32_t rx_sync_count;
    uint32_t rx_followup_count;
    uint32_t rx_pdelay_req_count;
    uint32_t rx_pdelay_resp_count;
    uint32_t rx_pdelay_resp_fup_count;
    uint32_t rx_announce_count;
    uint32_t rx_other_count;
    uint32_t rx_wrong_domain_count;
    uint8_t  rx_last_msg_type;
    uint8_t  rx_last_domain;
} gptp_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

void gptp_init(gptp_t *g, const uint8_t *mac_addr);
void gptp_poll(gptp_t *g);

// Helpers called from main loop
void gptp_process_rx(gptp_t *g, const uint8_t *frame, uint32_t len);
void gptp_send_pdelay_req(gptp_t *g);
void gptp_servo_update(gptp_t *g);

// TSU access
ptp_timestamp_t gptp_read_time(void);
ptp_timestamp_t gptp_read_rx_timestamp(void);
ptp_timestamp_t gptp_read_tx_timestamp(void);
void gptp_set_addend_full(uint64_t addend_full);  // 52-bit: (addend<<20)|frac
void gptp_step_time(ptp_timestamp_t t);
void gptp_adjust_offset(int64_t offset_ns);

// Utility
uint32_t gptp_uptime_ms(void);
int64_t  gptp_ts_diff_ns(ptp_timestamp_t a, ptp_timestamp_t b);

#endif // GPTP_H
