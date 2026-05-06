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

// PTP header flags
#define PTP_FLAG_TWO_STEP       (1 << 1)  // Byte offset 6, bit 1
#define PTP_FLAG_PTP_TIMESCALE  (1 << 3)  // Byte offset 6, bit 3

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

// PI servo gains (fixed-point, Q16.16)
#define SERVO_KP                0x00000400  // 0.0625 (proportional)
#define SERVO_KI                0x00000040  // 0.00390625 (integral)

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
    int64_t  freq_integral;           // Integral term (scaled)
    uint32_t base_addend;             // Nominal TSU addend value
    uint32_t current_addend;
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
void gptp_set_addend(uint32_t addend);
void gptp_step_time(ptp_timestamp_t t);
void gptp_adjust_offset(int64_t offset_ns);

// Utility
uint32_t gptp_uptime_ms(void);
int64_t  gptp_ts_diff_ns(ptp_timestamp_t a, ptp_timestamp_t b);

#endif // GPTP_H
