// IEEE 1722 AVTP — Audio Video Transport Protocol
// IEC 61883-6 AM824 format for pro audio
//
// Firmware-based talker + listener for a single stereo stream.
// Packs/unpacks 24-bit PCM audio into AVTP frames via LiteEth MAC SRAM.

#ifndef AVTP_H
#define AVTP_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

#define AVTP_ETHERTYPE          0x22F0

// AVTP multicast: 91:E0:F0:00:FE:xx (class A = 00, class B = 01..FF)
// For initial test we use a fixed multicast address.
#define AVTP_MCAST_ADDR         {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x00}

// Subtype
#define AVTP_SUBTYPE_61883_IIDC 0x00

// IEC 61883-6 constants
#define CIP_FMT_61883_6        0x10
#define CIP_SFC_48K             2       // SFC = 2 for 48 kHz
#define CIP_SFC_96K             4       // SFC = 4 for 96 kHz
#define AM824_LABEL_MBLA        0x40    // Multi-bit linear audio

// Frame sizing for 48 kHz, Class A (125 us interval)
#define AVTP_SAMPLES_PER_PACKET 6       // 48000 / 8000
#define AVTP_CHANNELS           2       // Stereo
#define AVTP_SYT_INTERVAL       8       // SYT timestamp every 8 data blocks

// Header sizes
#define AVTP_STREAM_HDR_LEN     24      // AVTP stream PDU header
#define AVTP_CIP_HDR_LEN        8       // CIP header (2 quadlets)
#define AVTP_AM824_QUADLET_LEN  4       // One AM824 sample

// Audio payload per packet
#define AVTP_AUDIO_PAYLOAD_LEN  (AVTP_SAMPLES_PER_PACKET * AVTP_CHANNELS * AVTP_AM824_QUADLET_LEN)
// = 6 * 2 * 4 = 48 bytes

// Total stream_data_length (CIP + audio)
#define AVTP_STREAM_DATA_LEN    (AVTP_CIP_HDR_LEN + AVTP_AUDIO_PAYLOAD_LEN)
// = 8 + 48 = 56 bytes

// Total Ethernet frame payload (AVTP header + CIP + audio)
#define AVTP_FRAME_PAYLOAD_LEN  (AVTP_STREAM_HDR_LEN + AVTP_STREAM_DATA_LEN)
// = 24 + 56 = 80 bytes

// Total Ethernet frame length
#define AVTP_FRAME_LEN          (14 + AVTP_FRAME_PAYLOAD_LEN)
// = 14 + 80 = 94 bytes

// Presentation time offset from capture (in nanoseconds)
// Class A: max transit time 2 ms, typical 1-2 ms
#define AVTP_PRESENTATION_OFFSET_NS  2000000  // 2 ms

// Talker TX interval
#define AVTP_TX_INTERVAL_US     125     // Class A: 125 us

// ---------------------------------------------------------------------------
// Audio sample ring buffer
// ---------------------------------------------------------------------------

// Ring buffer holds 24-bit audio samples (stored as int32_t, sign-extended).
// Size must be power of 2 for efficient masking.
#define AUDIO_RING_SIZE         256     // samples per channel (256 = ~5.3 ms at 48 kHz)
#define AUDIO_RING_MASK         (AUDIO_RING_SIZE - 1)

typedef struct {
    int32_t  left[AUDIO_RING_SIZE];
    int32_t  right[AUDIO_RING_SIZE];
    volatile uint32_t write_idx;    // Written by producer
    volatile uint32_t read_idx;     // Written by consumer
} audio_ring_t;

// ---------------------------------------------------------------------------
// AVTP stream state
// ---------------------------------------------------------------------------

typedef struct {
    // Stream identity
    uint8_t  stream_id[8];      // Typically MAC + 16-bit unique ID
    uint8_t  dst_mac[6];        // Destination multicast address
    uint8_t  src_mac[6];        // Our MAC address

    // TX (talker) state
    uint8_t  tx_enabled;
    uint8_t  tx_seq_num;        // 8-bit sequence counter
    uint8_t  tx_dbc;            // Data block counter
    uint32_t tx_last_time_us;   // Last TX time in microseconds (from gPTP)
    uint32_t tx_packet_count;

    // RX (listener) state
    uint8_t  rx_enabled;
    uint8_t  rx_stream_id[8];   // Stream ID to listen to
    uint8_t  rx_seq_num;        // Expected sequence number
    uint32_t rx_packet_count;
    uint32_t rx_seq_errors;
    uint32_t rx_last_avtp_ts;   // Last AVTP presentation timestamp

    // Audio buffers
    audio_ring_t *tx_ring;      // Audio source → AVTP TX (talker)
    audio_ring_t *rx_ring;      // AVTP RX → audio sink (listener)
} avtp_stream_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// Initialize AVTP stream state.
// tx_ring: ring buffer providing audio samples for talker (can be NULL if listener-only)
// rx_ring: ring buffer receiving audio samples from listener (can be NULL if talker-only)
void avtp_init(avtp_stream_t *s, const uint8_t *mac_addr,
               audio_ring_t *tx_ring, audio_ring_t *rx_ring);

// Set the stream ID for talker (derived from MAC + unique_id).
void avtp_set_stream_id(avtp_stream_t *s, uint16_t unique_id);

// Set the stream ID to listen to.
void avtp_set_listen_stream_id(avtp_stream_t *s, const uint8_t *stream_id);

// Enable/disable talker and listener.
void avtp_tx_enable(avtp_stream_t *s, uint8_t enable);
void avtp_rx_enable(avtp_stream_t *s, uint8_t enable);

// Main poll — call from main loop. Handles TX timing and RX processing.
void avtp_poll(avtp_stream_t *s);

// Process a received Ethernet frame (called from main RX dispatch).
void avtp_process_rx(avtp_stream_t *s, const uint8_t *frame, uint32_t len);

// Audio ring buffer helpers.
static inline uint32_t audio_ring_count(const audio_ring_t *r)
{
    return (r->write_idx - r->read_idx) & AUDIO_RING_MASK;
}

static inline uint32_t audio_ring_space(const audio_ring_t *r)
{
    return AUDIO_RING_MASK - audio_ring_count(r);
}

static inline void audio_ring_write(audio_ring_t *r, int32_t left, int32_t right)
{
    uint32_t idx = r->write_idx & AUDIO_RING_MASK;
    r->left[idx]  = left;
    r->right[idx] = right;
    r->write_idx++;
}

static inline void audio_ring_read(audio_ring_t *r, int32_t *left, int32_t *right)
{
    uint32_t idx = r->read_idx & AUDIO_RING_MASK;
    *left  = r->left[idx];
    *right = r->right[idx];
    r->read_idx++;
}

#endif // AVTP_H
