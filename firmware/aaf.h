// AAF (AVTP Audio Format) — 8-channel, 48 kHz, 32-bit PCM, 6 samples/frame.
//
// One AAF AVTPDU per 125 µs (class A) carries 6 samples × 8 channels × 4 B
// = 192 bytes of audio. RX path: parse header, validate stream_id and
// format, push samples into per-channel ring buffers indexed by sequence
// number. The MCR sample strobe pops one sample/channel per fs tick.

#ifndef AAF_H
#define AAF_H

#include <stdint.h>

// AAF stream parameters (must match the avdecc.h stream_fmt_aaf_8ch_48k word).
#define AAF_CHANNELS            8
#define AAF_BIT_DEPTH           32
#define AAF_BYTES_PER_SAMPLE    4
#define AAF_NSR_48000           5
#define AAF_FORMAT_INT_32BIT    2

// Per-channel jitter buffer. 64 samples = 1.33 ms at 48 kHz — enough
// margin for class A AVB (<125 µs talker-to-listener jitter target).
// Larger than 64 doesn't fit in the 8 KB SRAM when paired with TX (D2).
// Total RX buffer = 8 ch × 64 samples × 4 B = 2 KB.
#define AAF_BUFFER_SIZE         64
#define AAF_BUFFER_MASK         (AAF_BUFFER_SIZE - 1)

// AAF class A: 8000 packets/sec, 6 samples per packet at 48 kHz.
#define AAF_SAMPLES_PER_PACKET  6
#define AAF_PACKET_RATE_HZ      8000
// Max talker→listener latency, used as presentation_time offset (ns).
// Class A spec is 2 ms; we use 2 ms to match.
#define AAF_PRESENTATION_OFFSET_NS  2000000

typedef struct {
    uint8_t  bound;
    uint8_t  stream_id[8];

    // ---- RX path ----
    uint8_t  rx_enabled;
    // When 0, aaf_process_rx counts the frame + tracks header/seq but
    // SKIPS the per-channel audio copy into rx_buf. With 8000 fps AAF
    // flooding the dispatcher, the 8ch × 6-sample × 4-byte copy + the
    // downstream consumer path was starving gPTP RX (writer_errors
    // 1.6M+, gPTP locked=0). Enable only when a real consumer (AES3
    // TX once locked, or a real-time pipe) is actually pulling data.
    uint8_t  rx_audio_capture;
    uint32_t rx_count;
    uint32_t rx_other_count;
    uint32_t format_errors;
    uint32_t rx_seq_errors;
    uint8_t  rx_last_seq;
    uint8_t  rx_have_last_seq;
    int32_t  rx_buf[AAF_CHANNELS][AAF_BUFFER_SIZE];
    volatile uint32_t rx_write_idx;
    volatile uint32_t rx_read_idx;
    uint32_t last_presentation_ts;

    // ---- TX path ----
    uint8_t  tx_enabled;
    uint8_t  tx_seq;
    uint8_t  src_mac[6];
    uint8_t  dest_mac[6];
    // Per-channel TX buffer (firmware producer writes; aaf_tx_poll consumes).
    int32_t  tx_buf[AAF_CHANNELS][AAF_BUFFER_SIZE];
    volatile uint32_t tx_write_idx;
    volatile uint32_t tx_read_idx;
    uint32_t tx_packet_count;
    uint32_t tx_underrun_count;
    // Trigger anchor — MCR sample_count value at the last TX. We send the
    // next packet when (mcr_sample_count - tx_last_sample) ≥ samples/pkt.
    uint32_t tx_last_sample_count;

    // VLAN-PCP and VID used in the 802.1Q tag of outgoing AAF frames.
    // Defaults to Class A (PCP=3, VID=2); updated to follow the bridge's
    // MSRP Domain advertisement via aaf_set_vlan() so we match the
    // priority Auvitran's port expects — otherwise listeners reply with
    // MSRP failure 0x13 (SR Class Priority Mismatch).
    uint8_t  tx_pcp;
    uint16_t tx_vid;
} aaf_state_t;

void aaf_init       (aaf_state_t *a, const uint8_t *mac_addr,
                     const uint8_t *stream_id, const uint8_t *dest_mac);
void aaf_bind       (aaf_state_t *a, const uint8_t *stream_id);
void aaf_unbind     (aaf_state_t *a);
void aaf_process_rx (aaf_state_t *a, const uint8_t *frame, uint32_t len);
void aaf_tx_enable  (aaf_state_t *a, uint8_t enable);
// Update VLAN PCP/VID used in outgoing AAF frames. Call when MSRP Domain
// RX gives us a non-default mapping (per [[msrp-class-priority-mismatch]]).
void aaf_set_vlan   (aaf_state_t *a, uint8_t pcp, uint16_t vid);
// Producer-side push of one sample block (one sample per channel).
// Called by the audio source — for end-to-end test this can be the
// RX-side pop wired through (loopback) or a tone generator.
void aaf_tx_push    (aaf_state_t *a, const int32_t *block);
// Pop one received block into `out[AAF_CHANNELS]` (1=success, 0=empty).
int  aaf_rx_pop     (aaf_state_t *a, int32_t *out);
// Periodic TX driver — call from main loop. Builds and sends packets when
// enough samples have accumulated.
void aaf_tx_poll    (aaf_state_t *a);

static inline uint32_t aaf_rx_level(const aaf_state_t *a) {
    return (a->rx_write_idx - a->rx_read_idx) & AAF_BUFFER_MASK;
}
static inline uint32_t aaf_tx_level(const aaf_state_t *a) {
    return (a->tx_write_idx - a->tx_read_idx) & AAF_BUFFER_MASK;
}

#endif
