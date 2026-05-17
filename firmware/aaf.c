// AAF RX/TX path — 8 channels, 48 kHz, INT_32BIT, class A (6 samples/packet).
//
// AAF stream header layout (IEEE 1722-2016 figure 16):
//  +0  cd|subtype           = 0x02
//  +1  sv|version|mr|tv|rsvd
//  +2  sequence_num
//  +3  reserved|tu
//  +4..11   stream_id (8)
// +12..15   avtp_timestamp (32-bit, low 32 of gPTP ns at presentation_time)
// +16  format
// +17..18  nsr(4)|rsvd(2)|channels_per_frame(10) — big-endian 16-bit
// +19  bit_depth
// +20..21  stream_data_length (bytes of sample data following the header)
// +22  reserved(2)|sp(1)|evt(4)|rsvd(1)
// +23  reserved
// +24..    sample data (6 × 8 ch × 4 B = 192 B)

#include "aaf.h"
#include "avtp.h"
#include "gptp.h"
#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>
#include <string.h>
#include <stdio.h>

static inline uint16_t be16(const uint8_t *p) {
    return ((uint16_t)p[0] << 8) | p[1];
}
static inline uint32_t be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}
static inline void put_be16(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = v; }
static inline void put_be32(uint8_t *p, uint32_t v) {
    p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v;
}

#define AAF_STREAM_HDR_LEN  24
#define AAF_PACKET_PAYLOAD  (AAF_STREAM_HDR_LEN + \
                             AAF_SAMPLES_PER_PACKET * AAF_CHANNELS * AAF_BYTES_PER_SAMPLE)
#define AAF_FRAME_LEN       (18 + AAF_PACKET_PAYLOAD)   // 14 eth + 4 vlan + AVTP+payload

static uint32_t aaf_txslot;
static uint8_t *aaf_tx_buf_ptr(void) {
    return (uint8_t *)(ETHMAC_BASE + ETHMAC_SLOT_SIZE * (ETHMAC_RX_SLOTS + aaf_txslot));
}
static void aaf_eth_send(uint32_t len) {
    while (!ethmac_sram_reader_ready_read())
        ;
    ethmac_sram_reader_slot_write(aaf_txslot);
    ethmac_sram_reader_length_write(len);
    ethmac_sram_reader_start_write(1);
    aaf_txslot = (aaf_txslot + 1) % ETHMAC_TX_SLOTS;
}

void aaf_init(aaf_state_t *a, const uint8_t *mac_addr,
              const uint8_t *stream_id, const uint8_t *dest_mac)
{
    memset(a, 0, sizeof(*a));
    memcpy(a->src_mac,   mac_addr,  6);
    memcpy(a->dest_mac,  dest_mac,  6);
    memcpy(a->stream_id, stream_id, 8);
    aaf_txslot = 0;
}

void aaf_bind(aaf_state_t *a, const uint8_t *stream_id)
{
    memcpy(a->stream_id, stream_id, 8);
    a->bound          = 1;
    a->rx_enabled     = 1;
    a->rx_count       = 0;
    a->rx_seq_errors  = 0;
    a->format_errors  = 0;
    a->rx_have_last_seq = 0;
    a->rx_write_idx   = 0;
    a->rx_read_idx    = 0;
    printf("[AAF] bound to stream "
           "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
           stream_id[0], stream_id[1], stream_id[2], stream_id[3],
           stream_id[4], stream_id[5], stream_id[6], stream_id[7]);
}

void aaf_unbind(aaf_state_t *a)
{
    a->bound      = 0;
    a->rx_enabled = 0;
    printf("[AAF] unbound\n");
}

void aaf_tx_enable(aaf_state_t *a, uint8_t enable)
{
    if (enable) {
        a->tx_enabled = 1;
        a->tx_seq     = 0;
        a->tx_packet_count = 0;
        a->tx_underrun_count = 0;
        a->tx_write_idx = 0;
        a->tx_read_idx  = 0;
        a->tx_last_sample_count = mcr_sample_count_read();
        printf("[AAF] TX enabled\n");
    } else {
        a->tx_enabled = 0;
        printf("[AAF] TX disabled\n");
    }
}

void aaf_tx_push(aaf_state_t *a, const int32_t *block)
{
    uint32_t slot = a->tx_write_idx & AAF_BUFFER_MASK;
    for (int ch = 0; ch < AAF_CHANNELS; ch++)
        a->tx_buf[ch][slot] = block[ch];
    a->tx_write_idx++;
}

void aaf_process_rx(aaf_state_t *a, const uint8_t *frame, uint32_t len)
{
    if (!a->rx_enabled) return;
    if (len < 14 + AAF_STREAM_HDR_LEN) return;

    const uint8_t *pdu = frame + 14;
    if (pdu[0] != AVTP_SUBTYPE_AAF) return;

    for (int i = 0; i < 8; i++) {
        if (pdu[4 + i] != a->stream_id[i]) {
            a->rx_other_count++;
            return;
        }
    }

    uint8_t seq = pdu[2];
    if (a->rx_have_last_seq && seq != (uint8_t)(a->rx_last_seq + 1))
        a->rx_seq_errors++;
    a->rx_last_seq      = seq;
    a->rx_have_last_seq = 1;

    a->last_presentation_ts = be32(pdu + 12);

    uint8_t  format    = pdu[16];
    uint16_t nsr_ch    = be16(pdu + 17);
    uint8_t  nsr       = (nsr_ch >> 12) & 0xF;
    uint16_t channels  = nsr_ch & 0x3FF;
    uint8_t  bit_depth = pdu[19];
    uint16_t stream_data_len = be16(pdu + 20);

    if (format    != AAF_FORMAT_INT_32BIT ||
        nsr       != AAF_NSR_48000        ||
        channels  != AAF_CHANNELS         ||
        bit_depth != AAF_BIT_DEPTH) {
        a->format_errors++;
        return;
    }

    const uint8_t *audio = pdu + AAF_STREAM_HDR_LEN;
    uint32_t avail = (len > 14 + AAF_STREAM_HDR_LEN)
                       ? (len - 14 - AAF_STREAM_HDR_LEN) : 0;
    if (stream_data_len > avail) stream_data_len = avail;

    uint32_t bytes_per_block = AAF_CHANNELS * AAF_BYTES_PER_SAMPLE;
    uint32_t blocks          = stream_data_len / bytes_per_block;

    uint32_t wr = a->rx_write_idx;
    for (uint32_t b = 0; b < blocks; b++) {
        uint32_t slot = wr & AAF_BUFFER_MASK;
        for (int ch = 0; ch < AAF_CHANNELS; ch++) {
            int32_t s = (int32_t)be32(audio);
            a->rx_buf[ch][slot] = s;
            audio += 4;
        }
        wr++;
    }
    a->rx_write_idx = wr;
    a->rx_count++;
}

int aaf_rx_pop(aaf_state_t *a, int32_t *out)
{
    if (a->rx_write_idx == a->rx_read_idx) return 0;
    uint32_t slot = a->rx_read_idx & AAF_BUFFER_MASK;
    for (int ch = 0; ch < AAF_CHANNELS; ch++)
        out[ch] = a->rx_buf[ch][slot];
    a->rx_read_idx++;
    return 1;
}

// Build and send one AAF AVTPDU of 6 samples × 8 channels.
static void aaf_send_packet(aaf_state_t *a)
{
    uint8_t *frame = aaf_tx_buf_ptr();

    // Ethernet header with 802.1Q VLAN tag for AVB Class A streams.
    // TPID 0x8100, PCP=3 (Class A priority), DEI=0, VID=2 (default AVB VID).
    // Bridges enforce this tag — untagged stream frames get dropped or
    // downshifted to best-effort, which underruns downstream listeners.
    memcpy(frame,     a->dest_mac, 6);
    memcpy(frame + 6, a->src_mac,  6);
    put_be16(frame + 12, 0x8100);              // 802.1Q TPID
    put_be16(frame + 14, (3 << 13) | 2);       // PCP=3 (Class A), DEI=0, VID=2
    put_be16(frame + 16, AVTP_ETHERTYPE);      // 0x22F0

    uint8_t *pdu = frame + 18;
    pdu[0] = AVTP_SUBTYPE_AAF;        // cd=0, subtype=0x02
    pdu[1] = 0x81;                     // sv=1, mr=0, tv=1 (timestamp valid)
    pdu[2] = a->tx_seq;
    pdu[3] = 0x00;                     // reserved | tu=0
    memcpy(pdu + 4, a->stream_id, 8);

    // Presentation time: gPTP now + offset, low 32 of ns.
    ptp_timestamp_t now = gptp_read_time();
    uint64_t now_ns = (uint64_t)now.seconds * 1000000000ull + now.nanoseconds;
    uint32_t pres_ts = (uint32_t)(now_ns + AAF_PRESENTATION_OFFSET_NS);
    put_be32(pdu + 12, pres_ts);

    pdu[16] = AAF_FORMAT_INT_32BIT;
    // nsr(4)=5 | rsvd(2)=0 | channels(10)=8 → high nibble 0x5, then 0x0008
    put_be16(pdu + 17, ((uint16_t)AAF_NSR_48000 << 12) | (AAF_CHANNELS & 0x3FF));
    pdu[19] = AAF_BIT_DEPTH;
    put_be16(pdu + 20, AAF_SAMPLES_PER_PACKET * AAF_CHANNELS * AAF_BYTES_PER_SAMPLE);
    pdu[22] = 0x00;                    // sp=0 (normal), evt=0
    pdu[23] = 0x00;

    // Pack 6 blocks of 8 channels
    uint8_t *aud = pdu + AAF_STREAM_HDR_LEN;
    for (int s = 0; s < AAF_SAMPLES_PER_PACKET; s++) {
        int32_t block[AAF_CHANNELS];
        if (a->tx_read_idx == a->tx_write_idx) {
            // Underrun — emit silence to keep packet rate stable
            for (int ch = 0; ch < AAF_CHANNELS; ch++) block[ch] = 0;
            a->tx_underrun_count++;
        } else {
            uint32_t slot = a->tx_read_idx & AAF_BUFFER_MASK;
            for (int ch = 0; ch < AAF_CHANNELS; ch++)
                block[ch] = a->tx_buf[ch][slot];
            a->tx_read_idx++;
        }
        for (int ch = 0; ch < AAF_CHANNELS; ch++) {
            put_be32(aud, (uint32_t)block[ch]);
            aud += 4;
        }
    }

    aaf_eth_send(AAF_FRAME_LEN);
    a->tx_seq++;
    a->tx_packet_count++;
}

void aaf_tx_poll(aaf_state_t *a)
{
    if (!a->tx_enabled) return;

    // Pace packets to one per AAF_SAMPLES_PER_PACKET MCR ticks (8000 Hz).
    uint32_t now_cnt = mcr_sample_count_read();
    uint32_t elapsed = now_cnt - a->tx_last_sample_count;
    if (elapsed < AAF_SAMPLES_PER_PACKET) return;
    a->tx_last_sample_count += AAF_SAMPLES_PER_PACKET;

    aaf_send_packet(a);
}
