// IEEE 1722 AVTP — IEC 61883-6 AM824 Talker/Listener
// Firmware implementation for LiteX SoC with LiteEth MAC

#include "avtp.h"
#include "gptp.h"

#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>
#include <string.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Byte-order helpers (big-endian wire format)
// ---------------------------------------------------------------------------

static inline void put_be16(uint8_t *p, uint16_t v)
{
    p[0] = (v >> 8) & 0xFF;
    p[1] = v & 0xFF;
}

static inline void put_be32(uint8_t *p, uint32_t v)
{
    p[0] = (v >> 24) & 0xFF;
    p[1] = (v >> 16) & 0xFF;
    p[2] = (v >>  8) & 0xFF;
    p[3] = v & 0xFF;
}

static inline uint16_t get_be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static inline uint32_t get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] <<  8) | p[3];
}

static inline uint64_t get_be64(const uint8_t *p)
{
    return ((uint64_t)get_be32(p) << 32) | get_be32(p + 4);
}

static inline void put_be64(uint8_t *p, uint64_t v)
{
    put_be32(p, (uint32_t)(v >> 32));
    put_be32(p + 4, (uint32_t)v);
}

// ---------------------------------------------------------------------------
// Low-level MAC TX (shared with gPTP — uses same SRAM slots)
// ---------------------------------------------------------------------------

// TX slot counter — shared with gPTP module via extern.
// In a real system we'd have a proper TX slot allocator.
// For now, gPTP and AVTP each track their own slot usage.
static uint32_t avtp_txslot;

static uint8_t *avtp_tx_buf(void)
{
    return (uint8_t *)(ETHMAC_BASE + ETHMAC_SLOT_SIZE * (ETHMAC_RX_SLOTS + avtp_txslot));
}

static void avtp_eth_send(uint32_t len)
{
    while (!ethmac_sram_reader_ready_read())
        ;
    ethmac_sram_reader_slot_write(avtp_txslot);
    ethmac_sram_reader_length_write(len);
    ethmac_sram_reader_start_write(1);
    avtp_txslot = (avtp_txslot + 1) % ETHMAC_TX_SLOTS;
}

// ---------------------------------------------------------------------------
// AVTP Stream PDU construction
// ---------------------------------------------------------------------------

// Build Ethernet + AVTP + CIP header template into frame buffer.
// Returns pointer past the CIP header (start of AM824 payload area).
static uint8_t *build_avtp_header(uint8_t *frame, avtp_stream_t *s)
{
    uint8_t *p = frame;

    // --- Ethernet header (18 bytes: 14 std + 4 802.1Q VLAN tag) ---
    // AVB bridges require Class A stream frames to carry an 802.1Q tag
    // (TPID 0x8100, PCP=3, VID=2) — untagged stream frames get dropped or
    // downshifted to best-effort.
    memcpy(p, s->dst_mac, 6);                  // Destination MAC
    memcpy(p + 6, s->src_mac, 6);              // Source MAC
    put_be16(p + 12, 0x8100);                  // 802.1Q TPID
    put_be16(p + 14, (3 << 13) | 2);           // PCP=3, DEI=0, VID=2
    put_be16(p + 16, AVTP_ETHERTYPE);          // EtherType = 0x22F0
    p += 18;

    // --- AVTP Stream PDU header (24 bytes) ---
    // Byte 0: subtype = 0x00 (IEC 61883/IIDC)
    p[0] = AVTP_SUBTYPE_61883_IIDC;

    // Byte 1: sv=1 | version=0 | mr=0 | _r=0 | gv=0 | tv=1
    //         sv(1) version(3) mr(1) _r(1) gv(0) tv(1)
    //         1     000        0     0     0     1     = 0x81
    p[1] = 0x81;

    // Byte 2: sequence_num (updated per packet)
    p[2] = s->tx_seq_num;

    // Byte 3: reserved(7) | tu(1) = 0
    p[3] = 0x00;

    // Bytes 4-11: stream_id
    memcpy(p + 4, s->stream_id, 8);

    // Bytes 12-15: avtp_timestamp (filled per packet)
    put_be32(p + 12, 0);

    // Bytes 16-19: gateway_info = 0
    put_be32(p + 16, 0);

    // Bytes 20-21: stream_data_length
    put_be16(p + 20, AVTP_STREAM_DATA_LEN);

    // Byte 22: tag=1 (CIP) | channel=0x1F (any)
    //          tag(2)=01 | channel(6)=011111 = 0x5F
    p[22] = 0x5F;

    // Byte 23: tcode=0xA | sy=0x0
    //          tcode(4)=1010 | sy(4)=0000 = 0xA0
    p[23] = 0xA0;

    p += 24;

    // --- CIP Header (8 bytes) ---
    // CIP1: QI1(2)=00 | SID(6)=0x3F | DBS(8)=channels | FN(2)=00 | QPC(3)=000 | SPH(1)=0 | rsv(2)=00 | DBC(8)
    uint32_t cip1 = 0;
    cip1 |= (0x3F << 24);              // SID = 0x3F (any)
    cip1 |= (AVTP_CHANNELS << 16);     // DBS = 2 (stereo)
    cip1 |= (s->tx_dbc & 0xFF);        // DBC (updated per packet)
    put_be32(p, cip1);

    // CIP2: QI2(2)=10 | FMT(6)=0x10 | FDF(8)=SFC | SYT(16)=0xFFFF
    uint32_t cip2 = 0;
    cip2 |= (0x02 << 30);              // QI2 = 10b
    cip2 |= (CIP_FMT_61883_6 << 24);  // FMT = 0x10
    cip2 |= (CIP_SFC_48K << 16);       // SFC = 2 (48 kHz)
    cip2 |= 0xFFFF;                    // SYT = 0xFFFF (no SYT)
    put_be32(p + 4, cip2);

    p += 8;

    return p; // Points to start of AM824 audio data area
}

// ---------------------------------------------------------------------------
// AVTP TX — Talker
// ---------------------------------------------------------------------------

static void avtp_send_packet(avtp_stream_t *s)
{
    uint8_t *frame = avtp_tx_buf();

    // Build header
    uint8_t *audio_data = build_avtp_header(frame, s);

    // Get current gPTP time for AVTP timestamp
    ptp_timestamp_t now = gptp_read_time();
    uint32_t avtp_ts = now.nanoseconds + AVTP_PRESENTATION_OFFSET_NS;
    // Handle nanosecond wrap
    if (avtp_ts >= 1000000000) {
        avtp_ts -= 1000000000;
    }

    // Write AVTP timestamp into header (bytes 12-15 of AVTP PDU = offset 26 from frame start)
    put_be32(frame + 18 + 12, avtp_ts);

    // Update sequence number in header
    frame[18 + 2] = s->tx_seq_num;

    // Update DBC in CIP1 (offset: 14 eth + 24 avtp + 0 cip1, byte 3 of cip1)
    frame[18 + 24 + 3] = s->tx_dbc;

    // Pack AM824 audio samples from ring buffer
    uint32_t available = audio_ring_count(s->tx_ring);

    for (int i = 0; i < AVTP_SAMPLES_PER_PACKET; i++) {
        int32_t left, right;

        if (available > 0) {
            audio_ring_read(s->tx_ring, &left, &right);
            available--;
        } else {
            // Underrun — send silence
            left = 0;
            right = 0;
        }

        // AM824 quadlet: label(8) | audio_sample(24), big-endian
        // Left channel
        audio_data[0] = AM824_LABEL_MBLA;
        audio_data[1] = (left >> 16) & 0xFF;
        audio_data[2] = (left >>  8) & 0xFF;
        audio_data[3] = left & 0xFF;
        audio_data += 4;

        // Right channel
        audio_data[0] = AM824_LABEL_MBLA;
        audio_data[1] = (right >> 16) & 0xFF;
        audio_data[2] = (right >>  8) & 0xFF;
        audio_data[3] = right & 0xFF;
        audio_data += 4;
    }

    // Send frame
    avtp_eth_send(AVTP_FRAME_LEN);

    // Advance counters
    s->tx_seq_num++;
    s->tx_dbc += AVTP_SAMPLES_PER_PACKET;
    s->tx_packet_count++;
}

// ---------------------------------------------------------------------------
// AVTP RX — Listener
// ---------------------------------------------------------------------------

void avtp_process_rx(avtp_stream_t *s, const uint8_t *frame, uint32_t len)
{
    if (!s->rx_enabled)
        return;

    if (len < AVTP_FRAME_LEN)
        return;

    // Check ethertype
    if (get_be16(frame + 12) != AVTP_ETHERTYPE)
        return;

    const uint8_t *pdu = frame + 14;  // Past Ethernet header

    // Check subtype
    if ((pdu[0] & 0xFF) != AVTP_SUBTYPE_61883_IIDC)
        return;

    // Check stream_id matches what we're listening to
    const uint8_t *rx_sid = pdu + 4;
    int match = 1;
    for (int i = 0; i < 8; i++) {
        if (rx_sid[i] != s->rx_stream_id[i]) { match = 0; break; }
    }
    if (!match)
        return;

    // Extract fields
    uint8_t seq_num = pdu[2];
    uint32_t avtp_timestamp = get_be32(pdu + 12);
    uint16_t stream_data_len = get_be16(pdu + 20);

    // Sequence number check
    if (s->rx_packet_count > 0) {
        uint8_t expected = (uint8_t)(s->rx_seq_num + 1);
        if (seq_num != expected) {
            s->rx_seq_errors++;
        }
    }
    s->rx_seq_num = seq_num;
    s->rx_last_avtp_ts = avtp_timestamp;

    // CIP header at pdu + 24
    const uint8_t *cip = pdu + AVTP_STREAM_HDR_LEN;
    uint8_t dbs = cip[1];      // Data block size (channels per block)

    // Audio data starts after CIP header
    const uint8_t *audio = cip + AVTP_CIP_HDR_LEN;
    uint32_t audio_len = stream_data_len - AVTP_CIP_HDR_LEN;

    if (dbs == 0)
        dbs = AVTP_CHANNELS;

    // Calculate number of data blocks in this packet
    uint32_t num_blocks = audio_len / (dbs * AVTP_AM824_QUADLET_LEN);

    // Unpack AM824 samples into ring buffer
    if (s->rx_ring) {
        for (uint32_t i = 0; i < num_blocks; i++) {
            int32_t left = 0, right = 0;

            // First quadlet = left (or mono if dbs=1)
            if (audio[0] == AM824_LABEL_MBLA) {
                left = ((int32_t)audio[1] << 16) |
                       ((int32_t)audio[2] << 8) |
                        (int32_t)audio[3];
                // Sign extend from 24-bit
                if (left & 0x800000)
                    left |= (int32_t)0xFF000000;
            }
            audio += 4;

            // Second quadlet = right (if stereo)
            if (dbs >= 2) {
                if (audio[0] == AM824_LABEL_MBLA) {
                    right = ((int32_t)audio[1] << 16) |
                            ((int32_t)audio[2] << 8) |
                             (int32_t)audio[3];
                    if (right & 0x800000)
                        right |= (int32_t)0xFF000000;
                }
                audio += 4;
                // Skip any remaining channels beyond stereo
                audio += (dbs - 2) * 4;
            }

            if (audio_ring_space(s->rx_ring) > 0) {
                audio_ring_write(s->rx_ring, left, right);
            }
            // else: overrun — drop sample
        }
    }

    s->rx_packet_count++;
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void avtp_init(avtp_stream_t *s, const uint8_t *mac_addr,
               audio_ring_t *tx_ring, audio_ring_t *rx_ring)
{
    memset(s, 0, sizeof(*s));

    memcpy(s->src_mac, mac_addr, 6);

    // Default AVTP multicast destination
    static const uint8_t avtp_mcast[] = AVTP_MCAST_ADDR;
    memcpy(s->dst_mac, avtp_mcast, 6);

    // Default stream_id: MAC address + 0x0001
    memcpy(s->stream_id, mac_addr, 6);
    s->stream_id[6] = 0x00;
    s->stream_id[7] = 0x01;

    s->tx_ring = tx_ring;
    s->rx_ring = rx_ring;

    avtp_txslot = 0;

    printf("[AVTP] Initialized. Stream ID=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
           s->stream_id[0], s->stream_id[1], s->stream_id[2], s->stream_id[3],
           s->stream_id[4], s->stream_id[5], s->stream_id[6], s->stream_id[7]);
}

void avtp_set_stream_id(avtp_stream_t *s, uint16_t unique_id)
{
    memcpy(s->stream_id, s->src_mac, 6);
    s->stream_id[6] = (unique_id >> 8) & 0xFF;
    s->stream_id[7] = unique_id & 0xFF;
}

void avtp_set_listen_stream_id(avtp_stream_t *s, const uint8_t *stream_id)
{
    memcpy(s->rx_stream_id, stream_id, 8);
}

void avtp_tx_enable(avtp_stream_t *s, uint8_t enable)
{
    s->tx_enabled = enable;
    if (enable) {
        s->tx_seq_num = 0;
        s->tx_dbc = 0;
        s->tx_packet_count = 0;
        printf("[AVTP] Talker enabled\n");
    } else {
        printf("[AVTP] Talker disabled\n");
    }
}

void avtp_rx_enable(avtp_stream_t *s, uint8_t enable)
{
    s->rx_enabled = enable;
    if (enable) {
        s->rx_packet_count = 0;
        s->rx_seq_errors = 0;
        printf("[AVTP] Listener enabled\n");
    } else {
        printf("[AVTP] Listener disabled\n");
    }
}

// ---------------------------------------------------------------------------
// Poll — call from main loop
// ---------------------------------------------------------------------------

void avtp_poll(avtp_stream_t *s)
{
    if (!s->tx_enabled)
        return;

    if (!s->tx_ring)
        return;

    // Check if it's time to send the next AVTP packet.
    // Class A: every 125 us.
    ptp_timestamp_t now = gptp_read_time();
    uint32_t now_us = (uint32_t)(now.nanoseconds / 1000);
    // Also incorporate seconds to handle nanosecond wrap
    now_us += (uint32_t)(now.seconds & 0xFFF) * 1000000;

    uint32_t elapsed = now_us - s->tx_last_time_us;
    // Handle wrap (we only track ~4095 seconds worth)
    if (elapsed > 2000000000)
        elapsed = AVTP_TX_INTERVAL_US; // Force send on wrap

    if (elapsed >= AVTP_TX_INTERVAL_US) {
        // Check we have enough samples
        if (audio_ring_count(s->tx_ring) >= AVTP_SAMPLES_PER_PACKET) {
            avtp_send_packet(s);
        }
        s->tx_last_time_us = now_us;

        // Periodic status
        if (s->tx_packet_count > 0 && (s->tx_packet_count % 8000) == 0) {
            printf("[AVTP TX] pkts=%lu ring=%lu\n",
                   (unsigned long)s->tx_packet_count,
                   (unsigned long)audio_ring_count(s->tx_ring));
        }
    }
}
