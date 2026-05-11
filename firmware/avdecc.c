// IEEE 1722.1 AVDECC — minimal endpoint
//
// ADP: Periodic entity advertisement so controllers can discover us.
// ACMP: Handle connect/disconnect commands for talker and listener streams.
//
// PDU format (IEEE 1722.1-2013):
//   Ethernet header (14 bytes)
//   AVTPDU common header:
//     subtype(8) | sv(1) version(3) msg_type(4) | valid_time(5) control_data_len(11)
//     entity_id(64)
//   Protocol-specific fields follow.

#include "avdecc.h"
#include "gptp.h"

#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>
#include <string.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Byte-order helpers
// ---------------------------------------------------------------------------

static inline void av_put_be16(uint8_t *p, uint16_t v)
{
    p[0] = (v >> 8); p[1] = v;
}

static inline void av_put_be32(uint8_t *p, uint32_t v)
{
    p[0] = (v >> 24); p[1] = (v >> 16); p[2] = (v >> 8); p[3] = v;
}

static inline void av_put_be64(uint8_t *p, uint64_t v)
{
    av_put_be32(p, (uint32_t)(v >> 32));
    av_put_be32(p + 4, (uint32_t)v);
}

static inline uint16_t av_get_be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static inline uint32_t av_get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  | p[3];
}

// ---------------------------------------------------------------------------
// TX
// ---------------------------------------------------------------------------

// Per-stream identity
#define N_STREAM_INPUTS   2     // [0]=CRF Media Clock, [1]=AAF Audio Input
#define N_STREAM_OUTPUTS  1     // [0]=AAF Audio Output
#define N_CLOCK_SOURCES   2     // [0]=Internal osc, [1]=CRF stream
#define N_AUDIO_CHANNELS  8     // 8ch AAF I/O
#define LISTENER_CRF_INDEX 0
#define LISTENER_AAF_INDEX 1
#define TALKER_AAF_INDEX   0
#define CLK_SRC_INTERNAL   0
#define CLK_SRC_MEDIA      1    // CRF-driven media clock

// Stream-format constants (decoded from session_mgr.aemt + IEEE 1722.1-2013
// Annex A; see avdecc.h for byte-by-byte derivation).
//
// Listener[0] = CRF audio-sample 48 kHz (Media Clock Input).
// Listener[1] = AAF PCM 8ch / 48 kHz / 32-bit (Audio Input).
// Talker[0]   = AAF PCM 8ch / 48 kHz / 32-bit (Audio Output).
static const uint8_t stream_fmt_crf_48k[8]     = STREAM_FMT_CRF_48K;
static const uint8_t stream_fmt_aaf_8ch_48k[8] = STREAM_FMT_AAF_8CH_48K;

static uint32_t avdecc_txslot;

static uint8_t *avdecc_tx_buf(void)
{
    return (uint8_t *)(ETHMAC_BASE + ETHMAC_SLOT_SIZE * (ETHMAC_RX_SLOTS + avdecc_txslot));
}

static void avdecc_eth_send(uint32_t len)
{
    while (!ethmac_sram_reader_ready_read())
        ;
    ethmac_sram_reader_slot_write(avdecc_txslot);
    ethmac_sram_reader_length_write(len);
    ethmac_sram_reader_start_write(1);
    avdecc_txslot = (avdecc_txslot + 1) % ETHMAC_TX_SLOTS;
}

// Write Ethernet header, returns pointer past it.
static uint8_t *avdecc_eth_hdr(uint8_t *frame, const uint8_t *src_mac)
{
    static const uint8_t mcast[] = AVDECC_MCAST_ADDR;
    memcpy(frame, mcast, 6);
    memcpy(frame + 6, src_mac, 6);
    av_put_be16(frame + 12, AVDECC_ETHERTYPE);
    return frame + 14;
}

// ---------------------------------------------------------------------------
// ADP — AVDECC Discovery Protocol
// ---------------------------------------------------------------------------

// ADPDU layout (after Ethernet header):
//   [0]      subtype = 0x7A
//   [1]      sv(1)=0 | version(3)=0 | message_type(4)
//   [2..3]   valid_time(5) | control_data_length(11) = 56
//   [4..11]  entity_id (8 bytes)
//   [12..19] entity_model_id (8 bytes)
//   [20..23] entity_capabilities (4 bytes)
//   [24..25] talker_stream_sources (2 bytes)
//   [26..27] talker_capabilities (2 bytes)
//   [28..29] listener_stream_sinks (2 bytes)
//   [30..31] listener_capabilities (2 bytes)
//   [32..35] controller_capabilities (4 bytes)
//   [36..39] available_index (4 bytes)
//   [40..47] gptp_grandmaster_id (8 bytes)
//   [48]     gptp_domain_number
//   [49..51] reserved (3 bytes)
//   [52..53] identify_control_index (2 bytes)
//   [54..55] interface_index (2 bytes)
//   [56..63] association_id (8 bytes)
//   [64..67] reserved (4 bytes) — required: GenAVB and Hive treat the
//            ADPDU body as 64 bytes (struct adp_pdu) so the AVTP frame
//            after the 4-byte common header is 68 bytes. Without these
//            4 trailing bytes, Hive logs "Adpdu::deserialize error:
//            Not enough data in buffer".
// Total ADPDU = 68 bytes, control_data_length = 56 (per IEEE 1722.1)

#define ADPDU_LEN           68
#define ADP_CONTROL_DATA_LEN 56

static void adp_send(avdecc_state_t *s, uint8_t msg_type)
{
    uint8_t *frame = avdecc_tx_buf();
    uint8_t *p = avdecc_eth_hdr(frame, s->src_mac);

    // Subtype
    p[0] = AVTP_SUBTYPE_ADP;

    // sv=0 | version=0 | message_type
    p[1] = msg_type & 0x0F;

    // valid_time(5) | control_data_length(11) = 56
    uint16_t vt_cdl = ((uint16_t)ADP_VALID_TIME << 11) | ADP_CONTROL_DATA_LEN;
    av_put_be16(p + 2, vt_cdl);

    // entity_id
    memcpy(p + 4, s->entity_id, 8);

    // entity_model_id (vendor-specific, we use MAC + 0x0000)
    memcpy(p + 12, s->entity_id, 8);

    // entity_capabilities
    uint32_t caps = ADP_CAP_ADP_SUPPORTED | ADP_CAP_ACMP_SUPPORTED |
                    ADP_CAP_GPTP_SUPPORTED;
    av_put_be32(p + 20, caps);

    // talker_stream_sources = N_STREAM_OUTPUTS (1 AAF talker)
    av_put_be16(p + 24, N_STREAM_OUTPUTS);

    // talker_capabilities
    uint16_t talker_caps = ADP_TALKER_CAP_IMPLEMENTED | ADP_TALKER_CAP_AUDIO_SOURCE;
    av_put_be16(p + 26, talker_caps);

    // listener_stream_sinks = N_STREAM_INPUTS (1 CRF + 1 AAF)
    av_put_be16(p + 28, N_STREAM_INPUTS);

    // listener_capabilities
    uint16_t listener_caps = ADP_LISTENER_CAP_IMPLEMENTED | ADP_LISTENER_CAP_AUDIO_SINK;
    av_put_be16(p + 30, listener_caps);

    // controller_capabilities = 0
    av_put_be32(p + 32, 0);

    // available_index
    av_put_be32(p + 36, s->adp_available_index);

    // gptp_grandmaster_id (all zeros = unknown)
    memset(p + 40, 0, 8);

    // gptp_domain_number = 0
    p[48] = 0;

    // reserved
    p[49] = 0; p[50] = 0; p[51] = 0;

    // identify_control_index, interface_index
    av_put_be16(p + 52, 0);
    av_put_be16(p + 54, 0);

    // association_id
    memset(p + 56, 0, 8);

    // reserved trailing 4 bytes (rsvd2 in GenAVB struct adp_pdu)
    memset(p + 64, 0, 4);

    // Total frame
    uint32_t frame_len = 14 + ADPDU_LEN;
    if (frame_len < 64)
        frame_len = 64;  // Minimum Ethernet frame

    avdecc_eth_send(frame_len);

    if (msg_type == ADP_MSG_ENTITY_AVAILABLE)
        s->adp_available_index++;

    s->adp_tx_count++;
}

// ---------------------------------------------------------------------------
// ACMP — AVDECC Connection Management Protocol
// ---------------------------------------------------------------------------

// ACMPDU layout (after Ethernet header):
//   [0]      subtype = 0x7C
//   [1]      sv(1)=0 | version(3)=0 | message_type(4)
//   [2..3]   status(5) | control_data_length(11) = 44
//   [4..11]  stream_id (8 bytes)
//   [12..19] controller_entity_id (8 bytes)
//   [20..27] talker_entity_id (8 bytes)
//   [28..35] listener_entity_id (8 bytes)
//   [36..37] talker_unique_id (2 bytes)
//   [38..39] listener_unique_id (2 bytes)
//   [40..45] stream_dest_mac (6 bytes)
//   [46..47] connection_count (2 bytes)
//   [48..49] sequence_id (2 bytes)
//   [50..51] flags (2 bytes)
//   [52..53] stream_vlan_id (2 bytes)
//   [54..55] reserved (2 bytes)
// Total ACMPDU = 56 bytes, control_data_length = 44

#define ACMPDU_LEN              56
#define ACMP_CONTROL_DATA_LEN   44

// Offsets within ACMPDU (from start of PDU, past Ethernet header)
#define ACMP_OFF_STREAM_ID          4
#define ACMP_OFF_CONTROLLER_ID      12
#define ACMP_OFF_TALKER_ID          20
#define ACMP_OFF_LISTENER_ID        28
#define ACMP_OFF_TALKER_UID         36
#define ACMP_OFF_LISTENER_UID       38
#define ACMP_OFF_STREAM_DEST_MAC    40
#define ACMP_OFF_CONN_COUNT         46
#define ACMP_OFF_SEQ_ID             48
#define ACMP_OFF_FLAGS              50
#define ACMP_OFF_VLAN_ID            52

static int entity_id_match(const uint8_t *a, const uint8_t *b)
{
    for (int i = 0; i < 8; i++) {
        if (a[i] != b[i]) return 0;
    }
    return 1;
}

static void acmp_send_response(avdecc_state_t *s, uint8_t msg_type, uint8_t status,
                                const uint8_t *rx_pdu)
{
    uint8_t *frame = avdecc_tx_buf();
    uint8_t *p = avdecc_eth_hdr(frame, s->src_mac);

    // Copy the received ACMPDU as base and modify
    memcpy(p, rx_pdu, ACMPDU_LEN);

    // Subtype
    p[0] = AVTP_SUBTYPE_ACMP;

    // message_type = response type
    p[1] = msg_type & 0x0F;

    // status | control_data_length
    uint16_t st_cdl = ((uint16_t)(status & 0x1F) << 11) | ACMP_CONTROL_DATA_LEN;
    av_put_be16(p + 2, st_cdl);

    // Fill in our stream info for talker responses (by talker_unique_id)
    if (msg_type == ACMP_MSG_CONNECT_TX_RESPONSE ||
        msg_type == ACMP_MSG_GET_TX_STATE_RESPONSE ||
        msg_type == ACMP_MSG_DISCONNECT_TX_RESPONSE) {
        uint16_t tuid = av_get_be16(p + ACMP_OFF_TALKER_UID);
        if (tuid < AVDECC_MAX_TALKERS) {
            avdecc_talker_stream_t *t = &s->talkers[tuid];
            memcpy(p + ACMP_OFF_STREAM_ID, t->stream_id, 8);
            memcpy(p + ACMP_OFF_STREAM_DEST_MAC, t->dest_mac, 6);
            av_put_be16(p + ACMP_OFF_CONN_COUNT, t->connection_count);
        }
    }

    // Fill in connection info for listener responses (by listener_unique_id)
    if (msg_type == ACMP_MSG_CONNECT_RX_RESPONSE ||
        msg_type == ACMP_MSG_GET_RX_STATE_RESPONSE ||
        msg_type == ACMP_MSG_DISCONNECT_RX_RESPONSE) {
        uint16_t luid = av_get_be16(p + ACMP_OFF_LISTENER_UID);
        if (luid < AVDECC_MAX_LISTENERS) {
            av_put_be16(p + ACMP_OFF_CONN_COUNT,
                        s->listeners[luid].connection_count);
        }
    }

    uint32_t frame_len = 14 + ACMPDU_LEN;
    if (frame_len < 64)
        frame_len = 64;

    avdecc_eth_send(frame_len);
    s->acmp_tx_count++;
}

static void acmp_handle_connect_tx(avdecc_state_t *s, const uint8_t *pdu)
{
    // A controller or listener is asking us (talker) to start streaming
    const uint8_t *listener_id = pdu + ACMP_OFF_LISTENER_ID;
    uint16_t tuid = av_get_be16(pdu + ACMP_OFF_TALKER_UID);
    uint16_t luid = av_get_be16(pdu + ACMP_OFF_LISTENER_UID);

    if (tuid >= AVDECC_MAX_TALKERS) {
        acmp_send_response(s, ACMP_MSG_CONNECT_TX_RESPONSE,
                           ACMP_STATUS_TALKER_UNKNOWN_ID, pdu);
        return;
    }

    avdecc_talker_stream_t *t = &s->talkers[tuid];
    memcpy(t->listener_id, listener_id, 8);
    t->listener_uid = luid;
    t->connected = 1;
    t->connection_count++;

    printf("[AVDECC] CONNECT_TX uid=%u <- listener "
           "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
           tuid,
           listener_id[0], listener_id[1], listener_id[2], listener_id[3],
           listener_id[4], listener_id[5], listener_id[6], listener_id[7]);

    if (s->on_talker_connect)
        s->on_talker_connect(tuid, listener_id);

    acmp_send_response(s, ACMP_MSG_CONNECT_TX_RESPONSE, ACMP_STATUS_SUCCESS, pdu);
}

static void acmp_handle_disconnect_tx(avdecc_state_t *s, const uint8_t *pdu)
{
    uint16_t tuid = av_get_be16(pdu + ACMP_OFF_TALKER_UID);
    if (tuid < AVDECC_MAX_TALKERS) {
        avdecc_talker_stream_t *t = &s->talkers[tuid];
        t->connected = 0;
        if (t->connection_count > 0) t->connection_count--;
        if (s->on_talker_disconnect) s->on_talker_disconnect(tuid);
    }
    printf("[AVDECC] DISCONNECT_TX uid=%u\n", tuid);
    acmp_send_response(s, ACMP_MSG_DISCONNECT_TX_RESPONSE, ACMP_STATUS_SUCCESS, pdu);
}

static void acmp_handle_get_tx_state(avdecc_state_t *s, const uint8_t *pdu)
{
    acmp_send_response(s, ACMP_MSG_GET_TX_STATE_RESPONSE, ACMP_STATUS_SUCCESS, pdu);
}

static void acmp_handle_connect_rx(avdecc_state_t *s, const uint8_t *pdu)
{
    // A controller is telling us (listener) to connect to a talker stream
    const uint8_t *talker_id  = pdu + ACMP_OFF_TALKER_ID;
    const uint8_t *stream_id  = pdu + ACMP_OFF_STREAM_ID;
    const uint8_t *dest_mac   = pdu + ACMP_OFF_STREAM_DEST_MAC;
    uint16_t tuid = av_get_be16(pdu + ACMP_OFF_TALKER_UID);
    uint16_t luid = av_get_be16(pdu + ACMP_OFF_LISTENER_UID);

    if (luid >= AVDECC_MAX_LISTENERS) {
        acmp_send_response(s, ACMP_MSG_CONNECT_RX_RESPONSE,
                           ACMP_STATUS_LISTENER_UNKNOWN_ID, pdu);
        return;
    }

    avdecc_listener_stream_t *l = &s->listeners[luid];
    memcpy(l->talker_id, talker_id, 8);
    l->talker_uid = tuid;
    memcpy(l->stream_id, stream_id, 8);
    memcpy(l->dest_mac, dest_mac, 6);
    l->connected = 1;
    l->connection_count++;

    printf("[AVDECC] CONNECT_RX uid=%u stream "
           "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
           luid,
           stream_id[0], stream_id[1], stream_id[2], stream_id[3],
           stream_id[4], stream_id[5], stream_id[6], stream_id[7]);

    if (s->on_listener_connect)
        s->on_listener_connect(luid, stream_id, dest_mac, talker_id);

    acmp_send_response(s, ACMP_MSG_CONNECT_RX_RESPONSE, ACMP_STATUS_SUCCESS, pdu);
}

static void acmp_handle_disconnect_rx(avdecc_state_t *s, const uint8_t *pdu)
{
    uint16_t luid = av_get_be16(pdu + ACMP_OFF_LISTENER_UID);
    if (luid < AVDECC_MAX_LISTENERS) {
        avdecc_listener_stream_t *l = &s->listeners[luid];
        l->connected = 0;
        if (l->connection_count > 0) l->connection_count--;
        if (s->on_listener_disconnect) s->on_listener_disconnect(luid);
    }
    printf("[AVDECC] DISCONNECT_RX uid=%u\n", luid);
    acmp_send_response(s, ACMP_MSG_DISCONNECT_RX_RESPONSE, ACMP_STATUS_SUCCESS, pdu);
}

static void acmp_handle_get_rx_state(avdecc_state_t *s, const uint8_t *pdu)
{
    acmp_send_response(s, ACMP_MSG_GET_RX_STATE_RESPONSE, ACMP_STATUS_SUCCESS, pdu);
}

// ---------------------------------------------------------------------------
// AECP — AVDECC Enumeration and Control Protocol
// ---------------------------------------------------------------------------

// AECP PDU offsets (from PDU start, past Ethernet header)
#define AECP_OFF_TARGET_ID      4
#define AECP_OFF_CONTROLLER_ID  12
#define AECP_OFF_SEQ_ID         20
#define AECP_OFF_CMD_TYPE       22


// Helper: write zero-padded 64-byte name field
static void write_name64(uint8_t *p, const char *s)
{
    memset(p, 0, 64);
    for (int i = 0; i < 63 && s[i]; i++)
        p[i] = s[i];
}

// Build unicast AECP response header, returns PDU pointer (frame + 14)
static uint8_t *aecp_begin_response(uint8_t *frame, const uint8_t *src_mac,
                                     const uint8_t *rx_frame, const uint8_t *rx_pdu)
{
    memcpy(frame, rx_frame + 6, 6);    // dst = sender's MAC (unicast)
    memcpy(frame + 6, src_mac, 6);
    av_put_be16(frame + 12, AVDECC_ETHERTYPE);

    uint8_t *p = frame + 14;
    p[0] = AVTP_SUBTYPE_AECP;
    p[1] = AECP_MSG_AEM_RESPONSE;
    // status/cdl set by caller via aecp_set_status_cdl
    memcpy(p + 4, rx_pdu + 4, 20);     // target_id + controller_id + seq_id + cmd_type
    return p;
}

static void aecp_set_status_cdl(uint8_t *pdu, uint8_t status, uint16_t cdl)
{
    av_put_be16(pdu + 2, ((uint16_t)(status & 0x1F) << 11) | (cdl & 0x7FF));
}

// ---- Descriptor builders ----
// Each writes directly into buffer d, returns descriptor length.

static uint32_t build_desc_entity(uint8_t *d, uint16_t idx, avdecc_state_t *s)
{
    if (idx != 0) return 0;
    // IEEE 1722.1-2013 Section 7.2.1 — 312 bytes
    memset(d, 0, 312);
    av_put_be16(d + 0, AEM_DESC_ENTITY);
    av_put_be16(d + 2, 0);                       // descriptor_index
    memcpy(d + 4, s->entity_id, 8);             // entity_id
    memcpy(d + 12, s->entity_id, 8);            // entity_model_id
    av_put_be32(d + 20, ADP_CAP_ADP_SUPPORTED | ADP_CAP_ACMP_SUPPORTED |
                        ADP_CAP_GPTP_SUPPORTED); // entity_capabilities
    av_put_be16(d + 24, N_STREAM_OUTPUTS);       // talker_stream_sources = 1
    av_put_be16(d + 26, ADP_TALKER_CAP_IMPLEMENTED | ADP_TALKER_CAP_AUDIO_SOURCE);
    av_put_be16(d + 28, N_STREAM_INPUTS);        // listener_stream_sinks = 2
    av_put_be16(d + 30, ADP_LISTENER_CAP_IMPLEMENTED | ADP_LISTENER_CAP_AUDIO_SINK);
    av_put_be32(d + 36, s->adp_available_index); // available_index
    write_name64(d + 48, "AVB-AES3 Endpoint");   // entity_name
    write_name64(d + 116, "1.0.0");              // firmware_version
    av_put_be16(d + 308, 1);                     // configurations_count
    return 312;
}

static uint32_t build_desc_configuration(uint8_t *d, uint16_t idx)
{
    if (idx != 0) return 0;
    // 7.2.2 — 74 fixed + 11 descriptor_counts × 4 = 118 bytes
    memset(d, 0, 118);
    av_put_be16(d, AEM_DESC_CONFIGURATION);
    av_put_be16(d + 2, 0);
    write_name64(d + 4, "Default");
    av_put_be16(d + 68, 0xFFFF);   // localized_description (none)
    av_put_be16(d + 70, 11);       // descriptor_counts_count
    av_put_be16(d + 72, 74);       // descriptor_counts_offset

    uint8_t *c = d + 74;
    av_put_be16(c +  0, AEM_DESC_AUDIO_UNIT);          av_put_be16(c +  2, 1);
    av_put_be16(c +  4, AEM_DESC_STREAM_INPUT);        av_put_be16(c +  6, N_STREAM_INPUTS);
    av_put_be16(c +  8, AEM_DESC_STREAM_OUTPUT);       av_put_be16(c + 10, N_STREAM_OUTPUTS);
    av_put_be16(c + 12, AEM_DESC_AVB_INTERFACE);       av_put_be16(c + 14, 1);
    av_put_be16(c + 16, AEM_DESC_CLOCK_SOURCE);        av_put_be16(c + 18, N_CLOCK_SOURCES);
    av_put_be16(c + 20, AEM_DESC_LOCALE);              av_put_be16(c + 22, 1);
    av_put_be16(c + 24, AEM_DESC_STRINGS);             av_put_be16(c + 26, 1);
    av_put_be16(c + 28, AEM_DESC_STREAM_PORT_INPUT);   av_put_be16(c + 30, 1);
    av_put_be16(c + 32, AEM_DESC_STREAM_PORT_OUTPUT);  av_put_be16(c + 34, 1);
    av_put_be16(c + 36, AEM_DESC_AUDIO_CLUSTER);       av_put_be16(c + 38, N_AUDIO_CHANNELS * 2);
    av_put_be16(c + 40, AEM_DESC_CLOCK_DOMAIN);        av_put_be16(c + 42, 1);
    return 118;
}

static uint32_t build_desc_audio_unit(uint8_t *d, uint16_t idx)
{
    if (idx != 0) return 0;
    // 7.2.3 — 144 fixed + 1 sampling rate (4) = 148 bytes
    memset(d, 0, 148);
    av_put_be16(d, AEM_DESC_AUDIO_UNIT);
    av_put_be16(d + 2, 0);
    write_name64(d + 4, "Audio Unit");
    av_put_be16(d + 68, 0xFFFF);                 // localized_description
    av_put_be16(d + 70, 0);                       // clock_domain_index
    av_put_be16(d + 72, 1);                       // number_of_stream_input_ports
    av_put_be16(d + 74, 0);                       // base_stream_input_port
    av_put_be16(d + 76, 1);                       // number_of_stream_output_ports
    av_put_be16(d + 78, 0);                       // base_stream_output_port
    // 80..95 = other port counts (all 0)
    av_put_be32(d + 136, 48000);                  // current_sampling_rate
    av_put_be16(d + 140, 144);                    // sampling_rates_offset
    av_put_be16(d + 142, 1);                      // sampling_rates_count
    av_put_be32(d + 144, 48000);                  // rate[0]
    return 148;
}

static uint32_t build_desc_stream_input(uint8_t *d, uint16_t idx)
{
    // 7.2.6 — 132 fixed + 1 format (8) = 140 bytes
    if (idx >= N_STREAM_INPUTS) return 0;
    memset(d, 0, 140);
    av_put_be16(d, AEM_DESC_STREAM_INPUT);
    av_put_be16(d + 2, idx);

    const uint8_t *fmt;
    const char *name;
    uint16_t flags;
    if (idx == LISTENER_CRF_INDEX) {
        fmt   = stream_fmt_crf_48k;
        name  = "Media Clock Input";
        flags = STREAM_FLAG_CLOCK_SYNC_SOURCE | STREAM_FLAG_CLASS_A;
    } else {
        fmt   = stream_fmt_aaf_8ch_48k;
        name  = "Audio Input";
        flags = STREAM_FLAG_CLASS_A;
    }

    write_name64(d + 4, name);
    av_put_be16(d + 68, 0xFFFF);                  // localized_description
    av_put_be16(d + 70, 0);                       // clock_domain_index
    av_put_be16(d + 72, flags);                   // stream_flags
    memcpy(d + 74, fmt, 8);                       // current_format
    av_put_be16(d + 82, 132);                     // formats_offset
    av_put_be16(d + 84, 1);                       // number_of_formats
    // 86..103 = backup_talker_*  (zeroed)
    av_put_be16(d + 104, 0);                      // avb_interface_index
    av_put_be32(d + 106, 2000000);                // buffer_length (2ms in ns)
    memcpy(d + 132, fmt, 8);                      // format[0]
    return 140;
}

static uint32_t build_desc_stream_output(uint8_t *d, uint16_t idx)
{
    if (idx >= N_STREAM_OUTPUTS) return 0;
    memset(d, 0, 140);
    av_put_be16(d, AEM_DESC_STREAM_OUTPUT);
    av_put_be16(d + 2, idx);
    write_name64(d + 4, "Audio Output");
    av_put_be16(d + 68, 0xFFFF);
    av_put_be16(d + 70, 0);                       // clock_domain_index
    av_put_be16(d + 72, STREAM_FLAG_CLASS_A);
    memcpy(d + 74, stream_fmt_aaf_8ch_48k, 8);
    av_put_be16(d + 82, 132);
    av_put_be16(d + 84, 1);
    av_put_be16(d + 104, 0);
    av_put_be32(d + 106, 2000000);
    memcpy(d + 132, stream_fmt_aaf_8ch_48k, 8);
    return 140;
}

static uint32_t build_desc_avb_interface(uint8_t *d, uint16_t idx, avdecc_state_t *s)
{
    if (idx != 0) return 0;
    // 7.2.8 — 100 fixed + 1 MSRP mapping (4) = 104 bytes
    memset(d, 0, 104);
    av_put_be16(d, AEM_DESC_AVB_INTERFACE);
    av_put_be16(d + 2, 0);
    write_name64(d + 4, "Ethernet");
    av_put_be16(d + 68, 0xFFFF);
    memcpy(d + 70, s->src_mac, 6);                // mac_address
    av_put_be16(d + 76, AVB_INTERFACE_FLAG_GPTP_SUPPORTED |
                        AVB_INTERFACE_FLAG_SRP_SUPPORTED);
    memcpy(d + 78, s->entity_id, 8);              // clock_identity
    d[86] = 248;                                  // priority1
    d[87] = 248;                                  // clock_class
    d[90] = 0xFE;                                 // clock_accuracy (unknown)
    d[91] = 248;                                  // priority2
    av_put_be16(d + 96, 100);                     // msrp_mappings_offset
    av_put_be16(d + 98, 1);                       // msrp_mappings_count
    d[100] = 1;                                   // traffic_class = SR class A
    d[101] = 3;                                   // priority
    av_put_be16(d + 102, 2);                      // vlan_id
    return 104;
}

static uint32_t build_desc_clock_source(uint8_t *d, uint16_t idx, avdecc_state_t *s)
{
    if (idx >= N_CLOCK_SOURCES) return 0;
    // 7.2.9 — 86 bytes
    memset(d, 0, 86);
    av_put_be16(d, AEM_DESC_CLOCK_SOURCE);
    av_put_be16(d + 2, idx);

    if (idx == CLK_SRC_INTERNAL) {
        write_name64(d + 4, "Internal");
        av_put_be16(d + 68, 0xFFFF);
        av_put_be16(d + 70, 0);                   // flags
        av_put_be16(d + 72, CLOCK_SOURCE_TYPE_INTERNAL);
        memcpy(d + 74, s->entity_id, 8);          // identifier
        av_put_be16(d + 82, AEM_DESC_AUDIO_UNIT); // location_type
        av_put_be16(d + 84, 0);                   // location_index
    } else {
        write_name64(d + 4, "Media Clock");
        av_put_be16(d + 68, 0xFFFF);
        av_put_be16(d + 70, 0);
        av_put_be16(d + 72, CLOCK_SOURCE_TYPE_INPUT_STREAM);
        memcpy(d + 74, s->entity_id, 8);
        av_put_be16(d + 82, AEM_DESC_STREAM_INPUT);
        av_put_be16(d + 84, LISTENER_CRF_INDEX);  // points at CRF stream
    }
    return 86;
}

static uint32_t build_desc_clock_domain(uint8_t *d, uint16_t idx, avdecc_state_t *s)
{
    if (idx != 0) return 0;
    // 7.2.10 — 76 fixed + N_CLOCK_SOURCES × 2 bytes
    uint32_t len = 76 + N_CLOCK_SOURCES * 2;
    memset(d, 0, len);
    av_put_be16(d, AEM_DESC_CLOCK_DOMAIN);
    av_put_be16(d + 2, 0);
    write_name64(d + 4, "Clock Domain");
    av_put_be16(d + 68, 0xFFFF);
    av_put_be16(d + 70, s->current_clock_source); // clock_source_index (dynamic)
    av_put_be16(d + 72, 76);                       // clock_sources_offset
    av_put_be16(d + 74, N_CLOCK_SOURCES);          // clock_sources_count
    for (uint16_t i = 0; i < N_CLOCK_SOURCES; i++)
        av_put_be16(d + 76 + 2 * i, i);
    return len;
}

static uint32_t build_desc_locale(uint8_t *d, uint16_t idx)
{
    if (idx != 0) return 0;
    // 7.2.11 — 68 bytes
    memset(d, 0, 68);
    av_put_be16(d, AEM_DESC_LOCALE);
    av_put_be16(d + 2, 0);
    // locale_identifier: 64-byte ASCII string
    const char *loc = "en-US";
    memcpy(d + 4, loc, 5);
    av_put_be16(d + 68 - 4, 1);   // number_of_strings
    av_put_be16(d + 68 - 2, 0);   // base_strings = 0
    return 68;
}

// 7-string STRINGS descriptor: each string is fixed 64 bytes.
// strings[0] = "AVB-AES3", strings[1] = "Audio Unit", etc.
static uint32_t build_desc_strings(uint8_t *d, uint16_t idx)
{
    if (idx != 0) return 0;
    // 7.2.12 — 4 header + 7 × 64 string slots = 452 bytes
    memset(d, 0, 452);
    av_put_be16(d, AEM_DESC_STRINGS);
    av_put_be16(d + 2, 0);
    static const char *const strs[7] = {
        "AVB-AES3",          // 0
        "Audio Unit",        // 1
        "Media Clock Input", // 2
        "Audio Input",       // 3
        "Audio Output",      // 4
        "Clock Domain",      // 5
        "Ethernet"           // 6
    };
    for (int i = 0; i < 7; i++)
        write_name64(d + 4 + i * 64, strs[i]);
    return 452;
}

static uint32_t build_desc_stream_port(uint8_t *d, uint16_t desc_type, uint16_t idx)
{
    if (idx != 0) return 0;
    // 7.2.13 — 16 bytes
    memset(d, 0, 16);
    av_put_be16(d +  0, desc_type);
    av_put_be16(d +  2, 0);
    av_put_be16(d +  4, 0);                              // clock_domain_index
    av_put_be16(d +  6, STREAM_PORT_FLAG_CLOCK_SYNC_SOURCE);
    av_put_be16(d +  8, N_AUDIO_CHANNELS);               // number_of_controls = 0; reuse field
    // For STREAM_PORT_INPUT: number_of_clusters = 8, base_cluster = 0
    // For STREAM_PORT_OUTPUT: number_of_clusters = 8, base_cluster = 8
    // 7.2.13 layout:
    //   +0  desc_type
    //   +2  desc_index
    //   +4  clock_domain_index
    //   +6  port_flags
    //   +8  number_of_controls
    //  +10  base_control
    //  +12  number_of_clusters
    //  +14  base_cluster
    av_put_be16(d +  8, 0);                              // number_of_controls
    av_put_be16(d + 10, 0);                              // base_control
    av_put_be16(d + 12, N_AUDIO_CHANNELS);               // number_of_clusters
    av_put_be16(d + 14, (desc_type == AEM_DESC_STREAM_PORT_INPUT) ? 0 : N_AUDIO_CHANNELS);
    return 16;
}

static uint32_t build_desc_audio_cluster(uint8_t *d, uint16_t idx)
{
    // 7.2.16 — 83 bytes total:
    //  +0   descriptor_type (2)
    //  +2   descriptor_index (2)
    //  +4   object_name (64)
    //  +68  localized_description (2)
    //  +70  signal_type (2)
    //  +72  signal_index (2)
    //  +74  signal_output (2)
    //  +76  path_latency (4)
    //  +80  block_latency (4)
    //  +84  channel_count (2)
    //  +86  format (1)
    if (idx >= N_AUDIO_CHANNELS * 2) return 0;
    memset(d, 0, 87);
    av_put_be16(d, AEM_DESC_AUDIO_CLUSTER);
    av_put_be16(d + 2, idx);

    char name[8];
    int ch = (idx < N_AUDIO_CHANNELS) ? (idx + 1) : (idx - N_AUDIO_CHANNELS + 1);
    const char *prefix = (idx < N_AUDIO_CHANNELS) ? "In " : "Out ";
    int n = 0;
    while (prefix[n]) { name[n] = prefix[n]; n++; }
    if (ch >= 10) name[n++] = '0' + (ch / 10);
    name[n++] = '0' + (ch % 10);
    name[n] = 0;
    write_name64(d + 4, name);

    av_put_be16(d + 68, 0xFFFF);                  // localized_description
    // signal_type/index/output, path/block latency all 0
    av_put_be16(d + 84, 1);                       // channel_count
    d[86] = AUDIO_CLUSTER_FORMAT_MBLA;            // format
    return 87;
}

// ---- AECP command handler ----

static void aecp_handle(avdecc_state_t *s, const uint8_t *frame,
                         const uint8_t *pdu, uint32_t pdu_len)
{
    uint8_t msg_type = pdu[1] & 0x0F;

    if (msg_type != AECP_MSG_AEM_COMMAND)
        return;

    if (!entity_id_match(pdu + AECP_OFF_TARGET_ID, s->entity_id))
        return;

    s->aecp_rx_count++;

    uint16_t cmd_type = av_get_be16(pdu + AECP_OFF_CMD_TYPE) & 0x7FFF;

    switch (cmd_type) {

    case AEM_CMD_READ_DESCRIPTOR: {
        if (pdu_len < 32) return;

        uint16_t desc_type  = av_get_be16(pdu + 28);
        uint16_t desc_index = av_get_be16(pdu + 30);

        uint8_t *tf = avdecc_tx_buf();
        uint8_t *tp = aecp_begin_response(tf, s->src_mac, frame, pdu);
        av_put_be16(tp + 24, av_get_be16(pdu + 24));  // echo config_index
        av_put_be16(tp + 26, 0);                        // reserved

        uint8_t *desc = tp + 28;
        uint32_t desc_len = 0;

        switch (desc_type) {
            case AEM_DESC_ENTITY:
                desc_len = build_desc_entity(desc, desc_index, s); break;
            case AEM_DESC_CONFIGURATION:
                desc_len = build_desc_configuration(desc, desc_index); break;
            case AEM_DESC_AUDIO_UNIT:
                desc_len = build_desc_audio_unit(desc, desc_index); break;
            case AEM_DESC_STREAM_INPUT:
                desc_len = build_desc_stream_input(desc, desc_index); break;
            case AEM_DESC_STREAM_OUTPUT:
                desc_len = build_desc_stream_output(desc, desc_index); break;
            case AEM_DESC_AVB_INTERFACE:
                desc_len = build_desc_avb_interface(desc, desc_index, s); break;
            case AEM_DESC_CLOCK_SOURCE:
                desc_len = build_desc_clock_source(desc, desc_index, s); break;
            case AEM_DESC_CLOCK_DOMAIN:
                desc_len = build_desc_clock_domain(desc, desc_index, s); break;
            case AEM_DESC_LOCALE:
                desc_len = build_desc_locale(desc, desc_index); break;
            case AEM_DESC_STRINGS:
                desc_len = build_desc_strings(desc, desc_index); break;
            case AEM_DESC_STREAM_PORT_INPUT:
            case AEM_DESC_STREAM_PORT_OUTPUT:
                desc_len = build_desc_stream_port(desc, desc_type, desc_index); break;
            case AEM_DESC_AUDIO_CLUSTER:
                desc_len = build_desc_audio_cluster(desc, desc_index); break;
            default:
                break;
        }

        if (desc_len == 0) {
            aecp_set_status_cdl(tp, AECP_STATUS_NOT_IMPLEMENTED, 16);
            avdecc_eth_send(14 + 28);
            s->aecp_tx_count++;
            return;
        }

        aecp_set_status_cdl(tp, AECP_STATUS_SUCCESS, 16 + desc_len);
        uint32_t flen = 14 + 28 + desc_len;
        if (flen < 64) flen = 64;
        avdecc_eth_send(flen);
        s->aecp_tx_count++;
        break;
    }

    case AEM_CMD_ACQUIRE_ENTITY:
    case AEM_CMD_LOCK_ENTITY: {
        // Accept without tracking — simple endpoint
        uint8_t *tf = avdecc_tx_buf();
        uint8_t *tp = aecp_begin_response(tf, s->src_mac, frame, pdu);
        memcpy(tp + 24, pdu + 24, 16);  // echo flags + owner/locked_id + desc_type + desc_index

        // For ACQUIRE, set owner to controller
        if (cmd_type == AEM_CMD_ACQUIRE_ENTITY)
            memcpy(tp + 28, pdu + AECP_OFF_CONTROLLER_ID, 8);

        aecp_set_status_cdl(tp, AECP_STATUS_SUCCESS, 28);
        avdecc_eth_send(14 + 40 < 64 ? 64 : 14 + 40);
        s->aecp_tx_count++;
        break;
    }

    case AEM_CMD_GET_STREAM_FORMAT: {
        if (pdu_len < 28) return;
        uint16_t dt = av_get_be16(pdu + 24);
        uint16_t di = av_get_be16(pdu + 26);
        const uint8_t *fmt = NULL;
        if (dt == AEM_DESC_STREAM_INPUT) {
            if (di == LISTENER_CRF_INDEX)      fmt = stream_fmt_crf_48k;
            else if (di == LISTENER_AAF_INDEX) fmt = stream_fmt_aaf_8ch_48k;
        } else if (dt == AEM_DESC_STREAM_OUTPUT && di == TALKER_AAF_INDEX) {
            fmt = stream_fmt_aaf_8ch_48k;
        }
        uint8_t *tf = avdecc_tx_buf();
        uint8_t *tp = aecp_begin_response(tf, s->src_mac, frame, pdu);
        memcpy(tp + 24, pdu + 24, 4);                 // echo desc_type + desc_index
        if (fmt) {
            memcpy(tp + 28, fmt, 8);
            aecp_set_status_cdl(tp, AECP_STATUS_SUCCESS, 24);
        } else {
            memset(tp + 28, 0, 8);
            aecp_set_status_cdl(tp, AECP_STATUS_NO_SUCH_DESCRIPTOR, 24);
        }
        avdecc_eth_send(64);  // 14 + 36 = 50, pad to 64
        s->aecp_tx_count++;
        break;
    }

    case AEM_CMD_SET_STREAM_FORMAT: {
        // Payload: desc_type(2) desc_index(2) stream_format(8)
        if (pdu_len < 36) return;
        uint16_t dt = av_get_be16(pdu + 24);
        uint16_t di = av_get_be16(pdu + 26);
        const uint8_t *want = pdu + 28;

        const uint8_t *expect = NULL;
        if (dt == AEM_DESC_STREAM_INPUT) {
            if (di == LISTENER_CRF_INDEX)      expect = stream_fmt_crf_48k;
            else if (di == LISTENER_AAF_INDEX) expect = stream_fmt_aaf_8ch_48k;
        } else if (dt == AEM_DESC_STREAM_OUTPUT && di == TALKER_AAF_INDEX) {
            expect = stream_fmt_aaf_8ch_48k;
        }

        uint8_t st;
        if (!expect) {
            st = AECP_STATUS_NO_SUCH_DESCRIPTOR;
        } else {
            int diff = 0;
            for (int i = 0; i < 8; i++)
                diff |= want[i] ^ expect[i];
            st = diff ? AECP_STATUS_BAD_ARGUMENTS : AECP_STATUS_SUCCESS;
        }

        uint8_t *tf = avdecc_tx_buf();
        uint8_t *tp = aecp_begin_response(tf, s->src_mac, frame, pdu);
        memcpy(tp + 24, pdu + 24, 12);                // echo desc_type+idx+format
        aecp_set_status_cdl(tp, st, 24);
        avdecc_eth_send(64);
        s->aecp_tx_count++;
        printf("[AVDECC] SET_STREAM_FORMAT dt=%u di=%u -> status=%u\n", dt, di, st);
        break;
    }

    case AEM_CMD_GET_CLOCK_SOURCE: {
        // Payload (response): desc_type(2) desc_index(2) clock_source_index(2) rsvd(2)
        if (pdu_len < 28) return;
        uint16_t dt = av_get_be16(pdu + 24);
        uint16_t di = av_get_be16(pdu + 26);
        uint8_t st = (dt == AEM_DESC_CLOCK_DOMAIN && di == 0)
                       ? AECP_STATUS_SUCCESS : AECP_STATUS_NO_SUCH_DESCRIPTOR;
        uint8_t *tf = avdecc_tx_buf();
        uint8_t *tp = aecp_begin_response(tf, s->src_mac, frame, pdu);
        memcpy(tp + 24, pdu + 24, 4);                 // echo desc_type+idx
        av_put_be16(tp + 28, s->current_clock_source);
        av_put_be16(tp + 30, 0);                       // reserved
        aecp_set_status_cdl(tp, st, 20);
        avdecc_eth_send(64);
        s->aecp_tx_count++;
        break;
    }

    case AEM_CMD_SET_CLOCK_SOURCE: {
        if (pdu_len < 32) return;
        uint16_t dt  = av_get_be16(pdu + 24);
        uint16_t di  = av_get_be16(pdu + 26);
        uint16_t src = av_get_be16(pdu + 28);

        uint8_t st;
        if (dt != AEM_DESC_CLOCK_DOMAIN || di != 0)
            st = AECP_STATUS_NO_SUCH_DESCRIPTOR;
        else if (src >= N_CLOCK_SOURCES)
            st = AECP_STATUS_BAD_ARGUMENTS;
        else {
            s->current_clock_source = src;
            st = AECP_STATUS_SUCCESS;
            if (s->on_clock_source_change)
                s->on_clock_source_change(src);
        }

        uint8_t *tf = avdecc_tx_buf();
        uint8_t *tp = aecp_begin_response(tf, s->src_mac, frame, pdu);
        memcpy(tp + 24, pdu + 24, 4);
        av_put_be16(tp + 28, s->current_clock_source);
        av_put_be16(tp + 30, 0);
        aecp_set_status_cdl(tp, st, 20);
        avdecc_eth_send(64);
        s->aecp_tx_count++;
        printf("[AVDECC] SET_CLOCK_SOURCE -> %u (status=%u)\n",
               s->current_clock_source, st);
        break;
    }

    default: {
        // Unknown command — echo with NOT_IMPLEMENTED
        uint16_t orig_cdl = av_get_be16(pdu + 2) & 0x7FF;
        uint32_t copy_len = 12 + orig_cdl;
        if (copy_len > 500) copy_len = 500;

        uint8_t *tf = avdecc_tx_buf();
        memcpy(tf, frame + 6, 6);           // dst = sender's MAC
        memcpy(tf + 6, s->src_mac, 6);
        av_put_be16(tf + 12, AVDECC_ETHERTYPE);

        uint8_t *tp = tf + 14;
        memcpy(tp, pdu, copy_len);
        tp[0] = AVTP_SUBTYPE_AECP;
        tp[1] = AECP_MSG_AEM_RESPONSE;
        aecp_set_status_cdl(tp, AECP_STATUS_NOT_IMPLEMENTED, orig_cdl);

        uint32_t flen = 14 + copy_len;
        if (flen < 64) flen = 64;
        avdecc_eth_send(flen);
        s->aecp_tx_count++;
        break;
    }

    } // switch cmd_type
}

// ---------------------------------------------------------------------------
// RX dispatch
// ---------------------------------------------------------------------------

void avdecc_process_rx(avdecc_state_t *s, const uint8_t *frame, uint32_t len)
{
    if (len < 14 + 4)
        return;

    const uint8_t *pdu = frame + 14;
    uint8_t subtype = pdu[0];
    uint8_t msg_type = pdu[1] & 0x0F;

    if (subtype == AVTP_SUBTYPE_ADP) {
        // We could track other entities here, but for a minimal endpoint
        // we just note that we're on an AVDECC-capable network.
        return;
    }

    if (subtype == AVTP_SUBTYPE_ACMP) {
        if (len < 14 + ACMPDU_LEN)
            return;

        s->acmp_rx_count++;

        // Check if the command is addressed to us
        const uint8_t *talker_id   = pdu + ACMP_OFF_TALKER_ID;
        const uint8_t *listener_id = pdu + ACMP_OFF_LISTENER_ID;

        switch (msg_type) {
            case ACMP_MSG_CONNECT_TX_COMMAND:
                if (entity_id_match(talker_id, s->entity_id))
                    acmp_handle_connect_tx(s, pdu);
                break;
            case ACMP_MSG_DISCONNECT_TX_COMMAND:
                if (entity_id_match(talker_id, s->entity_id))
                    acmp_handle_disconnect_tx(s, pdu);
                break;
            case ACMP_MSG_GET_TX_STATE_COMMAND:
                if (entity_id_match(talker_id, s->entity_id))
                    acmp_handle_get_tx_state(s, pdu);
                break;
            case ACMP_MSG_CONNECT_RX_COMMAND:
                if (entity_id_match(listener_id, s->entity_id))
                    acmp_handle_connect_rx(s, pdu);
                break;
            case ACMP_MSG_DISCONNECT_RX_COMMAND:
                if (entity_id_match(listener_id, s->entity_id))
                    acmp_handle_disconnect_rx(s, pdu);
                break;
            case ACMP_MSG_GET_RX_STATE_COMMAND:
                if (entity_id_match(listener_id, s->entity_id))
                    acmp_handle_get_rx_state(s, pdu);
                break;
        }
        return;
    }

    if (subtype == AVTP_SUBTYPE_AECP) {
        aecp_handle(s, frame, pdu, len - 14);
        return;
    }
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void avdecc_init(avdecc_state_t *s, const uint8_t *mac_addr)
{
    memset(s, 0, sizeof(*s));

    memcpy(s->src_mac, mac_addr, 6);

    // Entity ID: MAC(6) + 0x0000(2)
    memcpy(s->entity_id, mac_addr, 6);
    s->entity_id[6] = 0x00;
    s->entity_id[7] = 0x00;

    avdecc_txslot = 0;

    printf("[AVDECC] Entity ID=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
           s->entity_id[0], s->entity_id[1], s->entity_id[2], s->entity_id[3],
           s->entity_id[4], s->entity_id[5], s->entity_id[6], s->entity_id[7]);
}

void avdecc_set_talker_stream(avdecc_state_t *s, uint16_t uid,
                              const uint8_t *stream_id,
                              const uint8_t *stream_dest_mac)
{
    if (uid >= AVDECC_MAX_TALKERS) return;
    memcpy(s->talkers[uid].stream_id, stream_id, 8);
    memcpy(s->talkers[uid].dest_mac,  stream_dest_mac, 6);
}

void avdecc_depart(avdecc_state_t *s)
{
    adp_send(s, ADP_MSG_ENTITY_DEPARTING);
    printf("[AVDECC] Entity departing\n");
}

// ---------------------------------------------------------------------------
// Poll — periodic ADP advertisement
// ---------------------------------------------------------------------------

void avdecc_poll(avdecc_state_t *s)
{
    ptp_timestamp_t now = gptp_read_time();
    uint32_t now_ms = (uint32_t)(now.seconds & 0xFFFF) * 1000 +
                      (uint32_t)(now.nanoseconds / 1000000);

    uint32_t elapsed = now_ms - s->last_adp_ms;
    if (elapsed > 2000000000)
        elapsed = ADP_ADVERTISE_PERIOD_MS;

    if (elapsed >= ADP_ADVERTISE_PERIOD_MS) {
        adp_send(s, ADP_MSG_ENTITY_AVAILABLE);
        s->last_adp_ms = now_ms;
    }
}
