// IEEE 1722.1 AVDECC — minimal endpoint
// ADP (discovery) + ACMP (connection management)
//
// Uses AVTP EtherType (0x22F0) with subtypes 0x7A (ADP) and 0x7C (ACMP).
// Multicast to 91:E0:F0:01:00:00.

#ifndef AVDECC_H
#define AVDECC_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

// AVDECC uses the AVTP EtherType
#define AVDECC_ETHERTYPE        0x22F0

// AVTP subtypes for AVDECC. Byte 0 of an AVTP control PDU is cd(1)|subtype(7);
// for ADP/AECP/ACMP the cd bit MUST be 1, so the on-the-wire byte is 0xFA/0xFB/0xFC
// (not 0x7A/0x7B/0x7C). Hive/L-Acoustics drops frames with cd=0 as malformed.
#define AVTP_SUBTYPE_ADP        0xFA
#define AVTP_SUBTYPE_AECP       0xFB
#define AVTP_SUBTYPE_ACMP       0xFC

// AVDECC multicast address (IEEE 1722.1-2013 Table 7.1)
#define AVDECC_MCAST_ADDR       {0x91, 0xE0, 0xF0, 0x01, 0x00, 0x00}

// ADP message types
#define ADP_MSG_ENTITY_AVAILABLE    0
#define ADP_MSG_ENTITY_DEPARTING    1
#define ADP_MSG_ENTITY_DISCOVER     2

// ADP valid_time: in 2-second units.  10 = 20 seconds before expiry.
#define ADP_VALID_TIME              10
#define ADP_ADVERTISE_PERIOD_MS     10000   // Send every 10s (< valid_time * 2)

// ADP Entity Capabilities (IEEE 1722.1-2013 Table 6.2)
#define ADP_CAP_ADP_SUPPORTED                   (1u << 0)
#define ADP_CAP_ACMP_SUPPORTED                  (1u << 1)
#define ADP_CAP_AVDECC_ENTITY_PROXY             (1u << 2)
// ...
#define ADP_CAP_GPTP_SUPPORTED                  (1u << 11)

// ADP Talker Capabilities
#define ADP_TALKER_CAP_IMPLEMENTED              (1u << 0)
#define ADP_TALKER_CAP_AUDIO_SOURCE             (1u << 14)

// ADP Listener Capabilities
#define ADP_LISTENER_CAP_IMPLEMENTED            (1u << 0)
#define ADP_LISTENER_CAP_AUDIO_SINK             (1u << 14)

// ACMP message types
#define ACMP_MSG_CONNECT_TX_COMMAND             0
#define ACMP_MSG_CONNECT_TX_RESPONSE            1
#define ACMP_MSG_DISCONNECT_TX_COMMAND          2
#define ACMP_MSG_DISCONNECT_TX_RESPONSE         3
#define ACMP_MSG_GET_TX_STATE_COMMAND           4
#define ACMP_MSG_GET_TX_STATE_RESPONSE          5
#define ACMP_MSG_CONNECT_RX_COMMAND             6
#define ACMP_MSG_CONNECT_RX_RESPONSE            7
#define ACMP_MSG_DISCONNECT_RX_COMMAND          8
#define ACMP_MSG_DISCONNECT_RX_RESPONSE         9
#define ACMP_MSG_GET_RX_STATE_COMMAND           10
#define ACMP_MSG_GET_RX_STATE_RESPONSE          11

// ACMP status codes
#define ACMP_STATUS_SUCCESS                     0
#define ACMP_STATUS_LISTENER_UNKNOWN_ID         1
#define ACMP_STATUS_TALKER_UNKNOWN_ID           2
#define ACMP_STATUS_TALKER_DEST_MAC_FAIL        3
#define ACMP_STATUS_TALKER_NO_STREAM_INDEX      4
#define ACMP_STATUS_TALKER_NO_BANDWIDTH         5
#define ACMP_STATUS_TALKER_EXCLUSIVE            6
#define ACMP_STATUS_LISTENER_TALKER_TIMEOUT     7
#define ACMP_STATUS_LISTENER_EXCLUSIVE          8
#define ACMP_STATUS_STATE_UNAVAILABLE           9
#define ACMP_STATUS_NOT_CONNECTED               10
#define ACMP_STATUS_NO_SUCH_CONNECTION          11
#define ACMP_STATUS_NOT_SUPPORTED               31

// ACMP flags
#define ACMP_FLAG_CLASS_B                       (1u << 0)
#define ACMP_FLAG_FAST_CONNECT                  (1u << 1)
#define ACMP_FLAG_SAVED_STATE                   (1u << 2)
#define ACMP_FLAG_STREAMING_WAIT                (1u << 3)
#define ACMP_FLAG_SUPPORTS_ENCRYPTED            (1u << 4)

// AECP message types
#define AECP_MSG_AEM_COMMAND                    0
#define AECP_MSG_AEM_RESPONSE                   1

// AECP/AEM status codes
#define AECP_STATUS_SUCCESS                     0
#define AECP_STATUS_NOT_IMPLEMENTED             1

// AEM command types (IEEE 1722.1-2013 Table 7.126)
#define AEM_CMD_ACQUIRE_ENTITY                  0x0000
#define AEM_CMD_LOCK_ENTITY                     0x0001
#define AEM_CMD_READ_DESCRIPTOR                 0x0004
#define AEM_CMD_GET_STREAM_FORMAT               0x0009

// AEM descriptor types
#define AEM_DESC_ENTITY                         0x0000
#define AEM_DESC_CONFIGURATION                  0x0001
#define AEM_DESC_AUDIO_UNIT                     0x0005
#define AEM_DESC_STREAM_INPUT                   0x0006
#define AEM_DESC_STREAM_OUTPUT                  0x0007
#define AEM_DESC_AVB_INTERFACE                  0x000A
#define AEM_DESC_CLOCK_SOURCE                   0x000B
#define AEM_DESC_CLOCK_DOMAIN                   0x000C

// ---------------------------------------------------------------------------
// AVDECC state
// ---------------------------------------------------------------------------

typedef struct {
    // Identity
    uint8_t  entity_id[8];      // Typically MAC + 0x0000 + unique_id
    uint8_t  src_mac[6];

    // ADP state
    uint32_t adp_available_index;   // Increments each advertisement
    uint32_t last_adp_ms;

    // Talker connection state
    uint8_t  talker_connected;
    uint8_t  talker_listener_id[8]; // Entity ID of connected listener
    uint16_t talker_connection_count;

    // Listener connection state
    uint8_t  listener_connected;
    uint8_t  listener_talker_id[8]; // Entity ID of connected talker
    uint8_t  listener_stream_id[8]; // Stream ID we're listening to
    uint8_t  listener_dest_mac[6];  // Stream destination MAC
    uint16_t listener_connection_count;

    // Stream config (set by caller)
    uint8_t  stream_id[8];
    uint8_t  stream_dest_mac[6];

    // Callbacks (set by caller to wire AVTP/SRP)
    void (*on_talker_connect)(const uint8_t *listener_entity_id);
    void (*on_talker_disconnect)(void);
    void (*on_listener_connect)(const uint8_t *stream_id, const uint8_t *dest_mac,
                                const uint8_t *talker_entity_id);
    void (*on_listener_disconnect)(void);

    // Stats
    uint32_t adp_tx_count;
    uint32_t acmp_rx_count;
    uint32_t acmp_tx_count;
    uint32_t aecp_rx_count;
    uint32_t aecp_tx_count;
} avdecc_state_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

void avdecc_init(avdecc_state_t *s, const uint8_t *mac_addr,
                 const uint8_t *stream_id, const uint8_t *stream_dest_mac);

// Process received AVDECC frame (called from RX dispatch for EtherType 0x22F0
// with subtypes 0x7A-0x7C).
void avdecc_process_rx(avdecc_state_t *s, const uint8_t *frame, uint32_t len);

// Poll — sends periodic ADP advertisements.
void avdecc_poll(avdecc_state_t *s);

// Send ADP departing message (call before shutdown).
void avdecc_depart(avdecc_state_t *s);

#endif // AVDECC_H
