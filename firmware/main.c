// AVB-AES3 Firmware — Main entry point
// gPTP clock sync + AVTP audio streaming
// Runs as BIOS replacement on LiteX SoC (VexRiscv)

#include <stdio.h>
#include <string.h>

#include <irq.h>
#include <libbase/uart.h>
#include <libbase/console.h>
#include <system.h>
#include <libliteeth/mdio.h>
#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>

#include "gptp.h"
#include "avtp.h"
#include "aes3.h"
#include "srp.h"
#include "avdecc.h"
#include "mcr.h"
#include "aaf.h"

// MAC address — locally administered, unique per device.
// TODO: read from SPI flash or EEPROM in production.
static const uint8_t mac_addr[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x42};

static gptp_t gptp;
static avtp_stream_t avtp;
static aes3_state_t aes3;
static srp_state_t srp;
static avdecc_state_t avdecc;
static mcr_state_t    mcr;
static aaf_state_t    aaf;

// Local stream UIDs (must match build_desc_* in avdecc.c)
#define LISTENER_UID_CRF  0
#define LISTENER_UID_AAF  1
#define TALKER_UID_AAF    0

// Audio ring buffers (talker + listener)
static audio_ring_t tx_audio_ring;
static audio_ring_t rx_audio_ring;

// RX EtherType counters for link-debug
static uint32_t rx_total, rx_ptp, rx_avtp, rx_msrp, rx_other;
static uint16_t rx_last_ethertype;
static uint8_t  rx_last_dst[6];
static uint8_t  rx_last_src[6];

// ---------------------------------------------------------------------------
// Central Ethernet RX dispatcher
// ---------------------------------------------------------------------------

#define ETHMAC_EV_SRAM_WRITER 0x1
#define PTP_ETHERTYPE   0x88F7
// AVTP_ETHERTYPE is 0x22F0, defined in avtp.h

static void dispatch_rx(void)
{
    // Drain ALL pending RX slots in one dispatcher call. nrxslots=2 is
    // pinned (>2 silently breaks TX — see avb_soc.py:537); under MSRP /
    // AVDECC / CRF bursts the writer overruns within microseconds if we
    // service only one slot per call. Symptom: rx_avtp accumulates ~100×
    // slower than wire rate, talker_last_seen ages out at 30 s, and CRF
    // audio is never delivered to mcr_process_rx. Mirrors the equivalent
    // fix in avb_session_mgr2's start_crf_rx_pipeline drain loop.
    while (ethmac_sram_writer_ev_pending_read() & ETHMAC_EV_SRAM_WRITER) {

    uint32_t slot = ethmac_sram_writer_slot_read();
    uint8_t *frame = (uint8_t *)(ETHMAC_BASE + ETHMAC_SLOT_SIZE * slot);
    uint32_t len = ethmac_sram_writer_length_read();

    // Advance the RX-timestamp ring in lock-step with slot consumption.
    // The gateware pushes one entry per committed frame; we pop one per
    // dispatched slot regardless of ethertype. After this write, any
    // gptp_read_rx_timestamp() within this dispatch returns the value
    // for THIS slot's frame.
    main_rx_ts_pop_write(1);

    // Route by ethertype. Strip one 802.1Q VLAN tag in place if present —
    // Auvitran (and most AVB talkers) emit stream frames tagged with VLAN 2
    // (Class A), so the real AVTP ethertype is at offset 16, not 12. The
    // existing handlers all parse from offset 14 of `frame`, so we shift
    // the 4-byte tag out of the way rather than threading a payload_off.
    if (len >= 18) {
        uint16_t ethertype_pre = ((uint16_t)frame[12] << 8) | frame[13];
        if (ethertype_pre == 0x8100) {
            // Shift the inner ethertype + payload down 4 bytes, leaving
            // dst/src MAC in place. Net effect: tagged frame becomes the
            // equivalent untagged frame.
            memmove(frame + 12, frame + 16, len - 16);
            len -= 4;
        }
    }
    if (len >= 14) {
        uint16_t ethertype = ((uint16_t)frame[12] << 8) | frame[13];

        rx_total++;
        rx_last_ethertype = ethertype;
        memcpy(rx_last_dst, frame, 6);
        memcpy(rx_last_src, frame + 6, 6);

        switch (ethertype) {
            case PTP_ETHERTYPE:
                rx_ptp++;
                gptp_process_rx(&gptp, frame, len);
                break;
            case AVTP_ETHERTYPE: {
                rx_avtp++;
                // AVTP and AVDECC share EtherType 0x22F0. Demux by subtype.
                // Control PDUs (ADP/AECP/ACMP) have cd=1 → byte 0 = 0xFA/B/C.
                // Data PDUs (61883/AAF/CRF) have cd=0 → byte 0 = 0x00/02/04.
                if (len >= 15) {
                    uint8_t subtype = frame[14];
                    switch (subtype) {
                        case AVTP_SUBTYPE_ADP:
                        case AVTP_SUBTYPE_AECP:
                        case AVTP_SUBTYPE_ACMP:
                            avdecc_process_rx(&avdecc, frame, len);
                            break;
                        case AVTP_SUBTYPE_CRF:
                            mcr_process_rx(&mcr, frame, len);
                            break;
                        case AVTP_SUBTYPE_AAF:
                            aaf_process_rx(&aaf, frame, len);
                            break;
                        case AVTP_SUBTYPE_61883_IIDC:
                            avtp_process_rx(&avtp, frame, len);
                            break;
                        default:
                            break;
                    }
                }
                break;
            }
            case MSRP_ETHERTYPE:
                rx_msrp++;
                srp_process_rx(&srp, frame, len);
                break;
            default:
                rx_other++;
                break;
        }
    }

    // Acknowledge RX event — releases this slot back to the MAC and
    // advances slot_read to the next pending frame (if any). The
    // while-loop re-checks ev_pending to drain the second buffered slot
    // in the same call.
    ethmac_sram_writer_ev_pending_write(ETHMAC_EV_SRAM_WRITER);
    }
}

// ---------------------------------------------------------------------------
// UART debug commands
// ---------------------------------------------------------------------------

static void check_uart_cmd(void)
{
    if (!readchar_nonblock())
        return;

    char c = getchar();
    switch (c) {
        case 's': {
            ptp_timestamp_t t = gptp_read_time();
            printf("\n[gPTP] state=%d time=%llu.%09lu\n",
                   gptp.state,
                   (unsigned long long)t.seconds,
                   (unsigned long)t.nanoseconds);
            printf("  sync=%lu pdelay=%lu offset=%lld ns delay=%lld ns\n",
                   (unsigned long)gptp.sync_count,
                   (unsigned long)gptp.pdelay_count,
                   (long long)gptp.offset_from_master_ns,
                   (long long)gptp.mean_path_delay_ns);
            printf("  addend=%lu/%lu locked=%d\n",
                   (unsigned long)(gptp.current_addend_full >> 20),
                   (unsigned long)(gptp.current_addend_full & 0xFFFFFu),
                   gptp.servo_locked);
            printf("  gm: id=%02x%02x%02x%02x%02x%02x%02x%02x p1=%u cc=%u p2=%u valid=%u\n",
                   gptp.gm_clock_id[0], gptp.gm_clock_id[1], gptp.gm_clock_id[2], gptp.gm_clock_id[3],
                   gptp.gm_clock_id[4], gptp.gm_clock_id[5], gptp.gm_clock_id[6], gptp.gm_clock_id[7],
                   gptp.gm_priority1, gptp.gm_clock_class, gptp.gm_priority2, gptp.gm_valid);
            printf("  rx: sync=%lu fup=%lu pdreq=%lu pdresp=%lu pdfup=%lu ann=%lu other=%lu wdom=%lu last=mt%u dom%u\n",
                   (unsigned long)gptp.rx_sync_count,
                   (unsigned long)gptp.rx_followup_count,
                   (unsigned long)gptp.rx_pdelay_req_count,
                   (unsigned long)gptp.rx_pdelay_resp_count,
                   (unsigned long)gptp.rx_pdelay_resp_fup_count,
                   (unsigned long)gptp.rx_announce_count,
                   (unsigned long)gptp.rx_other_count,
                   (unsigned long)gptp.rx_wrong_domain_count,
                   gptp.rx_last_msg_type, gptp.rx_last_domain);
            printf("[AVTP] tx=%lu/%lu rx=%lu/%lu err=%lu\n",
                   (unsigned long)avtp.tx_packet_count,
                   (unsigned long)audio_ring_count(&tx_audio_ring),
                   (unsigned long)avtp.rx_packet_count,
                   (unsigned long)audio_ring_count(&rx_audio_ring),
                   (unsigned long)avtp.rx_seq_errors);
            printf("[AES3] lk=%d rx=%lu tx=%lu ov=%lu un=%lu\n",
                   aes3.rx_locked,
                   (unsigned long)aes3.rx_sample_count,
                   (unsigned long)aes3.tx_sample_count,
                   (unsigned long)aes3.rx_overrun_count,
                   (unsigned long)aes3.tx_underrun_count);
            printf("[SRP] tx=%lu rx=%lu domain=%d talker_reg=%d\n",
                   (unsigned long)srp.join_count,
                   (unsigned long)srp.rx_pdu_count,
                   srp.domain_received,
                   srp.talker_registered);
            printf("[AVDECC] adp=%lu acmp=%lu/%lu aecp=%lu/%lu "
                   "tx[aaf]=%d rx[crf]=%d rx[aaf]=%d clk_src=%u\n",
                   (unsigned long)avdecc.adp_tx_count,
                   (unsigned long)avdecc.acmp_rx_count,
                   (unsigned long)avdecc.acmp_tx_count,
                   (unsigned long)avdecc.aecp_rx_count,
                   (unsigned long)avdecc.aecp_tx_count,
                   avdecc.talkers[TALKER_UID_AAF].connected,
                   avdecc.listeners[LISTENER_UID_CRF].connected,
                   avdecc.listeners[LISTENER_UID_AAF].connected,
                   avdecc.current_clock_source);
            printf("[I2S] %lu %lu %lu\n",
                   (unsigned long)main_i2s_mmcm_locked_read(),
                   (unsigned long)main_i2s_bck_count_read(),
                   (unsigned long)main_i2s_lrck_count_read());
            break;
        }
        case 'r':
            printf("\nRebooting...\n");
            ctrl_reset_write(1);
            break;
        case 'e':
            printf("\n[RX] total=%u ptp=%u avtp=%u msrp=%u other=%u\n",
                   rx_total, rx_ptp, rx_avtp, rx_msrp, rx_other);
            printf("  last et=%04x dst=%02x:%02x:%02x:%02x:%02x:%02x src=%02x:%02x:%02x:%02x:%02x:%02x\n",
                   rx_last_ethertype,
                   rx_last_dst[0], rx_last_dst[1], rx_last_dst[2],
                   rx_last_dst[3], rx_last_dst[4], rx_last_dst[5],
                   rx_last_src[0], rx_last_src[1], rx_last_src[2],
                   rx_last_src[3], rx_last_src[4], rx_last_src[5]);
            printf("  mac: writer_errors=%u preamble_err=%u crc_err=%u ev_en=%u ev_st=%u ev_pd=%u\n",
                   ethmac_sram_writer_errors_read(),
                   ethmac_rx_datapath_preamble_errors_read(),
                   ethmac_rx_datapath_crc_errors_read(),
                   ethmac_sram_writer_ev_enable_read(),
                   ethmac_sram_writer_ev_status_read(),
                   ethmac_sram_writer_ev_pending_read());
            printf("  rx_ts ring: commits=%lu level=%lu overflow=%lu\n",
                   (unsigned long)main_rx_ts_commit_count_read(),
                   (unsigned long)main_rx_ts_level_read(),
                   (unsigned long)main_rx_ts_overflow_count_read());
            {
                uint8_t hba = main_eth_rx_heartbeat_read();
                busy_wait(300);  // ≥1 top-byte tick at 125e6/2^24 ≈ 7.5 Hz
                uint8_t hbb = main_eth_rx_heartbeat_read();
                int d = (int)((uint8_t)(hbb - hba));
                printf("  eth_rx heartbeat (PHY1 L3): %u -> %u (delta=%d) %s\n",
                       hba, hbb, d, d ? "alive" : "*** DEAD ***");
            }
            break;
        case 'm':
            printf("\n[MCR] bound=%d locked=%d step=%lu rx=%lu seq_err=%lu other=%lu bad_type=%lu\n",
                   mcr.bound, mcr.servo_locked,
                   (unsigned long)mcr.servo_step_count,
                   (unsigned long)mcr.rx_count,
                   (unsigned long)mcr.seq_errors,
                   (unsigned long)mcr.rx_other_count,
                   (unsigned long)mcr.bad_type_count);
            printf("  base_freq=%lu ts_interval=%u ts/pdu=%u type=%u pull=%u\n",
                   (unsigned long)mcr.base_frequency,
                   mcr.timestamp_interval, mcr.timestamps_per_pdu,
                   mcr.type, mcr.pull);
            printf("  offset_ns=%08lx_%08lx integral=%08lx_%08lx\n",
                   (unsigned long)(uint32_t)(mcr.latest_offset_ns >> 32),
                   (unsigned long)(uint32_t)mcr.latest_offset_ns,
                   (unsigned long)(uint32_t)(mcr.servo_integral >> 32),
                   (unsigned long)(uint32_t)mcr.servo_integral);
            printf("  inc base=%08lx cur=%08lx hw_sample_count=%lu hw_phase=%08lx\n",
                   (unsigned long)mcr.base_increment,
                   (unsigned long)mcr.current_increment,
                   (unsigned long)mcr_sample_count_read(),
                   (unsigned long)mcr_phase_read());
            break;
        case 'a':
            printf("\n[AAF] bound=%d rx_en=%d tx_en=%d\n"
                   "  rx: count=%lu seq_err=%lu other=%lu fmt_err=%lu lvl=%lu\n"
                   "  tx: count=%lu underrun=%lu lvl=%lu seq=%u\n"
                   "  last_pres_ts=%08lx\n",
                   aaf.bound, aaf.rx_enabled, aaf.tx_enabled,
                   (unsigned long)aaf.rx_count, (unsigned long)aaf.rx_seq_errors,
                   (unsigned long)aaf.rx_other_count, (unsigned long)aaf.format_errors,
                   (unsigned long)aaf_rx_level(&aaf),
                   (unsigned long)aaf.tx_packet_count,
                   (unsigned long)aaf.tx_underrun_count,
                   (unsigned long)aaf_tx_level(&aaf),
                   aaf.tx_seq,
                   (unsigned long)aaf.last_presentation_ts);
            break;
        case 'h':
        case '?':
            printf("\n  s   status (gPTP / AVTP / AES3 / SRP / AVDECC / I2S)\n"
                     "  m   MCR servo state (CRF lock, NCO increment, offset)\n"
                     "  a   AAF stream state (RX/TX counts, jitter buffer level)\n"
                     "  e   RX ethertype counters + LiteEth heartbeat\n"
                     "  r   reboot\n"
                     "  h   help\n"
                     "AAF talker/listener are driven by AVDECC, no manual toggle.\n");
            break;
    }
}

// ---------------------------------------------------------------------------
// Pending FAST_CONNECT bindings — when Hive sends ACMP CONNECT_RX with
// stream_id all-zero (Milan saved-state restore before talker is up), we
// remember the dest_mac here. As soon as SRP observes a TalkerAdvertise
// to that dest_mac, we copy its stream_id into mcr/aaf and start
// filtering AVTP packets by it.
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  active;             // 1 = waiting for SRP TalkerAdvertise
    uint8_t  uid;                // listener_unique_id (CRF or AAF)
    uint8_t  dest_mac[6];        // saved-state dest_mac (may be all-zero)
    uint8_t  talker_entity_id[8];// saved-state talker_eid (may be all-zero)
    uint16_t talker_uid;
} pending_bind_t;

static pending_bind_t pending_listeners[2];   // one slot per listener

static int stream_id_is_zero(const uint8_t *sid)
{
    int v = 0;
    for (int i = 0; i < 8; i++) v |= sid[i];
    return v == 0;
}

static int mac_eq(const uint8_t *a, const uint8_t *b)
{
    int diff = 0;
    for (int i = 0; i < 6; i++) diff |= a[i] ^ b[i];
    return !diff;
}

static int dest_or_eid_match(const pending_bind_t *p,
                              const uint8_t *adv_sid, const uint8_t *adv_dest)
{
    // 1) Strong match: dest_mac matches the saved binding's dest_mac.
    int dest_zero = 1;
    for (int i = 0; i < 6; i++) if (p->dest_mac[i]) { dest_zero = 0; break; }
    if (!dest_zero && mac_eq(p->dest_mac, adv_dest)) return 1;

    // 2) Saved talker_entity_id present? Match its MAC portion against
    //    the advertise stream_id's MAC portion. Talker_entity_id is
    //    typically MAC + 0x0000 or EUI-64-expanded MAC; in either form
    //    the first 3 bytes are the OUI of the talker's MAC.
    //    Advertised stream_id is typically MAC + uid; first 3 bytes are
    //    the same OUI. So OUI-equality on bytes [0..2] indicates a
    //    plausible match.
    int eid_zero = 1;
    for (int i = 0; i < 8; i++) if (p->talker_entity_id[i]) { eid_zero = 0; break; }
    if (!eid_zero) {
        // Compare first 3 bytes (OUI). Then if the talker_entity_id
        // looks like MAC+uid (bytes [3..5] != FF:FE), compare bytes
        // [3..5] as well for stricter matching.
        if (p->talker_entity_id[0] == adv_sid[0] &&
            p->talker_entity_id[1] == adv_sid[1] &&
            p->talker_entity_id[2] == adv_sid[2]) {
            int looks_like_eui64 = (p->talker_entity_id[3] == 0xFF &&
                                    p->talker_entity_id[4] == 0xFE);
            if (looks_like_eui64) return 1;
            if (p->talker_entity_id[3] == adv_sid[3] &&
                p->talker_entity_id[4] == adv_sid[4] &&
                p->talker_entity_id[5] == adv_sid[5]) return 1;
        }
    }
    return 0;
}

static void on_talker_advertise(const uint8_t *stream_id, const uint8_t *dest_mac)
{
    for (int i = 0; i < (int)(sizeof(pending_listeners)/sizeof(pending_listeners[0])); i++) {
        pending_bind_t *p = &pending_listeners[i];
        if (!p->active) continue;
        if (!dest_or_eid_match(p, stream_id, dest_mac)) continue;

        printf("[SRP-learn] uid=%u dest=%02x:%02x:%02x:%02x:%02x:%02x → "
               "stream_id=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
               p->uid,
               dest_mac[0], dest_mac[1], dest_mac[2],
               dest_mac[3], dest_mac[4], dest_mac[5],
               stream_id[0], stream_id[1], stream_id[2], stream_id[3],
               stream_id[4], stream_id[5], stream_id[6], stream_id[7]);

        // Reflect the learned stream identity AND MSRP-derived
        // attributes (dest_mac, vlan_id, accumulated_latency) back into
        // AVDECC so subsequent GET_STREAM_INFO/GetStreamInputInfoEx
        // responses carry wire-truth.
        if (p->uid < AVDECC_MAX_LISTENERS) {
            avdecc_listener_stream_t *l = &avdecc.listeners[p->uid];
            memcpy(l->stream_id, stream_id, 8);
            memcpy(l->dest_mac,  dest_mac,  6);
            const srp_remote_talker_t *t = srp_find_talker(&srp, stream_id);
            if (t) {
                l->msrp_accumulated_latency_ns = t->accumulated_latency_ns;
                l->stream_vlan_id              = t->vlan_id;
            }
        }

        // Now declare SRP ListenerReady with the REAL stream_id and bind
        // the audio engines to filter on it.
        srp_listener_enable(&srp, stream_id, 1);
        if (p->uid == LISTENER_UID_CRF)      mcr_bind(&mcr, stream_id);
        else if (p->uid == LISTENER_UID_AAF) aaf_bind(&aaf, stream_id);

        p->active = 0;
    }
}

// ---------------------------------------------------------------------------
// AVDECC callbacks — wire connection management to AVTP/SRP
// ---------------------------------------------------------------------------

static void on_talker_connect(uint16_t uid, const uint8_t *listener_entity_id)
{
    (void)listener_entity_id;
    if (uid != TALKER_UID_AAF) return;
    aaf_tx_enable(&aaf, 1);
    srp_talker_enable(&srp, 1);
    printf("[main] AAF talker started via AVDECC\n");
}

static void on_talker_disconnect(uint16_t uid)
{
    if (uid != TALKER_UID_AAF) return;
    aaf_tx_enable(&aaf, 0);
    srp_talker_enable(&srp, 0);
    printf("[main] AAF talker stopped via AVDECC\n");
}

static void on_listener_connect(uint16_t uid, const uint8_t *stream_id,
                                const uint8_t *dest_mac,
                                const uint8_t *talker_entity_id)
{
    // Two cases:
    //  1. Normal connect (Hive matrix drag): controller has discovered
    //     the talker's stream_id via ACMP CONNECT_TX and forwards it in
    //     CONNECT_RX. Bind immediately to filter AVTP frames on it.
    //  2. Milan FAST_CONNECT (saved state restore): stream_id all zeros
    //     — talker may still be offline. Park a pending entry and wait
    //     for SRP TalkerAdvertise matching either dest_mac OR the
    //     saved talker_entity_id (which Hive always populates even when
    //     the other fields are empty). Then bind to the learned stream_id.
    if (uid >= (uint16_t)(sizeof(pending_listeners)/sizeof(pending_listeners[0])))
        return;

    if (stream_id_is_zero(stream_id)) {
        pending_bind_t *p = &pending_listeners[uid];
        p->active = 1;
        p->uid    = (uint8_t)uid;
        memcpy(p->dest_mac, dest_mac, 6);
        memcpy(p->talker_entity_id, talker_entity_id, 8);
        // talker_uid not exposed in callback signature yet; not needed
        // for matching but recorded as 0 default.
        p->talker_uid = 0;
        printf("[main] Listener uid=%u FAST_CONNECT pending — "
               "talker=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x "
               "dest=%02x:%02x:%02x:%02x:%02x:%02x\n", uid,
               talker_entity_id[0], talker_entity_id[1], talker_entity_id[2],
               talker_entity_id[3], talker_entity_id[4], talker_entity_id[5],
               talker_entity_id[6], talker_entity_id[7],
               dest_mac[0], dest_mac[1], dest_mac[2],
               dest_mac[3], dest_mac[4], dest_mac[5]);
        return;
    }

    // Normal path — stream_id is known up front.
    pending_listeners[uid].active = 0;
    // If the talker is already in our SRP registrar table, copy its
    // MSRP-learned attributes so the next GET_STREAM_INFO is accurate.
    {
        const srp_remote_talker_t *t = srp_find_talker(&srp, stream_id);
        if (t) {
            avdecc.listeners[uid].msrp_accumulated_latency_ns = t->accumulated_latency_ns;
            avdecc.listeners[uid].stream_vlan_id              = t->vlan_id;
        }
    }
    srp_listener_enable(&srp, stream_id, 1);
    if (uid == LISTENER_UID_CRF)      mcr_bind(&mcr, stream_id);
    else if (uid == LISTENER_UID_AAF) aaf_bind(&aaf, stream_id);
}

static void on_listener_disconnect(uint16_t uid)
{
    if (uid < (uint16_t)(sizeof(pending_listeners)/sizeof(pending_listeners[0])))
        pending_listeners[uid].active = 0;

    if (uid == LISTENER_UID_CRF) {
        if (mcr.bound) srp_listener_enable(&srp, mcr.stream_id, 0);
        mcr_unbind(&mcr);
    } else if (uid == LISTENER_UID_AAF) {
        if (aaf.bound) srp_listener_enable(&srp, aaf.stream_id, 0);
        aaf_unbind(&aaf);
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(void)
{
#ifdef CONFIG_CPU_HAS_INTERRUPT
    irq_setmask(0);
    irq_setie(1);
#endif
    uart_init();

    printf("\n[AVB-AES3]\n");

    // Enable PHY-side RGMII-ID delays at boot. The B50612D on i9plus needs:
    //   shadow_07 bit 8 = 1 (RXC delay) — reg 0x18 write 0xF1E7
    //   shadow_03 bit 9 = 1 (TXC delay) — reg 0x1C write 0x8E00
    // Without TXC delay our outgoing frames are mis-clocked at the PHY and
    // never make it onto the wire (tcpdump shows 0 packets from us even when
    // firmware reports adp_tx_count growing). PHY shadow regs persist across
    // bitstream reloads but NOT across power-cycle, so program at boot.
    {
        int a = 1;
        mdio_write(a, 0x00, 0x9140);       // soft-reset + autoneg + 1G + FD
        busy_wait(200);
        mdio_write(a, 0x18, 0xF1E7);       // shadow_07 bit 8 = 1
        busy_wait(10);
        mdio_write(a, 0x1C, 0x8E00);       // shadow_03 bit 9 = 1
        busy_wait(10);
    }
    busy_wait(100);

    // Init audio ring buffers
    memset(&tx_audio_ring, 0, sizeof(tx_audio_ring));
    memset(&rx_audio_ring, 0, sizeof(rx_audio_ring));

    // Init protocol stacks
    gptp_init(&gptp, mac_addr);
    avtp_init(&avtp, mac_addr, &tx_audio_ring, &rx_audio_ring);
    aes3_init(&aes3);
    srp_init(&srp, mac_addr);
    mcr_init(&mcr, CONFIG_CLOCK_FREQUENCY, 48000);
    {
        // AAF uses the same talker stream_id/dest_mac advertised in AVDECC.
        // stream_id = MAC + 0x00 0x01 (matches avtp_set_stream_id default).
        uint8_t aaf_stream_id[8] = {0,0,0,0,0,0, 0x00, 0x01};
        memcpy(aaf_stream_id, mac_addr, 6);
        static const uint8_t aaf_mcast[] = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x00};
        aaf_init(&aaf, mac_addr, aaf_stream_id, aaf_mcast);
    }

    // Configure SRP talker to advertise our AAF stream
    srp_talker_set(&srp, aaf.stream_id, aaf.dest_mac, 230);   // 14+24+192

    // Observe every TalkerAdvertise on the wire — used to learn the
    // real stream_id for a FAST_CONNECT listener that arrived with
    // stream_id=0 from ACMP.
    srp.on_talker_advertise = on_talker_advertise;

    // Init AVDECC (discovery + connection management) using the AAF talker
    // stream identity as the advertised one.
    avdecc_init(&avdecc, mac_addr);
    avdecc_set_gptp(&gptp);   // surface GM identity in ADP/AVB_INTERFACE
    avdecc_set_mcr(&mcr);     // CLOCK_DOMAIN lock follows MCR when source=1
    avdecc_set_talker_stream(&avdecc, TALKER_UID_AAF, aaf.stream_id, aaf.dest_mac);
    avdecc.on_talker_connect    = on_talker_connect;
    avdecc.on_talker_disconnect = on_talker_disconnect;
    avdecc.on_listener_connect  = on_listener_connect;
    avdecc.on_listener_disconnect = on_listener_disconnect;

    printf("[main] Press 'h' for commands.\n\n");

    while (1) {
        dispatch_rx();
        gptp_poll(&gptp);
        avtp_poll(&avtp);
        aes3_poll(&aes3, &tx_audio_ring, &rx_audio_ring);
        srp_poll(&srp);
        avdecc_poll(&avdecc);
        mcr_servo_update(&mcr);
        // Audio routing: drain AAF RX (8ch INT_32BIT) into:
        //   - tx_audio_ring (channels 0+1, >>8 to 24-bit) → AES3 TX wire
        //   - aaf_tx_push (all 8ch) → AAF talker (loopback for diagnostics)
        {
            int32_t blk[AAF_CHANNELS];
            while (aaf_rx_pop(&aaf, blk)) {
                if (audio_ring_space(&tx_audio_ring) > 0)
                    audio_ring_write(&tx_audio_ring, blk[0] >> 8, blk[1] >> 8);
                aaf_tx_push(&aaf, blk);
            }
        }
        aaf_tx_poll(&aaf);
        check_uart_cmd();
    }

    return 0;
}
