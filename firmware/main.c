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
#include "avtp_const.h"
#include "srp.h"
#include "avdecc.h"
#include "mcr.h"
#include "aaf.h"

// MAC address — locally administered, unique per device.
// TODO: read from SPI flash or EEPROM in production.
static const uint8_t mac_addr[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x42};

static gptp_t gptp;
static srp_state_t srp;
static avdecc_state_t avdecc;
static mcr_state_t    mcr;
static aaf_state_t    aaf;

// Local stream UIDs (must match build_desc_* in avdecc.c)
#define LISTENER_UID_CRF  0
#define LISTENER_UID_AAF  1
#define TALKER_UID_AAF    0

// AVTP gateware filter slots — Stage 1. Slot 0 = CRF, slot 1 = AAF in.
#define AVTP_FILTER_SLOT_CRF  0
#define AVTP_FILTER_SLOT_AAF  1

static void avtp_filter_set_slot(int slot,
                                  const uint8_t *stream_id,
                                  const uint8_t *dest_mac)
{
    uint32_t sid_hi = ((uint32_t)stream_id[0] << 24) |
                      ((uint32_t)stream_id[1] << 16) |
                      ((uint32_t)stream_id[2] <<  8) |
                       (uint32_t)stream_id[3];
    uint32_t sid_lo = ((uint32_t)stream_id[4] << 24) |
                      ((uint32_t)stream_id[5] << 16) |
                      ((uint32_t)stream_id[6] <<  8) |
                       (uint32_t)stream_id[7];
    uint32_t mac_hi = ((uint32_t)dest_mac[0] <<  8) |  (uint32_t)dest_mac[1];
    uint32_t mac_lo = ((uint32_t)dest_mac[2] << 24) |
                      ((uint32_t)dest_mac[3] << 16) |
                      ((uint32_t)dest_mac[4] <<  8) |
                       (uint32_t)dest_mac[5];
    switch (slot) {
        case 0:
            avtp_filter_slot0_stream_id_hi_write(sid_hi);
            avtp_filter_slot0_stream_id_lo_write(sid_lo);
            avtp_filter_slot0_dst_mac_hi_write   (mac_hi);
            avtp_filter_slot0_dst_mac_lo_write   (mac_lo);
            avtp_filter_slot0_enabled_write      (1);
            break;
        case 1:
            avtp_filter_slot1_stream_id_hi_write(sid_hi);
            avtp_filter_slot1_stream_id_lo_write(sid_lo);
            avtp_filter_slot1_dst_mac_hi_write   (mac_hi);
            avtp_filter_slot1_dst_mac_lo_write   (mac_lo);
            avtp_filter_slot1_enabled_write      (1);
            break;
        case 2:
            avtp_filter_slot2_stream_id_hi_write(sid_hi);
            avtp_filter_slot2_stream_id_lo_write(sid_lo);
            avtp_filter_slot2_dst_mac_hi_write   (mac_hi);
            avtp_filter_slot2_dst_mac_lo_write   (mac_lo);
            avtp_filter_slot2_enabled_write      (1);
            break;
        case 3:
            avtp_filter_slot3_stream_id_hi_write(sid_hi);
            avtp_filter_slot3_stream_id_lo_write(sid_lo);
            avtp_filter_slot3_dst_mac_hi_write   (mac_hi);
            avtp_filter_slot3_dst_mac_lo_write   (mac_lo);
            avtp_filter_slot3_enabled_write      (1);
            break;
    }
}

static void avtp_filter_clear_slot(int slot)
{
    switch (slot) {
        case 0: avtp_filter_slot0_enabled_write(0); break;
        case 1: avtp_filter_slot1_enabled_write(0); break;
        case 2: avtp_filter_slot2_enabled_write(0); break;
        case 3: avtp_filter_slot3_enabled_write(0); break;
    }
}

static uint32_t avtp_filter_match_count(int slot)
{
    switch (slot) {
        case 0: return avtp_filter_slot0_match_count_read();
        case 1: return avtp_filter_slot1_match_count_read();
        case 2: return avtp_filter_slot2_match_count_read();
        case 3: return avtp_filter_slot3_match_count_read();
    }
    return 0;
}

// I2S DAC writer state — paced from mcr_sample_count which ticks at fs
// (48 kHz) once MCR is locked. Each tick we pop one block from AAF RX
// (channels 0+1) and write to the I2S TX CSRs.
static uint32_t dac_sample_count;
static uint32_t dac_underrun_count;
static uint32_t dac_last_sample_tick;

// RX EtherType counters for link-debug
static uint32_t rx_total, rx_ptp, rx_avtp, rx_msrp, rx_other;
static uint16_t rx_last_ethertype;
static uint8_t  rx_last_dst[6];
static uint8_t  rx_last_src[6];
// Pending FAST_CONNECT bindings — forward-declared here so the UART
// 'D' (force-clear) command can also clear them. Full definition is
// below near on_listener_connect.
typedef struct {
    uint8_t  active;
    uint8_t  uid;
    uint8_t  dest_mac[6];
    uint8_t  talker_entity_id[8];
    uint16_t talker_uid;
} pending_bind_t;
static pending_bind_t pending_listeners[2];

// AVB-stream multicast bypass counter — any frame with dst MAC prefix
// 91:e0:f0:* (the AVB stream multicast range). Independent of ethertype /
// VLAN / subtype demux, so it catches frames that would otherwise be
// classified as "other" or get lost in dispatch. If the bridge IS
// forwarding Auvitran's CRF stream to our port, this counter climbs
// regardless of whether mcr_process_rx() actually consumes them.
static uint32_t rx_avb_stream_mcast;
// Per-subtype counters for AVTP RX. Diagnoses "MCR rx=0 but stream_mcast>0"
// — distinguishes "no CRF frames arriving" from "CRF arriving but mcr
// rejects on stream_id mismatch". Sampled in the 's' status print.
static uint32_t rx_avtp_crf, rx_avtp_aaf;
static uint8_t  rx_crf_last_sid[8];   // last CRF stream_id we saw on the wire

// Benchmark / rate-window state for the 'b' UART command. Lets us
// compare Stage-0 (firmware-on-audio) numbers against each later
// migration stage to prove the gateware work paid off.
typedef struct {
    uint32_t window_start_ms;
    // Snapshots taken at window start.
    uint32_t s_writer_errors;
    uint32_t s_rx_avtp_aaf;
    uint32_t s_rx_avtp_crf;
    uint32_t s_dac_samples;
    uint32_t s_dac_underruns;
    uint32_t s_aecp_rx;
    uint32_t s_aecp_tx;
    uint32_t s_sync_rx;
    uint32_t s_pdresp_rx;
    // Main-loop iteration timing (in TSU ns; resolution = ns).
    uint32_t last_iter_ns;
    uint32_t max_iter_ns;
    uint64_t sum_iter_ns;
    uint32_t iter_count;
    // Last computed rates (per-second).
    uint32_t r_writer_errors;
    uint32_t r_aaf;
    uint32_t r_crf;
    uint32_t r_dac_samples;
    uint32_t r_dac_underruns;
    uint32_t r_aecp_rx;
    uint32_t r_aecp_tx;
    uint32_t r_sync_rx;
    uint32_t r_pdresp_rx;
    uint32_t r_iter_per_sec;
    uint32_t r_iter_avg_ns;
    uint32_t r_iter_max_ns;
} bench_t;
static bench_t bench;

// ---------------------------------------------------------------------------
// Central Ethernet RX dispatcher
// ---------------------------------------------------------------------------

#define ETHMAC_EV_SRAM_WRITER 0x1
#define PTP_ETHERTYPE   0x88F7
// AVTP_ETHERTYPE is 0x22F0, defined in avtp.h

static void dispatch_rx(void)
{
    // Drain pending RX slots in one dispatcher call (bounded). nrxslots=2
    // is pinned (>2 silently breaks TX — see avb_soc.py:537); under MSRP /
    // AVDECC / CRF bursts the writer overruns within microseconds if we
    // service only one slot per call. But we MUST cap the drain — an
    // unbounded loop locks the CPU when wire-rate stream traffic exceeds
    // processing rate, never returning to the main loop. Symptom: UART
    // stops, Hive loses entity, gPTP servo can't update. 16 frames is
    // roughly one Class A burst window; main_loop drives dispatch_rx
    // continuously so any residual frames are picked up next call.
    // Drain budget. 16 was OK with nrxslots=2 in light traffic, but with
    // AAF + CRF + AVDECC + gPTP arriving simultaneously the 2-slot FIFO
    // overruns producing 1000+ writer_errors/sec under Hive refresh
    // (READ_DESCRIPTOR burst overlapping AAF flow). 64 lets one
    // dispatch_rx call empty an entire bridge burst window without
    // returning to main_loop where slower paths (printf, MCR servo,
    // descriptor builds) could let more frames pile up.
    int drain_budget = 64;
    while ((ethmac_sram_writer_ev_pending_read() & ETHMAC_EV_SRAM_WRITER)
           && (drain_budget-- > 0)) {

    uint32_t slot = ethmac_sram_writer_slot_read();
    uint8_t *slot_ptr = (uint8_t *)(ETHMAC_BASE + ETHMAC_SLOT_SIZE * slot);
    uint32_t len = ethmac_sram_writer_length_read();

    // Advance the RX-timestamp ring in lock-step with slot consumption.
    main_rx_ts_pop_write(1);

    // Peek directly into the LiteEth slot to determine ethertype and
    // (for AVTP) subtype. Slot SRAM is read-only from CPU but READS
    // are fine. We use this to fast-path AVTP stream data (AAF, CRF —
    // 8k+1k fps) by passing the slot pointer (offset-shifted for VLAN)
    // straight to the handler. The previous full-frame scratch memcpy
    // was burning ~30 µs/frame × 9000 fps ≈ 27% CPU on slow Wishbone
    // reads of bytes we then re-read in the handlers.
    //
    // Everything else (gPTP, AVDECC, SRP) still gets the memcpy + VLAN
    // strip because AVDECC needs src MAC at frame[6..11] for response
    // routing and we can't shift the pointer without breaking that.
    uint32_t v = 0;
    if (len >= 18) {
        uint16_t et_pre = ((uint16_t)slot_ptr[12] << 8) | slot_ptr[13];
        if (et_pre == 0x8100) v = 4;
    }
    uint16_t et_peek = (len >= 14u + v)
        ? (((uint16_t)slot_ptr[12 + v] << 8) | slot_ptr[13 + v]) : 0;
    uint8_t  st_peek = (et_peek == AVTP_ETHERTYPE && len >= 15u + v)
        ? slot_ptr[14 + v] : 0xFF;
    int is_avtp_data = (st_peek == AVTP_SUBTYPE_AAF ||
                        st_peek == AVTP_SUBTYPE_CRF);

    const uint8_t *frame;
    if (is_avtp_data) {
        // Fast path. AAF/CRF handlers only read from frame+14 onward.
        frame = slot_ptr + v;
        len  -= v;
    } else {
        static uint8_t scratch[1600];
        if (len > sizeof(scratch)) len = sizeof(scratch);
        memcpy(scratch, slot_ptr, len);
        if (v) {
            memmove(scratch + 12, scratch + 16, len - 16);
            len -= 4;
        }
        frame = scratch;
    }

    if (len >= 14) {
        uint16_t ethertype = ((uint16_t)frame[12] << 8) | frame[13];

        rx_total++;
        rx_last_ethertype = ethertype;
        // Read MAC fields from slot_ptr (NOT frame) — frame may be the
        // VLAN-shifted slot pointer where [0..5]/[6..11] are inside the
        // tag, not the real MAC fields.
        memcpy(rx_last_dst, slot_ptr, 6);
        memcpy(rx_last_src, slot_ptr + 6, 6);

        if (slot_ptr[0] == 0x91 && slot_ptr[1] == 0xe0 &&
            slot_ptr[2] == 0xf0 && slot_ptr[3] == 0x00) {
            rx_avb_stream_mcast++;
        }

        // BISECT NOTE: the previous "fast-drop with early ack+continue"
        // path for MVRP/MMRP appears to break the slot-advance state
        // machine — avtp counter went to 0 even with ADP traffic on the
        // wire. The early ethmac_sram_writer_ev_pending_write() may have
        // interacted badly with the drain loop. Reverting to letting
        // MVRP/MMRP fall through to the `default` case (rx_other++)
        // which works with the standard ack-at-end-of-iteration flow.
        // If MVRP RX flood becomes a problem again, raise nrxslots
        // (gateware change) instead.

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
                            rx_avtp_crf++;
                            if (len >= 14 + 12)
                                memcpy(rx_crf_last_sid, frame + 14 + 4, 8);
                            mcr_process_rx(&mcr, frame, len);
                            break;
                        case AVTP_SUBTYPE_AAF:
                            rx_avtp_aaf++;
                            aaf_process_rx(&aaf, frame, len);
                            break;
                        default:
                            // 61883/IIDC and unknown subtypes — silently
                            // drop. The legacy firmware/avtp.c 61883 stub
                            // was moved to firmware/_avtp_backup/.
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

// Bench tick — called once per main-loop iteration. Tracks max/avg
// iteration time (in TSU ns) and rolls a 1-second rate window for
// the counters that matter when comparing this Stage-0 firmware-on-
// audio baseline against later gateware-offload stages.
static void bench_tick(void)
{
    uint32_t now_ns = tsu_nanoseconds_read();
    if (bench.last_iter_ns) {
        // Modular arithmetic: TSU ns wraps at 1e9, so a single iter
        // crossing the second boundary shows as a huge negative dt.
        // Cap to 0 in that case (this iter "took no time" by this
        // measure — acceptable for the bench).
        int32_t dt = (int32_t)(now_ns - bench.last_iter_ns);
        if (dt < 0) dt += 1000000000;
        if ((uint32_t)dt > bench.max_iter_ns) bench.max_iter_ns = dt;
        bench.sum_iter_ns += (uint32_t)dt;
        bench.iter_count++;
    }
    bench.last_iter_ns = now_ns;

    uint32_t now_ms = gptp_uptime_ms();
    uint32_t elapsed = now_ms - bench.window_start_ms;
    if (elapsed > 2000000000u) elapsed = 0;   // wrap guard
    if (elapsed < 1000) return;

    bench.r_writer_errors   = ethmac_sram_writer_errors_read() - bench.s_writer_errors;
    bench.r_aaf             = rx_avtp_aaf  - bench.s_rx_avtp_aaf;
    bench.r_crf             = rx_avtp_crf  - bench.s_rx_avtp_crf;
    bench.r_dac_samples     = dac_sample_count   - bench.s_dac_samples;
    bench.r_dac_underruns   = dac_underrun_count - bench.s_dac_underruns;
    bench.r_aecp_rx         = avdecc.aecp_rx_count - bench.s_aecp_rx;
    bench.r_aecp_tx         = avdecc.aecp_tx_count - bench.s_aecp_tx;
    bench.r_sync_rx         = gptp.rx_sync_count - bench.s_sync_rx;
    bench.r_pdresp_rx       = gptp.rx_pdelay_resp_count - bench.s_pdresp_rx;
    bench.r_iter_per_sec    = bench.iter_count;
    bench.r_iter_max_ns     = bench.max_iter_ns;
    bench.r_iter_avg_ns     = bench.iter_count
                                ? (uint32_t)(bench.sum_iter_ns / bench.iter_count) : 0;

    // Re-snapshot for next window.
    bench.s_writer_errors  = ethmac_sram_writer_errors_read();
    bench.s_rx_avtp_aaf    = rx_avtp_aaf;
    bench.s_rx_avtp_crf    = rx_avtp_crf;
    bench.s_dac_samples    = dac_sample_count;
    bench.s_dac_underruns  = dac_underrun_count;
    bench.s_aecp_rx        = avdecc.aecp_rx_count;
    bench.s_aecp_tx        = avdecc.aecp_tx_count;
    bench.s_sync_rx        = gptp.rx_sync_count;
    bench.s_pdresp_rx      = gptp.rx_pdelay_resp_count;
    bench.window_start_ms  = now_ms;
    bench.iter_count       = 0;
    bench.sum_iter_ns      = 0;
    bench.max_iter_ns      = 0;
}

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
            printf("  30s avg: off=%lld ns |off|=%lld ns (n=%lu)\n",
                   (long long)gptp.off_avg_ns_last,
                   (long long)gptp.off_abs_avg_ns_last,
                   (unsigned long)gptp.off_avg_count_last);
            printf("  nrr=%ld ppb pdelay_outliers=%lu pairs=%lu\n",
                   (long)gptp.nrr_ppb,
                   (unsigned long)gptp.pdelay_outlier_count,
                   (unsigned long)gptp.pdelay_pair_count);
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
            printf("[DAC ] samples=%lu underruns=%lu\n",
                   (unsigned long)dac_sample_count,
                   (unsigned long)dac_underrun_count);
            printf("[SRP] tx=%lu rx=%lu domain=%d talker_reg=%d bridge_class=%u prio=%u vid=%u talker_prio_byte=0x%02x\n",
                   (unsigned long)srp.join_count,
                   (unsigned long)srp.rx_pdu_count,
                   srp.domain_received,
                   srp.talker_registered,
                   srp.rx_sr_class, srp.rx_sr_prio, srp.rx_sr_vid,
                   srp.talker.priority_and_rank);
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
            printf("\n[RX] total=%u ptp=%u avtp=%u msrp=%u other=%u stream_mcast=%u\n",
                   rx_total, rx_ptp, rx_avtp, rx_msrp, rx_other, rx_avb_stream_mcast);
            printf("  avtp subtypes: crf=%u aaf=%u (mcr.rx_other=%lu)\n",
                   rx_avtp_crf, rx_avtp_aaf, (unsigned long)mcr.rx_other_count);
            printf("  last CRF sid=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
                   rx_crf_last_sid[0], rx_crf_last_sid[1], rx_crf_last_sid[2],
                   rx_crf_last_sid[3], rx_crf_last_sid[4], rx_crf_last_sid[5],
                   rx_crf_last_sid[6], rx_crf_last_sid[7]);
            printf("  mcr bound sid=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
                   mcr.stream_id[0], mcr.stream_id[1], mcr.stream_id[2],
                   mcr.stream_id[3], mcr.stream_id[4], mcr.stream_id[5],
                   mcr.stream_id[6], mcr.stream_id[7]);
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
            printf("  delta stats (n=%lu): max|d|=%ld ns avg|d|=%ld ns streak=%u\n",
                   (unsigned long)mcr.delta_window_count,
                   (long)mcr.delta_max_abs,
                   (long)(mcr.delta_window_count ? mcr.delta_sum_abs / mcr.delta_window_count : 0),
                   mcr.lock_streak);
            // Reset window after print so next sample starts fresh
            mcr.delta_max_abs     = 0;
            mcr.delta_sum_abs     = 0;
            mcr.delta_window_count = 0;
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
        case 't': {
            // Diagnostic: force-enable AAF TX without waiting for Hive
            // CONNECT_TX_COMMAND. Sets a hardcoded dest_mac + stream_id
            // so the talker emits VLAN-tagged Class A AAF frames we can
            // catch with tcpdump on Linux. For wire-level TX path test.
            static const uint8_t dummy_dest[6] = {0x91, 0xe0, 0xf0, 0x00, 0xfe, 0x42};
            uint8_t dummy_sid[8];
            memcpy(dummy_sid, mac_addr, 6);
            dummy_sid[6] = 0x00; dummy_sid[7] = 0x01;
            aaf_bind(&aaf, dummy_sid);
            memcpy(aaf.dest_mac, dummy_dest, 6);
            aaf_tx_enable(&aaf, 1);
            srp_talker_set(&srp, dummy_sid, dummy_dest,
                           1500 /* max_frame_size */);
            srp_talker_enable(&srp, 1);
            printf("[DIAG] AAF TX force-enabled → dest=91:e0:f0:00:fe:42 "
                   "sid=%02x:%02x:%02x:%02x:%02x:%02x:00:01\n",
                   mac_addr[0], mac_addr[1], mac_addr[2],
                   mac_addr[3], mac_addr[4], mac_addr[5]);
            break;
        }
        case 'T':
            aaf_tx_enable(&aaf, 0);
            srp_talker_enable(&srp, 0);
            printf("[DIAG] AAF TX force-disabled\n");
            break;
        case 'b':
            // Per-second windowed rates — Stage-0 baseline for the
            // firmware-on-audio architecture. Each subsequent
            // gateware-offload stage should drive aaf/writer_errors
            // toward 0 and iter_per_sec up.
            printf("\n[BENCH] 1s window rates\n"
                   "  main_loop: %lu iter/s  avg %lu ns  max %lu ns\n"
                   "  RX:        %lu writer_err/s  %lu aaf/s  %lu crf/s\n"
                   "  AVDECC:    %lu aecp_rx/s  %lu aecp_tx/s\n"
                   "  gPTP:      %lu sync_rx/s  %lu pdresp_rx/s\n"
                   "  DAC:       %lu samples/s  %lu underruns/s\n"
                   "  HW filter: slot0=%lu slot1=%lu slot2=%lu slot3=%lu  (cumulative)\n",
                   (unsigned long)bench.r_iter_per_sec,
                   (unsigned long)bench.r_iter_avg_ns,
                   (unsigned long)bench.r_iter_max_ns,
                   (unsigned long)bench.r_writer_errors,
                   (unsigned long)bench.r_aaf,
                   (unsigned long)bench.r_crf,
                   (unsigned long)bench.r_aecp_rx,
                   (unsigned long)bench.r_aecp_tx,
                   (unsigned long)bench.r_sync_rx,
                   (unsigned long)bench.r_pdresp_rx,
                   (unsigned long)bench.r_dac_samples,
                   (unsigned long)bench.r_dac_underruns,
                   (unsigned long)avtp_filter_match_count(0),
                   (unsigned long)avtp_filter_match_count(1),
                   (unsigned long)avtp_filter_match_count(2),
                   (unsigned long)avtp_filter_match_count(3));
            break;
        case 'D':
            // Force-clear local listener bindings — useful when Hive's
            // saved state replays a stale FAST_CONNECT to a stream_id
            // that no longer exists on the network. Sends MSRP Listener
            // Leave for whatever we were registered to, then unbinds
            // mcr+aaf and clears the AVDECC listener slots so the next
            // Hive CONNECT_RX rebinds fresh.
            if (mcr.bound) {
                srp_listener_enable(&srp, mcr.stream_id, 0);
                mcr_unbind(&mcr);
            }
            if (aaf.bound) {
                srp_listener_enable(&srp, aaf.stream_id, 0);
                aaf_unbind(&aaf);
            }
            memset(&avdecc.listeners[LISTENER_UID_CRF], 0,
                   sizeof(avdecc.listeners[LISTENER_UID_CRF]));
            memset(&avdecc.listeners[LISTENER_UID_AAF], 0,
                   sizeof(avdecc.listeners[LISTENER_UID_AAF]));
            for (size_t i = 0; i < sizeof(pending_listeners)/sizeof(pending_listeners[0]); i++)
                pending_listeners[i].active = 0;
            printf("[DIAG] Listener bindings force-cleared (mcr + aaf + avdecc slots)\n");
            break;
        case 'h':
        case '?':
            printf("\n  s   status (gPTP / AVTP / DAC / SRP / AVDECC / I2S)\n"
                     "  m   MCR servo state (CRF lock, NCO increment, offset)\n"
                     "  a   AAF stream state (RX/TX counts, jitter buffer level)\n"
                     "  e   RX ethertype counters + LiteEth heartbeat\n"
                     "  b   1-second rate window (Stage-0 baseline metrics)\n"
                     "  t   force-enable AAF TX (diagnostic, bypasses AVDECC)\n"
                     "  T   force-disable AAF TX\n"
                     "  D   force-clear all listener bindings (clears stale FAST_CONNECT)\n"
                     "  r   reboot\n"
                     "  h   help\n");
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

// pending_bind_t + pending_listeners[] are forward-declared at top of file.

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
        if (p->uid == LISTENER_UID_CRF) {
            mcr_bind(&mcr, stream_id);
            avtp_filter_set_slot(AVTP_FILTER_SLOT_CRF, stream_id, p->dest_mac);
        } else if (p->uid == LISTENER_UID_AAF) {
            aaf_bind(&aaf, stream_id);
            aaf.rx_audio_capture = 1;
            avtp_filter_set_slot(AVTP_FILTER_SLOT_AAF, stream_id, p->dest_mac);
        }

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
    if (uid == LISTENER_UID_CRF) {
        mcr_bind(&mcr, stream_id);
        avtp_filter_set_slot(AVTP_FILTER_SLOT_CRF, stream_id, dest_mac);
    } else if (uid == LISTENER_UID_AAF) {
        aaf_bind(&aaf, stream_id);
        // Real consumer now waiting (I2S DAC) — turn on the per-packet
        // audio copy in aaf_process_rx. Without this, rx_buf stays
        // empty and the DAC writer would only emit underruns.
        aaf.rx_audio_capture = 1;
        avtp_filter_set_slot(AVTP_FILTER_SLOT_AAF, stream_id, dest_mac);
    }
}

static void on_listener_disconnect(uint16_t uid)
{
    if (uid < (uint16_t)(sizeof(pending_listeners)/sizeof(pending_listeners[0])))
        pending_listeners[uid].active = 0;

    if (uid == LISTENER_UID_CRF) {
        if (mcr.bound) srp_listener_enable(&srp, mcr.stream_id, 0);
        mcr_unbind(&mcr);
        avtp_filter_clear_slot(AVTP_FILTER_SLOT_CRF);
    } else if (uid == LISTENER_UID_AAF) {
        if (aaf.bound) srp_listener_enable(&srp, aaf.stream_id, 0);
        aaf_unbind(&aaf);
        aaf.rx_audio_capture = 0;
        avtp_filter_clear_slot(AVTP_FILTER_SLOT_AAF);
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

    // Init protocol stacks
    gptp_init(&gptp, mac_addr);
    srp_init(&srp, mac_addr);
    mcr_init(&mcr, CONFIG_CLOCK_FREQUENCY, 48000);
    {
        // AAF uses the same talker stream_id/dest_mac advertised in AVDECC.
        // stream_id = MAC + 0x00 0x01 (matches avtp_set_stream_id default).
        uint8_t aaf_stream_id[8] = {0,0,0,0,0,0, 0x00, 0x01};
        memcpy(aaf_stream_id, mac_addr, 6);
        // Derive dest_mac from MAC bytes 4..5 so we don't collide with another
        // AVB talker on the same network. A hardcoded 91:E0:F0:00:FE:00 makes
        // every FPGA build advertise the same address — bridges respond with
        // MSRP Failure 0x05 (Stream Destination Address In Use) the moment a
        // second talker shows up. The 91:E0:F0:00:FE:xx slice is the Milan
        // locally-administered range (IEEE 1722-2016 Annex B.1).
        uint8_t aaf_mcast[6] = {0x91, 0xE0, 0xF0, 0x00, 0xFE, mac_addr[5]};
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
    avdecc_set_srp(&srp);     // GET_AVB_INFO emits matching msrp_mapping
    avdecc_set_talker_stream(&avdecc, TALKER_UID_AAF, aaf.stream_id, aaf.dest_mac);
    avdecc.on_talker_connect    = on_talker_connect;
    avdecc.on_talker_disconnect = on_talker_disconnect;
    avdecc.on_listener_connect  = on_listener_connect;
    avdecc.on_listener_disconnect = on_listener_disconnect;

    printf("[main] Press 'h' for commands.\n\n");

    while (1) {
        bench_tick();
        dispatch_rx();
        gptp_poll(&gptp);
        srp_poll(&srp);

        // Track per-stream MEDIA_LOCKED for Hive's listener indicators.
        // CRF (uid 0) follows the MCR servo; AAF (uid 1) follows
        // aaf.rx_count crossing zero. Both call into avdecc, which
        // pushes unsolicited GET_COUNTERS_RESPONSE on transitions.
        {
            static uint8_t prev_aaf_locked = 0;
            uint8_t aaf_now = (aaf.bound && aaf.rx_count > 0) ? 1 : 0;
            if (aaf_now != prev_aaf_locked) {
                avdecc_listener_lock_changed(&avdecc, LISTENER_UID_AAF, aaf_now);
                prev_aaf_locked = aaf_now;
            }
            avdecc_listener_lock_changed(&avdecc, LISTENER_UID_CRF,
                                          mcr.servo_locked ? 1 : 0);

            // FRAMES_RX is a polled counter — mirror the underlying
            // counter (no per-frame hook needed).
            avdecc.stream_frames_rx[LISTENER_UID_CRF] = mcr.rx_count;
            avdecc.stream_frames_rx[LISTENER_UID_AAF] = aaf.rx_count;
        }

        // Sync AAF VLAN-PCP/VID with bridge-advertised SR domain on every
        // change (no-op once converged). If we keep emitting PCP=3/VID=2
        // while the bridge maps Class A to a different priority, listeners
        // (Auvitran) reject with MSRP failure 0x13 (SR Class Priority
        // Mismatch). Skip until we've heard a Domain to avoid bouncing.
        if (srp.domain_received) {
            static uint8_t  last_prio = 0xFF;
            static uint16_t last_vid  = 0xFFFF;
            if (srp.rx_sr_prio != last_prio || srp.rx_sr_vid != last_vid) {
                aaf_set_vlan(&aaf, srp.rx_sr_prio, srp.rx_sr_vid);
                last_prio = srp.rx_sr_prio;
                last_vid  = srp.rx_sr_vid;
                printf("[main] AAF VLAN sync: PCP=%u VID=%u\n",
                       (unsigned)srp.rx_sr_prio, (unsigned)srp.rx_sr_vid);
            }
        }
        avdecc_poll(&avdecc);
        mcr_servo_update(&mcr);

        // DAC writer: pace AAF RX ch 0/1 → I2S TX at the audio sample
        // rate. mcr_sample_count ticks at fs (48 kHz) — driven by the
        // MCR NCO. Each main-loop visit, drain ALL samples up to the
        // current tick (not just one) — otherwise when main_loop runs
        // slower than 48 kHz under AAF flood (~800 Hz observed), we
        // skip ~98% of samples → audio sounds aliased / synth-like.
        // I2S TX latches whatever's in i2s_audio_l/r at frame_start
        // (48 kHz), so writes faster than that get sample-and-hold'd
        // by the gateware — only the last write per audio-frame
        // actually goes out the wire, which is exactly what we want.
        {
            uint32_t tick = mcr_sample_count_read();
            uint32_t ticks_due = tick - dac_last_sample_tick;
            // Cap per-call drain to bound CSR write time, but ADVANCE
            // dac_last_sample_tick by what we actually processed —
            // NOT to `tick` directly. Otherwise capping = silently
            // skipping audio samples (the "underrun" counter actually
            // reflects samples we threw away because of the cap).
            // 32 samples ≈ 0.16 ms of CSR work, fine between
            // dispatch_rx visits now that the AAF fast-path skips
            // the scratch memcpy.
            if (ticks_due > 32) ticks_due = 32;
            uint32_t processed = ticks_due;
            while (ticks_due--) {
                int32_t sl = 0, sr = 0;
                int32_t blk[AAF_CHANNELS];
                if (aaf.bound && aaf.rx_audio_capture &&
                    aaf_rx_pop(&aaf, blk)) {
                    sl = blk[0] >> 8;
                    sr = blk[1] >> 8;
                    dac_sample_count++;
                } else {
                    dac_underrun_count++;
                }
                main_i2s_audio_l_write((uint32_t)sl & 0xFFFFFFu);
                main_i2s_audio_r_write((uint32_t)sr & 0xFFFFFFu);
                main_i2s_push_write(1);
            }
            dac_last_sample_tick += processed;
        }
        aaf_tx_poll(&aaf);
        check_uart_cmd();
    }

    return 0;
}
