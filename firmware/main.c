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

// MAC address — locally administered, unique per device.
// TODO: read from SPI flash or EEPROM in production.
static const uint8_t mac_addr[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

static gptp_t gptp;
static avtp_stream_t avtp;
static aes3_state_t aes3;
static srp_state_t srp;
static avdecc_state_t avdecc;

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
    if (!(ethmac_sram_writer_ev_pending_read() & ETHMAC_EV_SRAM_WRITER))
        return;

    uint32_t slot = ethmac_sram_writer_slot_read();
    uint8_t *frame = (uint8_t *)(ETHMAC_BASE + ETHMAC_SLOT_SIZE * slot);
    uint32_t len = ethmac_sram_writer_length_read();

    // Route by ethertype
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
                // AVTP and AVDECC share EtherType 0x22F0.
                // Peek at subtype byte to distinguish.
                if (len >= 15) {
                    uint8_t subtype = frame[14];
                    if (subtype == AVTP_SUBTYPE_ADP ||
                        subtype == AVTP_SUBTYPE_AECP ||
                        subtype == AVTP_SUBTYPE_ACMP) {
                        avdecc_process_rx(&avdecc, frame, len);
                    } else {
                        avtp_process_rx(&avtp, frame, len);
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

    // Acknowledge RX event
    ethmac_sram_writer_ev_pending_write(ETHMAC_EV_SRAM_WRITER);
}

// ---------------------------------------------------------------------------
// I2S DAC test tone (1 kHz sine, 48 kHz sample rate)
// ---------------------------------------------------------------------------

static uint8_t dac_test_active = 0;
static uint16_t dac_test_phase = 0;

// Quarter-wave sine table, 12 entries for 1 kHz at 48 kHz (48 samples/cycle).
// 24-bit signed max = 0x7FFFFF = 8388607. Quarter wave = 12 samples.
static const int32_t sine_q12[13] = {
    0, 1084752, 2139093, 3133494, 4041456, 4839708,
    5509524, 6036744, 6412260, 6632460, 6698916, 6617328, 6397536
};

static int32_t sine_lookup(uint16_t idx)
{
    // idx 0..47 → full sine via quarter-wave symmetry
    if (idx < 12)      return  sine_q12[idx];
    else if (idx < 24) return  sine_q12[24 - idx];
    else if (idx < 36) return -sine_q12[idx - 24];
    else               return -sine_q12[48 - idx];
}

static void dac_test_poll(void)
{
    if (!dac_test_active)
        return;

    // Phase advance must be locked to the actual LRCK frames the hardware
    // emits, not to main-loop iteration cadence. The hardware exposes a
    // free-running 48 kHz frame counter (incremented once per LRCK period in
    // the audio domain, CDC'd to sys); diff it against the last reading and
    // step phase by that many frames. No frequency error, no FM jitter.
    static uint32_t last_fs = 0;
    uint32_t fs = audio_clk_fs_48k_count_read();
    uint32_t frames = fs - last_fs;  // wraps mod 2^32
    if (frames == 0)
        return;
    last_fs = fs;
    dac_test_phase = (dac_test_phase + frames) % 48;

    main_i2s_source_write(1);

    int32_t sample = sine_lookup(dac_test_phase);
    main_i2s_audio_l_write(sample & 0xFFFFFF);
    main_i2s_audio_r_write(sample & 0xFFFFFF);
    main_i2s_push_write(1);
}

// ---------------------------------------------------------------------------
// MDIO probe — uses libliteeth's mdio_read (Clause 22 over the bit-banged
// MDC/MDIO CSRs). Scan all 32 PHY addresses; print any that respond with a
// plausible ID along with link status.
// ---------------------------------------------------------------------------

// Dump key registers of PHY at addr 1 (the live one) for B50612D RGMII diag.
static void phy_dump(void) {
    int a = 1;
    printf("\n[PHY dump addr=%d]\n", a);
    for (int r = 0; r <= 31; r++) {
        int v = mdio_read(a, r);
        printf("  r%2d = %04x%s", r, v, (r%4==3)?"\n":"  ");
    }
    if ((31 % 4) != 3) printf("\n");
    // Read shadow register 0x07 (Misc Control) of reg 0x18:
    // "read shadow": write 0x18 with bits[14:12]=0b111, bit15=0, bits[11:0]=0
    // Then read reg 0x18 to get shadow data.
    mdio_write(a, 0x18, 0x7000);
    int sh7 = mdio_read(a, 0x18);
    printf("  shadow_07 (Misc Ctrl) = %04x [bit8=RXC_skew bit9=OOBS]\n", sh7);
}

// Soft-reset the PHY at addr 1 via BMCR bit 15. Then re-enable autoneg.
static void phy_reset(void) {
    int a = 1;
    printf("\n[PHY soft reset addr=%d]\n", a);
    mdio_write(a, 0, 0x9140);  // reset + autoneg + 1000Mbps + FD
    busy_wait(100);
    int bmcr = mdio_read(a, 0);
    int bmsr = mdio_read(a, 1);
    printf("  after reset: bmcr=%04x bmsr=%04x\n", bmcr, bmsr);
}

// Send a single broadcast test frame and report TX done event timing.
// Note: ethmac_sram_reader_ready reflects the cmd-FIFO accepting more
// commands (4-deep FIFO), NOT whether TX completed. The proper TX-done
// indicator is the ev.done event, exposed via ev_pending (write 1 to
// clear). After start, ev_pending should go to 1 once the frame has
// been clocked out of the MAC.
static void tx_test(void)
{
    printf("\n[TX test]\n");

    // Clear any stale done event.
    ethmac_sram_reader_ev_pending_write(0xFFFFFFFFu);

    uint32_t pre_ready = ethmac_sram_reader_ready_read();
    uint32_t pre_level = ethmac_sram_reader_level_read();
    uint32_t pre_evp   = ethmac_sram_reader_ev_pending_read();
    uint32_t pre_evs   = ethmac_sram_reader_ev_status_read();
    printf("  pre:  ready=%lu level=%lu ev_pend=%lu ev_stat=%lu\n",
           (unsigned long)pre_ready, (unsigned long)pre_level,
           (unsigned long)pre_evp, (unsigned long)pre_evs);

    // Build test frame in TX slot 0.
    uint8_t *frame = (uint8_t *)(ETHMAC_BASE +
                                 ETHMAC_SLOT_SIZE * (ETHMAC_RX_SLOTS + 0));
    for (int i = 0; i < 6; i++) frame[i] = 0xFF;
    frame[6]  = 0x02; frame[7]  = 0x00; frame[8]  = 0x00;
    frame[9]  = 0x00; frame[10] = 0x00; frame[11] = 0x01;
    frame[12] = 0x88; frame[13] = 0xB5;
    const char *msg = "FPGAtest!";
    int n = 0;
    while (msg[n]) { frame[14 + n] = msg[n]; n++; }
    for (int i = 14 + n; i < 64; i++) frame[i] = 0;

    ethmac_sram_reader_slot_write(0);
    ethmac_sram_reader_length_write(64);
    ethmac_sram_reader_start_write(1);

    // Sample ev_pending (TX done) at various intervals.
    uint32_t e0 = ethmac_sram_reader_ev_pending_read();
    busy_wait_us(2);
    uint32_t e1 = ethmac_sram_reader_ev_pending_read();
    busy_wait_us(10);
    uint32_t e2 = ethmac_sram_reader_ev_pending_read();
    busy_wait_us(100);
    uint32_t e3 = ethmac_sram_reader_ev_pending_read();
    busy_wait(1);
    uint32_t e4 = ethmac_sram_reader_ev_pending_read();
    busy_wait(10);
    uint32_t e5 = ethmac_sram_reader_ev_pending_read();

    uint32_t post_ready = ethmac_sram_reader_ready_read();
    uint32_t post_level = ethmac_sram_reader_level_read();
    printf("  ev_pending samples: @0us=%lu @2us=%lu @12us=%lu @112us=%lu @1ms=%lu @11ms=%lu\n",
           (unsigned long)e0, (unsigned long)e1, (unsigned long)e2,
           (unsigned long)e3, (unsigned long)e4, (unsigned long)e5);
    printf("  post: ready=%lu level=%lu\n",
           (unsigned long)post_ready, (unsigned long)post_level);

    if (e5 == 0) {
        printf("  *** TX never completed (ev.done never fired) ***\n");
        printf("  -> reader FSM stuck, or cmd FIFO not draining to PHY\n");
    } else {
        printf("  TX done event fired -> MAC streamed frame to PHY\n");
        ethmac_sram_reader_ev_pending_write(0xFFFFFFFFu);  // clear
    }
}

// Configure the B50612D PHY for full RGMII-ID mode:
//   - RX skew: reg 0x18 shadow 7 (Misc Ctrl) bit 8 = 1   → write 0xF1E7
//   - TX skew: reg 0x1C shadow 3 (Clk Ctl)   bit 9 = 1   → write 0x8E00
// Both PHY-side delays restore the "TX works" state we had this morning
// when frames were reaching the controller. Combined with FPGA tx_delay=0,
// rx_delay=0 in the gateware (no double-shift). Note: shadow registers
// persist across bitstream reloads (only power-cycle or HW reset on
// pads.rst_n clears them), so we must re-write them on every boot.
// Reference: Linux drivers/net/phy/broadcom.c BCM54xx clock-delay setup.
static void phy_enable_rxc_skew(void) {
    int a = 1;
    mdio_write(a, 0x18, 0xF1E7);          // shadow_07 bit 8 = 1 (RXC delay)
    busy_wait(10);
    mdio_write(a, 0x18, 0x7000);
    int sh7 = mdio_read(a, 0x18);
    mdio_write(a, 0x1C, 0x8E00);          // shadow_03 bit 9 = 1 (TXC delay)
    busy_wait(10);
    mdio_write(a, 0x1C, 0x0C00);
    int sh3 = mdio_read(a, 0x1C);
    printf("\n[PHY skew] shadow_07=%04x shadow_03=%04x (full RGMII-ID)\n",
           sh7, sh3);
}

static void phy_probe(void) {
    printf("\n[MDIO probe]\n");
    for (int addr = 0; addr < 32; addr++) {
        int id1 = mdio_read(addr, 2);
        int id2 = mdio_read(addr, 3);
        if (id1 != 0xFFFF && id1 != 0x0000) {
            // BMSR bit 2 is latched-low; read twice to see current state.
            (void)mdio_read(addr, 1);
            int bmsr = mdio_read(addr, 1);
            int bmcr = mdio_read(addr, 0);
            int lpa  = mdio_read(addr, 5);   // 10/100 link partner ability
            int s1k  = mdio_read(addr, 10);  // 1000Base-T status
            int aux  = mdio_read(addr, 25);  // B50612D Aux Status Summary
            int hcd  = (aux >> 8) & 0x7;
            const char *spd = "?";
            switch (hcd) {
                case 0: spd = "none";       break;
                case 1: spd = "10-HD";      break;
                case 2: spd = "10-FD";      break;
                case 3: spd = "100-HD";     break;
                case 4: spd = "100-T4";     break;
                case 5: spd = "100-FD";     break;
                case 6: spd = "1000-HD";    break;
                case 7: spd = "1000-FD";    break;
            }
            printf("  a=%2d id=%04x:%04x bmcr=%04x bmsr=%04x lpa=%04x s1k=%04x aux=%04x lk=%d an=%d hcd=%s\n",
                   addr, id1, id2, bmcr, bmsr, lpa, s1k, aux,
                   (bmsr >> 2) & 1, (bmsr >> 5) & 1, spd);
        }
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
            printf("  addend=%lu locked=%d\n",
                   (unsigned long)gptp.current_addend,
                   gptp.servo_locked);
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
            printf("[AVDECC] adp=%lu acmp=%lu/%lu aecp=%lu/%lu t=%d l=%d\n",
                   (unsigned long)avdecc.adp_tx_count,
                   (unsigned long)avdecc.acmp_rx_count,
                   (unsigned long)avdecc.acmp_tx_count,
                   (unsigned long)avdecc.aecp_rx_count,
                   (unsigned long)avdecc.aecp_tx_count,
                   avdecc.talker_connected,
                   avdecc.listener_connected);
            printf("[I2S] %lu %lu %lu\n",
                   (unsigned long)main_i2s_mmcm_locked_read(),
                   (unsigned long)main_i2s_bck_count_read(),
                   (unsigned long)main_i2s_lrck_count_read());
            break;
        }
        case 't':
            // Toggle talker (AVTP + SRP)
            avtp_tx_enable(&avtp, !avtp.tx_enabled);
            if (avtp.tx_enabled)
                srp_talker_enable(&srp, 1);
            else
                srp_talker_enable(&srp, 0);
            break;
        case 'l':
            // Toggle listener (listen to our own stream ID for loopback test)
            avtp_set_listen_stream_id(&avtp, avtp.stream_id);
            avtp_rx_enable(&avtp, !avtp.rx_enabled);
            if (avtp.rx_enabled)
                srp_listener_enable(&srp, avtp.stream_id, 1);
            else
                srp_listener_enable(&srp, avtp.stream_id, 0);
            break;
        case 'd':
            dac_test_active = dac_test_active ? 0 : 1;
            if (dac_test_active) {
                dac_test_phase = 0;
                printf("\n[DAC] sine\n");
            } else {
                main_i2s_source_write(0);
                printf("\n[DAC] off\n");
            }
            break;
        case 'r':
            printf("\nRebooting...\n");
            ctrl_reset_write(1);
            break;
        case 'p':
            phy_probe();
            break;
        case 'P':
            phy_dump();
            break;
        case 'R':
            phy_reset();
            break;
        case 'k':
            phy_enable_rxc_skew();
            break;
        case 'T':
            tx_test();
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
            {
                uint8_t hba = main_eth_rx_heartbeat_read();
                busy_wait(300);  // ≥1 top-byte tick at 125e6/2^24 ≈ 7.5 Hz
                uint8_t hbb = main_eth_rx_heartbeat_read();
                int d = (int)((uint8_t)(hbb - hba));
                printf("  eth_rx heartbeat (PHY1 L3): %u -> %u (delta=%d) %s\n",
                       hba, hbb, d, d ? "alive" : "*** DEAD ***");
            }
            break;
        case 'h':
        case '?':
            printf("\ns t l d p P R k e r\n  P=phy dump  R=phy reset  k=RXC skew\n");
            break;
    }
}

// ---------------------------------------------------------------------------
// AVDECC callbacks — wire connection management to AVTP/SRP
// ---------------------------------------------------------------------------

static void on_talker_connect(const uint8_t *listener_entity_id)
{
    (void)listener_entity_id;
    avtp_tx_enable(&avtp, 1);
    srp_talker_enable(&srp, 1);
    printf("[main] Talker stream started via AVDECC\n");
}

static void on_talker_disconnect(void)
{
    avtp_tx_enable(&avtp, 0);
    srp_talker_enable(&srp, 0);
    printf("[main] Talker stream stopped via AVDECC\n");
}

static void on_listener_connect(const uint8_t *stream_id, const uint8_t *dest_mac,
                                const uint8_t *talker_entity_id)
{
    (void)dest_mac;
    (void)talker_entity_id;
    avtp_set_listen_stream_id(&avtp, stream_id);
    avtp_rx_enable(&avtp, 1);
    srp_listener_enable(&srp, stream_id, 1);
    printf("[main] Listener started via AVDECC\n");
}

static void on_listener_disconnect(void)
{
    avtp_rx_enable(&avtp, 0);
    srp_listener_enable(&srp, avtp.stream_id, 0);
    printf("[main] Listener stopped via AVDECC\n");
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

    // BASELINE TEST: NO PHY MDIO writes at all. PHY runs at factory
    // defaults set by board strap pins. Goal: reproduce yesterday's
    // working TX config (frames reaching network, marginal but real)
    // before adding fixes back in one at a time. RX will be broken
    // (preamble errors) — that's expected; we'll fix it once TX works.
    busy_wait(100);  // Wait for hw_reset to release

    // Init audio ring buffers
    memset(&tx_audio_ring, 0, sizeof(tx_audio_ring));
    memset(&rx_audio_ring, 0, sizeof(rx_audio_ring));

    // Init protocol stacks
    gptp_init(&gptp, mac_addr);
    avtp_init(&avtp, mac_addr, &tx_audio_ring, &rx_audio_ring);
    aes3_init(&aes3);
    srp_init(&srp, mac_addr);

    // Configure SRP talker parameters to match our AVTP stream
    {
        static const uint8_t avtp_mcast[] = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x00};
        srp_talker_set(&srp, avtp.stream_id, avtp_mcast, AVTP_FRAME_LEN);
    }

    // Init AVDECC (discovery + connection management)
    {
        static const uint8_t avtp_mcast[] = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x00};
        avdecc_init(&avdecc, mac_addr, avtp.stream_id, avtp_mcast);
        avdecc.on_talker_connect    = on_talker_connect;
        avdecc.on_talker_disconnect = on_talker_disconnect;
        avdecc.on_listener_connect  = on_listener_connect;
        avdecc.on_listener_disconnect = on_listener_disconnect;
    }

    printf("[main] Press 'h' for commands.\n\n");

    while (1) {
        dispatch_rx();
        gptp_poll(&gptp);
        avtp_poll(&avtp);
        aes3_poll(&aes3, &tx_audio_ring, &rx_audio_ring);
        srp_poll(&srp);
        avdecc_poll(&avdecc);
        dac_test_poll();
        check_uart_cmd();
    }

    return 0;
}
