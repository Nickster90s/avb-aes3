`default_nettype none
`timescale 1ns / 1ps

module aes3_tb;

    // Clock generation — 100 MHz
    reg clk = 0;
    always #5 clk = ~clk;  // 10 ns period = 100 MHz

    reg rst;

    // TX signals
    reg  [23:0] tx_audio_l;
    reg  [23:0] tx_audio_r;
    reg         tx_audio_valid;
    wire        tx_audio_ready;
    wire        aes3_wire;

    // RX signals
    wire [23:0] rx_audio_l;
    wire [23:0] rx_audio_r;
    wire        rx_audio_valid;
    wire [23:0] rx_audio_sub;
    wire        rx_sub_valid;
    wire [191:0] rx_channel_status;
    wire        rx_cs_valid;
    wire        rx_locked;
    wire        rx_is_96k;
    wire [3:0]  rx_error_count;

    // Channel status — professional format, 48 kHz
    reg [191:0] tx_channel_status;

    // Instantiate TX
    aes3_tx #(
        .CLK_FREQ(100_000_000),
        .FS(48000)
    ) u_tx (
        .clk(clk),
        .rst(rst),
        .audio_l(tx_audio_l),
        .audio_r(tx_audio_r),
        .audio_valid(tx_audio_valid),
        .audio_ready(tx_audio_ready),
        .channel_status(tx_channel_status),
        .aes3_out(aes3_wire)
    );

    // Instantiate RX — loopback from TX output
    aes3_rx #(
        .CLK_FREQ(100_000_000)
    ) u_rx (
        .clk(clk),
        .rst(rst),
        .aes3_in(aes3_wire),
        .audio_l(rx_audio_l),
        .audio_r(rx_audio_r),
        .audio_valid(rx_audio_valid),
        .audio_sub(rx_audio_sub),
        .sub_valid(rx_sub_valid),
        .channel_status(rx_channel_status),
        .cs_valid(rx_cs_valid),
        .locked(rx_locked),
        .is_96k(rx_is_96k),
        .error_count(rx_error_count)
    );

    // Test sequence
    integer frame_count;
    integer match_count;
    integer mismatch_count;
    reg [23:0] expected_l;
    reg [23:0] expected_r;

    initial begin
        $dumpfile("aes3_tb.vcd");
        $dumpvars(0, aes3_tb);

        rst = 1;
        tx_audio_l = 0;
        tx_audio_r = 0;
        tx_audio_valid = 0;
        tx_channel_status = 192'h0;
        frame_count = 0;
        match_count = 0;
        mismatch_count = 0;

        // Set professional channel status byte 0:
        // bit 0 = 1 (professional use)
        // bits 6-7 = 01 (48 kHz not indicated by fs bits, but by default)
        tx_channel_status[0] = 1'b1;  // Professional

        #200;
        rst = 0;

        // Wait for TX to start generating and RX PLL to lock
        $display("[%0t] Waiting for RX PLL lock...", $time);
        wait(rx_locked);
        $display("[%0t] RX PLL locked!", $time);

        // Feed test audio samples — ramp pattern
        repeat (300) begin
            @(posedge clk);
            if (tx_audio_ready) begin
                tx_audio_l     <= 24'h100000 + frame_count * 24'h1000;
                tx_audio_r     <= 24'h200000 + frame_count * 24'h1000;
                expected_l      = 24'h100000 + frame_count * 24'h1000;
                expected_r      = 24'h200000 + frame_count * 24'h1000;
                tx_audio_valid <= 1;
                frame_count     = frame_count + 1;
                @(posedge clk);
                tx_audio_valid <= 0;

                // Wait for this frame to be transmitted and received
                // At 48 kHz, one frame = ~2083 clocks at 100 MHz
                repeat (2500) @(posedge clk);
            end
        end

        // Check results
        $display("");
        $display("=== AES3 Loopback Test Results ===");
        $display("RX locked:       %0d", rx_locked);
        $display("RX is_96k:       %0d", rx_is_96k);
        $display("RX error_count:  %0d", rx_error_count);
        $display("Frames sent:     %0d", frame_count);
        $display("Last RX L:       0x%06h", rx_audio_l);
        $display("Last RX R:       0x%06h", rx_audio_r);
        $display("=================================");

        #10000;
        $finish;
    end

    // Monitor received audio
    always @(posedge clk) begin
        if (rx_audio_valid) begin
            $display("[%0t] RX frame: L=0x%06h R=0x%06h", $time, rx_audio_l, rx_audio_r);
        end
    end

    // Timeout
    initial begin
        #100_000_000;  // 100ms timeout
        $display("TIMEOUT!");
        $finish;
    end

endmodule
