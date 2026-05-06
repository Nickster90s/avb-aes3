`default_nettype none
`timescale 1ns / 1ps

module aes3_debug_tb;

    reg clk = 0;
    always #5 clk = ~clk;

    reg rst;
    reg  [23:0] tx_audio_l;
    reg  [23:0] tx_audio_r;
    reg         tx_audio_valid;
    wire        tx_audio_ready;
    wire        aes3_wire;

    wire [23:0] rx_audio_l, rx_audio_r;
    wire        rx_audio_valid;
    wire [23:0] rx_audio_sub;
    wire        rx_sub_valid;
    wire [191:0] rx_channel_status;
    wire        rx_cs_valid;
    wire        rx_locked;
    wire        rx_is_96k;
    wire [3:0]  rx_error_count;

    reg [191:0] tx_channel_status;

    aes3_tx #(.CLK_FREQ(100_000_000), .FS(48000)) u_tx (
        .clk(clk), .rst(rst),
        .audio_l(tx_audio_l), .audio_r(tx_audio_r),
        .audio_valid(tx_audio_valid), .audio_ready(tx_audio_ready),
        .channel_status(tx_channel_status), .aes3_out(aes3_wire)
    );

    aes3_rx #(.CLK_FREQ(100_000_000)) u_rx (
        .clk(clk), .rst(rst), .aes3_in(aes3_wire),
        .audio_l(rx_audio_l), .audio_r(rx_audio_r),
        .audio_valid(rx_audio_valid),
        .audio_sub(rx_audio_sub), .sub_valid(rx_sub_valid),
        .channel_status(rx_channel_status), .cs_valid(rx_cs_valid),
        .locked(rx_locked), .is_96k(rx_is_96k), .error_count(rx_error_count)
    );

    integer preamble_count = 0;
    integer sub_count = 0;
    integer audio_count = 0;

    // Monitor preambles
    always @(posedge clk) begin
        if (u_rx.preamble_any) begin
            preamble_count <= preamble_count + 1;
            if (preamble_count < 10)
                $display("[%0t] PREAMBLE: Z=%b X=%b Y=%b locked=%b pattern=%h",
                    $time, u_rx.preamble_z, u_rx.preamble_x, u_rx.preamble_y,
                    rx_locked, u_rx.level_pattern);
        end
        if (rx_sub_valid) begin
            sub_count <= sub_count + 1;
            if (sub_count < 10)
                $display("[%0t] SUBFRAME: audio=0x%06h ch_b=%b frame=%0d",
                    $time, rx_audio_sub, u_rx.is_channel_b, u_rx.frame_number);
        end
        if (rx_audio_valid) begin
            audio_count <= audio_count + 1;
            $display("[%0t] AUDIO: L=0x%06h R=0x%06h (#%0d)",
                $time, rx_audio_l, rx_audio_r, audio_count);
        end
    end

    // Monitor TX state periodically
    initial begin
        #500000;
        $display("[%0t] TX state: frame=%0d sub_b=%b sym=%0d bmc=%b out=%b",
            $time, u_tx.frame_count, u_tx.subframe_b, u_tx.symbol_count,
            u_tx.bmc_level, aes3_wire);
        $display("[%0t] TX biphase_tick rate check: BIPHASE_INC=%0d",
            $time, u_tx.BIPHASE_INC);
    end

    integer frame_count;
    initial begin
        $dumpfile("aes3_debug.vcd");
        $dumpvars(0, aes3_debug_tb);

        rst = 1;
        tx_audio_l = 0;
        tx_audio_r = 0;
        tx_audio_valid = 0;
        tx_channel_status = 192'h0;
        tx_channel_status[0] = 1'b1;
        frame_count = 0;

        #200;
        rst = 0;

        // Wait for lock
        wait(rx_locked);
        $display("[%0t] RX locked!", $time);

        // Send some frames
        repeat (50) begin
            @(posedge clk);
            if (tx_audio_ready) begin
                tx_audio_l     <= 24'hABCD00 + frame_count;
                tx_audio_r     <= 24'h123400 + frame_count;
                tx_audio_valid <= 1;
                frame_count     = frame_count + 1;
                @(posedge clk);
                tx_audio_valid <= 0;
                repeat (2500) @(posedge clk);
            end
        end

        #50000;
        $display("");
        $display("=== Debug Results ===");
        $display("Preambles detected: %0d", preamble_count);
        $display("Subframes decoded:  %0d", sub_count);
        $display("Audio pairs out:    %0d", audio_count);
        $display("Last RX L: 0x%06h  R: 0x%06h", rx_audio_l, rx_audio_r);
        $display("=====================");

        $finish;
    end

    initial begin
        #100_000_000;
        $display("TIMEOUT!");
        $finish;
    end

endmodule
