`default_nettype none
// I2S Transmitter for PCM5102A DAC
//
// Generates BCK, LRCK, and DIN from 24-bit stereo audio samples.
// Clocked from audio clock domain (12.288 MHz = 256 × 48 kHz).
//
// I2S format: MSB-first, data transitions on BCK falling edge,
// sampled by DAC on BCK rising edge. LRCK low = left, high = right.
// Data is one BCK cycle delayed from LRCK transition (I2S standard).

module i2s_tx (
    input  wire        clk,        // 12.288 MHz audio clock
    input  wire        rst,

    // Audio input (directly from sys_clk domain — active for 1 audio_clk cycle)
    input  wire [23:0] audio_l,
    input  wire [23:0] audio_r,
    input  wire        audio_valid, // Pulse: new sample pair ready
    output reg         audio_ready, // High when ready to accept new sample

    // I2S output
    output reg         bck,        // Bit clock (3.072 MHz)
    output reg         lrck,       // Word select (48 kHz)
    output reg         dout,       // Serial data

    // Externally-visible frame_start pulse — high for one audio_clk cycle
    // at the start of each 48 kHz frame. Used by an upstream FIFO to pulse
    // its read enable in lock-step with the I2S consumer, so samples are
    // delivered at the exact rate i2s_tx consumes them (no drop / repeat).
    output wire        frame_start_out
);

    // 12.288 MHz / 4 = 3.072 MHz BCK (2 clk cycles per BCK half-period)
    // 64 BCK per frame = 32 left + 32 right
    // Total: 256 audio clocks per frame (= 12.288 MHz / 48 kHz)

    reg [7:0] clk_cnt;    // 0-255: master counter
    reg [23:0] sr_l, sr_r; // Latched audio samples

    // BCK toggles every 2 audio clocks → bit 1 of clk_cnt
    // LRCK toggles every 128 audio clocks → bit 7 of clk_cnt
    // Each BCK period = 4 audio clocks
    // Bit index within 32-bit half-frame: clk_cnt[6:2]

    wire bck_falling = (clk_cnt[1:0] == 2'b01);  // Transition point for data
    wire frame_start = (clk_cnt == 8'd0);
    assign frame_start_out = frame_start;

    always @(posedge clk) begin
        if (rst) begin
            clk_cnt     <= 8'd0;
            bck         <= 1'b0;
            lrck        <= 1'b0;
            dout        <= 1'b0;
            sr_l        <= 24'd0;
            sr_r        <= 24'd0;
            audio_ready <= 1'b1;
        end else begin
            clk_cnt <= clk_cnt + 8'd1;

            // BCK: toggles on bit 1
            bck  <= clk_cnt[1];

            // LRCK: low = left (first half), high = right (second half)
            lrck <= clk_cnt[7];

            // Latch new samples at frame start
            if (frame_start) begin
                if (audio_valid) begin
                    sr_l <= audio_l;
                    sr_r <= audio_r;
                    audio_ready <= 1'b1;
                end else begin
                    // No new sample — repeat last (or zero)
                    audio_ready <= 1'b1;
                end
            end

            // Output data on BCK falling edge
            // I2S: 1 BCK delay after LRCK, then MSB first, 24 bits + 8 zeros
            if (bck_falling) begin
                case (clk_cnt[7])
                    1'b0: begin
                        // Left channel: bits [6:2] = 0..31
                        // Bit 0 = I2S delay (dummy), bits 1-24 = data MSB first, 25-31 = zero pad
                        if (clk_cnt[6:2] == 5'd0)
                            dout <= 1'b0;           // I2S one-bit delay
                        else if (clk_cnt[6:2] <= 5'd24)
                            dout <= sr_l[24 - clk_cnt[6:2]]; // MSB first: bit 23 down to 0
                        else
                            dout <= 1'b0;           // Zero pad
                    end
                    1'b1: begin
                        // Right channel
                        if (clk_cnt[6:2] == 5'd0)
                            dout <= 1'b0;
                        else if (clk_cnt[6:2] <= 5'd24)
                            dout <= sr_r[24 - clk_cnt[6:2]];
                        else
                            dout <= 1'b0;
                    end
                endcase
            end
        end
    end

endmodule
