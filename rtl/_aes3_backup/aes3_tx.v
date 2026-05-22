`default_nettype none
// AES3/EBU Transmitter
// Biphase-mark encoder with preamble generation
//
// Accepts 24-bit audio samples and generates AES3 serial output.
// Requires a bit clock at 2x the subframe bit rate:
//   48 kHz: 128 * 48000 = 6.144 MHz biphase clock
//   96 kHz: 128 * 96000 = 12.288 MHz biphase clock
//
// When USE_EXT_TICK=1, uses ext_biphase_tick from an external MMCM-derived
// audio clock for low-jitter output (<100 ps vs ±20 ns from accumulator).
// When USE_EXT_TICK=0, falls back to internal phase accumulator.

module aes3_tx #(
    parameter CLK_FREQ     = 100_000_000,  // System clock frequency
    parameter FS           = 48000,        // Audio sample rate (48000 or 96000)
    parameter USE_EXT_TICK = 0             // 1 = use ext_biphase_tick, 0 = internal accumulator
)(
    input  wire        clk,
    input  wire        rst,

    // Audio input
    input  wire [23:0] audio_l,        // Left channel (24-bit signed)
    input  wire [23:0] audio_r,        // Right channel (24-bit signed)
    input  wire        audio_valid,    // Pulse to load new L/R pair
    output reg         audio_ready,    // High when ready for new samples

    // External biphase tick (from MMCM audio clock, sys_clk domain)
    input  wire        ext_biphase_tick,

    // Configuration
    input  wire [191:0] channel_status, // Channel status block (professional format)

    // AES3 output
    output reg         aes3_out        // Biphase-mark encoded serial output
);

    // =========================================================================
    // Biphase clock generation
    //
    // Symbol rate = 128 * fs (6.144 MHz at 48 kHz, 12.288 MHz at 96 kHz).
    //
    // Two modes:
    //   USE_EXT_TICK=1: Use MMCM-derived tick from audio clock domain.
    //                   Jitter < 100 ps (PLL intrinsic).
    //   USE_EXT_TICK=0: Internal phase accumulator from sys_clk.
    //                   Jitter = ±1/CLK_FREQ (±20 ns at 50 MHz).
    // =========================================================================

    wire biphase_tick;

    generate
        if (USE_EXT_TICK) begin : gen_ext_tick
            // Use externally provided tick (already in sys_clk domain)
            assign biphase_tick = ext_biphase_tick;
        end else begin : gen_int_tick
            // Internal phase accumulator
            localparam [31:0] BIPHASE_INC = (48'd128 * FS * 48'd16_777_216) / CLK_FREQ;
            reg [23:0] biphase_acc;
            reg        biphase_tick_r;
            always @(posedge clk) begin
                if (rst) begin
                    biphase_acc    <= 0;
                    biphase_tick_r <= 0;
                end else begin
                    {biphase_tick_r, biphase_acc} <= {1'b0, biphase_acc} + {1'b0, BIPHASE_INC[23:0]};
                end
            end
            assign biphase_tick = biphase_tick_r;
        end
    endgenerate

    // =========================================================================
    // Frame/subframe state machine
    //
    // AES3 block = 192 frames
    // Each frame = 2 subframes (A=left, B=right)
    // Each subframe = 4 preamble symbols + 56 data symbols (28 bits * 2 symbols each)
    //               = 64 symbols total
    //
    // Subframe bit layout (32 bits total):
    //   [3:0]   = Preamble (4 bits, special encoding)
    //   [27:4]  = Audio sample (24 bits, LSB first)
    //   [28]    = Validity (V) — 0 = valid
    //   [29]    = User data (U) — 0
    //   [30]    = Channel status (C) — from channel_status block
    //   [31]    = Parity (P) — even parity over bits 4-31
    // =========================================================================

    // State
    reg [7:0]  frame_count;     // 0-191 within block
    reg        subframe_b;      // 0 = subframe A (left), 1 = subframe B (right)
    reg [5:0]  symbol_count;    // 0-63 within subframe
    reg [31:0] subframe_data;   // Current subframe being transmitted
    reg        bmc_level;       // Current biphase-mark output level

    // Subframe construction
    reg [23:0] tx_audio;        // Audio data for current subframe
    reg [23:0] pending_l;       // Buffered left channel
    reg [23:0] pending_r;       // Buffered right channel
    reg        pending_valid;   // New samples are buffered

    // Preamble level patterns (IEC 60958 / AES3)
    //
    // Each preamble is 8 cells (4 bit periods). The patterns below give
    // the output level at each cell when the PRECEDING cell was LOW.
    // When the preceding cell was HIGH, the complement is used.
    //
    // All patterns start with 3 cells opposite to the preceding level
    // (violating biphase-mark's mandatory transition at every bit boundary),
    // which is how the receiver detects them.
    //
    // Z (block start, subframe A): 3+3+1+1 → 11101000
    // X (subframe A, not block start): 3+3+2  → 11100010  (duration: 3+2+1+2)
    // Y (subframe B):                  3+2+1+2 → 11100100 (duration: 3+1+1+3)
    //
    // Property: all patterns end with bit[0]=0, so the ending level
    // equals the preceding level (bmc_level unchanged across preamble).
    localparam [7:0] PREAMBLE_Z = 8'b11101000;
    localparam [7:0] PREAMBLE_X = 8'b11100010;
    localparam [7:0] PREAMBLE_Y = 8'b11100100;

    reg [7:0] preamble_pattern;
    reg       in_preamble;

    // Determine which preamble to use
    always @(*) begin
        if (subframe_b)
            preamble_pattern = PREAMBLE_Y;
        else if (frame_count == 0)
            preamble_pattern = PREAMBLE_Z;
        else
            preamble_pattern = PREAMBLE_X;
    end

    // Audio sample buffering
    always @(posedge clk) begin
        if (rst) begin
            pending_valid <= 0;
            pending_l     <= 0;
            pending_r     <= 0;
        end else begin
            if (audio_valid) begin
                pending_l     <= audio_l;
                pending_r     <= audio_r;
                pending_valid <= 1;
            end
            // Clear pending when we load into the subframe
            if (biphase_tick && symbol_count == 63 && !subframe_b) begin
                // About to start subframe A — load new samples
                pending_valid <= 0;
            end
        end
    end

    // Ready signal — can accept new samples when not double-buffered
    always @(posedge clk) begin
        audio_ready <= ~pending_valid;
    end

    // =========================================================================
    // Subframe construction
    // =========================================================================

    wire [23:0] current_audio = subframe_b ? pending_r : pending_l;
    wire        current_cs    = channel_status[frame_count];
    wire        parity_bit    = ^{current_audio, 1'b0, 1'b0, current_cs}; // Even parity over bits 4-31

    // Current data bit for biphase-mark encoding
    // bit_index = (symbol_count - 8) / 2, valid when symbol_count >= 8
    wire [4:0] bit_index = (symbol_count - 6'd8) >> 1;
    reg data_bit_current;
    always @(*) begin
        case (bit_index)
            5'd24:   data_bit_current = 1'b0;         // V (validity) = 0 (valid audio)
            5'd25:   data_bit_current = 1'b0;         // U (user data) = 0
            5'd26:   data_bit_current = current_cs;   // C (channel status)
            5'd27:   data_bit_current = parity_bit;   // P (parity)
            default: data_bit_current = current_audio[bit_index]; // Audio bits 0-23, LSB first
        endcase
    end

    // =========================================================================
    // Biphase-mark encoder state machine
    //
    // bmc_level tracks the output level at the END of the last data cell.
    // During preamble, bmc_level is NOT modified (ending level = starting level
    // for all three preamble patterns since bit[0]=0).
    // During data, bmc_level tracks aes3_out for use by the next preamble.
    // =========================================================================

    always @(posedge clk) begin
        if (rst) begin
            frame_count   <= 0;
            subframe_b    <= 0;
            symbol_count  <= 0;
            bmc_level     <= 0;
            aes3_out      <= 0;
            in_preamble   <= 1;
        end else if (biphase_tick) begin

            if (symbol_count < 8) begin
                // ---- Preamble phase (8 cells) ----
                // Output the level pattern directly. XOR with bmc_level
                // selects the correct polarity (pattern is defined for
                // preceding-level = LOW; XOR inverts when preceding = HIGH).
                aes3_out <= bmc_level ^ preamble_pattern[7 - symbol_count];
                // bmc_level unchanged (all patterns end with bit[0]=0)
                symbol_count <= symbol_count + 1;

            end else begin
                // ---- Data phase (56 cells = 28 bits × 2 cells each) ----
                // Biphase-mark encoding:
                //   First cell of each bit: always transition from previous level
                //   Second cell: transition if data='1', else hold

                if (symbol_count[0] == 1'b0) begin
                    // First cell of this bit — always transition
                    aes3_out  <= ~aes3_out;
                    bmc_level <= ~aes3_out;
                end else begin
                    // Second cell of this bit
                    if (data_bit_current) begin
                        aes3_out  <= ~aes3_out;
                        bmc_level <= ~aes3_out;
                    end
                    // else: hold level, bmc_level already correct
                end

                symbol_count <= symbol_count + 1;

                // End of subframe
                if (symbol_count == 63) begin
                    symbol_count <= 0;

                    if (subframe_b) begin
                        // End of frame — advance to next
                        subframe_b <= 0;
                        if (frame_count == 191)
                            frame_count <= 0;
                        else
                            frame_count <= frame_count + 1;
                    end else begin
                        // Switch to subframe B
                        subframe_b <= 1;
                    end
                end
            end
        end
    end

endmodule
