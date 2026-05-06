`default_nettype none
// AES3/EBU Receiver — edge-interval biphase-mark decoder
//
// Recovers audio from an AES3 serial input by measuring intervals between
// consecutive transitions.  Intervals are classified as:
//   SHORT  ≈ 1 cell   (data transition)
//   LONG   ≈ 2 cells  (boundary-to-boundary, no mid-bit transition)
//   VLONG  ≈ 3 cells  (preamble violation — unique sync marker)
//
// No PLL needed.  Cell width is tracked as the minimum observed interval,
// giving automatic sample-rate detection (48 kHz / 96 kHz).
//
// At 50 MHz sys_clk:  ~8× oversampling for 48 kHz, ~4× for 96 kHz.
// Minimum supported: ~4× oversampling (cell width ≥ 4 ticks).
//
// Biphase-mark decoding:
//   Each data bit is 2 cells.  Mandatory transition at bit boundary.
//     data '1' → extra transition at mid-bit  → two SHORT intervals
//     data '0' → no mid-bit transition        → one LONG interval
//
// Preamble edge-interval signatures (after initial VLONG):
//   Z (block start, ch A): VLONG(3), SHORT(1), SHORT(1), VLONG(3)
//   X (ch A, not start):   VLONG(3), VLONG(3), SHORT(1), SHORT(1)
//   Y (ch B):              VLONG(3), LONG(2),  SHORT(1), LONG(2)
//   Classify from first interval after VLONG:
//     SHORT → Z,  VLONG → X,  LONG → Y

module aes3_rx #(
    parameter CLK_FREQ = 50_000_000
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        aes3_in,

    // Audio output
    output reg  [23:0] audio_l,       // Left channel (24-bit signed)
    output reg  [23:0] audio_r,       // Right channel (24-bit signed)
    output reg         audio_valid,   // Pulse when new L/R pair ready
    output reg  [23:0] audio_sub,     // Current subframe audio (before L/R assign)
    output reg         sub_valid,     // Pulse per subframe

    // Channel status
    output reg [191:0] channel_status,// Full 192-bit channel status block
    output reg         cs_valid,      // Pulse when channel_status is complete

    // Status
    output reg         locked,        // Receiver locked to valid AES3 signal
    output reg         is_96k,        // 1 = 96 kHz detected, 0 = 48 kHz
    output reg  [3:0]  error_count    // Saturating error counter
);

    // =========================================================================
    // Input synchronizer (3-stage metastability guard)
    // =========================================================================
    reg [2:0] in_sync;
    always @(posedge clk)
        in_sync <= rst ? 3'b0 : {in_sync[1:0], aes3_in};
    wire aes3 = in_sync[2];

    // =========================================================================
    // Edge detector
    // =========================================================================
    reg aes3_d;
    wire edge_det = aes3 ^ aes3_d;
    always @(posedge clk)
        aes3_d <= rst ? 1'b0 : aes3;

    // =========================================================================
    // Edge interval measurement
    // =========================================================================
    // Count sys_clk ticks between consecutive edges.
    // At 50 MHz / 48 kHz: 1 cell ≈ 8.1 ticks, 3-cell preamble ≈ 24.4 ticks.
    // 8-bit counter (max 255) covers all valid cases plus timeout.

    reg [7:0] edge_timer;
    reg [7:0] interval;
    reg       iv;              // interval_valid pulse

    always @(posedge clk) begin
        iv <= 1'b0;
        if (rst) begin
            edge_timer <= 8'd0;
        end else if (edge_det) begin
            if (edge_timer >= 8'd3) begin       // Reject sub-60 ns glitches
                interval <= edge_timer;
                iv       <= 1'b1;
            end
            edge_timer <= 8'd1;
        end else if (!edge_timer[7]) begin      // Saturate at 128
            edge_timer <= edge_timer + 8'd1;
        end
    end

    // =========================================================================
    // Cell width estimator — leaky minimum tracker
    // =========================================================================
    // The shortest valid edge interval = 1 cell.  We track this as a
    // running minimum.  A periodic decay (every 4096 clocks ≈ 82 µs)
    // bumps cell_w upward by 1, allowing re-acquisition on rate changes.

    localparam [7:0] CELL_NOM = CLK_FREQ / (128 * 48000);  // ~8 at 50 MHz

    reg [7:0]  cell_w;
    reg [11:0] decay_cnt;

    always @(posedge clk) begin
        if (rst) begin
            cell_w    <= CELL_NOM;
            decay_cnt <= 12'd0;
        end else begin
            // Snap to shorter interval (only accept ≥ 3 ticks)
            if (iv && interval >= 8'd3 && interval < cell_w)
                cell_w <= interval;

            // Periodic decay: allow cell_w to grow back if signal disappears or rate changes
            decay_cnt <= decay_cnt + 12'd1;
            if (&decay_cnt && cell_w < 8'd40)
                cell_w <= cell_w + 8'd1;
        end
    end

    // =========================================================================
    // Interval classification
    // =========================================================================
    //   SHORT:  interval < 1.5 × cell_w
    //   LONG:   1.5 × cell_w ≤ interval < 2.5 × cell_w
    //   VLONG:  2.5 × cell_w ≤ interval < 4 × cell_w
    //   ERROR:  anything else

    wire [8:0] th_sl = {1'b0, cell_w} + {2'b0, cell_w[7:1]};  // 1.5 × cell_w
    wire [8:0] th_lv = {cell_w, 1'b0} + {2'b0, cell_w[7:1]};  // 2.5 × cell_w

    localparam [1:0] T_S = 2'd0,   // SHORT  (1 cell)
                     T_L = 2'd1,   // LONG   (2 cells)
                     T_V = 2'd2,   // VLONG  (3 cells — preamble)
                     T_E = 2'd3;   // ERROR

    reg [1:0] itype;
    always @(*) begin
        if ({1'b0, interval} < th_sl)
            itype = T_S;
        else if ({1'b0, interval} < th_lv)
            itype = T_L;
        else if (interval < {cell_w[5:0], 2'b00})  // < 4 × cell_w
            itype = T_V;
        else
            itype = T_E;
    end

    // =========================================================================
    // Receiver state machine
    // =========================================================================
    //
    // S_HUNT → detect VLONG → S_SYNC → collect 3 preamble edges → S_DATA
    // S_DATA → decode 28 bits → process subframe → S_HUNT
    //
    // On subframe completion, if a VLONG arrives simultaneously, go
    // directly to S_SYNC (seamless preamble chaining in steady state).

    localparam [1:0] S_HUNT = 2'd0,
                     S_SYNC = 2'd1,
                     S_DATA = 2'd2;

    reg [1:0]  state;
    reg [1:0]  sync_cnt;       // Edge counter in SYNC (0, 1, 2 → then DATA)
    reg        phase;          // 0 = at bit boundary, 1 = at midpoint
    reg [4:0]  bit_cnt;        // Data bits received in current subframe (0-28)
    reg [27:0] sr;             // Shift register (LSB first, 28 bits)

    // Preamble type
    reg        pre_z, pre_x, pre_y;

    // Frame tracking
    reg [7:0]  frame_num;      // Frame within block (0-191)
    reg        ch_b;           // Current subframe is channel B

    // Channel status accumulator
    reg [191:0] cs_acc;
    reg [7:0]   cs_cnt;

    // Lock tracking
    reg [7:0]   lock_cnt;

    // Even parity check over all 28 bits
    wire parity_ok = ~(^sr);

    // No signal: edge_timer saturated at 128+
    wire no_signal = edge_timer[7];

    always @(posedge clk) begin
        // Default: clear single-cycle pulses
        audio_valid <= 1'b0;
        sub_valid   <= 1'b0;
        cs_valid    <= 1'b0;

        if (rst) begin
            state       <= S_HUNT;
            locked      <= 1'b0;
            lock_cnt    <= 8'd0;
            error_count <= 4'd0;
            frame_num   <= 8'd0;
            is_96k      <= 1'b0;
            audio_l     <= 24'd0;
            audio_r     <= 24'd0;
            audio_sub   <= 24'd0;
            cs_cnt      <= 8'd0;
            bit_cnt     <= 5'd0;
            ch_b        <= 1'b0;
            phase       <= 1'b0;
            sr          <= 28'd0;
        end

        // =============================================================
        // Priority 1: signal lost → reset to HUNT
        // =============================================================
        else if (no_signal) begin
            state    <= S_HUNT;
            locked   <= 1'b0;
            lock_cnt <= 8'd0;
        end

        // =============================================================
        // Priority 2: subframe complete (1 cycle after bit_cnt reaches 28)
        // sr is stable from the previous cycle's non-blocking update.
        // =============================================================
        else if (state == S_DATA && bit_cnt == 5'd28) begin
            // --- Extract subframe fields ---
            // sr[23:0]  = audio (24 bits, LSB first)
            // sr[24]    = validity (V)
            // sr[25]    = user data (U)
            // sr[26]    = channel status (C)
            // sr[27]    = parity (P)
            audio_sub <= sr[23:0];
            sub_valid <= 1'b1;

            if (ch_b) begin
                audio_r     <= sr[23:0];
                audio_valid <= 1'b1;
                if (frame_num < 8'd191)
                    frame_num <= frame_num + 8'd1;
            end else begin
                audio_l <= sr[23:0];
            end

            // --- Channel status (C bit from subframe A only) ---
            if (!ch_b) begin
                cs_acc <= {sr[26], cs_acc[191:1]};
                if (cs_cnt == 8'd191) begin
                    channel_status <= {sr[26], cs_acc[191:1]};
                    cs_valid       <= 1'b1;
                    cs_cnt         <= 8'd0;
                end else begin
                    cs_cnt <= cs_cnt + 8'd1;
                end
            end

            // --- Parity / lock tracking ---
            if (parity_ok) begin
                if (lock_cnt < 8'd255) lock_cnt <= lock_cnt + 8'd1;
                if (error_count > 4'd0) error_count <= error_count - 4'd1;
            end else begin
                if (error_count < 4'd15) error_count <= error_count + 4'd1;
                if (lock_cnt > 8'd8)     lock_cnt <= lock_cnt - 8'd8;
                else                     lock_cnt <= 8'd0;
            end
            locked <= (lock_cnt > 8'd32);

            // --- Sample rate from cell width ---
            is_96k <= (cell_w < (CELL_NOM - 8'd2));

            // --- Transition to next state ---
            // If a VLONG edge arrived this same cycle, chain directly to SYNC.
            if (iv && itype == T_V) begin
                state    <= S_SYNC;
                sync_cnt <= 2'd0;
            end else begin
                state <= S_HUNT;
            end
            bit_cnt <= 5'd0;
        end

        // =============================================================
        // Priority 3: edge processing
        // =============================================================
        else if (iv) begin
            case (state)

            // --- HUNT: look for VLONG (preamble start) ---
            S_HUNT: begin
                if (itype == T_V) begin
                    state    <= S_SYNC;
                    sync_cnt <= 2'd0;
                end
            end

            // --- SYNC: classify preamble + align to data start ---
            // After the initial VLONG, collect 3 more edges.
            // First edge interval identifies the preamble type.
            // Third edge lands on data bit 0 boundary.
            S_SYNC: begin
                if (sync_cnt == 2'd0) begin
                    // Classify from first post-VLONG interval
                    pre_z <= (itype == T_S);    // SHORT → Z
                    pre_y <= (itype == T_L);    // LONG  → Y
                    pre_x <= (itype == T_V);    // VLONG → X
                    if (itype == T_E) begin
                        state <= S_HUNT;        // Bad preamble
                    end else begin
                        sync_cnt <= 2'd1;
                    end
                end else if (sync_cnt == 2'd1) begin
                    sync_cnt <= 2'd2;
                end else begin
                    // Third edge — now at data bit 0 boundary
                    state   <= S_DATA;
                    phase   <= 1'b0;
                    bit_cnt <= 5'd0;

                    // Frame tracking
                    if (pre_z) begin
                        ch_b      <= 1'b0;
                        frame_num <= 8'd0;
                        cs_cnt    <= 8'd0;
                    end else if (pre_x) begin
                        ch_b <= 1'b0;
                    end else begin
                        ch_b <= 1'b1;
                    end
                end
            end

            // --- DATA: decode 28 biphase-mark bits ---
            S_DATA: begin
                if (itype == T_V) begin
                    // Unexpected VLONG during data — lost sync, resync on new preamble
                    state    <= S_SYNC;
                    sync_cnt <= 2'd0;
                    if (lock_cnt > 8'd8) lock_cnt <= lock_cnt - 8'd8;
                    else                 lock_cnt <= 8'd0;

                end else if (itype == T_E) begin
                    // Bad interval — lose sync
                    state <= S_HUNT;
                    if (lock_cnt > 8'd8) lock_cnt <= lock_cnt - 8'd8;
                    else                 lock_cnt <= 8'd0;

                end else if (!phase && itype == T_S) begin
                    // Boundary → midpoint: first half of '1' bit
                    phase <= 1'b1;

                end else if (phase && itype == T_S) begin
                    // Midpoint → boundary: completed '1' bit
                    phase   <= 1'b0;
                    sr      <= {1'b1, sr[27:1]};
                    bit_cnt <= bit_cnt + 5'd1;

                end else if (!phase && itype == T_L) begin
                    // Boundary → boundary: completed '0' bit
                    sr      <= {1'b0, sr[27:1]};
                    bit_cnt <= bit_cnt + 5'd1;

                end else begin
                    // Midpoint + LONG = invalid — lose sync
                    state <= S_HUNT;
                    if (lock_cnt > 8'd8) lock_cnt <= lock_cnt - 8'd8;
                    else                 lock_cnt <= 8'd0;
                end
            end

            default: state <= S_HUNT;

            endcase
        end // iv
    end

endmodule
