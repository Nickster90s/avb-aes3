`default_nettype none

module blinky (
    input  wire clk,
    output wire led,
    output wire uart_tx_r3,
    output wire uart_tx_m3
);

    // ---- Blinky LED ----
    reg [24:0] r_count = 0;
    always @(posedge clk) r_count <= r_count + 4;
    assign led = r_count[24];

    // ---- UART TX: sends 'U' (0x55) continuously at 115200 baud ----
    // 25 MHz / 115200 = 217 clocks per bit
    localparam BAUD_DIV = 217;

    reg [7:0]  baud_cnt = 0;
    reg        baud_tick = 0;
    always @(posedge clk) begin
        if (baud_cnt == BAUD_DIV - 1) begin
            baud_cnt  <= 0;
            baud_tick <= 1;
        end else begin
            baud_cnt  <= baud_cnt + 1;
            baud_tick <= 0;
        end
    end

    // 10-bit frame: start(0), D0-D7, stop(1)
    // 'U' = 0x55 = 01010101
    reg [3:0] bit_idx = 0;
    reg [9:0] frame = 10'b1_01010101_0;  // [9]=stop, [8:1]=D7..D0, [0]=start
    reg       tx_reg = 1;

    always @(posedge clk) begin
        if (baud_tick) begin
            tx_reg <= frame[bit_idx];
            if (bit_idx == 9)
                bit_idx <= 0;
            else
                bit_idx <= bit_idx + 1;
        end
    end

    // Drive UART on both R3 and M3 — one of them is the right pin
    assign uart_tx_r3 = tx_reg;
    assign uart_tx_m3 = tx_reg;

endmodule
