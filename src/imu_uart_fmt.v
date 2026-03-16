`timescale 1ns / 1ps

// Formats IMU sensor data as signed decimal over UART.
// Uses VT100 escape codes to redraw in-place at ~4 Hz.
//
// Output:
//   [ESC][2J[ESC][H
//   === IMU ===
//   Accel X:  -00123
//   Accel Y:   00045
//   Accel Z:   16384
//   Gyro  X:  -00002
//   Gyro  Y:   00015
//   Gyro  Z:  -00001
//   Temp   :   03350

module imu_uart_fmt #(
    parameter CLK_FREQ    = 100_000_000,
    parameter BAUD        = 115_200,
    parameter REFRESH_HZ  = 4
)(
    input  wire        clk,
    input  wire        rst,

    input  wire signed [15:0] accel_x,
    input  wire signed [15:0] accel_y,
    input  wire signed [15:0] accel_z,
    input  wire signed [15:0] gyro_x,
    input  wire signed [15:0] gyro_y,
    input  wire signed [15:0] gyro_z,
    input  wire signed [15:0] temp_raw,
    input  wire        data_valid,

    output wire        uart_tx_pin
);

    reg signed [15:0] lat_ax, lat_ay, lat_az;
    reg signed [15:0] lat_gx, lat_gy, lat_gz;
    reg signed [15:0] lat_t;

    reg        tx_start;
    reg  [7:0] tx_data;
    wire       tx_busy;

    uart_tx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD(BAUD)
    ) u_uart (
        .clk(clk),
        .rst(rst),
        .start(tx_start),
        .data(tx_data),
        .tx(uart_tx_pin),
        .busy(tx_busy)
    );

    // Rate limiter: only update display at REFRESH_HZ
    localparam REFRESH_TICKS = CLK_FREQ / REFRESH_HZ;
    localparam TICK_WIDTH    = $clog2(REFRESH_TICKS + 1);
    reg [TICK_WIDTH-1:0] refresh_cnt;
    reg                  refresh_ready;

    always @(posedge clk) begin
        if (rst) begin
            refresh_cnt   <= {TICK_WIDTH{1'b0}};
            refresh_ready <= 1'b0;
        end else if (refresh_cnt == REFRESH_TICKS[TICK_WIDTH-1:0] - 1) begin
            refresh_cnt   <= {TICK_WIDTH{1'b0}};
            refresh_ready <= 1'b1;
        end else begin
            refresh_cnt   <= refresh_cnt + 1'b1;
            if (refresh_ready && data_valid)
                refresh_ready <= 1'b0;
        end
    end

    // Buffer: VT100 header + 7 lines * ~20 chars + newlines ~ 160 bytes max
    localparam BUF_LEN = 192;
    reg [7:0] tx_buf [0:BUF_LEN-1];
    reg [7:0] tx_buf_len;
    reg [7:0] tx_buf_idx;

    localparam S_IDLE    = 2'd0;
    localparam S_BUILD   = 2'd1;
    localparam S_SEND    = 2'd2;
    localparam S_WAIT_TX = 2'd3;

    reg [1:0] state;
    reg [7:0] wp;

    /* verilator lint_off BLKSEQ */
    task put(input [7:0] b);
        begin
            tx_buf[wp] = b;
            wp = wp + 1;
        end
    endtask

    task put_str2(input [7:0] a, input [7:0] b);
        begin put(a); put(b); end
    endtask

    // Signed 16-bit to decimal: " 00000" or "-00000" (6 chars: sign + 5 digits)
    /* verilator lint_off WIDTHTRUNC */
    task put_s16(input signed [15:0] val);
        reg [15:0] abs_val;
        reg [3:0]  d4, d3, d2, d1, d0;
        begin
            if (val[15]) begin
                put("-");
                abs_val = (~val) + 1;
            end else begin
                put(" ");
                abs_val = val;
            end
            d4 = abs_val / 10000;
            d3 = (abs_val / 1000) % 10;
            d2 = (abs_val / 100) % 10;
            d1 = (abs_val / 10) % 10;
            d0 = abs_val % 10;
            put(8'h30 + {4'd0, d4});
            put(8'h30 + {4'd0, d3});
            put(8'h30 + {4'd0, d2});
            put(8'h30 + {4'd0, d1});
            put(8'h30 + {4'd0, d0});
        end
    endtask
    /* verilator lint_on WIDTHTRUNC */
    /* verilator lint_on BLKSEQ */

    integer i;

    /* verilator lint_off BLKSEQ */
    always @(posedge clk) begin
        if (rst) begin
            state      <= S_IDLE;
            tx_start   <= 1'b0;
            tx_data    <= 8'd0;
            tx_buf_len <= 8'd0;
            tx_buf_idx <= 8'd0;
            wp         = 8'd0;
            lat_ax     <= 16'd0;
            lat_ay     <= 16'd0;
            lat_az     <= 16'd0;
            lat_gx     <= 16'd0;
            lat_gy     <= 16'd0;
            lat_gz     <= 16'd0;
            lat_t      <= 16'd0;
            for (i = 0; i < BUF_LEN; i = i + 1)
                tx_buf[i] = 8'd0;
        end else begin
            tx_start <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (data_valid && refresh_ready) begin
                        lat_ax <= accel_x;
                        lat_ay <= accel_y;
                        lat_az <= accel_z;
                        lat_gx <= gyro_x;
                        lat_gy <= gyro_y;
                        lat_gz <= gyro_z;
                        lat_t  <= temp_raw;
                        state  <= S_BUILD;
                    end
                end

                S_BUILD: begin
                    wp = 8'd0;

                    // VT100: clear screen + cursor home
                    put(8'h1B); put("["); put("2"); put("J");
                    put(8'h1B); put("["); put("H");

                    // Header
                    put("="); put("="); put("="); put(" ");
                    put("I"); put("M"); put("U"); put(" ");
                    put("="); put("="); put("=");
                    put_str2(8'h0D, 8'h0A);

                    // Accel X
                    put("A"); put("c"); put("c"); put("e"); put("l");
                    put(" "); put("X"); put(":"); put(" ");
                    put_s16(lat_ax);
                    put_str2(8'h0D, 8'h0A);

                    // Accel Y
                    put("A"); put("c"); put("c"); put("e"); put("l");
                    put(" "); put("Y"); put(":"); put(" ");
                    put_s16(lat_ay);
                    put_str2(8'h0D, 8'h0A);

                    // Accel Z
                    put("A"); put("c"); put("c"); put("e"); put("l");
                    put(" "); put("Z"); put(":"); put(" ");
                    put_s16(lat_az);
                    put_str2(8'h0D, 8'h0A);

                    // Gyro X
                    put("G"); put("y"); put("r"); put("o");
                    put(" "); put(" "); put("X"); put(":"); put(" ");
                    put_s16(lat_gx);
                    put_str2(8'h0D, 8'h0A);

                    // Gyro Y
                    put("G"); put("y"); put("r"); put("o");
                    put(" "); put(" "); put("Y"); put(":"); put(" ");
                    put_s16(lat_gy);
                    put_str2(8'h0D, 8'h0A);

                    // Gyro Z
                    put("G"); put("y"); put("r"); put("o");
                    put(" "); put(" "); put("Z"); put(":"); put(" ");
                    put_s16(lat_gz);
                    put_str2(8'h0D, 8'h0A);

                    // Temp
                    put("T"); put("e"); put("m"); put("p");
                    put(" "); put(" "); put(" "); put(":"); put(" ");
                    put_s16(lat_t);
                    put_str2(8'h0D, 8'h0A);

                    tx_buf_len <= wp;
                    tx_buf_idx <= 8'd0;
                    state      <= S_SEND;
                end

                S_SEND: begin
                    if (tx_buf_idx == tx_buf_len) begin
                        state <= S_IDLE;
                    end else if (!tx_busy) begin
                        tx_data    <= tx_buf[tx_buf_idx];
                        tx_start   <= 1'b1;
                        tx_buf_idx <= tx_buf_idx + 1'b1;
                        state      <= S_WAIT_TX;
                    end
                end

                S_WAIT_TX: begin
                    if (!tx_busy)
                        state <= S_SEND;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    /* verilator lint_on BLKSEQ */

endmodule
