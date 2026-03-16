`timescale 1ns / 1ps

module mpu_driver #(
    parameter CLK_FREQ     = 100_000_000,
    parameter ACCEL_FS_SEL = 2'b00,     // 00=+/-2g, 01=+/-4g, 10=+/-8g, 11=+/-16g
    parameter GYRO_FS_SEL  = 2'b11      // 00=250, 01=500, 10=1000, 11=2000 deg/s
)(
    input  wire        clk,
    input  wire        rst,

    output reg         spi_start,
    output reg  [7:0]  spi_tx_data,
    input  wire [7:0]  spi_rx_data,
    input  wire        spi_done,
    input  wire        spi_busy,
    output reg         cs_n,

    input  wire        int_n,

    output reg  signed [15:0] accel_x,
    output reg  signed [15:0] accel_y,
    output reg  signed [15:0] accel_z,
    output reg  signed [15:0] gyro_x,
    output reg  signed [15:0] gyro_y,
    output reg  signed [15:0] gyro_z,
    output reg  signed [15:0] temp_raw,
    output reg         data_valid,
    output reg         init_done,
    output reg         init_fail
);

    // MPU register addresses
    localparam REG_SMPLRT_DIV  = 7'h19;
    localparam REG_CONFIG      = 7'h1A;
    localparam REG_GYRO_CFG    = 7'h1B;
    localparam REG_ACCEL_CFG   = 7'h1C;
    localparam REG_INT_ENABLE  = 7'h38;
    localparam REG_ACCEL_XOUT  = 7'h3B;
    localparam REG_PWR_MGMT_1  = 7'h6B;
    localparam REG_WHO_AM_I    = 7'h75;

    // Timing constants
    localparam DELAY_100MS = CLK_FREQ / 10;
    localparam DELAY_50MS  = CLK_FREQ / 20;
    localparam DELAY_WIDTH = $clog2(DELAY_100MS + 1);

    // Main state machine
    localparam ST_POR_WAIT      = 4'd0;
    localparam ST_RESET          = 4'd1;
    localparam ST_RESET_WAIT     = 4'd2;
    localparam ST_WAKE           = 4'd3;
    localparam ST_WAKE_WAIT      = 4'd4;
    localparam ST_WHO_AM_I       = 4'd5;
    localparam ST_WHO_CHECK      = 4'd6;
    localparam ST_CFG_SMPLRT     = 4'd7;
    localparam ST_CFG_DLPF       = 4'd8;
    localparam ST_CFG_GYRO       = 4'd9;
    localparam ST_CFG_ACCEL      = 4'd10;
    localparam ST_CFG_INT        = 4'd11;
    localparam ST_IDLE           = 4'd12;
    localparam ST_BURST_READ     = 4'd13;
    localparam ST_ASSEMBLE       = 4'd14;

    reg [3:0]              state;
    reg [DELAY_WIDTH-1:0]  delay_cnt;

    // SPI sub-state for multi-byte transactions
    localparam SUB_CS_LOW  = 2'd0;
    localparam SUB_SEND    = 2'd1;
    localparam SUB_WAIT    = 2'd2;
    localparam SUB_CS_HIGH = 2'd3;

    reg [1:0] sub_state;
    reg [3:0] byte_idx;
    reg [7:0] burst_buf [0:13];

    // Inter-transaction gap (~1 us = 100 clocks at 100 MHz)
    localparam CS_GAP = CLK_FREQ / 1_000_000;
    localparam GAP_WIDTH = $clog2(CS_GAP + 1);
    reg [GAP_WIDTH-1:0] gap_cnt;
    reg                 gap_active;

    // INT falling-edge detection (active-low, directly from MPU)
    reg int_n_meta, int_n_sync, int_n_prev;
    wire int_falling = int_n_prev && !int_n_sync;

    always @(posedge clk) begin
        int_n_meta <= int_n;
        int_n_sync <= int_n_meta;
        int_n_prev <= int_n_sync;
    end

    // WHO_AM_I check: accept both MPU-9250 (0x71) and MPU-6500 (0x70)
    wire who_am_i_ok = (spi_rx_data == 8'h71) || (spi_rx_data == 8'h70);

    // SPI write helper: sends [0 | addr] then [data] (2 bytes, CS wrapping)
    // SPI read helper: sends [1 | addr] then [0xFF] to clock in response
    // Both are sequenced inline using sub_state and byte_idx.

    integer i;

    always @(posedge clk) begin
        if (rst) begin
            state       <= ST_POR_WAIT;
            sub_state   <= SUB_CS_LOW;
            delay_cnt   <= {DELAY_WIDTH{1'b0}};
            cs_n        <= 1'b1;
            spi_start   <= 1'b0;
            spi_tx_data <= 8'd0;
            data_valid  <= 1'b0;
            init_done   <= 1'b0;
            init_fail   <= 1'b0;
            byte_idx    <= 4'd0;
            gap_cnt     <= {GAP_WIDTH{1'b0}};
            gap_active  <= 1'b0;
            accel_x     <= 16'd0;
            accel_y     <= 16'd0;
            accel_z     <= 16'd0;
            gyro_x      <= 16'd0;
            gyro_y      <= 16'd0;
            gyro_z      <= 16'd0;
            temp_raw    <= 16'd0;
            for (i = 0; i < 14; i = i + 1)
                burst_buf[i] <= 8'd0;
        end else begin
            spi_start  <= 1'b0;
            data_valid <= 1'b0;

            // Inter-transaction CS gap
            if (gap_active) begin
                if (gap_cnt == CS_GAP[GAP_WIDTH-1:0] - 1) begin
                    gap_active <= 1'b0;
                    gap_cnt    <= {GAP_WIDTH{1'b0}};
                end else begin
                    gap_cnt <= gap_cnt + 1'b1;
                end
            end else begin

            case (state)
                // ---- Init sequence ----
                ST_POR_WAIT: begin
                    if (delay_cnt == DELAY_100MS[DELAY_WIDTH-1:0] - 1) begin
                        delay_cnt <= {DELAY_WIDTH{1'b0}};
                        state     <= ST_RESET;
                        sub_state <= SUB_CS_LOW;
                    end else
                        delay_cnt <= delay_cnt + 1'b1;
                end

                ST_RESET: begin
                    case (sub_state)
                        SUB_CS_LOW: begin
                            cs_n      <= 1'b0;
                            sub_state <= SUB_SEND;
                            byte_idx  <= 4'd0;
                        end
                        SUB_SEND: begin
                            if (!spi_busy) begin
                                spi_tx_data <= (byte_idx == 0) ? {1'b0, REG_PWR_MGMT_1} : 8'h80;
                                spi_start   <= 1'b1;
                                sub_state   <= SUB_WAIT;
                            end
                        end
                        SUB_WAIT: begin
                            if (spi_done) begin
                                if (byte_idx == 4'd1) begin
                                    sub_state <= SUB_CS_HIGH;
                                end else begin
                                    byte_idx  <= byte_idx + 1'b1;
                                    sub_state <= SUB_SEND;
                                end
                            end
                        end
                        SUB_CS_HIGH: begin
                            cs_n       <= 1'b1;
                            gap_active <= 1'b1;
                            state      <= ST_RESET_WAIT;
                            delay_cnt  <= {DELAY_WIDTH{1'b0}};
                        end
                    endcase
                end

                ST_RESET_WAIT: begin
                    if (delay_cnt == DELAY_100MS[DELAY_WIDTH-1:0] - 1) begin
                        delay_cnt <= {DELAY_WIDTH{1'b0}};
                        state     <= ST_WAKE;
                        sub_state <= SUB_CS_LOW;
                    end else
                        delay_cnt <= delay_cnt + 1'b1;
                end

                ST_WAKE: begin
                    case (sub_state)
                        SUB_CS_LOW: begin
                            cs_n      <= 1'b0;
                            sub_state <= SUB_SEND;
                            byte_idx  <= 4'd0;
                        end
                        SUB_SEND: begin
                            if (!spi_busy) begin
                                spi_tx_data <= (byte_idx == 0) ? {1'b0, REG_PWR_MGMT_1} : 8'h01;
                                spi_start   <= 1'b1;
                                sub_state   <= SUB_WAIT;
                            end
                        end
                        SUB_WAIT: begin
                            if (spi_done) begin
                                if (byte_idx == 4'd1) begin
                                    sub_state <= SUB_CS_HIGH;
                                end else begin
                                    byte_idx  <= byte_idx + 1'b1;
                                    sub_state <= SUB_SEND;
                                end
                            end
                        end
                        SUB_CS_HIGH: begin
                            cs_n       <= 1'b1;
                            gap_active <= 1'b1;
                            state      <= ST_WAKE_WAIT;
                            delay_cnt  <= {DELAY_WIDTH{1'b0}};
                        end
                    endcase
                end

                ST_WAKE_WAIT: begin
                    if (delay_cnt == DELAY_50MS[DELAY_WIDTH-1:0] - 1) begin
                        delay_cnt <= {DELAY_WIDTH{1'b0}};
                        state     <= ST_WHO_AM_I;
                        sub_state <= SUB_CS_LOW;
                    end else
                        delay_cnt <= delay_cnt + 1'b1;
                end

                ST_WHO_AM_I: begin
                    case (sub_state)
                        SUB_CS_LOW: begin
                            cs_n      <= 1'b0;
                            sub_state <= SUB_SEND;
                            byte_idx  <= 4'd0;
                        end
                        SUB_SEND: begin
                            if (!spi_busy) begin
                                spi_tx_data <= (byte_idx == 0) ? {1'b1, REG_WHO_AM_I} : 8'hFF;
                                spi_start   <= 1'b1;
                                sub_state   <= SUB_WAIT;
                            end
                        end
                        SUB_WAIT: begin
                            if (spi_done) begin
                                if (byte_idx == 4'd1) begin
                                    sub_state <= SUB_CS_HIGH;
                                end else begin
                                    byte_idx  <= byte_idx + 1'b1;
                                    sub_state <= SUB_SEND;
                                end
                            end
                        end
                        SUB_CS_HIGH: begin
                            cs_n  <= 1'b1;
                            gap_active <= 1'b1;
                            state <= ST_WHO_CHECK;
                        end
                    endcase
                end

                ST_WHO_CHECK: begin
                    if (who_am_i_ok) begin
                        state     <= ST_CFG_SMPLRT;
                        sub_state <= SUB_CS_LOW;
                    end else begin
                        init_fail <= 1'b1;
                    end
                end

                // Config writes: SMPLRT_DIV, CONFIG, GYRO_CONFIG, ACCEL_CONFIG, INT_ENABLE
                // Each follows the same 2-byte SPI write pattern.

                ST_CFG_SMPLRT: begin
                    case (sub_state)
                        SUB_CS_LOW:  begin cs_n <= 1'b0; sub_state <= SUB_SEND; byte_idx <= 4'd0; end
                        SUB_SEND:    begin if (!spi_busy) begin spi_tx_data <= (byte_idx == 0) ? {1'b0, REG_SMPLRT_DIV} : 8'h00; spi_start <= 1'b1; sub_state <= SUB_WAIT; end end
                        SUB_WAIT:    begin if (spi_done) begin if (byte_idx == 4'd1) sub_state <= SUB_CS_HIGH; else begin byte_idx <= byte_idx + 1'b1; sub_state <= SUB_SEND; end end end
                        SUB_CS_HIGH: begin cs_n <= 1'b1; gap_active <= 1'b1; state <= ST_CFG_DLPF; sub_state <= SUB_CS_LOW; end
                    endcase
                end

                ST_CFG_DLPF: begin
                    case (sub_state)
                        SUB_CS_LOW:  begin cs_n <= 1'b0; sub_state <= SUB_SEND; byte_idx <= 4'd0; end
                        SUB_SEND:    begin if (!spi_busy) begin spi_tx_data <= (byte_idx == 0) ? {1'b0, REG_CONFIG} : 8'h03; spi_start <= 1'b1; sub_state <= SUB_WAIT; end end
                        SUB_WAIT:    begin if (spi_done) begin if (byte_idx == 4'd1) sub_state <= SUB_CS_HIGH; else begin byte_idx <= byte_idx + 1'b1; sub_state <= SUB_SEND; end end end
                        SUB_CS_HIGH: begin cs_n <= 1'b1; gap_active <= 1'b1; state <= ST_CFG_GYRO; sub_state <= SUB_CS_LOW; end
                    endcase
                end

                ST_CFG_GYRO: begin
                    case (sub_state)
                        SUB_CS_LOW:  begin cs_n <= 1'b0; sub_state <= SUB_SEND; byte_idx <= 4'd0; end
                        SUB_SEND:    begin if (!spi_busy) begin spi_tx_data <= (byte_idx == 0) ? {1'b0, REG_GYRO_CFG} : {3'b000, GYRO_FS_SEL, 3'b000}; spi_start <= 1'b1; sub_state <= SUB_WAIT; end end
                        SUB_WAIT:    begin if (spi_done) begin if (byte_idx == 4'd1) sub_state <= SUB_CS_HIGH; else begin byte_idx <= byte_idx + 1'b1; sub_state <= SUB_SEND; end end end
                        SUB_CS_HIGH: begin cs_n <= 1'b1; gap_active <= 1'b1; state <= ST_CFG_ACCEL; sub_state <= SUB_CS_LOW; end
                    endcase
                end

                ST_CFG_ACCEL: begin
                    case (sub_state)
                        SUB_CS_LOW:  begin cs_n <= 1'b0; sub_state <= SUB_SEND; byte_idx <= 4'd0; end
                        SUB_SEND:    begin if (!spi_busy) begin spi_tx_data <= (byte_idx == 0) ? {1'b0, REG_ACCEL_CFG} : {3'b000, ACCEL_FS_SEL, 3'b000}; spi_start <= 1'b1; sub_state <= SUB_WAIT; end end
                        SUB_WAIT:    begin if (spi_done) begin if (byte_idx == 4'd1) sub_state <= SUB_CS_HIGH; else begin byte_idx <= byte_idx + 1'b1; sub_state <= SUB_SEND; end end end
                        SUB_CS_HIGH: begin cs_n <= 1'b1; gap_active <= 1'b1; state <= ST_CFG_INT; sub_state <= SUB_CS_LOW; end
                    endcase
                end

                ST_CFG_INT: begin
                    case (sub_state)
                        SUB_CS_LOW:  begin cs_n <= 1'b0; sub_state <= SUB_SEND; byte_idx <= 4'd0; end
                        SUB_SEND:    begin if (!spi_busy) begin spi_tx_data <= (byte_idx == 0) ? {1'b0, REG_INT_ENABLE} : 8'h01; spi_start <= 1'b1; sub_state <= SUB_WAIT; end end
                        SUB_WAIT:    begin if (spi_done) begin if (byte_idx == 4'd1) sub_state <= SUB_CS_HIGH; else begin byte_idx <= byte_idx + 1'b1; sub_state <= SUB_SEND; end end end
                        SUB_CS_HIGH: begin cs_n <= 1'b1; gap_active <= 1'b1; init_done <= 1'b1; state <= ST_IDLE; end
                    endcase
                end

                // ---- Continuous read loop ----

                ST_IDLE: begin
                    if (int_falling) begin
                        state     <= ST_BURST_READ;
                        sub_state <= SUB_CS_LOW;
                        byte_idx  <= 4'd0;
                    end
                end

                // Burst read: 1 address byte + 14 data bytes from 0x3B
                ST_BURST_READ: begin
                    case (sub_state)
                        SUB_CS_LOW: begin
                            cs_n      <= 1'b0;
                            sub_state <= SUB_SEND;
                        end
                        SUB_SEND: begin
                            if (!spi_busy) begin
                                spi_tx_data <= (byte_idx == 0) ? {1'b1, REG_ACCEL_XOUT} : 8'hFF;
                                spi_start   <= 1'b1;
                                sub_state   <= SUB_WAIT;
                            end
                        end
                        SUB_WAIT: begin
                            if (spi_done) begin
                                // Byte 0 is the address response (discard)
                                if (byte_idx > 0)
                                    burst_buf[byte_idx - 1] <= spi_rx_data;

                                if (byte_idx == 4'd14) begin
                                    sub_state <= SUB_CS_HIGH;
                                end else begin
                                    byte_idx  <= byte_idx + 1'b1;
                                    sub_state <= SUB_SEND;
                                end
                            end
                        end
                        SUB_CS_HIGH: begin
                            cs_n       <= 1'b1;
                            gap_active <= 1'b1;
                            state      <= ST_ASSEMBLE;
                        end
                    endcase
                end

                ST_ASSEMBLE: begin
                    accel_x    <= {burst_buf[0],  burst_buf[1]};
                    accel_y    <= {burst_buf[2],  burst_buf[3]};
                    accel_z    <= {burst_buf[4],  burst_buf[5]};
                    temp_raw   <= {burst_buf[6],  burst_buf[7]};
                    gyro_x     <= {burst_buf[8],  burst_buf[9]};
                    gyro_y     <= {burst_buf[10], burst_buf[11]};
                    gyro_z     <= {burst_buf[12], burst_buf[13]};
                    data_valid <= 1'b1;
                    state      <= ST_IDLE;
                end

                default: state <= ST_POR_WAIT;
            endcase

            end // !gap_active
        end // !rst
    end

endmodule
