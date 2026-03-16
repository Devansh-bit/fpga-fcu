`timescale 1ns / 1ps

module spi_master #(
    parameter CLK_FREQ = 100_000_000,
    parameter SPI_FREQ = 1_000_000,
    parameter CPOL     = 0
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       start,
    input  wire [7:0] tx_data,
    output reg  [7:0] rx_data,
    output reg        done,
    output wire       busy,
    output reg        sclk,
    output reg        mosi,
    input  wire       miso
);

    localparam HALF_PERIOD = CLK_FREQ / (2 * SPI_FREQ);
    localparam DIV_WIDTH   = $clog2(HALF_PERIOD + 1);

    localparam S_IDLE = 2'd0;
    localparam S_CLK0 = 2'd1;
    localparam S_CLK1 = 2'd2;

    reg [1:0]           state;
    reg [DIV_WIDTH-1:0] div_cnt;
    reg [2:0]           bit_idx;
    /* verilator lint_off UNUSEDSIGNAL */
    reg [7:0]           shift_tx;
    /* verilator lint_on UNUSEDSIGNAL */
    reg [7:0]           shift_rx;

    assign busy = (state != S_IDLE);

    reg miso_meta, miso_sync;
    always @(posedge clk) begin
        miso_meta <= miso;
        miso_sync <= miso_meta;
    end

    always @(posedge clk) begin
        if (rst) begin
            state    <= S_IDLE;
            sclk     <= CPOL[0];
            mosi     <= 1'b0;
            done     <= 1'b0;
            rx_data  <= 8'd0;
            div_cnt  <= {DIV_WIDTH{1'b0}};
            bit_idx  <= 3'd0;
            shift_tx <= 8'd0;
            shift_rx <= 8'd0;
        end else begin
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    sclk <= CPOL[0];
                    if (start) begin
                        shift_tx <= tx_data;
                        shift_rx <= 8'd0;
                        bit_idx  <= 3'd7;
                        div_cnt  <= {DIV_WIDTH{1'b0}};
                        mosi     <= tx_data[7];
                        state    <= S_CLK0;
                    end
                end

                S_CLK0: begin
                    if (div_cnt == HALF_PERIOD[DIV_WIDTH-1:0] - 1) begin
                        div_cnt  <= {DIV_WIDTH{1'b0}};
                        sclk     <= ~CPOL[0];
                        shift_rx <= {shift_rx[6:0], miso_sync};
                        state    <= S_CLK1;
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end

                S_CLK1: begin
                    if (div_cnt == HALF_PERIOD[DIV_WIDTH-1:0] - 1) begin
                        div_cnt <= {DIV_WIDTH{1'b0}};
                        sclk    <= CPOL[0];

                        if (bit_idx == 3'd0) begin
                            rx_data <= shift_rx;
                            done    <= 1'b1;
                            state   <= S_IDLE;
                        end else begin
                            shift_tx <= {shift_tx[6:0], 1'b0};
                            mosi     <= shift_tx[6];
                            bit_idx  <= bit_idx - 1'b1;
                            state    <= S_CLK0;
                        end
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
