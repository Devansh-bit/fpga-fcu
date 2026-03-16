`timescale 1ns / 1ps

module uart_tx #(
    parameter CLK_FREQ = 100_000_000,
    parameter BAUD     = 115_200
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       start,
    input  wire [7:0] data,
    output reg        tx,
    output wire       busy
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD;
    localparam DIV_WIDTH    = $clog2(CLKS_PER_BIT + 1);

    localparam S_IDLE  = 2'd0;
    localparam S_START = 2'd1;
    localparam S_DATA  = 2'd2;
    localparam S_STOP  = 2'd3;

    reg [1:0]           state;
    reg [DIV_WIDTH-1:0] div_cnt;
    reg [2:0]           bit_idx;
    reg [7:0]           shift;

    assign busy = (state != S_IDLE);

    always @(posedge clk) begin
        if (rst) begin
            state   <= S_IDLE;
            tx      <= 1'b1;
            div_cnt <= {DIV_WIDTH{1'b0}};
            bit_idx <= 3'd0;
            shift   <= 8'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    tx <= 1'b1;
                    if (start) begin
                        shift   <= data;
                        div_cnt <= {DIV_WIDTH{1'b0}};
                        state   <= S_START;
                    end
                end

                S_START: begin
                    tx <= 1'b0;
                    if (div_cnt == CLKS_PER_BIT[DIV_WIDTH-1:0] - 1) begin
                        div_cnt <= {DIV_WIDTH{1'b0}};
                        bit_idx <= 3'd0;
                        state   <= S_DATA;
                    end else
                        div_cnt <= div_cnt + 1'b1;
                end

                S_DATA: begin
                    tx <= shift[0];
                    if (div_cnt == CLKS_PER_BIT[DIV_WIDTH-1:0] - 1) begin
                        div_cnt <= {DIV_WIDTH{1'b0}};
                        shift   <= {1'b0, shift[7:1]};
                        if (bit_idx == 3'd7)
                            state <= S_STOP;
                        else
                            bit_idx <= bit_idx + 1'b1;
                    end else
                        div_cnt <= div_cnt + 1'b1;
                end

                S_STOP: begin
                    tx <= 1'b1;
                    if (div_cnt == CLKS_PER_BIT[DIV_WIDTH-1:0] - 1) begin
                        div_cnt <= {DIV_WIDTH{1'b0}};
                        state   <= S_IDLE;
                    end else
                        div_cnt <= div_cnt + 1'b1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
