`timescale 1ns / 1ps

module timestamp_gen #(
    parameter CLK_FREQ = 100_000_000
)(
    input  wire        clk,
    input  wire        rst,
    output reg  [31:0] timestamp_us
);

    localparam PRESCALE = CLK_FREQ / 1_000_000;
    localparam PRE_WIDTH = $clog2(PRESCALE);

    reg [PRE_WIDTH-1:0] pre_cnt;

    always @(posedge clk) begin
        if (rst) begin
            pre_cnt      <= {PRE_WIDTH{1'b0}};
            timestamp_us <= 32'd0;
        end else begin
            if (pre_cnt == PRESCALE[PRE_WIDTH-1:0] - 1) begin
                pre_cnt      <= {PRE_WIDTH{1'b0}};
                timestamp_us <= timestamp_us + 1'b1;
            end else begin
                pre_cnt <= pre_cnt + 1'b1;
            end
        end
    end

endmodule
