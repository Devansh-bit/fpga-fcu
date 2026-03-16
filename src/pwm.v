`timescale 1ns / 1ps

module pwm #(
    parameter WIDTH  = 8,
    parameter PERIOD = 0
)(
    input  wire             clk,
    input  wire [WIDTH-1:0] duty,
    output wire             out
);

    reg [WIDTH-1:0] counter = {WIDTH{1'b0}};

    generate
        if (PERIOD == 0) begin : gen_freerun
            always @(posedge clk)
                counter <= counter + 1'b1;
        end else begin : gen_fixed
            always @(posedge clk) begin
                if (counter == PERIOD[WIDTH-1:0] - 1)
                    counter <= {WIDTH{1'b0}};
                else
                    counter <= counter + 1'b1;
            end
        end
    endgenerate

    assign out = (counter < duty);

endmodule
