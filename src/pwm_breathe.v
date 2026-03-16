`timescale 1ns / 1ps

module pwm_breathe (
    input  wire       CLK100MHZ,
    output wire [3:0] led,
    output wire       led0_r, led0_g, led0_b,
    output wire       led1_r, led1_g, led1_b,
    output wire       led2_r, led2_g, led2_b,
    output wire       led3_r, led3_g, led3_b
);

    // Sawtooth ramp rates (clocks per duty step, 8-bit → 256 steps)
    //   green LEDs:  390_625 → 1.000 s period
    //   red channel: 350_000 → 0.896 s period
    //   grn channel: 420_000 → 1.075 s period
    //   blu channel: 470_000 → 1.203 s period
    localparam STEP_GREEN = 390_625 - 1;
    localparam STEP_RED   = 350_000 - 1;
    localparam STEP_GRN   = 420_000 - 1;
    localparam STEP_BLU   = 470_000 - 1;

    // --- Sawtooth duty generators -------------------------------------------

    reg [7:0]  duty_led  = 8'd0;
    reg [18:0] cnt_led   = 19'd0;

    reg [7:0]  duty_r    = 8'd0;
    reg [18:0] cnt_r     = 19'd0;

    reg [7:0]  duty_g    = 8'd0;
    reg [18:0] cnt_g     = 19'd0;

    reg [7:0]  duty_b    = 8'd0;
    reg [18:0] cnt_b     = 19'd0;

    always @(posedge CLK100MHZ) begin
        // Green LEDs ramp
        if (cnt_led == STEP_GREEN[18:0]) begin
            cnt_led  <= 19'd0;
            duty_led <= duty_led + 1'b1;
        end else
            cnt_led <= cnt_led + 1'b1;

        // Red channel ramp
        if (cnt_r == STEP_RED[18:0]) begin
            cnt_r  <= 19'd0;
            duty_r <= duty_r + 1'b1;
        end else
            cnt_r <= cnt_r + 1'b1;

        // Green channel ramp
        if (cnt_g == STEP_GRN[18:0]) begin
            cnt_g  <= 19'd0;
            duty_g <= duty_g + 1'b1;
        end else
            cnt_g <= cnt_g + 1'b1;

        // Blue channel ramp
        if (cnt_b == STEP_BLU[18:0]) begin
            cnt_b  <= 19'd0;
            duty_b <= duty_b + 1'b1;
        end else
            cnt_b <= cnt_b + 1'b1;
    end

    // --- PWM instances: 4 green LEDs ----------------------------------------

    pwm #(.WIDTH(8)) pwm_led0 (.clk(CLK100MHZ), .duty(duty_led), .out(led[0]));
    pwm #(.WIDTH(8)) pwm_led1 (.clk(CLK100MHZ), .duty(duty_led), .out(led[1]));
    pwm #(.WIDTH(8)) pwm_led2 (.clk(CLK100MHZ), .duty(duty_led), .out(led[2]));
    pwm #(.WIDTH(8)) pwm_led3 (.clk(CLK100MHZ), .duty(duty_led), .out(led[3]));

    // --- PWM instances: RGB LED 0 -------------------------------------------

    pwm #(.WIDTH(8)) pwm_rgb0_r (.clk(CLK100MHZ), .duty(duty_r), .out(led0_r));
    pwm #(.WIDTH(8)) pwm_rgb0_g (.clk(CLK100MHZ), .duty(duty_g), .out(led0_g));
    pwm #(.WIDTH(8)) pwm_rgb0_b (.clk(CLK100MHZ), .duty(duty_b), .out(led0_b));

    // --- PWM instances: RGB LED 1 -------------------------------------------

    pwm #(.WIDTH(8)) pwm_rgb1_r (.clk(CLK100MHZ), .duty(duty_r), .out(led1_r));
    pwm #(.WIDTH(8)) pwm_rgb1_g (.clk(CLK100MHZ), .duty(duty_g), .out(led1_g));
    pwm #(.WIDTH(8)) pwm_rgb1_b (.clk(CLK100MHZ), .duty(duty_b), .out(led1_b));

    // --- PWM instances: RGB LED 2 -------------------------------------------

    pwm #(.WIDTH(8)) pwm_rgb2_r (.clk(CLK100MHZ), .duty(duty_r), .out(led2_r));
    pwm #(.WIDTH(8)) pwm_rgb2_g (.clk(CLK100MHZ), .duty(duty_g), .out(led2_g));
    pwm #(.WIDTH(8)) pwm_rgb2_b (.clk(CLK100MHZ), .duty(duty_b), .out(led2_b));

    // --- PWM instances: RGB LED 3 -------------------------------------------

    pwm #(.WIDTH(8)) pwm_rgb3_r (.clk(CLK100MHZ), .duty(duty_r), .out(led3_r));
    pwm #(.WIDTH(8)) pwm_rgb3_g (.clk(CLK100MHZ), .duty(duty_g), .out(led3_g));
    pwm #(.WIDTH(8)) pwm_rgb3_b (.clk(CLK100MHZ), .duty(duty_b), .out(led3_b));

endmodule
