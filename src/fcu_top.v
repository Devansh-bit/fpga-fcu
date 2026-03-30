`timescale 1ns / 1ps

module fcu_top (
    input  wire       CLK100MHZ,
    output wire [3:0] led,
    output wire       led0_r, led0_g, led0_b,
    output wire       led1_r, led1_g, led1_b,
    output wire       led2_r, led2_g, led2_b,
    output wire       led3_r, led3_g, led3_b,
    output wire       imu_sclk,
    output wire       imu_mosi,
    input  wire       imu_miso,
    output wire       imu_cs_n,
    input  wire       imu_int_n,
    output wire       uart_rxd_out
);

    // Power-on reset: hold high for 2^16 clocks (~655 us)
    reg [15:0] rst_cnt = 16'd0;
    reg        rst     = 1'b1;
    always @(posedge CLK100MHZ) begin
        if (!rst_cnt[15]) begin
            rst_cnt <= rst_cnt + 1'b1;
            rst     <= 1'b1;
        end else
            rst <= 1'b0;
    end

    wire       spi_start;
    wire [7:0] spi_tx_data;
    wire [7:0] spi_rx_data;
    wire       spi_done;
    wire       spi_busy;

    spi_master #(
        .CLK_FREQ(100_000_000),
        .SPI_FREQ(1_000_000)
    ) u_spi (
        .clk(CLK100MHZ),
        .rst(rst),
        .start(spi_start),
        .tx_data(spi_tx_data),
        .rx_data(spi_rx_data),
        .done(spi_done),
        .busy(spi_busy),
        .sclk(imu_sclk),
        .mosi(imu_mosi),
        .miso(imu_miso)
    );

    wire signed [15:0] accel_x, accel_y, accel_z;
    wire signed [15:0] gyro_x,  gyro_y,  gyro_z;
    wire signed [15:0] temp_raw;
    wire               data_valid;
    wire               init_done;
    wire               init_fail;

    mpu_driver #(
        .CLK_FREQ(100_000_000),
        .ACCEL_FS_SEL(2'b00),
        .GYRO_FS_SEL(2'b11)
    ) u_mpu (
        .clk(CLK100MHZ),
        .rst(rst),
        .spi_start(spi_start),
        .spi_tx_data(spi_tx_data),
        .spi_rx_data(spi_rx_data),
        .spi_done(spi_done),
        .spi_busy(spi_busy),
        .cs_n(imu_cs_n),
        .int_n(imu_int_n),
        .accel_x(accel_x),
        .accel_y(accel_y),
        .accel_z(accel_z),
        .gyro_x(gyro_x),
        .gyro_y(gyro_y),
        .gyro_z(gyro_z),
        .temp_raw(temp_raw),
        .data_valid(data_valid),
        .init_done(init_done),
        .init_fail(init_fail)
    );

    // Shared microsecond timestamp
    wire [31:0] timestamp_us;

    timestamp_gen #(
        .CLK_FREQ(100_000_000)
    ) u_ts (
        .clk(CLK100MHZ),
        .rst(rst),
        .timestamp_us(timestamp_us)
    );

    // IMU sensor port adapter
    wire [95:0] imu_port_data;
    wire [3:0]  imu_port_sensor_type;
    wire [31:0] imu_port_timestamp;
    wire        imu_port_valid;

    imu_sensor_port #(
        .DATA_WIDTH(96),
        .TS_WIDTH(32)
    ) u_imu_port (
        .clk(CLK100MHZ),
        .rst(rst),
        .accel_x(accel_x),
        .accel_y(accel_y),
        .accel_z(accel_z),
        .gyro_x(gyro_x),
        .gyro_y(gyro_y),
        .gyro_z(gyro_z),
        .data_valid(data_valid),
        .timestamp_us(timestamp_us),
        .port_data(imu_port_data),
        .port_sensor_type(imu_port_sensor_type),
        .port_timestamp(imu_port_timestamp),
        .port_valid(imu_port_valid)
    );

    // Sensor hub (single port for now, outputs consumed by future fusion core)
    /* verilator lint_off UNUSEDSIGNAL */
    wire [95:0] hub_out_data;
    wire [3:0]  hub_out_sensor_type;
    wire [31:0] hub_out_timestamp;
    wire        hub_out_valid;
    wire        hub_fifo_empty;
    /* verilator lint_on UNUSEDSIGNAL */
    wire        hub_fifo_overflow;

    sensor_hub #(
        .N_PORTS(1),
        .DATA_WIDTH(96),
        .TS_WIDTH(32),
        .FIFO_DEPTH(16)
    ) u_hub (
        .clk(CLK100MHZ),
        .rst(rst),
        .port_data(imu_port_data),
        .port_sensor_type(imu_port_sensor_type),
        .port_timestamp(imu_port_timestamp),
        .port_valid(imu_port_valid),
        .out_data(hub_out_data),
        .out_sensor_type(hub_out_sensor_type),
        .out_timestamp(hub_out_timestamp),
        .out_valid(hub_out_valid),
        .out_ready(1'b1),
        .fusion_en(1'b1),
        .fifo_empty(hub_fifo_empty),
        .fifo_overflow(hub_fifo_overflow)
    );

    // UART telemetry: hex dump of all sensor values at 115200 baud
    imu_uart_fmt #(
        .CLK_FREQ(100_000_000),
        .BAUD(115_200)
    ) u_fmt (
        .clk(CLK100MHZ),
        .rst(rst),
        .accel_x(accel_x),
        .accel_y(accel_y),
        .accel_z(accel_z),
        .gyro_x(gyro_x),
        .gyro_y(gyro_y),
        .gyro_z(gyro_z),
        .temp_raw(temp_raw),
        .data_valid(data_valid),
        .uart_tx_pin(uart_rxd_out)
    );

    // LED visualization
    wire [7:0] abs_accel_x = accel_x[15] ? ~accel_x[15:8] : accel_x[15:8];
    wire [7:0] abs_accel_y = accel_y[15] ? ~accel_y[15:8] : accel_y[15:8];
    wire [7:0] abs_accel_z = accel_z[15] ? ~accel_z[15:8] : accel_z[15:8];
    wire [7:0] abs_gyro_x  = gyro_x[15]  ? ~gyro_x[15:8]  : gyro_x[15:8];
    wire [7:0] abs_gyro_y  = gyro_y[15]  ? ~gyro_y[15:8]  : gyro_y[15:8];
    wire [7:0] abs_gyro_z  = gyro_z[15]  ? ~gyro_z[15:8]  : gyro_z[15:8];

    pwm #(.WIDTH(8)) pwm_led0 (.clk(CLK100MHZ), .duty(abs_accel_z), .out(led[0]));
    pwm #(.WIDTH(8)) pwm_led1 (.clk(CLK100MHZ), .duty(abs_accel_z), .out(led[1]));
    pwm #(.WIDTH(8)) pwm_led2 (.clk(CLK100MHZ), .duty(abs_accel_z), .out(led[2]));
    pwm #(.WIDTH(8)) pwm_led3 (.clk(CLK100MHZ), .duty(abs_accel_z), .out(led[3]));

    pwm #(.WIDTH(8)) pwm_rgb0_r (.clk(CLK100MHZ), .duty(abs_accel_x), .out(led0_r));
    pwm #(.WIDTH(8)) pwm_rgb0_g (.clk(CLK100MHZ), .duty(abs_accel_y), .out(led0_g));
    pwm #(.WIDTH(8)) pwm_rgb0_b (.clk(CLK100MHZ), .duty(abs_accel_z), .out(led0_b));

    pwm #(.WIDTH(8)) pwm_rgb1_r (.clk(CLK100MHZ), .duty(abs_gyro_x), .out(led1_r));
    pwm #(.WIDTH(8)) pwm_rgb1_g (.clk(CLK100MHZ), .duty(abs_gyro_y), .out(led1_g));
    pwm #(.WIDTH(8)) pwm_rgb1_b (.clk(CLK100MHZ), .duty(abs_gyro_z), .out(led1_b));

    pwm #(.WIDTH(8)) pwm_rgb2_r (.clk(CLK100MHZ), .duty(init_fail ? 8'hFF : 8'h00), .out(led2_r));
    pwm #(.WIDTH(8)) pwm_rgb2_g (.clk(CLK100MHZ), .duty(init_done ? 8'hFF : 8'h00), .out(led2_g));
    assign led2_b = 1'b0;

    reg [21:0] dv_stretch = 22'd0;
    always @(posedge CLK100MHZ) begin
        if (data_valid)
            dv_stretch <= {22{1'b1}};
        else if (dv_stretch != 22'd0)
            dv_stretch <= dv_stretch - 1'b1;
    end
    pwm #(.WIDTH(8)) pwm_rgb3_g (.clk(CLK100MHZ), .duty(dv_stretch[21:14]), .out(led3_g));
    assign led3_r = 1'b0;
    assign led3_b = hub_fifo_overflow;

endmodule
