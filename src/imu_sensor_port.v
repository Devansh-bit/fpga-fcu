`timescale 1ns / 1ps

module imu_sensor_port #(
    parameter DATA_WIDTH = 96,
    parameter TS_WIDTH   = 32
)(
    input  wire        clk,
    input  wire        rst,

    // From mpu_driver
    input  wire signed [15:0] accel_x,
    input  wire signed [15:0] accel_y,
    input  wire signed [15:0] accel_z,
    input  wire signed [15:0] gyro_x,
    input  wire signed [15:0] gyro_y,
    input  wire signed [15:0] gyro_z,
    input  wire               data_valid,

    // From timestamp_gen
    input  wire [TS_WIDTH-1:0] timestamp_us,

    // To sensor_hub port
    output reg  [DATA_WIDTH-1:0] port_data,
    output reg  [3:0]            port_sensor_type,
    output reg  [TS_WIDTH-1:0]   port_timestamp,
    output reg                   port_valid
);

    localparam STYPE_IMU = 4'd7;

    always @(posedge clk) begin
        if (rst) begin
            port_data        <= {DATA_WIDTH{1'b0}};
            port_sensor_type <= 4'd0;
            port_timestamp   <= {TS_WIDTH{1'b0}};
            port_valid       <= 1'b0;
        end else begin
            port_valid <= 1'b0;
            if (data_valid) begin
                port_data        <= {accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z};
                port_sensor_type <= STYPE_IMU;
                port_timestamp   <= timestamp_us;
                port_valid       <= 1'b1;
            end
        end
    end

endmodule
