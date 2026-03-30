`timescale 1ns / 1ps

module tb_sensor_hub;

    localparam N_PORTS    = 4;
    localparam DATA_WIDTH = 16;
    localparam TS_WIDTH   = 8;
    localparam FIFO_DEPTH = 4;

    reg clk, rst;
    initial clk = 0;
    always #5 clk = ~clk;

    reg [N_PORTS*DATA_WIDTH-1:0] port_data;
    reg [N_PORTS*4-1:0]          port_sensor_type;
    reg [N_PORTS*TS_WIDTH-1:0]   port_timestamp;
    reg [N_PORTS-1:0]            port_valid;

    wire [DATA_WIDTH-1:0]        out_data;
    wire [3:0]                   out_sensor_type;
    wire [TS_WIDTH-1:0]          out_timestamp;
    wire                         out_valid;
    reg                          out_ready;

    reg  [N_PORTS-1:0]           fusion_en;
    wire [N_PORTS-1:0]           fifo_empty;
    wire [N_PORTS-1:0]           fifo_overflow;

    sensor_hub #(
        .N_PORTS(N_PORTS),
        .DATA_WIDTH(DATA_WIDTH),
        .TS_WIDTH(TS_WIDTH),
        .FIFO_DEPTH(FIFO_DEPTH)
    ) uut (
        .clk(clk),
        .rst(rst),
        .port_data(port_data),
        .port_sensor_type(port_sensor_type),
        .port_timestamp(port_timestamp),
        .port_valid(port_valid),
        .out_data(out_data),
        .out_sensor_type(out_sensor_type),
        .out_timestamp(out_timestamp),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .fusion_en(fusion_en),
        .fifo_empty(fifo_empty),
        .fifo_overflow(fifo_overflow)
    );

    integer pass_cnt, fail_cnt;

    task check_out(input [DATA_WIDTH-1:0] exp_data, input [3:0] exp_type, input [TS_WIDTH-1:0] exp_ts, input [8*40-1:0] label);
        begin
            if (out_data === exp_data && out_sensor_type === exp_type && out_timestamp === exp_ts)
                pass_cnt = pass_cnt + 1;
            else begin
                fail_cnt = fail_cnt + 1;
                $display("FAIL [%0s]: data=%04h(exp %04h) type=%0d(exp %0d) ts=%02h(exp %02h) at %0t",
                         label, out_data, exp_data, out_sensor_type, exp_type, out_timestamp, exp_ts, $time);
            end
        end
    endtask

    task check_flag(input actual, input expected, input [8*40-1:0] label);
        begin
            if (actual === expected)
                pass_cnt = pass_cnt + 1;
            else begin
                fail_cnt = fail_cnt + 1;
                $display("FAIL [%0s]: expected %0b, got %0b at %0t", label, expected, actual, $time);
            end
        end
    endtask

    // Helper: write one sample to a port
    task write_port(input integer p, input [DATA_WIDTH-1:0] data, input [3:0] stype, input [TS_WIDTH-1:0] ts);
        begin
            port_data[p*DATA_WIDTH +: DATA_WIDTH] = data;
            port_sensor_type[p*4 +: 4]            = stype;
            port_timestamp[p*TS_WIDTH +: TS_WIDTH] = ts;
            port_valid[p]                          = 1'b1;
        end
    endtask

    // Helper: wait for out_valid, then check and acknowledge
    task wait_and_check(input [DATA_WIDTH-1:0] exp_data, input [3:0] exp_type, input [TS_WIDTH-1:0] exp_ts, input [8*40-1:0] label);
        integer timeout;
        begin
            timeout = 0;
            while (!out_valid && timeout < 20) begin
                @(posedge clk); #1;
                timeout = timeout + 1;
            end
            if (!out_valid) begin
                fail_cnt = fail_cnt + 1;
                $display("FAIL [%0s]: timeout waiting for out_valid at %0t", label, $time);
            end else begin
                check_out(exp_data, exp_type, exp_ts, label);
                out_ready = 1'b1;
                @(posedge clk); #1;
                out_ready = 1'b0;
            end
        end
    endtask

    task do_reset;
        begin
            rst = 1;
            port_valid = {N_PORTS{1'b0}};
            port_data = {(N_PORTS*DATA_WIDTH){1'b0}};
            port_sensor_type = {(N_PORTS*4){1'b0}};
            port_timestamp = {(N_PORTS*TS_WIDTH){1'b0}};
            out_ready = 0;
            fusion_en = {N_PORTS{1'b1}};
            @(posedge clk); #1;
            @(posedge clk); #1;
            rst = 0;
            @(posedge clk); #1;
        end
    endtask

    initial begin
        $dumpfile("tb_sensor_hub.vcd");
        $dumpvars(0, tb_sensor_hub);

        pass_cnt = 0;
        fail_cnt = 0;

        // ============================================
        // Test 1: Single port, single sample
        // ============================================
        do_reset;

        write_port(0, 16'hCAFE, 4'd7, 8'h01);
        @(posedge clk); #1;
        port_valid = {N_PORTS{1'b0}};

        wait_and_check(16'hCAFE, 4'd7, 8'h01, "T1: single sample");

        // ============================================
        // Test 2: Single port, burst of 4
        // ============================================
        do_reset;

        write_port(0, 16'h0001, 4'd7, 8'h10);
        @(posedge clk); #1; port_valid = 0;
        write_port(0, 16'h0002, 4'd7, 8'h11);
        @(posedge clk); #1; port_valid = 0;
        write_port(0, 16'h0003, 4'd7, 8'h12);
        @(posedge clk); #1; port_valid = 0;
        write_port(0, 16'h0004, 4'd7, 8'h13);
        @(posedge clk); #1; port_valid = 0;

        wait_and_check(16'h0001, 4'd7, 8'h10, "T2: burst[0]");
        wait_and_check(16'h0002, 4'd7, 8'h11, "T2: burst[1]");
        wait_and_check(16'h0003, 4'd7, 8'h12, "T2: burst[2]");
        wait_and_check(16'h0004, 4'd7, 8'h13, "T2: burst[3]");

        // ============================================
        // Test 3: FIFO overflow
        // ============================================
        do_reset;
        // Disable fusion during writes so arbiter doesn't latch pre-overflow data
        fusion_en = 4'b0000;

        // Write DEPTH+2 = 6 entries without reading
        write_port(0, 16'hA001, 4'd7, 8'h20);
        @(posedge clk); #1; port_valid = 0;
        write_port(0, 16'hA002, 4'd7, 8'h21);
        @(posedge clk); #1; port_valid = 0;
        write_port(0, 16'hA003, 4'd7, 8'h22);
        @(posedge clk); #1; port_valid = 0;
        write_port(0, 16'hA004, 4'd7, 8'h23);
        @(posedge clk); #1; port_valid = 0;
        write_port(0, 16'hA005, 4'd7, 8'h24);
        @(posedge clk); #1; port_valid = 0;
        write_port(0, 16'hA006, 4'd7, 8'h25);
        @(posedge clk); #1; port_valid = 0;

        check_flag(fifo_overflow[0], 1'b1, "T3: overflow flag");

        // Enable fusion, oldest 2 dropped, should get A003..A006
        fusion_en = 4'b0001;
        wait_and_check(16'hA003, 4'd7, 8'h22, "T3: overflow rd[0]");
        wait_and_check(16'hA004, 4'd7, 8'h23, "T3: overflow rd[1]");
        wait_and_check(16'hA005, 4'd7, 8'h24, "T3: overflow rd[2]");
        wait_and_check(16'hA006, 4'd7, 8'h25, "T3: overflow rd[3]");

        // ============================================
        // Test 4: Fusion mask — disabled port blocked
        // ============================================
        do_reset;
        fusion_en = 4'b0000;

        write_port(0, 16'hBEEF, 4'd7, 8'h30);
        @(posedge clk); #1; port_valid = 0;
        @(posedge clk); #1;

        // Give arbiter time to scan — should NOT produce out_valid
        repeat(5) @(posedge clk);
        #1;
        check_flag(out_valid, 1'b0, "T4: masked port no output");
        check_flag(fifo_empty[0], 1'b0, "T4: data still buffered");

        // Enable fusion — should now produce output
        fusion_en = 4'b0001;
        wait_and_check(16'hBEEF, 4'd7, 8'h30, "T4: unmasked output");

        // ============================================
        // Test 5: Multi-port round-robin
        // ============================================
        do_reset;

        // Write one sample to each port simultaneously
        write_port(0, 16'hD000, 4'd0, 8'h40);
        write_port(1, 16'hD001, 4'd1, 8'h41);
        write_port(2, 16'hD002, 4'd2, 8'h42);
        write_port(3, 16'hD003, 4'd3, 8'h43);
        @(posedge clk); #1;
        port_valid = {N_PORTS{1'b0}};

        // Round-robin from last_served=0, so first served should be port 1, then 2, 3, 0
        // Actually after reset last_served=0, so scan starts at 1
        wait_and_check(16'hD001, 4'd1, 8'h41, "T5: rr port 1");
        wait_and_check(16'hD002, 4'd2, 8'h42, "T5: rr port 2");
        wait_and_check(16'hD003, 4'd3, 8'h43, "T5: rr port 3");
        wait_and_check(16'hD000, 4'd0, 8'h40, "T5: rr port 0");

        // ============================================
        // Test 6: Back-pressure — out_ready stays low
        // ============================================
        do_reset;

        write_port(0, 16'hF001, 4'd7, 8'h50);
        @(posedge clk); #1; port_valid = 0;
        write_port(0, 16'hF002, 4'd7, 8'h51);
        @(posedge clk); #1; port_valid = 0;

        // Wait for out_valid but don't assert out_ready
        repeat(5) @(posedge clk); #1;
        check_flag(out_valid, 1'b1, "T6: valid held during backpressure");

        // Now accept
        wait_and_check(16'hF001, 4'd7, 8'h50, "T6: bp release[0]");
        wait_and_check(16'hF002, 4'd7, 8'h51, "T6: bp release[1]");

        // ============================================
        // Results
        // ============================================
        $display("");
        $display("=== tb_sensor_hub: %0d passed, %0d failed ===", pass_cnt, fail_cnt);
        if (fail_cnt == 0)
            $display("ALL TESTS PASSED");
        else
            $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
