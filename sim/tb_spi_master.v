`timescale 1ns / 1ps

module tb_spi_master;

    reg        clk   = 0;
    reg        rst   = 1;
    reg        start = 0;
    reg  [7:0] tx_data = 8'd0;
    wire [7:0] rx_data;
    wire       done;
    wire       busy;
    wire       sclk_w;
    wire       mosi_w;
    reg        miso  = 0;

    always #5 clk = ~clk;  // 100 MHz

    spi_master #(
        .CLK_FREQ(100_000_000),
        .SPI_FREQ(1_000_000),
        .CPOL(0)
    ) uut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .tx_data(tx_data),
        .rx_data(rx_data),
        .done(done),
        .busy(busy),
        .sclk(sclk_w),
        .mosi(mosi_w),
        .miso(miso)
    );

    // ---- Behavioral SPI slave (Mode 0) ----
    // Drives MISO on falling SCLK, samples MOSI on rising SCLK.
    // Uses negedge/posedge of sclk_w directly (behavioral, not synthesizable).

    reg [7:0] slave_rx;           // byte slave received
    reg [7:0] slave_shift_out;
    reg [7:0] slave_shift_in;
    reg [2:0] slave_bit_cnt;

    // Sample MOSI on rising SCLK
    always @(posedge sclk_w) begin
        slave_shift_in = {slave_shift_in[6:0], mosi_w};
        slave_bit_cnt  = slave_bit_cnt + 1;  // 0→1→...→7→0 (wraps after 8th edge)
        if (slave_bit_cnt == 0)
            slave_rx = slave_shift_in;
    end

    // Shift MISO on falling SCLK
    always @(negedge sclk_w) begin
        slave_shift_out = {slave_shift_out[6:0], 1'b0};
        miso = slave_shift_out[7];
    end

    // --- Test infrastructure ---
    integer pass_count = 0;
    integer fail_count = 0;

    task spi_xfer(
        input [7:0] master_tx,
        input [7:0] slave_resp,
        input [7:0] expect_master_rx,
        input [7:0] expect_slave_rx,
        input [31:0] test_num
    );
        begin
            // Load slave response
            slave_shift_out = slave_resp;
            slave_shift_in  = 8'd0;
            slave_bit_cnt   = 3'd0;
            miso            = slave_resp[7];  // present MSB before first rising edge

            // Start master transfer
            @(posedge clk);
            tx_data = master_tx;
            start   = 1'b1;
            @(posedge clk);
            start = 1'b0;

            // Wait for completion
            @(posedge done);
            @(posedge clk);

            // Check results
            if (rx_data !== expect_master_rx) begin
                $display("FAIL test %0d: master rx = 0x%02h, expected 0x%02h",
                         test_num, rx_data, expect_master_rx);
                fail_count = fail_count + 1;
            end else if (slave_rx !== expect_slave_rx) begin
                $display("FAIL test %0d: slave rx = 0x%02h, expected 0x%02h",
                         test_num, slave_rx, expect_slave_rx);
                fail_count = fail_count + 1;
            end else begin
                $display("PASS test %0d: master_tx=0x%02h master_rx=0x%02h slave_rx=0x%02h",
                         test_num, master_tx, rx_data, slave_rx);
                pass_count = pass_count + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("tb_spi_master.vcd");
        $dumpvars(0, tb_spi_master);

        #100;
        rst = 0;
        #100;

        // Test 1: basic exchange
        spi_xfer(8'hA5, 8'h3C, 8'h3C, 8'hA5, 1);

        // Test 2: all ones / all zeros
        spi_xfer(8'hFF, 8'h00, 8'h00, 8'hFF, 2);

        // Test 3: all zeros / all ones
        spi_xfer(8'h00, 8'hFF, 8'hFF, 8'h00, 3);

        // Test 4: same byte both directions
        spi_xfer(8'h71, 8'h71, 8'h71, 8'h71, 4);

        // Test 5: alternating bits
        spi_xfer(8'h55, 8'hAA, 8'hAA, 8'h55, 5);

        #200;
        $display("-----------------------------");
        $display("Results: %0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
