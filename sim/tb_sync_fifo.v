`timescale 1ns / 1ps

module tb_sync_fifo;

    parameter DATA_WIDTH = 8;
    parameter DEPTH      = 4;
    parameter ADDR_WIDTH = $clog2(DEPTH);

    reg                  clk;
    reg                  rst;
    reg                  wr_en;
    reg [DATA_WIDTH-1:0] wr_data;
    reg                  rd_en;
    wire [DATA_WIDTH-1:0] rd_data;
    wire                 empty;
    wire                 full;
    wire                 overflow;

    sync_fifo #(
        .DATA_WIDTH(DATA_WIDTH),
        .DEPTH(DEPTH)
    ) uut (
        .clk(clk),
        .rst(rst),
        .wr_en(wr_en),
        .wr_data(wr_data),
        .rd_en(rd_en),
        .rd_data(rd_data),
        .empty(empty),
        .full(full),
        .overflow(overflow)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    integer pass_cnt, fail_cnt;

    task check(input [DATA_WIDTH-1:0] expected, input [8*32-1:0] label);
        begin
            if (rd_data === expected)
                pass_cnt = pass_cnt + 1;
            else begin
                fail_cnt = fail_cnt + 1;
                $display("FAIL [%0s]: expected %0d, got %0d at time %0t", label, expected, rd_data, $time);
            end
        end
    endtask

    task check_flag(input actual, input expected, input [8*32-1:0] label);
        begin
            if (actual === expected)
                pass_cnt = pass_cnt + 1;
            else begin
                fail_cnt = fail_cnt + 1;
                $display("FAIL [%0s]: expected %0b, got %0b at time %0t", label, expected, actual, $time);
            end
        end
    endtask

    initial begin
        $dumpfile("tb_sync_fifo.vcd");
        $dumpvars(0, tb_sync_fifo);

        pass_cnt = 0;
        fail_cnt = 0;
        rst    = 1;
        wr_en  = 0;
        rd_en  = 0;
        wr_data = 0;

        @(posedge clk); #1;
        @(posedge clk); #1;
        rst = 0;
        @(posedge clk); #1;

        // === Test 1: Empty after reset ===
        check_flag(empty, 1'b1, "empty after reset");
        check_flag(full,  1'b0, "not full after reset");
        check_flag(overflow, 1'b0, "no overflow after reset");

        // === Test 2: Write and read single entry ===
        wr_en = 1; wr_data = 8'hAA;
        @(posedge clk); #1;
        wr_en = 0;
        @(posedge clk); #1;
        check_flag(empty, 1'b0, "not empty after write");
        check(8'hAA, "rd_data before read");

        rd_en = 1;
        @(posedge clk); #1;
        rd_en = 0;
        @(posedge clk); #1;
        check_flag(empty, 1'b1, "empty after read");

        // === Test 3: Fill to capacity ===
        wr_en = 1;
        wr_data = 8'h10; @(posedge clk); #1;
        wr_data = 8'h20; @(posedge clk); #1;
        wr_data = 8'h30; @(posedge clk); #1;
        wr_data = 8'h40; @(posedge clk); #1;
        wr_en = 0;
        @(posedge clk); #1;
        check_flag(full, 1'b1, "full after 4 writes");
        check_flag(overflow, 1'b0, "no overflow at capacity");

        // Read back in order — check rd_data before posedge advances rd_ptr
        check(8'h10, "fifo[0]");
        rd_en = 1;
        @(posedge clk); #1; check(8'h20, "fifo[1]");
        @(posedge clk); #1; check(8'h30, "fifo[2]");
        @(posedge clk); #1; check(8'h40, "fifo[3]");
        @(posedge clk); #1;
        rd_en = 0;
        check_flag(empty, 1'b1, "empty after draining");

        // === Test 4: Overflow — write DEPTH+2 without reading ===
        wr_en = 1;
        wr_data = 8'h01; @(posedge clk); #1;
        wr_data = 8'h02; @(posedge clk); #1;
        wr_data = 8'h03; @(posedge clk); #1;
        wr_data = 8'h04; @(posedge clk); #1;
        // FIFO is now full with [01, 02, 03, 04]
        // Write 2 more — should overwrite oldest
        wr_data = 8'h05; @(posedge clk); #1;
        wr_data = 8'h06; @(posedge clk); #1;
        wr_en = 0;
        @(posedge clk); #1;
        check_flag(overflow, 1'b1, "overflow flagged");
        check_flag(full, 1'b1, "still full after overflow");

        // Oldest 2 (01, 02) should be dropped. Read should give 03, 04, 05, 06
        check(8'h03, "overflow rd[0]");
        rd_en = 1;
        @(posedge clk); #1; check(8'h04, "overflow rd[1]");
        @(posedge clk); #1; check(8'h05, "overflow rd[2]");
        @(posedge clk); #1; check(8'h06, "overflow rd[3]");
        @(posedge clk); #1;
        rd_en = 0;
        check_flag(empty, 1'b1, "empty after overflow drain");

        // === Test 5: Read from empty FIFO — no-op ===
        rd_en = 1;
        @(posedge clk); #1;
        rd_en = 0;
        @(posedge clk); #1;
        check_flag(empty, 1'b1, "still empty after empty read");

        // === Test 6: Simultaneous read + write ===
        // First put one entry in
        rst = 1;
        @(posedge clk); #1;
        rst = 0;
        @(posedge clk); #1;

        wr_en = 1; wr_data = 8'hB0;
        @(posedge clk); #1;
        wr_en = 0;
        @(posedge clk); #1;
        // Now simultaneously write and read
        wr_en = 1; rd_en = 1; wr_data = 8'hB1;
        @(posedge clk); #1;
        wr_en = 0; rd_en = 0;
        @(posedge clk); #1;
        // Should have B1 in fifo (B0 was read out, B1 was written)
        check(8'hB1, "simul rw result");
        check_flag(empty, 1'b0, "not empty after simul rw");

        // Drain
        rd_en = 1;
        @(posedge clk); #1;
        rd_en = 0;
        @(posedge clk); #1;
        check_flag(empty, 1'b1, "empty after final drain");

        // === Results ===
        $display("");
        $display("=== tb_sync_fifo: %0d passed, %0d failed ===", pass_cnt, fail_cnt);
        if (fail_cnt == 0)
            $display("ALL TESTS PASSED");
        else
            $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
