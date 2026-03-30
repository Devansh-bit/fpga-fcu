`timescale 1ns / 1ps

module sensor_hub #(
    parameter N_PORTS    = 1,
    parameter DATA_WIDTH = 96,
    parameter TS_WIDTH   = 32,
    parameter FIFO_DEPTH = 16
)(
    input  wire        clk,
    input  wire        rst,

    // Sensor input ports (flat-packed, Verilog-2005)
    input  wire [N_PORTS*DATA_WIDTH-1:0] port_data,
    input  wire [N_PORTS*4-1:0]          port_sensor_type,
    input  wire [N_PORTS*TS_WIDTH-1:0]   port_timestamp,
    input  wire [N_PORTS-1:0]            port_valid,

    // Fusion output
    output reg  [DATA_WIDTH-1:0]         out_data,
    output reg  [3:0]                    out_sensor_type,
    output reg  [TS_WIDTH-1:0]           out_timestamp,
    output reg                           out_valid,
    input  wire                          out_ready,

    // Configuration
    input  wire [N_PORTS-1:0]            fusion_en,

    // Status
    output wire [N_PORTS-1:0]            fifo_empty,
    output wire [N_PORTS-1:0]            fifo_overflow
);

    localparam FIFO_WIDTH = 4 + TS_WIDTH + DATA_WIDTH;
    localparam PORT_WIDTH = $clog2(N_PORTS) > 0 ? $clog2(N_PORTS) : 1;

    // Per-port FIFO signals
    wire [FIFO_WIDTH-1:0] fifo_rd_data  [0:N_PORTS-1];
    wire [N_PORTS-1:0]    fifo_empty_i;
    /* verilator lint_off UNUSEDSIGNAL */
    wire [N_PORTS-1:0]    fifo_full_i;
    /* verilator lint_on UNUSEDSIGNAL */
    reg  [N_PORTS-1:0]    fifo_rd_en;

    genvar gi;
    generate
        for (gi = 0; gi < N_PORTS; gi = gi + 1) begin : gen_fifo
            wire [FIFO_WIDTH-1:0] wr_word = {
                port_sensor_type[gi*4 +: 4],
                port_timestamp[gi*TS_WIDTH +: TS_WIDTH],
                port_data[gi*DATA_WIDTH +: DATA_WIDTH]
            };

            sync_fifo #(
                .DATA_WIDTH(FIFO_WIDTH),
                .DEPTH(FIFO_DEPTH)
            ) u_fifo (
                .clk(clk),
                .rst(rst),
                .wr_en(port_valid[gi]),
                .wr_data(wr_word),
                .rd_en(fifo_rd_en[gi]),
                .rd_data(fifo_rd_data[gi]),
                .empty(fifo_empty_i[gi]),
                .full(fifo_full_i[gi]),
                .overflow(fifo_overflow[gi])
            );
        end
    endgenerate

    assign fifo_empty = fifo_empty_i;

    // Round-robin arbiter
    localparam ST_SCAN    = 2'd0;
    localparam ST_OUTPUT  = 2'd1;
    localparam ST_ADVANCE = 2'd2;

    reg [1:0]              arb_state;
    reg [PORT_WIDTH-1:0]   last_served;
    reg [PORT_WIDTH-1:0]   sel_port;

    // Combinational: find next non-empty, enabled port
    reg [PORT_WIDTH-1:0] next_port;
    reg                  next_found;

    integer ci;
    always @(*) begin
        next_found = 1'b0;
        next_port  = {PORT_WIDTH{1'b0}};
        for (ci = 0; ci < N_PORTS; ci = ci + 1) begin
            // Check ports starting from last_served+1, wrapping
            if (!next_found) begin : scan_block
                reg [PORT_WIDTH-1:0] idx;
                // Modular index: (last_served + 1 + ci) % N_PORTS
                if (N_PORTS == 1)
                    idx = 0;
                else if (last_served + 1 + ci[PORT_WIDTH-1:0] >= N_PORTS[PORT_WIDTH-1:0])
                    idx = last_served + 1 + ci[PORT_WIDTH-1:0] - N_PORTS[PORT_WIDTH-1:0];
                else
                    idx = last_served + 1 + ci[PORT_WIDTH-1:0];

                if (!fifo_empty_i[idx] && fusion_en[idx]) begin
                    next_port  = idx;
                    next_found = 1'b1;
                end
            end
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            arb_state   <= ST_SCAN;
            last_served <= {PORT_WIDTH{1'b0}};
            sel_port    <= {PORT_WIDTH{1'b0}};
            out_data    <= {DATA_WIDTH{1'b0}};
            out_sensor_type <= 4'd0;
            out_timestamp   <= {TS_WIDTH{1'b0}};
            out_valid       <= 1'b0;
            fifo_rd_en      <= {N_PORTS{1'b0}};
        end else begin
            fifo_rd_en <= {N_PORTS{1'b0}};

            case (arb_state)
                ST_SCAN: begin
                    if (next_found) begin
                        // Capture FIFO head combinationally (before rd_ptr advances)
                        out_data        <= fifo_rd_data[next_port][0 +: DATA_WIDTH];
                        out_timestamp   <= fifo_rd_data[next_port][DATA_WIDTH +: TS_WIDTH];
                        out_sensor_type <= fifo_rd_data[next_port][DATA_WIDTH + TS_WIDTH +: 4];
                        out_valid       <= 1'b1;
                        sel_port        <= next_port;
                        arb_state       <= ST_OUTPUT;
                    end else begin
                        out_valid <= 1'b0;
                    end
                end

                ST_OUTPUT: begin
                    if (out_ready) begin
                        fifo_rd_en[sel_port] <= 1'b1;
                        last_served <= sel_port;
                        out_valid   <= 1'b0;
                        arb_state   <= ST_ADVANCE;
                    end
                end

                ST_ADVANCE: begin
                    // 1-cycle settle: FIFO rd_ptr updated, now safe to scan
                    arb_state <= ST_SCAN;
                end

                default: arb_state <= ST_SCAN;
            endcase
        end
    end

endmodule
