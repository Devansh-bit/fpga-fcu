`timescale 1ns / 1ps

module sync_fifo #(
    parameter DATA_WIDTH = 128,
    parameter DEPTH      = 16,
    parameter ADDR_WIDTH = $clog2(DEPTH)
)(
    input  wire                  clk,
    input  wire                  rst,

    input  wire                  wr_en,
    input  wire [DATA_WIDTH-1:0] wr_data,

    input  wire                  rd_en,
    output wire [DATA_WIDTH-1:0] rd_data,

    output wire                  empty,
    output wire                  full,
    output reg                   overflow
);

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    reg [ADDR_WIDTH-1:0] wr_ptr;
    reg [ADDR_WIDTH-1:0] rd_ptr;
    reg [ADDR_WIDTH:0]   count;

    assign empty   = (count == 0);
    assign full    = (count == DEPTH[ADDR_WIDTH:0]);
    assign rd_data = mem[rd_ptr];

    integer i;

    always @(posedge clk) begin
        if (rst) begin
            wr_ptr   <= {ADDR_WIDTH{1'b0}};
            rd_ptr   <= {ADDR_WIDTH{1'b0}};
            count    <= {(ADDR_WIDTH+1){1'b0}};
            overflow <= 1'b0;
            for (i = 0; i < DEPTH; i = i + 1)
                mem[i] <= {DATA_WIDTH{1'b0}};
        end else begin
            case ({wr_en, rd_en})
                2'b10: begin // write only
                    mem[wr_ptr] <= wr_data;
                    wr_ptr      <= (wr_ptr == DEPTH[ADDR_WIDTH-1:0] - 1) ? {ADDR_WIDTH{1'b0}} : wr_ptr + 1'b1;
                    if (full) begin
                        // overwrite oldest
                        rd_ptr   <= (rd_ptr == DEPTH[ADDR_WIDTH-1:0] - 1) ? {ADDR_WIDTH{1'b0}} : rd_ptr + 1'b1;
                        overflow <= 1'b1;
                    end else begin
                        count <= count + 1'b1;
                    end
                end
                2'b01: begin // read only
                    if (!empty) begin
                        rd_ptr <= (rd_ptr == DEPTH[ADDR_WIDTH-1:0] - 1) ? {ADDR_WIDTH{1'b0}} : rd_ptr + 1'b1;
                        count  <= count - 1'b1;
                    end
                end
                2'b11: begin // simultaneous read + write
                    mem[wr_ptr] <= wr_data;
                    wr_ptr      <= (wr_ptr == DEPTH[ADDR_WIDTH-1:0] - 1) ? {ADDR_WIDTH{1'b0}} : wr_ptr + 1'b1;
                    if (empty) begin
                        // write takes effect, read is no-op
                        count <= count + 1'b1;
                    end else begin
                        rd_ptr <= (rd_ptr == DEPTH[ADDR_WIDTH-1:0] - 1) ? {ADDR_WIDTH{1'b0}} : rd_ptr + 1'b1;
                        // count stays same (one in, one out)
                        if (full)
                            overflow <= 1'b1;
                    end
                end
                default: ; // 2'b00: no-op
            endcase
        end
    end

endmodule
