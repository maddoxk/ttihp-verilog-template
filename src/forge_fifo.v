/*
 * ProtocolForge Synchronous FIFO (First-Word-Fall-Through)
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module forge_fifo #(
    parameter DATA_WIDTH = 16,
    parameter ADDR_WIDTH = 3   // Depth = 1 << ADDR_WIDTH (e.g. 3 => 8 words, 2 => 4 words)
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  clear,

    // Write Port
    input  wire [DATA_WIDTH-1:0] wdata,
    input  wire                  push,
    output wire                  full,

    // Read Port
    output wire [DATA_WIDTH-1:0] rdata,
    input  wire                  pop,
    output wire                  empty,

    // Status
    output wire [ADDR_WIDTH:0]   level
);

    localparam DEPTH = 1 << ADDR_WIDTH;

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH:0]   wptr;
    reg [ADDR_WIDTH:0]   rptr;

    wire push_en = push && (!full || pop);
    wire pop_en  = pop  && !empty;

    // Full & Empty Detection using extra MSB
    assign empty = (wptr == rptr);
    assign full  = (wptr[ADDR_WIDTH] != rptr[ADDR_WIDTH]) &&
                   (wptr[ADDR_WIDTH-1:0] == rptr[ADDR_WIDTH-1:0]);

    // Current FIFO level
    assign level = wptr - rptr;

    // First-Word-Fall-Through: rdata reflects current head of FIFO
    assign rdata = mem[rptr[ADDR_WIDTH-1:0]];

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr <= {(ADDR_WIDTH+1){1'b0}};
            rptr <= {(ADDR_WIDTH+1){1'b0}};
            for (i = 0; i < DEPTH; i = i + 1) begin
                mem[i] <= {DATA_WIDTH{1'b0}};
            end
        end else if (clear) begin
            wptr <= {(ADDR_WIDTH+1){1'b0}};
            rptr <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            if (push_en) begin
                mem[wptr[ADDR_WIDTH-1:0]] <= wdata;
                wptr <= wptr + 1'b1;
            end
            if (pop_en) begin
                rptr <= rptr + 1'b1;
            end
        end
    end

endmodule
