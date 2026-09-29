/*
 * ProtocolForge Multi-Polynomial Streaming CRC LFSR Coprocessor
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none
`include "forge_defs.vh"

module forge_crc (
    input  wire        clk,
    input  wire        rst_n,

    // Control & Configuration
    input  wire [1:0]  poly_sel,    // 00=CRC-16, 01=CRC-15, 10=CRC-8, 11=CRC-5
    input  wire        invert_out,  // Invert output for USB (XOR 0xFFFF)
    input  wire        clr,         // Force reset to 0
    input  wire        preset,      // Preset to standard polynomial init value

    // Serial Data Stream
    input  wire        crc_en,      // 1-bit clock tick enable
    input  wire        bit_in,      // Serial bit stream input

    // Direct Read / Write
    input  wire        write_en,
    input  wire [15:0] write_data,
    output wire [15:0] crc_out,
    output wire        crc_zero
);

    reg [15:0] crc_reg;
    reg [15:0] next_crc;
    wire       fb16 = bit_in ^ crc_reg[15];
    wire       fb15 = bit_in ^ crc_reg[14];
    wire       fb8  = bit_in ^ crc_reg[7];
    wire       fb5  = bit_in ^ crc_reg[4];

    always @(*) begin
        case (poly_sel)
            `FORGE_CRC_16: begin // Poly: 0x8005 (x^16 + x^15 + x^2 + 1)
                next_crc = {crc_reg[14:0], 1'b0} ^ (fb16 ? 16'h8005 : 16'h0000);
            end
            `FORGE_CRC_15: begin // Poly: 0x4599 (x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1)
                next_crc = {1'b0, crc_reg[13:0], 1'b0} ^ (fb15 ? 16'h4599 : 16'h0000);
            end
            `FORGE_CRC_8: begin  // Poly: 0x31 (x^8 + x^5 + x^4 + 1)
                next_crc = {8'b0, crc_reg[6:0], 1'b0} ^ (fb8 ? 16'h0031 : 16'h0000);
            end
            `FORGE_CRC_5: begin  // Poly: 0x05 (x^5 + x^2 + 1)
                next_crc = {11'b0, crc_reg[3:0], 1'b0} ^ (fb5 ? 16'h0005 : 16'h0000);
            end
            default: next_crc = crc_reg;
        endcase
    end

    wire [15:0] preset_val = (poly_sel == `FORGE_CRC_16) ? 16'hFFFF :
                             (poly_sel == `FORGE_CRC_5)  ? 16'h001F :
                             16'h0000;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            crc_reg <= 16'h0000;
        end else if (clr) begin
            crc_reg <= 16'h0000;
        end else if (preset) begin
            crc_reg <= preset_val;
        end else if (write_en) begin
            crc_reg <= write_data;
        end else if (crc_en) begin
            crc_reg <= next_crc;
        end
    end

    assign crc_out  = invert_out ? (~crc_reg) : crc_reg;
    assign crc_zero = (crc_reg == 16'h0000);

endmodule
