/*
 * ProtocolForge Stream Bit Engine (SBE) & Bit-Stuffing Unit (BSU)
 * Supports NRZ, NRZI, Manchester encoding/decoding and USB/CAN bit-stuffing.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none
`include "forge_defs.vh"

module forge_sbe_bsu (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       clear,          // Reset state / counters at start of packet

    // Configuration
    input  wire [2:0] sbe_mode,       // NRZ, NRZI, Manchester, Inverted NRZI
    input  wire [1:0] bsu_mode,       // Disabled, USB, CAN

    // -------------------------------------------------------------------------
    // Transmit Path (OSR -> BSU -> SBE -> Pin)
    // -------------------------------------------------------------------------
    input  wire       tx_tick,        // Bit clock tick enable
    input  wire       tx_bit_in,      // Unstuffed data bit from OSR
    output reg        tx_stall_osr,   // Stalls OSR shift on stuff bit cycle
    output wire       tx_bit_crc,     // Bit to feed CRC (unstuffed data)
    output wire       tx_crc_en,      // CRC enable on TX bit
    output wire       tx_pin_out,     // Final modulated pin output bit

    // -------------------------------------------------------------------------
    // Receive Path (Pin -> SBE -> BSU -> ISR)
    // -------------------------------------------------------------------------
    input  wire       rx_tick,        // Sample clock tick enable
    input  wire       rx_pin_in,      // Synchronized / filtered physical input
    output wire       rx_bit_out,     // De-stuffed data bit for ISR & CRC
    output wire       rx_valid,       // 1 when rx_bit_out is valid (not stuffed)
    output reg        rx_stuff_err    // Latched bit-stuff error
);

    // =========================================================================
    // TX BIT-STUFFING UNIT (BSU)
    // =========================================================================
    reg [2:0] tx_stuff_cnt;
    reg       tx_prev_bit;
    reg       tx_stuff_active; // 1 during the inserted stuff bit period
    reg       tx_stuffed_val;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_stuff_cnt    <= 3'd0;
            tx_prev_bit     <= 1'b1;
            tx_stuff_active <= 1'b0;
            tx_stuffed_val  <= 1'b0;
            tx_stall_osr    <= 1'b0;
        end else if (clear) begin
            tx_stuff_cnt    <= 3'd0;
            tx_prev_bit     <= 1'b1;
            tx_stuff_active <= 1'b0;
            tx_stuffed_val  <= 1'b0;
            tx_stall_osr    <= 1'b0;
        end else if (tx_tick) begin
            if (tx_stuff_active) begin
                // Just completed emitting the stuff bit
                tx_stuff_active <= 1'b0;
                tx_stall_osr    <= 1'b0;
                if (bsu_mode == `FORGE_BSU_CAN) begin
                    tx_stuff_cnt <= 3'd1;
                    tx_prev_bit  <= tx_stuffed_val;
                end else begin
                    tx_stuff_cnt <= 3'd0;
                end
            end else begin
                // Normal bit transmission
                case (bsu_mode)
                    `FORGE_BSU_USB: begin
                        if (tx_bit_in == 1'b1) begin
                            if (tx_stuff_cnt == 3'd5) begin
                                // 6th consecutive 1 transmitted; next tick is stuffed 0
                                tx_stuff_active <= 1'b1;
                                tx_stuffed_val  <= 1'b0;
                                tx_stall_osr    <= 1'b1;
                                tx_stuff_cnt    <= 3'd0;
                            end else begin
                                tx_stuff_cnt <= tx_stuff_cnt + 1'b1;
                            end
                        end else begin
                            tx_stuff_cnt <= 3'd0;
                        end
                    end

                    `FORGE_BSU_CAN: begin
                        if (tx_bit_in == tx_prev_bit) begin
                            if (tx_stuff_cnt == 3'd4) begin
                                // 5th consecutive identical bit; next tick is complement
                                tx_stuff_active <= 1'b1;
                                tx_stuffed_val  <= ~tx_prev_bit;
                                tx_stall_osr    <= 1'b1;
                                tx_stuff_cnt    <= 3'd0;
                            end else begin
                                tx_stuff_cnt <= tx_stuff_cnt + 1'b1;
                            end
                        end else begin
                            tx_stuff_cnt <= 3'd1;
                            tx_prev_bit  <= tx_bit_in;
                        end
                    end

                    default: begin
                        tx_stuff_active <= 1'b0;
                        tx_stall_osr    <= 1'b0;
                        tx_stuff_cnt    <= 3'd0;
                    end
                endcase
            end
        end
    end

    // Selected TX bit feeding SBE
    wire tx_sbe_in = tx_stuff_active ? tx_stuffed_val : tx_bit_in;

    // CRC snoops un-stuffed data only
    assign tx_bit_crc = tx_bit_in;
    assign tx_crc_en  = tx_tick && !tx_stuff_active;

    // =========================================================================
    // TX STREAM BIT ENGINE (SBE)
    // =========================================================================
    reg tx_nrzi_state;
    reg tx_man_phase;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_nrzi_state <= 1'b1;
            tx_man_phase  <= 1'b0;
        end else if (clear) begin
            tx_nrzi_state <= 1'b1;
            tx_man_phase  <= 1'b0;
        end else if (tx_tick) begin
            // NRZI: 0 toggles physical line, 1 maintains state
            if (tx_sbe_in == 1'b0) begin
                tx_nrzi_state <= ~tx_nrzi_state;
            end
            // Manchester phase toggles on every half-bit tick
            tx_man_phase <= ~tx_man_phase;
        end
    end

    // Manchester phase logic:
    // Phase 0: ~data; Phase 1: data (Data 1 produces 0->1 rising transition)
    wire tx_man_out = tx_man_phase ? tx_sbe_in : ~tx_sbe_in;

    reg tx_out_mux;
    always @(*) begin
        case (sbe_mode)
            `FORGE_SBE_NRZ:        tx_out_mux = tx_sbe_in;
            `FORGE_SBE_NRZI:       tx_out_mux = tx_nrzi_state;
            `FORGE_SBE_MANCHESTER: tx_out_mux = tx_man_out;
            `FORGE_SBE_INV_NRZI:   tx_out_mux = ~tx_nrzi_state;
            default:               tx_out_mux = tx_sbe_in;
        endcase
    end

    assign tx_pin_out = tx_out_mux;

    // =========================================================================
    // RX STREAM BIT ENGINE (SBE) & BIT-STUFFING UNIT (BSU)
    // =========================================================================
    reg       rx_prev_pin;
    reg [2:0] rx_stuff_cnt;
    reg       rx_prev_bit;
    reg       rx_suppress;

    // Demodulate physical pin
    wire rx_edge_trans = (rx_pin_in != rx_prev_pin);
    wire rx_demod_bit  = (sbe_mode == `FORGE_SBE_NRZI || sbe_mode == `FORGE_SBE_INV_NRZI) ?
                         (rx_edge_trans ? 1'b0 : 1'b1) : rx_pin_in;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_prev_pin  <= 1'b1;
            rx_stuff_cnt <= 3'd0;
            rx_prev_bit  <= 1'b1;
            rx_suppress  <= 1'b0;
            rx_stuff_err <= 1'b0;
        end else if (clear) begin
            rx_prev_pin  <= 1'b1;
            rx_stuff_cnt <= 3'd0;
            rx_prev_bit  <= 1'b1;
            rx_suppress  <= 1'b0;
            rx_stuff_err <= 1'b0;
        end else if (rx_tick) begin
            rx_prev_pin <= rx_pin_in;

            case (bsu_mode)
                `FORGE_BSU_USB: begin
                    if (rx_stuff_cnt == 3'd6) begin
                        // 7th bit must be a stuffed 0
                        if (rx_demod_bit == 1'b0) begin
                            rx_suppress  <= 1'b1; // Strip stuffed 0
                            rx_stuff_cnt <= 3'd0;
                        end else begin
                            rx_stuff_err <= 1'b1; // 7 consecutive 1s error!
                            rx_suppress  <= 1'b1;
                            rx_stuff_cnt <= 3'd0;
                        end
                    end else begin
                        rx_suppress <= 1'b0;
                        if (rx_demod_bit == 1'b1)
                            rx_stuff_cnt <= rx_stuff_cnt + 1'b1;
                        else
                            rx_stuff_cnt <= 3'd0;
                    end
                end

                `FORGE_BSU_CAN: begin
                    if (rx_stuff_cnt == 3'd5) begin
                        // 6th bit must be complement
                        if (rx_demod_bit == ~rx_prev_bit) begin
                            rx_suppress  <= 1'b1; // Strip stuffed complement
                            rx_stuff_cnt <= 3'd1;
                            rx_prev_bit  <= rx_demod_bit;
                        end else begin
                            rx_stuff_err <= 1'b1; // 6 consecutive identical bits error!
                            rx_suppress  <= 1'b1;
                            rx_stuff_cnt <= 3'd0;
                        end
                    end else begin
                        rx_suppress <= 1'b0;
                        if (rx_demod_bit == rx_prev_bit)
                            rx_stuff_cnt <= rx_stuff_cnt + 1'b1;
                        else begin
                            rx_stuff_cnt <= 3'd1;
                            rx_prev_bit  <= rx_demod_bit;
                        end
                    end
                end

                default: begin
                    rx_suppress <= 1'b0;
                    rx_stuff_cnt <= 3'd0;
                end
            endcase
        end
    end

    assign rx_bit_out = rx_demod_bit;
    assign rx_valid   = rx_tick && !rx_suppress;

endmodule
