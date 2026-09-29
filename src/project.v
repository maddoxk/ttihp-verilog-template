/*
 * Copyright (c) 2024 Maddox & Team
 * SPDX-License-Identifier: Apache-2.0
 *
 * Top Module: tt_um_maddox_protocolforge
 * Complete, synthesizable ProtocolForge ASIC for Tiny Tapeout (6x4 tiles).
 */

`default_nettype none
`include "forge_defs.vh"

module tt_um_maddox_protocolforge (
    input  wire [7:0] ui_in,    // Dedicated inputs  (GPIO 0..7)
    output wire [7:0] uo_out,   // Dedicated outputs (GPIO 8..15)
    input  wire [7:0] uio_in,   // IOs: Input path   (GPIO 16..19, SPI CS/SCK/MOSI/MISO)
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path  (0=input, 1=output)
    input  wire       ena,      // Tiny Tapeout power enable (always 1)
    input  wire       clk,      // System clock (50 MHz)
    input  wire       rst_n     // Active-low reset
);

    // =========================================================================
    // SYSTEM TIMEBASE
    // =========================================================================
    reg [15:0] sys_time;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sys_time <= 16'd0;
        end else begin
            sys_time <= sys_time + 16'd1;
        end
    end

    // =========================================================================
    // INPUT SYNCHRONIZATION & 3-TAP MAJORITY GLITCH FILTER
    // =========================================================================
    // 12 Inputs: ui_in[7:0] and uio_in[3:0]
    wire [11:0] raw_inputs = {uio_in[3:0], ui_in[7:0]};
    reg  [11:0] sync_1;
    reg  [11:0] sync_2;
    reg  [11:0] samp_0;
    reg  [11:0] samp_1;
    reg  [11:0] samp_2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_1 <= 12'd0;
            sync_2 <= 12'd0;
            samp_0 <= 12'd0;
            samp_1 <= 12'd0;
            samp_2 <= 12'd0;
        end else begin
            sync_1 <= raw_inputs;
            sync_2 <= sync_1;
            samp_0 <= sync_2;
            samp_1 <= samp_0;
            samp_2 <= samp_1;
        end
    end

    wire [11:0] infilt_mask;
    wire [11:0] maj_inputs;
    genvar g;
    generate
        for (g = 0; g < 12; g = g + 1) begin : gen_majority_filter
            wire maj = (samp_0[g] & samp_1[g]) | (samp_1[g] & samp_2[g]) | (samp_0[g] & samp_2[g]);
            assign maj_inputs[g] = infilt_mask[g] ? maj : sync_2[g];
        end
    endgenerate

    // 20-bit Full GPIO Array:
    // [7:0]   = Filtered ui_in[7:0]
    // [15:8]  = Current uo_out[7:0] loopback
    // [19:16] = Filtered uio_in[3:0]
    wire [19:0] gpio_in_all;
    wire [7:0]  uo_out_drivers;

    assign gpio_in_all[7:0]   = maj_inputs[7:0];
    assign gpio_in_all[15:8]  = uo_out_drivers;
    assign gpio_in_all[19:16] = maj_inputs[11:8];

    // =========================================================================
    // INTER-CORE 8-BIT FLAG CROSSBAR
    // =========================================================================
    reg  [7:0] flags;
    wire [7:0] flags_host_out;
    wire       flags_wr_en;
    wire [7:0] c0_flag_set, c0_flag_clr;
    wire [7:0] c1_flag_set, c1_flag_clr;
    wire [7:0] c2_flag_set, c2_flag_clr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            flags <= 8'h00;
        end else if (flags_wr_en) begin
            flags <= flags_host_out;
        end else begin
            flags <= (flags | c0_flag_set | c1_flag_set | c2_flag_set)
                           & ~(c0_flag_clr | c1_flag_clr | c2_flag_clr);
        end
    end

    // =========================================================================
    // HARDWARE AUTO-BAUD & TRANSITION SNIFFER (ABU)
    // =========================================================================
    wire [15:0] abu_min_pulse;
    wire        abu_clear;
    wire        abu_wr_en;
    wire [15:0] abu_wr_data;
    wire [15:0] abu_capture_val;
    wire        abu_edge_event;
    wire [15:0] abu_sniff_packet;
    wire        abu_sniff_valid;

    forge_abu u_abu (
        .clk(clk),
        .rst_n(rst_n),
        .pin_in(gpio_in_all[0]),
        .current_time(sys_time),
        .clear(abu_clear),
        .write_en(abu_wr_en),
        .write_data(abu_wr_data),
        .min_pulse(abu_min_pulse),
        .capture_val(abu_capture_val),
        .edge_detected(abu_edge_event),
        .sniff_packet(abu_sniff_packet),
        .sniff_valid(abu_sniff_valid)
    );

    // =========================================================================
    // STATE MACHINE CORES: CORE 0, CORE 1, CORE 2
    // =========================================================================
    wire [2:0]  core_enable;
    wire [2:0]  core_restart;
    wire [2:0]  core_step;

    wire [2:0]  core_reg_wr;
    wire [4:0]  core_reg_addr;
    wire [7:0]  core_reg_wdata;
    wire [7:0]  core0_reg_rdata, core1_reg_rdata, core2_reg_rdata;

    wire [2:0]  imem_wr_en;
    wire [5:0]  imem_addr;
    wire [15:0] imem_wdata;
    wire [15:0] core0_imem_rdata, core1_imem_rdata, core2_imem_rdata;

    wire [2:0]  tx_stream_push;
    wire [15:0] tx_stream_wdata;
    wire [2:0]  rx_stream_pop;
    wire [15:0] core0_rx_stream_data, core1_rx_stream_data, core2_rx_stream_data;

    wire [7:0]  core0_status, core1_status, core2_status;
    wire [3:0]  core0_rx_lvl, core1_rx_lvl, core2_rx_lvl;

    wire [19:0] c0_pin_out, c0_pin_oe;
    wire [19:0] c1_pin_out, c1_pin_oe;
    wire [19:0] c2_pin_out, c2_pin_oe;

    // Core 0 (Protocol Master / TX Engine, 48 words, 8-deep FIFOs)
    forge_core #(
        .IMEM_WORDS(48),
        .FIFO_AW(3),
        .CORE_ID(0)
    ) u_core0 (
        .clk(clk),
        .rst_n(rst_n),
        .enable(core_enable[0]),
        .restart(core_restart[0]),
        .step(core_step[0]),
        .gpio_in(gpio_in_all),
        .pin_out(c0_pin_out),
        .pin_oe(c0_pin_oe),
        .flags_in(flags),
        .flag_set(c0_flag_set),
        .flag_clr(c0_flag_clr),
        .sniff_packet(16'd0),
        .sniff_valid(1'b0),
        .reg_wr_en(core_reg_wr[0]),
        .reg_addr(core_reg_addr),
        .reg_wr_data(core_reg_wdata),
        .reg_rd_data(core0_reg_rdata),
        .imem_wr_en(imem_wr_en[0]),
        .imem_wr_addr(imem_addr),
        .imem_wr_data(imem_wdata),
        .imem_rd_addr(imem_addr),
        .imem_rd_data(core0_imem_rdata),
        .tx_stream_push(tx_stream_push[0]),
        .tx_stream_wdata(tx_stream_wdata),
        .rx_stream_pop(rx_stream_pop[0]),
        .rx_stream_rdata(core0_rx_stream_data),
        .status_byte(core0_status),
        .rx_fifo_lvl(core0_rx_lvl)
    );

    // Core 1 (Protocol Slave / RX Engine, 48 words, 8-deep FIFOs)
    forge_core #(
        .IMEM_WORDS(48),
        .FIFO_AW(3),
        .CORE_ID(1)
    ) u_core1 (
        .clk(clk),
        .rst_n(rst_n),
        .enable(core_enable[1]),
        .restart(core_restart[1]),
        .step(core_step[1]),
        .gpio_in(gpio_in_all),
        .pin_out(c1_pin_out),
        .pin_oe(c1_pin_oe),
        .flags_in(flags),
        .flag_set(c1_flag_set),
        .flag_clr(c1_flag_clr),
        .sniff_packet(16'd0),
        .sniff_valid(1'b0),
        .reg_wr_en(core_reg_wr[1]),
        .reg_addr(core_reg_addr),
        .reg_wr_data(core_reg_wdata),
        .reg_rd_data(core1_reg_rdata),
        .imem_wr_en(imem_wr_en[1]),
        .imem_wr_addr(imem_addr),
        .imem_wr_data(imem_wdata),
        .imem_rd_addr(imem_addr),
        .imem_rd_data(core1_imem_rdata),
        .tx_stream_push(tx_stream_push[1]),
        .tx_stream_wdata(tx_stream_wdata),
        .rx_stream_pop(rx_stream_pop[1]),
        .rx_stream_rdata(core1_rx_stream_data),
        .status_byte(core1_status),
        .rx_fifo_lvl(core1_rx_lvl)
    );

    // Core 2 (Sniffer / Aux Engine, 32 words, 4-deep FIFOs)
    forge_core #(
        .IMEM_WORDS(32),
        .FIFO_AW(2),
        .CORE_ID(2)
    ) u_core2 (
        .clk(clk),
        .rst_n(rst_n),
        .enable(core_enable[2]),
        .restart(core_restart[2]),
        .step(core_step[2]),
        .gpio_in(gpio_in_all),
        .pin_out(c2_pin_out),
        .pin_oe(c2_pin_oe),
        .flags_in(flags),
        .flag_set(c2_flag_set),
        .flag_clr(c2_flag_clr),
        .sniff_packet(abu_sniff_packet),
        .sniff_valid(abu_sniff_valid),
        .reg_wr_en(core_reg_wr[2]),
        .reg_addr(core_reg_addr),
        .reg_wr_data(core_reg_wdata),
        .reg_rd_data(core2_reg_rdata),
        .imem_wr_en(imem_wr_en[2]),
        .imem_wr_addr(imem_addr),
        .imem_wr_data(imem_wdata),
        .imem_rd_addr(imem_addr),
        .imem_rd_data(core2_imem_rdata),
        .tx_stream_push(tx_stream_push[2]),
        .tx_stream_wdata(tx_stream_wdata),
        .rx_stream_pop(rx_stream_pop[2]),
        .rx_stream_rdata(core2_rx_stream_data),
        .status_byte(core2_status),
        .rx_fifo_lvl(core2_rx_lvl)
    );

    // =========================================================================
    // HOST SPI SLAVE INTERFACE (uio[7:4])
    // =========================================================================
    wire       spi_miso;
    wire [3:0] pin_od_mask;

    forge_spi u_spi (
        .clk(clk),
        .rst_n(rst_n),
        .cs_n(uio_in[4]),
        .sck(uio_in[5]),
        .mosi(uio_in[6]),
        .miso(spi_miso),

        .core0_status(core0_status),
        .core1_status(core1_status),
        .core2_status(core2_status),
        .core0_rx_lvl(core0_rx_lvl),
        .core1_rx_lvl(core1_rx_lvl),
        .abu_edge_event(abu_edge_event),

        .core_enable(core_enable),
        .core_restart(core_restart),
        .core_step(core_step),
        .flags_in(flags),
        .flags_out(flags_host_out),
        .flags_wr_en(flags_wr_en),
        .gpio_in_all(gpio_in_all),
        .infilt_mask(infilt_mask),
        .pin_od_mask(pin_od_mask),

        .abu_min_pulse(abu_min_pulse),
        .abu_clear(abu_clear),
        .abu_wr_en(abu_wr_en),
        .abu_wr_data(abu_wr_data),

        .core_reg_wr(core_reg_wr),
        .core_reg_addr(core_reg_addr),
        .core_reg_wdata(core_reg_wdata),
        .core0_reg_rdata(core0_reg_rdata),
        .core1_reg_rdata(core1_reg_rdata),
        .core2_reg_rdata(core2_reg_rdata),

        .imem_wr_en(imem_wr_en),
        .imem_addr(imem_addr),
        .imem_wdata(imem_wdata),
        .core0_imem_rdata(core0_imem_rdata),
        .core1_imem_rdata(core1_imem_rdata),
        .core2_imem_rdata(core2_imem_rdata),

        .tx_stream_push(tx_stream_push),
        .tx_stream_wdata(tx_stream_wdata),
        .rx_stream_pop(rx_stream_pop),
        .core0_rx_stream_data(core0_rx_stream_data),
        .core1_rx_stream_data(core1_rx_stream_data),
        .core2_rx_stream_data(core2_rx_stream_data)
    );

    // =========================================================================
    // OUTPUT PIN MULTIPLEXING & OPEN-DRAIN LOGIC
    // =========================================================================
    // Dedicated Outputs (GPIO 8..15)
    assign uo_out_drivers = c0_pin_out[15:8] | c1_pin_out[15:8] | c2_pin_out[15:8];
    assign uo_out         = uo_out_drivers;

    // Bidirectional Pins (GPIO 16..19)
    wire [3:0] bidir_out = c0_pin_out[19:16] | c1_pin_out[19:16] | c2_pin_out[19:16];
    wire [3:0] bidir_oe  = c0_pin_oe[19:16]  | c1_pin_oe[19:16]  | c2_pin_oe[19:16];

    wire [3:0] uio_od_out;
    wire [3:0] uio_od_oe;
    genvar p;
    generate
        for (p = 0; p < 4; p = p + 1) begin : gen_od_pins
            // Open-drain: driving 0 pulls low (oe=1, out=0), driving 1 floats (oe=0, out=0)
            assign uio_od_out[p] = pin_od_mask[p] ? 1'b0 : bidir_out[p];
            assign uio_od_oe[p]  = pin_od_mask[p] ? (bidir_oe[p] & ~bidir_out[p]) : bidir_oe[p];
        end
    endgenerate

    // Final assignment to Tiny Tapeout uio pins
    assign uio_out = {spi_miso, 3'b000, uio_od_out};
    assign uio_oe  = {~uio_in[4], 3'b000, uio_od_oe};

    // Unused ena signal hook to prevent lint warnings
    wire _unused = &{ena, 1'b0};

endmodule
