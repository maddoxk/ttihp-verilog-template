/*
 * ProtocolForge State Machine Core
 * High-performance, deterministic protocol emulator core with local IMEM,
 * deadline scheduling, SBE/BSU line coding, and inline streaming CRC.
 * Optimized for minimal gate count and zero latches.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none
`include "forge_defs.vh"

module forge_core #(
    parameter IMEM_WORDS = 48, // 48 for Core 0/1, 32 for Core 2
    parameter FIFO_AW    = 3,  // 3 => 8-word FIFO, 2 => 4-word FIFO
    parameter CORE_ID    = 0
) (
    input  wire        clk,
    input  wire        rst_n,

    // Execution Control
    input  wire        enable,
    input  wire        restart,
    input  wire        step,

    // 20-bit Synchronized / Filtered GPIO Bus
    input  wire [19:0] gpio_in,
    output wire [19:0] pin_out,
    output wire [19:0] pin_oe,

    // Inter-Core Flags (8 bits)
    input  wire [7:0]  flags_in,
    output reg  [7:0]  flag_set,
    output reg  [7:0]  flag_clr,

    // Hardware Sniffer Input (Core 2)
    input  wire [15:0] sniff_packet,
    input  wire        sniff_valid,

    // Host Register Interface (Single-byte accesses)
    input  wire        reg_wr_en,
    input  wire [4:0]  reg_addr,
    input  wire [7:0]  reg_wr_data,
    output reg  [7:0]  reg_rd_data,

    // Host Instruction Memory Direct Programming
    input  wire        imem_wr_en,
    input  wire [5:0]  imem_wr_addr,
    input  wire [15:0] imem_wr_data,
    input  wire [5:0]  imem_rd_addr,
    output wire [15:0] imem_rd_data,

    // Host Burst FIFO Streaming (16-bit DMA Port)
    input  wire        tx_stream_push,
    input  wire [15:0] tx_stream_wdata,
    input  wire        rx_stream_pop,
    output wire [15:0] rx_stream_rdata,

    // Telemetry Outputs
    output wire [7:0]  status_byte,
    output wire [3:0]  rx_fifo_lvl
);

    // =========================================================================
    // HELPER FUNCTIONS (Hardware-optimized, no generic dividers)
    // =========================================================================
    function [4:0] mod20;
        input [5:0] val;
        reg   [5:0] tmp;
        begin
            tmp = (val >= 6'd40) ? (val - 6'd40) :
                  (val >= 6'd20) ? (val - 6'd20) : val;
            mod20 = tmp[4:0];
        end
    endfunction

    function [15:0] bitmask16;
        input [4:0] cnt;
        begin
            case (cnt)
                5'd1:    bitmask16 = 16'h0001;
                5'd2:    bitmask16 = 16'h0003;
                5'd3:    bitmask16 = 16'h0007;
                5'd4:    bitmask16 = 16'h000F;
                5'd5:    bitmask16 = 16'h001F;
                5'd6:    bitmask16 = 16'h003F;
                5'd7:    bitmask16 = 16'h007F;
                5'd8:    bitmask16 = 16'h00FF;
                5'd9:    bitmask16 = 16'h01FF;
                5'd10:   bitmask16 = 16'h03FF;
                5'd11:   bitmask16 = 16'h07FF;
                5'd12:   bitmask16 = 16'h0FFF;
                5'd13:   bitmask16 = 16'h1FFF;
                5'd14:   bitmask16 = 16'h3FFF;
                5'd15:   bitmask16 = 16'h7FFF;
                default: bitmask16 = 16'hFFFF;
            endcase
        end
    endfunction

    function [15:0] bitrev16;
        input [15:0] in;
        integer b;
        begin
            for (b = 0; b < 16; b = b + 1)
                bitrev16[b] = in[15 - b];
        end
    endfunction

    function [15:0] byteswap16;
        input [15:0] in;
        begin
            byteswap16 = {in[7:0], in[15:8]};
        end
    endfunction

    // =========================================================================
    // LOCAL INSTRUCTION MEMORY (Dedicated Flip-Flop Array)
    // =========================================================================
    reg [15:0] imem [0:IMEM_WORDS-1];
    reg [5:0]  pc;

    integer init_i;
    initial begin
        for (init_i = 0; init_i < IMEM_WORDS; init_i = init_i + 1) begin
            imem[init_i] = 16'h0000;
        end
    end

    wire [15:0] instr = (pc < IMEM_WORDS) ? imem[pc] : 16'h0000;
    assign imem_rd_data = (imem_rd_addr < IMEM_WORDS) ? imem[imem_rd_addr] : 16'h0000;

    // =========================================================================
    // REGISTERS & DATAPATH
    // =========================================================================
    reg [15:0] reg_x;
    reg [15:0] reg_y;
    reg [15:0] reg_t;          // Free-running 16-bit timebase (increments every 20ns)
    reg [15:0] reg_dl;         // Absolute deadline register
    reg [15:0] reg_capture;    // Edge timestamp capture snapshot
    reg [15:0] reg_isr;
    reg [4:0]  isr_count;
    reg [15:0] reg_osr;
    reg [4:0]  osr_count;

    reg [19:0] pindirs;        // Pin direction mask
    reg [19:0] pin_drivers;    // Pin output drive values

    // Configuration Registers
    reg [15:0] div_int;
    reg [7:0]  div_frac;
    reg [4:0]  out_base;
    reg [4:0]  out_count;
    reg [4:0]  set_base;
    reg [4:0]  set_count;
    reg [4:0]  in_base;
    reg [4:0]  sideset_base;
    reg [2:0]  sideset_count;
    reg [4:0]  jmp_pin;
    reg [5:0]  wrap_top;
    reg [5:0]  wrap_bot;

    reg        in_shift_dir;   // 0=left, 1=right
    reg        out_shift_dir;  // 0=left, 1=right
    reg        autopush;
    reg        autopull;
    reg [4:0]  push_thresh;    // 1..16
    reg [4:0]  pull_thresh;    // 1..16

    reg        scl_stretch_en;
    reg        arb_detect_en;
    reg [1:0]  jmp_cond_sel;   // 00=!crc, 01=arb_lost, 10=!osre, 11=stuff_err
    reg        sniffer_en;
    reg [4:0]  scl_pin_idx;

    reg [2:0]  sbe_mode;
    reg [1:0]  bsu_mode;
    reg [1:0]  crc_poly;
    reg        crc_snoop_rx;   // 0=TX, 1=RX
    reg        crc_enable;
    reg        crc_invert;

    reg        arb_lost;

    // Buffer for host 8-bit to 16-bit FIFO assembly
    reg [7:0]  txf_buf_l;

    // Execution temporary variables
    reg        cond_met;
    reg [4:0]  in_cnt;
    reg [4:0]  out_cnt;
    reg [15:0] shifted_bits;
    integer    k;

    // =========================================================================
    // SYNCHRONOUS FIFOS (TX and RX)
    // =========================================================================
    wire [15:0] txf_rdata;
    wire        txf_push;
    wire [15:0] txf_wdata;
    reg         txf_pop;
    wire        txf_full;
    wire        txf_empty;
    wire [FIFO_AW:0] txf_level;

    forge_fifo #(
        .DATA_WIDTH(16),
        .ADDR_WIDTH(FIFO_AW)
    ) u_tx_fifo (
        .clk(clk),
        .rst_n(rst_n),
        .clear(restart),
        .wdata(txf_wdata),
        .push(txf_push),
        .full(txf_full),
        .rdata(txf_rdata),
        .pop(txf_pop),
        .empty(txf_empty),
        .level(txf_level)
    );

    wire [15:0] rxf_wdata;
    reg         rxf_push;
    wire        rxf_pop;
    wire [15:0] rxf_rdata;
    wire        rxf_full;
    wire        rxf_empty;
    wire [FIFO_AW:0] rxf_level;

    forge_fifo #(
        .DATA_WIDTH(16),
        .ADDR_WIDTH(FIFO_AW)
    ) u_rx_fifo (
        .clk(clk),
        .rst_n(rst_n),
        .clear(restart),
        .wdata(rxf_wdata),
        .push(rxf_push),
        .full(rxf_full),
        .rdata(rxf_rdata),
        .pop(rxf_pop),
        .empty(rxf_empty),
        .level(rxf_level)
    );

    // Host stream connections
    assign txf_push  = tx_stream_push || (reg_wr_en && (reg_addr == `FORGE_OFF_TXF_STREAM_H));
    assign txf_wdata = tx_stream_push ? tx_stream_wdata : {reg_wr_data, txf_buf_l};

    assign rxf_pop         = rx_stream_pop || (reg_wr_en && (reg_addr == `FORGE_OFF_RXF_STREAM_H));
    assign rx_stream_rdata = rxf_rdata;
    assign rx_fifo_lvl     = {{(4-FIFO_AW){1'b0}}, rxf_level[FIFO_AW-1:0]};

    // =========================================================================
    // FRACTIONAL CLOCK DIVIDER (16.8)
    // =========================================================================
    reg [15:0] div_cnt;
    reg [7:0]  frac_acc;
    reg        div_tick;
    wire [8:0] next_frac = {1'b0, frac_acc} + {1'b0, div_frac};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt  <= 16'd0;
            frac_acc <= 8'd0;
            div_tick <= 1'b0;
        end else if (restart) begin
            div_cnt  <= 16'd0;
            frac_acc <= 8'd0;
            div_tick <= 1'b0;
        end else begin
            if (div_int <= 16'd1 && div_frac == 8'd0) begin
                div_tick <= 1'b1;
            end else if (div_cnt == 16'd0) begin
                div_tick <= 1'b1;
                div_cnt  <= (div_int > 16'd0 ? div_int - 1'b1 : 16'd0) + {15'd0, next_frac[8]};
                frac_acc <= next_frac[7:0];
            end else begin
                div_tick <= 1'b0;
                div_cnt  <= div_cnt - 1'b1;
            end
        end
    end

    // =========================================================================
    // HARDWARE ACCELERATORS: SBE / BSU & CRC
    // =========================================================================
    wire       tx_stall_osr;
    wire       tx_bit_crc;
    wire       tx_crc_en;
    wire       sbe_pin_out;
    wire       rx_bit_out;
    wire       rx_valid;
    wire       rx_stuff_err;

    wire tx_bit_raw = out_shift_dir ? reg_osr[0] : reg_osr[15];
    wire rx_pin_phys = gpio_in[mod20({1'b0, in_base})];

    forge_sbe_bsu u_sbe_bsu (
        .clk(clk),
        .rst_n(rst_n),
        .clear(restart),
        .sbe_mode(sbe_mode),
        .bsu_mode(bsu_mode),
        .tx_tick(div_tick),
        .tx_bit_in(tx_bit_raw),
        .tx_stall_osr(tx_stall_osr),
        .tx_bit_crc(tx_bit_crc),
        .tx_crc_en(tx_crc_en),
        .tx_pin_out(sbe_pin_out),
        .rx_tick(div_tick),
        .rx_pin_in(rx_pin_phys),
        .rx_bit_out(rx_bit_out),
        .rx_valid(rx_valid),
        .rx_stuff_err(rx_stuff_err)
    );

    // CRC Coprocessor
    wire        crc_zero;
    wire [15:0] crc_val;
    reg         crc_clr;
    reg         crc_preset;
    reg         crc_wr_en;
    reg  [15:0] crc_wr_val;

    wire crc_bit_in = crc_snoop_rx ? rx_bit_out : tx_bit_crc;
    wire crc_tick   = crc_enable && (crc_snoop_rx ? rx_valid : tx_crc_en);

    forge_crc u_crc (
        .clk(clk),
        .rst_n(rst_n),
        .poly_sel(crc_poly),
        .invert_out(crc_invert),
        .clr(crc_clr || restart),
        .preset(crc_preset),
        .crc_en(crc_tick),
        .bit_in(crc_bit_in),
        .write_en(crc_wr_en),
        .write_data(crc_wr_val),
        .crc_out(crc_val),
        .crc_zero(crc_zero)
    );

    // Output pin drive logic with SBE override
    wire [4:0] out_base_m20 = mod20({1'b0, out_base});
    assign pin_out = (sbe_mode != `FORGE_SBE_NRZ) ?
                     ((pin_drivers & ~(20'b1 << out_base_m20)) | ({19'b0, sbe_pin_out} << out_base_m20)) :
                     pin_drivers;
    assign pin_oe  = pindirs;

    // =========================================================================
    // INSTRUCTION DECODE & EXECUTION ENGINE
    // =========================================================================
    wire [2:0] opcode     = instr[15:13];
    wire [3:0] delay_side = instr[12:9];
    wire [8:0] operand    = instr[8:0];

    // Sideset decoding (Verilog-2001 safe without variable part-selects)
    reg [4:0] sideset_data;
    always @(*) begin
        case (sideset_count)
            3'd1: sideset_data = {4'b0, delay_side[3]};
            3'd2: sideset_data = {3'b0, delay_side[3:2]};
            3'd3: sideset_data = {2'b0, delay_side[3:1]};
            3'd4: sideset_data = {1'b0, delay_side[3:0]};
            default: sideset_data = 5'd0;
        endcase
    end

    reg [4:0] effective_delay;
    always @(*) begin
        case (sideset_count)
            3'd0: effective_delay = {1'b0, delay_side};
            3'd1: effective_delay = {2'b0, delay_side[2:0]};
            3'd2: effective_delay = {3'b0, delay_side[1:0]};
            3'd3: effective_delay = {4'b0, delay_side[0]};
            default: effective_delay = 5'd0;
        endcase
    end

    // Deadline check (wrap-around safe signed comparison)
    wire signed [15:0] t_diff = reg_t - reg_dl;
    wire dl_reached = !t_diff[15];

    // Stall conditions
    reg [4:0] delay_cnt;
    wire wait_stall;
    wire fifo_stall;
    wire scl_stall;
    wire is_stalled;

    // SCL clock stretching detection
    wire [4:0] scl_idx_m20 = mod20({1'b0, scl_pin_idx});
    assign scl_stall = scl_stretch_en && (pin_drivers[scl_idx_m20] == 1'b1) &&
                       (gpio_in[scl_idx_m20] == 1'b0);

    // WAIT stall logic
    wire wait_pol  = operand[8];
    wire [1:0] wait_src  = operand[7:6];
    wire [4:0] wait_idx  = operand[4:0];
    reg  wait_cond_met;
    reg  wait_prev_edge_pin;

    always @(*) begin
        case (wait_src)
            `FORGE_WAIT_SRC_GPIO: wait_cond_met = (gpio_in[mod20({1'b0, wait_idx})] == wait_pol);
            `FORGE_WAIT_SRC_PIN:  wait_cond_met = (gpio_in[mod20(in_base + wait_idx)] == wait_pol);
            `FORGE_WAIT_SRC_FLAG: wait_cond_met = (flags_in[operand[2:0]] == wait_pol);
            `FORGE_WAIT_SRC_EXT: begin
                if (operand[5:0] == 6'd0) begin
                    wait_cond_met = dl_reached;
                end else begin
                    wait_cond_met = (gpio_in[mod20(in_base + wait_idx)] != wait_prev_edge_pin);
                end
            end
            default: wait_cond_met = 1'b1;
        endcase
    end

    assign wait_stall = (opcode == `FORGE_OP_WAIT) && !wait_cond_met;

    // FIFO stall logic for blocking PUSH/PULL/autopush/autopull
    wire push_block = (opcode == `FORGE_OP_PUSH_PULL && operand[8] == 1'b0 && operand[6] == 1'b1 && rxf_full);
    wire pull_block = (opcode == `FORGE_OP_PUSH_PULL && operand[8] == 1'b1 && operand[6] == 1'b1 && txf_empty);
    wire autopush_block = autopush && (isr_count >= push_thresh) && rxf_full;
    wire autopull_block = autopull && (osr_count >= pull_thresh) && txf_empty;

    assign fifo_stall = push_block || pull_block || autopush_block || autopull_block;
    assign is_stalled = (delay_cnt > 5'd0) || wait_stall || fifo_stall || scl_stall || tx_stall_osr;


    // Status Byte for Host Telemetry
    assign status_byte = {crc_zero, arb_lost, rx_stuff_err, is_stalled,
                          rxf_full, rxf_empty, txf_full, txf_empty};

    // 16-bit Input Mux for IN and MOV
    wire [15:0] status_word = {8'b0, status_byte};
    wire [4:0] in_base_m20 = mod20({1'b0, in_base});

    reg [15:0] in_src_data;
    always @(*) begin
        case (operand[8:6])
            `FORGE_IN_SRC_PINS:    in_src_data = {gpio_in, gpio_in} >> in_base_m20;
            `FORGE_IN_SRC_X:       in_src_data = reg_x;
            `FORGE_IN_SRC_Y:       in_src_data = reg_y;
            `FORGE_IN_SRC_NULL:    in_src_data = 16'h0000;
            `FORGE_IN_SRC_T:       in_src_data = reg_t;
            `FORGE_IN_SRC_STATUS:  in_src_data = status_word;
            `FORGE_IN_SRC_CRC:     in_src_data = crc_val;
            `FORGE_IN_SRC_CAPTURE: in_src_data = reg_capture;
            default:               in_src_data = 16'h0000;
        endcase
    end

    reg [15:0] mov_src_data;
    always @(*) begin
        case (operand[2:0])
            3'b000: mov_src_data = {gpio_in, gpio_in} >> in_base_m20;
            3'b001: mov_src_data = reg_x;
            3'b010: mov_src_data = reg_y;
            3'b011: mov_src_data = 16'h0000;
            3'b100: mov_src_data = reg_t;
            3'b101: mov_src_data = status_word;
            3'b110: mov_src_data = crc_val;
            3'b111: mov_src_data = reg_capture;
        endcase
    end

    reg [15:0] mov_alu_result;
    always @(*) begin
        case (operand[5:4])
            `FORGE_MOV_OP_NONE:     mov_alu_result = mov_src_data;
            `FORGE_MOV_OP_INVERT:   mov_alu_result = ~mov_src_data;
            `FORGE_MOV_OP_BITREV:   mov_alu_result = bitrev16(mov_src_data);
            `FORGE_MOV_OP_BYTESWAP: mov_alu_result = byteswap16(mov_src_data);
        endcase
    end

    // RX FIFO Push source
    assign rxf_wdata = (sniffer_en && CORE_ID == 2) ? sniff_packet : reg_isr;

    // Execution Step Condition
    wire exec_step = (enable && div_tick && !is_stalled) || step;

    // =========================================================================
    // MAIN CLOCKED STATE & DATAPATH
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc                 <= 6'd0;
            reg_x              <= 16'd0;
            reg_y              <= 16'd0;
            reg_t              <= 16'd0;
            reg_dl             <= 16'd0;
            reg_capture        <= 16'd0;
            reg_isr            <= 16'd0;
            isr_count          <= 5'd0;
            reg_osr            <= 16'd0;
            osr_count          <= 5'd0;
            pindirs            <= 20'd0;
            pin_drivers        <= 20'd0;
            delay_cnt          <= 5'd0;
            flag_set           <= 8'd0;
            flag_clr           <= 8'd0;
            txf_pop            <= 1'b0;
            rxf_push           <= 1'b0;
            crc_clr            <= 1'b0;
            crc_preset         <= 1'b0;
            arb_lost           <= 1'b0;
            wait_prev_edge_pin <= 1'b0;
        end else if (restart) begin
            pc                 <= 6'd0;
            reg_x              <= 16'd0;
            reg_y              <= 16'd0;
            reg_dl             <= 16'd0;
            reg_capture        <= 16'd0;
            reg_isr            <= 16'd0;
            isr_count          <= 5'd0;
            reg_osr            <= 16'd0;
            osr_count          <= 5'd0;
            pindirs            <= 20'd0;
            pin_drivers        <= 20'd0;
            delay_cnt          <= 5'd0;
            flag_set           <= 8'd0;
            flag_clr           <= 8'd0;
            txf_pop            <= 1'b0;
            rxf_push           <= 1'b0;
            crc_clr            <= 1'b0;
            crc_preset         <= 1'b0;
            arb_lost           <= 1'b0;
        end else begin
            // Free-running 16-bit timebase (20ns resolution)
            reg_t <= reg_t + 16'd1;

            // Clear single-cycle strobe outputs
            flag_set   <= 8'd0;
            flag_clr   <= 8'd0;
            txf_pop    <= 1'b0;
            rxf_push   <= 1'b0;
            if (rxf_push && !sniffer_en) begin
                reg_isr   <= 16'd0;
                isr_count <= 5'd0;
            end
            crc_clr    <= 1'b0;
            crc_preset <= 1'b0;

            // Sub-cycle edge timestamp capture
            if (gpio_in[in_base_m20] != wait_prev_edge_pin) begin
                reg_capture <= reg_t;
            end
            wait_prev_edge_pin <= gpio_in[in_base_m20];

            // Arbitration loss monitoring or host clear
            if (reg_wr_en && (reg_addr == `FORGE_OFF_STATUS)) begin
                arb_lost <= 1'b0;
            end else if (arb_detect_en) begin
                for (k = 0; k < 20; k = k + 1) begin
                    if (pindirs[k] && pin_drivers[k] && !gpio_in[k]) begin
                        arb_lost <= 1'b1;
                    end
                end
            end

            // Core 2 hardware sniffer push
            if (sniffer_en && CORE_ID == 2 && sniff_valid && !rxf_full) begin
                rxf_push <= 1'b1;
            end

            // Delay cycle counting
            if (enable && div_tick && delay_cnt > 5'd0) begin
                delay_cnt <= delay_cnt - 1'b1;
            end

            // Autopull check
            if (autopull && (osr_count >= pull_thresh) && !txf_empty) begin
                reg_osr   <= txf_rdata;
                osr_count <= 5'd0;
                txf_pop   <= 1'b1;
            end

            // Autopush check
            if (autopush && (isr_count >= push_thresh) && !rxf_full) begin
                rxf_push  <= 1'b1;
            end

            // PC host write or execution step
            if (reg_wr_en && (reg_addr == `FORGE_OFF_PC)) begin
                pc <= reg_wr_data[5:0];
            end else if (exec_step) begin
                // Sideset output execution
                if (sideset_count > 0) begin
                    for (k = 0; k < 4; k = k + 1) begin
                        if (k < sideset_count)
                            pin_drivers[mod20(sideset_base + k)] <= sideset_data[k];
                    end
                end

                // Set initial delay for this instruction
                if (effective_delay > 5'd0) begin
                    delay_cnt <= effective_delay;
                end

                case (opcode)
                    `FORGE_OP_JMP: begin
                        case (operand[8:6])
                            `FORGE_COND_ALWAYS:     cond_met = 1'b1;
                            `FORGE_COND_NOT_X:      cond_met = (reg_x == 16'd0);
                            `FORGE_COND_POST_DEC_X: begin
                                cond_met = (reg_x != 16'd0);
                                if (reg_x != 16'd0) reg_x <= reg_x - 1'b1;
                            end
                            `FORGE_COND_NOT_Y:      cond_met = (reg_y == 16'd0);
                            `FORGE_COND_POST_DEC_Y: begin
                                cond_met = (reg_y != 16'd0);
                                if (reg_y != 16'd0) reg_y <= reg_y - 1'b1;
                            end
                            `FORGE_COND_X_NEQ_Y:    cond_met = (reg_x != reg_y);
                            `FORGE_COND_PIN:        cond_met = gpio_in[mod20({1'b0, jmp_pin})];
                            `FORGE_COND_EXT: begin
                                case (jmp_cond_sel)
                                    2'b00: cond_met = crc_zero;
                                    2'b01: cond_met = arb_lost;
                                    2'b10: cond_met = (osr_count < pull_thresh && !txf_empty);
                                    2'b11: cond_met = rx_stuff_err;
                                endcase
                            end
                        endcase

                        if (cond_met) begin
                            pc <= operand[5:0];
                        end else begin
                            pc <= (pc == wrap_top) ? wrap_bot : (pc + 1'b1);
                        end
                    end

                    `FORGE_OP_WAIT: begin
                        // Clear flag if woken up on flag high
                        if (wait_src == `FORGE_WAIT_SRC_FLAG && wait_pol == 1'b1) begin
                            flag_clr[operand[2:0]] <= 1'b1;
                        end
                        pc <= (pc == wrap_top) ? wrap_bot : (pc + 1'b1);
                    end

                    `FORGE_OP_IN: begin
                        in_cnt = (operand[4:0] == 5'd0) ? 5'd16 : operand[4:0];

                        if (in_shift_dir == 1'b0) begin // Left shift
                            reg_isr <= (reg_isr << in_cnt) | (in_src_data & bitmask16(in_cnt));
                        end else begin // Right shift
                            reg_isr <= (reg_isr >> in_cnt) | ((in_src_data & bitmask16(in_cnt)) << (5'd16 - in_cnt));
                        end
                        isr_count <= isr_count + in_cnt;
                        pc <= (pc == wrap_top) ? wrap_bot : (pc + 1'b1);
                    end

                    `FORGE_OP_OUT: begin
                        out_cnt = (operand[4:0] == 5'd0) ? 5'd16 : operand[4:0];

                        if (out_shift_dir == 1'b0) begin // Left shift
                            shifted_bits = reg_osr >> (5'd16 - out_cnt);
                            reg_osr <= reg_osr << out_cnt;
                        end else begin // Right shift
                            shifted_bits = reg_osr & bitmask16(out_cnt);
                            reg_osr <= reg_osr >> out_cnt;
                        end
                        osr_count <= osr_count + out_cnt;

                        case (operand[8:6])
                            `FORGE_OUT_DST_PINS: begin
                                for (k = 0; k < 5; k = k + 1) begin
                                    if (k < out_count)
                                        pin_drivers[mod20(out_base + k)] <= shifted_bits[k];
                                end
                            end
                            `FORGE_OUT_DST_X:       reg_x <= shifted_bits;
                            `FORGE_OUT_DST_Y:       reg_y <= shifted_bits;
                            `FORGE_OUT_DST_NULL:    ; // Discard
                            `FORGE_OUT_DST_PINDIRS: begin
                                for (k = 0; k < 5; k = k + 1) begin
                                    if (k < out_count)
                                        pindirs[mod20(out_base + k)] <= shifted_bits[k];
                                end
                            end
                            `FORGE_OUT_DST_PC:      pc <= shifted_bits[5:0];
                            `FORGE_OUT_DST_ISR:     begin reg_isr <= shifted_bits; isr_count <= out_cnt; end
                            `FORGE_OUT_DST_DL:      reg_dl <= shifted_bits;
                        endcase

                        if (operand[8:6] != `FORGE_OUT_DST_PC) begin
                            pc <= (pc == wrap_top) ? wrap_bot : (pc + 1'b1);
                        end
                    end

                    `FORGE_OP_PUSH_PULL: begin
                        if (operand[8] == 1'b0) begin // PUSH
                            if (!operand[7] || (isr_count >= push_thresh)) begin
                                if (!rxf_full) begin
                                    rxf_push  <= 1'b1;
                                end
                            end
                        end else begin // PULL
                            if (!operand[7] || (osr_count >= pull_thresh)) begin
                                if (!txf_empty) begin
                                    reg_osr   <= txf_rdata;
                                    osr_count <= 5'd0;
                                    txf_pop   <= 1'b1;
                                end else if (!operand[6]) begin // noblock
                                    reg_osr   <= reg_x;
                                    osr_count <= 5'd0;
                                end
                            end
                        end
                        pc <= (pc == wrap_top) ? wrap_bot : (pc + 1'b1);
                    end

                    `FORGE_OP_MOV: begin
                        case (operand[8:6])
                            `FORGE_MOV_DST_PINS: begin
                                for (k = 0; k < 5; k = k + 1) begin
                                    if (k < out_count)
                                        pin_drivers[mod20(out_base + k)] <= mov_alu_result[k];
                                end
                            end
                            `FORGE_MOV_DST_X:       reg_x <= mov_alu_result;
                            `FORGE_MOV_DST_Y:       reg_y <= mov_alu_result;
                            `FORGE_MOV_DST_DL:      reg_dl <= mov_alu_result;
                            `FORGE_MOV_DST_PINDIRS: begin
                                for (k = 0; k < 5; k = k + 1) begin
                                    if (k < out_count)
                                        pindirs[mod20(out_base + k)] <= mov_alu_result[k];
                                end
                            end
                            `FORGE_MOV_DST_PC:      pc <= mov_alu_result[5:0];
                            `FORGE_MOV_DST_ISR:     begin reg_isr <= mov_alu_result; isr_count <= 5'd16; end
                            `FORGE_MOV_DST_OSR:     begin reg_osr <= mov_alu_result; osr_count <= 5'd0; end
                        endcase

                        if (operand[8:6] != `FORGE_MOV_DST_PC) begin
                            pc <= (pc == wrap_top) ? wrap_bot : (pc + 1'b1);
                        end
                    end

                    `FORGE_OP_SET: begin
                        case (operand[8:6])
                            `FORGE_SET_DST_PINS: begin
                                for (k = 0; k < 5; k = k + 1) begin
                                    if (k < set_count)
                                        pin_drivers[mod20(set_base + k)] <= operand[k];
                                end
                            end
                            `FORGE_SET_DST_X:       reg_x <= {10'b0, operand[5:0]};
                            `FORGE_SET_DST_Y:       reg_y <= {10'b0, operand[5:0]};
                            `FORGE_SET_DST_PINDIRS: begin
                                for (k = 0; k < 5; k = k + 1) begin
                                    if (k < set_count)
                                        pindirs[mod20(set_base + k)] <= operand[k];
                                end
                            end
                            `FORGE_SET_DST_FLAGSET: flag_set[operand[2:0]] <= 1'b1;
                            `FORGE_SET_DST_FLAGCLR: flag_clr[operand[2:0]] <= 1'b1;
                            `FORGE_SET_DST_T:       reg_t <= {10'b0, operand[5:0]};
                            `FORGE_SET_DST_CRC: begin
                                if (operand[0] == 1'b0) crc_clr <= 1'b1;
                                else crc_preset <= 1'b1;
                            end
                        endcase
                        pc <= (pc == wrap_top) ? wrap_bot : (pc + 1'b1);
                    end

                    `FORGE_OP_TIME: begin
                        case (operand[8:7])
                            `FORGE_TIME_T_ADD_N:  reg_dl <= reg_t + {9'b0, operand[6:0]};
                            `FORGE_TIME_DL_ADD_N: reg_dl <= reg_dl + {9'b0, operand[6:0]};
                            `FORGE_TIME_T_ADD_X:  reg_dl <= reg_t + reg_x;
                            `FORGE_TIME_DL_ADD_X: reg_dl <= reg_dl + reg_x;
                        endcase
                        pc <= (pc == wrap_top) ? wrap_bot : (pc + 1'b1);
                    end
                endcase
            end
        end
    end

    // =========================================================================
    // HOST CONTROL & REGISTER FILE INTERFACE
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_int        <= 16'd1;
            div_frac       <= 8'd0;
            out_base       <= 5'd0;
            out_count      <= 5'd1;
            set_base       <= 5'd0;
            set_count      <= 5'd1;
            in_base        <= 5'd0;
            sideset_base   <= 5'd0;
            sideset_count  <= 3'd0;
            jmp_pin        <= 5'd0;
            wrap_top       <= (IMEM_WORDS - 1);
            wrap_bot       <= 6'd0;
            in_shift_dir   <= 1'b0;
            out_shift_dir  <= 1'b0;
            autopush       <= 1'b0;
            autopull       <= 1'b0;
            push_thresh    <= 5'd16;
            pull_thresh    <= 5'd16;
            scl_stretch_en <= 1'b0;
            arb_detect_en  <= 1'b0;
            jmp_cond_sel   <= 2'b00;
            sniffer_en     <= 1'b0;
            scl_pin_idx    <= 5'd16;
            sbe_mode       <= `FORGE_SBE_NRZ;
            bsu_mode       <= `FORGE_BSU_DIS;
            crc_poly       <= `FORGE_CRC_16;
            crc_snoop_rx   <= 1'b0;
            crc_enable     <= 1'b0;
            crc_invert     <= 1'b0;
            crc_wr_en      <= 1'b0;
            crc_wr_val     <= 16'd0;
            txf_buf_l      <= 8'd0;
            reg_rd_data    <= 8'd0;
        end else begin
            // Single-cycle strobe
            crc_wr_en <= 1'b0;

            // IMEM Host Write
            if (imem_wr_en && imem_wr_addr < IMEM_WORDS) begin
                imem[imem_wr_addr] <= imem_wr_data;
            end

            // CSR Host Write
            if (reg_wr_en) begin
                case (reg_addr)
                    `FORGE_OFF_DIV_INT_L:    div_int[7:0]   <= reg_wr_data;
                    `FORGE_OFF_DIV_INT_H:    div_int[15:8]  <= reg_wr_data;
                    `FORGE_OFF_DIV_FRAC:     div_frac       <= reg_wr_data;
                    `FORGE_OFF_OUT_BASE:     out_base       <= reg_wr_data[4:0];
                    `FORGE_OFF_OUT_COUNT:    out_count      <= reg_wr_data[4:0];
                    `FORGE_OFF_SET_BASE:     set_base       <= reg_wr_data[4:0];
                    `FORGE_OFF_SET_COUNT:    set_count      <= reg_wr_data[4:0];
                    `FORGE_OFF_IN_BASE:      in_base        <= reg_wr_data[4:0];
                    `FORGE_OFF_SIDESET:      begin sideset_base <= reg_wr_data[4:0]; sideset_count <= reg_wr_data[7:5]; end
                    `FORGE_OFF_JMP_PIN:      jmp_pin        <= reg_wr_data[4:0];
                    `FORGE_OFF_WRAP_TOP:     wrap_top       <= reg_wr_data[5:0];
                    `FORGE_OFF_WRAP_BOT:     wrap_bot       <= reg_wr_data[5:0];
                    `FORGE_OFF_SHIFTCTRL: begin
                        in_shift_dir  <= reg_wr_data[0];
                        out_shift_dir <= reg_wr_data[1];
                        autopush      <= reg_wr_data[2];
                        autopull      <= reg_wr_data[3];
                        push_thresh   <= (reg_wr_data[7:4] == 4'd0) ? 5'd16 : {1'b0, reg_wr_data[7:4]};
                    end
                    `FORGE_OFF_PULL_THRESH:  pull_thresh    <= (reg_wr_data[3:0] == 4'd0) ? 5'd16 : {1'b0, reg_wr_data[3:0]};
                    `FORGE_OFF_EXECCTRL: begin
                        scl_stretch_en <= reg_wr_data[0];
                        arb_detect_en  <= reg_wr_data[1];
                        jmp_cond_sel   <= reg_wr_data[3:2];
                        sniffer_en     <= reg_wr_data[4];
                        scl_pin_idx    <= {1'b0, reg_wr_data[7:5]};
                    end
                    `FORGE_OFF_SBE_CTRL:     sbe_mode       <= reg_wr_data[2:0];
                    `FORGE_OFF_BSU_CTRL:     bsu_mode       <= reg_wr_data[1:0];
                    `FORGE_OFF_CRC_CTRL: begin
                        crc_poly     <= reg_wr_data[1:0];
                        crc_snoop_rx <= reg_wr_data[2];
                        crc_enable   <= reg_wr_data[3];
                        crc_invert   <= reg_wr_data[4];
                    end
                    `FORGE_OFF_CRC_VAL_L:    crc_wr_val[7:0]  <= reg_wr_data;
                    `FORGE_OFF_CRC_VAL_H: begin
                        crc_wr_val[15:8] <= reg_wr_data;
                        crc_wr_en        <= 1'b1;
                    end
                    `FORGE_OFF_TXF_STREAM_L: txf_buf_l      <= reg_wr_data;
                    default: ;
                endcase
            end

            // CSR Host Read
            case (reg_addr)
                `FORGE_OFF_DIV_INT_L:    reg_rd_data <= div_int[7:0];
                `FORGE_OFF_DIV_INT_H:    reg_rd_data <= div_int[15:8];
                `FORGE_OFF_DIV_FRAC:     reg_rd_data <= div_frac;
                `FORGE_OFF_OUT_BASE:     reg_rd_data <= {3'b0, out_base};
                `FORGE_OFF_OUT_COUNT:    reg_rd_data <= {3'b0, out_count};
                `FORGE_OFF_SET_BASE:     reg_rd_data <= {3'b0, set_base};
                `FORGE_OFF_SET_COUNT:    reg_rd_data <= {3'b0, set_count};
                `FORGE_OFF_IN_BASE:      reg_rd_data <= {3'b0, in_base};
                `FORGE_OFF_SIDESET:      reg_rd_data <= {sideset_count, sideset_base};
                `FORGE_OFF_JMP_PIN:      reg_rd_data <= {3'b0, jmp_pin};
                `FORGE_OFF_WRAP_TOP:     reg_rd_data <= {2'b0, wrap_top};
                `FORGE_OFF_WRAP_BOT:     reg_rd_data <= {2'b0, wrap_bot};
                `FORGE_OFF_SHIFTCTRL:    reg_rd_data <= {push_thresh[3:0], autopull, autopush, out_shift_dir, in_shift_dir};
                `FORGE_OFF_PULL_THRESH:  reg_rd_data <= {4'b0, pull_thresh[3:0]};
                `FORGE_OFF_EXECCTRL:     reg_rd_data <= {scl_pin_idx[2:0], sniffer_en, jmp_cond_sel, arb_detect_en, scl_stretch_en};
                `FORGE_OFF_PC:           reg_rd_data <= {2'b0, pc};
                `FORGE_OFF_STATUS:       reg_rd_data <= status_byte;
                `FORGE_OFF_SBE_CTRL:     reg_rd_data <= {5'b0, sbe_mode};
                `FORGE_OFF_BSU_CTRL:     reg_rd_data <= {6'b0, bsu_mode};
                `FORGE_OFF_CRC_CTRL:     reg_rd_data <= {3'b0, crc_invert, crc_enable, crc_snoop_rx, crc_poly};
                `FORGE_OFF_CRC_VAL_L:    reg_rd_data <= crc_val[7:0];
                `FORGE_OFF_CRC_VAL_H:    reg_rd_data <= crc_val[15:8];
                `FORGE_OFF_CAPTURE_L:    reg_rd_data <= reg_capture[7:0];
                `FORGE_OFF_CAPTURE_H:    reg_rd_data <= reg_capture[15:8];
                `FORGE_OFF_RXF_STREAM_L: reg_rd_data <= rxf_rdata[7:0];
                `FORGE_OFF_RXF_STREAM_H: reg_rd_data <= rxf_rdata[15:8];
                `FORGE_OFF_FIFO_LEVELS:  reg_rd_data <= {rxf_level[3:0], txf_level[3:0]};
                default:                 reg_rd_data <= 8'h00;
            endcase
        end
    end

endmodule
