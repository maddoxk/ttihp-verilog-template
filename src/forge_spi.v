/*
 * ProtocolForge Fast SPI Slave Host Interface (Mode 0, clk/4 = 12.5MHz)
 * Features simultaneous telemetry, 16-bit burst FIFO streaming,
 * and direct IMEM programming.
 * Cycle-accurate registered pipeline for 100% reliable core accesses.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none
`include "forge_defs.vh"

module forge_spi (
    input  wire        clk,
    input  wire        rst_n,

    // SPI Physical Pins (Mode 0)
    input  wire        cs_n,
    input  wire        sck,
    input  wire        mosi,
    output wire        miso,

    // Telemetry Inputs
    input  wire [7:0]  core0_status,
    input  wire [7:0]  core1_status,
    input  wire [7:0]  core2_status,
    input  wire [3:0]  core0_rx_lvl,
    input  wire [3:0]  core1_rx_lvl,
    input  wire        abu_edge_event,

    // Global Registers
    output reg  [2:0]  core_enable,
    output reg  [2:0]  core_restart,
    output reg  [2:0]  core_step,
    input  wire [7:0]  flags_in,
    output reg  [7:0]  flags_out,
    output reg         flags_wr_en,
    input  wire [19:0] gpio_in_all,
    output reg  [11:0] infilt_mask,
    output reg  [3:0]  pin_od_mask,

    // ABU Interface
    input  wire [15:0] abu_min_pulse,
    output reg         abu_clear,
    output reg         abu_wr_en,
    output reg  [15:0] abu_wr_data,

    // Core Register File Access (Cores 0, 1, 2)
    output reg  [2:0]  core_reg_wr,
    output reg  [4:0]  core_reg_addr,
    output reg  [7:0]  core_reg_wdata,
    input  wire [7:0]  core0_reg_rdata,
    input  wire [7:0]  core1_reg_rdata,
    input  wire [7:0]  core2_reg_rdata,

    // Core IMEM Programming
    output reg  [2:0]  imem_wr_en,
    output reg  [5:0]  imem_addr,
    output reg  [15:0] imem_wdata,
    input  wire [15:0] core0_imem_rdata,
    input  wire [15:0] core1_imem_rdata,
    input  wire [15:0] core2_imem_rdata,

    // 16-bit Burst FIFO DMA Streaming
    output reg  [2:0]  tx_stream_push,
    output reg  [15:0] tx_stream_wdata,
    output reg  [2:0]  rx_stream_pop,
    input  wire [15:0] core0_rx_stream_data,
    input  wire [15:0] core1_rx_stream_data,
    input  wire [15:0] core2_rx_stream_data
);

    // =========================================================================
    // DUAL-RANK SYNCHRONIZERS FOR METASTABILITY HARDENING
    // =========================================================================
    reg [1:0] sync_cs_n;
    reg [2:0] sync_sck;
    reg [1:0] sync_mosi;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_cs_n <= 2'b11;
            sync_sck  <= 3'b000;
            sync_mosi <= 2'b00;
        end else begin
            sync_cs_n <= {sync_cs_n[0], cs_n};
            sync_sck  <= {sync_sck[1:0], sck};
            sync_mosi <= {sync_mosi[0], mosi};
        end
    end

    wire cs_active = !sync_cs_n[1];
    wire sck_rise  = (sync_sck[2:1] == 2'b01);
    wire sck_fall  = (sync_sck[2:1] == 2'b10);

    // =========================================================================
    // PROTOCOL STATE MACHINE & REGISTERS
    // =========================================================================
    localparam STATE_CMD  = 2'd0;
    localparam STATE_ADDR = 2'd1;
    localparam STATE_DATA = 2'd2;

    reg [1:0]  spi_state;
    reg [2:0]  bit_cnt;
    reg [7:0]  mosi_shift;
    reg [7:0]  miso_shift;
    reg        miso_out_reg;

    reg [7:0]  cmd_reg;
    reg [7:0]  addr_reg;
    reg [5:0]  imem_ptr;
    reg        byte_toggle;   // 0 for low byte, 1 for high byte in 16-bit mode
    reg [7:0]  data_buf_l;

    // Current byte as sampled on SCK rising edge
    wire [7:0] current_rx_byte = {mosi_shift[6:0], sync_mosi[1]};

    wire cmd_write  = cmd_reg[7];
    wire cmd_space  = cmd_reg[6]; // 0=CSR, 1=IMEM
    wire cmd_stream = cmd_reg[5]; // 1=Burst FIFO streaming

    // Identify target core from command or address
    wire [1:0] core_sel = (cmd_space || cmd_stream) ?
                          ((cmd_reg[1:0] != 2'd0) ? cmd_reg[1:0] : addr_reg[7:6]) :
                          (addr_reg[7:5] == 3'b001) ? 2'd0 :
                          (addr_reg[7:5] == 3'b010) ? 2'd1 :
                          (addr_reg[7:5] == 3'b011) ? 2'd2 : 2'd3;

    // Telemetry Bytes
    wire [7:0] telem_byte0 = {core0_status[7], core1_status[7], core2_status[7], abu_edge_event, 4'h5};
    wire [7:0] telem_byte1 = {core0_rx_lvl, core1_rx_lvl};

    // Current Register Read Value Mux
    reg [7:0] reg_read_val;
    always @(*) begin
        if (addr_reg < 8'h20) begin
            // Global Registers
            case (addr_reg)
                `FORGE_REG_ID:        reg_read_val = 8'hC7;
                `FORGE_REG_VERSION:   reg_read_val = 8'h01;
                `FORGE_REG_NUM_CORES: reg_read_val = 8'h03;
                `FORGE_REG_ENABLE:    reg_read_val = {5'b0, core_enable};
                `FORGE_REG_FLAGS:     reg_read_val = flags_in;
                `FORGE_REG_GPIO_IN_0: reg_read_val = gpio_in_all[7:0];
                `FORGE_REG_GPIO_IN_1: reg_read_val = gpio_in_all[15:8];
                `FORGE_REG_GPIO_IN_2: reg_read_val = {4'b0, gpio_in_all[19:16]};
                `FORGE_REG_INFILT:    reg_read_val = infilt_mask[7:0];
                `FORGE_REG_ABU_MIN_L: reg_read_val = abu_min_pulse[7:0];
                `FORGE_REG_ABU_MIN_H: reg_read_val = abu_min_pulse[15:8];
                `FORGE_REG_PIN_OD:    reg_read_val = {4'b0, pin_od_mask};
                `FORGE_REG_IRQ_STATUS:reg_read_val = {core0_status[7:5], core1_status[7:5], core2_status[7:6]};
                default:              reg_read_val = 8'h00;
            endcase
        end else begin
            case (core_sel)
                2'd0:    reg_read_val = core0_reg_rdata;
                2'd1:    reg_read_val = core1_reg_rdata;
                2'd2:    reg_read_val = core2_reg_rdata;
                default: reg_read_val = 8'h00;
            endcase
        end
    end

    // Current IMEM Read Value Mux
    wire [15:0] imem_read_word = (core_sel == 2'd0) ? core0_imem_rdata :
                                 (core_sel == 2'd1) ? core1_imem_rdata : core2_imem_rdata;

    // Current RX Stream Data Mux
    wire [15:0] rx_stream_word = (core_sel == 2'd0) ? core0_rx_stream_data :
                                 (core_sel == 2'd1) ? core1_rx_stream_data : core2_rx_stream_data;

    // Next MISO Data Byte Preparation
    reg [7:0] next_miso_byte;
    always @(*) begin
        case (spi_state)
            STATE_CMD:  next_miso_byte = telem_byte1;
            STATE_ADDR: begin
                if (cmd_space) begin
                    next_miso_byte = imem_read_word[7:0];
                end else if (cmd_stream) begin
                    next_miso_byte = rx_stream_word[7:0];
                end else begin
                    next_miso_byte = reg_read_val;
                end
            end
            STATE_DATA: begin
                if (cmd_space) begin
                    next_miso_byte = byte_toggle ? imem_read_word[15:8] : imem_read_word[7:0];
                end else if (cmd_stream) begin
                    next_miso_byte = byte_toggle ? rx_stream_word[15:8] : rx_stream_word[7:0];
                end else begin
                    next_miso_byte = reg_read_val;
                end
            end
            default: next_miso_byte = 8'h00;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_state       <= STATE_CMD;
            bit_cnt         <= 3'd0;
            mosi_shift      <= 8'd0;
            miso_shift      <= 8'd0;
            miso_out_reg    <= 1'b0;
            cmd_reg         <= 8'd0;
            addr_reg        <= 8'd0;
            imem_ptr        <= 6'd0;
            byte_toggle     <= 1'b0;
            data_buf_l      <= 8'd0;

            core_enable     <= 3'b000;
            core_restart    <= 3'b000;
            core_step       <= 3'b000;
            flags_out       <= 8'd0;
            flags_wr_en     <= 1'b0;
            infilt_mask     <= 12'd0;
            pin_od_mask     <= 4'd0;

            abu_clear       <= 1'b0;
            abu_wr_en       <= 1'b0;
            abu_wr_data     <= 16'd0;

            core_reg_wr     <= 3'b000;
            core_reg_addr   <= 5'd0;
            core_reg_wdata  <= 8'd0;

            imem_wr_en      <= 3'b000;
            imem_addr       <= 6'd0;
            imem_wdata      <= 16'd0;

            tx_stream_push  <= 3'b000;
            tx_stream_wdata <= 16'd0;
            rx_stream_pop   <= 3'b000;
        end else if (!cs_active) begin
            // Transaction terminated / inactive CS
            spi_state       <= STATE_CMD;
            bit_cnt         <= 3'd0;
            byte_toggle     <= 1'b0;
            miso_out_reg    <= telem_byte0[7];
            miso_shift      <= {telem_byte0[6:0], 1'b0};

            core_restart    <= 3'b000;
            core_step       <= 3'b000;
            flags_wr_en     <= 1'b0;
            abu_clear       <= 1'b0;
            abu_wr_en       <= 1'b0;
            core_reg_wr     <= 3'b000;
            imem_wr_en      <= 3'b000;
            tx_stream_push  <= 3'b000;
            rx_stream_pop   <= 3'b000;
        end else begin
            // Clear single-cycle strobes
            core_restart   <= 3'b000;
            core_step      <= 3'b000;
            flags_wr_en    <= 1'b0;
            abu_clear      <= 1'b0;
            abu_wr_en      <= 1'b0;
            core_reg_wr    <= 3'b000;
            imem_wr_en     <= 3'b000;
            tx_stream_push <= 3'b000;
            rx_stream_pop  <= 3'b000;

            // SCK Rising: Sample MOSI
            if (sck_rise) begin
                mosi_shift <= {mosi_shift[6:0], sync_mosi[1]};
                bit_cnt    <= bit_cnt + 3'd1;

                if (bit_cnt == 3'd7) begin
                    case (spi_state)
                        STATE_CMD: begin
                            cmd_reg   <= current_rx_byte;
                            spi_state <= STATE_ADDR;
                        end

                        STATE_ADDR: begin
                            addr_reg      <= current_rx_byte;
                            imem_ptr      <= current_rx_byte[5:0];
                            imem_addr     <= current_rx_byte[5:0];
                            core_reg_addr <= current_rx_byte[4:0];
                            byte_toggle   <= 1'b0;
                            spi_state     <= STATE_DATA;
                        end

                        STATE_DATA: begin
                            if (cmd_write) begin
                                if (cmd_space) begin
                                    // 16-bit IMEM Programming
                                    if (byte_toggle == 1'b0) begin
                                        data_buf_l  <= current_rx_byte;
                                        byte_toggle <= 1'b1;
                                    end else begin
                                        imem_wdata <= {current_rx_byte, data_buf_l};
                                        imem_addr  <= imem_ptr;
                                        case (core_sel)
                                            2'd0: imem_wr_en[0] <= 1'b1;
                                            2'd1: imem_wr_en[1] <= 1'b1;
                                            2'd2: imem_wr_en[2] <= 1'b1;
                                            default: ;
                                        endcase
                                        imem_ptr    <= imem_ptr + 1'b1;
                                        byte_toggle <= 1'b0;
                                    end
                                end else if (cmd_stream) begin
                                    // 16-bit Burst FIFO DMA TX Stream
                                    if (byte_toggle == 1'b0) begin
                                        data_buf_l  <= current_rx_byte;
                                        byte_toggle <= 1'b1;
                                    end else begin
                                        tx_stream_wdata <= {current_rx_byte, data_buf_l};
                                        case (core_sel)
                                            2'd0: tx_stream_push[0] <= 1'b1;
                                            2'd1: tx_stream_push[1] <= 1'b1;
                                            2'd2: tx_stream_push[2] <= 1'b1;
                                            default: ;
                                        endcase
                                        byte_toggle <= 1'b0;
                                    end
                                end else begin
                                    // Register Write
                                    if (addr_reg < 8'h20) begin
                                        case (addr_reg)
                                            `FORGE_REG_ENABLE:  core_enable  <= current_rx_byte[2:0];
                                            `FORGE_REG_RESTART: core_restart <= current_rx_byte[2:0];
                                            `FORGE_REG_STEP:    core_step    <= current_rx_byte[2:0];
                                            `FORGE_REG_FLAGS: begin
                                                flags_out   <= current_rx_byte;
                                                flags_wr_en <= 1'b1;
                                            end
                                            `FORGE_REG_INFILT:  infilt_mask[7:0] <= current_rx_byte;
                                            `FORGE_REG_ABU_MIN_L: begin
                                                abu_wr_data[7:0] <= current_rx_byte;
                                            end
                                            `FORGE_REG_ABU_MIN_H: begin
                                                abu_wr_data[15:8] <= current_rx_byte;
                                                abu_wr_en         <= 1'b1;
                                            end
                                            `FORGE_REG_PIN_OD:  pin_od_mask <= current_rx_byte[3:0];
                                            default: ;
                                        endcase
                                    end else begin
                                        core_reg_addr  <= addr_reg[4:0];
                                        core_reg_wdata <= current_rx_byte;
                                        case (core_sel)
                                            2'd0: core_reg_wr[0] <= 1'b1;
                                            2'd1: core_reg_wr[1] <= 1'b1;
                                            2'd2: core_reg_wr[2] <= 1'b1;
                                            default: ;
                                        endcase
                                    end
                                    addr_reg <= addr_reg + 1'b1;
                                end
                            end else begin
                                // Read Mode
                                if (cmd_space) begin
                                    if (byte_toggle == 1'b1) begin
                                        imem_ptr    <= imem_ptr + 1'b1;
                                        imem_addr   <= imem_ptr + 1'b1;
                                        byte_toggle <= 1'b0;
                                    end else begin
                                        byte_toggle <= 1'b1;
                                    end
                                end else if (cmd_stream) begin
                                    if (byte_toggle == 1'b1) begin
                                        case (core_sel)
                                            2'd0: rx_stream_pop[0] <= 1'b1;
                                            2'd1: rx_stream_pop[1] <= 1'b1;
                                            2'd2: rx_stream_pop[2] <= 1'b1;
                                            default: ;
                                        endcase
                                        byte_toggle <= 1'b0;
                                    end else begin
                                        byte_toggle <= 1'b1;
                                    end
                                end else begin
                                    addr_reg      <= addr_reg + 1'b1;
                                    core_reg_addr <= addr_reg[4:0] + 1'b1;
                                end
                            end
                        end
                    endcase
                end
            end

            // SCK Falling: Shift MISO
            if (sck_fall) begin
                if (bit_cnt == 3'd0) begin
                    miso_out_reg <= next_miso_byte[7];
                    miso_shift   <= {next_miso_byte[6:0], 1'b0};
                end else begin
                    miso_out_reg <= miso_shift[7];
                    miso_shift   <= {miso_shift[6:0], 1'b0};
                end
            end
        end
    end

    assign miso = miso_out_reg;

endmodule
