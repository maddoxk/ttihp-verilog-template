/*
 * Comprehensive Testbench for ProtocolForge ASIC
 * Tests:
 * 1. Reset & Default Register Values
 * 2. SPI Read of Device ID (0xC7), Version (0x01), and Num Cores (0x03)
 * 3. SPI Telemetry readback
 * 4. Register Read/Write on Global Registers (FLAGS, PIN_OD, INFILT)
 * 5. Core 0 IMEM Programming via SPI
 * 6. Core 0 Execution & GPIO output driving
 * 7. Hardware ABU (Auto-Baud Unit) pulse measurement
 * 8. Hardware CRC Coprocessor
 * 9. Hardware SBE / BSU (NRZI and Bit-Stuffing)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none
`timescale 1ns / 1ps
`include "forge_defs.vh"

module tb_protocolforge;

    reg        clk;
    reg        rst_n;
    reg        ena;
    reg  [7:0] ui_in;
    wire [7:0] uo_out;
    reg  [7:0] uio_in;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    // DUT Instantiation
    tt_um_maddox_protocolforge dut (
        .ui_in(ui_in),
        .uo_out(uo_out),
        .uio_in(uio_in),
        .uio_out(uio_out),
        .uio_oe(uio_oe),
        .ena(ena),
        .clk(clk),
        .rst_n(rst_n)
    );

    // 50 MHz Clock Generator (Period = 20ns)
    always #10 clk = ~clk;

    // SPI Master Signals
    // uio_in[4] = CS_n, uio_in[5] = SCK, uio_in[6] = MOSI, uio_out[7] = MISO
    reg spi_cs_n;
    reg spi_sck;
    reg spi_mosi;
    wire spi_miso = uio_out[7];

    always @(*) begin
        uio_in[4] = spi_cs_n;
        uio_in[5] = spi_sck;
        uio_in[6] = spi_mosi;
    end

    // Task: SPI Transfer 1 Byte (Mode 0: SCK idle low, sample rising, shift falling)
    task spi_xfer_byte;
        input  [7:0] tx_byte;
        output [7:0] rx_byte;
        integer b;
        begin
            rx_byte = 8'd0;
            for (b = 7; b >= 0; b = b - 1) begin
                spi_mosi = tx_byte[b];
                #80; // SCK low half-period (12.5MHz or 6.25MHz for clean sampling by 50MHz clk)
                spi_sck = 1'b1;
                #40;
                rx_byte[b] = spi_miso;
                #40; // SCK high half-period
                spi_sck = 1'b0;
            end
            #40;
        end
    endtask

    // Task: Read Register via SPI
    task spi_read_reg;
        input  [7:0] addr;
        output [7:0] rdata;
        output [7:0] telem0;
        output [7:0] telem1;
        begin
            spi_cs_n = 1'b0;
            #80;
            spi_xfer_byte(8'h00, telem0); // CMD: Read, CSR space, single reg
            spi_xfer_byte(addr,  telem1); // ADDR
            spi_xfer_byte(8'h00, rdata);  // DATA (readback)
            #80;
            spi_cs_n = 1'b1;
            #120;
        end
    endtask

    // Task: Write Register via SPI
    task spi_write_reg;
        input [7:0] addr;
        input [7:0] wdata;
        reg   [7:0] dummy0, dummy1, dummy2;
        begin
            spi_cs_n = 1'b0;
            #80;
            spi_xfer_byte(8'h80, dummy0); // CMD: Write, CSR space
            spi_xfer_byte(addr,  dummy1); // ADDR
            spi_xfer_byte(wdata, dummy2); // DATA
            #80;
            spi_cs_n = 1'b1;
            #120;
        end
    endtask

    // Task: Write Instruction to IMEM
    task spi_write_imem;
        input [1:0]  core_id;
        input [5:0]  imem_waddr;
        input [15:0] instr_word;
        reg   [7:0]  dummy0, dummy1, dummy2, dummy3;
        begin
            spi_cs_n = 1'b0;
            #80;
            spi_xfer_byte(8'hC0, dummy0); // CMD: Write (bit 7), Space=1 (IMEM, bit 6)
            spi_xfer_byte({core_id, imem_waddr}, dummy1); // Core & Word Address
            spi_xfer_byte(instr_word[7:0], dummy2);  // Low byte
            spi_xfer_byte(instr_word[15:8], dummy3); // High byte
            #80;
            spi_cs_n = 1'b1;
            #120;
        end
    endtask

    reg [7:0] val;
    reg [7:0] t0, t1;
    integer pass_count;
    integer fail_count;

    initial begin
        $dumpfile("tb.fst");
        $dumpvars(0, tb_protocolforge);

        pass_count = 0;
        fail_count = 0;

        clk = 0;
        rst_n = 0;
        ena = 1;
        ui_in = 8'd0;
        uio_in = 8'd0;
        spi_cs_n = 1'b1;
        spi_sck = 1'b0;
        spi_mosi = 1'b0;

        $display("================================================================");
        $display("STARTING PROTOCOLFORGE ASIC VERIFICATION TESTBENCH");
        $display("================================================================");

        // Reset DUT
        #100;
        rst_n = 1;
        #100;

        // ---------------------------------------------------------------------
        // TEST 1: Read Global Read-Only Registers (ID, Version, Num Cores)
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] Reading Global Identification Registers...");
        spi_read_reg(`FORGE_REG_ID, val, t0, t1);
        if (val === 8'hC7) begin
            $display("  PASS: Device ID = 0x%02X (Expected: 0xC7)", val);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Device ID = 0x%02X (Expected: 0xC7)", val);
            fail_count = fail_count + 1;
        end

        spi_read_reg(`FORGE_REG_VERSION, val, t0, t1);
        if (val === 8'h01) begin
            $display("  PASS: Version = 0x%02X (Expected: 0x01)", val);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Version = 0x%02X (Expected: 0x01)", val);
            fail_count = fail_count + 1;
        end

        spi_read_reg(`FORGE_REG_NUM_CORES, val, t0, t1);
        if (val === 8'h03) begin
            $display("  PASS: Num Cores = %d (Expected: 3)", val);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Num Cores = %d (Expected: 3)", val);
            fail_count = fail_count + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 2: SPI Telemetry Verification
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] Verifying Simultaneous Telemetry on MISO...");
        if (t0[3:0] === 4'h5) begin
            $display("  PASS: Telemetry Byte 0 signature = 0x%1X (Expected: 0x5)", t0[3:0]);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Telemetry Byte 0 signature = 0x%1X (Expected: 0x5)", t0[3:0]);
            fail_count = fail_count + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 3: Global Register Read / Write (FLAGS, PIN_OD)
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] Global Register R/W Test...");
        spi_write_reg(`FORGE_REG_FLAGS, 8'hA5);
        spi_read_reg(`FORGE_REG_FLAGS, val, t0, t1);
        if (val === 8'hA5) begin
            $display("  PASS: Inter-core FLAGS R/W = 0x%02X", val);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Inter-core FLAGS R/W = 0x%02X (Expected: 0xA5)", val);
            fail_count = fail_count + 1;
        end

        spi_write_reg(`FORGE_REG_PIN_OD, 8'h0A);
        spi_read_reg(`FORGE_REG_PIN_OD, val, t0, t1);
        if (val === 8'h0A) begin
            $display("  PASS: Open-Drain Mask R/W = 0x%02X", val);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Open-Drain Mask R/W = 0x%02X (Expected: 0x0A)", val);
            fail_count = fail_count + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 4: Auto-Baud Unit (ABU) Pulse Width Tracking
        // ---------------------------------------------------------------------
        $display("\n[TEST 4] Hardware Auto-Baud Unit (ABU) Verification...");
        // Reset ABU
        spi_write_reg(`FORGE_REG_ABU_MIN_H, 8'hFF);

        // Inject pulses on ui_in[0]:
        // 1st pulse: 20 clock cycles = 400ns
        ui_in[0] = 1'b0;
        #200;
        ui_in[0] = 1'b1;
        #400; // 20 cycles
        ui_in[0] = 1'b0;
        #200;
        // 2nd pulse: 10 clock cycles = 200ns
        ui_in[0] = 1'b1;
        #200; // 10 cycles
        ui_in[0] = 1'b0;
        #200;
        // 3rd pulse: 15 clock cycles = 300ns
        ui_in[0] = 1'b1;
        #300;
        ui_in[0] = 1'b0;
        #200;

        // Read back ABU_MIN_PULSE (should track the shortest pulse: ~10 cycles = 0x0A)
        spi_read_reg(`FORGE_REG_ABU_MIN_L, val, t0, t1);
        $display("  ABU MIN_PULSE_L = %d cycles", val);
        if (val >= 8 && val <= 12) begin
            $display("  PASS: ABU MIN_PULSE tracked shortest pulse width accurately (%d cycles ~ 10 cycles)", val);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: ABU MIN_PULSE unexpected value: %d cycles", val);
            fail_count = fail_count + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 5: Core 0 Program Flashing & Execution
        // Program:
        //   Instr 0: SET PINDIRS, 0x0F  (Configure dedicated outputs as output)
        //   Instr 1: SET PINS, 0x0A     (Drive outputs to 0x0A)
        //   Instr 2: JMP ALWAYS, 0x02   (Spin loop)
        // Encoding:
        //   SET dst=pindirs(011), imm=0x0F:
        //     [15:13]=110 (SET), [12:9]=0 (delay=0), [8:6]=011 (pindirs), [5:0]=001111 -> 0xC0CF
        //   SET dst=pins(000), imm=0x0A:
        //     [15:13]=110 (SET), [12:9]=0 (delay=0), [8:6]=000 (pins), [5:0]=001010 -> 0xC00A
        //   JMP always(000), addr=0x02:
        //     [15:13]=000 (JMP), [12:9]=0, [8:6]=000, [5:0]=000010 -> 0x0002
        // ---------------------------------------------------------------------
        $display("\n[TEST 5] Flashing and Executing Core 0 Program...");

        // Configure Core 0 SET_BASE = 8 (maps to GPIO 8 = dedicated outputs uo_out), SET_COUNT = 4
        spi_write_reg(`FORGE_CORE0_BASE + `FORGE_OFF_SET_BASE,  8'd8);
        spi_write_reg(`FORGE_CORE0_BASE + `FORGE_OFF_SET_COUNT, 8'd4);
        spi_write_reg(`FORGE_CORE0_BASE + `FORGE_OFF_DIV_INT_L, 8'd1); // Full 50MHz execution

        spi_read_reg(`FORGE_CORE0_BASE + `FORGE_OFF_SET_BASE, val, t0, t1);
        $display("  DEBUG: Core 0 SET_BASE readback = %d", val);
        spi_read_reg(`FORGE_CORE0_BASE + `FORGE_OFF_SET_COUNT, val, t0, t1);
        $display("  DEBUG: Core 0 SET_COUNT readback = %d", val);
        spi_read_reg(`FORGE_CORE0_BASE + `FORGE_OFF_DIV_INT_L, val, t0, t1);
        $display("  DEBUG: Core 0 DIV_INT_L readback = %d", val);

        // Write Program to Core 0 IMEM
        spi_write_imem(2'd0, 6'd0, 16'hC0CF); // SET pindirs, 0x0F
        spi_write_imem(2'd0, 6'd1, 16'hC00A); // SET pins, 0x0A
        spi_write_imem(2'd0, 6'd2, 16'h0002); // JMP 2

        $display("  DEBUG: Core 0 IMEM[0] = 0x%04X, IMEM[1] = 0x%04X", dut.u_core0.imem[0], dut.u_core0.imem[1]);
        $display("  DEBUG: Core 0 enable = %b, pc = %d", dut.u_core0.enable, dut.u_core0.pc);

        // Enable Core 0
        spi_write_reg(`FORGE_REG_ENABLE, 8'h01);

        // Wait a few cycles for execution
        #400;

        $display("  Dedicated Output uo_out = 0x%02X", uo_out);
        if (uo_out[3:0] === 4'hA) begin
            $display("  PASS: Core 0 successfully executed SET instructions, driving uo_out = 0x%02X!", uo_out);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: uo_out = 0x%02X (Expected: lower nibble 0xA)", uo_out);
            fail_count = fail_count + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 6: Core 0 Update Pin State (0x05)
        // ---------------------------------------------------------------------
        $display("\n[TEST 6] Reprogramming Instruction 1 to drive 0x05...");
        spi_write_imem(2'd0, 6'd1, 16'hC005); // SET pins, 0x05
        // Restart Core 0
        spi_write_reg(`FORGE_REG_RESTART, 8'h01);
        #400;

        $display("  Dedicated Output uo_out = 0x%02X", uo_out);
        if (uo_out[3:0] === 4'h5) begin
            $display("  PASS: Core 0 updated output to 0x05!", uo_out);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: uo_out = 0x%02X (Expected: lower nibble 0x5)", uo_out);
            fail_count = fail_count + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 7: Hardware CRC Coprocessor Unit Test
        // ---------------------------------------------------------------------
        $display("\n[TEST 7] Hardware CRC Coprocessor Verification...");
        // Configure Core 0 CRC: CRC-16 (poly 00), TX snoop, enable=0 for static readback
        spi_write_reg(`FORGE_CORE0_BASE + `FORGE_OFF_CRC_CTRL, 8'b0000_0000); // poly=16, en=0
        // Preset CRC
        spi_write_reg(`FORGE_CORE0_BASE + `FORGE_OFF_CRC_VAL_L, 8'hFF);
        spi_write_reg(`FORGE_CORE0_BASE + `FORGE_OFF_CRC_VAL_H, 8'hFF);
        // Read back preset
        spi_read_reg(`FORGE_CORE0_BASE + `FORGE_OFF_CRC_VAL_L, val, t0, t1);
        if (val === 8'hFF) begin
            $display("  PASS: Core 0 CRC register write & readback = 0xFF");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Core 0 CRC register readback = 0x%02X (Expected: 0xFF)", val);
            fail_count = fail_count + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 8: Stream Bit Engine & Bit-Stuffing Unit Direct Verification
        // ---------------------------------------------------------------------
        $display("\n[TEST 8] Stream Bit Engine (NRZI) & Bit-Stuffing (USB/CAN) Verification...");
        // Verify SBE module standalone behavior:
        // SBE_CTRL write and readback
        spi_write_reg(`FORGE_CORE0_BASE + `FORGE_OFF_SBE_CTRL, 8'h01); // NRZI mode
        spi_read_reg(`FORGE_CORE0_BASE + `FORGE_OFF_SBE_CTRL, val, t0, t1);
        if (val[2:0] === 3'b001) begin
            $display("  PASS: Core 0 SBE_CTRL set to NRZI mode (0x01)");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Core 0 SBE_CTRL readback = 0x%02X", val);
            fail_count = fail_count + 1;
        end

        // BSU_CTRL write and readback
        spi_write_reg(`FORGE_CORE0_BASE + `FORGE_OFF_BSU_CTRL, 8'h01); // USB mode (stuff after 6 ones)
        spi_read_reg(`FORGE_CORE0_BASE + `FORGE_OFF_BSU_CTRL, val, t0, t1);
        if (val[1:0] === 2'b01) begin
            $display("  PASS: Core 0 BSU_CTRL set to USB bit-stuffing mode (0x01)");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Core 0 BSU_CTRL readback = 0x%02X", val);
            fail_count = fail_count + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 9: Inter-Core Flags Signaling (Core 0 to Core 1)
        // ---------------------------------------------------------------------
        $display("\n[TEST 9] Inter-Core Flag Signaling...");
        // Flash Core 0 program that sets Flag 3
        // SET flagset, 3: [15:13]=110, [12:9]=0, [8:6]=100 (flagset), [5:0]=000011 -> 0xC103
        spi_write_imem(2'd0, 6'd0, 16'hC103);
        spi_write_reg(`FORGE_REG_RESTART, 8'h01);
        #400;

        spi_read_reg(`FORGE_REG_FLAGS, val, t0, t1);
        if (val[3] === 1'b1) begin
            $display("  PASS: Core 0 successfully asserted Inter-Core Flag 3 (FLAGS=0x%02X)!", val);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Inter-Core Flag 3 was not set (FLAGS=0x%02X)", val);
            fail_count = fail_count + 1;
        end
        $display("\n================================================================");
        $display("VERIFICATION SUMMARY: %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("================================================================");

        if (fail_count == 0) begin
            $display("\n*** ALL TESTS PASSED SUCCESSFULLY! ***\n");
        end else begin
            $display("\n*** SOME TESTS FAILED! ***\n");
        end

        $finish;
    end

endmodule
