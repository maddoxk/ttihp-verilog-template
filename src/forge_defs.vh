/*
 * ProtocolForge Definitions & Constants
 * SPDX-License-Identifier: Apache-2.0
 */

`ifndef FORGE_DEFS_VH
`define FORGE_DEFS_VH

// =============================================================================
// Instruction Opcodes (Bits [15:13])
// =============================================================================
`define FORGE_OP_JMP        3'b000
`define FORGE_OP_WAIT       3'b001
`define FORGE_OP_IN         3'b010
`define FORGE_OP_OUT        3'b011
`define FORGE_OP_PUSH_PULL  3'b100
`define FORGE_OP_MOV        3'b101
`define FORGE_OP_SET        3'b110
`define FORGE_OP_TIME       3'b111

// =============================================================================
// JMP Conditions (Bits [8:6])
// =============================================================================
`define FORGE_COND_ALWAYS      3'b000
`define FORGE_COND_NOT_X       3'b001  // X == 0
`define FORGE_COND_POST_DEC_X  3'b010  // if X != 0: X <= X - 1, branch
`define FORGE_COND_NOT_Y       3'b011  // Y == 0
`define FORGE_COND_POST_DEC_Y  3'b100  // if Y != 0: Y <= Y - 1, branch
`define FORGE_COND_X_NEQ_Y     3'b101  // X != Y
`define FORGE_COND_PIN         3'b110  // JMP_PIN == 1
`define FORGE_COND_EXT         3'b111  // !crc (CRC residual == 0) or arb_lost

// =============================================================================
// WAIT Sources (Bits [7:6])
// =============================================================================
`define FORGE_WAIT_SRC_GPIO    2'b00   // Absolute GPIO [n]
`define FORGE_WAIT_SRC_PIN     2'b01   // Relative Pin [(IN_BASE + n) % 20]
`define FORGE_WAIT_SRC_FLAG    2'b10   // Inter-core Flag [n]
`define FORGE_WAIT_SRC_EXT     2'b11   // idx=0: time (T >= DL); idx!=0: edge

// =============================================================================
// IN Sources (Bits [8:6])
// =============================================================================
`define FORGE_IN_SRC_PINS      3'b000  // GPIOs starting from IN_BASE
`define FORGE_IN_SRC_X         3'b001
`define FORGE_IN_SRC_Y         3'b010
`define FORGE_IN_SRC_NULL      3'b011  // 0s
`define FORGE_IN_SRC_T         3'b100  // Timebase snapshot
`define FORGE_IN_SRC_STATUS    3'b101  // Core status register
`define FORGE_IN_SRC_CRC       3'b110  // Current CRC accumulator
`define FORGE_IN_SRC_CAPTURE   3'b111  // Sub-cycle edge timestamp snapshot

// =============================================================================
// OUT Destinations (Bits [8:6])
// =============================================================================
`define FORGE_OUT_DST_PINS     3'b000  // GPIOs starting from OUT_BASE
`define FORGE_OUT_DST_X        3'b001
`define FORGE_OUT_DST_Y        3'b010
`define FORGE_OUT_DST_NULL     3'b011  // Discard
`define FORGE_OUT_DST_PINDIRS  3'b100  // Pin output enables
`define FORGE_OUT_DST_PC       3'b101  // Computed jump
`define FORGE_OUT_DST_ISR      3'b110  // Write to ISR and set shift count
`define FORGE_OUT_DST_DL       3'b111  // Direct deadline write

// =============================================================================
// MOV Operations (Bits [5:4])
// =============================================================================
`define FORGE_MOV_OP_NONE      2'b00   // Direct copy: src
`define FORGE_MOV_OP_INVERT    2'b01   // Bitwise invert: ~src
`define FORGE_MOV_OP_BITREV    2'b10   // Bit-reverse: ::src
`define FORGE_MOV_OP_BYTESWAP  2'b11   // Byte-swap: {src[7:0], src[15:8]}

// =============================================================================
// MOV Destinations (Bits [8:6])
// =============================================================================
`define FORGE_MOV_DST_PINS     3'b000
`define FORGE_MOV_DST_X        3'b001
`define FORGE_MOV_DST_Y        3'b010
`define FORGE_MOV_DST_DL       3'b011
`define FORGE_MOV_DST_PINDIRS  3'b100
`define FORGE_MOV_DST_PC       3'b101
`define FORGE_MOV_DST_ISR      3'b110
`define FORGE_MOV_DST_OSR      3'b111

// =============================================================================
// SET Destinations (Bits [8:6])
// =============================================================================
`define FORGE_SET_DST_PINS     3'b000  // Drive SET_COUNT pins from SET_BASE
`define FORGE_SET_DST_X        3'b001
`define FORGE_SET_DST_Y        3'b010
`define FORGE_SET_DST_PINDIRS  3'b011
`define FORGE_SET_DST_FLAGSET  3'b100  // Set flag imm[2:0] = 1
`define FORGE_SET_DST_FLAGCLR  3'b101  // Clear flag imm[2:0] = 0
`define FORGE_SET_DST_T        3'b110  // Load timebase T
`define FORGE_SET_DST_CRC      3'b111  // 0=clear to 0, 1=preset 0xFFFF

// =============================================================================
// TIME Modes (Bits [8:7])
// =============================================================================
`define FORGE_TIME_T_ADD_N     2'b00   // DL = T + imm
`define FORGE_TIME_DL_ADD_N    2'b01   // DL = DL + imm (jitter-free)
`define FORGE_TIME_T_ADD_X     2'b10   // DL = T + X
`define FORGE_TIME_DL_ADD_X    2'b11   // DL = DL + X

// =============================================================================
// Stream Bit Engine (SBE) Modes
// =============================================================================
`define FORGE_SBE_NRZ          3'b000  // Standard level pass-through
`define FORGE_SBE_NRZI         3'b001  // Toggle on 0, maintain on 1 (USB)
`define FORGE_SBE_MANCHESTER   3'b010  // Mid-bit transition: 1=0->1, 0=1->0 (10BASE-T)
`define FORGE_SBE_INV_NRZI     3'b011  // Inverted NRZI / differential pair

// =============================================================================
// Bit-Stuffing Unit (BSU) Modes
// =============================================================================
`define FORGE_BSU_DIS          2'b00   // Disabled
`define FORGE_BSU_USB          2'b01   // USB: stuff 0 after 6 consecutive 1s
`define FORGE_BSU_CAN          2'b10   // CAN: stuff complement after 5 identical bits

// =============================================================================
// CRC Polynomials
// =============================================================================
`define FORGE_CRC_16           2'b00   // 0x8005 (x^16 + x^15 + x^2 + 1)
`define FORGE_CRC_15           2'b01   // 0x4599 (x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1)
`define FORGE_CRC_8            2'b10   // 0x31   (x^8 + x^5 + x^4 + 1)
`define FORGE_CRC_5            2'b11   // 0x05   (x^5 + x^2 + 1)

// =============================================================================
// Host Register Map Addresses
// =============================================================================
// Global Registers (0x00 - 0x1F)
`define FORGE_REG_ID           8'h00   // Device ID (0xC7)
`define FORGE_REG_VERSION      8'h01   // Version (0x01)
`define FORGE_REG_NUM_CORES    8'h02   // Number of Cores (3)
`define FORGE_REG_ENABLE       8'h04   // Core Enables [2:0]
`define FORGE_REG_RESTART      8'h05   // Core Synchronous Reset pulse [2:0]
`define FORGE_REG_STEP         8'h06   // Single-step trigger [2:0]
`define FORGE_REG_FLAGS        8'h07   // Inter-core Flags [7:0]
`define FORGE_REG_GPIO_IN_0    8'h08   // ui_in[7:0]
`define FORGE_REG_GPIO_IN_1    8'h09   // uo_out[7:0]
`define FORGE_REG_GPIO_IN_2    8'h0A   // uio_in[3:0]
`define FORGE_REG_INFILT       8'h0B   // Majority glitch filter enable mask
`define FORGE_REG_ABU_MIN_L    8'h0C   // Auto-Baud MIN_PULSE low byte
`define FORGE_REG_ABU_MIN_H    8'h0D   // Auto-Baud MIN_PULSE high byte
`define FORGE_REG_PIN_OD       8'h0E   // Open-Drain mask for GPIOs 19..16
`define FORGE_REG_IRQ_STATUS   8'h0F   // Interrupt status flags

// Core Register Bases
`define FORGE_CORE0_BASE       8'h20
`define FORGE_CORE1_BASE       8'h40
`define FORGE_CORE2_BASE       8'h60

// Core Register Offsets (+0x00 to +0x1F)
`define FORGE_OFF_DIV_INT_L    5'h00   // Divider integer low byte
`define FORGE_OFF_DIV_INT_H    5'h01   // Divider integer high byte
`define FORGE_OFF_DIV_FRAC     5'h02   // Divider fractional byte
`define FORGE_OFF_OUT_BASE     5'h03   // OUT base pin (0..19)
`define FORGE_OFF_OUT_COUNT    5'h04   // OUT pin count (1..20)
`define FORGE_OFF_SET_BASE     5'h05   // SET base pin (0..19)
`define FORGE_OFF_SET_COUNT    5'h06   // SET pin count (1..5)
`define FORGE_OFF_IN_BASE      5'h07   // IN base pin (0..19)
`define FORGE_OFF_SIDESET      5'h08   // [4:0]=base, [7:5]=count
`define FORGE_OFF_JMP_PIN      5'h09   // JMP pin index (0..19)
`define FORGE_OFF_WRAP_TOP     5'h0A   // Wrap top address (0..47/31)
`define FORGE_OFF_WRAP_BOT     5'h0B   // Wrap bottom address (0..47/31)
`define FORGE_OFF_SHIFTCTRL    5'h0C   // [0]=in_dir, [1]=out_dir, [2]=autopush, [3]=autopull, [7:4]=push_thresh
`define FORGE_OFF_PULL_THRESH  5'h0D   // [3:0]=pull_thresh
`define FORGE_OFF_EXECCTRL     5'h0E   // [0]=scl_stretch, [1]=arb_detect, [2]=jmp_cond_sel, [3]=sniffer_en, [7:4]=scl_pin
`define FORGE_OFF_PC           5'h0F   // Program Counter
`define FORGE_OFF_STATUS       5'h10   // [0]=tx_e, [1]=tx_f, [2]=rx_e, [3]=rx_f, [4]=stalled, [5]=stuff_err, [6]=arb_lost, [7]=crc_ok
`define FORGE_OFF_SBE_CTRL     5'h11   // SBE mode [2:0]
`define FORGE_OFF_BSU_CTRL     5'h12   // BSU mode [1:0]
`define FORGE_OFF_CRC_CTRL     5'h13   // [1:0]=poly, [2]=snoop (0=TX,1=RX), [3]=en, [4]=invert
`define FORGE_OFF_CRC_VAL_L    5'h14   // CRC accumulator low byte
`define FORGE_OFF_CRC_VAL_H    5'h15   // CRC accumulator high byte
`define FORGE_OFF_CAPTURE_L    5'h16   // Edge timestamp capture low byte
`define FORGE_OFF_CAPTURE_H    5'h17   // Edge timestamp capture high byte
`define FORGE_OFF_TXF_STREAM_L 5'h18   // TX FIFO data low byte
`define FORGE_OFF_TXF_STREAM_H 5'h19   // TX FIFO data high byte (triggers 16-bit push)
`define FORGE_OFF_RXF_STREAM_L 5'h1A   // RX FIFO data low byte
`define FORGE_OFF_RXF_STREAM_H 5'h1B   // RX FIFO data high byte (triggers 16-bit pop)
`define FORGE_OFF_FIFO_LEVELS  5'h1C   // [3:0]=tx_level, [7:4]=rx_level

`endif // FORGE_DEFS_VH
