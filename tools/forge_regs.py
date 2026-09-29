"""
ProtocolForge Register Map, Bitfield Definitions, and SPI Protocol Constants.
Architecture Specification: ARCH_PROPOSAL_CHIEF.md
"""

from typing import Dict

# ---------------------------------------------------------------------------
# Architecture Constants
# ---------------------------------------------------------------------------
NUM_CORES = 3
CORE0_IMEM_SIZE = 48
CORE1_IMEM_SIZE = 48
CORE2_IMEM_SIZE = 32
CORE0_FIFO_DEPTH = 8
CORE1_FIFO_DEPTH = 8
CORE2_FIFO_DEPTH = 4

CHIP_ID = 0xC7
CHIP_VERSION = 0x01
CLOCK_FREQ_HZ = 50_000_000
SPI_CLOCK_MAX_HZ = 12_500_000

# ---------------------------------------------------------------------------
# Global System Registers (0x00 - 0x1F)
# ---------------------------------------------------------------------------
REG_ID              = 0x00  # Chip ID (Read-only, returns 0xC7)
REG_VERSION         = 0x01  # Silicon version (returns 0x01)
REG_NUM_CORES       = 0x02  # Number of state machine cores (returns 3)
REG_ENABLE          = 0x04  # Core enable mask (bits 2:0)
REG_RESTART         = 0x05  # Core restart/reset pulse (bits 2:0)
REG_STEP            = 0x06  # Single-step execution pulse (bits 2:0)
REG_FLAGS           = 0x07  # Inter-core 8-bit flag crossbar
REG_GPIO_IN0        = 0x08  # GPIO inputs [7:0]
REG_GPIO_IN1        = 0x09  # GPIO inputs [15:8]
REG_GPIO_IN2        = 0x0A  # GPIO inputs [19:16]
REG_INFILT          = 0x0B  # 3-tap majority glitch filter enable mask
REG_ABU_MIN_PULSE_L = 0x0C  # Auto-Baud Unit minimum pulse width (low byte)
REG_ABU_MIN_PULSE_H = 0x0D  # Auto-Baud Unit minimum pulse width (high byte)
REG_PIN_OD          = 0x0E  # Open-drain mode mask for GPIO pins
REG_IRQ_STATUS      = 0x0F  # Interrupt status flags

# Global Enable bitmasks
CORE0_EN = 1 << 0
CORE1_EN = 1 << 1
CORE2_EN = 1 << 2
ALL_CORES_EN = CORE0_EN | CORE1_EN | CORE2_EN

# ---------------------------------------------------------------------------
# Per-Core Base Addresses
# ---------------------------------------------------------------------------
CORE0_BASE = 0x20
CORE1_BASE = 0x40
CORE2_BASE = 0x60

CORE_BASE: Dict[int, int] = {
    0: CORE0_BASE,
    1: CORE1_BASE,
    2: CORE2_BASE,
}

CORE_IMEM_SIZE: Dict[int, int] = {
    0: CORE0_IMEM_SIZE,
    1: CORE1_IMEM_SIZE,
    2: CORE2_IMEM_SIZE,
}

CORE_FIFO_DEPTH: Dict[int, int] = {
    0: CORE0_FIFO_DEPTH,
    1: CORE1_FIFO_DEPTH,
    2: CORE2_FIFO_DEPTH,
}

# ---------------------------------------------------------------------------
# Per-Core Register Offsets (+0x00 to +0x1F)
# ---------------------------------------------------------------------------
OFFSET_DIV_INT_L    = 0x00  # Clock divider integer part, low byte
OFFSET_DIV_INT_H    = 0x01  # Clock divider integer part, high byte
OFFSET_DIV_FRAC     = 0x02  # Clock divider fractional part (8-bit / 256)
OFFSET_OUT_BASE     = 0x03  # OUT starting pin index
OFFSET_OUT_COUNT    = 0x04  # OUT pin count (1..16)
OFFSET_SET_BASE     = 0x05  # SET starting pin index
OFFSET_SET_COUNT    = 0x06  # SET pin count (1..5)
OFFSET_IN_BASE      = 0x07  # IN starting pin index
OFFSET_SIDESET      = 0x08  # Side-set config: [4:0] = {pindirs, opt, count[2:0]}
OFFSET_JMP_PIN      = 0x09  # Pin index used for `jmp pin` condition
OFFSET_WRAP_TOP     = 0x0A  # Program wrap target address (PC)
OFFSET_WRAP_BOT     = 0x0B  # Program wrap bottom address (PC)
OFFSET_SHIFTCTRL    = 0x0C  # Autopush/autopull thresholds & shift directions
OFFSET_PC           = 0x0F  # Program Counter read/write
OFFSET_STATUS       = 0x10  # Core execution status & error flags
OFFSET_SBE_CTRL     = 0x11  # Stream Bit Engine line encoding (NRZ, NRZI, Man)
OFFSET_BSU_CTRL     = 0x12  # Bit-Stuffing Unit mode (None, USB, CAN)
OFFSET_CRC_CTRL     = 0x13  # CRC coprocessor polynomial & snoop mode
OFFSET_CRC_VAL_L    = 0x14  # Current CRC accumulator (low byte)
OFFSET_CRC_VAL_H    = 0x15  # Current CRC accumulator (high byte)
OFFSET_CAPTURE_L    = 0x16  # Sub-cycle edge timestamp register (low byte)
OFFSET_CAPTURE_H    = 0x17  # Sub-cycle edge timestamp register (high byte)
OFFSET_TXF_STREAM_L = 0x18  # 16-bit TX FIFO stream port (low byte)
OFFSET_TXF_STREAM_H = 0x19  # 16-bit TX FIFO stream port (high byte)
OFFSET_RXF_STREAM_L = 0x1A  # 16-bit RX FIFO stream port (low byte)
OFFSET_RXF_STREAM_H = 0x1B  # 16-bit RX FIFO stream port (high byte)

def core_reg(core_id: int, offset: int) -> int:
    """Compute absolute register address for a core."""
    if core_id not in CORE_BASE:
        raise ValueError(f"Invalid core_id {core_id}. Must be 0, 1, or 2.")
    return CORE_BASE[core_id] + offset

# ---------------------------------------------------------------------------
# Per-Core Status Bitfield Flags (STATUS register +0x10)
# ---------------------------------------------------------------------------
STATUS_TX_EMPTY   = 1 << 0  # TX FIFO is empty
STATUS_TX_FULL    = 1 << 1  # TX FIFO is full
STATUS_RX_EMPTY   = 1 << 2  # RX FIFO is empty
STATUS_RX_FULL    = 1 << 3  # RX FIFO is full
STATUS_STALLED    = 1 << 4  # Core is stalled waiting on FIFO, pin, or deadline
STATUS_STUFF_ERR  = 1 << 5  # Bit-stuffing rule violation detected on RX
STATUS_ARB_LOST   = 1 << 6  # Bus arbitration lost on open-drain collision
STATUS_CRC_OK     = 1 << 7  # Received frame CRC residual check matched zero

# ---------------------------------------------------------------------------
# Stream Bit Engine (SBE_CTRL +0x11)
# ---------------------------------------------------------------------------
SBE_NRZ        = 0x00  # Default NRZ pass-through
SBE_NRZI       = 0x01  # NRZI encoding/decoding (USB Low/Full Speed)
SBE_MANCHESTER = 0x02  # Manchester phase encoding (10BASE-T Ethernet)
SBE_DIFF_NRZI  = 0x03  # Differential / Inverted NRZI

# ---------------------------------------------------------------------------
# Bit-Stuffing Unit (BSU_CTRL +0x12)
# ---------------------------------------------------------------------------
BSU_DISABLED = 0x00  # No bit-stuffing
BSU_USB      = 0x01  # USB 1.1 mode: stuff '0' after six consecutive '1's
BSU_CAN      = 0x02  # CAN 2.0 mode: stuff complement after 5 identical bits

# ---------------------------------------------------------------------------
# Hardware CRC Coprocessor (CRC_CTRL +0x13)
# ---------------------------------------------------------------------------
CRC_SNOOP_TX = 0 << 4  # Snoop bits shifting out of OSR
CRC_SNOOP_RX = 1 << 4  # Snoop bits shifting into ISR

CRC_POLY_USB16   = 0x00  # CRC-16-USB: x^16 + x^15 + x^2 + 1 (0x8005)
CRC_POLY_CAN15   = 0x01  # CRC-15-CAN: x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1 (0x4599)
CRC_POLY_DALLAS8 = 0x02  # CRC-8-Dallas: x^8 + x^5 + x^4 + 1 (0x31)
CRC_POLY_USB5    = 0x03  # CRC-5-USB: x^5 + x^2 + 1 (0x05)
CRC_POLY_ETH32   = 0x04  # CRC-32-Ethernet: 0x04C11DB7

# ---------------------------------------------------------------------------
# Shift Control Bitfields (SHIFTCTRL +0x0C)
# ---------------------------------------------------------------------------
SHIFTCTRL_IN_SHIFTDIR   = 1 << 0  # 0 = Left (MSB in), 1 = Right (LSB in)
SHIFTCTRL_OUT_SHIFTDIR  = 1 << 1  # 0 = Left (MSB out), 1 = Right (LSB out)
SHIFTCTRL_AUTOPUSH_EN   = 1 << 2
SHIFTCTRL_AUTOPULL_EN   = 1 << 3
SHIFTCTRL_PUSH_THRESH_POS = 4     # [7:4] Push threshold (0..16 bits, 0=16)

def pack_shiftctrl(
    autopush: bool = False,
    autopull: bool = False,
    in_right: bool = False,
    out_right: bool = False,
    push_thresh: int = 16,
    pull_thresh: int = 16
) -> int:
    """Build SHIFTCTRL register value."""
    val = 0
    if in_right:
        val |= SHIFTCTRL_IN_SHIFTDIR
    if out_right:
        val |= SHIFTCTRL_OUT_SHIFTDIR
    if autopush:
        val |= SHIFTCTRL_AUTOPUSH_EN
    if autopull:
        val |= SHIFTCTRL_AUTOPULL_EN
    val |= (push_thresh & 0x0F) << SHIFTCTRL_PUSH_THRESH_POS
    return val

# ---------------------------------------------------------------------------
# SPI Slave Protocol Definitions
# ---------------------------------------------------------------------------
SPI_CMD_WRITE      = 0x80  # 1 = Write, 0 = Read
SPI_CMD_READ       = 0x00
SPI_CMD_IMEM       = 0x40  # 1 = Instruction Memory, 0 = Registers
SPI_CMD_REG        = 0x00
SPI_CMD_STREAM     = 0x20  # 1 = Auto-Burst 16-bit FIFO Stream, 0 = Single Register
SPI_CMD_CORE_MASK  = 0x1F  # Core Select / Address High bits

# Telemetry on MISO during Command/Address shift
TELEMETRY_CORE0_IRQ = 1 << 7
TELEMETRY_CORE1_IRQ = 1 << 6
TELEMETRY_CORE2_IRQ = 1 << 5
TELEMETRY_GLITCH    = 1 << 4
TELEMETRY_MAGIC     = 0x05  # Lower 4 bits of byte 0 always return 0x5

# Telemetry Byte 1: FIFO Levels
# byte 1 = {Core0_RX_Lvl[3:0], Core1_RX_Lvl[3:0]}
def unpack_telemetry(byte0: int, byte1: int) -> dict:
    """Unpack simultaneous telemetry bytes shifted out during SPI CMD/ADDR."""
    return {
        "core0_irq": bool(byte0 & TELEMETRY_CORE0_IRQ),
        "core1_irq": bool(byte0 & TELEMETRY_CORE1_IRQ),
        "core2_irq": bool(byte0 & TELEMETRY_CORE2_IRQ),
        "glitch_detected": bool(byte0 & TELEMETRY_GLITCH),
        "magic_valid": (byte0 & 0x0F) == TELEMETRY_MAGIC,
        "core0_rx_level": (byte1 >> 4) & 0x0F,
        "core1_rx_level": byte1 & 0x0F,
    }

# Human-readable register name map for debugging
REG_NAMES: Dict[int, str] = {
    REG_ID: "REG_ID",
    REG_VERSION: "REG_VERSION",
    REG_NUM_CORES: "REG_NUM_CORES",
    REG_ENABLE: "REG_ENABLE",
    REG_RESTART: "REG_RESTART",
    REG_STEP: "REG_STEP",
    REG_FLAGS: "REG_FLAGS",
    REG_GPIO_IN0: "REG_GPIO_IN0",
    REG_GPIO_IN1: "REG_GPIO_IN1",
    REG_GPIO_IN2: "REG_GPIO_IN2",
    REG_INFILT: "REG_INFILT",
    REG_ABU_MIN_PULSE_L: "REG_ABU_MIN_PULSE_L",
    REG_ABU_MIN_PULSE_H: "REG_ABU_MIN_PULSE_H",
    REG_PIN_OD: "REG_PIN_OD",
    REG_IRQ_STATUS: "REG_IRQ_STATUS",
}

for c_id, base in CORE_BASE.items():
    REG_NAMES[base + OFFSET_DIV_INT_L] = f"C{c_id}_DIV_INT_L"
    REG_NAMES[base + OFFSET_DIV_INT_H] = f"C{c_id}_DIV_INT_H"
    REG_NAMES[base + OFFSET_DIV_FRAC] = f"C{c_id}_DIV_FRAC"
    REG_NAMES[base + OFFSET_OUT_BASE] = f"C{c_id}_OUT_BASE"
    REG_NAMES[base + OFFSET_OUT_COUNT] = f"C{c_id}_OUT_COUNT"
    REG_NAMES[base + OFFSET_SET_BASE] = f"C{c_id}_SET_BASE"
    REG_NAMES[base + OFFSET_SET_COUNT] = f"C{c_id}_SET_COUNT"
    REG_NAMES[base + OFFSET_IN_BASE] = f"C{c_id}_IN_BASE"
    REG_NAMES[base + OFFSET_SIDESET] = f"C{c_id}_SIDESET"
    REG_NAMES[base + OFFSET_JMP_PIN] = f"C{c_id}_JMP_PIN"
    REG_NAMES[base + OFFSET_WRAP_TOP] = f"C{c_id}_WRAP_TOP"
    REG_NAMES[base + OFFSET_WRAP_BOT] = f"C{c_id}_WRAP_BOT"
    REG_NAMES[base + OFFSET_SHIFTCTRL] = f"C{c_id}_SHIFTCTRL"
    REG_NAMES[base + OFFSET_PC] = f"C{c_id}_PC"
    REG_NAMES[base + OFFSET_STATUS] = f"C{c_id}_STATUS"
    REG_NAMES[base + OFFSET_SBE_CTRL] = f"C{c_id}_SBE_CTRL"
    REG_NAMES[base + OFFSET_BSU_CTRL] = f"C{c_id}_BSU_CTRL"
    REG_NAMES[base + OFFSET_CRC_CTRL] = f"C{c_id}_CRC_CTRL"
    REG_NAMES[base + OFFSET_CRC_VAL_L] = f"C{c_id}_CRC_VAL_L"
    REG_NAMES[base + OFFSET_CRC_VAL_H] = f"C{c_id}_CRC_VAL_H"
    REG_NAMES[base + OFFSET_CAPTURE_L] = f"C{c_id}_CAPTURE_L"
    REG_NAMES[base + OFFSET_CAPTURE_H] = f"C{c_id}_CAPTURE_H"
    REG_NAMES[base + OFFSET_TXF_STREAM_L] = f"C{c_id}_TXF_STREAM_L"
    REG_NAMES[base + OFFSET_TXF_STREAM_H] = f"C{c_id}_TXF_STREAM_H"
    REG_NAMES[base + OFFSET_RXF_STREAM_L] = f"C{c_id}_RXF_STREAM_L"
    REG_NAMES[base + OFFSET_RXF_STREAM_H] = f"C{c_id}_RXF_STREAM_H"
