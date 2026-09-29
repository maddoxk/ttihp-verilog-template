"""
ProtocolForge Python Host Driver API.
Communicates with ProtocolForge chip over 12.5 MHz SPI Slave Interface.
Specification: ARCH_PROPOSAL_CHIEF.md
"""

from __future__ import annotations
from typing import List, Dict, Optional, Tuple, Any, Protocol
import collections

from tools.forge_regs import (
    NUM_CORES, CHIP_ID, CHIP_VERSION, CLOCK_FREQ_HZ,
    REG_ID, REG_VERSION, REG_NUM_CORES, REG_ENABLE, REG_RESTART, REG_STEP,
    REG_FLAGS, REG_GPIO_IN0, REG_GPIO_IN1, REG_GPIO_IN2, REG_INFILT,
    REG_ABU_MIN_PULSE_L, REG_ABU_MIN_PULSE_H, REG_PIN_OD, REG_IRQ_STATUS,
    CORE_BASE, CORE_IMEM_SIZE, CORE_FIFO_DEPTH,
    OFFSET_DIV_INT_L, OFFSET_DIV_INT_H, OFFSET_DIV_FRAC,
    OFFSET_OUT_BASE, OFFSET_OUT_COUNT, OFFSET_SET_BASE, OFFSET_SET_COUNT,
    OFFSET_IN_BASE, OFFSET_SIDESET, OFFSET_JMP_PIN, OFFSET_WRAP_TOP, OFFSET_WRAP_BOT,
    OFFSET_SHIFTCTRL, OFFSET_PC, OFFSET_STATUS, OFFSET_SBE_CTRL, OFFSET_BSU_CTRL,
    OFFSET_CRC_CTRL, OFFSET_CRC_VAL_L, OFFSET_CRC_VAL_H,
    OFFSET_CAPTURE_L, OFFSET_CAPTURE_H,
    OFFSET_TXF_STREAM_L, OFFSET_TXF_STREAM_H, OFFSET_RXF_STREAM_L, OFFSET_RXF_STREAM_H,
    SPI_CMD_WRITE, SPI_CMD_READ, SPI_CMD_IMEM, SPI_CMD_REG, SPI_CMD_STREAM,
    SPI_CMD_CORE_MASK,
    STATUS_TX_EMPTY, STATUS_TX_FULL, STATUS_RX_EMPTY, STATUS_RX_FULL,
    STATUS_STALLED, STATUS_STUFF_ERR, STATUS_ARB_LOST, STATUS_CRC_OK,
    SBE_NRZ, BSU_DISABLED,
    unpack_telemetry, core_reg
)


class SpiTransport(Protocol):
    """Abstract SPI transport interface."""
    def transfer(self, tx_data: bytes) -> bytes:
        """Full-duplex SPI transfer."""
        ...


class MockSpiTransport:
    """
    Bit-accurate in-memory simulator of ProtocolForge SPI Slave interface
    and register space for development and unit testing.
    """
    def __init__(self):
        # Global register file
        self.regs: Dict[int, int] = {
            REG_ID: CHIP_ID,
            REG_VERSION: CHIP_VERSION,
            REG_NUM_CORES: NUM_CORES,
            REG_ENABLE: 0x00,
            REG_RESTART: 0x00,
            REG_STEP: 0x00,
            REG_FLAGS: 0x00,
            REG_GPIO_IN0: 0x00,
            REG_GPIO_IN1: 0x00,
            REG_GPIO_IN2: 0x00,
            REG_INFILT: 0x00,
            REG_ABU_MIN_PULSE_L: 0x32,  # Example: 50 cycles default
            REG_ABU_MIN_PULSE_H: 0x00,
            REG_PIN_OD: 0x00,
            REG_IRQ_STATUS: 0x00,
        }

        # Per-core registers
        for c in range(NUM_CORES):
            base = CORE_BASE[c]
            self.regs[base + OFFSET_DIV_INT_L] = 1
            self.regs[base + OFFSET_DIV_INT_H] = 0
            self.regs[base + OFFSET_DIV_FRAC] = 0
            self.regs[base + OFFSET_STATUS] = STATUS_TX_EMPTY | STATUS_RX_EMPTY
            self.regs[base + OFFSET_PC] = 0

        # Instruction memories (per core)
        self.imem: Dict[int, List[int]] = {
            c: [0] * CORE_IMEM_SIZE[c] for c in range(NUM_CORES)
        }

        # FIFOs
        self.tx_fifo: Dict[int, collections.deque] = {c: collections.deque() for c in range(NUM_CORES)}
        self.rx_fifo: Dict[int, collections.deque] = {c: collections.deque() for c in range(NUM_CORES)}

        # Telemetry state
        self.irqs = [False, False, False]
        self.glitch_detected = False

    def transfer(self, tx_data: bytes) -> bytes:
        if len(tx_data) < 2:
            return b"\x00" * len(tx_data)

        cmd = tx_data[0]
        addr = tx_data[1]
        data = tx_data[2:]

        is_write = bool(cmd & SPI_CMD_WRITE)
        is_imem = bool(cmd & SPI_CMD_IMEM)
        is_stream = bool(cmd & SPI_CMD_STREAM)
        core_id = cmd & SPI_CMD_CORE_MASK

        # Build telemetry bytes shifted out during byte 0 and byte 1
        t_byte0 = (0x05) | (int(self.glitch_detected) << 4)
        if self.irqs[0]: t_byte0 |= (1 << 7)
        if self.irqs[1]: t_byte0 |= (1 << 6)
        if self.irqs[2]: t_byte0 |= (1 << 5)

        c0_lvl = min(15, len(self.rx_fifo[0]))
        c1_lvl = min(15, len(self.rx_fifo[1]))
        t_byte1 = (c0_lvl << 4) | c1_lvl

        rx_payload = bytearray()

        if is_imem:
            # Instruction Memory Access
            if is_write:
                # 2 bytes per instruction word
                for i in range(0, len(data), 2):
                    if i + 1 < len(data):
                        word = data[i] | (data[i + 1] << 8)
                        target_addr = addr + (i // 2)
                        if core_id in self.imem and target_addr < len(self.imem[core_id]):
                            self.imem[core_id][target_addr] = word
            else:
                for i in range(0, len(data), 2):
                    target_addr = addr + (i // 2)
                    word = self.imem[core_id][target_addr] if (core_id in self.imem and target_addr < len(self.imem[core_id])) else 0
                    rx_payload.append(word & 0xFF)
                    rx_payload.append((word >> 8) & 0xFF)

        elif is_stream:
            # Streaming FIFO access
            if is_write:
                # Write to TX FIFO (2 bytes per 16-bit word)
                for i in range(0, len(data), 2):
                    if i + 1 < len(data):
                        word = data[i] | (data[i + 1] << 8)
                        if core_id in self.tx_fifo:
                            self.tx_fifo[core_id].append(word)
                rx_payload = bytearray(len(data))
            else:
                # Read from RX FIFO
                for i in range(0, len(data), 2):
                    word = self.rx_fifo[core_id].popleft() if (core_id in self.rx_fifo and self.rx_fifo[core_id]) else 0
                    rx_payload.append(word & 0xFF)
                    rx_payload.append((word >> 8) & 0xFF)

        else:
            # Single Register Access
            if is_write:
                if len(data) > 0:
                    self.regs[addr] = data[0]
                rx_payload.append(0x00)
            else:
                val = self.regs.get(addr, 0x00)
                rx_payload.append(val)

        return bytes([t_byte0, t_byte1]) + bytes(rx_payload)


class ForgeHost:
    """
    Python Host Driver for ProtocolForge.
    Controls execution, loads microcode, streams TX/RX data, configures pin routing,
    and reads diagnostic telemetry over SPI.
    """

    def __init__(self, transport: Optional[SpiTransport] = None):
        self.transport: SpiTransport = transport if transport is not None else MockSpiTransport()
        self.last_telemetry: Dict[str, Any] = {}

    def _spi_transfer(self, tx_bytes: bytes) -> bytes:
        rx_bytes = self.transport.transfer(tx_bytes)
        if len(rx_bytes) >= 2:
            self.last_telemetry = unpack_telemetry(rx_bytes[0], rx_bytes[1])
        return rx_bytes

    def get_last_telemetry(self) -> Dict[str, Any]:
        """Return telemetry metadata captured during most recent SPI transfer."""
        return self.last_telemetry

    # -----------------------------------------------------------------------
    # Register Access
    # -----------------------------------------------------------------------
    def write_reg(self, addr: int, val: int) -> None:
        """Write single 8-bit register."""
        cmd = SPI_CMD_WRITE | SPI_CMD_REG | ((addr >> 8) & SPI_CMD_CORE_MASK)
        tx = bytes([cmd, addr & 0xFF, val & 0xFF])
        self._spi_transfer(tx)

    def read_reg(self, addr: int) -> int:
        """Read single 8-bit register."""
        cmd = SPI_CMD_READ | SPI_CMD_REG | ((addr >> 8) & SPI_CMD_CORE_MASK)
        tx = bytes([cmd, addr & 0xFF, 0x00])
        rx = self._spi_transfer(tx)
        if len(rx) >= 3:
            return rx[2]
        return 0

    def write_core_reg(self, core_id: int, offset: int, val: int) -> None:
        """Write per-core register by core ID and offset."""
        self.write_reg(core_reg(core_id, offset), val)

    def read_core_reg(self, core_id: int, offset: int) -> int:
        """Read per-core register by core ID and offset."""
        return self.read_reg(core_reg(core_id, offset))

    # -----------------------------------------------------------------------
    # Instruction Memory Loading
    # -----------------------------------------------------------------------
    def load_imem(self, core_id: int, code_words: List[int], start_addr: int = 0) -> None:
        """
        Load microcode words into Core's local instruction memory.
        Uses burst SPI write.
        """
        if core_id not in CORE_BASE:
            raise ValueError(f"Invalid core_id {core_id}")
        max_size = CORE_IMEM_SIZE[core_id]
        if start_addr + len(code_words) > max_size:
            raise ValueError(f"Code size {len(code_words)} exceeds Core {core_id} memory limit ({max_size} words)")

        cmd = SPI_CMD_WRITE | SPI_CMD_IMEM | (core_id & SPI_CMD_CORE_MASK)
        payload = bytearray([cmd, start_addr & 0xFF])
        for w in code_words:
            payload.append(w & 0xFF)
            payload.append((w >> 8) & 0xFF)

        self._spi_transfer(bytes(payload))

    # -----------------------------------------------------------------------
    # Core Execution Control
    # -----------------------------------------------------------------------
    def enable_cores(self, mask: int) -> None:
        """Enable state machine cores (bits 2:0 for Cores 2, 1, 0)."""
        self.write_reg(REG_ENABLE, mask & 0x07)

    def disable_cores(self) -> None:
        """Disable all state machine cores."""
        self.write_reg(REG_ENABLE, 0x00)

    def restart_cores(self, mask: int = 0x07) -> None:
        """Pulse restart/reset for selected cores."""
        self.write_reg(REG_RESTART, mask & 0x07)

    def step_cores(self, mask: int) -> None:
        """Single-step clock tick for selected cores."""
        self.write_reg(REG_STEP, mask & 0x07)

    def set_flags(self, flags: int) -> None:
        """Set inter-core 8-bit flag crossbar register."""
        self.write_reg(REG_FLAGS, flags & 0xFF)

    def read_flags(self) -> int:
        """Read inter-core 8-bit flag crossbar register."""
        return self.read_reg(REG_FLAGS)

    # -----------------------------------------------------------------------
    # FIFO Streaming
    # -----------------------------------------------------------------------
    def push_tx(self, core_id: int, words: List[int]) -> None:
        """
        Stream 16-bit words into Core's TX FIFO using 16-bit auto-burst streaming.
        """
        if not words:
            return
        addr = core_reg(core_id, OFFSET_TXF_STREAM_L)
        cmd = SPI_CMD_WRITE | SPI_CMD_REG | SPI_CMD_STREAM | (core_id & SPI_CMD_CORE_MASK)
        payload = bytearray([cmd, addr & 0xFF])
        for w in words:
            payload.append(w & 0xFF)
            payload.append((w >> 8) & 0xFF)
        self._spi_transfer(bytes(payload))

    def pop_rx(self, core_id: int, count: int) -> List[int]:
        """
        Stream 16-bit words out of Core's RX FIFO using 16-bit auto-burst streaming.
        """
        if count <= 0:
            return []
        addr = core_reg(core_id, OFFSET_RXF_STREAM_L)
        cmd = SPI_CMD_READ | SPI_CMD_REG | SPI_CMD_STREAM | (core_id & SPI_CMD_CORE_MASK)
        dummy_data = bytes([0x00] * (2 * count))
        tx = bytes([cmd, addr & 0xFF]) + dummy_data
        rx = self._spi_transfer(tx)

        # rx[0:2] is telemetry; rx[2:] is payload data
        payload = rx[2:]
        words = []
        for i in range(0, len(payload), 2):
            if i + 1 < len(payload):
                word = payload[i] | (payload[i + 1] << 8)
                words.append(word)
        return words

    # -----------------------------------------------------------------------
    # Hardware Status & Telemetry
    # -----------------------------------------------------------------------
    def read_status(self, core_id: int) -> int:
        """Read Core execution status register."""
        return self.read_core_reg(core_id, OFFSET_STATUS)

    def get_status_dict(self, core_id: int) -> Dict[str, bool]:
        """Read status register and return human-readable flags."""
        s = self.read_status(core_id)
        return {
            "tx_empty": bool(s & STATUS_TX_EMPTY),
            "tx_full": bool(s & STATUS_TX_FULL),
            "rx_empty": bool(s & STATUS_RX_EMPTY),
            "rx_full": bool(s & STATUS_RX_FULL),
            "stalled": bool(s & STATUS_STALLED),
            "stuff_err": bool(s & STATUS_STUFF_ERR),
            "arb_lost": bool(s & STATUS_ARB_LOST),
            "crc_ok": bool(s & STATUS_CRC_OK),
        }

    def read_auto_baud(self) -> int:
        """Read 16-bit minimum pulse width measured by Auto-Baud Unit (ABU)."""
        low = self.read_reg(REG_ABU_MIN_PULSE_L)
        high = self.read_reg(REG_ABU_MIN_PULSE_H)
        return low | (high << 8)

    def get_auto_baud_hz(self, clock_hz: int = CLOCK_FREQ_HZ) -> Optional[float]:
        """Compute detected baud rate in Hz based on minimum observed pulse width."""
        min_pulse = self.read_auto_baud()
        if min_pulse == 0:
            return None
        return clock_hz / min_pulse

    # -----------------------------------------------------------------------
    # Core & Pin Configuration Helpers
    # -----------------------------------------------------------------------
    def configure_clock(self, core_id: int, div_int: int, div_frac: int = 0) -> None:
        """Set fractional clock divider (16.8 fixed point)."""
        self.write_core_reg(core_id, OFFSET_DIV_INT_L, div_int & 0xFF)
        self.write_core_reg(core_id, OFFSET_DIV_INT_H, (div_int >> 8) & 0xFF)
        self.write_core_reg(core_id, OFFSET_DIV_FRAC, div_frac & 0xFF)

    def configure_pins(
        self,
        core_id: int,
        out_base: int = 0,
        out_count: int = 1,
        set_base: int = 0,
        set_count: int = 1,
        in_base: int = 0,
        jmp_pin: int = 0,
        sideset: int = 0
    ) -> None:
        """Configure pin mappings for a core."""
        self.write_core_reg(core_id, OFFSET_OUT_BASE, out_base)
        self.write_core_reg(core_id, OFFSET_OUT_COUNT, out_count)
        self.write_core_reg(core_id, OFFSET_SET_BASE, set_base)
        self.write_core_reg(core_id, OFFSET_SET_COUNT, set_count)
        self.write_core_reg(core_id, OFFSET_IN_BASE, in_base)
        self.write_core_reg(core_id, OFFSET_JMP_PIN, jmp_pin)
        self.write_core_reg(core_id, OFFSET_SIDESET, sideset)

    def configure_accelerators(
        self,
        core_id: int,
        sbe_mode: int = SBE_NRZ,
        bsu_mode: int = BSU_DISABLED,
        crc_mode: int = 0
    ) -> None:
        """Configure Stream Bit Engine, Bit-Stuffing Unit, and CRC coprocessor."""
        self.write_core_reg(core_id, OFFSET_SBE_CTRL, sbe_mode)
        self.write_core_reg(core_id, OFFSET_BSU_CTRL, bsu_mode)
        self.write_core_reg(core_id, OFFSET_CRC_CTRL, crc_mode)

    def set_open_drain(self, pin_mask: int) -> None:
        """Enable hardware open-drain mode on specified pins."""
        self.write_reg(REG_PIN_OD, pin_mask & 0xFF)
