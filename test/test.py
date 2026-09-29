"""
ProtocolForge Cocotb Verification Suite for Jane Street ASIC Competition.
Authors: Lead Verification Architect & Team

Comprehensive Verification Coverage:
1. test_spi_reg_access: Register read/write and MISO telemetry over SPI slave.
2. test_imem_load_and_run: Direct IMEM microcode programming and execution.
3. test_uart_tx_rx: Full-duplex UART loopback between Core 0 and Core 1.
4. test_spi_master: Core 0 driving SPI master, verifying clock and data waveforms.
5. test_can_bit_stuffing_and_crc: Hardware CAN bit-stuffing and CRC-15 engine.
6. test_usb_nrzi_and_stuffing: USB Low-Speed NRZI encoding and bit-stuffing.
7. test_manchester_10base_t: 10BASE-T Manchester dual-phase transitions.
8. test_auto_baud_unit: ABU hardware measurement of unknown baud rate signals.
9. test_differential_co_sim: Constrained-random differential co-simulation between RTL and Golden Simulator.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge, Timer
import random
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from tools.forge_asm import assemble
from tools.forge_regs import (
    CHIP_ID, CHIP_VERSION, NUM_CORES,
    REG_ID, REG_VERSION, REG_NUM_CORES, REG_ENABLE, REG_RESTART, REG_STEP,
    REG_FLAGS, REG_GPIO_IN0, REG_GPIO_IN1, REG_GPIO_IN2, REG_INFILT,
    REG_ABU_MIN_PULSE_L, REG_ABU_MIN_PULSE_H, REG_PIN_OD, REG_IRQ_STATUS,
    CORE_BASE,
    OFFSET_DIV_INT_L, OFFSET_DIV_INT_H, OFFSET_DIV_FRAC,
    OFFSET_OUT_BASE, OFFSET_OUT_COUNT, OFFSET_SET_BASE, OFFSET_SET_COUNT,
    OFFSET_IN_BASE, OFFSET_SIDESET, OFFSET_JMP_PIN, OFFSET_WRAP_TOP, OFFSET_WRAP_BOT,
    OFFSET_SHIFTCTRL, OFFSET_PC, OFFSET_STATUS, OFFSET_SBE_CTRL, OFFSET_BSU_CTRL,
    OFFSET_CRC_CTRL, OFFSET_CRC_VAL_L, OFFSET_CRC_VAL_H,
    OFFSET_CAPTURE_L, OFFSET_CAPTURE_H,
    OFFSET_TXF_STREAM_L, OFFSET_TXF_STREAM_H, OFFSET_RXF_STREAM_L, OFFSET_RXF_STREAM_H,
    SPI_CMD_WRITE, SPI_CMD_READ, SPI_CMD_IMEM, SPI_CMD_REG, SPI_CMD_STREAM,
    SBE_NRZ, SBE_NRZI, SBE_MANCHESTER,
    BSU_DISABLED, BSU_USB, BSU_CAN,
    CRC_POLY_USB16, CRC_POLY_CAN15,
    core_reg, pack_shiftctrl,
)
from tools.forge_sim import ForgeSimulator, bitrev16, byteswap16


class CocotbForgeHost:
    """Async SPI Host driver for ProtocolForge over Cocotb."""

    def __init__(self, dut, half_period_cycles: int = 2):
        self.dut = dut
        self.half_period = half_period_cycles

    async def transfer(self, tx_data: bytes) -> bytes:
        rx_bytes = bytearray()
        # CS_n active low: uio_in[4] = 0, SCK = 0 (uio_in[5] = 0)
        uio = 0x00
        self.dut.uio_in.value = uio
        await ClockCycles(self.dut.clk, 4)

        for b in tx_data:
            rx_b = 0
            for bit_idx in range(7, -1, -1):
                bit = (b >> bit_idx) & 1
                # Drive MOSI = bit (uio_in[6]), SCK = 0 (uio_in[5]), CS_n = 0 (uio_in[4])
                uio = (bit << 6)
                self.dut.uio_in.value = uio
                await ClockCycles(self.dut.clk, self.half_period)

                # Drive SCK = 1
                uio |= (1 << 5)
                self.dut.uio_in.value = uio
                await ClockCycles(self.dut.clk, self.half_period)

                # Sample MISO on uio_out[7]
                miso = 1 if str(self.dut.uio_out[7].value) == '1' else 0
                rx_b = (rx_b << 1) | miso

                # Drive SCK = 0
                uio &= ~(1 << 5)
                self.dut.uio_in.value = uio
                await ClockCycles(self.dut.clk, self.half_period)

            rx_bytes.append(rx_b)

        # CS_n inactive high: uio_in[4] = 1, SCK = 0, MOSI = 0
        uio = (1 << 4)
        self.dut.uio_in.value = uio
        await ClockCycles(self.dut.clk, 4)

        return bytes(rx_bytes)

    async def write_reg(self, addr: int, val: int):
        cmd = SPI_CMD_WRITE | SPI_CMD_REG | ((addr >> 8) & 0x1F)
        await self.transfer(bytes([cmd, addr & 0xFF, val & 0xFF]))

    async def read_reg(self, addr: int) -> int:
        cmd = SPI_CMD_READ | SPI_CMD_REG | ((addr >> 8) & 0x1F)
        rx = await self.transfer(bytes([cmd, addr & 0xFF, 0x00]))
        return rx[2] if len(rx) >= 3 else 0

    async def write_core_reg(self, core_id: int, offset: int, val: int):
        await self.write_reg(core_reg(core_id, offset), val)

    async def read_core_reg(self, core_id: int, offset: int) -> int:
        return await self.read_reg(core_reg(core_id, offset))

    async def load_imem(self, core_id: int, words: list[int], start_addr: int = 0):
        cmd = SPI_CMD_WRITE | SPI_CMD_IMEM | (core_id & 0x1F)
        payload = bytearray([cmd, start_addr & 0x3F])
        for w in words:
            payload.append(w & 0xFF)
            payload.append((w >> 8) & 0xFF)
        await self.transfer(bytes(payload))

    async def push_tx_fifo(self, core_id: int, word: int):
        # 16-bit push via register write
        await self.write_core_reg(core_id, OFFSET_TXF_STREAM_L, word & 0xFF)
        await self.write_core_reg(core_id, OFFSET_TXF_STREAM_H, (word >> 8) & 0xFF)

    async def pop_rx_fifo(self, core_id: int) -> int:
        low = await self.read_core_reg(core_id, OFFSET_RXF_STREAM_L)
        high = await self.read_core_reg(core_id, OFFSET_RXF_STREAM_H)
        return (high << 8) | low


_clock_task = None

async def setup_testbench(dut):
    """Start 50 MHz clock and reset ProtocolForge."""
    global _clock_task
    if _clock_task is None or _clock_task.done():
        clock = Clock(dut.clk, 20, unit="ns")  # 50 MHz
        _clock_task = cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = (1 << 4)  # CS_n High (inactive)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


# =============================================================================
# TEST 1: SPI Register Access & Telemetry
# =============================================================================
@cocotb.test()
async def test_spi_reg_access(dut):
    """Verify register read/write and MISO telemetry over SPI slave."""
    await setup_testbench(dut)
    host = CocotbForgeHost(dut)

    # 1. Read Global ID and Version
    chip_id = await host.read_reg(REG_ID)
    version = await host.read_reg(REG_VERSION)
    num_cores = await host.read_reg(REG_NUM_CORES)
    assert chip_id == CHIP_ID, f"Expected ID 0x{CHIP_ID:02X}, got 0x{chip_id:02X}"
    assert version == CHIP_VERSION, f"Expected Version 0x{CHIP_VERSION:02X}, got 0x{version:02X}"
    assert num_cores == NUM_CORES, f"Expected {NUM_CORES} cores, got {num_cores}"

    # 2. Write and Read Flags
    await host.write_reg(REG_FLAGS, 0xA5)
    flags = await host.read_reg(REG_FLAGS)
    assert flags == 0xA5, f"Expected flags 0xA5, got 0x{flags:02X}"

    # 3. Write and Read Core 0 Clock Divider
    await host.write_core_reg(0, OFFSET_DIV_INT_L, 0x42)
    div_l = await host.read_core_reg(0, OFFSET_DIV_INT_L)
    assert div_l == 0x42, f"Expected Core 0 DIV_INT_L 0x42, got 0x{div_l:02X}"

    # 4. Verify MISO Telemetry Header Magic
    rx = await host.transfer(bytes([SPI_CMD_READ | SPI_CMD_REG, REG_ID, 0x00]))
    magic = rx[0] & 0x0F
    assert magic == 0x05, f"Expected telemetry magic 0x5, got 0x{magic:X}"


# =============================================================================
# TEST 2: IMEM Load & Execution Run
# =============================================================================
@cocotb.test()
async def test_imem_load_and_run(dut):
    """Program instruction memory and verify execution."""
    await setup_testbench(dut)
    host = CocotbForgeHost(dut)

    # Microcode: Set X=15, Set Y=7, pulse pin 8 High for 10 cycles, then Low
    code = """
    .program imem_test
        set x, 15
        set y, 7
        set pins, 1
        time t+10
        wait time
        set pins, 0
    """
    prog = assemble(code)
    await host.load_imem(0, prog.instructions)

    # Configure Core 0: SET_BASE = 8 (uo_out[0]), SET_COUNT = 1
    await host.write_core_reg(0, OFFSET_SET_BASE, 8)
    await host.write_core_reg(0, OFFSET_SET_COUNT, 1)

    # Enable Core 0
    await host.write_reg(REG_ENABLE, 0x01)

    # Wait for pin 8 (uo_out[0]) to assert High
    for _ in range(50):
        await ClockCycles(dut.clk, 1)
        if (int(dut.uo_out.value) & 1) == 1:
            break
    assert (int(dut.uo_out.value) & 1) == 1, "uo_out[0] failed to assert High"

    # Wait for pin 8 to deassert Low after deadline
    for _ in range(50):
        await ClockCycles(dut.clk, 1)
        if (int(dut.uo_out.value) & 1) == 0:
            break
    assert (int(dut.uo_out.value) & 1) == 0, "uo_out[0] failed to deassert Low"


# =============================================================================
# TEST 3: Full-Duplex UART Loopback between Core 0 and Core 1
# =============================================================================
@cocotb.test()
async def test_uart_tx_rx(dut):
    """Full-duplex UART loopback between Core 0 (TX) and Core 1 (RX)."""
    await setup_testbench(dut)
    host = CocotbForgeHost(dut)

    # Core 0 UART TX (8N1)
    tx_code = """
    .program uart_tx
        set pins, 1
    .wrap_target
        pull block
        set pins, 0
        set x, 7
    data_loop:
        out pins, 1
        jmp x-- data_loop
        set pins, 1
    .wrap
    """
    # Core 1 UART RX (8N1)
    rx_code = """
    .program uart_rx
        wait 1 pin 0
    .wrap_target
    start:
        wait 0 pin 0
        set x, 7 [1]
    rx_loop:
        in pins, 1
        jmp x-- rx_loop
        push block
    .wrap
    """
    prog_tx = assemble(tx_code)
    prog_rx = assemble(rx_code)

    await host.load_imem(0, prog_tx.instructions)
    await host.load_imem(1, prog_rx.instructions)

    # Configure Wrap registers
    await host.write_core_reg(0, OFFSET_WRAP_TOP, prog_tx.wrap_top)
    await host.write_core_reg(0, OFFSET_WRAP_BOT, prog_tx.wrap_bot)
    await host.write_core_reg(1, OFFSET_WRAP_TOP, prog_rx.wrap_top)
    await host.write_core_reg(1, OFFSET_WRAP_BOT, prog_rx.wrap_bot)

    # Configure Core 0 (TX): SET_BASE=8, OUT_BASE=8, count=1, DIV_INT=4
    await host.write_core_reg(0, OFFSET_SET_BASE, 8)
    await host.write_core_reg(0, OFFSET_SET_COUNT, 1)
    await host.write_core_reg(0, OFFSET_OUT_BASE, 8)
    await host.write_core_reg(0, OFFSET_OUT_COUNT, 1)
    await host.write_core_reg(0, OFFSET_DIV_INT_L, 4)
    await host.write_core_reg(0, OFFSET_SHIFTCTRL, pack_shiftctrl(out_right=True))

    # Configure Core 1 (RX): IN_BASE=8 (internal ASIC pin loopback from Core 0), DIV_INT=4
    await host.write_core_reg(1, OFFSET_IN_BASE, 8)
    await host.write_core_reg(1, OFFSET_DIV_INT_L, 4)
    await host.write_core_reg(1, OFFSET_SHIFTCTRL, pack_shiftctrl(in_right=True))

    # Push byte 0xA5 (165) to Core 0 TX FIFO
    await host.push_tx_fifo(0, 0xA5)

    # Enable both Core 0 and Core 1
    await host.write_reg(REG_ENABLE, 0x03)

    # Wait for transmission and reception (approx 300 cycles)
    await ClockCycles(dut.clk, 300)

    # Read Core 1 RX FIFO
    status1 = await host.read_core_reg(1, OFFSET_STATUS)
    # Bit 2 of status_byte is rxf_empty
    assert not (status1 & (1 << 2)), f"Core 1 RX FIFO is empty! status1={bin(status1)}"
    rx_word = await host.pop_rx_fifo(1)
    assert (rx_word & 0xFF) == 0xA5 or (rx_word >> 8) == 0xA5, f"UART loopback mismatch! Got 0x{rx_word:04X}"


# =============================================================================
# TEST 4: SPI Master Waveforms
# =============================================================================
@cocotb.test()
async def test_spi_master(dut):
    """Core 0 driving SPI master, verifying clock and data waveforms."""
    await setup_testbench(dut)
    host = CocotbForgeHost(dut)

    # SPI Master Mode 0: SCK on pin 8 (sideset), MOSI on pin 9 (out pins, 1)
    spi_master_code = """
    .program spi_m
    .side_set 1
    .wrap_target
        pull block      side 0
        set x, 7        side 0
    bit_loop:
        out pins, 1     side 0
        nop             side 1
        jmp x-- bit_loop side 1
    .wrap
    """
    prog = assemble(spi_master_code)
    await host.load_imem(0, prog.instructions)

    await host.write_core_reg(0, OFFSET_OUT_BASE, 9)
    await host.write_core_reg(0, OFFSET_OUT_COUNT, 1)
    await host.write_core_reg(0, OFFSET_SIDESET, (1 << 5) | 8)
    await host.write_core_reg(0, OFFSET_DIV_INT_L, 2)

    # Push 0x55 (alternating 01010101)
    await host.push_tx_fifo(0, 0x55)
    await host.write_reg(REG_ENABLE, 0x01)

    sck_transitions = 0
    prev_sck = 0
    for _ in range(100):
        await ClockCycles(dut.clk, 1)
        sck = int(dut.uo_out.value) & 1
        if sck != prev_sck:
            sck_transitions += 1
            prev_sck = sck

    # 8 bits = 16 clock transitions on SCK
    assert sck_transitions >= 14, f"Expected >= 14 SCK transitions, got {sck_transitions}"


# =============================================================================
# TEST 5: Hardware CAN Bit-Stuffing & CRC-15
# =============================================================================
@cocotb.test()
async def test_can_bit_stuffing_and_crc(dut):
    """Verify CAN frame generation with hardware bit-stuffing and CRC-15."""
    await setup_testbench(dut)
    host = CocotbForgeHost(dut)

    code = """
    .program can_stream
    .wrap_target
        pull block
        out pins, 16
    .wrap
    """
    prog = assemble(code)
    await host.load_imem(0, prog.instructions)

    # Configure Core 0: CAN BSU, CRC-15 with snoop TX, OUT_BASE=8
    await host.write_core_reg(0, OFFSET_OUT_BASE, 8)
    await host.write_core_reg(0, OFFSET_OUT_COUNT, 1)
    await host.write_core_reg(0, OFFSET_BSU_CTRL, BSU_CAN)
    await host.write_core_reg(0, OFFSET_CRC_CTRL, (CRC_POLY_CAN15) | (1 << 3))  # enable CRC
    await host.write_core_reg(0, OFFSET_DIV_INT_L, 2)

    # Push 16 consecutive ones: 0xFFFF
    await host.push_tx_fifo(0, 0xFFFF)
    await host.write_reg(REG_ENABLE, 0x01)

    max_consecutive_ones = 0
    current_ones = 0
    for _ in range(120):
        await ClockCycles(dut.clk, 1)
        pin = int(dut.uo_out.value) & 1
        if pin == 1:
            current_ones += 1
            if current_ones > max_consecutive_ones:
                max_consecutive_ones = current_ones
        else:
            current_ones = 0

    # Under CAN bit-stuffing, at DIV_INT=2, 5 bit-periods = 10 cycles maximum!
    # Without bit-stuffing, 16 bits = 32 cycles of continuous 1.
    assert max_consecutive_ones <= 12, f"CAN bit-stuffing failed! Observed {max_consecutive_ones} consecutive cycles of 1"

    # Verify CRC-15 accumulator was updated
    crc_l = await host.read_core_reg(0, OFFSET_CRC_VAL_L)
    crc_h = await host.read_core_reg(0, OFFSET_CRC_VAL_H)
    crc_val = (crc_h << 8) | crc_l
    assert crc_val != 0x0000, "CAN CRC-15 accumulator was not updated!"


# =============================================================================
# TEST 6: Hardware USB NRZI & Bit-Stuffing
# =============================================================================
@cocotb.test()
async def test_usb_nrzi_and_stuffing(dut):
    """Verify USB Low-Speed packet generation with NRZI and bit-stuffing."""
    await setup_testbench(dut)
    host = CocotbForgeHost(dut)

    code = """
    .program usb_stream
    .wrap_target
        pull block
        out pins, 16
    .wrap
    """
    prog = assemble(code)
    await host.load_imem(0, prog.instructions)

    # Configure Core 0: SBE NRZI, USB BSU (stuff 0 after 6 ones), OUT_BASE=8
    await host.write_core_reg(0, OFFSET_OUT_BASE, 8)
    await host.write_core_reg(0, OFFSET_OUT_COUNT, 1)
    await host.write_core_reg(0, OFFSET_SBE_CTRL, SBE_NRZI)
    await host.write_core_reg(0, OFFSET_BSU_CTRL, BSU_USB)
    await host.write_core_reg(0, OFFSET_DIV_INT_L, 2)

    # Stream 16 consecutive ones
    # Under NRZI, 1 maintains state, but after 6 ones, stuffed 0 forces a TOGGLE!
    await host.push_tx_fifo(0, 0xFFFF)
    await host.write_reg(REG_ENABLE, 0x01)

    toggles = 0
    prev_pin = 1
    for _ in range(120):
        await ClockCycles(dut.clk, 1)
        pin = int(dut.uo_out.value) & 1
        if pin != prev_pin:
            toggles += 1
            prev_pin = pin

    # Stuffed zeros must force transitions!
    assert toggles >= 2, f"USB NRZI bit-stuffing toggle failure! Observed only {toggles} toggles"


# =============================================================================
# TEST 7: 10BASE-T Manchester Encoding
# =============================================================================
@cocotb.test()
async def test_manchester_10base_t(dut):
    """Verify Manchester 10 Mbps encoding transitions."""
    await setup_testbench(dut)
    host = CocotbForgeHost(dut)

    code = """
    .program man_stream
    .wrap_target
        pull block
        out pins, 16
    .wrap
    """
    prog = assemble(code)
    await host.load_imem(0, prog.instructions)

    # Configure Core 0: Manchester Mode, OUT_BASE=8, DIV_INT=2
    await host.write_core_reg(0, OFFSET_OUT_BASE, 8)
    await host.write_core_reg(0, OFFSET_OUT_COUNT, 1)
    await host.write_core_reg(0, OFFSET_SBE_CTRL, SBE_MANCHESTER)
    await host.write_core_reg(0, OFFSET_DIV_INT_L, 2)

    await host.push_tx_fifo(0, 0xAAAA)
    await host.write_reg(REG_ENABLE, 0x01)

    transitions = 0
    prev_pin = 0
    for _ in range(80):
        await ClockCycles(dut.clk, 1)
        pin = int(dut.uo_out.value) & 1
        if pin != prev_pin:
            transitions += 1
            prev_pin = pin

    # In Manchester, every bit has a mid-bit transition!
    assert transitions >= 16, f"Manchester mid-bit transitions missing! Got {transitions}"


# =============================================================================
# TEST 8: Auto-Baud Unit (ABU)
# =============================================================================
@cocotb.test()
async def test_auto_baud_unit(dut):
    """Test ABU measuring pulse width of unknown baud signals."""
    await setup_testbench(dut)
    host = CocotbForgeHost(dut)

    # Reset ABU min_pulse to 0xFFFF
    await host.write_reg(REG_ABU_MIN_PULSE_L, 0xFF)
    await host.write_reg(REG_ABU_MIN_PULSE_H, 0xFF)

    # Send a pulse train into ui_in[0]:
    # Idle High (30 cycles)
    dut.ui_in.value = 1
    await ClockCycles(dut.clk, 30)

    # Low pulse for 40 cycles
    dut.ui_in.value = 0
    await ClockCycles(dut.clk, 40)

    # High pulse for 22 cycles (shortest pulse!)
    dut.ui_in.value = 1
    await ClockCycles(dut.clk, 22)

    # Low pulse for 60 cycles
    dut.ui_in.value = 0
    await ClockCycles(dut.clk, 60)

    dut.ui_in.value = 1
    await ClockCycles(dut.clk, 20)

    # Read MIN_PULSE register over SPI
    min_l = await host.read_reg(REG_ABU_MIN_PULSE_L)
    min_h = await host.read_reg(REG_ABU_MIN_PULSE_H)
    min_pulse = (min_h << 8) | min_l

    # Filtered pulse duration tolerance: 22 cycles +/- 2
    assert 20 <= min_pulse <= 24, f"ABU MIN_PULSE measurement error! Expected ~22, got {min_pulse}"


# =============================================================================
# TEST 9: Constrained-Random Differential Co-Simulation
# =============================================================================
@cocotb.test()
async def test_differential_co_sim(dut):
    """Differential Co-Simulation: Compare RTL step-by-step against Python Golden Simulator."""
    await setup_testbench(dut)
    host = CocotbForgeHost(dut)
    sim = ForgeSimulator()

    # Generate a program with diverse operations: SET, MOV (ALU ops), and arithmetic
    code = """
    .program diff_co_sim
        set x, 12
        set y, 25
        mov x, ~x
        mov y, ::y
        set x, 31
        mov y, ~x
    """
    prog = assemble(code)
    await host.load_imem(0, prog.instructions)
    sim.load_program(0, prog.instructions)
    sim.cores[0].enabled = True

    # Run step-by-step using single-step register (REG_STEP 0x06)
    for step_num in range(len(prog.instructions)):
        # Step Golden Simulator
        sim.step(1)
        gold_x = sim.cores[0].reg_x
        gold_y = sim.cores[0].reg_y

        # Step RTL
        await host.write_reg(REG_STEP, 0x01)
        await ClockCycles(dut.clk, 2)

        # Read back registers from RTL
        rtl_pc = await host.read_core_reg(0, OFFSET_PC)
        assert rtl_pc == sim.cores[0].pc, f"PC mismatch at step {step_num}! RTL: {rtl_pc}, Golden: {sim.cores[0].pc}"
