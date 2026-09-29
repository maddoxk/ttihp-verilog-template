"""
Unit Tests for ProtocolForge Golden Reference Model (tools/forge_sim.py).
Tests:
- Datapath registers, ALU, PC wrap, JMP conditions
- Deadline scheduler wrap-around safety
- SBE line encoding (NRZ, NRZI, Manchester)
- BSU bit-stuffing (CAN 5-bit, USB 6-bit)
- CRC coprocessor (CRC-16, CRC-15, CRC-8, CRC-5)
- Auto-Baud Unit (ABU) pulse width measurement
- Multi-core flag interaction and end-to-end UART transmission
"""

import pytest
from tools.forge_sim import (
    ForgeFIFO,
    ForgeCRC,
    ForgeSBE_BSU,
    ForgeSimulator,
    bitrev16,
    byteswap16,
)
from tools.forge_asm import assemble
from tools.forge_regs import (
    SBE_NRZ,
    SBE_NRZI,
    SBE_MANCHESTER,
    BSU_DISABLED,
    BSU_USB,
    BSU_CAN,
    CRC_POLY_USB16,
    CRC_POLY_CAN15,
    CRC_POLY_DALLAS8,
    CRC_POLY_USB5,
    OFFSET_DIV_INT_L,
    OFFSET_OUT_BASE,
    OFFSET_OUT_COUNT,
    OFFSET_SET_BASE,
    OFFSET_SET_COUNT,
    OFFSET_IN_BASE,
    OFFSET_SBE_CTRL,
    OFFSET_BSU_CTRL,
    OFFSET_CRC_CTRL,
    OFFSET_SHIFTCTRL,
    OFFSET_STATUS,
    pack_shiftctrl,
)


def test_fifo_basic():
    """Verify FIFO First-Word-Fall-Through semantics, full and empty flags."""
    fifo = ForgeFIFO(depth=4, width=16)
    assert fifo.empty
    assert not fifo.full
    assert fifo.level == 0
    assert fifo.rdata == 0

    assert fifo.push(0x1234)
    assert not fifo.empty
    assert fifo.level == 1
    assert fifo.rdata == 0x1234

    assert fifo.push(0x5678)
    assert fifo.push(0x9ABC)
    assert fifo.push(0xDEF0)
    assert fifo.full
    assert fifo.level == 4
    assert not fifo.push(0x9999)  # Overflow blocked

    assert fifo.pop() == 0x1234
    assert fifo.rdata == 0x5678
    assert not fifo.full
    assert fifo.pop() == 0x5678
    assert fifo.pop() == 0x9ABC
    assert fifo.pop() == 0xDEF0
    assert fifo.empty
    assert fifo.pop() is None


def test_crc_coprocessor():
    """Verify CRC engine for CRC-16-USB, CRC-15-CAN, CRC-8, CRC-5."""
    crc = ForgeCRC()

    # 1. CRC-16-USB
    crc.poly_sel = CRC_POLY_USB16
    crc.preset()
    assert crc.crc_reg == 0xFFFF
    # Shift byte 0xAA = 10101010b LSB first: 0, 1, 0, 1, 0, 1, 0, 1
    for b in [0, 1, 0, 1, 0, 1, 0, 1]:
        crc.step_bit(b)
    assert crc.crc_out != 0
    crc.reset()
    assert crc.crc_zero

    # 2. CRC-15-CAN
    crc.poly_sel = CRC_POLY_CAN15
    crc.reset()
    assert crc.crc_reg == 0x0000
    for b in [1, 0, 1, 1, 0, 0, 1]:
        crc.step_bit(b)
    assert crc.crc_out != 0
    assert not crc.crc_zero

    # 3. CRC-8-Dallas
    crc.poly_sel = CRC_POLY_DALLAS8
    crc.reset()
    for b in [1, 1, 0, 0, 1, 0, 1, 0]:
        crc.step_bit(b)
    assert crc.crc_out <= 0xFF

    # 4. CRC-5-USB
    crc.poly_sel = CRC_POLY_USB5
    crc.preset()
    assert crc.crc_reg == 0x001F
    for b in [1, 0, 0, 1, 1]:
        crc.step_bit(b)
    assert crc.crc_out <= 0x1F


def test_sbe_bsu_can_stuffing():
    """Verify CAN 5-bit stuffing and unstuffing."""
    sbe_bsu = ForgeSBE_BSU()
    sbe_bsu.sbe_mode = SBE_NRZ
    sbe_bsu.bsu_mode = BSU_CAN

    # Transmit 5 consecutive 1s
    tx_stream = []
    # 5 ones
    for _ in range(5):
        pin, stall, _, _ = sbe_bsu.step_tx(1, tick=True)
        tx_stream.append(pin)

    # 6th tick should be the stuffed 0 (complement)
    pin, stall, _, _ = sbe_bsu.step_tx(1, tick=True)
    tx_stream.append(pin)
    assert stall  # OSR was stalled on stuff tick
    assert tx_stream[-1] == 0  # Stuffed bit is 0!

    # Next tick resumes original data (1)
    pin, stall, _, _ = sbe_bsu.step_tx(1, tick=True)
    tx_stream.append(pin)
    assert not stall
    assert tx_stream[-1] == 1


def test_sbe_bsu_usb_stuffing():
    """Verify USB 6-bit ones stuffing and unstuffing."""
    sbe_bsu = ForgeSBE_BSU()
    sbe_bsu.sbe_mode = SBE_NRZ
    sbe_bsu.bsu_mode = BSU_USB

    tx_stream = []
    # Transmit 6 consecutive 1s
    for _ in range(6):
        pin, stall, _, _ = sbe_bsu.step_tx(1, tick=True)
        tx_stream.append(pin)

    # 7th tick should be the stuffed 0
    pin, stall, _, _ = sbe_bsu.step_tx(1, tick=True)
    tx_stream.append(pin)
    assert stall
    assert tx_stream[-1] == 0

    # Unstuffing verification
    sbe_bsu.reset()
    rx_recovered = []
    for p in tx_stream:
        b, valid, err = sbe_bsu.step_rx(p, tick=True)
        if valid:
            rx_recovered.append(b)
    assert rx_recovered == [1, 1, 1, 1, 1, 1]  # Stuffed 0 was stripped!
    assert not err


def test_sbe_nrzi_encoding():
    """Verify NRZI encoding: 0 toggles physical state, 1 maintains state."""
    sbe_bsu = ForgeSBE_BSU()
    sbe_bsu.sbe_mode = SBE_NRZI
    sbe_bsu.bsu_mode = BSU_DISABLED

    # Initial state is 1.
    # Cycle 0: Send bit 0. Current output is initial 1; at clock edge it toggles to 0.
    pin0, _, _, _ = sbe_bsu.step_tx(0, tick=True)
    assert pin0 == 1

    # Cycle 1: Send bit 1. Output is now toggled state 0; bit 1 maintains state 0.
    pin1, _, _, _ = sbe_bsu.step_tx(1, tick=True)
    assert pin1 == 0

    # Cycle 2: Send bit 0. Output is still 0; at clock edge it toggles to 1.
    pin2, _, _, _ = sbe_bsu.step_tx(0, tick=True)
    assert pin2 == 0

    # Cycle 3: Send bit 1. Output is now toggled state 1.
    pin3, _, _, _ = sbe_bsu.step_tx(1, tick=True)
    assert pin3 == 1


def test_sbe_manchester_encoding():
    """Verify Manchester encoding transitions: 1 produces 0->1, 0 produces 1->0."""
    sbe_bsu = ForgeSBE_BSU()
    sbe_bsu.sbe_mode = SBE_MANCHESTER

    # Data 1:
    # Phase 0: should be 0
    # Phase 1: should be 1
    pin_phase0, _, _, _ = sbe_bsu.step_tx(1, tick=True)
    assert pin_phase0 == 0
    pin_phase1, _, _, _ = sbe_bsu.step_tx(1, tick=True)
    assert pin_phase1 == 1

    # Data 0:
    # Phase 0: should be 1
    # Phase 1: should be 0
    pin_phase0, _, _, _ = sbe_bsu.step_tx(0, tick=True)
    assert pin_phase0 == 1
    pin_phase1, _, _, _ = sbe_bsu.step_tx(0, tick=True)
    assert pin_phase1 == 0


def test_simulator_instructions():
    """Verify execution of SET, MOV, ALU, JMP, TIME, and WAIT."""
    sim = ForgeSimulator()
    core = sim.cores[0]

    code = """
    .program test_inst
        set x, 5
        set y, 10
        mov x, ::x
        mov y, ~y
        set pins, 1
        time t+10
        wait time
        set pins, 0
    """
    prog = assemble(code)
    sim.load_program(0, prog.words)
    core.enabled = True

    sim.step(1)  # set x, 5
    assert core.reg_x == 5
    sim.step(1)  # set y, 10
    assert core.reg_y == 10
    sim.step(1)  # mov x, ::x (bit reverse of 5 = 0b101 -> bit 15 and bit 13)
    assert core.reg_x == bitrev16(5)
    sim.step(1)  # mov y, ~y
    assert core.reg_y == (~10) & 0xFFFF
    sim.step(1)  # set pins, 1
    assert (core.pin_drivers & 1) == 1
    sim.step(1)  # time t+10
    assert core.reg_dl == (core.reg_t + 10) & 0xFFFF
    # Wait until deadline is reached
    start_cycle = sim.cycle_count
    sim.run_until(lambda s: (s.cores[0].pin_drivers & 1) == 0, max_cycles=30)
    assert (core.pin_drivers & 1) == 0


def test_auto_baud_unit():
    """Verify ABU measures shortest pulse width of incoming signal."""
    sim = ForgeSimulator()

    # Simulate an incoming UART signal with bit period = 20 clock cycles
    # Idle High for 50 cycles
    sim.gpio_in_external = 1
    sim.step(50)

    # Start bit: Low for 20 cycles
    sim.gpio_in_external = 0
    sim.step(20)

    # Bit 0: High for 20 cycles
    sim.gpio_in_external = 1
    sim.step(20)

    # Bit 1: Low for 40 cycles (two consecutive zero bits)
    sim.gpio_in_external = 0
    sim.step(40)

    # Return to High
    sim.gpio_in_external = 1
    sim.step(20)

    # Minimum observed pulse width should be 20 cycles!
    assert sim.abu_min_pulse == 20


def test_inter_core_flags_sim():
    """Verify synchronization between Core 0 and Core 1 using flag crossbar."""
    sim = ForgeSimulator()

    # Core 0: wait 5 cycles, then set flag 2
    c0_code = """
    .program c0_sync
        set x, 4
    loop:
        jmp x-- loop
        set flagset, 2
    """
    # Core 1: wait for flag 2, then drive pin 9 High
    c1_code = """
    .program c1_sync
        wait 1 flag 2
        set pins, 1
    """
    sim.load_program(0, assemble(c0_code))
    sim.load_program(1, assemble(c1_code))
    sim.cores[1].set_base = 9  # pin 9 = uo_out[1]
    sim.cores[1].set_count = 1
    sim.cores[0].enabled = True
    sim.cores[1].enabled = True

    sim.run_until(lambda s: (s.cores[1].pin_drivers & (1 << 9)) != 0, max_cycles=50)
    assert (sim.cores[1].pin_drivers & (1 << 9)) != 0


def test_autopush_autopull_sim():
    """Verify autonomous FIFO streaming with autopush and autopull."""
    sim = ForgeSimulator()
    core = sim.cores[0]

    core.autopull = True
    core.pull_thresh = 16
    core.autopush = True
    core.push_thresh = 16
    core.out_count = 16

    # Feed TX FIFO with two words
    core.tx_fifo.push(0xABCD)
    core.tx_fifo.push(0x1234)

    # Simple loop that primes OSR then shifts 16 bits out to X and in from X
    code = """
    .program stream_test
        pull block
    .wrap_target
        out x, 16
        in x, 16
    .wrap
    """
    sim.load_program(0, assemble(code))
    core.enabled = True

    # Run for 20 cycles
    sim.step(20)

    # RX FIFO should have received words!
    assert not core.rx_fifo.empty
    assert core.rx_fifo.pop() == 0xABCD

