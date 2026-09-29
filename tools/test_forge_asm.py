"""
Unit and Integration Tests for ProtocolForge Software Toolchain.
Tests Assembler, Disassembler, Register Map, Host Driver, and Firmware Library.
"""

import os
import glob
import pytest

from tools.forge_regs import (
    CHIP_ID, CHIP_VERSION, NUM_CORES,
    REG_ID, REG_ENABLE, REG_ABU_MIN_PULSE_L, REG_ABU_MIN_PULSE_H,
    CORE_BASE, CORE_IMEM_SIZE,
    OFFSET_STATUS, OFFSET_SBE_CTRL, OFFSET_BSU_CTRL, OFFSET_CRC_CTRL,
    SBE_NRZI, SBE_MANCHESTER, BSU_USB, BSU_CAN, CRC_POLY_USB16,
    SHIFTCTRL_IN_SHIFTDIR, SHIFTCTRL_OUT_SHIFTDIR, SHIFTCTRL_AUTOPUSH_EN, SHIFTCTRL_AUTOPULL_EN,
    pack_shiftctrl, unpack_telemetry
)
from tools.forge_asm import (
    assemble, assemble_file, disassemble, disassemble_instruction,
    Program, Assembler, AssemblyError,
    OPCODE_JMP, OPCODE_WAIT, OPCODE_IN, OPCODE_OUT,
    OPCODE_PUSH_PULL, OPCODE_MOV, OPCODE_SET, OPCODE_TIME
)
from tools.forge_host import ForgeHost, MockSpiTransport


# ===========================================================================
# 1. Opcode & Instruction Encoding Tests
# ===========================================================================

def test_jmp_conditions():
    """Test all JMP conditions and target address resolution."""
    tests = [
        ("jmp 12", 0, 0, 12),
        ("jmp !x 5", 0, 1, 5),
        ("jmp x-- 8", 0, 2, 8),
        ("jmp !y 15", 0, 3, 15),
        ("jmp y-- 20", 0, 4, 20),
        ("jmp x!=y 3", 0, 5, 3),
        ("jmp pin 10", 0, 6, 10),
        ("jmp !crc 1", 0, 7, 1),
        ("jmp arb_lost 2", 0, 7, 2),
        ("jmp !osre 4", 0, 7, 4),
    ]
    for asm_code, exp_op, exp_cond, exp_addr in tests:
        prog = assemble(asm_code)
        assert len(prog) == 1
        word = prog[0]
        opcode = (word >> 13) & 0x7
        cond = (word >> 6) & 0x7
        addr = word & 0x3F
        assert opcode == exp_op, f"Opcode mismatch for '{asm_code}'"
        assert cond == exp_cond, f"Condition mismatch for '{asm_code}'"
        assert addr == exp_addr, f"Address mismatch for '{asm_code}'"


def test_wait_sources():
    """Test WAIT instruction with all sources and polarities."""
    tests = [
        ("wait 1 gpio 5", 1, 0b00, 5),
        ("wait 0 pin 3", 0, 0b01, 3),
        ("wait 1 flag 2", 1, 0b10, 2),
        ("wait time", 0, 0b11, 0),
        ("wait edge", 0, 0b11, 1),
    ]
    for asm_code, exp_pol, exp_src, exp_idx in tests:
        prog = assemble(asm_code)
        word = prog[0]
        opcode = (word >> 13) & 0x7
        pol = (word >> 8) & 0x1
        src = (word >> 6) & 0x3
        idx = word & 0x3F
        assert opcode == OPCODE_WAIT
        assert pol == exp_pol
        assert src == exp_src
        assert idx == exp_idx


def test_in_sources():
    """Test IN instruction with all sources and bit counts."""
    sources = ["pins", "x", "y", "null", "t", "status", "crc", "capture"]
    for src_idx, src_name in enumerate(sources):
        prog = assemble(f"in {src_name}, 8")
        word = prog[0]
        assert ((word >> 13) & 0x7) == OPCODE_IN
        assert ((word >> 6) & 0x7) == src_idx
        assert (word & 0x3F) == 8


def test_out_destinations():
    """Test OUT instruction with all destinations and bit counts."""
    dests = ["pins", "x", "y", "null", "pindirs", "pc", "isr", "dl"]
    for dst_idx, dst_name in enumerate(dests):
        prog = assemble(f"out {dst_name}, 16")
        word = prog[0]
        assert ((word >> 13) & 0x7) == OPCODE_OUT
        assert ((word >> 6) & 0x7) == dst_idx
        assert (word & 0x3F) == 16


def test_push_pull():
    """Test PUSH and PULL instructions with all flag permutations."""
    tests = [
        ("push", 0, 0, 1),
        ("push block", 0, 0, 1),
        ("push noblock", 0, 0, 0),
        ("push iffull", 0, 1, 1),
        ("push iffull noblock", 0, 1, 0),
        ("pull", 1, 0, 1),
        ("pull block", 1, 0, 1),
        ("pull noblock", 1, 0, 0),
        ("pull ifempty", 1, 1, 1),
        ("pull ifempty noblock", 1, 1, 0),
    ]
    for asm_code, exp_pull, exp_cond, exp_block in tests:
        prog = assemble(asm_code)
        word = prog[0]
        assert ((word >> 13) & 0x7) == OPCODE_PUSH_PULL
        is_pull = (word >> 8) & 1
        cond = (word >> 7) & 1
        block = (word >> 6) & 1
        assert is_pull == exp_pull, f"Pull flag mismatch for '{asm_code}'"
        assert cond == exp_cond, f"Cond flag mismatch for '{asm_code}'"
        assert block == exp_block, f"Block flag mismatch for '{asm_code}'"


def test_mov_alu_operations():
    """Test MOV ALU operations: none, invert (~), reverse (::), and byte-swap."""
    tests = [
        ("mov x, y", 0b00),
        ("mov x, ~y", 0b01),
        ("mov x, !y", 0b01),
        ("mov x, ::y", 0b10),
        ("mov x, swap(y)", 0b11),
        ("mov x, swap y", 0b11),
    ]
    for asm_code, exp_op in tests:
        prog = assemble(asm_code)
        word = prog[0]
        assert ((word >> 13) & 0x7) == OPCODE_MOV
        op = (word >> 4) & 0x3
        assert op == exp_op, f"ALU op mismatch for '{asm_code}'"


def test_mov_extended_sources():
    """Test MOV with all sources including crc, capture, isr, osr."""
    sources = [
        ("pins", 0), ("x", 1), ("y", 2), ("null", 3),
        ("t", 4), ("status", 5), ("crc", 6), ("capture", 7),
        ("isr", 8), ("osr", 9)
    ]
    for src_name, exp_code in sources:
        prog = assemble(f"mov x, {src_name}")
        word = prog[0]
        assert ((word >> 13) & 0x7) == OPCODE_MOV
        bit3 = (word >> 3) & 1
        src_val = (bit3 << 3) | (word & 7)
        assert src_val == exp_code, f"Source code mismatch for '{src_name}'"


def test_set_destinations():
    """Test SET instruction with all destinations and immediate values."""
    dests = [
        ("pins", 0), ("x", 1), ("y", 2), ("pindirs", 3),
        ("flagset", 4), ("flagclr", 5), ("t", 6), ("crc", 7)
    ]
    for dst_name, exp_dst in dests:
        prog = assemble(f"set {dst_name}, 42")
        word = prog[0]
        assert ((word >> 13) & 0x7) == OPCODE_SET
        dst = (word >> 6) & 0x7
        imm = word & 0x3F
        assert dst == exp_dst
        assert imm == 42


def test_time_modes():
    """Test TIME instruction with t+N, dl+N, t+x, dl+x."""
    tests = [
        ("time t+50", 0b00, 50),
        ("time dl+10", 0b01, 10),
        ("time t+x", 0b10, 0),
        ("time dl+x", 0b11, 0),
    ]
    for asm_code, exp_mode, exp_imm in tests:
        prog = assemble(asm_code)
        word = prog[0]
        assert ((word >> 13) & 0x7) == OPCODE_TIME
        mode = (word >> 7) & 0x3
        imm = word & 0x7F
        assert mode == exp_mode, f"Mode mismatch for '{asm_code}'"
        assert imm == exp_imm, f"Imm mismatch for '{asm_code}'"


def test_nop():
    """Test NOP pseudo-instruction."""
    prog = assemble("nop")
    assert len(prog) == 1
    # Decodes as nop
    dis = disassemble_instruction(prog[0])
    assert dis == "nop"


# ===========================================================================
# 2. Side-Set and Delay Annotation Tests
# ===========================================================================

def test_delay_annotations():
    """Test [N] delay annotations."""
    prog = assemble("""
    set pins, 1 [5]
    nop [15]
    """)
    assert len(prog) == 2
    # Bits [12:9] hold delay
    delay0 = (prog[0] >> 9) & 0xF
    delay1 = (prog[1] >> 9) & 0xF
    assert delay0 == 5
    assert delay1 == 15


def test_side_set_and_delay_combined():
    """Test combined side-set and delay annotations."""
    prog = assemble("""
    .side_set 1
    set pins, 1 side 1 [3]
    nop side 0 [7]
    """)
    assert len(prog) == 2
    assert prog.sideset_count == 1

    # 1 side bit (bit 12), 3 delay bits (bits 11:9)
    delay_side0 = (prog[0] >> 9) & 0xF
    side0 = (delay_side0 >> 3) & 1
    delay0 = delay_side0 & 0x7
    assert side0 == 1
    assert delay0 == 3

    delay_side1 = (prog[1] >> 9) & 0xF
    side1 = (delay_side1 >> 3) & 1
    delay1 = delay_side1 & 0x7
    assert side1 == 0
    assert delay1 == 7


def test_side_set_2_bits():
    """Test 2-bit side-set configuration."""
    prog = assemble("""
    .side_set 2
    nop side 0b10
    nop side 0b01 [2]
    """)
    assert prog.sideset_count == 2
    # Top 2 bits are side, lower 2 bits are delay
    delay_side0 = (prog[0] >> 9) & 0xF
    assert (delay_side0 >> 2) == 0b10
    assert (delay_side0 & 0x3) == 0

    delay_side1 = (prog[1] >> 9) & 0xF
    assert (delay_side1 >> 2) == 0b01
    assert (delay_side1 & 0x3) == 2


# ===========================================================================
# 3. Label Resolution and Directives
# ===========================================================================

def test_labels_and_wrap():
    """Test label resolution and wrap directives."""
    code = """
    .program test_wrap
    .wrap_target
    start:
        set x, 5
    loop:
        jmp x-- loop
        jmp !x end
        nop
    end:
        push block
    .wrap
    """
    prog = assemble(code)
    assert len(prog) == 5
    assert prog.wrap_target == 0
    assert prog.wrap_bottom == 4
    assert prog.labels["start"] == 0
    assert prog.labels["loop"] == 1
    assert prog.labels["end"] == 4

    # Check loop target in jmp x-- loop (PC = 1)
    jmp_loop = prog[1]
    assert (jmp_loop & 0x3F) == 1

    # Check end target in jmp !x end (PC = 4)
    jmp_end = prog[2]
    assert (jmp_end & 0x3F) == 4


def test_define_constants():
    """Test .define directive for constants."""
    code = """
    .define BAUD_DIV 50
    .define PIN_TX 1
    set pins, PIN_TX
    time t+BAUD_DIV
    """
    prog = assemble(code)
    assert len(prog) == 2
    assert (prog[0] & 0x3F) == 1
    assert (prog[1] & 0x7F) == 50


# ===========================================================================
# 4. Disassembler Round-Trip Tests
# ===========================================================================

def test_disassembler_round_trip():
    """
    Verify that assembling, disassembling, and re-assembling
    preserves exact machine code bitwise.
    """
    instructions = [
        "jmp 15",
        "jmp !x 10",
        "jmp x-- 5",
        "jmp !y 3",
        "jmp y-- 7",
        "jmp x!=y 12",
        "jmp pin 8",
        "jmp !crc 2",
        "wait time",
        "wait edge",
        "wait 1 gpio 4",
        "wait 0 pin 2",
        "wait 1 flag 3",
        "in pins, 8",
        "in capture, 16",
        "in crc, 15",
        "out pins, 1",
        "out pindirs, 4",
        "out dl, 16",
        "push block",
        "push noblock",
        "push iffull block",
        "pull block",
        "pull noblock",
        "pull ifempty block",
        "mov x, y",
        "mov x, ~pins",
        "mov x, ::y",
        "mov x, swap y",
        "mov osr, crc",
        "mov x, isr",
        "set pins, 1",
        "set x, 31",
        "set crc, 0",
        "set flagset, 2",
        "time t+50",
        "time dl+10",
        "time t+x",
        "time dl+x",
        "nop",
    ]

    for orig_asm in instructions:
        prog1 = assemble(orig_asm)
        word1 = prog1[0]
        disasm_text = disassemble_instruction(word1)
        prog2 = assemble(disasm_text)
        word2 = prog2[0]
        assert word1 == word2, f"Round-trip failed for '{orig_asm}':\nDisassembled as: '{disasm_text}'\nWord1: 0x{word1:04x}, Word2: 0x{word2:04x}"


# ===========================================================================
# 5. Error Handling Tests
# ===========================================================================

def test_assembly_errors():
    """Verify informative errors are raised on invalid syntax."""
    bad_snippets = [
        ("foo_bar pins, 1", "Unknown instruction mnemonic"),
        ("jmp invalid_cond 5", "Invalid jmp condition"),
        ("in invalid_src, 8", "Invalid in source"),
        ("out invalid_dst, 8", "Invalid out destination"),
        ("set pins, 100", "out of range"),
        ("time t+200", "out of range"),
        ("set pins, 1 [20]", "Delay value 20 exceeds maximum"),
        ("start:\nstart:\nnop", "Duplicate label"),
    ]
    for code, err_substring in bad_snippets:
        with pytest.raises(AssemblyError) as exc_info:
            assemble(code)
        assert err_substring in str(exc_info.value)


# ===========================================================================
# 6. Firmware Library Assembly Tests (All 12 Protocols)
# ===========================================================================

PROGRAM_FILES = [
    ("programs/uart_tx.asm", 48),
    ("programs/uart_rx.asm", 48),
    ("programs/spi_master.asm", 48),
    ("programs/spi_slave.asm", 48),
    ("programs/i2c_master.asm", 48),
    ("programs/i2c_slave.asm", 48),
    ("programs/can_tx.asm", 48),
    ("programs/usb_ls_tx.asm", 48),
    ("programs/manchester_tx.asm", 48),
    ("programs/onewire.asm", 48),
    ("programs/ws2812.asm", 48),
    ("programs/sniffer.asm", 32),
]

@pytest.mark.parametrize("filepath,max_size", PROGRAM_FILES)
def test_firmware_programs(filepath, max_size):
    """Verify all 12 firmware library programs assemble cleanly within silicon budget."""
    assert os.path.exists(filepath), f"Firmware file {filepath} not found"
    prog = assemble_file(filepath)

    assert len(prog) > 0, f"{filepath} assembled to empty program"
    assert len(prog) <= max_size, f"{filepath} ({len(prog)} words) exceeds IMEM limit of {max_size}"

    # Verify every machine code word is a valid 16-bit unsigned integer
    for i, w in enumerate(prog.instructions):
        assert 0 <= w <= 0xFFFF, f"Word {i} in {filepath} invalid: {hex(w)}"

    # Test C-array and hex generation
    c_arr = prog.to_c_array()
    assert "static const uint16_t" in c_arr
    hex_list = prog.to_hex()
    assert len(hex_list) == len(prog)


# ===========================================================================
# 7. Register Map & Host Driver Integration Tests
# ===========================================================================

def test_register_map_integrity():
    """Verify register addresses and bitfield helpers."""
    assert REG_ID == 0x00
    assert CHIP_ID == 0xC7
    assert CHIP_VERSION == 0x01
    assert NUM_CORES == 3

    assert CORE_BASE[0] == 0x20
    assert CORE_BASE[1] == 0x40
    assert CORE_BASE[2] == 0x60

    assert CORE_IMEM_SIZE[0] == 48
    assert CORE_IMEM_SIZE[1] == 48
    assert CORE_IMEM_SIZE[2] == 32

    # SHIFTCTRL packing
    ctrl = pack_shiftctrl(autopush=True, autopull=True, in_right=True, out_right=False, push_thresh=8, pull_thresh=16)
    assert ctrl & SHIFTCTRL_IN_SHIFTDIR != 0   # bit 0: in_right
    assert ctrl & SHIFTCTRL_OUT_SHIFTDIR == 0  # bit 1: out_right
    assert ctrl & SHIFTCTRL_AUTOPUSH_EN != 0   # bit 2: autopush
    assert ctrl & SHIFTCTRL_AUTOPULL_EN != 0   # bit 3: autopull


def test_host_driver_mock_transport():
    """Test ForgeHost operations with MockSpiTransport."""
    host = ForgeHost()

    # Read ID and version
    chip_id = host.read_reg(REG_ID)
    assert chip_id == CHIP_ID

    # Enable cores
    host.enable_cores(0x05)  # Enable Core 0 & Core 2
    assert host.read_reg(REG_ENABLE) == 0x05

    # Configure clock divider on Core 0
    host.configure_clock(core_id=0, div_int=50, div_frac=128)
    assert host.read_core_reg(0, 0x00) == 50   # DIV_INT_L
    assert host.read_core_reg(0, 0x02) == 128  # DIV_FRAC

    # Configure accelerators
    host.configure_accelerators(core_id=0, sbe_mode=SBE_NRZI, bsu_mode=BSU_USB, crc_mode=CRC_POLY_USB16)
    assert host.read_core_reg(0, OFFSET_SBE_CTRL) == SBE_NRZI
    assert host.read_core_reg(0, OFFSET_BSU_CTRL) == BSU_USB
    assert host.read_core_reg(0, OFFSET_CRC_CTRL) == CRC_POLY_USB16

    # Load instruction memory
    code_words = [0xC001, 0xA050, 0x20C0]
    host.load_imem(core_id=0, code_words=code_words)
    mock = host.transport
    assert isinstance(mock, MockSpiTransport)
    assert mock.imem[0][:3] == code_words

    # Push to TX FIFO
    tx_data = [0x1234, 0x5678, 0x9ABC]
    host.push_tx(core_id=0, words=tx_data)
    assert list(mock.tx_fifo[0]) == tx_data

    # Pop from RX FIFO
    rx_test_data = [0xFEED, 0xCAFE]
    mock.rx_fifo[0].extend(rx_test_data)
    popped = host.pop_rx(core_id=0, count=2)
    assert popped == rx_test_data

    # Auto-Baud Unit readback
    min_pulse = host.read_auto_baud()
    assert min_pulse == 50
    baud_hz = host.get_auto_baud_hz(clock_hz=50_000_000)
    assert baud_hz == 1_000_000.0  # 50 MHz / 50 = 1 MHz

    # Read status
    status = host.get_status_dict(core_id=0)
    assert status["tx_empty"] is True

    # Telemetry check
    telemetry = host.get_last_telemetry()
    assert telemetry["magic_valid"] is True
