"""
ProtocolForge Cycle-Accurate Python Golden Reference Simulator.
Architecture Specification: ARCH_PROPOSAL_CHIEF.md
Protocol Requirements: PROTOCOL_REQUIREMENTS.md

Provides cycle-by-cycle and instruction-by-instruction emulation of:
- State machine datapath: PC, X, Y, ISR, OSR, T, DL, CAPTURE, CRC accumulator
- Hardware accelerators:
    * SBE (Stream Bit Engine): NRZ, NRZI, Manchester, Inverted NRZI
    * BSU (Bit-Stuffing Unit): CAN 5-bit, USB 6-bit (TX stuff and RX unstuff)
    * CRC LFSR Engine: CRC-16, CRC-15, CRC-8, CRC-5
    * ABU (Auto-Baud & Edge Capture Unit)
- Synchronous FIFOs (FWFT)
- Multi-core interaction with 20-bit GPIO bus and 8-bit inter-core flags
"""

from __future__ import annotations
from typing import List, Dict, Optional, Tuple, Any
from dataclasses import dataclass, field

from tools.forge_regs import (
    NUM_CORES,
    CHIP_ID,
    CHIP_VERSION,
    OFFSET_DIV_INT_L,
    OFFSET_DIV_INT_H,
    OFFSET_DIV_FRAC,
    OFFSET_OUT_BASE,
    OFFSET_OUT_COUNT,
    OFFSET_SET_BASE,
    OFFSET_SET_COUNT,
    OFFSET_IN_BASE,
    OFFSET_SIDESET,
    OFFSET_JMP_PIN,
    OFFSET_WRAP_TOP,
    OFFSET_WRAP_BOT,
    OFFSET_SHIFTCTRL,
    OFFSET_PC,
    OFFSET_STATUS,
    OFFSET_SBE_CTRL,
    OFFSET_BSU_CTRL,
    OFFSET_CRC_CTRL,
    OFFSET_CRC_VAL_L,
    OFFSET_CRC_VAL_H,
    OFFSET_CAPTURE_L,
    OFFSET_CAPTURE_H,
    OFFSET_TXF_STREAM_L,
    OFFSET_TXF_STREAM_H,
    OFFSET_RXF_STREAM_L,
    OFFSET_RXF_STREAM_H,
    SBE_NRZ,
    SBE_NRZI,
    SBE_MANCHESTER,
    SBE_DIFF_NRZI,
    BSU_DISABLED,
    BSU_USB,
    BSU_CAN,
    CRC_POLY_USB16,
    CRC_POLY_CAN15,
    CRC_POLY_DALLAS8,
    CRC_POLY_USB5,
)


def bitrev16(val: int) -> int:
    """Reverse the 16 bits of val."""
    res = 0
    for i in range(16):
        if (val >> i) & 1:
            res |= 1 << (15 - i)
    return res


def byteswap16(val: int) -> int:
    """Swap high and low bytes of 16-bit val."""
    return ((val & 0xFF) << 8) | ((val >> 8) & 0xFF)


class ForgeFIFO:
    """First-Word-Fall-Through (FWFT) Synchronous FIFO model."""

    def __init__(self, depth: int = 8, width: int = 16):
        self.depth = depth
        self.width = width
        self.mask = (1 << width) - 1
        self.mem: List[int] = []

    def push(self, data: int) -> bool:
        if len(self.mem) < self.depth:
            self.mem.append(data & self.mask)
            return True
        return False

    def pop(self) -> Optional[int]:
        if self.mem:
            return self.mem.pop(0)
        return None

    @property
    def rdata(self) -> int:
        return self.mem[0] if self.mem else 0

    @property
    def full(self) -> bool:
        return len(self.mem) >= self.depth

    @property
    def empty(self) -> bool:
        return len(self.mem) == 0

    @property
    def level(self) -> int:
        return len(self.mem)

    def clear(self):
        self.mem.clear()


class ForgeCRC:
    """Hardware Multi-Polynomial Streaming CRC LFSR Coprocessor."""

    def __init__(self):
        self.poly_sel: int = CRC_POLY_USB16
        self.invert_out: bool = False
        self.crc_reg: int = 0x0000

    def reset(self):
        self.crc_reg = 0x0000

    def preset(self):
        if self.poly_sel == CRC_POLY_USB16:
            self.crc_reg = 0xFFFF
        elif self.poly_sel == CRC_POLY_USB5:
            self.crc_reg = 0x001F
        else:
            self.crc_reg = 0x0000

    def step_bit(self, bit: int):
        bit = bit & 1
        if self.poly_sel == CRC_POLY_USB16:
            fb = bit ^ ((self.crc_reg >> 15) & 1)
            self.crc_reg = ((self.crc_reg << 1) & 0xFFFF) ^ (0x8005 if fb else 0)
        elif self.poly_sel == CRC_POLY_CAN15:
            fb = bit ^ ((self.crc_reg >> 14) & 1)
            self.crc_reg = ((self.crc_reg << 1) & 0x7FFE) ^ (0x4599 if fb else 0)
        elif self.poly_sel == CRC_POLY_DALLAS8:
            fb = bit ^ ((self.crc_reg >> 7) & 1)
            self.crc_reg = ((self.crc_reg << 1) & 0x00FE) ^ (0x0031 if fb else 0)
        elif self.poly_sel == CRC_POLY_USB5:
            fb = bit ^ ((self.crc_reg >> 4) & 1)
            self.crc_reg = ((self.crc_reg << 1) & 0x001E) ^ (0x0005 if fb else 0)

    @property
    def crc_out(self) -> int:
        if self.invert_out:
            return (~self.crc_reg) & 0xFFFF
        return self.crc_reg & 0xFFFF

    @property
    def crc_zero(self) -> bool:
        return self.crc_reg == 0


class ForgeSBE_BSU:
    """Stream Bit Engine (SBE) and Bit-Stuffing Unit (BSU) model."""

    def __init__(self):
        self.sbe_mode: int = SBE_NRZ
        self.bsu_mode: int = BSU_DISABLED

        # TX State
        self.tx_stuff_cnt: int = 0
        self.tx_prev_bit: int = 1
        self.tx_stuff_active: bool = False
        self.tx_stuffed_val: int = 0
        self.tx_stall_osr: bool = False
        self.tx_nrzi_state: int = 1
        self.tx_man_phase: int = 0

        # RX State
        self.rx_prev_pin: int = 1
        self.rx_stuff_cnt: int = 0
        self.rx_prev_bit: int = 1
        self.rx_suppress: bool = False
        self.rx_stuff_err: bool = False
        self.rx_bit_out: int = 0
        self.rx_valid: bool = False

    def reset(self):
        self.tx_stuff_cnt = 0
        self.tx_prev_bit = 1
        self.tx_stuff_active = False
        self.tx_stuffed_val = 0
        self.tx_stall_osr = False
        self.tx_nrzi_state = 1
        self.tx_man_phase = 0

        self.rx_prev_pin = 1
        self.rx_stuff_cnt = 0
        self.rx_prev_bit = 1
        self.rx_suppress = False
        self.rx_stuff_err = False
        self.rx_bit_out = 0
        self.rx_valid = False

    def step_tx(self, tx_bit_in: int, tick: bool) -> Tuple[int, bool, int, bool]:
        """
        Advance TX path by one clock cycle with synchronous clock-edge semantics.
        Returns: (tx_pin_out, tx_stall_osr, tx_bit_crc, tx_crc_en)
        """
        tx_bit_in = tx_bit_in & 1

        # Current state drives outputs during this cycle
        tx_sbe_in = self.tx_stuffed_val if self.tx_stuff_active else tx_bit_in
        tx_man_out = tx_sbe_in if self.tx_man_phase else (1 - tx_sbe_in)

        if self.sbe_mode == SBE_NRZ:
            curr_pin_out = tx_sbe_in
        elif self.sbe_mode == SBE_NRZI:
            curr_pin_out = self.tx_nrzi_state
        elif self.sbe_mode == SBE_MANCHESTER:
            curr_pin_out = tx_man_out
        elif self.sbe_mode == SBE_DIFF_NRZI:
            curr_pin_out = 1 - self.tx_nrzi_state
        else:
            curr_pin_out = tx_sbe_in

        curr_stall = self.tx_stall_osr
        tx_bit_crc = tx_bit_in
        tx_crc_en = tick and not self.tx_stuff_active

        # Clock-edge updates at posedge clk when tick is high
        if tick:
            if self.tx_stuff_active:
                self.tx_stuff_active = False
                self.tx_stall_osr = False
                if self.bsu_mode == BSU_CAN:
                    self.tx_stuff_cnt = 1
                    self.tx_prev_bit = self.tx_stuffed_val
                else:
                    self.tx_stuff_cnt = 0
            else:
                if self.bsu_mode == BSU_USB:
                    if tx_bit_in == 1:
                        if self.tx_stuff_cnt == 5:
                            self.tx_stuff_active = True
                            self.tx_stuffed_val = 0
                            self.tx_stall_osr = True
                            self.tx_stuff_cnt = 0
                        else:
                            self.tx_stuff_cnt += 1
                    else:
                        self.tx_stuff_cnt = 0
                elif self.bsu_mode == BSU_CAN:
                    if tx_bit_in == self.tx_prev_bit:
                        if self.tx_stuff_cnt == 4:
                            self.tx_stuff_active = True
                            self.tx_stuffed_val = 1 - self.tx_prev_bit
                            self.tx_stall_osr = True
                            self.tx_stuff_cnt = 0
                        else:
                            self.tx_stuff_cnt += 1
                    else:
                        self.tx_stuff_cnt = 1
                        self.tx_prev_bit = tx_bit_in
                else:
                    self.tx_stuff_active = False
                    self.tx_stall_osr = False
                    self.tx_stuff_cnt = 0

            # SBE register updates at clock edge
            sbe_data = self.tx_stuffed_val if self.tx_stuff_active else tx_bit_in
            if sbe_data == 0:
                self.tx_nrzi_state = 1 - self.tx_nrzi_state
            self.tx_man_phase = 1 - self.tx_man_phase

        return curr_pin_out, curr_stall, tx_bit_crc, tx_crc_en

    def step_rx(self, rx_pin_in: int, tick: bool) -> Tuple[int, bool, bool]:
        """
        Advance RX path by one clock cycle.
        Returns: (rx_bit_out, rx_valid, rx_stuff_err)
        """
        rx_pin_in = rx_pin_in & 1
        rx_edge_trans = (rx_pin_in != self.rx_prev_pin)
        if self.sbe_mode in (SBE_NRZI, SBE_DIFF_NRZI):
            rx_demod_bit = 0 if rx_edge_trans else 1
        else:
            rx_demod_bit = rx_pin_in

        if tick:
            self.rx_prev_pin = rx_pin_in

            if self.bsu_mode == BSU_USB:
                if self.rx_stuff_cnt == 6:
                    if rx_demod_bit == 0:
                        self.rx_suppress = True
                        self.rx_stuff_cnt = 0
                    else:
                        self.rx_stuff_err = True
                        self.rx_suppress = True
                        self.rx_stuff_cnt = 0
                else:
                    self.rx_suppress = False
                    if rx_demod_bit == 1:
                        self.rx_stuff_cnt += 1
                    else:
                        self.rx_stuff_cnt = 0

            elif self.bsu_mode == BSU_CAN:
                if self.rx_stuff_cnt == 5:
                    if rx_demod_bit == (1 - self.rx_prev_bit):
                        self.rx_suppress = True
                        self.rx_stuff_cnt = 1
                        self.rx_prev_bit = rx_demod_bit
                    else:
                        self.rx_stuff_err = True
                        self.rx_suppress = True
                        self.rx_stuff_cnt = 0
                else:
                    self.rx_suppress = False
                    if rx_demod_bit == self.rx_prev_bit:
                        self.rx_stuff_cnt += 1
                    else:
                        self.rx_stuff_cnt = 1
                        self.rx_prev_bit = rx_demod_bit
            else:
                self.rx_suppress = False
                self.rx_stuff_cnt = 0

        self.rx_bit_out = rx_demod_bit
        self.rx_valid = tick and not self.rx_suppress
        return self.rx_bit_out, self.rx_valid, self.rx_stuff_err


class ForgeCore:
    """Cycle-accurate model of a single ProtocolForge state machine core."""

    def __init__(self, core_id: int, imem_words: int = 48, fifo_depth: int = 8):
        self.core_id = core_id
        self.imem_words = imem_words
        self.imem: List[int] = [0] * imem_words
        self.tx_fifo = ForgeFIFO(depth=fifo_depth, width=16)
        self.rx_fifo = ForgeFIFO(depth=fifo_depth, width=16)
        self.sbe_bsu = ForgeSBE_BSU()
        self.crc = ForgeCRC()

        # Architecture Registers
        self.pc: int = 0
        self.reg_x: int = 0
        self.reg_y: int = 0
        self.reg_t: int = 0
        self.reg_dl: int = 0
        self.reg_capture: int = 0
        self.reg_isr: int = 0
        self.isr_count: int = 0
        self.reg_osr: int = 0
        self.osr_count: int = 0

        # Pin I/O drivers & direction
        self.pindirs: int = 0
        self.pin_drivers: int = 0

        # Configuration Registers
        self.div_int: int = 1
        self.div_frac: int = 0
        self.div_cnt: int = 0
        self.frac_acc: int = 0
        self.div_tick: bool = False

        self.out_base: int = 0
        self.out_count: int = 1
        self.set_base: int = 0
        self.set_count: int = 1
        self.in_base: int = 0
        self.sideset_base: int = 0
        self.sideset_count: int = 0
        self.jmp_pin: int = 0
        self.wrap_top: int = imem_words - 1
        self.wrap_bot: int = 0

        self.in_shift_dir: int = 0   # 0=left, 1=right
        self.out_shift_dir: int = 0  # 0=left, 1=right
        self.autopush: bool = False
        self.autopull: bool = False
        self.push_thresh: int = 16
        self.pull_thresh: int = 16

        self.scl_stretch_en: bool = False
        self.arb_detect_en: bool = False
        self.jmp_cond_sel: int = 0   # 00=!crc, 01=arb_lost, 10=!osre, 11=stuff_err
        self.sniffer_en: bool = False
        self.scl_pin_idx: int = 16

        self.crc_snoop_rx: bool = False
        self.crc_enable: bool = False
        self.crc_invert: bool = False

        # Status flags
        self.enabled: bool = False
        self.arb_lost: bool = False
        self.delay_cnt: int = 0
        self.wait_prev_edge_pin: int = 0

        # Inter-core flag strobes
        self.flag_set_strobe: int = 0
        self.flag_clr_strobe: int = 0

    def load_program(self, prog: Any, origin: int = 0):
        if hasattr(prog, "instructions"):
            words = prog.instructions
        elif hasattr(prog, "words"):
            words = prog.words
        else:
            words = list(prog)
        for i, w in enumerate(words):
            if origin + i < self.imem_words:
                self.imem[origin + i] = int(w) & 0xFFFF

    def reset(self):
        self.pc = 0
        self.reg_x = 0
        self.reg_y = 0
        self.reg_dl = 0
        self.reg_capture = 0
        self.reg_isr = 0
        self.isr_count = 0
        self.reg_osr = 0
        self.osr_count = 0
        self.pindirs = 0
        self.pin_drivers = 0
        self.delay_cnt = 0
        self.arb_lost = False
        self.div_cnt = 0
        self.frac_acc = 0
        self.div_tick = False
        self.tx_fifo.clear()
        self.rx_fifo.clear()
        self.sbe_bsu.reset()
        self.crc.reset()

    @property
    def status_byte(self) -> int:
        is_stalled = int(self.is_stalled)
        b = 0
        if self.tx_fifo.empty: b |= (1 << 0)
        if self.tx_fifo.full:  b |= (1 << 1)
        if self.rx_fifo.empty: b |= (1 << 2)
        if self.rx_fifo.full:  b |= (1 << 3)
        if is_stalled:         b |= (1 << 4)
        if self.sbe_bsu.rx_stuff_err: b |= (1 << 5)
        if self.arb_lost:      b |= (1 << 6)
        if self.crc.crc_zero:  b |= (1 << 7)
        return b

    @property
    def dl_reached(self) -> bool:
        diff = (self.reg_t - self.reg_dl) & 0xFFFF
        return (diff & 0x8000) == 0

    @property
    def is_stalled(self) -> bool:
        instr = self.imem[self.pc]
        op = (instr >> 13) & 0x7
        operand = instr & 0x1FF

        # WAIT stall
        wait_stall = False
        if op == 1:
            wait_stall = not self._eval_wait_cond(operand)

        # FIFO stall
        fifo_stall = False
        if op == 4:
            is_pull = (operand >> 8) & 1
            block = (operand >> 6) & 1
            if is_pull == 0 and block and self.rx_fifo.full:
                fifo_stall = True
            elif is_pull == 1 and block and self.tx_fifo.empty:
                fifo_stall = True

        if self.autopush and self.isr_count >= self.push_thresh and self.rx_fifo.full:
            fifo_stall = True
        if self.autopull and self.osr_count >= self.pull_thresh and self.tx_fifo.empty:
            fifo_stall = True

        return (self.delay_cnt > 0) or wait_stall or fifo_stall or self.sbe_bsu.tx_stall_osr

    def _eval_wait_cond(self, operand: int) -> bool:
        pol = (operand >> 8) & 1
        src = (operand >> 6) & 3
        idx = operand & 0x1F
        if src == 0:  # gpio
            return ((self.gpio_in >> (idx % 20)) & 1) == pol
        elif src == 1:  # pin
            return ((self.gpio_in >> ((self.in_base + idx) % 20)) & 1) == pol
        elif src == 2:  # flag
            return ((self.flags_in >> (operand & 7)) & 1) == pol
        elif src == 3:  # time / edge
            if (operand & 0x3F) == 0:
                return self.dl_reached
            else:
                pin_val = (self.gpio_in >> ((self.in_base + idx) % 20)) & 1
                return pin_val != self.wait_prev_edge_pin
        return True

    def clock_step(self, gpio_in: int, flags_in: int, single_step: bool = False) -> Dict[str, Any]:
        """
        Advance core state by exactly one 20ns clock cycle.
        """
        self.gpio_in = gpio_in
        self.flags_in = flags_in
        self.flag_set_strobe = 0
        self.flag_clr_strobe = 0

        # Increment free-running timebase
        self.reg_t = (self.reg_t + 1) & 0xFFFF

        # Edge timestamp capture
        current_in_pin = (gpio_in >> (self.in_base % 20)) & 1
        if current_in_pin != self.wait_prev_edge_pin:
            self.reg_capture = self.reg_t
        self.wait_prev_edge_pin = current_in_pin

        # Fractional divider update
        if self.div_int <= 1 and self.div_frac == 0:
            self.div_tick = True
        elif self.div_cnt == 0:
            self.div_tick = True
            add_cycle = 1 if (self.frac_acc + self.div_frac) >= 0x100 else 0
            base_cnt = (self.div_int - 1) if self.div_int > 0 else 0
            self.div_cnt = base_cnt + add_cycle
            self.frac_acc = (self.frac_acc + self.div_frac) & 0xFF
        else:
            self.div_tick = False
            self.div_cnt -= 1

        # SBE / BSU Serial Processing
        tx_bit_raw = (self.reg_osr & 1) if self.out_shift_dir else ((self.reg_osr >> 15) & 1)
        tx_pin_out, tx_stall_osr, tx_bit_crc, tx_crc_en = self.sbe_bsu.step_tx(tx_bit_raw, self.div_tick)
        rx_bit_out, rx_valid, rx_stuff_err = self.sbe_bsu.step_rx(current_in_pin, self.div_tick)

        # CRC snoop update
        if self.crc_enable:
            if self.crc_snoop_rx:
                if rx_valid:
                    self.crc.step_bit(rx_bit_out)
            else:
                if tx_crc_en:
                    self.crc.step_bit(tx_bit_crc)

        # Autopull
        if self.autopull and (self.osr_count >= self.pull_thresh) and not self.tx_fifo.empty:
            val = self.tx_fifo.pop()
            if val is not None:
                self.reg_osr = val
                self.osr_count = 0

        # Autopush
        if self.autopush and (self.isr_count >= self.push_thresh) and not self.rx_fifo.full:
            self.rx_fifo.push(self.reg_isr)
            self.reg_isr = 0
            self.isr_count = 0

        # Decrement delay
        if self.enabled and self.div_tick and self.delay_cnt > 0:
            self.delay_cnt -= 1

        # Execute instruction if triggered
        exec_step = (self.enabled and self.div_tick and not self.is_stalled) or single_step

        if exec_step:
            self._execute_current_instruction()

        # Arbitration loss detection
        if self.arb_detect_en:
            for k in range(20):
                if ((self.pindirs >> k) & 1) and ((self.pin_drivers >> k) & 1) and not ((gpio_in >> k) & 1):
                    self.arb_lost = True

        return {
            "pc": self.pc,
            "x": self.reg_x,
            "y": self.reg_y,
            "t": self.reg_t,
            "dl": self.reg_dl,
            "isr": self.reg_isr,
            "osr": self.reg_osr,
            "status": self.status_byte,
            "pin_out": self.active_pin_out,
            "pin_oe": self.pindirs,
        }

    @property
    def active_pin_out(self) -> int:
        if self.sbe_bsu.sbe_mode != SBE_NRZ:
            pin_idx = self.out_base % 20
            tx_pin = self.sbe_bsu.step_tx((self.reg_osr & 1) if self.out_shift_dir else ((self.reg_osr >> 15) & 1), False)[0]
            cleared = self.pin_drivers & ~(1 << pin_idx)
            return cleared | (tx_pin << pin_idx)
        return self.pin_drivers

    def _execute_current_instruction(self):
        instr = self.imem[self.pc]
        op = (instr >> 13) & 0x7
        delay_side = (instr >> 9) & 0xF
        operand = instr & 0x1FF

        # Apply sideset
        if self.sideset_count > 0:
            sideset_val = delay_side >> (4 - self.sideset_count)
            for k in range(self.sideset_count):
                bit = (sideset_val >> k) & 1
                pin_idx = (self.sideset_base + k) % 20
                if bit:
                    self.pin_drivers |= (1 << pin_idx)
                else:
                    self.pin_drivers &= ~(1 << pin_idx)

        # Set delay count
        eff_delay = delay_side & ((1 << (4 - self.sideset_count)) - 1)
        if eff_delay > 0:
            self.delay_cnt = eff_delay

        next_pc = self.wrap_bot if self.pc == self.wrap_top else (self.pc + 1)

        if op == 0:  # JMP
            cond = (operand >> 6) & 7
            target = operand & 0x3F
            cond_met = False
            if cond == 0:
                cond_met = True
            elif cond == 1:
                cond_met = (self.reg_x == 0)
            elif cond == 2:
                cond_met = (self.reg_x != 0)
                if self.reg_x != 0:
                    self.reg_x = (self.reg_x - 1) & 0xFFFF
            elif cond == 3:
                cond_met = (self.reg_y == 0)
            elif cond == 4:
                cond_met = (self.reg_y != 0)
                if self.reg_y != 0:
                    self.reg_y = (self.reg_y - 1) & 0xFFFF
            elif cond == 5:
                cond_met = (self.reg_x != self.reg_y)
            elif cond == 6:
                cond_met = bool((self.gpio_in >> (self.jmp_pin % 20)) & 1)
            elif cond == 7:
                if self.jmp_cond_sel == 0:
                    cond_met = self.crc.crc_zero
                elif self.jmp_cond_sel == 1:
                    cond_met = self.arb_lost
                elif self.jmp_cond_sel == 2:
                    cond_met = (self.osr_count < self.pull_thresh and not self.tx_fifo.empty)
                elif self.jmp_cond_sel == 3:
                    cond_met = self.sbe_bsu.rx_stuff_err

            self.pc = target if cond_met else next_pc

        elif op == 1:  # WAIT
            src = (operand >> 6) & 3
            pol = (operand >> 8) & 1
            if src == 2 and pol == 1:
                self.flag_clr_strobe |= (1 << (operand & 7))
            self.pc = next_pc

        elif op == 2:  # IN
            src_sel = (operand >> 6) & 7
            cnt = operand & 0x1F
            if cnt == 0: cnt = 16

            src_val = 0
            if src_sel == 0:
                src_val = (self.gpio_in >> (self.in_base % 20)) & ((1 << cnt) - 1)
            elif src_sel == 1:
                src_val = self.reg_x
            elif src_sel == 2:
                src_val = self.reg_y
            elif src_sel == 3:
                src_val = 0
            elif src_sel == 4:
                src_val = self.reg_t
            elif src_sel == 5:
                src_val = self.status_byte
            elif src_sel == 6:
                src_val = self.crc.crc_out
            elif src_sel == 7:
                src_val = self.reg_capture

            data_bits = src_val & ((1 << cnt) - 1)
            if self.in_shift_dir == 0:  # Left
                self.reg_isr = ((self.reg_isr << cnt) | data_bits) & 0xFFFF
            else:  # Right
                self.reg_isr = ((self.reg_isr >> cnt) | (data_bits << (16 - cnt))) & 0xFFFF
            self.isr_count += cnt
            self.pc = next_pc

        elif op == 3:  # OUT
            dst_sel = (operand >> 6) & 7
            cnt = operand & 0x1F
            if cnt == 0: cnt = 16

            if self.out_shift_dir == 0:  # Left
                shifted = (self.reg_osr >> (16 - cnt)) & ((1 << cnt) - 1)
                self.reg_osr = (self.reg_osr << cnt) & 0xFFFF
            else:  # Right
                shifted = self.reg_osr & ((1 << cnt) - 1)
                self.reg_osr = (self.reg_osr >> cnt) & 0xFFFF
            self.osr_count += cnt

            if dst_sel == 0:  # pins
                for k in range(min(cnt, self.out_count)):
                    bit = (shifted >> k) & 1
                    pin_idx = (self.out_base + k) % 20
                    if bit:
                        self.pin_drivers |= (1 << pin_idx)
                    else:
                        self.pin_drivers &= ~(1 << pin_idx)
            elif dst_sel == 1:
                self.reg_x = shifted
            elif dst_sel == 2:
                self.reg_y = shifted
            elif dst_sel == 4:  # pindirs
                for k in range(min(cnt, self.out_count)):
                    bit = (shifted >> k) & 1
                    pin_idx = (self.out_base + k) % 20
                    if bit:
                        self.pindirs |= (1 << pin_idx)
                    else:
                        self.pindirs &= ~(1 << pin_idx)
            elif dst_sel == 5:  # pc
                self.pc = shifted & 0x3F
                return
            elif dst_sel == 6:  # isr
                self.reg_isr = shifted
                self.isr_count = cnt
            elif dst_sel == 7:  # dl
                self.reg_dl = shifted

            self.pc = next_pc

        elif op == 4:  # PUSH / PULL
            is_pull = (operand >> 8) & 1
            cond_flag = (operand >> 7) & 1
            block = (operand >> 6) & 1

            if is_pull == 0:  # PUSH
                if not cond_flag or (self.isr_count >= self.push_thresh):
                    if not self.rx_fifo.full:
                        self.rx_fifo.push(self.reg_isr)
                        self.reg_isr = 0
                        self.isr_count = 0
            else:  # PULL
                if not cond_flag or (self.osr_count >= self.pull_thresh):
                    if not self.tx_fifo.empty:
                        val = self.tx_fifo.pop()
                        if val is not None:
                            self.reg_osr = val
                            self.osr_count = 0
                    elif not block:
                        self.reg_osr = self.reg_x
                        self.osr_count = 0
            self.pc = next_pc

        elif op == 5:  # MOV
            dst_sel = (operand >> 6) & 7
            alu_op = (operand >> 4) & 3
            src_sel = operand & 7

            src_val = 0
            if src_sel == 0:
                src_val = (self.gpio_in >> (self.in_base % 20)) & 0xFFFF
            elif src_sel == 1:
                src_val = self.reg_x
            elif src_sel == 2:
                src_val = self.reg_y
            elif src_sel == 3:
                src_val = 0
            elif src_sel == 4:
                src_val = self.reg_t
            elif src_sel == 5:
                src_val = self.status_byte
            elif src_sel == 6:
                src_val = self.crc.crc_out
            elif src_sel == 7:
                src_val = self.reg_capture

            if alu_op == 0:
                res = src_val
            elif alu_op == 1:
                res = (~src_val) & 0xFFFF
            elif alu_op == 2:
                res = bitrev16(src_val)
            elif alu_op == 3:
                res = byteswap16(src_val)
            else:
                res = src_val

            if dst_sel == 0:
                for k in range(self.out_count):
                    bit = (res >> k) & 1
                    pin_idx = (self.out_base + k) % 20
                    if bit:
                        self.pin_drivers |= (1 << pin_idx)
                    else:
                        self.pin_drivers &= ~(1 << pin_idx)
            elif dst_sel == 1:
                self.reg_x = res
            elif dst_sel == 2:
                self.reg_y = res
            elif dst_sel == 3:
                self.reg_dl = res
            elif dst_sel == 4:
                for k in range(self.out_count):
                    bit = (res >> k) & 1
                    pin_idx = (self.out_base + k) % 20
                    if bit:
                        self.pindirs |= (1 << pin_idx)
                    else:
                        self.pindirs &= ~(1 << pin_idx)
            elif dst_sel == 5:
                self.pc = res & 0x3F
                return
            elif dst_sel == 6:
                self.reg_isr = res
                self.isr_count = 16
            elif dst_sel == 7:
                self.reg_osr = res
                self.osr_count = 0

            self.pc = next_pc

        elif op == 6:  # SET
            dst_sel = (operand >> 6) & 7
            imm = operand & 0x3F

            if dst_sel == 0:  # pins
                for k in range(self.set_count):
                    bit = (imm >> k) & 1
                    pin_idx = (self.set_base + k) % 20
                    if bit:
                        self.pin_drivers |= (1 << pin_idx)
                    else:
                        self.pin_drivers &= ~(1 << pin_idx)
            elif dst_sel == 1:
                self.reg_x = imm
            elif dst_sel == 2:
                self.reg_y = imm
            elif dst_sel == 3:
                for k in range(self.set_count):
                    bit = (imm >> k) & 1
                    pin_idx = (self.set_base + k) % 20
                    if bit:
                        self.pindirs |= (1 << pin_idx)
                    else:
                        self.pindirs &= ~(1 << pin_idx)
            elif dst_sel == 4:  # flagset
                self.flag_set_strobe |= (1 << (imm & 7))
            elif dst_sel == 5:  # flagclr
                self.flag_clr_strobe |= (1 << (imm & 7))
            elif dst_sel == 6:  # t
                self.reg_t = imm
            elif dst_sel == 7:  # crc
                if imm == 0:
                    self.crc.reset()
                else:
                    self.crc.preset()

            self.pc = next_pc

        elif op == 7:  # TIME
            mode = (operand >> 7) & 3
            imm = operand & 0x7F
            if mode == 0:
                self.reg_dl = (self.reg_t + imm) & 0xFFFF
            elif mode == 1:
                self.reg_dl = (self.reg_dl + imm) & 0xFFFF
            elif mode == 2:
                self.reg_dl = (self.reg_t + self.reg_x) & 0xFFFF
            elif mode == 3:
                self.reg_dl = (self.reg_dl + self.reg_x) & 0xFFFF
            self.pc = next_pc


class ForgeSimulator:
    """
    Top-Level Bit-Accurate & Cycle-Accurate Golden Reference Simulator.
    Simulates Cores 0, 1, 2, Inter-Core Crossbar, 20-bit GPIO Bus, and ABU.
    """

    def __init__(self):
        self.cores = [
            ForgeCore(core_id=0, imem_words=48, fifo_depth=8),
            ForgeCore(core_id=1, imem_words=48, fifo_depth=8),
            ForgeCore(core_id=2, imem_words=32, fifo_depth=4),
        ]
        self.flags: int = 0
        self.gpio_in_external: int = 0  # External input pins (e.g. ui_in, uio_in)
        self.pin_od_mask: int = 0       # Open drain mask for GPIO 16..19
        self.cycle_count: int = 0

        # Auto-Baud Unit
        self.abu_min_pulse: int = 0xFFFF
        self.abu_pulse_cnt: int = 0
        self.abu_prev_pin: int = 1
        self.abu_capture: int = 0

    def reset(self):
        self.flags = 0
        self.cycle_count = 0
        self.abu_min_pulse = 0xFFFF
        self.abu_pulse_cnt = 0
        self.abu_prev_pin = 1
        self.abu_capture = 0
        for c in self.cores:
            c.reset()

    def load_program(self, core_id: int, prog: Any, origin: int = 0):
        self.cores[core_id].load_program(prog, origin)

    def write_reg(self, core_id: int, offset: int, val: int):
        core = self.cores[core_id]
        val = val & 0xFF
        if offset == OFFSET_DIV_INT_L:
            core.div_int = (core.div_int & 0xFF00) | val
        elif offset == OFFSET_DIV_INT_H:
            core.div_int = (core.div_int & 0x00FF) | (val << 8)
        elif offset == OFFSET_DIV_FRAC:
            core.div_frac = val
        elif offset == OFFSET_OUT_BASE:
            core.out_base = val & 0x1F
        elif offset == OFFSET_OUT_COUNT:
            core.out_count = val & 0x1F
        elif offset == OFFSET_SET_BASE:
            core.set_base = val & 0x1F
        elif offset == OFFSET_SET_COUNT:
            core.set_count = val & 0x1F
        elif offset == OFFSET_IN_BASE:
            core.in_base = val & 0x1F
        elif offset == OFFSET_SIDESET:
            core.sideset_base = val & 0x1F
            core.sideset_count = (val >> 5) & 7
        elif offset == OFFSET_JMP_PIN:
            core.jmp_pin = val & 0x1F
        elif offset == OFFSET_WRAP_TOP:
            core.wrap_top = val & 0x3F
        elif offset == OFFSET_WRAP_BOT:
            core.wrap_bot = val & 0x3F
        elif offset == OFFSET_SHIFTCTRL:
            core.in_shift_dir = val & 1
            core.out_shift_dir = (val >> 1) & 1
            core.autopush = bool((val >> 2) & 1)
            core.autopull = bool((val >> 3) & 1)
            thresh = (val >> 4) & 0xF
            core.push_thresh = 16 if thresh == 0 else thresh
        elif offset == 0x0D:  # PULL_THRESH
            thresh = val & 0xF
            core.pull_thresh = 16 if thresh == 0 else thresh
        elif offset == 0x0E:  # EXECCTRL
            core.scl_stretch_en = bool(val & 1)
            core.arb_detect_en = bool((val >> 1) & 1)
            core.jmp_cond_sel = (val >> 2) & 3
            core.sniffer_en = bool((val >> 4) & 1)
            core.scl_pin_idx = (val >> 5) & 7
        elif offset == OFFSET_PC:
            core.pc = val & 0x3F
        elif offset == OFFSET_SBE_CTRL:
            core.sbe_bsu.sbe_mode = val & 7
        elif offset == OFFSET_BSU_CTRL:
            core.sbe_bsu.bsu_mode = val & 3
        elif offset == OFFSET_CRC_CTRL:
            core.crc.poly_sel = val & 3
            core.crc_snoop_rx = bool((val >> 2) & 1)
            core.crc_enable = bool((val >> 3) & 1)
            core.crc.invert_out = bool((val >> 4) & 1)
        elif offset == OFFSET_CRC_VAL_L:
            core.crc.crc_reg = (core.crc.crc_reg & 0xFF00) | val
        elif offset == OFFSET_CRC_VAL_H:
            core.crc.crc_reg = (core.crc.crc_reg & 0x00FF) | (val << 8)

    def read_reg(self, core_id: int, offset: int) -> int:
        core = self.cores[core_id]
        if offset == OFFSET_DIV_INT_L:
            return core.div_int & 0xFF
        elif offset == OFFSET_DIV_INT_H:
            return (core.div_int >> 8) & 0xFF
        elif offset == OFFSET_DIV_FRAC:
            return core.div_frac
        elif offset == OFFSET_PC:
            return core.pc
        elif offset == OFFSET_STATUS:
            return core.status_byte
        elif offset == OFFSET_CRC_VAL_L:
            return core.crc.crc_out & 0xFF
        elif offset == OFFSET_CRC_VAL_H:
            return (core.crc.crc_out >> 8) & 0xFF
        elif offset == OFFSET_CAPTURE_L:
            return core.reg_capture & 0xFF
        elif offset == OFFSET_CAPTURE_H:
            return (core.reg_capture >> 8) & 0xFF
        return 0

    def step(self, cycles: int = 1) -> List[Dict[str, Any]]:
        """Advance the simulation by N cycles."""
        traces = []
        for _ in range(cycles):
            self.cycle_count += 1

            # Compute combined output pin states
            combined_pin_out = 0
            for c in self.cores:
                combined_pin_out |= c.active_pin_out

            # Resolve Open-Drain pins 16..19
            resolved_gpio = combined_pin_out
            for p in range(4):
                pin_idx = 16 + p
                if (self.pin_od_mask >> p) & 1:
                    # Open-drain: driving 0 pulls down, driving 1 releases
                    if ((combined_pin_out >> pin_idx) & 1) == 0:
                        resolved_gpio &= ~(1 << pin_idx)
                    else:
                        ext_val = (self.gpio_in_external >> pin_idx) & 1
                        if ext_val:
                            resolved_gpio |= (1 << pin_idx)
                        else:
                            resolved_gpio &= ~(1 << pin_idx)

            # Combined GPIO input bus seen by cores
            full_gpio_in = (self.gpio_in_external & 0x000FF) | (resolved_gpio & 0xFFFF0)

            # Auto-Baud Unit update
            abu_pin = full_gpio_in & 1
            if abu_pin != self.abu_prev_pin:
                self.abu_prev_pin = abu_pin
                self.abu_capture = self.cycle_count & 0xFFFF
                if self.abu_pulse_cnt > 2 and self.abu_pulse_cnt < self.abu_min_pulse:
                    self.abu_min_pulse = self.abu_pulse_cnt
                self.abu_pulse_cnt = 1
            else:
                if self.abu_pulse_cnt < 0xFFFF:
                    self.abu_pulse_cnt += 1

            # Step all cores
            core_traces = []
            for c in self.cores:
                tr = c.clock_step(full_gpio_in, self.flags)
                core_traces.append(tr)

            # Crossbar flag update
            for c in self.cores:
                self.flags = (self.flags | c.flag_set_strobe) & ~c.flag_clr_strobe

            traces.append({
                "cycle": self.cycle_count,
                "gpio": full_gpio_in,
                "cores": core_traces,
            })
        return traces

    def run_until(self, condition_fn, max_cycles: int = 100_000) -> int:
        """Run until condition_fn(sim) evaluates to True or max_cycles is reached."""
        for _ in range(max_cycles):
            self.step(1)
            if condition_fn(self):
                return self.cycle_count
        return self.cycle_count
