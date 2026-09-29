"""
ProtocolForge Macro Assembler & Disassembler.
Architecture Specification: ARCH_PROPOSAL_CHIEF.md
"""

from __future__ import annotations
import re
import sys
from typing import List, Dict, Tuple, Optional, Any, Union

# ---------------------------------------------------------------------------
# Opcode & Field Definitions (16-bit word)
# [15:13] = Opcode (3 bits)
# [12:9]  = Delay / Side-set (4 bits)
# [8:0]   = Operand (9 bits)
# ---------------------------------------------------------------------------

OPCODE_JMP  = 0b000  # 0
OPCODE_WAIT = 0b001  # 1
OPCODE_IN   = 0b010  # 2
OPCODE_OUT  = 0b011  # 3
OPCODE_PUSH_PULL = 0b100  # 4
OPCODE_MOV  = 0b101  # 5
OPCODE_SET  = 0b110  # 6
OPCODE_TIME = 0b111  # 7

# JMP Conditions (operand[8:6])
JMP_COND_MAP = {
    "": 0,
    "none": 0,
    "!x": 1,
    "x--": 2,
    "!y": 3,
    "y--": 4,
    "x!=y": 5,
    "pin": 6,
    "!crc": 7,
    "arb_lost": 7,
    "arblost": 7,
    "!osre": 7,
    "crc_err": 7,
    "stuff_err": 7,
    "bus_event": 7,
}
JMP_COND_REV = {
    0: "",
    1: "!x",
    2: "x--",
    3: "!y",
    4: "y--",
    5: "x!=y",
    6: "pin",
    7: "!crc",
}

# WAIT Sources (operand[7:6])
WAIT_SRC_MAP = {
    "gpio": 0b00,
    "pin": 0b01,
    "flag": 0b10,
    "time": 0b11,
    "edge": 0b11,
}
WAIT_SRC_REV = {
    0b00: "gpio",
    0b01: "pin",
    0b10: "flag",
    0b11: "time",
}

# IN Sources (operand[8:6])
IN_SRC_MAP = {
    "pins": 0,
    "x": 1,
    "y": 2,
    "null": 3,
    "t": 4,
    "status": 5,
    "crc": 6,
    "capture": 7,
}
IN_SRC_REV = {v: k for k, v in IN_SRC_MAP.items()}

# OUT Destinations (operand[8:6])
OUT_DST_MAP = {
    "pins": 0,
    "x": 1,
    "y": 2,
    "null": 3,
    "pindirs": 4,
    "pc": 5,
    "isr": 6,
    "dl": 7,
}
OUT_DST_REV = {v: k for k, v in OUT_DST_MAP.items()}

# MOV Destinations (operand[8:6])
MOV_DST_MAP = {
    "pins": 0,
    "x": 1,
    "y": 2,
    "dl": 3,
    "null": 3,
    "pindirs": 4,
    "exec": 4,
    "pc": 5,
    "isr": 6,
    "osr": 7,
    "crc": 7,
}
MOV_DST_REV = {
    0: "pins",
    1: "x",
    2: "y",
    3: "dl",
    4: "pindirs",
    5: "pc",
    6: "isr",
    7: "osr",
}

# MOV ALU Operations (operand[5:4])
MOV_OP_NONE   = 0b00
MOV_OP_INVERT = 0b01
MOV_OP_REV    = 0b10
MOV_OP_SWAP   = 0b11

# MOV Sources (operand[3:0]: bit 3 enables extended sources isr/osr)
MOV_SRC_MAP = {
    "pins": 0,
    "x": 1,
    "y": 2,
    "null": 3,
    "t": 4,
    "status": 5,
    "crc": 6,
    "capture": 7,
    "isr": 8,
    "osr": 9,
}
MOV_SRC_REV = {
    0: "pins",
    1: "x",
    2: "y",
    3: "null",
    4: "t",
    5: "status",
    6: "crc",
    7: "capture",
    8: "isr",
    9: "osr",
}

# SET Destinations (operand[8:6])
SET_DST_MAP = {
    "pins": 0,
    "x": 1,
    "y": 2,
    "pindirs": 3,
    "flagset": 4,
    "flags": 4,
    "flag_set": 4,
    "flagclr": 5,
    "flag_clr": 5,
    "t": 6,
    "crc": 7,
}
SET_DST_REV = {
    0: "pins",
    1: "x",
    2: "y",
    3: "pindirs",
    4: "flagset",
    5: "flagclr",
    6: "t",
    7: "crc",
}

# TIME Modes (operand[8:7])
TIME_MODE_T_IMM  = 0b00  # time t+N
TIME_MODE_DL_IMM = 0b01  # time dl+N
TIME_MODE_T_X    = 0b10  # time t+x
TIME_MODE_DL_X   = 0b11  # time dl+x


class AssemblyError(Exception):
    """Assembly error with line number and source context."""
    def __init__(self, message: str, line_no: Optional[int] = None, line_text: Optional[str] = None):
        super().__init__(message)
        self.message = message
        self.line_no = line_no
        self.line_text = line_text

    def __str__(self) -> str:
        loc = f"Line {self.line_no}: " if self.line_no is not None else ""
        text = f"\n  --> {self.line_text}" if self.line_text else ""
        return f"{loc}{self.message}{text}"


class Program:
    """Assembled ProtocolForge Program."""
    def __init__(
        self,
        name: str = "protocol_forge_program",
        instructions: Optional[List[int]] = None,
        labels: Optional[Dict[str, int]] = None,
        wrap_target: int = 0,
        wrap_bottom: int = 0,
        sideset_count: int = 0,
        sideset_opt: bool = False,
    ):
        self.name = name
        self.instructions: List[int] = instructions or []
        self.labels: Dict[str, int] = labels or {}
        self.wrap_target = wrap_target
        self.wrap_bottom = wrap_bottom
        self.sideset_count = sideset_count
        self.sideset_opt = sideset_opt

    @property
    def wrap_top(self) -> int:
        return self.wrap_bottom

    @property
    def wrap_bot(self) -> int:
        return self.wrap_target

    @property
    def words(self) -> List[int]:
        return self.instructions

    def __len__(self) -> int:
        return len(self.instructions)

    def __getitem__(self, idx: int) -> int:
        return self.instructions[idx]

    def to_hex(self, prefix: bool = True, uppercase: bool = True) -> List[str]:
        """Return list of hex strings for each instruction word."""
        fmt = "0x{:04X}" if (prefix and uppercase) else ("0x{:04x}" if prefix else ("{:04X}" if uppercase else "{:04x}"))
        return [fmt.format(w) for w in self.instructions]

    def to_bin(self) -> List[str]:
        """Return list of 16-bit binary strings."""
        return [f"{w:016b}" for w in self.instructions]

    def to_bytes(self, byteorder: str = "little") -> bytes:
        """Convert program to raw byte stream."""
        b = bytearray()
        for w in self.instructions:
            b.extend(w.to_bytes(2, byteorder=byteorder))
        return bytes(b)

    def to_c_array(self, var_name: Optional[str] = None) -> str:
        """Generate C header array."""
        name = var_name or self.name
        lines = [f"// ProtocolForge Program: {name} ({len(self.instructions)} words)"]
        lines.append(f"// wrap_target = {self.wrap_target}, wrap_bottom = {self.wrap_bottom}, side_set = {self.sideset_count}")
        lines.append(f"static const uint16_t {name}_program_instructions[] = {{")
        for i, w in enumerate(self.instructions):
            lines.append(f"    0x{w:04x}, // {i:2d}")
        lines.append("};")
        return "\n".join(lines)


def _parse_int(val_str: str, defines: Dict[str, Any]) -> int:
    """Parse integer from decimal, hex (0x..), binary (0b..), or defined constant."""
    val_str = val_str.strip()
    if val_str in defines:
        return int(defines[val_str])
    for k, v in defines.items():
        if k.lower() == val_str.lower():
            return int(v)
    if val_str.startswith("0x") or val_str.startswith("0X"):
        return int(val_str, 16)
    if val_str.startswith("0b") or val_str.startswith("0B"):
        return int(val_str, 2)
    return int(val_str)


class Assembler:
    """Two-Pass Macro Assembler for ProtocolForge."""

    def __init__(self):
        self.defines: Dict[str, Any] = {}
        self.labels: Dict[str, int] = {}
        self.instructions: List[int] = []
        self.sideset_count = 0
        self.sideset_opt = False
        self.sideset_pindirs = False
        self.wrap_target: Optional[int] = None
        self.wrap_bottom: Optional[int] = None
        self.program_name = "program"

    def assemble(self, source: str) -> Program:
        """Assemble source code string into a Program object."""
        self.__init__()  # Reset state
        lines = source.splitlines()

        # Pass 1: Parse directives, labels, side_set config, definitions
        clean_lines: List[Tuple[int, str, Optional[str]]] = []
        pc = 0

        for line_no, raw_line in enumerate(lines, start=1):
            # Strip comments (; or //)
            line = re.split(r";|//", raw_line)[0].strip()
            if not line:
                continue

            # Check for directives
            if line.startswith("."):
                tokens = line.split(maxsplit=2)
                directive = tokens[0].lower()

                if directive == ".program":
                    if len(tokens) > 1:
                        self.program_name = tokens[1].strip()
                    continue
                elif directive == ".side_set":
                    if len(tokens) > 1:
                        self.sideset_count = _parse_int(tokens[1], self.defines)
                    if len(tokens) > 2:
                        rest = tokens[2].lower()
                        if "opt" in rest:
                            self.sideset_opt = True
                        if "pindirs" in rest:
                            self.sideset_pindirs = True
                    continue
                elif directive == ".wrap_target":
                    self.wrap_target = pc
                    continue
                elif directive == ".wrap":
                    self.wrap_bottom = max(0, pc - 1)
                    continue
                elif directive == ".define":
                    parts = line.split(maxsplit=2)
                    if len(parts) >= 3:
                        name = parts[1].strip()
                        val = _parse_int(parts[2].strip(), self.defines)
                        self.defines[name] = val
                    continue
                elif directive == ".origin":
                    parts = line.split(maxsplit=1)
                    if len(parts) >= 2:
                        pc = _parse_int(parts[1].strip(), self.defines)
                    continue

            # Check for labels (e.g., "label:" or "label: opcode ...")
            while ":" in line:
                colon_idx = line.index(":")
                label_name = line[:colon_idx].strip()
                if " " in label_name:
                    # Not a pure label prefix
                    break
                if label_name in self.labels:
                    raise AssemblyError(f"Duplicate label '{label_name}'", line_no, raw_line)
                self.labels[label_name] = pc
                line = line[colon_idx + 1:].strip()
                if not line:
                    break

            if line:
                clean_lines.append((line_no, line, raw_line))
                pc += 1

        # Fallback defaults for wrap
        if self.wrap_target is None:
            self.wrap_target = 0
        if self.wrap_bottom is None:
            self.wrap_bottom = max(0, len(clean_lines) - 1)

        # Check if side_set was not explicitly declared but `side` is used in clean_lines
        if self.sideset_count == 0:
            max_side_val = 0
            has_side = False
            for _, text, _ in clean_lines:
                side_m = re.search(r"\bside\s+(0b[01]+|0x[0-9a-fA-F]+|\d+)", text)
                if side_m:
                    has_side = True
                    val = _parse_int(side_m.group(1), self.defines)
                    if val > max_side_val:
                        max_side_val = val
            if has_side:
                # Infer required side-set bits
                bits = max(1, max_side_val.bit_length())
                self.sideset_count = min(4, bits)

        # Pass 2: Assemble instructions
        assembled_words: List[int] = []
        for current_pc, (line_no, line_text, raw_line) in enumerate(clean_lines):
            try:
                word = self._assemble_line(line_text, current_pc)
                assembled_words.append(word)
            except AssemblyError as e:
                e.line_no = line_no
                e.line_text = raw_line
                raise e
            except Exception as e:
                raise AssemblyError(f"Syntax error: {e}", line_no, raw_line) from e

        return Program(
            name=self.program_name,
            instructions=assembled_words,
            labels=self.labels,
            wrap_target=self.wrap_target,
            wrap_bottom=self.wrap_bottom,
            sideset_count=self.sideset_count,
            sideset_opt=self.sideset_opt,
        )

    def _assemble_line(self, line: str, current_pc: int) -> int:
        """Parse instruction line with side-set and delay annotations."""
        # Extract side-set: `side <val>`
        side_val = 0
        has_explicit_side = False
        side_match = re.search(r"\bside\s+([0-9a-zA-Z_]+)", line)
        if side_match:
            side_str = side_match.group(1)
            side_val = _parse_int(side_str, self.defines)
            has_explicit_side = True
            # Remove `side ...` from line
            line = line[:side_match.start()] + line[side_match.end():]

        # Extract delay / bracketed side: `[<val>]`
        delay_val = 0
        delay_match = re.search(r"\[\s*([0-9a-zA-Z_]+)\s*\]", line)
        if delay_match:
            d_str = delay_match.group(1)
            parsed_d = _parse_int(d_str, self.defines)
            # If user configured full 4-bit sideset and no delay bits, [val] can specify side
            if self.sideset_count == 4 and not has_explicit_side:
                side_val = parsed_d
                has_explicit_side = True
            else:
                delay_val = parsed_d
            line = line[:delay_match.start()] + line[delay_match.end():]

        # Pack delay and side-set into 4 bits: bits [12:9]
        delay_bits = max(0, 4 - self.sideset_count)
        max_delay = (1 << delay_bits) - 1 if delay_bits > 0 else 0
        max_side = (1 << self.sideset_count) - 1 if self.sideset_count > 0 else 0

        if delay_val > max_delay:
            raise AssemblyError(f"Delay value {delay_val} exceeds maximum allowed ({max_delay}) with {self.sideset_count} side-set bits")
        if side_val > max_side:
            raise AssemblyError(f"Side-set value {side_val} exceeds maximum allowed ({max_side}) for {self.sideset_count} bits")

        delay_side = ((side_val & max_side) << delay_bits) | (delay_val & max_delay)

        # Clean remaining line for mnemonic and operands
        line = line.strip()
        if not line:
            # Standalone delay/side-set without instruction defaults to nop
            line = "nop"

        parts = line.split(maxsplit=1)
        mnemonic = parts[0].lower()
        args_str = parts[1].strip() if len(parts) > 1 else ""

        # Parse opcode and operand (9 bits [8:0])
        opcode, operand = self._parse_instruction(mnemonic, args_str, current_pc)

        return ((opcode & 0x7) << 13) | ((delay_side & 0xF) << 9) | (operand & 0x1FF)

    def _parse_instruction(self, mnemonic: str, args_str: str, current_pc: int) -> Tuple[int, int]:
        """Parse mnemonic and args into (opcode_3b, operand_9b)."""
        args = [a.strip() for a in args_str.split(",") if a.strip()] if args_str else []

        if mnemonic == "nop":
            # NOP is encoded as `mov y, y` (opcode 5, dst=2, op=0, src=2)
            opcode = OPCODE_MOV
            # dst=2 (y), op=0 (none), bit3=0, src=2 (y)
            operand = (MOV_DST_MAP["y"] << 6) | (MOV_OP_NONE << 4) | (0 << 3) | MOV_SRC_MAP["y"]
            return opcode, operand

        elif mnemonic == "jmp":
            return self._parse_jmp(args_str, current_pc)

        elif mnemonic == "wait":
            return self._parse_wait(args_str)

        elif mnemonic == "in":
            return self._parse_in(args)

        elif mnemonic == "out":
            return self._parse_out(args)

        elif mnemonic in ("push", "pull"):
            return self._parse_push_pull(mnemonic, args_str)

        elif mnemonic == "mov":
            return self._parse_mov(args)

        elif mnemonic == "set":
            return self._parse_set(args)

        elif mnemonic == "time":
            return self._parse_time(args_str)

        else:
            raise AssemblyError(f"Unknown instruction mnemonic '{mnemonic}'")

    def _parse_jmp(self, args_str: str, current_pc: int) -> Tuple[int, int]:
        """
        000 — JMP jmp [cond] addr
        operand[8:6] = Condition, operand[5:0] = Target Address (0..63).
        """
        tokens = args_str.split()
        if not tokens:
            raise AssemblyError("jmp requires target address")

        cond_str = ""
        target_token = ""

        if len(tokens) == 1:
            # Unconditional: jmp target
            cond_str = ""
            target_token = tokens[0]
        elif len(tokens) == 2:
            # Conditional: jmp cond target
            cond_str = tokens[0].lower()
            target_token = tokens[1]
        else:
            raise AssemblyError(f"Too many arguments for jmp: '{args_str}'")

        if cond_str not in JMP_COND_MAP:
            raise AssemblyError(f"Invalid jmp condition '{cond_str}'. Valid: {list(JMP_COND_MAP.keys())}")
        cond_code = JMP_COND_MAP[cond_str]

        # Resolve target address
        if target_token in self.labels:
            target_addr = self.labels[target_token]
        else:
            target_addr = _parse_int(target_token, self.defines)

        if not (0 <= target_addr <= 63):
            raise AssemblyError(f"JMP target address {target_addr} out of range (0..63)")

        operand = ((cond_code & 0x7) << 6) | (target_addr & 0x3F)
        return OPCODE_JMP, operand

    def _parse_wait(self, args_str: str) -> Tuple[int, int]:
        """
        001 — WAIT wait [pol] [src] [idx] / wait time / wait edge [n]
        operand[8] = Polarity, operand[7:6] = Source, operand[5:0] = Index/Pin.
        """
        tokens = args_str.split()
        if not tokens:
            raise AssemblyError("wait instruction requires arguments")

        # Handle aliases: `wait time`, `wait edge`, `wait edge <idx>`
        first = tokens[0].lower()
        if first == "time":
            # wait time -> pol=0, src=time(3), idx=0
            return OPCODE_WAIT, (0 << 8) | (WAIT_SRC_MAP["time"] << 6) | 0
        elif first == "edge":
            idx = _parse_int(tokens[1], self.defines) if len(tokens) > 1 else 1
            return OPCODE_WAIT, (0 << 8) | (WAIT_SRC_MAP["edge"] << 6) | (idx & 0x3F)

        # General syntax: wait <pol> <src> <idx>
        if len(tokens) < 3:
            # Check for `wait <pol> time` or `wait <pol> edge`
            if len(tokens) == 2 and tokens[1].lower() in ("time", "edge"):
                pol = _parse_int(tokens[0], self.defines)
                src = WAIT_SRC_MAP[tokens[1].lower()]
                idx = 0 if tokens[1].lower() == "time" else 1
                return OPCODE_WAIT, ((pol & 1) << 8) | ((src & 3) << 6) | (idx & 0x3F)
            raise AssemblyError(f"wait syntax: wait [pol] [gpio|pin|flag] [idx] or wait time/wait edge. Got: '{args_str}'")

        pol = _parse_int(tokens[0], self.defines) & 1
        src_name = tokens[1].lower()
        if src_name not in WAIT_SRC_MAP:
            raise AssemblyError(f"Invalid wait source '{src_name}'. Valid: gpio, pin, flag, time, edge")
        src = WAIT_SRC_MAP[src_name]
        idx = _parse_int(tokens[2], self.defines)

        if not (0 <= idx <= 63):
            raise AssemblyError(f"Wait index {idx} out of range (0..63)")

        operand = (pol << 8) | ((src & 3) << 6) | (idx & 0x3F)
        return OPCODE_WAIT, operand

    def _parse_in(self, args: List[str]) -> Tuple[int, int]:
        """
        010 — IN in [src], n
        operand[8:6] = Source, operand[5:0] = n (1..16).
        """
        if len(args) != 2:
            raise AssemblyError("in instruction requires 2 arguments: in <src>, <bitcount>")

        src_name = args[0].lower()
        if src_name not in IN_SRC_MAP:
            raise AssemblyError(f"Invalid in source '{src_name}'. Valid: {list(IN_SRC_MAP.keys())}")
        src = IN_SRC_MAP[src_name]

        count = _parse_int(args[1], self.defines)
        if not (1 <= count <= 16):
            raise AssemblyError(f"in bit count must be 1..16, got {count}")

        operand = ((src & 7) << 6) | (count & 0x3F)
        return OPCODE_IN, operand

    def _parse_out(self, args: List[str]) -> Tuple[int, int]:
        """
        011 — OUT out [dst], n
        operand[8:6] = Destination, operand[5:0] = n (1..16).
        """
        if len(args) != 2:
            raise AssemblyError("out instruction requires 2 arguments: out <dst>, <bitcount>")

        dst_name = args[0].lower()
        if dst_name not in OUT_DST_MAP:
            raise AssemblyError(f"Invalid out destination '{dst_name}'. Valid: {list(OUT_DST_MAP.keys())}")
        dst = OUT_DST_MAP[dst_name]

        count = _parse_int(args[1], self.defines)
        if not (1 <= count <= 16):
            raise AssemblyError(f"out bit count must be 1..16, got {count}")

        operand = ((dst & 7) << 6) | (count & 0x3F)
        return OPCODE_OUT, operand

    def _parse_push_pull(self, mnemonic: str, args_str: str) -> Tuple[int, int]:
        """
        100 — PUSH / PULL
        operand[8] = 0 for PUSH, 1 for PULL.
        operand[7] = iffull / ifempty (1=conditional).
        operand[6] = block (1=block, 0=noblock).
        operand[5:0] = 0.
        """
        is_pull = 1 if mnemonic == "pull" else 0
        cond = 0
        block = 1  # default is block

        tokens = args_str.lower().split()
        for t in tokens:
            if t in ("iffull", "ifempty"):
                cond = 1
            elif t == "block":
                block = 1
            elif t == "noblock":
                block = 0
            else:
                raise AssemblyError(f"Unknown {mnemonic} argument '{t}'")

        operand = (is_pull << 8) | (cond << 7) | (block << 6)
        return OPCODE_PUSH_PULL, operand

    def _parse_mov(self, args: List[str]) -> Tuple[int, int]:
        """
        101 — MOV mov [dst], [op][src]
        operand[8:6] = Destination
        operand[5:4] = ALU Operation (00=none, 01=invert, 10=reverse, 11=byteswap)
        operand[3]   = 0
        operand[2:0] = Source
        """
        if len(args) != 2:
            raise AssemblyError("mov requires 2 arguments: mov <dst>, [op]<src>")

        dst_name = args[0].lower()
        if dst_name not in MOV_DST_MAP:
            raise AssemblyError(f"Invalid mov destination '{dst_name}'. Valid: {list(MOV_DST_MAP.keys())}")
        dst = MOV_DST_MAP[dst_name]

        src_expr = args[1].strip()
        op = MOV_OP_NONE
        src_name = src_expr.lower()

        if src_name.startswith("~") or src_name.startswith("!"):
            op = MOV_OP_INVERT
            src_name = src_name[1:].strip()
        elif src_name.startswith("::"):
            op = MOV_OP_REV
            src_name = src_name[2:].strip()
        elif src_name.startswith("swap(") and src_name.endswith(")"):
            op = MOV_OP_SWAP
            src_name = src_name[5:-1].strip()
        elif src_name.startswith("swap "):
            op = MOV_OP_SWAP
            src_name = src_name[5:].strip()

        if src_name not in MOV_SRC_MAP:
            raise AssemblyError(f"Invalid mov source '{src_name}'. Valid: {list(MOV_SRC_MAP.keys())}")
        src = MOV_SRC_MAP[src_name]

        bit3 = 1 if src >= 8 else 0
        operand = ((dst & 7) << 6) | ((op & 3) << 4) | (bit3 << 3) | (src & 7)
        return OPCODE_MOV, operand

    def _parse_set(self, args: List[str]) -> Tuple[int, int]:
        """
        110 — SET set [dst], imm
        operand[8:6] = Destination, operand[5:0] = Immediate (0..63).
        """
        if len(args) != 2:
            raise AssemblyError("set requires 2 arguments: set <dst>, <imm>")

        dst_name = args[0].lower()
        if dst_name not in SET_DST_MAP:
            raise AssemblyError(f"Invalid set destination '{dst_name}'. Valid: {list(SET_DST_MAP.keys())}")
        dst = SET_DST_MAP[dst_name]

        imm = _parse_int(args[1], self.defines)
        if not (0 <= imm <= 63):
            raise AssemblyError(f"set immediate {imm} out of range (0..63)")

        operand = ((dst & 7) << 6) | (imm & 0x3F)
        return OPCODE_SET, operand

    def _parse_time(self, args_str: str) -> Tuple[int, int]:
        """
        111 — TIME / EXT time [base]+[add]
        00: time t+N   (operand[8:7]=00, operand[6:0]=N)
        01: time dl+N  (operand[8:7]=01, operand[6:0]=N)
        10: time t+x   (operand[8:7]=10, operand[6:0]=0)
        11: time dl+x  (operand[8:7]=11, operand[6:0]=0)
        """
        expr = args_str.replace(" ", "")
        if not expr:
            raise AssemblyError("time requires base+offset specification (e.g. t+50, dl+10, t+x, dl+x)")

        if "+" not in expr:
            raise AssemblyError(f"time expression must be format base+offset, got '{args_str}'")

        base_raw, offset = expr.split("+", 1)
        base = base_raw.lower()
        offset_lower = offset.lower()

        if base == "t":
            if offset_lower in ("x", "y"):
                mode = TIME_MODE_T_X
                imm = 0 if offset_lower == "x" else 1
            else:
                mode = TIME_MODE_T_IMM
                imm = _parse_int(offset, self.defines)
        elif base == "dl":
            if offset_lower in ("x", "y"):
                mode = TIME_MODE_DL_X
                imm = 0 if offset_lower == "x" else 1
            else:
                mode = TIME_MODE_DL_IMM
                imm = _parse_int(offset, self.defines)
        else:
            raise AssemblyError(f"Unknown time base '{base}'. Valid: 't', 'dl'")

        if not (0 <= imm <= 127):
            raise AssemblyError(f"time immediate {imm} out of range (0..127)")

        operand = ((mode & 3) << 7) | (imm & 0x7F)
        return OPCODE_TIME, operand


# ---------------------------------------------------------------------------
# Disassembler
# ---------------------------------------------------------------------------

def disassemble_instruction(word: int, sideset_count: int = 0) -> str:
    """Disassemble a 16-bit binary instruction word into ProtocolForge assembly."""
    word &= 0xFFFF
    opcode = (word >> 13) & 0x7
    delay_side = (word >> 9) & 0xF
    operand = word & 0x1FF

    # Unpack side-set and delay
    delay_bits = max(0, 4 - sideset_count)
    side_mask = (1 << sideset_count) - 1 if sideset_count > 0 else 0
    delay_mask = (1 << delay_bits) - 1 if delay_bits > 0 else 0

    side_val = (delay_side >> delay_bits) & side_mask if sideset_count > 0 else 0
    delay_val = delay_side & delay_mask if delay_bits > 0 else 0

    inst_str = ""

    if opcode == OPCODE_JMP:
        cond_code = (operand >> 6) & 0x7
        target = operand & 0x3F
        cond = JMP_COND_REV.get(cond_code, f"cond{cond_code}")
        if cond:
            inst_str = f"jmp {cond} {target}"
        else:
            inst_str = f"jmp {target}"

    elif opcode == OPCODE_WAIT:
        pol = (operand >> 8) & 0x1
        src = (operand >> 6) & 0x3
        idx = operand & 0x3F
        if src == 0b11 and idx == 0:
            inst_str = "wait time"
        elif src == 0b11 and idx == 1:
            inst_str = "wait edge"
        elif src == 0b11:
            inst_str = f"wait edge {idx}"
        else:
            src_name = WAIT_SRC_REV.get(src, f"src{src}")
            inst_str = f"wait {pol} {src_name} {idx}"

    elif opcode == OPCODE_IN:
        src = (operand >> 6) & 0x7
        count = operand & 0x3F
        bit_count = 16 if count == 0 else count
        src_name = IN_SRC_REV.get(src, f"src{src}")
        inst_str = f"in {src_name}, {bit_count}"

    elif opcode == OPCODE_OUT:
        dst = (operand >> 6) & 0x7
        count = operand & 0x3F
        bit_count = 16 if count == 0 else count
        dst_name = OUT_DST_REV.get(dst, f"dst{dst}")
        inst_str = f"out {dst_name}, {bit_count}"

    elif opcode == OPCODE_PUSH_PULL:
        is_pull = (operand >> 8) & 0x1
        cond = (operand >> 7) & 0x1
        block = (operand >> 6) & 0x1
        mnemonic = "pull" if is_pull else "push"
        cond_str = (" ifempty" if is_pull else " iffull") if cond else ""
        block_str = " block" if block else " noblock"
        inst_str = f"{mnemonic}{cond_str}{block_str}"

    elif opcode == OPCODE_MOV:
        dst = (operand >> 6) & 0x7
        op = (operand >> 4) & 0x3
        bit3 = (operand >> 3) & 0x1
        src = (bit3 << 3) | (operand & 0x7)
        # Check for NOP: mov y, y with op=0 and src=2
        if dst == 2 and op == 0 and src == 2:
            inst_str = "nop"
        else:
            dst_name = MOV_DST_REV.get(dst, f"dst{dst}")
            src_name = MOV_SRC_REV.get(src, f"src{src}")
            if op == MOV_OP_NONE:
                op_str = ""
            elif op == MOV_OP_INVERT:
                op_str = "~"
            elif op == MOV_OP_REV:
                op_str = "::"
            elif op == MOV_OP_SWAP:
                op_str = "swap "
            else:
                op_str = ""
            inst_str = f"mov {dst_name}, {op_str}{src_name}"

    elif opcode == OPCODE_SET:
        dst = (operand >> 6) & 0x7
        imm = operand & 0x3F
        dst_name = SET_DST_REV.get(dst, f"dst{dst}")
        inst_str = f"set {dst_name}, {imm}"

    elif opcode == OPCODE_TIME:
        mode = (operand >> 7) & 0x3
        imm = operand & 0x7F
        if mode == TIME_MODE_T_IMM:
            inst_str = f"time t+{imm}"
        elif mode == TIME_MODE_DL_IMM:
            inst_str = f"time dl+{imm}"
        elif mode == TIME_MODE_T_X:
            reg = "y" if imm == 1 else "x"
            inst_str = f"time t+{reg}"
        elif mode == TIME_MODE_DL_X:
            reg = "y" if imm == 1 else "x"
            inst_str = f"time dl+{reg}"

    else:
        inst_str = f"// unknown opcode {opcode}"

    # Append side-set and delay
    suffix = ""
    if sideset_count > 0 and (side_val != 0 or sideset_count > 0):
        suffix += f" side {side_val}"
    if delay_val != 0:
        suffix += f" [{delay_val}]"

    return (inst_str + suffix).strip()


def disassemble(words: List[int], sideset_count: int = 0) -> str:
    """Disassemble a list of 16-bit machine code words into an assembly program."""
    lines = []
    if sideset_count > 0:
        lines.append(f".side_set {sideset_count}")
    for i, w in enumerate(words):
        asm = disassemble_instruction(w, sideset_count=sideset_count)
        lines.append(f"    {asm:<30} ; {i:02d}: 0x{w:04x}")
    return "\n".join(lines)


def assemble(source: str) -> Program:
    """Convenience wrapper to assemble source code."""
    asm = Assembler()
    return asm.assemble(source)


def assemble_file(path: str) -> Program:
    """Assemble an assembly file from disk."""
    with open(path, "r", encoding="utf-8") as f:
        return assemble(f.read())


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python forge_asm.py <input.asm> [-o output.hex]")
        sys.exit(1)
    infile = sys.argv[1]
    outfile = None
    if "-o" in sys.argv:
        idx = sys.argv.index("-o")
        if idx + 1 < len(sys.argv):
            outfile = sys.argv[idx + 1]

    prog = assemble_file(infile)
    hex_lines = prog.to_hex()
    if outfile:
        with open(outfile, "w", encoding="utf-8") as f:
            for h in hex_lines:
                f.write(h + "\n")
        print(f"Assembled {len(prog)} instructions to {outfile}")
    else:
        for i, h in enumerate(hex_lines):
            print(f"{i:02d}: {h}")
