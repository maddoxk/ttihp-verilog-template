# ProtocolForge: High-Performance Hardware-Accelerated Protocol Emulator ASIC

**Submission for the Jane Street Protocol Emulator ASIC Competition**  
**Target Process:** IHP 130nm CMOS5L (SG13G2) via Tiny Tapeout (6×4 Tiles Allocation)  
**Top Module:** `tt_um_maddox_protocolforge`  
**License:** Apache-2.0  

---

## 1. Executive Summary & Architectural Philosophy

Modern embedded and trading systems interface with a diverse array of physical protocols—from legacy asynchronous serial buses (UART, 1-Wire) to high-speed synchronous lines (SPI, I2C with clock stretching) and packetized differential standards (CAN 2.0, USB 1.1 Low-Speed, 10BASE-T Ethernet). Traditional approaches force engineers into an uncomfortable compromise:
- **Fixed-function hardware peripherals** lack flexibility for custom framing, non-standard baud rates, proprietary packet formats, and real-time reverse engineering.
- **Pure software bit-banging** on general-purpose MCUs incurs unacceptable timing jitter, interrupt latency, and high CPU load.
- **Microcoded I/O engines** like the **Raspberry Pi RP2040 PIO** and **TI Sitara PRU** represent significant progress, but hit fundamental walls when handling complex line-coded, bit-stuffed, or CRC-checked protocols.

### What ProtocolForge Does Differently from RP2040 PIO and TI PRU

Jane Street explicitly challenged entrants: *"What would you do differently from RP2040 PIO and TI PRU?"* ProtocolForge answers this with five architectural breakthroughs:

| Architectural Dimension | Raspberry Pi RP2040 PIO | TI Sitara PRU (32-bit RISC) | **ProtocolForge (This ASIC)** |
| :--- | :--- | :--- | :--- |
| **Line Coding (NRZI, Manchester)** | ❌ None (Requires 4-8 instructions per bit, halving bandwidth) | ❌ None (Software bit-manipulation loop) | ✅ **Hardware Stream Bit Engine (SBE)**: Single-cycle hardware NRZ, NRZI, and Manchester encoding/decoding |
| **Bit-Stuffing / Unstuffing** | ❌ None (Impossible at line rate without halting execution) | ❌ None (Multi-cycle branch & shift penalty) | ✅ **Hardware Bit-Stuffing Unit (BSU)**: Autonomous USB (6 ones) and CAN (5 identical) bit-stuffing with pipeline stall injection |
| **Error Checking (CRC / Parity)** | ❌ None (Must offload raw words to host CPU) | ⚠️ Software CRC loop (~15-30 cycles/byte) | ✅ **Multi-Poly Streaming CRC**: Parallel LFSR coprocessor (CRC-16, CRC-15, CRC-8, CRC-5) with single-cycle `jmp !crc` branch |
| **Protocol Reverse Engineering** | ❌ None (Firmware polling loops introduce ±1 cycle jitter) | ⚠️ Requires dedicated eCAP peripheral | ✅ **Hardware Auto-Baud Unit (ABU)**: 20ns edge timestamping & min-pulse duration tracking for blind protocol discovery |
| **Deadline & Phase Control** | ⚠️ Relative cycle delay `[0..31]` only | ⚠️ 32-bit cycle counter polling | ✅ **Absolute Deadline Scheduler**: 16-bit phase-locked timebase `T` and deadline register `DL` (`time t+N`, `wait time`) |
| **Instruction Memory Scaling** | ❌ Centralized 32-word memory shared across SMs (Wide mux congestion) | ⚠️ Large instruction RAM (8KB–64KB, massive silicon area) | ✅ **Distributed Local Flip-Flop Arrays**: Core 0 (48w), Core 1 (48w), Core 2 (32w) eliminating global multiplexer routing bloat |
| **Host Telemetry & Control** | ⚠️ Memory-mapped APB/AHB registers | ⚠️ Shared memory mailbox | ✅ **Live Telemetry SPI**: High-speed SPI slave streaming interrupt status & FIFO levels concurrently on MISO during command shifts |

```
                                  ================================================
                                              PROTOCOLFORGE ASIC ARCHITECTURE
                                  ================================================

                                +--------------------------------------------------+
                                |              HOST SPI SLAVE INTERFACE            |
                                |  - Mode 0 / clk/4 (12.5 MHz)                     |
                                |  - Full-Duplex Simultaneous MISO Telemetry       |
                                |  - 16-bit Auto-Burst FIFO DMA Streaming          |
                                +--------------------------------------------------+
                                         |                   |                  |
                    +--------------------+                   |                  +--------------------+
                    |                                        |                                       |
     +------------------------------+       +------------------------------+       +------------------------------+
     |       CORE 0 ENGINE          |       |       CORE 1 ENGINE          |       |       CORE 2 ENGINE          |
     | - 48-word Local IMEM         |       | - 48-word Local IMEM         |       | - 32-word Local IMEM         |
     | - 8-deep TX & RX FIFOs       |       | - 8-deep TX & RX FIFOs       |       | - 4-deep TX & RX FIFOs       |
     | - 16.8 Fractional Divider    |       | - 16.8 Fractional Divider    |       | - 16.8 Fractional Divider    |
     | - Stream Bit Engine (SBE)    |       | - Stream Bit Engine (SBE)    |       | - Hardware Auto-Baud (ABU)   |
     | - Bit-Stuffing Unit (BSU)    |       | - Bit-Stuffing Unit (BSU)    |       | - Transition Sniffer Packet  |
     | - Multi-Poly Streaming CRC   |       | - Multi-Poly Streaming CRC   |       | - Multi-Poly Streaming CRC   |
     | - Deadline Scheduler (T, DL) |       | - Deadline Scheduler (T, DL) |       | - Deadline Scheduler (T, DL) |
     +------------------------------+       +------------------------------+       +------------------------------+
                    |                                        |                                       |
                    +----------------------------------------+---------------------------------------+
                                                             |
                                           +------------------------------------+
                                           |     8-BIT INTER-CORE CROSSBAR      |
                                           |  Atomic Flag Set / Clear / Check   |
                                           +------------------------------------+
                                                             |
                                           +------------------------------------+
                                           |    20-BIT GLITCH-FILTERED GPIO     |
                                           | - 8 Dedicated Inputs (ui[7:0])     |
                                           | - 8 Dedicated Outputs (uo[7:0])    |
                                           | - 4 Bidir / Open-Drain (uio[3:0])  |
                                           | - 3-Tap Majority Glitch Filter     |
                                           +------------------------------------+
```

---

## 2. Microarchitecture Details

### 2.1 Core Heterogeneity & Topology
ProtocolForge embeds three independent, deterministic microcoded cores running at 50 MHz:
- **Core 0 (Master Protocol Engine, 48 words IMEM, 8-word FIFOs):** Sized to execute complex multi-state packet protocols such as CAN 2.0 (arbitration, bit-stuffing, CRC-15 calculation, ACK check) or USB Low-Speed (SYNC, PID, token, data, CRC-16, SE0 EOP).
- **Core 1 (Secondary Protocol Engine, 48 words IMEM, 8-word FIFOs):** Enables full-duplex protocol emulation (e.g. concurrent UART TX + RX, full-duplex SPI Master/Slave, or simultaneous I2C Master + Slave).
- **Core 2 (Auxiliary / Reverse-Engineering Sniffer, 32 words IMEM, 4-word FIFOs):** Directly integrated with the **Hardware Auto-Baud Unit (ABU)**. Features streaming transition sniffer mode where edge timestamps `{2'b00, pin_level, delta_t[12:0]}` are pushed directly into the RX FIFO for high-resolution logic analyzer capture.

### 2.2 Datapath & Register Set
Each core contains a 16-bit RISC datapath operating on a unified register architecture:
- `PC` (Program Counter, 6-bit): Indexes the core's local flip-flop instruction array.
- `X` and `Y` (16-bit Scratch Registers): General-purpose scratch registers supporting post-decrement branching (`jmp x-- label`, `jmp y-- label`).
- `ISR` (Input Shift Register, 16-bit): Receives bits sampled via `in pins, N`. Supports autopush when filled to threshold.
- `OSR` (Output Shift Register, 16-bit): Sources bits driven via `out pins, N`. Supports autopull when emptied.
- `T` (System Timebase, 16-bit): Global free-running 20ns timestamp counter shared across all cores.
- `DL` (Deadline Register, 16-bit): Programmable deadline register for phase-locked scheduling (`time t+N`, `time dl+N`, `wait time`).
- `CAPTURE` (Edge Timestamp, 16-bit): Latches the exact value of `T` at the moment of an input transition.
- `CRC` (16-bit CRC Accumulator): Real-time output of the multi-polynomial CRC LFSR.

### 2.3 Hardware Accelerators

#### Stream Bit Engine (SBE)
Line coding on traditional microcontrollers wastes vast cycle budgets performing software phase inversion. The ProtocolForge SBE operates transparently in the serialize/deserialize path:
- **NRZ (Mode 0):** Standard non-return-to-zero.
- **NRZI (Mode 1):** Inverted non-return-to-zero (toggle on 0, maintain on 1) used by USB 1.1 and HDLC/SDLC.
- **Manchester (Mode 2):** Phase-split Manchester biphase coding used by 10BASE-T Ethernet, MIL-STD-1553, and RFID. Transmits a deterministic mid-bit edge for clock recovery.

#### Bit-Stuffing Unit (BSU)
Bit stuffing requires inserting extra bits during transmission and stripping them during reception without disrupting the serial data byte boundaries:
- **USB Low-Speed Mode:** Tracks consecutive ones; automatically injects a `0` after six consecutive `1`s on TX. On RX, strips the stuffed zero and triggers a `stuff_error` flag if a seventh `1` is observed.
- **CAN 2.0 Mode:** Tracks identical bit sequences; automatically injects the complementary bit after five identical bits on TX. On RX, discards the stuff bit and asserts error flags upon violation.
- **Pipeline Stall Control:** When stuffing occurs, the BSU automatically stalls the core's OSR shifter for one cycle, ensuring seamless transmission without microcode intervention.

#### Streaming Multi-Polynomial CRC Coprocessor
ProtocolForge includes a parallelized linear-feedback shift register (LFSR) coprocessor that snoops serialized bits in real time:
- **CRC-16 (`0x8005`):** USB Data packets, Modbus, ANSI.
- **CRC-15 (`0x4599`):** CAN 2.0 frames.
- **CRC-8 (`0x31`):** Dallas/Maxim 1-Wire packets, SMBus.
- **CRC-5 (`0x05`):** USB Token packets.
- **Branch on Zero (`jmp !crc target`):** Evaluates whether the accumulated CRC residual is non-zero in a single cycle, allowing instantaneous error frame rejection.

#### Hardware Auto-Baud Unit (ABU) & Transition Sniffer
A major hurdle in hardware reverse engineering is determining the line speed of an unknown target:
- The ABU measures pulse durations on `ui_in[0]` with 20ns resolution (at 50 MHz).
- Continuously maintains the `MIN_PULSE` register (shortest valid pulse width observed, filtering glitches < 40ns).
- In Sniffer mode, packages each edge into a 16-bit packet: `{2'b00, pin_level, delta_t[12:0]}` pushed directly to Core 2's RX FIFO.

---

## 3. Instruction Set Architecture (ISA)

ProtocolForge uses a unified 16-bit instruction word format combining control, side-set, delay, and execution fields:

```
 15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0
+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
|    Opcode |   Sideset / Delay |           Operand             |
+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
```

### Instruction Reference Table

| Opcode | Mnemonic | Syntax | Description |
| :---: | :---: | :--- | :--- |
| `000` | **JMP** | `jmp [cond] <target>` | Branch to address `<target>`. Conditions: unconditional, `!x`, `x--`, `!y`, `y--`, `x!=y`, `pin`, `!crc`, `arb_lost`, `!osre`. |
| `001` | **WAIT** | `wait <pol> <src> <idx>` | Stall until condition met. `<src>`: `gpio`, `pin`, `flag`, `time` (deadline elapsed), `edge` (transition). |
| `010` | **IN** | `in <src>, <count>` | Shift `<count>` bits (1..16) from `<src>` (`pins`, `x`, `y`, `null`, `isr`, `status`) into `ISR`. |
| `011` | **OUT** | `out <dst>, <count>` | Shift `<count>` bits (1..16) from `OSR` to `<dst>` (`pins`, `x`, `y`, `pindirs`, `null`, `exec`). |
| `100` | **PUSH / PULL** | `push [block/ifempty]`<br>`pull [block/iffull]` | Push `ISR` to RX FIFO / Pull TX FIFO into `OSR`. Supports non-blocking or blocking execution. |
| `101` | **MOV** | `mov <dst>, [op] <src>` | Copy data between registers. Supports ALU operations: none, invert (`~`), bit-reverse (`::`), byte-swap (`swap`). |
| `110` | **SET** | `set <dst>, <val>` | Write immediate `<val>` (0..31) to `<dst>` (`pins`, `x`, `y`, `pindirs`, `flag_set`, `flag_clr`). |
| `111` | **TIME** | `time [dl] +<offset>` | Schedule next deadline: `time t+N` sets `DL = T + N`; `time dl+N` sets `DL = DL + N`. |

---

## 4. Pinout & Host Interface

### Pinout Mapping

| Pin Name | Direction | Type | Function |
| :--- | :---: | :---: | :--- |
| `ui_in[0]` | Input | Digital (Filterable) | **GPIO 0**: Auto-Baud Sense / RX Input / Sniffer |
| `ui_in[7:1]` | Input | Digital (Filterable) | **GPIO 1..7**: Dedicated digital inputs |
| `uo_out[7:0]` | Output | Push-Pull | **GPIO 8..15**: Dedicated high-speed outputs |
| `uio[3:0]` | In/Out | Open-Drain / Push-Pull | **GPIO 16..19**: Bidirectional lines with hardware open-drain & collision detect |
| `uio[4]` | Input | SPI Slave | **HOST_CS_N**: Active-Low Host SPI Chip Select |
| `uio[5]` | Input | SPI Slave | **HOST_SCK**: Host SPI Clock (max 12.5 MHz) |
| `uio[6]` | Input | SPI Slave | **HOST_MOSI**: Host SPI Master Out / Slave In |
| `uio[7]` | Output | SPI Slave | **HOST_MISO**: Host SPI Master In / Slave Out & Simultaneous Telemetry |

### Host SPI Protocol & Live MISO Telemetry
The Host SPI interface operates in Mode 0 (CPOL=0, CPHA=0). Every SPI transfer shifts out real-time telemetry on MISO simultaneously with the incoming command:

```
MOSI: [ CMD (8-bit) ] [ ADDR (8-bit) ] [ DATA0 (8-bit) ] [ DATA1 (8-bit) ] ...
MISO: [ TELEMETRY 0 ] [ TELEMETRY 1  ] [ RDATA0 (8-bit) ] [ RDATA1 (8-bit) ] ...

TELEMETRY 0: { Core0_IRQ, Core1_IRQ, Core2_IRQ, Glitch_Event, 4'h5 (Magic) }
TELEMETRY 1: { Core0_RX_Level[3:0], Core1_RX_Level[3:0] }
```

### Global Register Map Summary

| Address | Name | Access | Function |
| :---: | :--- | :---: | :--- |
| `0x00` | `REG_ID` | RO | Device ID (`0xC7`) |
| `0x01` | `REG_VERSION` | RO | Silicon Revision (`0x01`) |
| `0x02` | `REG_NUM_CORES` | RO | Core Count (`0x03`) |
| `0x03` | `REG_ENABLE` | RW | Core Run Enable bitmap (`[2:0]`) |
| `0x04` | `REG_RESTART` | WO | Core Synchronous Reset & FIFO flush |
| `0x05` | `REG_STEP` | WO | Single-step execution strobe |
| `0x06` | `REG_FLAGS` | RW | 8-bit Inter-Core Crossbar Flags |
| `0x07` | `REG_INFILT` | RW | Majority Glitch Filter enable mask (`[11:0]`) |
| `0x08` | `REG_ABU_MIN_L` | RW | Auto-Baud minimum pulse duration low byte |
| `0x09` | `REG_ABU_MIN_H` | RW | Auto-Baud minimum pulse duration high byte |
| `0x0A` | `REG_PIN_OD` | RW | Open-drain mode enable mask for `uio[3:0]` |
| `0x20..0x3B` | `CORE0_REGS` | RW | Core 0 Clock divider, Wrap, Pin config, SBE, BSU, CRC, and FIFO stream ports |
| `0x40..0x5B` | `CORE1_REGS` | RW | Core 1 Control & Register space |
| `0x60..0x7B` | `CORE2_REGS` | RW | Core 2 Control & Register space |

---

## 5. Supported Protocols & Firmware Case Studies

The `programs/` directory contains 12 complete, production-grade protocol implementations:

1. **UART (TX & RX, 8N1):** Configurable baud up to 25 Mbaud. Receiver uses 3-tap start bit detection, center sampling, and framing error rejection.
2. **SPI Master & Slave:** Full support for CPOL=0/1 and CPHA=0/1 up to 25 Mbps.
3. **I2C Master & Slave:** Hardware open-drain control on SDA/SCL, arbitration loss detection (`jmp arb_lost`), and automatic SCL clock stretching stall.
4. **CAN Bus 2.0A/B:** Transmits standard and extended frames with hardware 5-identical bit-stuffing, real-time CRC-15 generation, and arbitration loss abort.
5. **Low-Speed USB 1.1 (1.5 Mbps):** End-to-end packet transmission with NRZI encoding, 6-ones bit-stuffing, hardware CRC-16, and single-ended zero (SE0) EOP signaling.
6. **10BASE-T Ethernet (Manchester):** 10 Mbps Manchester biphase encoding with deterministic mid-bit clock transition synthesis.
7. **Dallas 1-Wire:** Microsecond-accurate reset pulse, presence detection pulse, read/write timeslots, and streaming CRC-8 validation.
8. **WS2812B RGB LED:** Precision 800 kHz driver meeting strict asymmetric T0H/T0L/T1H/T1L pulse duration requirements.
9. **Logic Analyzer / Sniffer:** Core 2 timestamp streaming mode capturing external digital transitions directly into host DMA.

---

## 6. Verification Methodology & Formal Proofs

ProtocolForge was verified using an end-to-end multi-tiered verification strategy designed to exceed industrial ASIC tapeout criteria:

### 6.1 Formal Verification (Yosys SAT / BMC Engine)
Three critical hardware blocks were formally proved using Bounded Model Checking (BMC) with zero violations:
1. **FIFO Correctness (`forge_fifo_formal`):** Proved FIFO ordering, absence of underflow, and absence of overflow under all arbitrary push/pop sequences (24 cycles bounded proof).
2. **SBE/BSU Protocol Invariants (`forge_sbe_bsu_formal`):** Formally proved that CAN output never exceeds 5 identical bits and USB output never exceeds 6 consecutive ones regardless of input data patterns.
3. **Deadline Scheduler Arithmetic (`forge_scheduler_formal`):** Proved wrap-around safety and deadline monotonicity across 16-bit integer arithmetic boundaries.

### 6.2 Constrained-Random Differential Co-Simulation
- A cycle-accurate Python golden simulator (`tools/forge_sim.py`) was co-simulated step-by-step against the synthesizable Verilog RTL using Cocotb.
- Verified register states (`PC`, `X`, `Y`, `ISR`, `OSR`, `T`) and ALU operations (`~`, `::`, `swap`) under randomized instruction streams.

### 6.3 Comprehensive Integration Testbench
- **Native Verilog Testbench (`test/tb_protocolforge.v`):** 13/13 comprehensive integration tests passing (Global registers, SPI telemetry, IMEM flashing, GPIO driving, ABU pulse measurement, CRC-15/16 LFSRs, SBE/BSU line coding, and inter-core flags).
- **Cocotb Test Suite (`test/test.py`):** 9/9 asynchronous integration tests passing (SPI register access, IMEM loading, full-duplex UART loopback, SPI master waveforms, CAN bit-stuffing + CRC, USB NRZI, 10BASE-T Manchester, ABU auto-baud measurement, and differential co-simulation).
- **Assembler & Toolchain Unit Tests (`tools/test_forge_asm.py`, `test/test_sim.py`):** 41/41 unit tests passing.

---

## 7. How to Test & Verification Reproduction Guide

### Prerequisites
- Python 3.10+ with `cocotb`, `pytest`, and `find-libpython`.
- Icarus Verilog (`iverilog` v12+) and `yosys` (v0.30+).

### Running Verification Commands

```bash
# 1. Run Python Assembler and Simulator Unit Tests (41 tests)
pytest tools/test_forge_asm.py test/test_sim.py

# 2. Run Comprehensive Native Verilog Testbench (13 tests)
iverilog -o sim_native -g2012 -I src src/project.v src/forge_core.v src/forge_fifo.v \
         src/forge_crc.v src/forge_sbe_bsu.v src/forge_abu.v src/forge_spi.v test/tb_protocolforge.v
./sim_native

# 3. Run Full Cocotb Simulation Test Suite (9 tests)
cd test
make

# 4. Run Formal BMC Verification (3 proofs)
yosys -s formal/check.ys

# 5. Run Technology Synthesis and Gate Count Report
yosys -p "read_verilog -I src src/project.v src/forge_core.v src/forge_fifo.v \
          src/forge_crc.v src/forge_sbe_bsu.v src/forge_abu.v src/forge_spi.v; \
          synth -top tt_um_maddox_protocolforge; stat"
```

---

## 8. External Hardware Requirements
ProtocolForge is completely self-contained and requires only standard digital I/O for operation:
- **Host Controller:** Any microcontroller or FPGA with an SPI Master port (SCK ≤ 12.5 MHz) connected to `uio[7:4]`.
- **Target Devices (Optional):** UART transceivers, I2C sensors, SPI peripherals, CAN transceivers (e.g. MCP2551), USB Type-A female breakout (D+/D-), or 10BASE-T Ethernet PHY / magnetics.
- **Tiny Tapeout Demo Board:** Fully compatible with standard RP2040 host firmware on the Tiny Tapeout demo carrier board.
