# Jane Street Protocol Emulator ASIC Competition

Source post: [Can you design a chip? Announcing the protocol emulator ASIC competition](https://blog.janestreet.com/protocol-emulator-asic-competition/) — Jane Street Blog, Sep 10, 2026, by Benjamin Devlin and Anish Singhani.

This is a follow-up to Jane Street's earlier puzzle, [Can you reverse engineer an ASIC?](https://blog.janestreet.com/can-you-reverse-engineer-an-asic/), where participants reverse engineered a chip from nothing but its layout. That competition is closed; results and writeups were teased as "coming soon" as of this post.

This time, instead of reverse engineering a chip, participants design one, and Jane Street will pay to fabricate their favorite entries.

## The Challenge

Design an **open-source, general-purpose protocol emulator ASIC**.

Hardware protocols like UART, SPI, and I2C are simple enough that people routinely "bit-bang" them: toggling pins from software with careful timing instead of using a dedicated peripheral. A protocol emulator is a small chip purpose-built to do exactly that — a tiny CPU with an instruction set designed for reading pins, writing pins, counting cycles, and hitting timing precisely enough to implement a real protocol in firmware rather than fixed logic. This is a useful tool for hardware debugging and reverse engineering, which is a significant part of what Jane Street's hardware team does.

**The hard part is flexibility.** The goal isn't to slap a UART block, an SPI block, and an I2C block onto one die and call it done. The chip should be reprogrammable enough to support new protocols *after* fabrication, within its timing and I/O constraints. Jane Street points to two existing designs for inspiration:

- **PIO state machines on the RP2040** (Raspberry Pi's microcontroller) — small programmable I/O state machines that can be scripted to implement arbitrary bit-level protocols.
- **PRU cores on TI's Sitara parts** — Programmable Realtime Units, tiny co-processors on TI Sitara SoCs used for deterministic, real-time I/O tasks.

Entrants are asked to consider what they'd do differently from these.

### Scope

- **Start with:** UART, SPI, and I2C.
- **Stretch goals:** low-speed USB and 10Mbit Ethernet.
- **Other protocols to consider:** JTAG, SWD, PS/2, CAN bus.
- If you have access to an FPGA, test your RTL there before running the ASIC flow.
- Show Jane Street anything else your architecture makes possible that they haven't thought of.

### On verification

Jane Street notes that internally they use **[Hardcaml](https://hardcaml.org/)** (their open-source OCaml hardware design library) to generate RTL for their FPGA and ASIC designs, and they're excited to see whatever languages and verification techniques entrants bring — formal methods, random constrained testing, AI-assisted verification, etc. They explicitly call out that as AI-assisted chip design becomes more common, verification is expected to become an increasingly important part of the ASIC design flow. Judging emphasizes **unique functionality** and **novel design/verification methodology**, not just a working chip.

## The Rules

| Item | Detail |
|---|---|
| **Process** | IHP's 130nm CMOS5L process, submitted through [Tiny Tapeout](https://www.tinytapeout.com/) |
| **Starting template** | [CMOS5L Verilog template](https://github.com/TinyTapeout/ttihp-verilog-template/tree/cmos5l) (RTL → GDS flow) |
| **Tile size setting** | Set to `6x4` in `info.yaml` |
| **Max area** | 6×4 tiles (24 tiles). Jane Street is exploring scaling this to 8×4 tiles (~30% more area) and will update the post / email signed-up participants if that becomes available |
| **License** | Must be open source — no need to keep work private until the deadline; entrants are encouraged to build in public |
| **Teams** | Strongly recommended, given the scope of the project |
| **Deadline** | **January 18, 2027** |
| **Prize** | Jane Street pays to tape out the most novel designs on a Tiny Tapeout shuttle, targeting the **March 2027 CMOS5L shuttle** (subject to foundry schedule). Winners get their fabricated chip back, mounted on a dev board, to test in real silicon |
| **Sign-up** | [Google Form](https://docs.google.com/forms/d/e/1FAIpQLSeF7fq756MegxZRQxotBwUJYZx-cL9MrGjxV0z4uD_J0sADxQ/viewform) — not a commitment, just gets you competition updates. A final submission form will be added to the blog post closer to the deadline |
| **Questions** | asic-competition@janestreet.com |

## How Much Fits?

- A 6×4 allocation = **24 tiles**.
- At roughly 200µm × 150µm per tile, that's about **0.7 mm²** of nominal tile area.
- Rough budgeting rule of thumb: **~1K logic cells per tile**.
- You'll likely need to get creative to fit real functionality in that budget.
- **Instruction memory:** SRAM can be more area-efficient than flip-flops for this. Tiny Tapeout has a reference design: [SRAM example on this process node](https://www.tinytapeout.com/chips/ttihp0p2/tt_um_urish_sram_test).
- **Process tip:** Run synthesis early and check mapped cell area, leaving room for clock-tree buffers and routing. Then run full place-and-route and check timing — a design that looks small enough after synthesis can still be hard to route, or too slow at your target clock frequency.

## Getting Started

Jane Street's suggested on-ramp: **start by getting a UART transmitter out of a pin, then make it programmable.** The [Tiny Tapeout documentation](https://www.tinytapeout.com/) walks through the whole flow end to end, and all the tools are free and open source.

## Deep Dive: Linked Resources

### Tiny Tapeout ([tinytapeout.com](https://www.tinytapeout.com/))

Tiny Tapeout is an educational/community platform ("proud winners" of a European Open Source Academy award) that makes it dramatically cheaper and easier to get a digital or analog design fabricated as a real chip, by pooling many small designs ("tiles") onto a shared silicon shuttle. Highlights relevant to this competition:

- **Submission templates** (GitHub, under the TinyTapeout org): IHP Wokwi template, **IHP Verilog template** (used here, on the `cmos5l` branch), SKY Wokwi/Verilog/Analog templates, GF Wokwi/Verilog templates.
- **Digital Design Guide** — lessons on logic gates, flip-flops, and puzzle-style intros (full adder, padlock, UART) for people new to digital design, plus a Wokwi-based browser IDE for drag-and-drop / HDL design.
- **"How do semiconductors work?" (SiliWiz)** — an interactive intro to semiconductor physics: resistors, parasitics, voltage dividers, capacitors, NMOS/PMOS transistors, logic inverters, CMOS inverters — drawn from first principles.
- **Making ASICs** and **Working with HDLs** guides — including an "Important!" primer, FPGA-vs-ASIC differences, HDL resources/templates, and how to test your design.
- **Tech specs** pages for clock, GPIO pins, analog specs, memory, pinouts, and PCB revisions — the electrical/physical constraints a submission must respect.
- **Past/ongoing shuttles**: e.g. Tiny Tapeout IHP 26a/26b, SKY 26a/b/c, SKY 25a/b, IHP 25a/b, GF 26a/b, CAD 25a, and the numbered TT06–TT09 shuttles, each with submitted chips browsable online.
- **Competitions** section — this Jane Street challenge sits alongside other Tiny Tapeout competitions like the TTSKY26a Demoscene Competition and the "Crowd Sourced RISC-V Peripheral" competition (with published winners).
- **Guides** — practical walkthroughs: demoboard quickstarts, local hardening (running the physical-design flow yourself instead of via CI), testing with an Analog Discovery, flashing pmods, laying out standard cells with Magic VLSI, and a full two-track Workshop / Advanced Workshop (draw a MOSFET → simulate a gate → generate GDS → submit → activate & test).
- **Submission mechanics**: pricing depends on design size, chosen shuttle, and digital vs. analog; a [live calculator](https://app.tinytapeout.com/calculator) estimates cost; early-bird pricing exists for individuals; submissions go through [app.tinytapeout.com](https://app.tinytapeout.com/projects/create) and are governed by Tiny Tapeout's [terms of service](https://www.tinytapeout.com/terms/).
- Support channels: [FAQ](https://www.tinytapeout.com/faq/) and a [Discord server](https://discord.gg/qZHPrPsmt6).
- Background reading: a TechRxiv paper, ["Tiny Tapeout: A Shared Silicon Tapeout Platform Accessible to Everyone."](https://www.techrxiv.org/users/799365/articles/1165896-tiny-tapeout-a-shared-silicon-tapeout-platform-accessible-to-everyone)

### CMOS5L Verilog Template ([github.com/TinyTapeout/ttihp-verilog-template](https://github.com/TinyTapeout/ttihp-verilog-template/tree/cmos5l), `cmos5l` branch)

This is the starting scaffold entrants are told to use. Based on descriptions from teams already building on it (see "Community Activity" below), the template provides:

- A skeleton Verilog project (`src/project.v` placeholder top module) wired up to Tiny Tapeout's standard I/O harness.
- `info.yaml`, where you declare project metadata and — critically for this competition — the **tile allocation** (set to `6x4`).
- `docs/info.md` for the project's public-facing description/datasheet.
- A GitHub Actions-driven flow: on every push, CI runs simulation/lint, and (usually as a manual/heavier step) the full **LibreLane** RTL-to-GDS physical design flow (synthesis → place & route → DRC/LVS precheck → gate-level simulation) targeting the IHP PDK.
- A `test/` directory for adapting the testbench to your design (commonly done with `cocotb` in the community projects observed).

### IHP 130nm CMOS5L Process

The target silicon process is IHP Microelectronics' 130nm **SG13G2/CMOS5L** open-source PDK (process design kit) — one of the open PDKs Tiny Tapeout supports alongside SkyWater (SKY) and GlobalFoundries (GF) processes. It's a real, foundry-fabricated CMOS process, not a simulation-only target.

### Hardcaml ([hardcaml.org](https://hardcaml.org/))

Jane Street's own open-source hardware description language, referenced in the post as what they use internally to generate RTL for FPGA and ASIC designs (entrants aren't required to use it, but it's offered as inspiration/context). Key points from the Hardcaml site:

- **What it is:** An OCaml library/HDL (not a High-Level Synthesis tool) for designing, simulating, and formally verifying hardware entirely within OCaml.
- **Design goals:** productive abstractions for rapid circuit design; rigorous validation via fast native simulation and formal verification; seamless integration between hardware accelerators and software drivers; full low-level control for performance.
- **Four pillars:**
  - *Design* — OCaml's higher-order functions (lists, maps) to programmatically generate logic; a strong static type system for safety with zero runtime overhead; a functor system enabling parametrized, reusable hardware modules; ability to build custom embedded DSLs.
  - *Verification* — simulation runs directly in the OCaml runtime so testbenches get the full power of a general-purpose language; integrates with [Quickcheck](https://blog.janestreet.com/quickcheck-for-core/) for constrained-random testing; inline ASCII-waveform expect-tests; standard waveform viewer integration; formal verification via SAT solver integration.
  - *Optimization* — full control over every register/wire, predictable generated RTL mappable back to source, functor-based design-space exploration.
  - *Integration* — hardware/software boundary drivers written in the same language as the RTL, backed by Jane Street's [OxCaml](https://oxcaml.org/) compiler for performance; hardware/software interfaces generated together so they can't drift out of sync.
- **Availability:** open source, installable via `opam`; comes with PPX syntax extensions, an interactive waveform viewer, and editor tooling (Dune, Merlin, OCaml-LSP).
- **Ecosystem highlights:** [Advent of FPGA 2025](https://blog.janestreet.com/advent-of-fpga-challenge-2025-results/) drew 150+ Hardcaml-based community submissions; [Hardcaml ZPrize](https://zprize.hardcaml.com/) (award-winning MSM/NTT implementation); [Hardcaml Hobby Boards](https://github.com/janestreet/hardcaml_hobby_boards_kernel) for Xilinx board integration; an ["OCaml All the Way Down"](https://www.janestreet.com/tech-talks/ocaml-all-the-way-down/) tech talk.
- Docs live at [docs.hardcaml.org](https://docs.hardcaml.org); a getting-started guide is at [hardcaml.org/getting-started](https://hardcaml.org/getting-started).

### Sign-up Form

A [Google Form](https://docs.google.com/forms/d/e/1FAIpQLSeF7fq756MegxZRQxotBwUJYZx-cL9MrGjxV0z4uD_J0sADxQ/viewform) to receive competition updates (tapeout template changes, deadline reminders, and the eventual final submission link). Explicitly **not** a commitment to participate.

### Hardware at Jane Street

The post closes by pointing at Jane Street's hardware team, which designs FPGAs and ASICs for low-latency trading systems, and links to:

- [Hardware internship openings](https://www.janestreet.com/join-jane-street/position/8624440002/)
- [Hardware full-time roles](https://www.janestreet.com/join-jane-street/position/8646893002/)
- [Hardcaml](https://hardcaml.org/) (described above)
- A ["stay in touch"](https://bit.ly/4lEqiqb) link for people interested in Jane Street more generally

**Post authors:**
- **Benjamin Devlin** — hardware developer at Jane Street, originally from New Zealand, PhD in EE from the University of Tokyo. Interests: outdoors, sci-fi, cryptography.
- **Anish Singhani** — FPGA engineer at Jane Street since 2024, Carnegie Mellon graduate. Enjoys building software tools that make hardware design more efficient.

## Community Activity (as of this writing)

Several teams/individuals have already started public repos for this competition (consistent with the "feel free to build in public" rule). Observed approaches vary widely in maturity and architecture — worth a look for inspiration or collaboration, not as authoritative references:

- **BitLoom** ([sheehanmunim/bitloom](https://github.com/sheehanmunim/bitloom)) — most fleshed-out of those found: four programmable I/O state machines (PIO-style, with "deadline scheduling") that bit-bang protocols from firmware with cycle-exact timing. Includes an assembler, a `cocotb`-based host driver, Python models of UART/SPI/I2C/WS2812/Manchester encoding for test, bounded formal checks via Yosys SAT, and a MicroPython driver for the Tiny Tapeout demo board. CI runs both the test suite and the full Tiny Tapeout LibreLane GDS flow on every push. Apache-2.0 licensed.
- **sg13cmos5l-protocol-emulator** ([2AMLogic/sg13cmos5l-protocol-emulator](https://github.com/2AMLogic/sg13cmos5l-protocol-emulator)) — an entry described as being designed by AI agents driving the open-source flow (cocotb + Icarus for verification, Yosys + OpenROAD for implementation). As of writing it's in a "specification phase" with an open design-decision-record issue thread discussing ISA ratification, timing model, and firmware toolchain choices.
- **protocol-emulator** ([pfernandez35/protocol-emulator](https://github.com/pfernandez35/protocol-emulator)) — early-stage; a tiny programmable core targeting UART/SPI/I2C bit-banging.
- **janestreet.asic.protocol-emulator** ([mtanneer/janestreet.asic.protocol-emulator](https://github.com/mtanneer/janestreet.asic.protocol-emulator)) — scaffold stage, notable for splitting CI into a fast lint/sim/synth-check workflow (custom slim Docker image) versus a slow, manually-triggered full LibreLane GDS build.
- **ProtocolGremlin** ([Vedant817/ProtocolGremlin](https://github.com/Vedant817/ProtocolGremlin)) — early-stage, references a "PROJECT_MASTER_PLAN.md" for the full brief and roadmap.
- **jane-street-asic-lab** ([raphaelroshan/jane-street-asic-lab](https://github.com/raphaelroshan/jane-street-asic-lab)) — explicitly framed as a "learning lab": a simplified Lab 1 design (a serially-programmable timing engine) meant to teach clock-by-clock reasoning before attempting the real competition architecture. Notably keeps the tile allocation at 1×1 rather than the competition's 6×4 until template issues are resolved, and deliberately keeps human judgment (protocol timing, ISA decisions, CDC, area/timing tradeoffs) out of automation.
- **protocol-emulator-tt** ([PankajNair/protocol-emulator-tt](https://github.com/PankajNair/protocol-emulator-tt)) — scaffolded submission template, ISA/CPU design not yet started as of writing.

## Quick Reference: All Links

| Resource | URL |
|---|---|
| Competition announcement post | https://blog.janestreet.com/protocol-emulator-asic-competition/ |
| Previous puzzle (reverse engineering) | https://blog.janestreet.com/can-you-reverse-engineer-an-asic/ |
| Tiny Tapeout homepage | https://www.tinytapeout.com/ |
| CMOS5L Verilog template (cmos5l branch) | https://github.com/TinyTapeout/ttihp-verilog-template/tree/cmos5l |
| SRAM reference design | https://www.tinytapeout.com/chips/ttihp0p2/tt_um_urish_sram_test |
| Tiny Tapeout documentation hub | https://www.tinytapeout.com/ |
| Sign-up form | https://docs.google.com/forms/d/e/1FAIpQLSeF7fq756MegxZRQxotBwUJYZx-cL9MrGjxV0z4uD_J0sADxQ/viewform |
| Hardcaml | https://hardcaml.org/ |
| Hardcaml docs | https://docs.hardcaml.org |
| Hardware internships | https://www.janestreet.com/join-jane-street/position/8624440002/ |
| Hardware full-time roles | https://www.janestreet.com/join-jane-street/position/8646893002/ |
| "Stay in touch" | https://bit.ly/4lEqiqb |
| Competition contact email | asic-competition@janestreet.com |

## TL;DR Timeline

- **Now – Jan 18, 2027:** Design, verify, and open-source your protocol emulator ASIC on the Tiny Tapeout CMOS5L template (6×4 tiles, possibly 8×4 if Jane Street expands the limit).
- **Jan 18, 2027:** Submission deadline.
- **March 2027 (target):** Winning designs taped out on the CMOS5L shuttle, subject to foundry scheduling.
- **After fabrication:** Winners receive their chip mounted on a dev board to test in real silicon.
