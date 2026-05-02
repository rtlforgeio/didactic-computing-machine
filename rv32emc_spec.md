# RV32EMC Core — Full RTL Specification
**Version:** 1.0.0  
**ISA:** RISC-V RV32E + M (MUL/DIV) + C (Compressed)  
**Bus:** AHB-Lite (IHI0033)  
**Debug:** RISC-V Debug Spec 0.13.2  

---

## 1. Overview

The RV32EMC is a small, 3-stage in-order pipelined 32-bit RISC-V processor core targeting area-constrained SoCs such as IoT nodes, wearable MCUs, and deeply embedded controllers. It implements:

- **RV32E** — 16 general-purpose registers (x0–x15), 32-bit integers
- **M extension** — Hardware multiply (2-cycle) and divide (32-cycle)
- **C extension** — 16-bit compressed instructions (decompressed in IF stage)
- **M-mode only** — Machine-mode privileged architecture
- **RISC-V Debug Spec 0.13.2** — Full halt/resume/step/abstract-command debug
- **JTAG TAP** — IEEE 1149.1 with RISC-V DTM (IDCODE, DTMCS, DMI registers)

---

## 2. Microarchitecture

### 2.1 Pipeline Stages

```
  Clock →      1           2           3
  ┌──────────────────────────────────────────┐
  │  IF       ID/EX       MEM/WB             │
  │ ┌──────┐ ┌────────┐  ┌────────┐          │
  │ │Fetch │→│Decode  │→ │LSU     │          │
  │ │BPU   │ │RF Read │  │CSR R/W │          │
  │ │C-Dec │ │ALU     │  │Writebck│          │
  │ │PC mux│ │MUL/DIV │  │Trap    │          │
  │ └──────┘ └────────┘  └────────┘          │
  └──────────────────────────────────────────┘
```

| Stage    | Function |
|----------|----------|
| **IF**   | Fetch instruction via AHB-Lite iBus. Decompress RVC → RVI. BPU predicts branch targets. |
| **ID/EX**| Decode opcode, read register file, compute ALU result, evaluate branch condition, kick off MUL/DIV. |
| **MEM/WB**| Execute load/store via AHB-Lite dBus. Read/write CSRs. Select writeback data. Write register file. Handle traps. |

### 2.2 Pipeline Hazards and Forwarding

| Hazard Type | Resolution |
|-------------|-----------|
| Load-use (LW → dependent) | 1-cycle interlock stall |
| MUL latency (2 cycles) | Stall until `mul_valid` |
| DIV latency (32 cycles) | Stall until `div_valid` |
| RAW (non-load) EX→ID | Forward EX/MEM `alu_result` to ID/EX inputs |
| RAW (non-load) MEM→ID | Forward MEM/WB `alu_result` or `mem_rdata` |
| iBus not ready | Stall all stages |
| dBus not ready | Stall all stages |

### 2.3 Branch Prediction

- **2-bit saturating counter BPU** with configurable BTB depth (default 256 entries)
- Predicted PC presented to IF before decode; flushed on misprediction
- Reduces branch penalty from 2 cycles to 0 cycles on correct predictions
- BTB updated at end of EX stage (after branch resolution)

---

## 3. Top-Level Port List

### 3.1 `rv32emc_core` — Core Module

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `clk_i` | in | 1 | Core clock |
| `rst_ni` | in | 1 | Async active-low reset |
| `ibus_haddr_o` | out | 32 | iBus AHB address |
| `ibus_htrans_o` | out | 2 | iBus transfer type (IDLE/NONSEQ) |
| `ibus_hsize_o` | out | 3 | Always WORD (3'b010) |
| `ibus_hburst_o` | out | 3 | Always SINGLE |
| `ibus_hprot_o` | out | 4 | 4'b0001 = privileged opcode |
| `ibus_hwrite_o` | out | 1 | Always 0 |
| `ibus_hrdata_i` | in | 32 | Instruction data |
| `ibus_hready_i` | in | 1 | Slave ready |
| `ibus_hresp_i` | in | 1 | Bus error |
| `dbus_haddr_o` | out | 32 | dBus AHB address |
| `dbus_htrans_o` | out | 2 | Transfer type |
| `dbus_hsize_o` | out | 3 | BYTE/HALF/WORD |
| `dbus_hburst_o` | out | 3 | Always SINGLE |
| `dbus_hprot_o` | out | 4 | 4'b0011 = privileged data |
| `dbus_hwrite_o` | out | 1 | 1=write 0=read |
| `dbus_hwdata_o` | out | 32 | Store data (valid 1 cycle after addr) |
| `dbus_hrdata_i` | in | 32 | Load data |
| `dbus_hready_i` | in | 1 | Slave ready |
| `dbus_hresp_i` | in | 1 | Bus error |
| `irq_i` | in | 16 | External interrupt lines (active-high, level) |
| `timer_irq_i` | in | 1 | Machine timer interrupt (from CLINT) |
| `soft_irq_i` | in | 1 | Machine software interrupt (from CLINT) |
| `nmi_i` | in | 1 | Non-maskable interrupt |
| `debug_req_i` | in | 1 | Halt request from DM |
| `debug_halted_o` | out | 1 | Core is in Debug Mode |
| `debug_resume_i` | in | 1 | Resume request |
| `debug_step_i` | in | 1 | Single-step enable |
| `dbg_reg_rd_i` | in | 1 | Abstract: read GPR |
| `dbg_reg_wr_i` | in | 1 | Abstract: write GPR |
| `dbg_reg_addr_i` | in | 5 | GPR index |
| `dbg_reg_wdata_i` | in | 32 | Abstract write data |
| `dbg_reg_rdata_o` | out | 32 | Abstract read data |
| `dbg_progbuf_i` | in | 64 | 2-instruction program buffer |
| `dbg_progbuf_exec_i` | in | 1 | Execute program buffer |
| `core_sleep_o` | out | 1 | WFI sleep indicator |
| `core_pc_o` | out | 32 | Current fetch PC |

### 3.2 `rv32emc_soc_wrapper` — Top-Level Integration

Adds the following flat ports on top of core ports:

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `por_ni` | in | 1 | Power-on-reset |
| `jtag_tck_i` | in | 1 | JTAG test clock |
| `jtag_tms_i` | in | 1 | JTAG test mode select |
| `jtag_tdi_i` | in | 1 | JTAG test data in |
| `jtag_trst_ni` | in | 1 | JTAG async reset |
| `jtag_tdo_o` | out | 1 | JTAG test data out |
| `jtag_tdo_oe_o` | out | 1 | TDO output enable (tri-state) |

---

## 4. ISA Support

### 4.1 RV32E Base (all 47 instructions)

| Group | Instructions |
|-------|-------------|
| Integer ALU | ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU |
| Immediate ALU | ADDI, ANDI, ORI, XORI, SLLI, SRLI, SRAI, SLTI, SLTIU |
| Upper immediate | LUI, AUIPC |
| Jump | JAL, JALR |
| Branch | BEQ, BNE, BLT, BGE, BLTU, BGEU |
| Load | LB, LH, LW, LBU, LHU |
| Store | SB, SH, SW |
| System | ECALL, EBREAK, MRET, WFI, FENCE |
| CSR | CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI |

### 4.2 M Extension (8 instructions)

| Instruction | Operation | Latency |
|-------------|-----------|---------|
| MUL rd,rs1,rs2 | rd = (rs1×rs2)[31:0] | 2 cycles |
| MULH rd,rs1,rs2 | rd = (signed×signed)[63:32] | 2 cycles |
| MULHSU rd,rs1,rs2 | rd = (signed×unsigned)[63:32] | 2 cycles |
| MULHU rd,rs1,rs2 | rd = (unsigned×unsigned)[63:32] | 2 cycles |
| DIV rd,rs1,rs2 | rd = signed quotient | 32 cycles |
| DIVU rd,rs1,rs2 | rd = unsigned quotient | 32 cycles |
| REM rd,rs1,rs2 | rd = signed remainder | 32 cycles |
| REMU rd,rs1,rs2 | rd = unsigned remainder | 32 cycles |

All RISC-V corner cases handled: division by zero, signed overflow (INT_MIN / -1).

### 4.3 C Extension (all Quadrant 0/1/2 instructions for RV32)

Decompressed in IF stage — zero execution penalty. All standard RVC instructions supported including C.ADDI4SPN, C.LW, C.SW, C.JAL, C.J, C.BEQZ, C.BNEZ, C.LI, C.LUI, C.ADDI16SP, C.SLLI, C.LWSP, C.SWSP, C.JR, C.JALR, C.MV, C.ADD, C.EBREAK, C.SUB, C.XOR, C.OR, C.AND, C.SRLI, C.SRAI, C.ANDI.

---

## 5. CSR Register Map (M-mode only)

| Address | Name | Access | Description |
|---------|------|--------|-------------|
| 0xF11 | mvendorid | RO | Vendor ID (parameter) |
| 0xF12 | marchid | RO | Architecture ID |
| 0xF13 | mimpid | RO | Implementation ID |
| 0xF14 | mhartid | RO | Hardware thread ID |
| 0x300 | mstatus | RW | MIE[3], MPIE[7] implemented |
| 0x301 | misa | RO | MXL=1, E+M+C bits set |
| 0x304 | mie | RW | MSIE[3], MTIE[7], MEIE[11] |
| 0x305 | mtvec | RW | Trap vector: direct[1:0=00] or vectored[1:0=01] |
| 0x340 | mscratch | RW | Scratch register |
| 0x341 | mepc | RW | Exception PC |
| 0x342 | mcause | RW | Trap cause (interrupt[31] + code[30:0]) |
| 0x343 | mtval | RW | Trap value (bad address / instruction) |
| 0x344 | mip | RO | Pending interrupts (read-only shadow) |
| 0xB00 | mcycle | RW | Cycle counter [31:0] |
| 0xB80 | mcycleh | RW | Cycle counter [63:32] |
| 0xB02 | minstret | RW | Retired instruction counter [31:0] |
| 0xB82 | minstreth | RW | Retired instruction counter [63:32] |
| 0x7B0 | dcsr | RW | Debug control/status (Debug Mode only) |
| 0x7B1 | dpc | RW | Debug PC (Debug Mode only) |
| 0x7B2 | dscratch0 | RW | Debug scratch 0 |
| 0x7B3 | dscratch1 | RW | Debug scratch 1 |

---

## 6. Exception and Interrupt Handling

### 6.1 Exception Priority (highest → lowest)

1. NMI (Non-maskable interrupt)
2. Instruction fetch access fault
3. Illegal instruction
4. Load address misalignment
5. Store address misalignment
6. Load access fault (bus error)
7. Store access fault (bus error)
8. ECALL from M-mode
9. External interrupts (gated by `mstatus.MIE`)

### 6.2 Trap Entry Sequence

1. `mepc` ← faulting / interrupted PC
2. `mcause` ← cause code
3. `mtval` ← bad address or instruction word
4. `mstatus.MPIE` ← `mstatus.MIE`; `mstatus.MIE` ← 0
5. PC ← `mtvec` (direct) or `mtvec + 4×cause` (vectored)

### 6.3 MRET Sequence

1. `mstatus.MIE` ← `mstatus.MPIE`; `mstatus.MPIE` ← 1
2. PC ← `mepc`

---

## 7. Debug Architecture

### 7.1 Debug Spec 0.13 Compliance

| Feature | Support |
|---------|---------|
| Halt / Resume | ✓ |
| Single-step | ✓ |
| Abstract: Access Register (GPR x0–x15) | ✓ |
| Abstract: Access Memory (via progbuf) | ✓ |
| Program Buffer (2 instructions) | ✓ |
| System Bus Access | — (external) |
| Trigger Module | — (future) |

### 7.2 DMI Register Map

| DMI Addr | Register | Description |
|----------|----------|-------------|
| 0x04 | data0 | Abstract data register |
| 0x10 | dmcontrol | DM control (haltreq, resumereq, dmactive) |
| 0x11 | dmstatus | Core status (allhalted, allrunning, version=2) |
| 0x12 | hartinfo | Hart info (nscratch=2, datacount=1) |
| 0x16 | abstractcs | Command status (cmderr, busy, progbufsize=2) |
| 0x17 | command | Abstract command trigger |
| 0x20 | progbuf0 | Program buffer word 0 |
| 0x21 | progbuf1 | Program buffer word 1 |

### 7.3 JTAG TAP

- IR length: 5 bits
- Supported instructions: IDCODE (0x01), DTMCS (0x10), DMI (0x11), BYPASS (0x1F)
- DMI shift register: 41 bits = 7 (addr) + 32 (data) + 2 (op)
- TCK → sys_clk CDC: 2-FF synchronizer on request pulse

---

## 8. CLINT Memory Map

Base address: `0xE000_0000` (configurable via address decoder)

| Offset | Register | Width | Description |
|--------|----------|-------|-------------|
| 0x0000 | msip | 32 | Software interrupt pending bit[0] |
| 0x4000 | mtimecmp_lo | 32 | Timer compare value [31:0] |
| 0x4004 | mtimecmp_hi | 32 | Timer compare value [63:32] |
| 0xBFF8 | mtime_lo | 32 | Real-time counter [31:0] |
| 0xBFFC | mtime_hi | 32 | Real-time counter [63:32] |

Timer interrupt fires when `mtime >= mtimecmp` (64-bit comparison).

---

## 9. Physical Implementation Targets

| Parameter | Value |
|-----------|-------|
| Target process | TSMC 28nm HPM / GF 22nm FDX |
| Frequency (28nm) | 200–350 MHz typical |
| Frequency (22nm FDX) | 400–600 MHz |
| Core gate count | ~8K–12K NAND2-eq (RV32EMC, no BPU cache) |
| Core area (28nm) | ~0.08–0.15 mm² |
| Static power (28nm) | ~5–20 µW @ 0.9V |
| Dynamic power (28nm) | ~50–150 µW/MHz @ 0.9V |
| Pipeline depth | 3 stages |
| CPI (typical Dhrystone) | ~1.4 |
| Dhrystone DMIPS/MHz | ~0.9 |

---

## 10. File Structure

```
rv32emc/
├── rtl/
│   ├── rv32emc_pkg.sv          # Types, enums, structs, parameters
│   ├── rv32emc_if.sv           # SystemVerilog interfaces (AHB-Lite, JTAG, IRQ, DBG)
│   ├── rv32emc_core.sv         # Top-level 3-stage core
│   ├── rv32emc_if_stage.sv     # IF stage + C-extension decompressor + BPU
│   ├── rv32emc_decode_exec.sv  # Decode, Execute, ALU, MUL, Register File, Hazard
│   ├── rv32emc_lsu_csr_aux.sv  # LSU, CSR file, DIV, BPU, Trap controller
│   ├── rv32emc_debug_dm.sv     # Debug controller, JTAG TAP, Debug Module
│   └── rv32emc_soc_wrapper.sv  # SoC integration wrapper + CLINT
├── tb/
│   └── rv32emc_tb.sv           # Self-checking testbench (7 test groups)
├── constraints/
│   └── rv32emc_constraints.sdc # (Synthesis constraints template)
└── docs/
    └── rv32emc_spec.md         # This document
```

---

## 11. Verification Plan

| Test | Method | Coverage Target |
|------|--------|----------------|
| ISA compliance | riscv-tests (rv32ei, rv32em, rv32ec) | 100% instruction coverage |
| Pipeline hazards | Directed + random sequences | All forwarding paths |
| Load/Store alignment | Directed tests for all sizes/alignments | All byte lane combinations |
| Interrupt handling | Timer + external IRQ + NMI | All priority combinations |
| Debug | OpenOCD + GDB over JTAG sim model | Halt, step, BP, mem access |
| Corner cases | DIV by zero, MUL overflow, JALR LSB | Per-spec table |
| Formal | SymbiYosys bounded model check (k=20) | Register file liveness, x0=0 |
| Gate-sim | Post-synthesis simulation with SDF | Timing closure check |

---

## 12. Synthesis Constraints (Template)

```tcl
# rv32emc_constraints.sdc
create_clock -name clk_core -period 5.0 [get_ports clk_i]
create_clock -name clk_jtag -period 100.0 [get_ports jtag_tck_i]

# Async reset false path
set_false_path -from [get_ports rst_ni]

# JTAG <-> system clock domain: set as asynchronous
set_clock_groups -asynchronous \
  -group [get_clocks clk_core] \
  -group [get_clocks clk_jtag]

# I/O constraints
set_input_delay  -clock clk_core -max 1.0 [all_inputs]
set_output_delay -clock clk_core -max 1.0 [all_outputs]

# Multicycle path: DIV (32-cycle result)
set_multicycle_path -setup 2 -from [get_cells *u_div*] -to [get_cells *div_result*]
set_multicycle_path -hold  1 -from [get_cells *u_div*] -to [get_cells *div_result*]
```

---

*Specification generated for AI-assisted RTL design platform — RV32EMC v1.0.0*
