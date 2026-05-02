// =============================================================================
// rv32emc_if.sv — SystemVerilog Interfaces
// =============================================================================

`ifndef RV32EMC_IF_SV
`define RV32EMC_IF_SV

import rv32emc_pkg::*;

// -----------------------------------------------------------------------------
// AHB-Lite Master Interface (IEEE 1003.1 / ARM IHI0033)
// Used for both instruction fetch bus and data/LSU bus
// -----------------------------------------------------------------------------
interface ahb_lite_if #(
  parameter int unsigned AW = 32,
  parameter int unsigned DW = 32
)(
  input logic hclk,
  input logic hresetn
);
  // Address/control phase (registered by master)
  logic [AW-1:0]  haddr;    // Transfer address
  htrans_e        htrans;   // Transfer type
  hsize_e         hsize;    // Transfer size
  hburst_e        hburst;   // Burst type (SINGLE for core)
  logic [3:0]     hprot;    // Protection [3]=cacheable [2]=buff [1]=priv [0]=data
  logic           hwrite;   // 1=write 0=read
  logic           hmastlock;// Locked sequence (unused, tie 0)

  // Data phase
  logic [DW-1:0]  hwdata;   // Write data (registered 1 cycle after address)
  logic [DW-1:0]  hrdata;   // Read data from slave
  logic           hready;   // Slave ready / extend transfer
  logic           hresp;    // 0=OKAY 1=ERROR

  // Master port (driven by core)
  modport master (
    input  hclk, hresetn, hrdata, hready, hresp,
    output haddr, htrans, hsize, hburst, hprot, hwrite, hmastlock, hwdata
  );

  // Slave port (driven by memory/peripheral)
  modport slave (
    input  hclk, hresetn, haddr, htrans, hsize, hburst, hprot, hwrite,
           hmastlock, hwdata,
    output hrdata, hready, hresp
  );

  // Monitor port (read-only, for verification)
  modport monitor (
    input  hclk, hresetn, haddr, htrans, hsize, hburst, hprot, hwrite,
           hmastlock, hwdata, hrdata, hready, hresp
  );

endinterface : ahb_lite_if


// -----------------------------------------------------------------------------
// JTAG Interface (IEEE 1149.1 / RISC-V Debug Spec 0.13)
// -----------------------------------------------------------------------------
interface jtag_if;
  logic tck;    // Test clock (from external)
  logic tms;    // Test mode select
  logic tdi;    // Test data in
  logic tdo;    // Test data out (driven by DM)
  logic tdo_oe; // TDO output enable (tri-state control)
  logic trst_n; // Async TAP reset (optional, active-low)

  modport tap_ctrl (
    input  tck, tms, tdi, trst_n,
    output tdo, tdo_oe
  );

  modport host (
    output tck, tms, tdi, trst_n,
    input  tdo
  );

endinterface : jtag_if


// -----------------------------------------------------------------------------
// Interrupt Interface (Platform-Level — 32 external IRQ lines)
// -----------------------------------------------------------------------------
interface irq_if #(parameter int unsigned NUM_IRQ = 32);
  logic [NUM_IRQ-1:0] irq;       // External interrupt lines (level, active-high)
  logic               nmi;       // Non-maskable interrupt
  logic               timer_irq; // Machine timer interrupt (from CLINT)
  logic               soft_irq;  // Machine software interrupt (from CLINT)

  modport core_in (
    input irq, nmi, timer_irq, soft_irq
  );

  modport platform_out (
    output irq, nmi, timer_irq, soft_irq
  );

endinterface : irq_if


// -----------------------------------------------------------------------------
// Core Status/Debug Interface (observation bus for DM)
// -----------------------------------------------------------------------------
interface core_dbg_if;
  // Halt/resume control (driven by Debug Module)
  logic        debug_req;     // Request core to halt
  logic        debug_halted;  // Core has entered Debug Mode
  logic        debug_resume;  // Resume from halt
  // Single-step
  logic        step;          // Execute one instruction then halt
  // Abstract command register access
  logic        reg_rd_en;
  logic        reg_wr_en;
  logic [4:0]  reg_addr;      // Register index (x0–x15 for RV32E)
  logic [31:0] reg_wdata;
  logic [31:0] reg_rdata;
  // Program buffer (2 instructions for abstract mem access)
  logic [63:0] progbuf;       // 2 × 32-bit program buffer
  logic        progbuf_exec;  // Execute program buffer

  modport dm_side (
    output debug_req, debug_resume, step, reg_rd_en, reg_wr_en,
           reg_addr, reg_wdata, progbuf, progbuf_exec,
    input  debug_halted, reg_rdata
  );

  modport core_side (
    input  debug_req, debug_resume, step, reg_rd_en, reg_wr_en,
           reg_addr, reg_wdata, progbuf, progbuf_exec,
    output debug_halted, reg_rdata
  );

endinterface : core_dbg_if

`endif // RV32EMC_IF_SV
