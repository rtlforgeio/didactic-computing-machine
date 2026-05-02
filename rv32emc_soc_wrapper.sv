// =============================================================================
// rv32emc_soc_wrapper.sv — SoC Integration Wrapper
// =============================================================================
// Bundles: core + DM + JTAG TAP + CLINT (timer) into a single synthesisable top
// Provides flat port list suitable for physical design / PAD ring insertion.
// =============================================================================

`include "rv32emc_pkg.sv"
`include "rv32emc_if.sv"

module rv32emc_soc_wrapper #(
  // Core boot address (usually beginning of ROM)
  parameter logic [31:0] BOOT_ADDR       = 32'h0000_0000,
  parameter logic [31:0] MTVEC_RESET_VAL = 32'h0000_0100,
  // Debug
  parameter logic [31:0] JTAG_IDCODE     = 32'h1001_04B3,
  // Features
  parameter bit          ENABLE_MUL      = 1,
  parameter bit          ENABLE_DIV      = 1,
  parameter bit          ENABLE_COMPRESSED = 1,
  parameter bit          ENABLE_DEBUG    = 1,
  parameter bit          ENABLE_BPU      = 1
)(
  // -------------------------------------------------------------------------
  // Clock / Reset
  // -------------------------------------------------------------------------
  input  logic        clk_i,       // System clock (core + DM)
  input  logic        rst_ni,      // Async active-low system reset
  input  logic        por_ni,      // Power-on-reset (held for boot init)

  // -------------------------------------------------------------------------
  // Instruction AHB-Lite (to external I-mem / ROM / I-cache)
  // -------------------------------------------------------------------------
  output logic [31:0] ibus_haddr_o,
  output logic [1:0]  ibus_htrans_o,
  output logic [2:0]  ibus_hsize_o,
  output logic [2:0]  ibus_hburst_o,
  output logic [3:0]  ibus_hprot_o,
  output logic        ibus_hwrite_o,
  output logic [31:0] ibus_hwdata_o,   // Always 0 (read-only bus)
  input  logic [31:0] ibus_hrdata_i,
  input  logic        ibus_hready_i,
  input  logic        ibus_hresp_i,

  // -------------------------------------------------------------------------
  // Data AHB-Lite (to external D-mem / SRAM / peripherals)
  // -------------------------------------------------------------------------
  output logic [31:0] dbus_haddr_o,
  output logic [1:0]  dbus_htrans_o,
  output logic [2:0]  dbus_hsize_o,
  output logic [2:0]  dbus_hburst_o,
  output logic [3:0]  dbus_hprot_o,
  output logic        dbus_hwrite_o,
  output logic [31:0] dbus_hwdata_o,
  input  logic [31:0] dbus_hrdata_i,
  input  logic        dbus_hready_i,
  input  logic        dbus_hresp_i,

  // -------------------------------------------------------------------------
  // Interrupts
  // -------------------------------------------------------------------------
  input  logic [15:0] irq_i,         // Platform-level external IRQs
  input  logic        nmi_i,         // Non-maskable interrupt

  // -------------------------------------------------------------------------
  // JTAG (Debug Transport Module)
  // -------------------------------------------------------------------------
  input  logic        jtag_tck_i,
  input  logic        jtag_tms_i,
  input  logic        jtag_tdi_i,
  input  logic        jtag_trst_ni,
  output logic        jtag_tdo_o,
  output logic        jtag_tdo_oe_o,

  // -------------------------------------------------------------------------
  // Core status (for PMU / power controller)
  // -------------------------------------------------------------------------
  output logic        core_sleep_o,  // WFI sleep indicator
  output logic [31:0] core_pc_o      // Current fetch PC (debug/trace)
);

  import rv32emc_pkg::*;

  // =========================================================================
  // Internal wires
  // =========================================================================

  // CLINT timer wires
  logic        timer_irq_w;
  logic        soft_irq_w;

  // Debug interface
  logic        debug_req_w, debug_halted_w, debug_resume_w, debug_step_w;
  logic        dbg_reg_rd_w, dbg_reg_wr_w;
  logic [4:0]  dbg_reg_addr_w;
  logic [31:0] dbg_reg_wdata_w, dbg_reg_rdata_w;
  logic [63:0] progbuf_w;
  logic        progbuf_exec_w;

  // DMI (between TAP and DM)
  logic        dmi_req_valid_w, dmi_req_ready_w;
  logic [6:0]  dmi_req_addr_w;
  logic [31:0] dmi_req_data_w;
  logic [1:0]  dmi_req_op_w;
  logic        dmi_rsp_valid_w, dmi_rsp_ready_w;
  logic [31:0] dmi_rsp_data_w;
  logic [1:0]  dmi_rsp_op_w;

  // =========================================================================
  // 1. RISC-V Core
  // =========================================================================
  rv32emc_core #(
    .BOOT_ADDR       (BOOT_ADDR),
    .MTVEC_RESET_VAL (MTVEC_RESET_VAL),
    .ENABLE_MUL      (ENABLE_MUL),
    .ENABLE_DIV      (ENABLE_DIV),
    .ENABLE_COMPRESSED(ENABLE_COMPRESSED),
    .ENABLE_DEBUG    (ENABLE_DEBUG),
    .ENABLE_BPU      (ENABLE_BPU)
  ) u_core (
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),
    // iBus
    .ibus_haddr_o     (ibus_haddr_o),
    .ibus_htrans_o    (ibus_htrans_o),
    .ibus_hsize_o     (ibus_hsize_o),
    .ibus_hburst_o    (ibus_hburst_o),
    .ibus_hprot_o     (ibus_hprot_o),
    .ibus_hwrite_o    (ibus_hwrite_o),
    .ibus_hrdata_i    (ibus_hrdata_i),
    .ibus_hready_i    (ibus_hready_i),
    .ibus_hresp_i     (ibus_hresp_i),
    // dBus
    .dbus_haddr_o     (dbus_haddr_o),
    .dbus_htrans_o    (dbus_htrans_o),
    .dbus_hsize_o     (dbus_hsize_o),
    .dbus_hburst_o    (dbus_hburst_o),
    .dbus_hprot_o     (dbus_hprot_o),
    .dbus_hwrite_o    (dbus_hwrite_o),
    .dbus_hwdata_o    (dbus_hwdata_o),
    .dbus_hrdata_i    (dbus_hrdata_i),
    .dbus_hready_i    (dbus_hready_i),
    .dbus_hresp_i     (dbus_hresp_i),
    // Interrupts
    .irq_i            (irq_i),
    .timer_irq_i      (timer_irq_w),
    .soft_irq_i       (soft_irq_w),
    .nmi_i            (nmi_i),
    // Debug
    .debug_req_i      (debug_req_w),
    .debug_halted_o   (debug_halted_w),
    .debug_resume_i   (debug_resume_w),
    .debug_step_i     (debug_step_w),
    .dbg_reg_rd_i     (dbg_reg_rd_w),
    .dbg_reg_wr_i     (dbg_reg_wr_w),
    .dbg_reg_addr_i   (dbg_reg_addr_w),
    .dbg_reg_wdata_i  (dbg_reg_wdata_w),
    .dbg_reg_rdata_o  (dbg_reg_rdata_w),
    .dbg_progbuf_i    (progbuf_w),
    .dbg_progbuf_exec_i(progbuf_exec_w),
    // Status
    .core_sleep_o     (core_sleep_o),
    .core_pc_o        (core_pc_o)
  );

  // =========================================================================
  // 2. CLINT — Core-Local Interrupt Controller
  //    Implements mtime, mtimecmp, msip (mapped at 0xXXXX_0000)
  // =========================================================================
  rv32emc_clint u_clint (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),
    // AHB-Lite slave port (shared dBus — address-decoded externally)
    // Simplified: direct register map
    .haddr_i     (dbus_haddr_o[15:0]),
    .hwrite_i    (dbus_hwrite_o),
    .hwdata_i    (dbus_hwdata_o),
    .hrdata_o    (/* muxed into dbus_hrdata */),
    .hsel_i      (dbus_haddr_o[31:16] == 16'hE000), // CLINT @ 0xE000_0000
    // IRQ outputs
    .timer_irq_o (timer_irq_w),
    .soft_irq_o  (soft_irq_w)
  );

  // =========================================================================
  // 3. Debug Module (DM)
  // =========================================================================
  generate if (ENABLE_DEBUG) begin : gen_dm
    rv32emc_dm u_dm (
      .clk_i            (clk_i),
      .rst_ni           (rst_ni),
      // DMI
      .dmi_req_valid_i  (dmi_req_valid_w),
      .dmi_req_ready_o  (dmi_req_ready_w),
      .dmi_req_addr_i   (dmi_req_addr_w),
      .dmi_req_data_i   (dmi_req_data_w),
      .dmi_req_op_i     (dmi_req_op_w),
      .dmi_rsp_valid_o  (dmi_rsp_valid_w),
      .dmi_rsp_ready_i  (dmi_rsp_ready_w),
      .dmi_rsp_data_o   (dmi_rsp_data_w),
      .dmi_rsp_op_o     (dmi_rsp_op_w),
      // Core debug
      .debug_req_o      (debug_req_w),
      .debug_halted_i   (debug_halted_w),
      .debug_resume_o   (debug_resume_w),
      .debug_step_o     (debug_step_w),
      .dbg_reg_rd_o     (dbg_reg_rd_w),
      .dbg_reg_wr_o     (dbg_reg_wr_w),
      .dbg_reg_addr_o   (dbg_reg_addr_w),
      .dbg_reg_wdata_o  (dbg_reg_wdata_w),
      .dbg_reg_rdata_i  (dbg_reg_rdata_w),
      .progbuf_o        (progbuf_w),
      .progbuf_exec_o   (progbuf_exec_w)
    );

    // =========================================================================
    // 4. JTAG TAP / DTM
    // =========================================================================
    rv32emc_jtag_tap #(.IDCODE_VAL(JTAG_IDCODE)) u_tap (
      .tck_i            (jtag_tck_i),
      .tms_i            (jtag_tms_i),
      .tdi_i            (jtag_tdi_i),
      .trst_ni          (jtag_trst_ni),
      .tdo_o            (jtag_tdo_o),
      .tdo_oe_o         (jtag_tdo_oe_o),
      .sys_clk_i        (clk_i),
      .sys_rst_ni       (rst_ni),
      .dmi_req_valid_o  (dmi_req_valid_w),
      .dmi_req_ready_i  (dmi_req_ready_w),
      .dmi_req_addr_o   (dmi_req_addr_w),
      .dmi_req_data_o   (dmi_req_data_w),
      .dmi_req_op_o     (dmi_req_op_w),
      .dmi_rsp_valid_i  (dmi_rsp_valid_w),
      .dmi_rsp_ready_o  (dmi_rsp_ready_w),
      .dmi_rsp_data_i   (dmi_rsp_data_w),
      .dmi_rsp_op_i     (dmi_rsp_op_w)
    );

  end else begin : gen_no_dm
    assign debug_req_w    = 1'b0;
    assign debug_resume_w = 1'b0;
    assign debug_step_w   = 1'b0;
    assign dbg_reg_rd_w   = 1'b0;
    assign dbg_reg_wr_w   = 1'b0;
    assign dbg_reg_addr_w = '0;
    assign dbg_reg_wdata_w= '0;
    assign progbuf_w      = '0;
    assign progbuf_exec_w = 1'b0;
    assign jtag_tdo_o     = 1'b1;
    assign jtag_tdo_oe_o  = 1'b0;
  end endgenerate

  // iBus write data always 0 (read-only)
  assign ibus_hwdata_o = '0;

endmodule : rv32emc_soc_wrapper


// =============================================================================
// rv32emc_clint.sv — Core-Local Interrupt Controller
// =============================================================================
// Memory map (base configurable):
//   +0x0000 : msip      [31:0] — machine software interrupt pending
//   +0x4000 : mtimecmp  [63:0] — compare value (triggers irq when mtime≥mtimecmp)
//   +0xBFF8 : mtime     [63:0] — real-time counter (64-bit)
// =============================================================================

module rv32emc_clint (
  input  logic        clk_i,
  input  logic        rst_ni,
  // Simple register interface (not full AHB — address decoder external)
  input  logic [15:0] haddr_i,
  input  logic        hwrite_i,
  input  logic [31:0] hwdata_i,
  output logic [31:0] hrdata_o,
  input  logic        hsel_i,
  // IRQ outputs
  output logic        timer_irq_o,
  output logic        soft_irq_o
);

  logic [63:0] mtime_r;
  logic [63:0] mtimecmp_r;
  logic [31:0] msip_r;

  // mtime increments every clock (should be RTC-derived in real SoC)
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) mtime_r <= '0;
    else         mtime_r <= mtime_r + 1;
  end

  // Register write
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      mtimecmp_r <= 64'hFFFF_FFFF_FFFF_FFFF;
      msip_r     <= '0;
    end else if (hsel_i && hwrite_i) begin
      unique case (haddr_i)
        16'h0000: msip_r[0]          <= hwdata_i[0];
        16'h4000: mtimecmp_r[31:0]   <= hwdata_i;
        16'h4004: mtimecmp_r[63:32]  <= hwdata_i;
        default:  ;
      endcase
    end
  end

  // Register read
  always_comb begin
    hrdata_o = '0;
    if (hsel_i) begin
      unique case (haddr_i)
        16'h0000: hrdata_o = msip_r;
        16'h4000: hrdata_o = mtimecmp_r[31:0];
        16'h4004: hrdata_o = mtimecmp_r[63:32];
        16'hBFF8: hrdata_o = mtime_r[31:0];
        16'hBFFC: hrdata_o = mtime_r[63:32];
        default:  hrdata_o = '0;
      endcase
    end
  end

  assign timer_irq_o = (mtime_r >= mtimecmp_r);
  assign soft_irq_o  = msip_r[0];

endmodule : rv32emc_clint
