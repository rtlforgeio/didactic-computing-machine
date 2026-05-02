// =============================================================================
// rv32emc_core.sv — RISC-V RV32EMC Top-Level Core
// -----------------------------------------------------------------------------
//
// Microarchitecture: 3-Stage In-Order Pipeline
//   ┌────────┐     ┌────────┐     ┌────────┐
//   │  IF    │────▶│  ID/EX │────▶│  MEM/WB│
//   └────────┘     └────────┘     └────────┘
//
// IF  : Instruction Fetch + C-extension decompressor
// ID/EX: Decode, register read, ALU/MUL execute, branch resolve
// MEM/WB: LSU access, CSR read/write, writeback
//
// Pipeline is stalled on:
//   • Load-use hazard  (1 cycle stall)
//   • MUL/DIV latency  (2/32 cycle stall)
//   • IFetch bus not ready
//   • LSU bus not ready
//
// Forwarding: EX→ID/EX, MEM→ID/EX (covers all RAW hazards except load-use)
//
// Interrupts: vectored or direct (MTVEC.MODE)
// Debug: RISC-V Debug Spec 0.13.2, abstract commands, 2-instruction progbuf
//
// =============================================================================

`include "rv32emc_pkg.sv"
`include "rv32emc_if.sv"

module rv32emc_core
  import rv32emc_pkg::*;
#(
  // -----------------------------------------------------------------------
  // Microarchitecture parameters
  // -----------------------------------------------------------------------
  parameter logic [31:0] BOOT_ADDR       = 32'h0000_0000,
  parameter logic [31:0] MTVEC_RESET_VAL = 32'h0000_0100,
  parameter logic [31:0] MVENDORID_VAL   = 32'h0000_0000,
  parameter logic [31:0] MARCHID_VAL     = 32'h0000_0016, // Unofficial
  parameter logic [31:0] MIMPID_VAL      = 32'h0000_0001,
  parameter logic [31:0] MHARTID_VAL     = 32'h0000_0000,
  // -----------------------------------------------------------------------
  // Feature enables
  // -----------------------------------------------------------------------
  parameter bit ENABLE_MUL        = 1,   // M-extension multiply
  parameter bit ENABLE_DIV        = 1,   // M-extension divide
  parameter bit ENABLE_COMPRESSED = 1,   // C-extension
  parameter bit ENABLE_DEBUG      = 1,   // RISC-V Debug Spec 0.13
  parameter bit ENABLE_PERF_CTRS  = 1,   // mcycle / minstret
  parameter bit ENABLE_PMP        = 0,   // Physical Memory Protection (future)
  // -----------------------------------------------------------------------
  // Branch prediction
  // -----------------------------------------------------------------------
  parameter bit ENABLE_BPU        = 1,   // 2-bit saturating counter BPU
  parameter int unsigned BPU_ENTRIES = 256 // BTB entries (power-of-2)
)(
  // -----------------------------------------------------------------------
  // Clock and Reset
  // -----------------------------------------------------------------------
  input  logic        clk_i,       // Core clock
  input  logic        rst_ni,      // Async active-low reset

  // -----------------------------------------------------------------------
  // Instruction Fetch AHB-Lite Master (iBus)
  // -----------------------------------------------------------------------
  output logic [31:0] ibus_haddr_o,
  output htrans_e     ibus_htrans_o,
  output hsize_e      ibus_hsize_o,
  output hburst_e     ibus_hburst_o,
  output logic [3:0]  ibus_hprot_o,
  output logic        ibus_hwrite_o,  // Always 0 for fetch bus
  input  logic [31:0] ibus_hrdata_i,
  input  logic        ibus_hready_i,
  input  logic        ibus_hresp_i,

  // -----------------------------------------------------------------------
  // Data / LSU AHB-Lite Master (dBus)
  // -----------------------------------------------------------------------
  output logic [31:0] dbus_haddr_o,
  output htrans_e     dbus_htrans_o,
  output hsize_e      dbus_hsize_o,
  output hburst_e     dbus_hburst_o,
  output logic [3:0]  dbus_hprot_o,
  output logic        dbus_hwrite_o,
  output logic [31:0] dbus_hwdata_o,
  input  logic [31:0] dbus_hrdata_i,
  input  logic        dbus_hready_i,
  input  logic        dbus_hresp_i,

  // -----------------------------------------------------------------------
  // Interrupt inputs
  // -----------------------------------------------------------------------
  input  logic [15:0] irq_i,        // 16 external IRQ lines (M-mode)
  input  logic        timer_irq_i,  // CLINT machine timer interrupt
  input  logic        soft_irq_i,   // CLINT machine software interrupt
  input  logic        nmi_i,        // Non-maskable interrupt

  // -----------------------------------------------------------------------
  // Debug interface (RISC-V Debug Spec 0.13 — connected to DM externally)
  // -----------------------------------------------------------------------
  input  logic        debug_req_i,     // Halt request from DM
  output logic        debug_halted_o,  // Core is in Debug Mode
  input  logic        debug_resume_i,  // Resume request from DM
  input  logic        debug_step_i,    // Single-step enable
  input  logic        dbg_reg_rd_i,    // Abstract: read GPR
  input  logic        dbg_reg_wr_i,    // Abstract: write GPR
  input  logic [4:0]  dbg_reg_addr_i,  // GPR index (0–15 for RV32E)
  input  logic [31:0] dbg_reg_wdata_i, // Write data for abstract cmd
  output logic [31:0] dbg_reg_rdata_o, // Read data for abstract cmd
  input  logic [63:0] dbg_progbuf_i,   // 2-instruction program buffer
  input  logic        dbg_progbuf_exec_i,

  // -----------------------------------------------------------------------
  // Core status outputs
  // -----------------------------------------------------------------------
  output logic        core_sleep_o,    // WFI sleep indicator
  output logic [31:0] core_pc_o        // Current PC (for DM / trace)
);

  // =========================================================================
  // Internal wires — pipeline bundles
  // =========================================================================

  // IF outputs
  logic [31:0]  if_pc;
  logic [31:0]  if_instr;     // Decompressed instruction
  logic         if_valid;
  logic         if_is_comp;   // Was 16-bit
  logic         if_fetch_err; // Fetch bus error

  // ID/EX bundle (decode → execute)
  id_ex_bundle_t  id_ex_reg, id_ex_nxt;

  // EX/MEM bundle
  ex_mem_bundle_t ex_mem_reg, ex_mem_nxt;

  // MEM/WB bundle
  mem_wb_bundle_t mem_wb_reg, mem_wb_nxt;

  // =========================================================================
  // Stall / flush control
  // =========================================================================
  logic stall_if;    // Freeze IF stage
  logic stall_id;    // Freeze ID/EX latch
  logic flush_if;    // Squash IF instruction (taken branch)
  logic flush_id;    // Squash ID/EX (taken branch, exception)
  logic flush_ex;    // Squash EX/MEM

  // =========================================================================
  // Forwarding
  // =========================================================================
  fwd_sel_e fwd_a_sel, fwd_b_sel;
  logic [31:0] fwd_a_data, fwd_b_data;

  // =========================================================================
  // Register file
  // =========================================================================
  logic [REG_AW-1:0] rf_rs1_addr, rf_rs2_addr;
  logic [31:0]       rf_rs1_data, rf_rs2_data;
  logic              rf_we;
  logic [REG_AW-1:0] rf_waddr;
  logic [31:0]       rf_wdata;

  // =========================================================================
  // ALU / MUL / DIV
  // =========================================================================
  logic [31:0]  alu_result;
  logic [31:0]  mul_result;
  logic [31:0]  div_result;
  logic         mul_valid;     // MUL result ready
  logic         div_valid;     // DIV result ready
  logic         mul_busy;
  logic         div_busy;

  // =========================================================================
  // Branch signals
  // =========================================================================
  logic         br_taken;
  logic [31:0]  br_target;
  logic [31:0]  predicted_pc; // BPU prediction

  // =========================================================================
  // CSR
  // =========================================================================
  logic [31:0]  csr_rdata;
  logic         csr_we;
  logic [11:0]  csr_waddr;
  logic [31:0]  csr_wdata;
  logic         mstatus_mie;    // Global interrupt enable
  logic [31:0]  mtvec;
  logic [31:0]  mepc;
  logic         take_trap;
  logic [31:0]  trap_cause;
  logic [31:0]  trap_val;
  logic [31:0]  trap_pc;

  // =========================================================================
  // Debug
  // =========================================================================
  logic         in_debug_mode;
  logic         ebreak_hit;

  // =========================================================================
  // Performance counters
  // =========================================================================
  logic [63:0]  mcycle;
  logic [63:0]  minstret;

  // =========================================================================
  // Module instantiations
  // =========================================================================

  // -------------------------------------------------------------------------
  // 1. Instruction Fetch Stage
  // -------------------------------------------------------------------------
  rv32emc_if_stage #(
    .BOOT_ADDR      (BOOT_ADDR),
    .ENABLE_COMPRESSED(ENABLE_COMPRESSED),
    .ENABLE_BPU     (ENABLE_BPU),
    .BPU_ENTRIES    (BPU_ENTRIES)
  ) u_if_stage (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    // AHB-Lite iBus
    .haddr_o        (ibus_haddr_o),
    .htrans_o       (ibus_htrans_o),
    .hsize_o        (ibus_hsize_o),
    .hburst_o       (ibus_hburst_o),
    .hprot_o        (ibus_hprot_o),
    .hwrite_o       (ibus_hwrite_o),
    .hrdata_i       (ibus_hrdata_i),
    .hready_i       (ibus_hready_i),
    .hresp_i        (ibus_hresp_i),
    // Control
    .stall_i        (stall_if),
    .flush_i        (flush_if),
    .br_taken_i     (br_taken),
    .br_target_i    (br_target),
    // Outputs to ID
    .pc_o           (if_pc),
    .instr_o        (if_instr),
    .valid_o        (if_valid),
    .is_comp_o      (if_is_comp),
    .fetch_err_o    (if_fetch_err)
  );

  // -------------------------------------------------------------------------
  // 2. Decode Stage
  // -------------------------------------------------------------------------
  rv32emc_decode #(
    .ENABLE_MUL     (ENABLE_MUL),
    .ENABLE_DIV     (ENABLE_DIV),
    .ENABLE_COMPRESSED(ENABLE_COMPRESSED)
  ) u_decode (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    .valid_i        (if_valid),
    .pc_i           (if_pc),
    .instr_i        (if_instr),
    .is_comp_i      (if_is_comp),
    .fetch_err_i    (if_fetch_err),
    // Register file read
    .rf_rs1_addr_o  (rf_rs1_addr),
    .rf_rs2_addr_o  (rf_rs2_addr),
    .rf_rs1_data_i  (rf_rs1_data),
    .rf_rs2_data_i  (rf_rs2_data),
    // Forwarded operands from EX/MEM
    .fwd_a_sel_i    (fwd_a_sel),
    .fwd_b_sel_i    (fwd_b_sel),
    .fwd_a_data_i   (fwd_a_data),
    .fwd_b_data_i   (fwd_b_data),
    // Stall/flush
    .stall_i        (stall_id),
    .flush_i        (flush_id),
    // Output bundle
    .id_ex_o        (id_ex_nxt)
  );

  // ID/EX pipeline register
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) id_ex_reg <= '0;
    else if (!stall_id) begin
      if (flush_id) id_ex_reg <= '0;
      else          id_ex_reg <= id_ex_nxt;
    end
  end

  // -------------------------------------------------------------------------
  // 3. Execute Stage
  // -------------------------------------------------------------------------
  rv32emc_execute #(
    .ENABLE_MUL (ENABLE_MUL),
    .ENABLE_DIV (ENABLE_DIV)
  ) u_execute (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    .id_ex_i        (id_ex_reg),
    .alu_result_o   (alu_result),
    .mul_result_o   (mul_result),
    .div_result_o   (div_result),
    .mul_valid_o    (mul_valid),
    .div_valid_o    (div_valid),
    .mul_busy_o     (mul_busy),
    .div_busy_o     (div_busy),
    .br_taken_o     (br_taken),
    .br_target_o    (br_target),
    .ex_mem_o       (ex_mem_nxt)
  );

  // EX/MEM pipeline register
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) ex_mem_reg <= '0;
    else if (!stall_if) begin  // stall only if MEM is stalled
      if (flush_ex) ex_mem_reg <= '0;
      else          ex_mem_reg <= ex_mem_nxt;
    end
  end

  // -------------------------------------------------------------------------
  // 4. Load/Store Unit (MEM stage)
  // -------------------------------------------------------------------------
  rv32emc_lsu u_lsu (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    .ex_mem_i       (ex_mem_reg),
    // AHB-Lite dBus
    .haddr_o        (dbus_haddr_o),
    .htrans_o       (dbus_htrans_o),
    .hsize_o        (dbus_hsize_o),
    .hburst_o       (dbus_hburst_o),
    .hprot_o        (dbus_hprot_o),
    .hwrite_o       (dbus_hwrite_o),
    .hwdata_o       (dbus_hwdata_o),
    .hrdata_i       (dbus_hrdata_i),
    .hready_i       (dbus_hready_i),
    .hresp_i        (dbus_hresp_i),
    // Stall to pipeline
    .lsu_stall_o    (/* connects to hazard unit */),
    .mem_wb_o       (mem_wb_nxt)
  );

  // MEM/WB pipeline register
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) mem_wb_reg <= '0;
    else         mem_wb_reg <= mem_wb_nxt;
  end

  // -------------------------------------------------------------------------
  // 5. Register File (16 × 32-bit, RV32E)
  // -------------------------------------------------------------------------
  rv32emc_regfile u_regfile (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    // Read ports
    .rs1_addr_i     (rf_rs1_addr),
    .rs2_addr_i     (rf_rs2_addr),
    .rs1_data_o     (rf_rs1_data),
    .rs2_data_o     (rf_rs2_data),
    // Write port
    .rd_we_i        (rf_we),
    .rd_addr_i      (rf_waddr),
    .rd_data_i      (rf_wdata),
    // Debug abstract command access
    .dbg_rd_en_i    (dbg_reg_rd_i),
    .dbg_wr_en_i    (dbg_reg_wr_i),
    .dbg_addr_i     (dbg_reg_addr_i[REG_AW-1:0]),
    .dbg_wdata_i    (dbg_reg_wdata_i),
    .dbg_rdata_o    (dbg_reg_rdata_o)
  );

  // -------------------------------------------------------------------------
  // 6. Writeback mux
  // -------------------------------------------------------------------------
  always_comb begin
    rf_we    = mem_wb_reg.rf_we;
    rf_waddr = mem_wb_reg.rd;
    unique case (mem_wb_reg.wb_src)
      WB_ALU : rf_wdata = mem_wb_reg.alu_result;
      WB_MEM : rf_wdata = mem_wb_reg.mem_rdata;
      WB_PC4 : rf_wdata = mem_wb_reg.pc + 32'd4;
      WB_CSR : rf_wdata = mem_wb_reg.csr_rdata;
      WB_MUL : rf_wdata = mem_wb_reg.alu_result; // mux already in EX
      WB_DIV : rf_wdata = mem_wb_reg.alu_result;
      default: rf_wdata = mem_wb_reg.alu_result;
    endcase
    // Suppress write in debug halt (DM owns the bus)
    if (in_debug_mode) rf_we = 1'b0;
  end

  // -------------------------------------------------------------------------
  // 7. Hazard detection and forwarding
  // -------------------------------------------------------------------------
  rv32emc_hazard u_hazard (
    // ID/EX source registers
    .id_rs1_i       (id_ex_nxt.rs1),
    .id_rs2_i       (id_ex_nxt.rs2),
    // EX/MEM destination
    .ex_rd_i        (ex_mem_reg.rd),
    .ex_rf_we_i     (ex_mem_reg.rf_we),
    .ex_mem_read_i  (ex_mem_reg.mem_read),
    // MEM/WB destination
    .mem_rd_i       (mem_wb_reg.rd),
    .mem_rf_we_i    (mem_wb_reg.rf_we),
    // Mul/Div busy
    .mul_busy_i     (mul_busy),
    .div_busy_i     (div_busy),
    // Bus stalls
    .ibus_stall_i   (~ibus_hready_i),
    .lsu_stall_i    (~dbus_hready_i & (ex_mem_reg.mem_read | ex_mem_reg.mem_write)),
    // Forwarding selects
    .fwd_a_sel_o    (fwd_a_sel),
    .fwd_b_sel_o    (fwd_b_sel),
    // Forwarding data
    .ex_alu_result_i(alu_result),
    .mem_alu_result_i(mem_wb_reg.alu_result),
    .mem_rdata_i    (mem_wb_reg.mem_rdata),
    .wb_data_i      (rf_wdata),
    .fwd_a_data_o   (fwd_a_data),
    .fwd_b_data_o   (fwd_b_data),
    // Stall/flush
    .stall_if_o     (stall_if),
    .stall_id_o     (stall_id),
    .flush_if_o     (flush_if),
    .flush_id_o     (flush_id),
    .flush_ex_o     (flush_ex)
  );

  // -------------------------------------------------------------------------
  // 8. CSR File
  // -------------------------------------------------------------------------
  rv32emc_csr #(
    .MTVEC_RESET_VAL (MTVEC_RESET_VAL),
    .MVENDORID_VAL   (MVENDORID_VAL),
    .MARCHID_VAL     (MARCHID_VAL),
    .MIMPID_VAL      (MIMPID_VAL),
    .MHARTID_VAL     (MHARTID_VAL),
    .ENABLE_PERF_CTRS(ENABLE_PERF_CTRS)
  ) u_csr (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    // Decode stage access
    .csr_addr_i     (id_ex_reg.csr_addr),
    .csr_op_i       (id_ex_reg.csr_op),
    .csr_en_i       (id_ex_reg.csr_en),
    .csr_wdata_i    (id_ex_reg.rs1[0] ? rf_rs1_data : {27'b0, id_ex_reg.rs1}),
    .csr_rdata_o    (csr_rdata),
    // Trap control (from EX/MEM)
    .take_trap_i    (take_trap),
    .trap_cause_i   (trap_cause),
    .trap_val_i     (trap_val),
    .trap_pc_i      (trap_pc),
    .mret_i         (ex_mem_reg.ecall ? 1'b0 : id_ex_reg.mret),
    // Interrupt inputs
    .irq_i          (irq_i),
    .timer_irq_i    (timer_irq_i),
    .soft_irq_i     (soft_irq_i),
    // Outputs
    .mstatus_mie_o  (mstatus_mie),
    .mtvec_o        (mtvec),
    .mepc_o         (mepc),
    // Counter feeds
    .instret_incr_i (if_valid & ~stall_if),
    // Debug mode
    .in_debug_mode_i(in_debug_mode)
  );

  // -------------------------------------------------------------------------
  // 9. Debug controller
  // -------------------------------------------------------------------------
  generate if (ENABLE_DEBUG) begin : gen_debug
    rv32emc_debug u_debug (
      .clk_i          (clk_i),
      .rst_ni         (rst_ni),
      .debug_req_i    (debug_req_i),
      .debug_resume_i (debug_resume_i),
      .debug_step_i   (debug_step_i),
      .progbuf_i      (dbg_progbuf_i),
      .progbuf_exec_i (dbg_progbuf_exec_i),
      .ebreak_hit_i   (ebreak_hit),
      .in_debug_mode_o(in_debug_mode),
      .debug_halted_o (debug_halted_o),
      .halt_ack_o     (/* drives stall_if */)
    );
  end else begin : gen_no_debug
    assign in_debug_mode  = 1'b0;
    assign debug_halted_o = 1'b0;
  end endgenerate

  // -------------------------------------------------------------------------
  // 10. Trap / exception arbitration
  // -------------------------------------------------------------------------
  rv32emc_trap u_trap (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    // Sources
    .fetch_err_i    (if_fetch_err),
    .illegal_inst_i (ex_mem_reg.illegal),
    .ecall_i        (ex_mem_reg.ecall),
    .ebreak_i       (ex_mem_reg.ebreak),
    .misalign_ld_i  (1'b0), // driven by LSU
    .misalign_st_i  (1'b0),
    .bus_err_ld_i   (dbus_hresp_i & ex_mem_reg.mem_read),
    .bus_err_st_i   (dbus_hresp_i & ex_mem_reg.mem_write),
    .irq_pending_i  (|irq_i | timer_irq_i | soft_irq_i),
    .mstatus_mie_i  (mstatus_mie),
    .in_debug_mode_i(in_debug_mode),
    .curr_pc_i      (ex_mem_reg.pc),
    .nmi_i          (nmi_i),
    // Outputs
    .take_trap_o    (take_trap),
    .trap_cause_o   (trap_cause),
    .trap_val_o     (trap_val),
    .trap_pc_o      (trap_pc),
    .trap_target_o  (/* sent to IF stage pc_sel */),
    .ebreak_hit_o   (ebreak_hit)
  );

  // -------------------------------------------------------------------------
  // Status outputs
  // -------------------------------------------------------------------------
  assign core_pc_o    = if_pc;
  assign core_sleep_o = id_ex_reg.wfi & ~(|irq_i | timer_irq_i | soft_irq_i);

  // =========================================================================
  // Assertions (formal/simulation only — synthesisable guard)
  // =========================================================================
`ifdef FORMAL
  // x0 always reads 0
  assert_x0_zero: assert property (
    @(posedge clk_i) disable iff (!rst_ni)
    (rf_rs1_addr == '0) |-> (rf_rs1_data == '0)
  );

  // No simultaneous memory and fetch bus errors
  assert_no_dual_err: assert property (
    @(posedge clk_i) disable iff (!rst_ni)
    ~(if_fetch_err & dbus_hresp_i)
  );
`endif

endmodule : rv32emc_core
