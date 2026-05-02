// =============================================================================
// rv32emc_lsu.sv — Load/Store Unit (AHB-Lite dBus master)
// =============================================================================

import rv32emc_pkg::*;

module rv32emc_lsu (
  input  logic          clk_i,
  input  logic          rst_ni,
  input  ex_mem_bundle_t ex_mem_i,

  // AHB-Lite data bus
  output logic [31:0]   haddr_o,
  output htrans_e       htrans_o,
  output hsize_e        hsize_o,
  output hburst_e       hburst_o,
  output logic [3:0]    hprot_o,
  output logic          hwrite_o,
  output logic [31:0]   hwdata_o,
  input  logic [31:0]   hrdata_i,
  input  logic          hready_i,
  input  logic          hresp_i,

  output logic          lsu_stall_o,
  output logic          addr_misalign_o,
  output mem_wb_bundle_t mem_wb_o
);

  // Address phase (combinational)
  logic [31:0] byte_addr;
  assign byte_addr = ex_mem_i.alu_result;

  // Misalignment detection
  always_comb begin
    addr_misalign_o = 1'b0;
    unique case (ex_mem_i.mem_size)
      MEM_HALF, MEM_HALFU: addr_misalign_o = byte_addr[0];
      MEM_WORD:             addr_misalign_o = |byte_addr[1:0];
      default:              addr_misalign_o = 1'b0;
    endcase
  end

  // AHB address phase
  assign haddr_o  = {byte_addr[31:2], 2'b00}; // Word-aligned AHB beat
  assign hwrite_o = ex_mem_i.mem_write;
  assign hburst_o = HBURST_SINGLE;
  assign hprot_o  = 4'b0011; // Privileged data access
  assign htrans_o = (ex_mem_i.mem_read | ex_mem_i.mem_write) && !addr_misalign_o
                    ? HTRANS_NONSEQ : HTRANS_IDLE;

  always_comb begin
    unique case (ex_mem_i.mem_size)
      MEM_BYTE, MEM_BYTEU: hsize_o = HSIZE_BYTE;
      MEM_HALF, MEM_HALFU: hsize_o = HSIZE_HALF;
      default:             hsize_o = HSIZE_WORD;
    endcase
  end

  // AHB data phase — write data (byte/half-word replicated to word lanes)
  always_comb begin
    hwdata_o = '0;
    unique case ({ex_mem_i.mem_size, byte_addr[1:0]})
      // Byte writes
      {MEM_BYTE, 2'b00}: hwdata_o = {24'h0, ex_mem_i.rs2_data[7:0]};
      {MEM_BYTE, 2'b01}: hwdata_o = {16'h0, ex_mem_i.rs2_data[7:0], 8'h0};
      {MEM_BYTE, 2'b10}: hwdata_o = {8'h0,  ex_mem_i.rs2_data[7:0], 16'h0};
      {MEM_BYTE, 2'b11}: hwdata_o = {ex_mem_i.rs2_data[7:0], 24'h0};
      // Half-word writes
      {MEM_HALF, 2'b00}: hwdata_o = {16'h0, ex_mem_i.rs2_data[15:0]};
      {MEM_HALF, 2'b10}: hwdata_o = {ex_mem_i.rs2_data[15:0], 16'h0};
      // Word writes
      default:           hwdata_o = ex_mem_i.rs2_data;
    endcase
  end

  // Stall when bus not ready during active transfer
  assign lsu_stall_o = (ex_mem_i.mem_read | ex_mem_i.mem_write) & ~hready_i;

  // Load data sign/zero extension
  logic [31:0] rdata_ext;
  always_comb begin
    rdata_ext = '0;
    unique case ({ex_mem_i.mem_size, byte_addr[1:0]})
      {MEM_BYTE,  2'b00}: rdata_ext = {{24{hrdata_i[7]}},  hrdata_i[7:0]};
      {MEM_BYTE,  2'b01}: rdata_ext = {{24{hrdata_i[15]}}, hrdata_i[15:8]};
      {MEM_BYTE,  2'b10}: rdata_ext = {{24{hrdata_i[23]}}, hrdata_i[23:16]};
      {MEM_BYTE,  2'b11}: rdata_ext = {{24{hrdata_i[31]}}, hrdata_i[31:24]};
      {MEM_BYTEU, 2'b00}: rdata_ext = {24'd0, hrdata_i[7:0]};
      {MEM_BYTEU, 2'b01}: rdata_ext = {24'd0, hrdata_i[15:8]};
      {MEM_BYTEU, 2'b10}: rdata_ext = {24'd0, hrdata_i[23:16]};
      {MEM_BYTEU, 2'b11}: rdata_ext = {24'd0, hrdata_i[31:24]};
      {MEM_HALF,  2'b00}: rdata_ext = {{16{hrdata_i[15]}}, hrdata_i[15:0]};
      {MEM_HALF,  2'b10}: rdata_ext = {{16{hrdata_i[31]}}, hrdata_i[31:16]};
      {MEM_HALFU, 2'b00}: rdata_ext = {16'd0, hrdata_i[15:0]};
      {MEM_HALFU, 2'b10}: rdata_ext = {16'd0, hrdata_i[31:16]};
      default:             rdata_ext = hrdata_i;
    endcase
  end

  // Pack MEM/WB bundle
  assign mem_wb_o.pc         = ex_mem_i.pc;
  assign mem_wb_o.alu_result = ex_mem_i.alu_result;
  assign mem_wb_o.mem_rdata  = rdata_ext;
  assign mem_wb_o.csr_rdata  = ex_mem_i.csr_rdata;
  assign mem_wb_o.wb_src     = ex_mem_i.wb_src;
  assign mem_wb_o.rf_we      = ex_mem_i.rf_we;
  assign mem_wb_o.rd         = ex_mem_i.rd;

endmodule : rv32emc_lsu


// =============================================================================
// rv32emc_csr.sv — Machine-mode CSR file
// =============================================================================
// Implements RISC-V Privileged ISA v20211203 M-mode subset:
//   mstatus, misa, mie, mtvec, mscratch, mepc, mcause, mtval, mip
//   mcycle[h], minstret[h], mvendorid, marchid, mimpid, mhartid
//   dcsr, dpc, dscratch0/1 (when in Debug Mode)
// =============================================================================

module rv32emc_csr
  import rv32emc_pkg::*;
#(
  parameter logic [31:0] MTVEC_RESET_VAL  = 32'h0000_0100,
  parameter logic [31:0] MVENDORID_VAL    = 32'h0,
  parameter logic [31:0] MARCHID_VAL      = 32'h16,
  parameter logic [31:0] MIMPID_VAL       = 32'h1,
  parameter logic [31:0] MHARTID_VAL      = 32'h0,
  parameter bit          ENABLE_PERF_CTRS = 1
)(
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic [11:0] csr_addr_i,
  input  logic [1:0]  csr_op_i,     // 00=rw 01=set 10=clr
  input  logic        csr_en_i,
  input  logic [31:0] csr_wdata_i,
  output logic [31:0] csr_rdata_o,

  // Trap inputs (from trap controller)
  input  logic        take_trap_i,
  input  logic [31:0] trap_cause_i,
  input  logic [31:0] trap_val_i,
  input  logic [31:0] trap_pc_i,
  input  logic        mret_i,

  // Interrupt inputs
  input  logic [15:0] irq_i,
  input  logic        timer_irq_i,
  input  logic        soft_irq_i,

  // Outputs
  output logic        mstatus_mie_o,
  output logic [31:0] mtvec_o,
  output logic [31:0] mepc_o,

  // Counters
  input  logic        instret_incr_i,
  input  logic        in_debug_mode_i
);

  // CSR storage
  logic        mstatus_mie, mstatus_mpie;
  logic [31:0] mtvec_r;
  logic [31:0] mscratch_r;
  logic [31:0] mepc_r;
  logic [31:0] mcause_r;
  logic [31:0] mtval_r;
  logic [31:0] mie_r;      // Interrupt enable bits
  logic [31:0] mip_r;      // Interrupt pending (read-only shadow)
  logic [63:0] mcycle_r;
  logic [63:0] minstret_r;
  // Debug CSRs
  logic [31:0] dcsr_r;
  logic [31:0] dpc_r;
  logic [31:0] dscratch0_r, dscratch1_r;

  // misa: RV32EMIC  (E=bit4, M=bit12, C=bit2, I=bit8 — we set E not I)
  localparam logic [31:0] MISA_VAL = {2'b01, 4'b0, 26'b0_0000_0001_0001_0000_0000_0001_00};
  //                                    MXL=1                E    M            C

  // MIP: wired to external inputs
  assign mip_r = {20'b0, irq_i[3:0] & mie_r[19:16],
                  4'b0,
                  timer_irq_i & mie_r[7],
                  3'b0,
                  soft_irq_i  & mie_r[3],
                  3'b0};

  // ---------------------------------------------------------------------------
  // CSR read
  // ---------------------------------------------------------------------------
  always_comb begin
    csr_rdata_o = '0;
    unique case (csr_addr_i)
      CSR_MSTATUS  : csr_rdata_o = {19'b0, 2'b11/*MPP=M*/, 3'b0, mstatus_mpie, 3'b0, mstatus_mie, 3'b0};
      CSR_MISA     : csr_rdata_o = MISA_VAL;
      CSR_MIE      : csr_rdata_o = mie_r;
      CSR_MTVEC    : csr_rdata_o = mtvec_r;
      CSR_MSCRATCH : csr_rdata_o = mscratch_r;
      CSR_MEPC     : csr_rdata_o = {mepc_r[31:1], 1'b0};
      CSR_MCAUSE   : csr_rdata_o = mcause_r;
      CSR_MTVAL    : csr_rdata_o = mtval_r;
      CSR_MIP      : csr_rdata_o = mip_r;
      CSR_MVENDORID: csr_rdata_o = MVENDORID_VAL;
      CSR_MARCHID  : csr_rdata_o = MARCHID_VAL;
      CSR_MIMPID   : csr_rdata_o = MIMPID_VAL;
      CSR_MHARTID  : csr_rdata_o = MHARTID_VAL;
      CSR_MCYCLE   : if (ENABLE_PERF_CTRS) csr_rdata_o = mcycle_r[31:0];
      CSR_MCYCLEH  : if (ENABLE_PERF_CTRS) csr_rdata_o = mcycle_r[63:32];
      CSR_MINSTRET : if (ENABLE_PERF_CTRS) csr_rdata_o = minstret_r[31:0];
      CSR_MINSTRETH: if (ENABLE_PERF_CTRS) csr_rdata_o = minstret_r[63:32];
      // Debug
      CSR_DCSR     : if (in_debug_mode_i) csr_rdata_o = dcsr_r;
      CSR_DPC      : if (in_debug_mode_i) csr_rdata_o = dpc_r;
      CSR_DSCRATCH0: if (in_debug_mode_i) csr_rdata_o = dscratch0_r;
      CSR_DSCRATCH1: if (in_debug_mode_i) csr_rdata_o = dscratch1_r;
      default      : csr_rdata_o = '0;
    endcase
  end

  // ---------------------------------------------------------------------------
  // Write data (CSRRW/S/C)
  // ---------------------------------------------------------------------------
  logic [31:0] csr_wdata_nxt;
  always_comb begin
    unique case (csr_op_i)
      2'b00: csr_wdata_nxt = csr_wdata_i;                    // CSRRW
      2'b01: csr_wdata_nxt = csr_rdata_o | csr_wdata_i;      // CSRRS
      2'b10: csr_wdata_nxt = csr_rdata_o & ~csr_wdata_i;     // CSRRC
      default: csr_wdata_nxt = csr_wdata_i;
    endcase
  end

  // ---------------------------------------------------------------------------
  // CSR write
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      mstatus_mie  <= 1'b0;
      mstatus_mpie <= 1'b0;
      mtvec_r      <= MTVEC_RESET_VAL;
      mscratch_r   <= '0;
      mepc_r       <= '0;
      mcause_r     <= '0;
      mtval_r      <= '0;
      mie_r        <= '0;
      dcsr_r       <= 32'h4000_0003; // xdebugver=4, prv=M
      dpc_r        <= '0;
      dscratch0_r  <= '0;
      dscratch1_r  <= '0;
    end else begin
      // Trap entry
      if (take_trap_i && !in_debug_mode_i) begin
        mepc_r       <= trap_pc_i;
        mcause_r     <= trap_cause_i;
        mtval_r      <= trap_val_i;
        mstatus_mpie <= mstatus_mie;
        mstatus_mie  <= 1'b0;
      end
      // MRET
      if (mret_i) begin
        mstatus_mie  <= mstatus_mpie;
        mstatus_mpie <= 1'b1;
      end
      // CSR instruction write
      if (csr_en_i) begin
        unique case (csr_addr_i)
          CSR_MSTATUS : begin
            mstatus_mie  <= csr_wdata_nxt[3];
            mstatus_mpie <= csr_wdata_nxt[7];
          end
          CSR_MIE      : mie_r     <= csr_wdata_nxt & 32'h00FF_0888;
          CSR_MTVEC    : mtvec_r   <= {csr_wdata_nxt[31:2], csr_wdata_nxt[1:0] & 2'b01};
          CSR_MSCRATCH : mscratch_r<= csr_wdata_nxt;
          CSR_MEPC     : mepc_r    <= {csr_wdata_nxt[31:1], 1'b0};
          CSR_MCAUSE   : mcause_r  <= csr_wdata_nxt;
          CSR_MTVAL    : mtval_r   <= csr_wdata_nxt;
          CSR_DCSR     : if (in_debug_mode_i) dcsr_r     <= csr_wdata_nxt;
          CSR_DPC      : if (in_debug_mode_i) dpc_r      <= csr_wdata_nxt;
          CSR_DSCRATCH0: if (in_debug_mode_i) dscratch0_r<= csr_wdata_nxt;
          CSR_DSCRATCH1: if (in_debug_mode_i) dscratch1_r<= csr_wdata_nxt;
          default: ;
        endcase
      end
    end
  end

  // Performance counters
  generate if (ENABLE_PERF_CTRS) begin : gen_perf
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        mcycle_r   <= '0;
        minstret_r <= '0;
      end else begin
        mcycle_r   <= mcycle_r + 1;
        if (instret_incr_i) minstret_r <= minstret_r + 1;
      end
    end
  end else begin
    assign mcycle_r   = '0;
    assign minstret_r = '0;
  end endgenerate

  assign mstatus_mie_o = mstatus_mie;
  assign mtvec_o       = mtvec_r;
  assign mepc_o        = mepc_r;

endmodule : rv32emc_csr


// =============================================================================
// rv32emc_div.sv — Non-restoring 32-bit integer divider (32 cycles latency)
// =============================================================================

module rv32emc_div
  import rv32emc_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        start_i,
  input  alu_op_e     op_i,
  input  logic [31:0] a_i,   // Dividend
  input  logic [31:0] b_i,   // Divisor
  output logic [31:0] result_o,
  output logic        valid_o,
  output logic        busy_o
);

  logic [4:0]  cnt;
  logic [63:0] remainder;
  logic [31:0] quotient;
  logic [31:0] divisor_r;
  logic        negate_result;
  logic        is_rem;
  alu_op_e     op_r;

  logic signed_op;
  assign signed_op = (op_i == ALU_DIV || op_i == ALU_REM);

  logic [31:0] a_abs, b_abs;
  assign a_abs = (signed_op && a_i[31]) ? -a_i : a_i;
  assign b_abs = (signed_op && b_i[31]) ? -b_i : b_i;

  // Division by zero / overflow handling per RISC-V spec
  logic div_zero, div_overflow;
  assign div_zero     = (b_i == 32'd0);
  assign div_overflow = signed_op && (a_i == 32'h8000_0000) && (b_i == 32'hFFFF_FFFF);

  typedef enum logic [1:0] { IDLE, CALC, DONE } div_state_e;
  div_state_e state_r;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_r <= IDLE; cnt <= '0; valid_o <= '0; busy_o <= '0;
    end else begin
      unique case (state_r)
        IDLE: begin
          valid_o <= 1'b0;
          if (start_i && !div_zero && !div_overflow) begin
            remainder   <= {32'd0, a_abs};
            divisor_r   <= b_abs;
            quotient    <= '0;
            cnt         <= 5'd31;
            op_r        <= op_i;
            is_rem      <= (op_i == ALU_REM || op_i == ALU_REMU);
            negate_result <= signed_op && (
              is_rem ? a_i[31] :
              (a_i[31] ^ b_i[31])
            );
            state_r <= CALC;
            busy_o  <= 1'b1;
          end
        end
        CALC: begin
          // Non-restoring step
          if (remainder[62:31] >= divisor_r) begin
            remainder <= {remainder[62:0], 1'b0} - {1'b0, divisor_r, 31'b0};
            quotient  <= {quotient[30:0], 1'b1};
          end else begin
            remainder <= {remainder[62:0], 1'b0};
            quotient  <= {quotient[30:0], 1'b0};
          end
          if (cnt == 0) begin state_r <= DONE; end
          else          cnt <= cnt - 1;
        end
        DONE: begin
          valid_o <= 1'b1;
          busy_o  <= 1'b0;
          state_r <= IDLE;
        end
      endcase
    end
  end

  always_comb begin
    if (div_zero)     result_o = is_rem ? a_i : 32'hFFFF_FFFF;
    else if (div_overflow) result_o = is_rem ? 32'd0 : 32'h8000_0000;
    else begin
      logic [31:0] raw = is_rem ? remainder[31:0] : quotient;
      result_o = negate_result ? -raw : raw;
    end
  end

endmodule : rv32emc_div


// =============================================================================
// rv32emc_bpu.sv — 2-bit saturating counter Branch Prediction Unit
// =============================================================================

module rv32emc_bpu
  import rv32emc_pkg::*;
#(
  parameter int unsigned ENTRIES = 256  // Must be power-of-2
)(
  input  logic        clk_i,
  input  logic        rst_ni,
  // Prediction (combinational)
  input  logic [31:0] fetch_pc_i,
  output logic        predict_taken_o,
  output logic [31:0] predict_target_o,
  // Training (from EX stage)
  input  logic        update_en_i,
  input  logic [31:0] update_pc_i,
  input  logic        update_taken_i,
  input  logic [31:0] update_target_i
);

  localparam int IDX_W = $clog2(ENTRIES);

  // Branch Target Buffer: tag + target
  logic [31-IDX_W:0] btb_tag   [0:ENTRIES-1];
  logic [31:0]       btb_target[0:ENTRIES-1];
  logic [1:0]        bht       [0:ENTRIES-1]; // 2-bit saturating counter

  logic [IDX_W-1:0] fetch_idx, update_idx;
  assign fetch_idx  = fetch_pc_i[IDX_W+1:2];
  assign update_idx = update_pc_i[IDX_W+1:2];

  // Prediction
  logic tag_hit;
  assign tag_hit = (btb_tag[fetch_idx] == fetch_pc_i[31:IDX_W+2]);
  assign predict_taken_o  = tag_hit && bht[fetch_idx][1]; // Taken if counter MSB=1
  assign predict_target_o = btb_target[fetch_idx];

  // Training
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < ENTRIES; i++) begin
        btb_tag[i]    <= '1;
        btb_target[i] <= '0;
        bht[i]        <= 2'b01; // Weakly not-taken
      end
    end else if (update_en_i) begin
      btb_tag[update_idx]    <= update_pc_i[31:IDX_W+2];
      btb_target[update_idx] <= update_target_i;
      // Saturating increment/decrement
      if (update_taken_i)
        bht[update_idx] <= (bht[update_idx] == 2'b11) ? 2'b11 : bht[update_idx] + 1;
      else
        bht[update_idx] <= (bht[update_idx] == 2'b00) ? 2'b00 : bht[update_idx] - 1;
    end
  end

endmodule : rv32emc_bpu


// =============================================================================
// rv32emc_trap.sv — Trap / Exception / Interrupt arbitration
// =============================================================================

module rv32emc_trap
  import rv32emc_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  // Exception sources
  input  logic        fetch_err_i,
  input  logic        illegal_inst_i,
  input  logic        ecall_i,
  input  logic        ebreak_i,
  input  logic        misalign_ld_i,
  input  logic        misalign_st_i,
  input  logic        bus_err_ld_i,
  input  logic        bus_err_st_i,
  // Interrupt
  input  logic        irq_pending_i,
  input  logic        mstatus_mie_i,
  input  logic        in_debug_mode_i,
  input  logic [31:0] curr_pc_i,
  input  logic        nmi_i,
  // Outputs
  output logic        take_trap_o,
  output logic [31:0] trap_cause_o,
  output logic [31:0] trap_val_o,
  output logic [31:0] trap_pc_o,
  output logic [31:0] trap_target_o,
  output logic        ebreak_hit_o
);

  logic any_exception;
  assign any_exception = fetch_err_i | illegal_inst_i | ecall_i |
                         misalign_ld_i | misalign_st_i |
                         bus_err_ld_i | bus_err_st_i;

  // EBREAK triggers debug entry, not a normal trap
  assign ebreak_hit_o = ebreak_i && !in_debug_mode_i;

  assign take_trap_o  = (any_exception | (irq_pending_i & mstatus_mie_i) | nmi_i)
                        && !in_debug_mode_i && !ebreak_i;

  // Priority: NMI > exceptions (in program order) > interrupts
  always_comb begin
    trap_cause_o  = '0;
    trap_val_o    = '0;
    trap_pc_o     = curr_pc_i;
    trap_target_o = '0; // Set from mtvec in CSR

    if (nmi_i) begin
      trap_cause_o = {1'b1, 31'd31}; // Custom NMI cause
    end else if (fetch_err_i) begin
      trap_cause_o = {1'b0, 27'd0, EXC_IACCESS_FAULT};
      trap_val_o   = curr_pc_i;
    end else if (illegal_inst_i) begin
      trap_cause_o = {1'b0, 27'd0, EXC_ILLEGAL_INST};
    end else if (misalign_ld_i) begin
      trap_cause_o = {1'b0, 27'd0, EXC_LADDR_MISALIGN};
    end else if (misalign_st_i) begin
      trap_cause_o = {1'b0, 27'd0, EXC_SADDR_MISALIGN};
    end else if (bus_err_ld_i) begin
      trap_cause_o = {1'b0, 27'd0, EXC_LACCESS_FAULT};
    end else if (bus_err_st_i) begin
      trap_cause_o = {1'b0, 27'd0, EXC_SACCESS_FAULT};
    end else if (ecall_i) begin
      trap_cause_o = {1'b0, 27'd0, EXC_ECALL_M};
    end else if (irq_pending_i & mstatus_mie_i) begin
      // Interrupt cause (bit 31 = 1)
      trap_cause_o = {1'b1, 27'd0, IRQ_MEXT}; // Simplified: real impl checks MIP/MIE priority
    end
  end

endmodule : rv32emc_trap
