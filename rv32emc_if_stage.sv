// =============================================================================
// rv32emc_if_stage.sv — Instruction Fetch Stage
// =============================================================================
// Responsibilities:
//   1. Drive AHB-Lite iBus with sequential / branch-target addresses
//   2. Handle 16-bit (C-extension) and 32-bit instruction boundaries
//   3. Decompress RVC instructions to RVI before passing to decode
//   4. 2-bit saturating counter BPU with configurable BTB depth
// =============================================================================

import rv32emc_pkg::*;

module rv32emc_if_stage #(
  parameter logic [31:0] BOOT_ADDR        = 32'h0000_0000,
  parameter bit          ENABLE_COMPRESSED= 1,
  parameter bit          ENABLE_BPU       = 1,
  parameter int unsigned BPU_ENTRIES      = 256
)(
  input  logic        clk_i,
  input  logic        rst_ni,

  // AHB-Lite iBus
  output logic [31:0] haddr_o,
  output htrans_e     htrans_o,
  output hsize_e      hsize_o,
  output hburst_e     hburst_o,
  output logic [3:0]  hprot_o,
  output logic        hwrite_o,
  input  logic [31:0] hrdata_i,
  input  logic        hready_i,
  input  logic        hresp_i,

  // Pipeline control
  input  logic        stall_i,
  input  logic        flush_i,
  input  logic        br_taken_i,
  input  logic [31:0] br_target_i,

  // To decode
  output logic [31:0] pc_o,
  output logic [31:0] instr_o,    // Decompressed 32-bit instruction
  output logic        valid_o,
  output logic        is_comp_o,  // Instruction was 16-bit
  output logic        fetch_err_o
);

  // ---------------------------------------------------------------------------
  // PC register and next-PC logic
  // ---------------------------------------------------------------------------
  logic [31:0] pc_r, pc_nxt;
  logic [31:0] bpu_target;
  logic        bpu_predict_taken;

  always_comb begin : pc_sel
    if (br_taken_i)
      pc_nxt = br_target_i;               // Resolved branch target
    else if (ENABLE_BPU && bpu_predict_taken)
      pc_nxt = bpu_target;                // BPU predicted taken
    else
      pc_nxt = pc_r + (ENABLE_COMPRESSED && is_comp_o ? 32'd2 : 32'd4);
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)       pc_r <= BOOT_ADDR;
    else if (!stall_i) pc_r <= pc_nxt;
  end

  // ---------------------------------------------------------------------------
  // AHB-Lite fetch request — always word-aligned reads
  // ---------------------------------------------------------------------------
  assign haddr_o  = {pc_r[31:2], 2'b00}; // Align to word
  assign htrans_o = stall_i ? HTRANS_IDLE : HTRANS_NONSEQ;
  assign hsize_o  = HSIZE_WORD;
  assign hburst_o = HBURST_SINGLE;
  assign hprot_o  = 4'b0001; // Privileged opcode fetch
  assign hwrite_o = 1'b0;

  // ---------------------------------------------------------------------------
  // Instruction data register (data phase of AHB)
  // ---------------------------------------------------------------------------
  logic [31:0] fetch_data;
  logic        fetch_valid;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      fetch_data  <= '0;
      fetch_valid <= 1'b0;
    end else if (hready_i) begin
      fetch_data  <= hrdata_i;
      fetch_valid <= (htrans_o == HTRANS_NONSEQ);
    end
  end

  assign fetch_err_o = hresp_i & hready_i;

  // ---------------------------------------------------------------------------
  // RVC half-word alignment handling
  // ---------------------------------------------------------------------------
  // If PC[1]=1 the instruction starts in the upper half of the fetched word.
  // A 32-bit instruction then spans two fetch words — we buffer the high half.
  logic [15:0] hold_reg;   // Holds upper 16 bits while fetching second word
  logic        hold_valid;
  logic [31:0] raw_instr;
  logic        is_comp_raw;

  always_comb begin
    if (hold_valid)
      raw_instr = {fetch_data[15:0], hold_reg};  // Reassemble 32-bit span
    else if (pc_r[1])
      raw_instr = {16'hNOP, fetch_data[31:16]};  // Upper half first
    else
      raw_instr = fetch_data;
  end

  // RVC: opcode[1:0] != 2'b11 → 16-bit instruction
  assign is_comp_raw = ENABLE_COMPRESSED & (raw_instr[1:0] != 2'b11);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      hold_reg   <= '0;
      hold_valid <= 1'b0;
    end else if (hready_i && !stall_i) begin
      if (pc_r[1] && !is_comp_raw) begin
        // 32-bit instruction starting at [1] → buffer upper 16 bits
        hold_reg   <= fetch_data[31:16];
        hold_valid <= 1'b1;
      end else begin
        hold_valid <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // C-Extension Decompressor (RVC → RVI)
  // ---------------------------------------------------------------------------
  logic [31:0] decompressed;

  generate if (ENABLE_COMPRESSED) begin : gen_rvc
    rv32emc_compress_dec u_compress (
      .instr16_i  (raw_instr[15:0]),
      .instr32_o  (decompressed),
      .illegal_o  (/* propagated via id_ex_reg.illegal */)
    );
  end else begin : gen_no_rvc
    assign decompressed = raw_instr;
  end endgenerate

  // ---------------------------------------------------------------------------
  // 2-bit saturating counter Branch Prediction Unit
  // ---------------------------------------------------------------------------
  generate if (ENABLE_BPU) begin : gen_bpu
    rv32emc_bpu #(.ENTRIES(BPU_ENTRIES)) u_bpu (
      .clk_i            (clk_i),
      .rst_ni           (rst_ni),
      // Prediction request
      .fetch_pc_i       (pc_r),
      .predict_taken_o  (bpu_predict_taken),
      .predict_target_o (bpu_target),
      // Training (from EX stage branch resolution)
      .update_en_i      (br_taken_i | flush_i),
      .update_pc_i      (pc_r),        // PC of resolved branch
      .update_taken_i   (br_taken_i),
      .update_target_i  (br_target_i)
    );
  end else begin : gen_no_bpu
    assign bpu_predict_taken = 1'b0;
    assign bpu_target        = '0;
  end endgenerate

  // ---------------------------------------------------------------------------
  // Output assignments
  // ---------------------------------------------------------------------------
  assign pc_o      = pc_r;
  assign instr_o   = is_comp_raw ? decompressed : raw_instr;
  assign valid_o   = fetch_valid & ~stall_i & ~flush_i;
  assign is_comp_o = is_comp_raw;

endmodule : rv32emc_if_stage


// =============================================================================
// rv32emc_compress_dec.sv — RVC → RVI Decompressor (embedded in same file)
// =============================================================================
// Implements RISC-V C-extension ISA v2.0 decompression for:
//   RVC Quadrant 0: C.ADDI4SPN, C.LW, C.SW
//   RVC Quadrant 1: C.NOP, C.ADDI, C.JAL, C.LI, C.ADDI16SP, C.LUI,
//                   C.SRLI, C.SRAI, C.ANDI, C.SUB, C.XOR, C.OR, C.AND,
//                   C.J, C.BEQZ, C.BNEZ
//   RVC Quadrant 2: C.SLLI, C.LWSP, C.JR, C.MV, C.EBREAK, C.JALR,
//                   C.ADD, C.SWSP
// =============================================================================

module rv32emc_compress_dec (
  input  logic [15:0] instr16_i,
  output logic [31:0] instr32_o,
  output logic        illegal_o
);

  // CIW / CL / CS / CA / CB / CJ / CSS field extraction
  logic [4:0] rs1c, rs2c, rdc;   // 3-bit reg offset → x8..x15
  logic [4:0] rs1,  rs2,  rd;    // Full 5-bit reg

  // CL/CS compressed regs map to x8..x15
  assign rs1c = {2'b01, instr16_i[9:7]};
  assign rs2c = {2'b01, instr16_i[4:2]};
  assign rdc  = {2'b01, instr16_i[4:2]};

  // CR/CI full registers
  assign rs1  = instr16_i[11:7];
  assign rs2  = instr16_i[6:2];
  assign rd   = instr16_i[11:7];

  logic [31:0] imm_ciw, imm_cl, imm_cs, imm_ci, imm_cb, imm_cj;
  logic [31:0] imm_css, imm_caddi16sp;

  // CIW: addi4spn
  assign imm_ciw = {22'd0, instr16_i[10:7], instr16_i[12:11], instr16_i[5], instr16_i[6], 2'b00};

  // CL/CS: lw/sw
  assign imm_cl  = {25'd0, instr16_i[5], instr16_i[12:10], instr16_i[6], 2'b00};
  assign imm_cs  = {25'd0, instr16_i[5], instr16_i[12:10], instr16_i[6], 2'b00};

  // CI immediate (sign-extended 6-bit)
  assign imm_ci  = {{26{instr16_i[12]}}, instr16_i[12], instr16_i[6:2]};

  // CADDI16SP
  assign imm_caddi16sp = {{22{instr16_i[12]}}, instr16_i[12], instr16_i[4:3],
                           instr16_i[5], instr16_i[2], instr16_i[6], 4'b0000};

  // CB: beqz/bnez
  assign imm_cb  = {{23{instr16_i[12]}}, instr16_i[12], instr16_i[6:5],
                     instr16_i[2], instr16_i[11:10], instr16_i[4:3], 1'b0};

  // CJ: j/jal
  assign imm_cj  = {{20{instr16_i[12]}}, instr16_i[12], instr16_i[8],
                     instr16_i[10:9], instr16_i[6], instr16_i[7],
                     instr16_i[2], instr16_i[11], instr16_i[5:3], 1'b0};

  // CSS: swsp
  assign imm_css = {24'd0, instr16_i[8:7], instr16_i[12:9], 2'b00};

  // LWSP
  logic [31:0] imm_lwsp;
  assign imm_lwsp = {24'd0, instr16_i[3:2], instr16_i[12], instr16_i[6:4], 2'b00};

  always_comb begin
    instr32_o = 32'h0000_0013;  // Default: NOP (ADDI x0,x0,0)
    illegal_o = 1'b0;

    unique casez ({instr16_i[15:13], instr16_i[1:0]})
      // ---- Quadrant 0 ----
      5'b000_00: begin // C.ADDI4SPN → ADDI rd′, x2, nzuimm
        if (imm_ciw == 0) illegal_o = 1'b1;
        instr32_o = {imm_ciw[11:0], 5'd2, 3'b000, rdc, 7'b001_0011};
      end
      5'b010_00: begin // C.LW → LW rd′, offset(rs1′)
        instr32_o = {imm_cl[11:0], rs1c, 3'b010, rdc, 7'b000_0011};
      end
      5'b110_00: begin // C.SW → SW rs2′, offset(rs1′)
        instr32_o = {imm_cs[11:5], rs2c, rs1c, 3'b010, imm_cs[4:0], 7'b010_0011};
      end

      // ---- Quadrant 1 ----
      5'b000_01: begin // C.ADDI / C.NOP
        instr32_o = {imm_ci[11:0], rd, 3'b000, rd, 7'b001_0011};
      end
      5'b001_01: begin // C.JAL (RV32 only) → JAL x1, offset
        instr32_o = {imm_cj[20], imm_cj[10:1], imm_cj[11], imm_cj[19:12], 5'd1, 7'b110_1111};
      end
      5'b010_01: begin // C.LI → ADDI rd, x0, imm
        instr32_o = {imm_ci[11:0], 5'd0, 3'b000, rd, 7'b001_0011};
      end
      5'b011_01: begin // C.ADDI16SP / C.LUI
        if (rd == 5'd2)
          instr32_o = {imm_caddi16sp[11:0], 5'd2, 3'b000, 5'd2, 7'b001_0011};
        else begin
          if (imm_ci == 0) illegal_o = 1'b1;
          instr32_o = {imm_ci[31:12], rd, 7'b011_0111};
        end
      end
      5'b100_01: begin // C.SRLI / C.SRAI / C.ANDI / C.SUB/XOR/OR/AND
        unique casez ({instr16_i[11:10], instr16_i[6:5]})
          4'b00??: instr32_o = {6'd0, instr16_i[12], instr16_i[6:2], rs1c, 3'b101, rs1c, 7'b001_0011}; // SRLI
          4'b01??: instr32_o = {6'b010000, instr16_i[12], instr16_i[6:2], rs1c, 3'b101, rs1c, 7'b001_0011}; // SRAI
          4'b10??: instr32_o = {imm_ci[11:0], rs1c, 3'b111, rs1c, 7'b001_0011}; // ANDI
          4'b1100: instr32_o = {7'b010_0000, rs2c, rs1c, 3'b000, rs1c, 7'b011_0011}; // SUB
          4'b1101: instr32_o = {7'b000_0000, rs2c, rs1c, 3'b100, rs1c, 7'b011_0011}; // XOR
          4'b1110: instr32_o = {7'b000_0000, rs2c, rs1c, 3'b110, rs1c, 7'b011_0011}; // OR
          4'b1111: instr32_o = {7'b000_0000, rs2c, rs1c, 3'b111, rs1c, 7'b011_0011}; // AND
          default: illegal_o = 1'b1;
        endcase
      end
      5'b101_01: // C.J → JAL x0, offset
        instr32_o = {imm_cj[20], imm_cj[10:1], imm_cj[11], imm_cj[19:12], 5'd0, 7'b110_1111};
      5'b110_01: // C.BEQZ → BEQ rs1′, x0, offset
        instr32_o = {imm_cb[12], imm_cb[10:5], 5'd0, rs1c, 3'b000, imm_cb[4:1], imm_cb[11], 7'b110_0011};
      5'b111_01: // C.BNEZ → BNE rs1′, x0, offset
        instr32_o = {imm_cb[12], imm_cb[10:5], 5'd0, rs1c, 3'b001, imm_cb[4:1], imm_cb[11], 7'b110_0011};

      // ---- Quadrant 2 ----
      5'b000_10: // C.SLLI → SLLI rd, rd, shamt
        instr32_o = {6'd0, instr16_i[12], instr16_i[6:2], rd, 3'b001, rd, 7'b001_0011};
      5'b010_10: // C.LWSP → LW rd, offset(x2)
        instr32_o = {imm_lwsp[11:0], 5'd2, 3'b010, rd, 7'b000_0011};
      5'b100_10: begin // C.JR / C.MV / C.EBREAK / C.JALR / C.ADD
        if (!instr16_i[12]) begin
          if (rs2 == 0)
            instr32_o = {12'd0, rd, 3'b000, 5'd0, 7'b110_0111}; // C.JR
          else
            instr32_o = {7'd0, rs2, 5'd0, 3'b000, rd, 7'b011_0011}; // C.MV
        end else begin
          if (rd == 0 && rs2 == 0)
            instr32_o = 32'h0010_0073; // C.EBREAK
          else if (rs2 == 0)
            instr32_o = {12'd0, rd, 3'b000, 5'd1, 7'b110_0111}; // C.JALR
          else
            instr32_o = {7'd0, rs2, rd, 3'b000, rd, 7'b011_0011}; // C.ADD
        end
      end
      5'b110_10: // C.SWSP → SW rs2, offset(x2)
        instr32_o = {imm_css[11:5], rs2, 5'd2, 3'b010, imm_css[4:0], 7'b010_0011};

      default: illegal_o = 1'b1;
    endcase
  end

endmodule : rv32emc_compress_dec
