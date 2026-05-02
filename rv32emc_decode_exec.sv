// =============================================================================
// rv32emc_decode.sv — Instruction Decode Stage
// =============================================================================

import rv32emc_pkg::*;

module rv32emc_decode #(
  parameter bit ENABLE_MUL        = 1,
  parameter bit ENABLE_DIV        = 1,
  parameter bit ENABLE_COMPRESSED = 1
)(
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic        valid_i,
  input  logic [31:0] pc_i,
  input  logic [31:0] instr_i,    // Decompressed 32-bit instruction
  input  logic        is_comp_i,
  input  logic        fetch_err_i,

  // Register file read
  output logic [REG_AW-1:0] rf_rs1_addr_o,
  output logic [REG_AW-1:0] rf_rs2_addr_o,
  input  logic [31:0]       rf_rs1_data_i,
  input  logic [31:0]       rf_rs2_data_i,

  // Forwarded operands
  input  fwd_sel_e    fwd_a_sel_i,
  input  fwd_sel_e    fwd_b_sel_i,
  input  logic [31:0] fwd_a_data_i,
  input  logic [31:0] fwd_b_data_i,

  input  logic        stall_i,
  input  logic        flush_i,

  output id_ex_bundle_t id_ex_o
);

  logic [6:0]  opcode;
  logic [2:0]  funct3;
  logic [6:0]  funct7;
  logic [4:0]  rs1_idx, rs2_idx, rd_idx;
  logic [11:0] csr_addr;

  assign opcode   = instr_i[6:0];
  assign funct3   = instr_i[14:12];
  assign funct7   = instr_i[31:25];
  assign rs1_idx  = instr_i[19:15];
  assign rs2_idx  = instr_i[24:20];
  assign rd_idx   = instr_i[11:7];
  assign csr_addr = instr_i[31:20];

  // Force upper bits to 0 for RV32E (only x0–x15 valid)
  assign rf_rs1_addr_o = rs1_idx[REG_AW-1:0];
  assign rf_rs2_addr_o = rs2_idx[REG_AW-1:0];

  // ---------------------------------------------------------------------------
  // Immediate generation
  // ---------------------------------------------------------------------------
  logic [31:0] imm_i, imm_s, imm_b, imm_u, imm_j;

  assign imm_i = {{20{instr_i[31]}}, instr_i[31:20]};
  assign imm_s = {{20{instr_i[31]}}, instr_i[31:25], instr_i[11:7]};
  assign imm_b = {{19{instr_i[31]}}, instr_i[31], instr_i[7], instr_i[30:25], instr_i[11:8], 1'b0};
  assign imm_u = {instr_i[31:12], 12'd0};
  assign imm_j = {{11{instr_i[31]}}, instr_i[31], instr_i[19:12], instr_i[20], instr_i[30:21], 1'b0};

  // ---------------------------------------------------------------------------
  // Main decode
  // ---------------------------------------------------------------------------
  id_ex_bundle_t dec;

  always_comb begin : decode_logic
    dec          = '0;
    dec.pc       = pc_i;
    dec.instr    = instr_i;
    dec.is_compressed = is_comp_i;
    dec.rs1      = rs1_idx[REG_AW-1:0];
    dec.rs2      = rs2_idx[REG_AW-1:0];
    dec.rd       = rd_idx[REG_AW-1:0];
    dec.csr_addr = csr_addr;
    dec.illegal  = fetch_err_i | ~valid_i;

    // RV32E: trap on register index > 15
    if (rs1_idx[4] | rs2_idx[4] | rd_idx[4]) dec.illegal = 1'b1;

    unique case (opcode)
      // ------- OP-IMM -------
      OPC_OP_IMM: begin
        dec.rf_we  = 1'b1;
        dec.opa_src= OPA_RS1;
        dec.opb_src= OPB_IMM;
        dec.wb_src = WB_ALU;
        dec.imm    = imm_i;
        unique case (funct3)
          3'b000: dec.alu_op = ALU_ADD;
          3'b010: dec.alu_op = ALU_SLT;
          3'b011: dec.alu_op = ALU_SLTU;
          3'b100: dec.alu_op = ALU_XOR;
          3'b110: dec.alu_op = ALU_OR;
          3'b111: dec.alu_op = ALU_AND;
          3'b001: dec.alu_op = ALU_SLL;
          3'b101: dec.alu_op = funct7[5] ? ALU_SRA : ALU_SRL;
          default: dec.illegal = 1'b1;
        endcase
      end

      // ------- OP (R-type + M-ext) -------
      OPC_OP: begin
        dec.rf_we  = 1'b1;
        dec.opa_src= OPA_RS1;
        dec.opb_src= OPB_RS2;
        dec.wb_src = WB_ALU;
        if (funct7 == 7'b000_0001) begin // M-extension
          if (!ENABLE_MUL && !ENABLE_DIV) dec.illegal = 1'b1;
          unique case (funct3)
            3'b000: begin dec.alu_op = ALU_MUL;    dec.wb_src = WB_MUL; end
            3'b001: begin dec.alu_op = ALU_MULH;   dec.wb_src = WB_MUL; end
            3'b010: begin dec.alu_op = ALU_MULHSU; dec.wb_src = WB_MUL; end
            3'b011: begin dec.alu_op = ALU_MULHU;  dec.wb_src = WB_MUL; end
            3'b100: begin dec.alu_op = ALU_DIV;    dec.wb_src = WB_DIV; if (!ENABLE_DIV) dec.illegal = 1'b1; end
            3'b101: begin dec.alu_op = ALU_DIVU;   dec.wb_src = WB_DIV; if (!ENABLE_DIV) dec.illegal = 1'b1; end
            3'b110: begin dec.alu_op = ALU_REM;    dec.wb_src = WB_DIV; if (!ENABLE_DIV) dec.illegal = 1'b1; end
            3'b111: begin dec.alu_op = ALU_REMU;   dec.wb_src = WB_DIV; if (!ENABLE_DIV) dec.illegal = 1'b1; end
            default: dec.illegal = 1'b1;
          endcase
        end else begin
          unique case ({funct7, funct3})
            {7'b000_0000, 3'b000}: dec.alu_op = ALU_ADD;
            {7'b010_0000, 3'b000}: dec.alu_op = ALU_SUB;
            {7'b000_0000, 3'b001}: dec.alu_op = ALU_SLL;
            {7'b000_0000, 3'b010}: dec.alu_op = ALU_SLT;
            {7'b000_0000, 3'b011}: dec.alu_op = ALU_SLTU;
            {7'b000_0000, 3'b100}: dec.alu_op = ALU_XOR;
            {7'b000_0000, 3'b101}: dec.alu_op = ALU_SRL;
            {7'b010_0000, 3'b101}: dec.alu_op = ALU_SRA;
            {7'b000_0000, 3'b110}: dec.alu_op = ALU_OR;
            {7'b000_0000, 3'b111}: dec.alu_op = ALU_AND;
            default: dec.illegal = 1'b1;
          endcase
        end
      end

      // ------- LUI -------
      OPC_LUI: begin
        dec.rf_we   = 1'b1;
        dec.opa_src = OPA_ZERO;
        dec.opb_src = OPB_IMM;
        dec.alu_op  = ALU_LUI;
        dec.wb_src  = WB_ALU;
        dec.imm     = imm_u;
      end

      // ------- AUIPC -------
      OPC_AUIPC: begin
        dec.rf_we   = 1'b1;
        dec.opa_src = OPA_PC;
        dec.opb_src = OPB_IMM;
        dec.alu_op  = ALU_ADD;
        dec.wb_src  = WB_ALU;
        dec.imm     = imm_u;
      end

      // ------- JAL -------
      OPC_JAL: begin
        dec.rf_we   = (rd_idx != '0);
        dec.opa_src = OPA_PC;
        dec.opb_src = OPB_IMM;
        dec.alu_op  = ALU_ADD;
        dec.wb_src  = WB_PC4;
        dec.br_type = BR_JAL;
        dec.imm     = imm_j;
      end

      // ------- JALR -------
      OPC_JALR: begin
        if (funct3 != 3'b000) dec.illegal = 1'b1;
        dec.rf_we   = (rd_idx != '0);
        dec.opa_src = OPA_RS1;
        dec.opb_src = OPB_IMM;
        dec.alu_op  = ALU_ADD;
        dec.wb_src  = WB_PC4;
        dec.br_type = BR_JAL;
        dec.imm     = imm_i;
      end

      // ------- BRANCH -------
      OPC_BRANCH: begin
        dec.opa_src = OPA_PC;
        dec.opb_src = OPB_IMM;
        dec.alu_op  = ALU_ADD;  // Branch target = PC + imm_b
        dec.imm     = imm_b;
        unique case (funct3)
          3'b000: dec.br_type = BR_EQ;
          3'b001: dec.br_type = BR_NE;
          3'b100: dec.br_type = BR_LT;
          3'b101: dec.br_type = BR_GE;
          3'b110: dec.br_type = BR_LTU;
          3'b111: dec.br_type = BR_GEU;
          default: dec.illegal = 1'b1;
        endcase
      end

      // ------- LOAD -------
      OPC_LOAD: begin
        dec.rf_we   = 1'b1;
        dec.opa_src = OPA_RS1;
        dec.opb_src = OPB_IMM;
        dec.alu_op  = ALU_ADD;
        dec.wb_src  = WB_MEM;
        dec.mem_read= 1'b1;
        dec.imm     = imm_i;
        unique case (funct3)
          3'b000: dec.mem_size = MEM_BYTE;
          3'b001: dec.mem_size = MEM_HALF;
          3'b010: dec.mem_size = MEM_WORD;
          3'b100: dec.mem_size = MEM_BYTEU;
          3'b101: dec.mem_size = MEM_HALFU;
          default: dec.illegal = 1'b1;
        endcase
      end

      // ------- STORE -------
      OPC_STORE: begin
        dec.opa_src  = OPA_RS1;
        dec.opb_src  = OPB_IMM;
        dec.alu_op   = ALU_ADD;
        dec.wb_src   = WB_NONE;
        dec.mem_write= 1'b1;
        dec.imm      = imm_s;
        unique case (funct3)
          3'b000: dec.mem_size = MEM_BYTE;
          3'b001: dec.mem_size = MEM_HALF;
          3'b010: dec.mem_size = MEM_WORD;
          default: dec.illegal = 1'b1;
        endcase
      end

      // ------- SYSTEM (CSR / ECALL / EBREAK / MRET / WFI) -------
      OPC_SYSTEM: begin
        unique case (funct3)
          3'b000: begin
            unique case (instr_i[31:7])
              25'h0000073: dec.ecall  = 1'b1;
              25'h0010073: dec.ebreak = 1'b1;
              25'h3020073: dec.mret   = 1'b1;
              25'h1050073: dec.wfi    = 1'b1;
              default:     dec.illegal= 1'b1;
            endcase
          end
          3'b001, 3'b010, 3'b011,  // CSRRW/S/C
          3'b101, 3'b110, 3'b111: begin // CSRRWI/SI/CI
            dec.csr_en  = 1'b1;
            dec.rf_we   = (rd_idx != '0);
            dec.wb_src  = WB_CSR;
            dec.csr_op  = funct3[1:0] - 2'd1; // {rw=0, set=1, clr=2}
            // For CSRRWI/SI/CI, rs1 field is zero-extended uimm[4:0]
          end
          default: dec.illegal = 1'b1;
        endcase
      end

      // ------- MISC-MEM (FENCE) -------
      OPC_MISC_MEM: dec.fence = 1'b1; // Treated as NOP in simple core

      default: dec.illegal = 1'b1;
    endcase

    if (flush_i) dec = '0;
  end

  // Output registration
  assign id_ex_o = dec;

endmodule : rv32emc_decode


// =============================================================================
// rv32emc_execute.sv — Execute Stage (ALU + branch resolution + MUL kick-off)
// =============================================================================

module rv32emc_execute #(
  parameter bit ENABLE_MUL = 1,
  parameter bit ENABLE_DIV = 1
)(
  input  logic          clk_i,
  input  logic          rst_ni,
  input  id_ex_bundle_t id_ex_i,

  output logic [31:0]   alu_result_o,
  output logic [31:0]   mul_result_o,
  output logic [31:0]   div_result_o,
  output logic          mul_valid_o,
  output logic          div_valid_o,
  output logic          mul_busy_o,
  output logic          div_busy_o,

  output logic          br_taken_o,
  output logic [31:0]   br_target_o,
  output ex_mem_bundle_t ex_mem_o
);

  logic [31:0] opa, opb;

  // Operand A
  always_comb begin
    unique case (id_ex_i.opa_src)
      OPA_RS1 : opa = 32'hDEAD_DEAD; // driven by forwarded value from hazard unit
      OPA_PC  : opa = id_ex_i.pc;
      OPA_ZERO: opa = '0;
      default : opa = '0;
    endcase
  end

  // Operand B
  always_comb begin
    unique case (id_ex_i.opb_src)
      OPB_RS2 : opb = 32'hDEAD_BEEF; // driven by forwarded value
      OPB_IMM : opb = id_ex_i.imm;
      OPB_FOUR: opb = 32'd4;
      default : opb = '0;
    endcase
  end

  // ALU
  rv32emc_alu u_alu (
    .alu_op_i   (id_ex_i.alu_op),
    .a_i        (opa),
    .b_i        (opb),
    .result_o   (alu_result_o)
  );

  // Branch condition evaluation
  logic rs1_lt_rs2_s, rs1_lt_rs2_u;
  assign rs1_lt_rs2_s = $signed(opa) < $signed(opb);
  assign rs1_lt_rs2_u = opa < opb;

  always_comb begin
    br_taken_o = 1'b0;
    unique case (id_ex_i.br_type)
      BR_JAL : br_taken_o = 1'b1;
      BR_EQ  : br_taken_o = (opa == opb);
      BR_NE  : br_taken_o = (opa != opb);
      BR_LT  : br_taken_o = rs1_lt_rs2_s;
      BR_GE  : br_taken_o = ~rs1_lt_rs2_s;
      BR_LTU : br_taken_o = rs1_lt_rs2_u;
      BR_GEU : br_taken_o = ~rs1_lt_rs2_u;
      default: br_taken_o = 1'b0;
    endcase
  end

  // JALR clears LSB of target per spec
  assign br_target_o = (id_ex_i.br_type == BR_JAL && id_ex_i.instr[6:0] == 7'b110_0111)
                       ? {alu_result_o[31:1], 1'b0}
                       : alu_result_o;

  // MUL/DIV units
  generate if (ENABLE_MUL) begin : gen_mul
    rv32emc_mul u_mul (
      .clk_i      (clk_i),
      .rst_ni     (rst_ni),
      .start_i    (id_ex_i.wb_src == WB_MUL),
      .op_i       (id_ex_i.alu_op),
      .a_i        (opa),
      .b_i        (opb),
      .result_o   (mul_result_o),
      .valid_o    (mul_valid_o),
      .busy_o     (mul_busy_o)
    );
  end else begin
    assign mul_result_o = '0;
    assign mul_valid_o  = 1'b0;
    assign mul_busy_o   = 1'b0;
  end endgenerate

  generate if (ENABLE_DIV) begin : gen_div
    rv32emc_div u_div (
      .clk_i      (clk_i),
      .rst_ni     (rst_ni),
      .start_i    (id_ex_i.wb_src == WB_DIV),
      .op_i       (id_ex_i.alu_op),
      .a_i        (opa),
      .b_i        (opb),
      .result_o   (div_result_o),
      .valid_o    (div_valid_o),
      .busy_o     (div_busy_o)
    );
  end else begin
    assign div_result_o = '0;
    assign div_valid_o  = 1'b0;
    assign div_busy_o   = 1'b0;
  end endgenerate

  // Pack EX/MEM bundle
  always_comb begin
    ex_mem_o.pc         = id_ex_i.pc;
    ex_mem_o.alu_result = (id_ex_i.wb_src == WB_MUL) ? mul_result_o :
                          (id_ex_i.wb_src == WB_DIV) ? div_result_o :
                          alu_result_o;
    ex_mem_o.rs2_data   = 32'h0; // filled by forwarding in hazard unit
    ex_mem_o.mul_result = mul_result_o;
    ex_mem_o.br_taken   = br_taken_o;
    ex_mem_o.br_target  = br_target_o;
    ex_mem_o.wb_src     = id_ex_i.wb_src;
    ex_mem_o.mem_size   = id_ex_i.mem_size;
    ex_mem_o.mem_read   = id_ex_i.mem_read;
    ex_mem_o.mem_write  = id_ex_i.mem_write;
    ex_mem_o.rf_we      = id_ex_i.rf_we;
    ex_mem_o.rd         = id_ex_i.rd;
    ex_mem_o.csr_en     = id_ex_i.csr_en;
    ex_mem_o.csr_rdata  = '0; // filled by CSR block
    ex_mem_o.illegal    = id_ex_i.illegal;
    ex_mem_o.ecall      = id_ex_i.ecall;
    ex_mem_o.ebreak     = id_ex_i.ebreak;
  end

endmodule : rv32emc_execute


// =============================================================================
// rv32emc_alu.sv — 32-bit ALU
// =============================================================================

module rv32emc_alu
  import rv32emc_pkg::*;
(
  input  alu_op_e     alu_op_i,
  input  logic [31:0] a_i,
  input  logic [31:0] b_i,
  output logic [31:0] result_o
);

  logic [4:0] shamt;
  assign shamt = b_i[4:0];

  always_comb begin
    result_o = '0;
    unique case (alu_op_i)
      ALU_ADD  : result_o = a_i + b_i;
      ALU_SUB  : result_o = a_i - b_i;
      ALU_AND  : result_o = a_i & b_i;
      ALU_OR   : result_o = a_i | b_i;
      ALU_XOR  : result_o = a_i ^ b_i;
      ALU_SLL  : result_o = a_i << shamt;
      ALU_SRL  : result_o = a_i >> shamt;
      ALU_SRA  : result_o = $signed(a_i) >>> shamt;
      ALU_SLT  : result_o = {31'd0, $signed(a_i) < $signed(b_i)};
      ALU_SLTU : result_o = {31'd0, a_i < b_i};
      ALU_LUI  : result_o = b_i;
      ALU_COPY_A: result_o= a_i;
      default  : result_o = '0;
    endcase
  end

endmodule : rv32emc_alu


// =============================================================================
// rv32emc_mul.sv — 2-cycle pipelined multiplier (32×32 → 64-bit)
// =============================================================================

module rv32emc_mul
  import rv32emc_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        start_i,
  input  alu_op_e     op_i,
  input  logic [31:0] a_i,
  input  logic [31:0] b_i,
  output logic [31:0] result_o,
  output logic        valid_o,
  output logic        busy_o
);

  // Sign-extend to 33 bits to handle MULHSU
  logic signed [32:0] a_ext, b_ext;
  logic [63:0] product;
  logic [63:0] product_r;
  logic        valid_r1, valid_r2;
  alu_op_e     op_r1, op_r2;

  always_comb begin
    unique case (op_i)
      ALU_MUL, ALU_MULH:   begin a_ext = {a_i[31], a_i}; b_ext = {b_i[31], b_i}; end
      ALU_MULHSU:           begin a_ext = {a_i[31], a_i}; b_ext = {1'b0, b_i};    end
      ALU_MULHU:            begin a_ext = {1'b0, a_i};    b_ext = {1'b0, b_i};    end
      default:              begin a_ext = '0; b_ext = '0; end
    endcase
  end

  // Stage 1: multiply
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      product_r <= '0; valid_r1 <= '0; op_r1 <= ALU_NOP;
    end else begin
      product_r <= $signed(a_ext) * $signed(b_ext);
      valid_r1  <= start_i;
      op_r1     <= op_i;
    end
  end

  // Stage 2: select high/low
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin valid_r2 <= '0; op_r2 <= ALU_NOP; end
    else begin valid_r2 <= valid_r1; op_r2 <= op_r1; end
  end

  always_comb begin
    unique case (op_r2)
      ALU_MUL   : result_o = product_r[31:0];
      ALU_MULH,
      ALU_MULHSU,
      ALU_MULHU : result_o = product_r[63:32];
      default   : result_o = '0;
    endcase
  end

  assign valid_o = valid_r2;
  assign busy_o  = valid_r1;

endmodule : rv32emc_mul


// =============================================================================
// rv32emc_regfile.sv — 16×32 register file (RV32E)
// =============================================================================

module rv32emc_regfile
  import rv32emc_pkg::*;
(
  input  logic              clk_i,
  input  logic              rst_ni,
  // Normal dual read port
  input  logic [REG_AW-1:0] rs1_addr_i,
  input  logic [REG_AW-1:0] rs2_addr_i,
  output logic [31:0]       rs1_data_o,
  output logic [31:0]       rs2_data_o,
  // Single write port
  input  logic              rd_we_i,
  input  logic [REG_AW-1:0] rd_addr_i,
  input  logic [31:0]       rd_data_i,
  // Debug abstract command access (priority over normal write)
  input  logic              dbg_rd_en_i,
  input  logic              dbg_wr_en_i,
  input  logic [REG_AW-1:0] dbg_addr_i,
  input  logic [31:0]       dbg_wdata_i,
  output logic [31:0]       dbg_rdata_o
);

  logic [31:0] regs [0:NUM_REGS-1];

  // x0 hardwired to 0
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < NUM_REGS; i++) regs[i] <= '0;
    end else begin
      if (dbg_wr_en_i)
        regs[dbg_addr_i] <= dbg_wdata_i;
      else if (rd_we_i && rd_addr_i != '0)
        regs[rd_addr_i] <= rd_data_i;
    end
  end

  // Async read — forward write-through for same-cycle write
  assign rs1_data_o = (rs1_addr_i == '0) ? '0 :
                      (rd_we_i && rs1_addr_i == rd_addr_i) ? rd_data_i :
                      regs[rs1_addr_i];

  assign rs2_data_o = (rs2_addr_i == '0) ? '0 :
                      (rd_we_i && rs2_addr_i == rd_addr_i) ? rd_data_i :
                      regs[rs2_addr_i];

  assign dbg_rdata_o = regs[dbg_addr_i];

endmodule : rv32emc_regfile


// =============================================================================
// rv32emc_hazard.sv — Hazard detection, stall/flush, forwarding
// =============================================================================

module rv32emc_hazard
  import rv32emc_pkg::*;
(
  input  logic [REG_AW-1:0] id_rs1_i,
  input  logic [REG_AW-1:0] id_rs2_i,
  input  logic [REG_AW-1:0] ex_rd_i,
  input  logic              ex_rf_we_i,
  input  logic              ex_mem_read_i,
  input  logic [REG_AW-1:0] mem_rd_i,
  input  logic              mem_rf_we_i,

  input  logic              mul_busy_i,
  input  logic              div_busy_i,
  input  logic              ibus_stall_i,
  input  logic              lsu_stall_i,

  output fwd_sel_e          fwd_a_sel_o,
  output fwd_sel_e          fwd_b_sel_o,
  input  logic [31:0]       ex_alu_result_i,
  input  logic [31:0]       mem_alu_result_i,
  input  logic [31:0]       mem_rdata_i,
  input  logic [31:0]       wb_data_i,
  output logic [31:0]       fwd_a_data_o,
  output logic [31:0]       fwd_b_data_o,

  output logic              stall_if_o,
  output logic              stall_id_o,
  output logic              flush_if_o,
  output logic              flush_id_o,
  output logic              flush_ex_o
);

  // Forwarding — EX/MEM takes priority over MEM/WB
  always_comb begin
    fwd_a_sel_o = FWD_REG;
    if      (ex_rf_we_i  && ex_rd_i  != '0 && ex_rd_i  == id_rs1_i) fwd_a_sel_o = FWD_EX_MEM;
    else if (mem_rf_we_i && mem_rd_i != '0 && mem_rd_i == id_rs1_i) fwd_a_sel_o = FWD_MEM_WB;

    fwd_b_sel_o = FWD_REG;
    if      (ex_rf_we_i  && ex_rd_i  != '0 && ex_rd_i  == id_rs2_i) fwd_b_sel_o = FWD_EX_MEM;
    else if (mem_rf_we_i && mem_rd_i != '0 && mem_rd_i == id_rs2_i) fwd_b_sel_o = FWD_MEM_WB;
  end

  // Forward data mux
  assign fwd_a_data_o = (fwd_a_sel_o == FWD_EX_MEM) ? ex_alu_result_i :
                        (fwd_a_sel_o == FWD_MEM_WB) ? mem_alu_result_i :
                        wb_data_i;

  assign fwd_b_data_o = (fwd_b_sel_o == FWD_EX_MEM) ? ex_alu_result_i :
                        (fwd_b_sel_o == FWD_MEM_WB) ? mem_alu_result_i :
                        wb_data_i;

  // Load-use hazard: stall for 1 cycle
  logic load_use;
  assign load_use = ex_mem_read_i && ex_rf_we_i &&
                    (ex_rd_i == id_rs1_i || ex_rd_i == id_rs2_i);

  // Global stall
  logic any_stall;
  assign any_stall = load_use | mul_busy_i | div_busy_i | ibus_stall_i | lsu_stall_i;

  assign stall_if_o = any_stall;
  assign stall_id_o = any_stall;
  assign flush_if_o = 1'b0; // Branch flush is separate (from EX stage)
  assign flush_id_o = load_use;
  assign flush_ex_o = 1'b0;

endmodule : rv32emc_hazard
