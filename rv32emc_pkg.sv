// =============================================================================
// rv32emc_pkg.sv — RISC-V RV32EMC Core Package
// -----------------------------------------------------------------------------
// ISA    : RV32E (16 GPRs) + M (MUL/DIV) + C (16-bit compressed)
// Spec   : RISC-V Unprivileged ISA v20191213
//          RISC-V Privileged ISA v20211203
//          RISC-V Debug Spec 0.13.2
// Style  : Synthesisable SystemVerilog-2017
// =============================================================================

`ifndef RV32EMC_PKG_SV
`define RV32EMC_PKG_SV

package rv32emc_pkg;

  // ---------------------------------------------------------------------------
  // Core parameters (override at instantiation)
  // ---------------------------------------------------------------------------
  parameter int unsigned XLEN       = 32;   // Data/address width
  parameter int unsigned NUM_REGS   = 16;   // RV32E: 16 registers
  parameter int unsigned REG_AW     = 4;    // log2(NUM_REGS)
  parameter int unsigned RESET_ADDR = 32'h0000_0000; // Boot address
  parameter int unsigned MTVEC_BASE = 32'h0000_0100; // Default trap vector

  // ---------------------------------------------------------------------------
  // RISC-V Opcode map (inst[6:2])
  // ---------------------------------------------------------------------------
  typedef enum logic [6:0] {
    OPC_LOAD    = 7'b000_0011,
    OPC_LOAD_FP = 7'b000_0111,
    OPC_MISC_MEM= 7'b000_1111,
    OPC_OP_IMM  = 7'b001_0011,
    OPC_AUIPC   = 7'b001_0111,
    OPC_STORE   = 7'b010_0011,
    OPC_AMO     = 7'b010_1111,
    OPC_OP      = 7'b011_0011,
    OPC_LUI     = 7'b011_0111,
    OPC_BRANCH  = 7'b110_0011,
    OPC_JALR    = 7'b110_0111,
    OPC_JAL     = 7'b110_1111,
    OPC_SYSTEM  = 7'b111_0011
  } opcode_e;

  // ---------------------------------------------------------------------------
  // ALU operation select
  // ---------------------------------------------------------------------------
  typedef enum logic [4:0] {
    ALU_ADD   = 5'd0,
    ALU_SUB   = 5'd1,
    ALU_AND   = 5'd2,
    ALU_OR    = 5'd3,
    ALU_XOR   = 5'd4,
    ALU_SLL   = 5'd5,
    ALU_SRL   = 5'd6,
    ALU_SRA   = 5'd7,
    ALU_SLT   = 5'd8,   // signed less-than
    ALU_SLTU  = 5'd9,   // unsigned less-than
    ALU_LUI   = 5'd10,  // pass B (for LUI/AUIPC)
    ALU_COPY_A= 5'd11,  // pass A (for JALR)
    ALU_MUL   = 5'd12,  // routed to MUL unit
    ALU_MULH  = 5'd13,
    ALU_MULHSU= 5'd14,
    ALU_MULHU = 5'd15,
    ALU_DIV   = 5'd16,
    ALU_DIVU  = 5'd17,
    ALU_REM   = 5'd18,
    ALU_REMU  = 5'd19,
    ALU_NOP   = 5'd31
  } alu_op_e;

  // ---------------------------------------------------------------------------
  // Branch condition
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    BR_NONE = 3'b000,
    BR_EQ   = 3'b001,  // BEQ
    BR_NE   = 3'b010,  // BNE
    BR_LT   = 3'b011,  // BLT
    BR_GE   = 3'b100,  // BGE
    BR_LTU  = 3'b101,  // BLTU
    BR_GEU  = 3'b110,  // BGEU
    BR_JAL  = 3'b111   // Unconditional JAL/JALR
  } branch_e;

  // ---------------------------------------------------------------------------
  // Writeback source select
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    WB_ALU  = 3'b000,  // ALU result
    WB_MEM  = 3'b001,  // Load data
    WB_PC4  = 3'b010,  // PC+4 (JAL/JALR link)
    WB_CSR  = 3'b011,  // CSR read value
    WB_MUL  = 3'b100,  // Multiplier result
    WB_DIV  = 3'b101,  // Divider result
    WB_NONE = 3'b111
  } wb_src_e;

  // ---------------------------------------------------------------------------
  // Operand A/B source
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    OPA_RS1  = 2'b00,
    OPA_PC   = 2'b01,
    OPA_ZERO = 2'b10
  } opa_src_e;

  typedef enum logic [1:0] {
    OPB_RS2  = 2'b00,
    OPB_IMM  = 2'b01,
    OPB_FOUR = 2'b10  // constant 4 for PC+4 calc
  } opb_src_e;

  // ---------------------------------------------------------------------------
  // Memory access width
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    MEM_BYTE  = 3'b000,
    MEM_HALF  = 3'b001,
    MEM_WORD  = 3'b010,
    MEM_BYTEU = 3'b100,  // LBU
    MEM_HALFU = 3'b101   // LHU
  } mem_size_e;

  // ---------------------------------------------------------------------------
  // CSR addresses (M-mode only for RV32EMC)
  // ---------------------------------------------------------------------------
  typedef enum logic [11:0] {
    // Machine Information
    CSR_MVENDORID  = 12'hF11,
    CSR_MARCHID    = 12'hF12,
    CSR_MIMPID     = 12'hF13,
    CSR_MHARTID    = 12'hF14,
    // Machine Trap Setup
    CSR_MSTATUS    = 12'h300,
    CSR_MISA       = 12'h301,
    CSR_MIE        = 12'h304,
    CSR_MTVEC      = 12'h305,
    CSR_MCOUNTEREN = 12'h306,
    // Machine Trap Handling
    CSR_MSCRATCH   = 12'h340,
    CSR_MEPC       = 12'h341,
    CSR_MCAUSE     = 12'h342,
    CSR_MTVAL      = 12'h343,
    CSR_MIP        = 12'h344,
    // Machine Counters
    CSR_MCYCLE     = 12'hB00,
    CSR_MINSTRET   = 12'hB02,
    CSR_MCYCLEH    = 12'hB80,
    CSR_MINSTRETH  = 12'hB82,
    // Debug/Trace
    CSR_DCSR       = 12'h7B0,
    CSR_DPC        = 12'h7B1,
    CSR_DSCRATCH0  = 12'h7B2,
    CSR_DSCRATCH1  = 12'h7B3
  } csr_addr_e;

  // ---------------------------------------------------------------------------
  // Exception / Interrupt cause codes (mcause)
  // ---------------------------------------------------------------------------
  typedef enum logic [4:0] {
    EXC_IADDR_MISALIGN = 5'd0,
    EXC_IACCESS_FAULT  = 5'd1,
    EXC_ILLEGAL_INST   = 5'd2,
    EXC_BREAKPOINT     = 5'd3,
    EXC_LADDR_MISALIGN = 5'd4,
    EXC_LACCESS_FAULT  = 5'd5,
    EXC_SADDR_MISALIGN = 5'd6,
    EXC_SACCESS_FAULT  = 5'd7,
    EXC_ECALL_M        = 5'd11,
    IRQ_MSOFT          = 5'd3,   // when mcause[31]=1
    IRQ_MTIMER         = 5'd7,
    IRQ_MEXT           = 5'd11
  } exc_cause_e;

  // ---------------------------------------------------------------------------
  // Pipeline control — decoded instruction bundle (ID → EX)
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [31:0]  pc;          // Program counter of this instruction
    logic [31:0]  instr;       // Full 32-bit instruction (after C decompression)
    logic         is_compressed; // Original instruction was 16-bit
    alu_op_e      alu_op;      // ALU operation
    opa_src_e     opa_src;     // Operand A source
    opb_src_e     opb_src;     // Operand B source
    branch_e      br_type;     // Branch/jump type
    wb_src_e      wb_src;      // Writeback data source
    mem_size_e    mem_size;    // Memory access size
    logic         mem_read;    // LSU load enable
    logic         mem_write;   // LSU store enable
    logic         rf_we;       // Register file write enable
    logic [REG_AW-1:0] rs1;   // Source register 1 index
    logic [REG_AW-1:0] rs2;   // Source register 2 index
    logic [REG_AW-1:0] rd;    // Destination register index
    logic [31:0]  imm;         // Sign-extended immediate
    logic         csr_en;      // CSR instruction
    logic [1:0]   csr_op;      // 00=rw 01=set 10=clr
    logic [11:0]  csr_addr;    // CSR address
    logic         ecall;       // ECALL instruction
    logic         ebreak;      // EBREAK instruction
    logic         mret;        // MRET instruction
    logic         wfi;         // WFI instruction
    logic         fence;       // FENCE instruction
    logic         illegal;     // Illegal instruction detected
  } id_ex_bundle_t;

  // ---------------------------------------------------------------------------
  // EX → MEM pipeline register
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [31:0]  pc;
    logic [31:0]  alu_result;  // Primary ALU output
    logic [31:0]  rs2_data;    // Store data (forwarded)
    logic [31:0]  mul_result;  // Multiplier output (registered)
    logic         br_taken;    // Branch resolved as taken
    logic [31:0]  br_target;   // Resolved branch/jump target
    wb_src_e      wb_src;
    mem_size_e    mem_size;
    logic         mem_read;
    logic         mem_write;
    logic         rf_we;
    logic [REG_AW-1:0] rd;
    logic         csr_en;
    logic [31:0]  csr_rdata;   // CSR read result
    logic         illegal;
    logic         ecall;
    logic         ebreak;
  } ex_mem_bundle_t;

  // ---------------------------------------------------------------------------
  // MEM → WB pipeline register
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [31:0]  pc;
    logic [31:0]  alu_result;
    logic [31:0]  mem_rdata;   // Load data (sign/zero extended)
    logic [31:0]  csr_rdata;
    wb_src_e      wb_src;
    logic         rf_we;
    logic [REG_AW-1:0] rd;
  } mem_wb_bundle_t;

  // ---------------------------------------------------------------------------
  // Forwarding select (EX stage operand bypass)
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    FWD_REG    = 2'b00,  // From register file (no hazard)
    FWD_EX_MEM = 2'b01,  // Forward from EX/MEM pipeline reg
    FWD_MEM_WB = 2'b10,  // Forward from MEM/WB pipeline reg
    FWD_WB     = 2'b11   // Forward from WB output
  } fwd_sel_e;

  // ---------------------------------------------------------------------------
  // AHB-Lite transaction type
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    HTRANS_IDLE   = 2'b00,
    HTRANS_BUSY   = 2'b01,
    HTRANS_NONSEQ = 2'b10,
    HTRANS_SEQ    = 2'b11
  } htrans_e;

  typedef enum logic [2:0] {
    HBURST_SINGLE = 3'b000,
    HBURST_INCR   = 3'b001,
    HBURST_WRAP4  = 3'b010,
    HBURST_INCR4  = 3'b011,
    HBURST_WRAP8  = 3'b100,
    HBURST_INCR8  = 3'b101,
    HBURST_WRAP16 = 3'b110,
    HBURST_INCR16 = 3'b111
  } hburst_e;

  typedef enum logic [2:0] {
    HSIZE_BYTE  = 3'b000,
    HSIZE_HALF  = 3'b001,
    HSIZE_WORD  = 3'b010
  } hsize_e;

  // ---------------------------------------------------------------------------
  // Debug abstract command types (DM spec 0.13)
  // ---------------------------------------------------------------------------
  typedef enum logic [7:0] {
    ABST_ACCESS_REG  = 8'd0,
    ABST_QUICK_ACCESS= 8'd1,
    ABST_ACCESS_MEM  = 8'd2
  } abst_cmd_e;

endpackage : rv32emc_pkg

`endif // RV32EMC_PKG_SV
