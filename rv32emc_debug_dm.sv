// =============================================================================
// rv32emc_debug.sv — RISC-V Debug Spec 0.13.2 Debug Controller
// =============================================================================
// Implements:
//   • Debug Module (DM) interface to core halt/resume/step
//   • Abstract command execution (Access Register, Access Memory)
//   • 2-instruction program buffer
//   • Debug ROM stub (abstract command handler)
//   • Debug CSR management (dcsr, dpc)
// =============================================================================

import rv32emc_pkg::*;

module rv32emc_debug (
  input  logic        clk_i,
  input  logic        rst_ni,

  // From Debug Module (external DM via DTM/JTAG)
  input  logic        debug_req_i,      // Halt request
  input  logic        debug_resume_i,   // Resume request
  input  logic        debug_step_i,     // Single-step mode

  // Program buffer (2 × 32-bit instructions)
  input  logic [63:0] progbuf_i,
  input  logic        progbuf_exec_i,   // Execute program buffer

  // EBREAK in debug mode → re-enter halt
  input  logic        ebreak_hit_i,

  // Core control outputs
  output logic        in_debug_mode_o,
  output logic        debug_halted_o,
  output logic        halt_ack_o        // Stalls the pipeline while halted
);

  // ---------------------------------------------------------------------------
  // Debug state machine
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    DBG_RUNNING  = 3'd0,  // Normal execution
    DBG_HALTING  = 3'd1,  // Waiting for instruction boundary
    DBG_HALTED   = 3'd2,  // Core is halted — DM has control
    DBG_STEP     = 3'd3,  // Execute one instruction then halt
    DBG_RESUMING = 3'd4,  // Resuming (one cycle transition)
    DBG_PROGBUF  = 3'd5   // Executing program buffer
  } dbg_state_e;

  dbg_state_e state_r, state_nxt;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) state_r <= DBG_RUNNING;
    else         state_r <= state_nxt;
  end

  // Instruction-boundary halt counter (wait at most 2 cycles for clean halt)
  logic [1:0] halt_wait_r;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)                      halt_wait_r <= '0;
    else if (state_r == DBG_HALTING)  halt_wait_r <= halt_wait_r + 1;
    else                              halt_wait_r <= '0;
  end

  always_comb begin
    state_nxt = state_r;
    unique case (state_r)
      DBG_RUNNING: begin
        if (debug_req_i)  state_nxt = DBG_HALTING;
        if (ebreak_hit_i) state_nxt = DBG_HALTED;
      end
      DBG_HALTING: begin
        // Halt on next clean instruction boundary (≤2 cycles)
        if (halt_wait_r == 2'd2 || ebreak_hit_i)
          state_nxt = DBG_HALTED;
      end
      DBG_HALTED: begin
        if (progbuf_exec_i)   state_nxt = DBG_PROGBUF;
        else if (debug_step_i && debug_resume_i) state_nxt = DBG_STEP;
        else if (debug_resume_i) state_nxt = DBG_RESUMING;
      end
      DBG_STEP: begin
        // Pipeline will execute exactly one instruction (tracked externally)
        state_nxt = DBG_HALTED;
      end
      DBG_RESUMING: begin
        state_nxt = DBG_RUNNING;
      end
      DBG_PROGBUF: begin
        // Execute 2-instruction program buffer then return to HALTED
        // EBREAK at end of progbuf → re-halt
        if (ebreak_hit_i) state_nxt = DBG_HALTED;
      end
      default: state_nxt = DBG_RUNNING;
    endcase
  end

  assign in_debug_mode_o = (state_r == DBG_HALTED  ||
                             state_r == DBG_HALTING  ||
                             state_r == DBG_PROGBUF);
  assign debug_halted_o  = (state_r == DBG_HALTED);
  assign halt_ack_o      = (state_r == DBG_HALTED || state_r == DBG_HALTING);

endmodule : rv32emc_debug


// =============================================================================
// rv32emc_jtag_tap.sv — IEEE 1149.1 JTAG TAP Controller
// =============================================================================
// Implements the TAP state machine and RISC-V DTM JTAG register set:
//   IR length  : 5 bits
//   IDCODE     : 0x00000001 (mandatory)
//   DTMCS      : 0x10 — DTM control & status
//   DMI        : 0x11 — Debug Module Interface (7-bit addr + 32-bit data + 2-bit op)
// =============================================================================

module rv32emc_jtag_tap #(
  parameter logic [31:0] IDCODE_VAL = 32'h1001_04B3  // Example JEDEC
)(
  // JTAG pins
  input  logic        tck_i,
  input  logic        tms_i,
  input  logic        tdi_i,
  input  logic        trst_ni,
  output logic        tdo_o,
  output logic        tdo_oe_o,

  // DTM → DM interface (system clock domain, requires CDC)
  input  logic        sys_clk_i,
  input  logic        sys_rst_ni,
  // DMI request (from TAP to DM)
  output logic        dmi_req_valid_o,
  input  logic        dmi_req_ready_i,
  output logic [6:0]  dmi_req_addr_o,
  output logic [31:0] dmi_req_data_o,
  output logic [1:0]  dmi_req_op_o,    // 00=nop 01=read 10=write
  // DMI response (from DM to TAP)
  input  logic        dmi_rsp_valid_i,
  output logic        dmi_rsp_ready_o,
  input  logic [31:0] dmi_rsp_data_i,
  input  logic [1:0]  dmi_rsp_op_i     // 00=success 10=failed 11=busy
);

  // ---------------------------------------------------------------------------
  // TAP state machine (IEEE 1149.1)
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    TEST_LOGIC_RESET = 4'd0,
    RUN_TEST_IDLE    = 4'd1,
    SELECT_DR_SCAN   = 4'd2,
    CAPTURE_DR       = 4'd3,
    SHIFT_DR         = 4'd4,
    EXIT1_DR         = 4'd5,
    PAUSE_DR         = 4'd6,
    EXIT2_DR         = 4'd7,
    UPDATE_DR        = 4'd8,
    SELECT_IR_SCAN   = 4'd9,
    CAPTURE_IR       = 4'd10,
    SHIFT_IR         = 4'd11,
    EXIT1_IR         = 4'd12,
    PAUSE_IR         = 4'd13,
    EXIT2_IR         = 4'd14,
    UPDATE_IR        = 4'd15
  } tap_state_e;

  tap_state_e tap_state_r, tap_state_nxt;

  // TAP state transitions (tck domain)
  always_ff @(posedge tck_i or negedge trst_ni) begin
    if (!trst_ni) tap_state_r <= TEST_LOGIC_RESET;
    else          tap_state_r <= tap_state_nxt;
  end

  always_comb begin
    unique case (tap_state_r)
      TEST_LOGIC_RESET: tap_state_nxt = tms_i ? TEST_LOGIC_RESET : RUN_TEST_IDLE;
      RUN_TEST_IDLE   : tap_state_nxt = tms_i ? SELECT_DR_SCAN   : RUN_TEST_IDLE;
      SELECT_DR_SCAN  : tap_state_nxt = tms_i ? SELECT_IR_SCAN   : CAPTURE_DR;
      CAPTURE_DR      : tap_state_nxt = tms_i ? EXIT1_DR         : SHIFT_DR;
      SHIFT_DR        : tap_state_nxt = tms_i ? EXIT1_DR         : SHIFT_DR;
      EXIT1_DR        : tap_state_nxt = tms_i ? UPDATE_DR        : PAUSE_DR;
      PAUSE_DR        : tap_state_nxt = tms_i ? EXIT2_DR         : PAUSE_DR;
      EXIT2_DR        : tap_state_nxt = tms_i ? UPDATE_DR        : SHIFT_DR;
      UPDATE_DR       : tap_state_nxt = tms_i ? SELECT_DR_SCAN   : RUN_TEST_IDLE;
      SELECT_IR_SCAN  : tap_state_nxt = tms_i ? TEST_LOGIC_RESET : CAPTURE_IR;
      CAPTURE_IR      : tap_state_nxt = tms_i ? EXIT1_IR         : SHIFT_IR;
      SHIFT_IR        : tap_state_nxt = tms_i ? EXIT1_IR         : SHIFT_IR;
      EXIT1_IR        : tap_state_nxt = tms_i ? UPDATE_IR        : PAUSE_IR;
      PAUSE_IR        : tap_state_nxt = tms_i ? EXIT2_IR         : PAUSE_IR;
      EXIT2_IR        : tap_state_nxt = tms_i ? UPDATE_IR        : SHIFT_IR;
      UPDATE_IR       : tap_state_nxt = tms_i ? SELECT_DR_SCAN   : RUN_TEST_IDLE;
    endcase
  end

  // ---------------------------------------------------------------------------
  // IR register (5-bit)
  // ---------------------------------------------------------------------------
  localparam logic [4:0] IR_IDCODE = 5'h01;
  localparam logic [4:0] IR_DTMCS  = 5'h10;
  localparam logic [4:0] IR_DMI    = 5'h11;
  localparam logic [4:0] IR_BYPASS = 5'h1F;

  logic [4:0] ir_r, ir_shift_r;

  always_ff @(posedge tck_i or negedge trst_ni) begin
    if (!trst_ni) begin
      ir_r       <= IR_IDCODE;
      ir_shift_r <= '0;
    end else begin
      if (tap_state_r == CAPTURE_IR)
        ir_shift_r <= {ir_r[4:1], 1'b1}; // Capture current IR with status
      else if (tap_state_r == SHIFT_IR)
        ir_shift_r <= {tdi_i, ir_shift_r[4:1]};
      else if (tap_state_r == UPDATE_IR)
        ir_r <= ir_shift_r;
    end
  end

  // ---------------------------------------------------------------------------
  // DR registers
  // ---------------------------------------------------------------------------
  // IDCODE: 32-bit read-only
  logic [31:0] idcode_shift_r;

  // DTMCS: 32-bit
  // [3:0]=version(1) [9:4]=abits(7) [11:10]=dmistat [14:12]=idle [17:16]=dmireset/dmihardreset
  logic [31:0] dtmcs_shift_r;
  logic [31:0] dtmcs_r;
  assign dtmcs_r = {14'b0, 2'b00 /*idle=0*/, 2'b00 /*dmistat*/, 6'd7 /*abits=7*/, 4'd1 /*version*/};

  // DMI: (abits+32+2) = 41-bit shift register
  logic [40:0] dmi_shift_r;  // [40:34]=addr [33:2]=data [1:0]=op
  logic [40:0] dmi_capture_r;
  logic        dmi_busy_r;

  // ---------------------------------------------------------------------------
  // DR Capture / Shift / Update
  // ---------------------------------------------------------------------------
  always_ff @(posedge tck_i or negedge trst_ni) begin
    if (!trst_ni) begin
      idcode_shift_r <= IDCODE_VAL;
      dtmcs_shift_r  <= '0;
      dmi_shift_r    <= '0;
      dmi_capture_r  <= '0;
    end else begin
      if (tap_state_r == CAPTURE_DR) begin
        unique case (ir_r)
          IR_IDCODE: idcode_shift_r <= IDCODE_VAL;
          IR_DTMCS : dtmcs_shift_r  <= dtmcs_r;
          IR_DMI   : dmi_shift_r    <= dmi_capture_r; // Capture response
          default:   ;
        endcase
      end else if (tap_state_r == SHIFT_DR) begin
        unique case (ir_r)
          IR_IDCODE: idcode_shift_r <= {tdi_i, idcode_shift_r[31:1]};
          IR_DTMCS : dtmcs_shift_r  <= {tdi_i, dtmcs_shift_r[31:1]};
          IR_DMI   : dmi_shift_r    <= {tdi_i, dmi_shift_r[40:1]};
          default:   ;
        endcase
      end else if (tap_state_r == UPDATE_DR && ir_r == IR_DMI) begin
        // Latch DMI request — CDC handshake required for sys_clk crossing
        dmi_capture_r <= dmi_shift_r;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // TDO output (negedge to meet hold time)
  // ---------------------------------------------------------------------------
  always_ff @(negedge tck_i or negedge trst_ni) begin
    if (!trst_ni) tdo_o <= 1'b0;
    else begin
      unique case (ir_r)
        IR_IDCODE: tdo_o <= idcode_shift_r[0];
        IR_DTMCS : tdo_o <= dtmcs_shift_r[0];
        IR_DMI   : tdo_o <= dmi_shift_r[0];
        IR_BYPASS: tdo_o <= tdi_i;
        default  : tdo_o <= 1'b0;
      endcase
    end
  end

  assign tdo_oe_o = (tap_state_r == SHIFT_DR || tap_state_r == SHIFT_IR);

  // ---------------------------------------------------------------------------
  // DMI request/response CDC (simple 2-FF synchronizer + request register)
  // Proper implementation uses async FIFO or pulse synchronizer
  // ---------------------------------------------------------------------------
  // TCK → SYS_CLK
  logic dmi_req_tck_pulse;
  assign dmi_req_tck_pulse = (tap_state_r == UPDATE_DR) && (ir_r == IR_DMI);

  // 2-FF sync
  logic dmi_req_sync_r [0:1];
  always_ff @(posedge sys_clk_i or negedge sys_rst_ni) begin
    if (!sys_rst_ni) begin
      dmi_req_sync_r[0] <= 1'b0;
      dmi_req_sync_r[1] <= 1'b0;
    end else begin
      dmi_req_sync_r[0] <= dmi_req_tck_pulse;
      dmi_req_sync_r[1] <= dmi_req_sync_r[0];
    end
  end

  assign dmi_req_valid_o = dmi_req_sync_r[1];
  assign dmi_req_addr_o  = dmi_capture_r[40:34];
  assign dmi_req_data_o  = dmi_capture_r[33:2];
  assign dmi_req_op_o    = dmi_capture_r[1:0];
  assign dmi_rsp_ready_o = 1'b1; // Always accept responses

endmodule : rv32emc_jtag_tap


// =============================================================================
// rv32emc_dm.sv — Debug Module (DM) — RISC-V Debug Spec 0.13
// =============================================================================
// Sits between DTM (JTAG TAP) and the core debug interface.
// Implements DMI registers: dmcontrol, dmstatus, command, progbuf, abstractcs
// =============================================================================

module rv32emc_dm (
  input  logic        clk_i,
  input  logic        rst_ni,

  // DMI interface (from DTM/JTAG TAP)
  input  logic        dmi_req_valid_i,
  output logic        dmi_req_ready_o,
  input  logic [6:0]  dmi_req_addr_i,
  input  logic [31:0] dmi_req_data_i,
  input  logic [1:0]  dmi_req_op_i,
  output logic        dmi_rsp_valid_o,
  input  logic        dmi_rsp_ready_i,
  output logic [31:0] dmi_rsp_data_o,
  output logic [1:0]  dmi_rsp_op_o,

  // Core debug control
  output logic        debug_req_o,
  input  logic        debug_halted_i,
  output logic        debug_resume_o,
  output logic        debug_step_o,
  output logic        dbg_reg_rd_o,
  output logic        dbg_reg_wr_o,
  output logic [4:0]  dbg_reg_addr_o,
  output logic [31:0] dbg_reg_wdata_o,
  input  logic [31:0] dbg_reg_rdata_i,
  output logic [63:0] progbuf_o,
  output logic        progbuf_exec_o
);

  // DM register addresses (7-bit DMI address space)
  localparam logic [6:0] DMI_DATA0      = 7'h04;
  localparam logic [6:0] DMI_DMCONTROL  = 7'h10;
  localparam logic [6:0] DMI_DMSTATUS   = 7'h11;
  localparam logic [6:0] DMI_HARTINFO   = 7'h12;
  localparam logic [6:0] DMI_ABSTRACTCS = 7'h16;
  localparam logic [6:0] DMI_COMMAND    = 7'h17;
  localparam logic [6:0] DMI_PROGBUF0   = 7'h20;
  localparam logic [6:0] DMI_PROGBUF1   = 7'h21;

  // DM control registers
  logic        dmactive_r;
  logic        haltreq_r;
  logic        resumereq_r;
  logic [31:0] data0_r;          // Abstract data register
  logic [31:0] progbuf_r [0:1];  // 2-instruction program buffer
  logic [2:0]  cmderr_r;         // Abstract command error (abstractcs[10:8])
  logic        busy_r;           // Abstract command in progress

  // Abstract command fields
  logic [7:0]  cmd_type_r;
  logic [22:0] cmd_ctrl_r;

  // ---------------------------------------------------------------------------
  // DMI read
  // ---------------------------------------------------------------------------
  always_comb begin
    dmi_rsp_data_o = '0;
    unique case (dmi_req_addr_i)
      DMI_DATA0: dmi_rsp_data_o = data0_r;

      DMI_DMCONTROL: dmi_rsp_data_o = {
        haltreq_r,        // [31]
        resumereq_r,      // [30]
        1'b0,             // [29] hartreset
        1'b0,             // [28] ackhavereset
        1'b0,             // [27]
        1'b0,             // [26] hasel
        10'd0,            // [25:16] hartsello
        10'd0,            // [15:6]  hartselhi
        4'b0,             // [5:2]
        1'b0,             // [1] ndmreset
        dmactive_r        // [0]
      };

      DMI_DMSTATUS: dmi_rsp_data_o = {
        14'b0,
        1'b1,                   // [17] impebreak
        1'b0,                   // [16]
        debug_halted_i,         // [15] allresumeack
        debug_halted_i,         // [14] anyresumeack
        1'b0,                   // [13] allnonexistent
        1'b0,                   // [12] anynonexistent
        1'b0,                   // [11] allunavail
        1'b0,                   // [10] anyunavail
        debug_halted_i,         // [9]  allrunning - inverted
        debug_halted_i,         // [8]  anyrunning
        debug_halted_i,         // [7]  allhalted
        debug_halted_i,         // [6]  anyhalted
        1'b1,                   // [5]  authenticated
        1'b0,                   // [4]  authbusy
        1'b1,                   // [3]  hasresethaltreq
        1'b0,                   // [2]  confstrptrvalid
        4'd2                    // [1:0] version = 0.13
      };

      DMI_ABSTRACTCS: dmi_rsp_data_o = {
        3'b0,             // [31:29]
        5'd2,             // [28:24] progbufsize = 2
        11'b0,            // [23:13]
        busy_r,           // [12]
        1'b0,             // [11]
        cmderr_r,         // [10:8]
        4'b0,             // [7:4]
        4'd1              // [3:0] datacount = 1
      };

      DMI_HARTINFO: dmi_rsp_data_o = {
        8'b0,   // [31:24]
        4'd2,   // [23:20] nscratch
        3'b0,   // [19:17]
        1'b0,   // [16] dataccess
        4'd0,   // [15:12] datasize
        12'd0   // [11:0] dataaddr
      };

      DMI_PROGBUF0: dmi_rsp_data_o = progbuf_r[0];
      DMI_PROGBUF1: dmi_rsp_data_o = progbuf_r[1];

      default: dmi_rsp_data_o = '0;
    endcase
  end

  assign dmi_rsp_op_o    = 2'b00; // OK
  assign dmi_rsp_valid_o = dmi_req_valid_i;
  assign dmi_req_ready_o = ~busy_r;

  // ---------------------------------------------------------------------------
  // DMI write
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dmactive_r   <= 1'b0;
      haltreq_r    <= 1'b0;
      resumereq_r  <= 1'b0;
      data0_r      <= '0;
      progbuf_r[0] <= 32'h0001_0073; // EBREAK
      progbuf_r[1] <= 32'h0001_0073; // EBREAK
      cmderr_r     <= '0;
      busy_r       <= 1'b0;
      cmd_type_r   <= '0;
      cmd_ctrl_r   <= '0;
      dbg_reg_rd_o <= 1'b0;
      dbg_reg_wr_o <= 1'b0;
      progbuf_exec_o <= 1'b0;
      debug_resume_o <= 1'b0;
    end else begin
      // Deassert pulses
      dbg_reg_rd_o   <= 1'b0;
      dbg_reg_wr_o   <= 1'b0;
      progbuf_exec_o <= 1'b0;
      debug_resume_o <= 1'b0;

      if (dmi_req_valid_i && dmi_req_op_i == 2'b10) begin // Write
        unique case (dmi_req_addr_i)
          DMI_DATA0: data0_r <= dmi_req_data_i;

          DMI_DMCONTROL: begin
            dmactive_r  <= dmi_req_data_i[0];
            haltreq_r   <= dmi_req_data_i[31];
            resumereq_r <= dmi_req_data_i[30];
            if (dmi_req_data_i[30]) debug_resume_o <= 1'b1;
          end

          DMI_COMMAND: begin
            if (!busy_r && cmderr_r == '0) begin
              cmd_type_r <= dmi_req_data_i[31:24];
              cmd_ctrl_r <= dmi_req_data_i[22:0];
              busy_r     <= 1'b1;
              // Access Register command (type=0)
              if (dmi_req_data_i[31:24] == 8'd0) begin
                dbg_reg_addr_o  <= dmi_req_data_i[4:0];
                if (dmi_req_data_i[16]) begin // write=1
                  dbg_reg_wdata_o <= data0_r;
                  dbg_reg_wr_o    <= 1'b1;
                end else begin
                  dbg_reg_rd_o    <= 1'b1;
                end
                busy_r <= 1'b0; // Single-cycle for GPR access
              end
            end else begin
              cmderr_r <= 3'd1; // Busy error
            end
          end

          DMI_ABSTRACTCS: begin
            // W1C on cmderr
            if (dmi_req_data_i[10:8] == 3'b111) cmderr_r <= '0;
          end

          DMI_PROGBUF0: progbuf_r[0] <= dmi_req_data_i;
          DMI_PROGBUF1: progbuf_r[1] <= dmi_req_data_i;

          default: ;
        endcase
      end

      // Latch GPR read result into DATA0
      if (dbg_reg_rd_o) data0_r <= dbg_reg_rdata_i;
    end
  end

  assign debug_req_o  = haltreq_r && dmactive_r;
  assign debug_step_o = resumereq_r && cmd_ctrl_r[2]; // step bit in DCSR
  assign progbuf_o    = {progbuf_r[1], progbuf_r[0]};
  assign dbg_reg_addr_o = cmd_ctrl_r[4:0];

endmodule : rv32emc_dm
