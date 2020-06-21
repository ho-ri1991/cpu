module pipelined_cpu_with_interrupts(
  input        clk,
  input        clrn,
  output[31:0] pc,
  output[31:0] inst,
  output[31:0] ealu,
  output[31:0] malu,
  output[31:0] wdi,
  input        intr,
  output       inta
);

parameter int_base = 32'h00000008;

wire[31:0] next_pc, bpc, jpc, epc, pc4, instruction, dpc4, dinst;
wire irq;
assign inst = dinst;
wire[1:0] pcsrc, selpc;
wire wpc; // program counter write enable (stall when wpc is zero)
reg[31:0] program_counter;
assign pc = program_counter;
always @(posedge clk or negedge clrn) begin
  if(!clrn)
    program_counter <= 32'b0;
  else begin
    if(wpc)
      program_counter <= next_pc;
  end
end

wire wreg, m2reg, wmem, jal, aluimm, shift, nostall, cancel, isbranch, ove;
wire[1:0] mfc0;
wire[1:0] sepc;
wire[3:0] aluc;
wire[4:0] rn;
wire[31:0] a, b, dimm, pcd, sta, cau;

wire ewreg0, ewreg, em2reg, ewmem, ejal, ealuimm, eshift, ecancel, eisbranch, exc_ovr, eove;
wire[1:0] emfc0;
wire[3:0] ealuc;
wire[4:0] ern0, ern;
wire[31:0] ea, eb, ealu, epc4, eimm, pce;

wire mwreg, mm2reg, mwmem, mexc_ovr, misbranch;
wire[4:0] mrn;
wire[31:0] malu, mmo, mb, pcm;

wire wwreg, wm2reg;
wire[4:0] wrn;
wire[31:0] wdi, wmo, walu;

pipelined_with_int_if_stage if_stage(
  .pc(program_counter), .bpc(bpc), .rpc(a), .jpc(jpc), .epc(epc), .base(int_base),
  .pcsrc(pcsrc), .selpc(selpc),
  .next_pc(next_pc), .pc4(pc4), .inst(instruction));

pipelined_with_int_if_id_reg if_id_reg(
  .pc4(pc4), .inst(instruction), .pc(program_counter), .intr(intr),
  .wir(wpc), .clk(clk), .clrn(clrn),
  .dpc4(dpc4), .dinst(dinst), .dpc(pcd), .irq(irq));

pipelined_with_int_id_stage id_stage(
  .mwreg(mwreg), .mrn(mrn), .ern(ern), .ewreg(ewreg), .em2reg(em2reg),
  .mm2reg(mm2reg), .dpc4(dpc4), .inst(dinst), .wrn(wrn), .wdi(wdi), .ealu(ealu),
  .malu(malu), .mmo(mmo), .wwreg(wwreg), .clk(clk), .clrn(clrn),
  .bpc(bpc), .jpc(jpc), .pcsrc(pcsrc), .nostall(wpc), .wreg(wreg), .m2reg(m2reg), .wmem(wmem),
  .aluc(aluc), .aluimm(aluimm), .a(a), .b(b), .dimm(dimm), .rn(rn), .shift(shift), .jal(jal),
  .pc(pc), .pcd(pcd), .pce(pce), .pcm(pcm), .c0_cause(cau), .c0_status(sta), .c0_epc(epc),
  .irq(irq), .inta(inta), .mexc_ovr(mexc_ovr), .misbranch(misbranch),
  .exc_ovr(exc_ovr), .eisbranch(eisbranch), .ecancel(ecancel),
  .cancel(cancel), .isbranch(isbranch), .ove(ove), .mfc0(mfc0), .selpc(selpc));

pipelined_with_int_id_exe_reg id_exe_reg(
  .dwreg(wreg), .dm2reg(m2reg), .dwmem(wmem), .daluc(aluc), .daluimm(aluimm),
  .da(a), .db(b), .dimm(dimm), .drn(rn), .dshift(shift), .djal(jal), .dpc4(dpc4),
  .dove(ove), .dcancel(cancel), .disbranch(isbranch), .dmfc0(mfc0), .pcd(pcd), .clk(clk), .clrn(clrn),
  .ewreg(ewreg0), .em2reg(em2reg), .ewmem(ewmem), .ealuc(ealuc), .ealuimm(ealuimm), .ea(ea), .eb(eb), .eimm(eimm), .ern(ern0), .eshift(eshift), .ejal(ejal), .epc4(epc4),
  .eove(eove), .ecancel(ecancel), .eisbranch(eisbranch), .emfc0(emfc0), .pce(pce));

pipelined_with_int_exe_stage exe_stage(
  .ealuc(ealuc), .ealuimm(ealuimm), .ea(ea), .eb(eb), .eimm(eimm), .eshift(eshift),
  .ern0(ern0), .epc4(epc4), .ejal(ejal), .eove(eove), .ewreg0(ewreg0), .emfc0(emfc0), .sta(sta), .cau(cau), .epc(epc),
  .ern(ern), .ealu(ealu), .exc_ovr(exc_ovr), .ewreg(ewreg));

pipelined_with_int_exe_mem_reg exe_mem_reg(
  .ewreg(ewreg), .em2reg(em2reg), .ewmem(ewmem), .ealu(ealu), .eb(eb), .ern(ern), .eisbranch(eisbranch), .exc_ovr(exc_ovr), .pce(pce),
  .clk(clk), .clrn(clrn),
  .mwreg(mwreg), .mm2reg(mm2reg), .mwmem(mwmem), .malu(malu), .mb(mb), .mrn(mrn), .misbranch(misbranch), .mexc_ovr(mexc_ovr), .pcm(pcm));

pipelined_mem_stage mem_stage(
  .we(mwmem), .addr(malu), .datain(mb), .clk(clk), .dataout(mmo));

pipelined_mem_wb_reg mem_wb_reg(
  .mwreg(mwreg), .mm2reg(mm2reg), .mmo(mmo), .malu(malu), .mrn(mrn), .clk(clk), .clrn(clrn),
  .wwreg(wwreg), .wm2reg(wm2reg), .wmo(wmo), .walu(walu), .wrn(wrn));

pipelined_wb_stage wb_state(
  .walu(walu), .wmo(wmo), .wm2reg(wm2reg), .wdi(wdi));

endmodule

// IF stage
module pipelined_with_int_if_stage(
  input[31:0] pc,  // current program counter
  input[31:0] bpc, // branch program counter
  input[31:0] rpc, // jr program counter
  input[31:0] jpc, // jump program counter
  input[31:0] epc,
  input[31:0] base,
  input[1:0]  pcsrc,
  input[1:0]  selpc,
  output reg[31:0] next_pc,
  output reg[31:0] pc4,
  output[31:0] inst
);

reg[31:0] npc;
always @* begin
  pc4 = pc + 4;
  case(pcsrc)
    2'b00: npc = pc4;
    2'b01: npc = bpc;
    2'b10: npc = rpc;
    2'b11: npc = jpc;
  endcase
  case(selpc)
    2'b00: next_pc = npc;
    2'b01: next_pc = epc;
    2'b10: next_pc = base;
    2'b11: next_pc = 32'b0;
  endcase
end
pl_exc_i_mem inst_mem(pc, inst);

endmodule

// IF/ID pipeline register
module pipelined_with_int_if_id_reg(
  input[31:0] pc4,
  input[31:0] inst,
  input[31:0] pc,
  input       intr,
  input       wir, // IF/ID stage reg write enable (pipeline stall when wir is zero)
  input       clk,
  input       clrn,
  output reg[31:0] dpc4,
  output reg[31:0] dinst,
  output reg[31:0] dpc,
  output reg       irq
);

always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    dpc4 <= 32'b0;
    dinst <= 32'b0;
    dpc <= 32'b0;
    irq <= 1'b0;
  end else begin
    if(wir) begin
      dpc4 <= pc4;
      dinst <= inst;
      dpc <= pc;
    end
    irq <= intr;
  end
end

endmodule

// ID stage
module pipelined_with_int_id_stage(
  input            mwreg, // register file write enable flag from EXE/MEM pipeline register
  input[4:0]       mrn,   // write register number from EXE/MEM pipeline register
  input[4:0]       ern,   // write register number from ID/EXE pipeline register
  input            ewreg, // register file write enable flag from ID/EXE pipeline register
  input            em2reg,// load instruction flag (selector of data from ALU or data memory) from ID/EXE pipeline register
  input            mm2reg,// load instruction flag (selector of data from ALU or data memory) from ID/EXE pipeline register
  input[31:0]      dpc4,  // pc + 4 from IF/ID pipeline register
  input[31:0]      inst,  // instruction from IF/ID pipeline register
  input[4:0]       wrn,   // write register number from MEM/WB pipeline register
  input[31:0]      wdi,   // write data for register file from MEM/WB pipeline register
  input[31:0]      ealu,  // ALU out from EXE stage
  input[31:0]      malu,  // ALU out from EXE/MEM pipeline register
  input[31:0]      mmo,   // memory out from MEM stage
  input            wwreg, // register file write enable flag from MEM/WB stage
  input            clk,
  input            clrn,
  output[31:0]     bpc,     // branch program counter
  output[31:0]     jpc,     // jump program counter
  output[1:0]      pcsrc,   // program couner selector
  output           nostall, // no pipeline stall
  output           wreg,    // register file write enable
  output           m2reg,   // selector for register file input (ALU out or data memory out)
  output           wmem,    // write enable for data memory
  output[3:0]      aluc,    // ALU control signal
  output           aluimm,  // input b of ALU is immediate
  output reg[31:0] a,       // operand a
  output reg[31:0] b,       // operand b
  output[31:0]     dimm,    // decoded immediate
  output[4:0]      rn,      // decoded write register number
  output           shift,   // instruction is shift
  output           jal,     // instruction is jal
  input[31:0]      pc,   // pc in IF stage
  input[31:0]      pcd,  // pc in ID stage
  input[31:0]      pce,  // pc in EXE stage
  input[31:0]      pcm,  // pc in MEM stage
  output reg[31:0] c0_status,
  output reg[31:0] c0_cause,
  output reg[31:0] c0_epc,
  input            irq,
  output           inta,
  input            mexc_ovr,
  input            misbranch,
  input            exc_ovr,
  input            eisbranch,
  input            ecancel,
  output           cancel,
  output           isbranch,
  output           ove, // overflow enable (arithmetic op and mask)
  output[1:0]      mfc0,
  output[1:0]      selpc
);

wire[5:0] op    = inst[31:26];
wire[4:0] rs    = inst[25:21];
wire[4:0] rt    = inst[20:16];
wire[4:0] rd    = inst[15:11];
wire[5:0] func  = inst[5:0];
wire[15:0] imm  = inst[15:0];
wire[25:0] addr = inst[25:0];
wire[4:0] op1   = rs;

wire[31:0] qa, qb;
regfile regfile0(
  .rna(rs), .rnb(rt), .d(wdi),
  .wn(wrn), .we(wwreg), .clk(~clk), .clrn(clrn), // invert clk for negative edge trigger
  .qa(qa), .qb(qb));

// op decode
wire i_add     = (op == 6'b000000) & (func == 6'b100000); // add
wire i_sub     = (op == 6'b000000) & (func == 6'b100010); // sub
wire i_and     = (op == 6'b000000) & (func == 6'b100100); // and
wire i_or      = (op == 6'b000000) & (func == 6'b100101); // or
wire i_xor     = (op == 6'b000000) & (func == 6'b100110); // xor
wire i_sll     = (op == 6'b000000) & (func == 6'b000000); // shift left logical
wire i_srl     = (op == 6'b000000) & (func == 6'b000010); // shift right logical
wire i_sra     = (op == 6'b000000) & (func == 6'b000011); // shift right arithmetic
wire i_jr      = (op == 6'b000000) & (func == 6'b001000); // jump register
wire i_addi    = (op == 6'b001000); // add immediate
wire i_andi    = (op == 6'b001100); // and immediate
wire i_ori     = (op == 6'b001101); // or immediate
wire i_xori    = (op == 6'b001110); // xor immediate
wire i_lw      = (op == 6'b100011); // load word
wire i_sw      = (op == 6'b101011); // store word
wire i_beq     = (op == 6'b000100); // branch equal
wire i_bne     = (op == 6'b000101); // branch not equal
wire i_lui     = (op == 6'b001111); // load upper immediate
wire i_j       = (op == 6'b000010); // jump
wire i_jal     = (op == 6'b000011); // jump and link
wire i_mfc0    = (op == 6'b010000) & (op1 == 5'b00000); // move from c0 register
wire i_mtc0    = (op == 6'b010000) & (op1 == 5'b00100); // move to c0 register
wire i_eret    = (op == 6'b010000) & (op1 == 5'b10000) & (func == 6'b011000); // return from exception handler
wire i_syscall = (op == 6'b000000) & (func == 6'b001100); // syscall

wire unimplemented_inst =
  ~(i_add|i_sub|i_and|i_or|i_xor|i_sll|i_srl|i_sra|
    i_jr|i_addi|i_andi|i_ori|i_xori|i_lw|i_sw|
    i_beq|i_bne|i_lui|i_j|i_jal|i_mfc0|i_mtc0|i_eret|i_syscall);

// instructions that use rs
wire i_rs = i_add | i_sub | i_and | i_or | i_xor | i_jr |
            i_addi | i_andi | i_ori | i_xori | i_lw | i_sw | i_beq | i_bne;
// instructions that use rt
wire i_rt = i_add | i_sub | i_and | i_or | i_xor |
            i_sll | i_srl | i_sra | i_sw | i_beq | i_bne | i_mtc0;
// stall occurs the instruction in EXE stage is load word instruction
assign nostall = ~(ewreg & em2reg & (ern != 0) & (i_rs & (ern == rs) | i_rt & (ern == rt)));

// selector of ALU input a and b
reg[1:0] fwda, fwdb;
always @* begin
  fwda = 2'b00; // default, no hazard
  if(ewreg & (ern != 0) & (ern == rs) & ~em2reg)
    fwda = 2'b01; // use ALU out in EXE stage
  else begin
    if(mwreg & (mrn != 0) & (mrn == rs) & ~mm2reg)
      fwda = 2'b10; // use ALU out in MEM stage
    else begin
      if(mwreg & (mrn != 0) & (mrn == rs) & mm2reg)
        fwda = 2'b11; // use data memory out in MEM stage
    end
  end
  fwdb = 2'b00; // default no hazard
  if(ewreg & (ern != 0) & (ern == rt) & ~em2reg)
    fwdb = 2'b01; // use ALU out in EXE stage
  else begin
    if(mwreg & (mrn != 0) & (mrn == rt) & ~mm2reg)
      fwdb = 2'b10; // use ALU out in EXE stage
    else begin
      if(mwreg & (mrn != 0) & (mrn == rt) & mm2reg)
        fwdb = 2'b11;
    end
  end
end

wire rs_rt_equal = ~|(a ^ b);

assign jal      = i_jal;
assign m2reg    = i_lw;
assign wmem     = i_sw & nostall & ~ecancel & ~exc_ovr & ~mexc_ovr;
assign aluc[3]  = i_sra;
assign aluc[2]  = i_sub | i_or | i_srl | i_sra | i_ori | i_lui;
assign aluc[1]  = i_xor | i_sll| i_srl | i_sra | i_lui | i_xori | i_beq | i_bne;
assign aluc[0]  = i_and | i_or | i_sll | i_srl | i_sra | i_andi | i_ori;
assign shift    = i_sll | i_srl| i_sra;
assign aluimm   = i_addi | i_andi | i_ori | i_xori | i_lw | i_lui | i_sw;
assign pcsrc[1] = i_jr | i_j | i_jal;
assign pcsrc[0] = (i_beq & rs_rt_equal) | (i_bne & ~rs_rt_equal) | i_j | i_jal;
assign wreg     = (i_add | i_sub | i_and | i_or | i_xor | i_sll | i_srl | i_sra |
                   i_addi | i_andi | i_ori | i_xori | i_lw | i_lui | i_jal | i_mfc0)
                   & nostall & ~ecancel & ~exc_ovr & ~mexc_ovr;
wire regrt    = i_addi | i_andi | i_ori | i_xori | i_lw | i_lui | i_mfc0;
wire sext     = i_addi | i_lw | i_sw | i_beq | i_bne;
assign rn = (regrt ? rt : rd);

always @* begin
  case(fwda)
    2'b00: a = qa;
    2'b01: a = ealu;
    2'b10: a = malu;
    2'b11: a = mmo;
  endcase
  case(fwdb)
    2'b00: b = qb;
    2'b01: b = ealu;
    2'b10: b = malu;
    2'b11: b = mmo;
  endcase
end

assign dimm = {{16{sext & imm[15]}}, imm};
assign jpc = {dpc4[31:28], addr, 2'b00};
assign bpc = dpc4 + {dimm[29:0], 2'b00};

wire arith = i_add | i_sub | i_addi; // for overflow
assign isbranch = i_beq | i_bne | i_jr | i_j | i_jal; // has delay slot
wire exc_int =  c0_status[0] & irq;
wire exc_sys =  c0_status[1] & i_syscall;
wire exc_uni =  c0_status[2] & unimplemented_inst;
wire exc_over = c0_status[3] & exc_ovr;
assign ove = c0_status[3] & arith;
assign inta = exc_int;
wire exc = exc_int | exc_sys | exc_uni | exc_over;
assign cancel = exc | i_eret;
// epc select
wire[1:0] sel_epc;
assign sel_epc[0] = exc_int & isbranch | exc_sys | exc_uni | exc_ovr & misbranch;
assign sel_epc[1] = exc_ovr;
reg[31:0] epc_in;
always @* begin
  case(sel_epc)
    2'b00: epc_in = pc;  // interrupt in ordinary case or delay slot
    2'b01: epc_in = pcd; // syscall or unimpl in ordinary case, or interrupt when branch/jump is in ID
    2'b10: epc_in = pce; // overflow in ordinary case
    2'b11: epc_in = pcm; // overflow in delay slot
  endcase
end
// cause
wire[1:0] exccode;
assign exccode[0] = i_syscall | exc_ovr;
assign exccode[1] = unimplemented_inst | exc_ovr;
wire[31:0] cause;
assign cause = {eisbranch, 27'h0, exccode, 2'b00};
wire rd_is_status = rd == 5'd12;
wire rd_is_cause = rd == 5'd13;
wire rd_is_epc = rd == 5'd14;
wire wstatus = exc | i_mtc0 & rd_is_status | i_eret;
wire wcause = exc | i_mtc0 & rd_is_cause;
wire wepc = exc | i_mtc0 & rd_is_epc;
assign mfc0[0] = i_mfc0 & rd_is_status | i_mfc0 & rd_is_epc;
assign mfc0[1] = i_mfc0 & rd_is_cause  | i_mfc0 & rd_is_epc;
assign selpc[0] = i_eret;
assign selpc[1] = exc;
always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    c0_status <= 0;
    c0_cause <= 0;
    c0_epc <= 0;
  end
  else begin
    if(wstatus)
      c0_status <= (i_mtc0 ? b : (exc ? (c0_status << 4) : (c0_status >> 4)));
    if(wcause)
      c0_cause <= (i_mtc0 ? b : cause);
    if(wepc)
      c0_epc <= (i_mtc0 ? b : epc_in);
  end
end

endmodule

// ID/EXE pipeline register
module pipelined_with_int_id_exe_reg(
  input        dwreg,
  input        dm2reg,
  input        dwmem,
  input[3:0]   daluc,
  input        daluimm,
  input[31:0]  da,
  input[31:0]  db,
  input[31:0]  dimm,
  input[4:0]   drn,
  input        dshift,
  input        djal,
  input[31:0]  dpc4,
  output       dove,
  output       dcancel,
  output       disbranch,
  output[1:0]  dmfc0,
  output[31:0] pcd,
  input        clk,
  input        clrn,
  output reg       ewreg,
  output reg       em2reg,
  output reg       ewmem,
  output reg[3:0]  ealuc,
  output reg       ealuimm,
  output reg[31:0] ea,
  output reg[31:0] eb,
  output reg[31:0] eimm,
  output reg[4:0]  ern,
  output reg       eshift,
  output reg       ejal,
  output reg[31:0] epc4,
  output reg       eove,
  output reg       ecancel,
  output reg       eisbranch,
  output reg[1:0]  emfc0,
  output reg[31:0] pce
);

always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    ewreg <= 0;
    em2reg <= 0;
    ewmem <= 0;
    ealuc <= 0;
    ealuimm <= 0;
    ea <= 0;
    eb <= 0;
    eimm <= 0;
    ern <= 0;
    eshift <= 0;
    ejal <= 0;
    epc4 <= 0;
    eove <= 0;
    ecancel <= 0;
    eisbranch <= 0;
    emfc0 <= 0;
    pce <= 0;
  end
  else begin
    ewreg <= dwreg;
    em2reg <= dm2reg;
    ewmem <= dwmem;
    ealuc <= daluc;
    ealuimm <= daluimm;
    ea <= da;
    eb <= db;
    eimm <= dimm;
    ern <= drn;
    eshift <= dshift;
    ejal <= djal;
    epc4 <= dpc4;
    eove <= dove;
    ecancel <= dcancel;
    eisbranch <= disbranch;
    emfc0 <= dmfc0;
    pce <= pcd;
  end
end

endmodule

// EXE stage
module pipelined_with_int_exe_stage(
  input[3:0]   ealuc,
  input        ealuimm,
  input[31:0]  ea,
  input[31:0]  eb,
  input[31:0]  eimm,
  input        eshift,
  input[4:0]   ern0,
  input[31:0]  epc4,
  input        ejal,
  input        eove,
  input        ewreg0,
  input[1:0]   emfc0,
  input[31:0]  sta,
  input[31:0]  cau,
  input[31:0]  epc,
  output[4:0]  ern,
  output[31:0] ealu,
  output       exc_ovr,
  output       ewreg
);

wire[31:0] alu_a = (eshift ? eimm : ea);
wire[31:0] alu_b = (ealuimm ? eimm : eb);
reg[31:0] alu_out;
reg alu_overflow;
reg[31:0] pc8c0r;
always @* begin
  casex(ealuc)
    4'bx000: alu_out <= alu_a + alu_b;
    4'bx100: alu_out <= alu_a - alu_b;
    4'bx001: alu_out <= alu_a & alu_b;
    4'bx101: alu_out <= alu_a | alu_b;
    4'bx010: alu_out <= alu_a ^ alu_b;
    4'bx110: alu_out <= {alu_b[15:0], 16'b0};
    4'b0011: alu_out <= alu_b << ea;
    4'b0111: alu_out <= alu_b >> ea;
    4'b1111: alu_out <= $signed(alu_b) >>> ea;
    default: alu_out <= 32'b0;
  endcase
  casex(ealuc)
    4'bx000: alu_overflow <= (~alu_a[31]&~alu_b[31]& alu_out[31]) | ( alu_a[31]& alu_b[31]&~alu_out[31]);
    4'bx100: alu_overflow <= (~alu_a[31]& alu_b[31]&~alu_out[31]) | ( alu_a[31]&~alu_b[31]&~alu_out[31]);
    default: alu_overflow <= 1'b0;
  endcase
  case(emfc0)
    2'b00: pc8c0r = epc4 + 32'd4; // return address
    2'b01: pc8c0r = sta;
    2'b10: pc8c0r = cau;
    2'b11: pc8c0r = epc;
  endcase
end
assign ealu = (ejal|emfc0[0]|emfc0[1] ? pc8c0r : alu_out);
assign ern = (ejal ? 32'd31 : ern0);
assign exc_ovr = alu_overflow & eove;
assign ewreg = ewreg0 & ~exc_ovr;

endmodule

// EXE/MEM pipeline register
module pipelined_with_int_exe_mem_reg(
  input            ewreg,
  input            em2reg,
  input            ewmem,
  input[31:0]      ealu,
  input[31:0]      eb,
  input[4:0]       ern,
  input            eisbranch,
  input            exc_ovr,
  input[31:0]      pce,
  input            clk,
  input            clrn,
  output reg       mwreg,
  output reg       mm2reg,
  output reg       mwmem,
  output reg[31:0] malu,
  output reg[31:0] mb,
  output reg[4:0]  mrn,
  output reg       misbranch,
  output reg       mexc_ovr,
  output reg[31:0] pcm
);

always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    mwreg <= 0;
    mm2reg <= 0;
    mwmem <= 0;
    malu <= 0;
    mb <= 0;
    mrn <= 0;
    misbranch <= 0;
    pcm <= 0;
    mexc_ovr <= 0;
  end
  else begin
    mwreg <= ewreg;
    mm2reg <= em2reg;
    mwmem <= ewmem;
    malu <= ealu;
    mb <= eb;
    mrn <= ern;
    misbranch <= eisbranch;
    pcm <= pce;
    mexc_ovr <= exc_ovr;
  end
end

endmodule

// MEM stage
module pipelined_mem_stage(
  input        we,
  input[31:0]  addr,
  input[31:0]  datain,
  input        clk,
  output[31:0] dataout
);
pl_exc_d_mem data_mem(clk, dataout, datain, addr, we);
endmodule

// MEM/WB pipeline register
module pipelined_mem_wb_reg(
  input           mwreg,
  input           mm2reg,
  input[31:0]     mmo,
  input[31:0]     malu,
  input[4:0]      mrn,
  input           clk,
  input           clrn,
  output reg      wwreg,
  output reg      wm2reg,
  output reg[31:0] wmo,
  output reg[31:0] walu,
  output reg[4:0]  wrn
);

always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    wwreg <= 0;
    wm2reg <= 0;
    wmo <= 0;
    walu <= 0;
    wrn <= 0;
  end
  else begin
    wwreg <= mwreg;
    wm2reg <= mm2reg;
    wmo <= mmo;
    walu <= malu;
    wrn <= mrn;
  end
end

endmodule

// WB stage
module pipelined_wb_stage(
  input[31:0]  walu,
  input[31:0]  wmo,
  input        wm2reg,
  output[31:0] wdi
);
assign wdi = (wm2reg ? wmo : walu);
endmodule

