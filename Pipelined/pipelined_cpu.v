module pipelined_cpu(
  input        clk,
  input        clrn,
  output[31:0] pc,
  output[31:0] inst,
  output[31:0] ealu,
  output[31:0] malu,
  output[31:0] wdi
);

wire[31:0] npc, bpc, jpc, pc4, instruction, dpc4, ins;
assign inst = ins;
wire[1:0] pcsrc;
wire wpc; // program counter write enable (stall when wpc is zero)
reg[31:0] program_counter;
assign pc = program_counter;
always @(posedge clk or negedge clrn) begin
  if(!clrn)
    program_counter <= 32'b0;
  else begin
    if(wpc)
      program_counter <= npc;
  end
end

wire wreg, m2reg, wmem, jal, aluimm, shift, nostall;
wire[3:0] aluc;
wire[4:0] rn;
wire[31:0] a, b, dimm;

wire ewreg, em2reg, ewmem, ejal, ealuimm, eshift;
wire[3:0] ealuc;
wire[4:0] ern0, ern;
wire[31:0] ea, eb, ealu, epc4, eimm;

wire mwreg, mm2reg, mwmem;
wire[4:0] mrn;
wire[31:0] malu, mmo, mb;

wire wwreg, wm2reg;
wire[4:0] wrn;
wire[31:0] wdi, wmo, walu;

pipelined_if_stage if_stage(
  .pc(program_counter), .bpc(bpc), .rpc(a), .jpc(jpc), .pcsrc(pcsrc),
  .npc(npc), .pc4(pc4), .inst(instruction));

pipelined_if_id_reg if_id_reg(
  .pc4(pc4), .inst(instruction), .wir(wpc), .clk(clk), .clrn(clrn),
  .dpc4(dpc4), .ins(ins));

pipelined_id_stage id_stage(
  .mwreg(mwreg), .mrn(mrn), .ern(ern), .ewreg(ewreg), .em2reg(em2reg),
  .mm2reg(mm2reg), .dpc4(dpc4), .inst(ins), .wrn(wrn), .wdi(wdi), .ealu(ealu),
  .malu(malu), .mmo(mmo), .wwreg(wwreg), .clk(clk), .clrn(clrn),
  .bpc(bpc), .jpc(jpc), .pcsrc(pcsrc), .nostall(wpc), .wreg(wreg), .m2reg(m2reg), .wmem(wmem),
  .aluc(aluc), .aluimm(aluimm), .a(a), .b(b), .dimm(dimm), .rn(rn), .shift(shift), .jal(jal));

pipelined_id_exe_reg id_exe_reg(
  .dwreg(wreg), .dm2reg(m2reg), .dwmem(wmem), .daluc(aluc), .daluimm(aluimm),
  .da(a), .db(b), .dimm(dimm), .drn(rn), .dshift(shift), .djal(jal), .dpc4(dpc4), .clk(clk), .clrn(clrn),
  .ewreg(ewreg), .em2reg(em2reg), .ewmem(ewmem), .ealuc(ealuc), .ealuimm(ealuimm), .ea(ea), .eb(eb), .eimm(eimm), .ern(ern0), .eshift(eshift), .ejal(ejal), .epc4(epc4));

pipelined_exe_stage exe_stage(
  .ealuc(ealuc), .ealuimm(ealuimm), .ea(ea), .eb(eb), .eimm(eimm), .eshift(eshift),
  .ern0(ern0), .epc4(epc4), .ejal(ejal), .ern(ern), .ealu(ealu));

pipelined_exe_mem_reg exe_mem_reg(
  .ewreg(ewreg), .em2reg(em2reg), .ewmem(ewmem), .ealu(ealu), .eb(eb), .ern(ern), .clk(clk), .clrn(clrn),
  .mwreg(mwreg), .mm2reg(mm2reg), .mwmem(mwmem), .malu(malu), .mb(mb), .mrn(mrn));

pipelined_mem_stage mem_stage(
  .we(mwmem), .addr(malu), .datain(mb), .clk(clk), .dataout(mmo));

pipelined_mem_wb_reg mem_wb_reg(
  .mwreg(mwreg), .mm2reg(mm2reg), .mmo(mmo), .malu(malu), .mrn(mrn), .clk(clk), .clrn(clrn),
  .wwreg(wwreg), .wm2reg(wm2reg), .wmo(wmo), .walu(walu), .wrn(wrn));

pipelined_wb_stage wb_state(
  .walu(walu), .wmo(wmo), .wm2reg(wm2reg), .wdi(wdi));

endmodule

// IF stage
module pipelined_if_stage(
  input[31:0] pc,  // current program counter
  input[31:0] bpc, // branch program counter
  input[31:0] rpc, // jr program counter
  input[31:0] jpc, // jump program counter
  input[1:0]  pcsrc,
  output reg[31:0] npc,
  output reg[31:0] pc4,
  output[31:0] inst
);

always @* begin
  pc4 = pc + 4;
  case(pcsrc)
    2'b00: npc = pc4;
    2'b01: npc = bpc;
    2'b10: npc = rpc;
    2'b11: npc = jpc;
  endcase
end
pl_inst_mem inst_mem(pc, inst);

endmodule

// IF/ID pipeline register
module pipelined_if_id_reg(
  input[31:0] pc4,
  input[31:0] inst,
  input       wir, // IF/ID stage reg write enable (pipeline stall when wir is zero)
  input       clk,
  input       clrn,
  output[31:0] dpc4,
  output[31:0] ins
);

reg[31:0] pc_plus4;
reg[31:0] instruction;

always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    pc_plus4 <= 32'b0;
    instruction <= 32'b0;
  end else begin
    if(wir) begin
      pc_plus4 <= pc4;
      instruction <= inst;
    end
  end
end
assign dpc4 = pc_plus4;
assign ins = instruction;

endmodule

// ID stage
module pipelined_id_stage(
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
  output[3:0]     aluc,    // ALU control signal
  output           aluimm,  // input b of ALU is immediate
  output reg[31:0] a,       // operand a
  output reg[31:0] b,       // operand b
  output[31:0] dimm,        // decoded immediate
  output[4:0]  rn,          // decoded write register number
  output       shift,       // instruction is shift
  output       jal          // instruction is jal
);

wire[5:0] op    = inst[31:26];
wire[4:0] rs    = inst[25:21];
wire[4:0] rt    = inst[20:16];
wire[4:0] rd    = inst[15:11];
wire[5:0] func  = inst[5:0];
wire[15:0] imm  = inst[15:0];
wire[25:0] addr = inst[25:0];

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

// instructions that use rs
wire i_rs = i_add | i_sub | i_and | i_or | i_xor | i_jr |
            i_addi | i_andi | i_ori | i_xori | i_lw | i_sw | i_beq | i_bne;
// instructions that use rt
wire i_rt = i_add | i_sub | i_and | i_or | i_xor |
            i_sll | i_srl | i_sra | i_sw | i_beq | i_bne;
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
assign wmem     = i_sw & nostall;
assign aluc[3]  = i_sra;
assign aluc[2]  = i_sub | i_or | i_srl | i_sra | i_ori | i_lui;
assign aluc[1]  = i_xor | i_sll| i_srl | i_sra | i_lui | i_xori | i_beq | i_bne;
assign aluc[0]  = i_and | i_or | i_sll | i_srl | i_sra | i_andi | i_ori;
assign shift    = i_sll | i_srl| i_sra;
assign aluimm   = i_addi | i_andi | i_ori | i_xori | i_lw | i_lui | i_sw;
assign pcsrc[1] = i_jr | i_j | i_jal;
assign pcsrc[0] = (i_beq & rs_rt_equal) | (i_bne & ~rs_rt_equal) | i_j | i_jal;
assign wreg     = (i_add | i_sub | i_and | i_or | i_xor | i_sll | i_srl | i_sra |
                   i_addi | i_andi | i_ori | i_xori | i_lw | i_lui | i_jal) & nostall;
wire regrt    = i_addi | i_andi | i_ori | i_xori | i_lw | i_lui;
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

endmodule

// ID/EXE pipeline register
module pipelined_id_exe_reg(
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
  output reg[31:0] epc4
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
  end
end

endmodule

// EXE stage
module pipelined_exe_stage(
  input[3:0]   ealuc,
  input        ealuimm,
  input[31:0]  ea,
  input[31:0]  eb,
  input[31:0]  eimm,
  input        eshift,
  input[4:0]   ern0,
  input[31:0]  epc4,
  input        ejal,
  output[4:0]  ern,
  output[31:0] ealu
);

wire[31:0] esa = {27'b0, eimm[10:6]};
wire[31:0] alu_a = (eshift ? esa : ea);
wire[31:0] alu_b = (ealuimm ? eimm : eb);
reg[31:0] alu_out;
always @* begin
  casex(ealuc)
    4'bx000: alu_out <= alu_a + alu_b;
    4'bx100: alu_out <= alu_a - alu_b;
    4'bx001: alu_out <= alu_a & alu_b;
    4'bx101: alu_out <= alu_a | alu_b;
    4'bx010: alu_out <= alu_a ^ alu_b;
    4'bx110: alu_out <= {alu_b[15:0], 16'b0};
    4'b0011: alu_out <= alu_b << alu_a;
    4'b0111: alu_out <= alu_b >> alu_a;
    4'b1111: alu_out <= $signed(alu_b) >>> alu_a;
    default: alu_out <= 32'b0;
  endcase
end
assign ealu = (ejal ? epc4 + 32'd4 : alu_out);
assign ern = (ejal ? 32'd31 : ern0);

endmodule

// EXE/MEM pipeline register
module pipelined_exe_mem_reg(
  input            ewreg,
  input            em2reg,
  input            ewmem,
  input[31:0]      ealu,
  input[31:0]      eb,
  input[4:0]       ern,
  input            clk,
  input            clrn,
  output reg       mwreg,
  output reg       mm2reg,
  output reg       mwmem,
  output reg[31:0] malu,
  output reg[31:0] mb,
  output reg[4:0]  mrn
);

always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    mwreg <= 0;
    mm2reg <= 0;
    mwmem <= 0;
    malu <= 0;
    mb <= 0;
    mrn <= 0;
  end
  else begin
    mwreg <= ewreg;
    mm2reg <= em2reg;
    mwmem <= ewmem;
    malu <= ealu;
    mb <= eb;
    mrn <= ern;
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
pl_data_mem data_mem(clk, dataout, datain, addr, we);
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

