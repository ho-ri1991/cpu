module integer_unit(
  input[4:0]   e1n,// write port number from fpu E1 stage, for detecting data hazard
  input[4:0]   e2n,// write port number from fpu E2 stage, for detecting data hazard
  input[4:0]   e3n,// write port number from fpu E3 stage, for detecting data hazard
  input        e1w,// write enable of fpu register file from fpu E1 stage, for detecting data hazard
  input        e2w,// write enable of fpu register file from fpu E2 stage, for detecting data hazard
  input        e3w,// write enable of fpu register file from fpu E3 stage, for detecting data hazard
  input        stall_fdiv_fsqrt,// stall signal from fpu (newton iteration for divider and sqrt)
  input        st,// not used now, connected with 0 signal
  input[31:0]  dfb,// data from b read port of fpu register file, for storing floating point data
  input[31:0]  e3d,// calculation result from E3 stage, data forwarding for store floating point data
  input        clk,
  input        memclk,
  input        clrn,
  output[4:0]  fs,// read port number for fpu register
  output[4:0]  ft,// read port number for fpu register
  output[31:0] fpu_wd,// write data from integer unit to fpu register (for mtc1 and lwc1)
  output[4:0]  fpu_rn,// write port number from integer unit to fpu register (for mtc1 and lwc1)
  output       fpu_we,// write enable of fpu register (for mtc1 and lwc1)
  output[31:0] mmo,// loaded fpu data fro data memory, data forwarding from MEM stage in integer unit
  output       fwdla,// selector of input to fpu (fpu register file or forwarded data from MEM state)
  output       fwdlb,// selector of input to fpu (fpu register file or forwarded data from MEM state)
  output       fwdfa,// selector of input to fpu (fpu register file or fpu result in E3 stege)
  output       fwdfb,// selector of input to fpu (fpu register file or fpu result in E3 stege)
  output[4:0]  fd,// write port number of fpu (from ID stage)
  output[2:0]  fc,// fpu control signal (from ID stage)
  output       wf,// write enable signal of fpu register (from ID stage)
  output       fasmds,// not used now
  output[31:0] pc,// program counter in IF stage
  output[31:0] inst,// instruction in ID stage
  output[31:0] ealu,// ALU output in EXE stage for data forwarding
  output[31:0] malu,// ALU output in MEM stage for data forwarding
  output[31:0] walu,// ALU output in WB stage for data forwarding
  output       stall_lw,
  output       stall_fp,
  output       stall_lwc1,
  output       stall_swc1,
  output       stall_mfc1,
  output       stall_mtc1,
  output[31:0] vramaddr,
  output       vramwe,
  output[31:0] vramdata
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

wire wreg, m2reg, wmem, jal, aluimm, shift, wfpr, fwdfe;
wire[3:0] aluc;
wire[4:0] rn;
wire[31:0] a, b, dimm;

wire ewreg, em2reg, ewmem, ejal, ealuimm, eshift, ewfpr, efwdfe;
wire[3:0] ealuc;
wire[4:0] ern0, ern;
wire[31:0] ea, eb, epc4, eimm, eb_out;

wire mwreg, mm2reg, mwmem, mwfpr;
wire[4:0] mrn;
wire[31:0] mb;

wire wwreg, wm2reg, wwfpr;
wire[31:0] wdi, wmo;
wire[4:0] wrn;

pipelined_if_stage if_stage(
  .pc(program_counter), .bpc(bpc), .rpc(a), .jpc(jpc), .pcsrc(pcsrc),
  .memclk(memclk),
  .npc(npc), .pc4(pc4), .inst(instruction));

pipelined_if_id_reg if_id_reg(
  .pc4(pc4), .inst(instruction), .wir(wpc), .clk(clk), .clrn(clrn),
  .dpc4(dpc4), .ins(ins));

pipelined_id_stage id_stage(
  .mwreg(mwreg), .mrn(mrn), .ern(ern), .ewreg(ewreg), .em2reg(em2reg),
  .mm2reg(mm2reg), .dpc4(dpc4), .inst(ins), .wrn(wrn), .wmo(wmo), .wdi(wdi), .ealu(ealu),
  .malu(malu), .mmo(mmo), .wwreg(wwreg), .clk(clk), .clrn(clrn),
  .bpc(bpc), .jpc(jpc), .pcsrc(pcsrc), .nostall(wpc), .wreg(wreg), .m2reg(m2reg), .wmem(wmem),
  .aluc(aluc), .aluimm(aluimm), .a(a), .b(b), .dimm(dimm), .rn(rn), .shift(shift), .jal(jal),
  .fs(fs), .ft(ft), .fd(fd), .fc(fc), .wf(wf), .dfb(dfb), .e3d(e3d),
  .e1n(e1n),.e2n(e2n), .e3n(e3n), .e1w(e1w), .e2w(e2w), .e3w(e3w), .stall_fdiv_fsqrt(stall_fdiv_fsqrt),
  .st(st), .mwfpr(mwfpr), .ewfpr(ewfpr), .wwfpr(wwfpr), .wfpr(wfpr), .fwdfe(fwdfe), .fwdla(fwdla), .fwdlb(fwdlb),
  .fwdfa(fwdfa), .fwdfb(fwdfb), .fasmds(fasmds),
  .stall_lw(stall_lw), .stall_fp(stall_fp), .stall_lwc1(stall_lwc1), .stall_swc1(stall_swc1),
  .stall_mfc1(stall_mfc1), .stall_mtc1(stall_mtc1), .fpu_wd(fpu_wd), .fpu_rn(fpu_rn), .fpu_we(fpu_we));

pipelined_id_exe_reg id_exe_reg(
  .dwreg(wreg), .dm2reg(m2reg), .dwmem(wmem), .daluc(aluc), .daluimm(aluimm),
  .da(a), .db(b), .dimm(dimm), .drn(rn), .dshift(shift), .djal(jal), .dpc4(dpc4),
  .dwfpr(wfpr), .dfwdfe(fwdfe), .clk(clk), .clrn(clrn),
  .ewreg(ewreg), .em2reg(em2reg), .ewmem(ewmem), .ealuc(ealuc), .ealuimm(ealuimm),
  .ea(ea), .eb(eb), .eimm(eimm), .ern(ern0), .eshift(eshift), .ejal(ejal), .epc4(epc4),
  .ewfpr(ewfpr), .efwdfe(efwdfe));

pipelined_exe_stage exe_stage(
  .ealuc(ealuc), .ealuimm(ealuimm), .ea(ea), .eb(eb), .eimm(eimm), .eshift(eshift),
  .efwdfe(efwdfe), .e3d(e3d),
  .ern0(ern0), .epc4(epc4), .ejal(ejal), .ern(ern), .ealu(ealu), .eb_out(eb_out));

pipelined_exe_mem_reg exe_mem_reg(
  .ewreg(ewreg), .em2reg(em2reg), .ewmem(ewmem), .ealu(ealu), .eb(eb_out), .ern(ern),
  .ewfpr(ewfpr), .clk(clk), .clrn(clrn),
  .mwreg(mwreg), .mm2reg(mm2reg), .mwmem(mwmem), .malu(malu), .mb(mb), .mrn(mrn),
  .mwfpr(mwfpr));

pipelined_mem_stage mem_stage(
  .we(mwmem), .addr(malu), .datain(mb), .clk(memclk), .dataout(mmo),
  .vramaddr(vramaddr), .vramwe(vramwe), .vramdata(vramdata));

pipelined_mem_wb_reg mem_wb_reg(
  .mwreg(mwreg), .mm2reg(mm2reg), .mmo(mmo), .malu(malu), .mrn(mrn), .mwfpr(mwfpr),
  .clk(clk), .clrn(clrn),
  .wwreg(wwreg), .wm2reg(wm2reg), .wmo(wmo), .walu(walu), .wrn(wrn), .wwfpr(wwfpr));

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
  input       memclk,
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
inst_mem mem(pc[9:2], memclk, inst);

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
  input[31:0]      wmo,   // data memory out from WB stage
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
  output[31:0] dimm,        // decoded immediate
  output[4:0]  rn,          // decoded write register number
  output       shift,       // instruction is shift
  output       jal,         // instruction is jal
  output[4:0]  fs,          // read port number of fpu register file
  output[4:0]  ft,          // read port number of fpu register file
  output[4:0]  fd,          // write port number of fpu register file (writing fpu result)
  output       wf,          // write enable of fpu register file (writing fpu result)
  output[2:0]  fc,          // fpu control signal
  input[31:0]  dfb,         // data from fpu register file (for store fpu data)
  input[31:0]  e3d,         // data from E3 stage, data forwarding
  input[4:0]   e1n,
  input[4:0]   e2n,
  input[4:0]   e3n,
  input        e1w,
  input        e2w,
  input        e3w,
  input        stall_fdiv_fsqrt,
  input        st,
  input        mwfpr,
  input        ewfpr,
  input        wwfpr,
  output       wfpr,
  output       fwdfe,
  output       fwdla,
  output       fwdlb,
  output       fwdfa,
  output       fwdfb,
  output       fasmds,
  output       stall_lw,
  output       stall_fp,
  output       stall_lwc1,
  output       stall_swc1,
  output       stall_mfc1,
  output       stall_mtc1,
  output[31:0] fpu_wd,
  output[4:0]  fpu_rn,
  output       fpu_we
);

wire[5:0] op    = inst[31:26];
wire[4:0] rs    = inst[25:21];
wire[4:0] op1   = rs;
wire[4:0] rt    = inst[20:16];
wire[4:0] rd    = inst[15:11];
wire[5:0] func  = inst[5:0];
wire[15:0] imm  = inst[15:0];
wire[25:0] addr = inst[25:0];

assign fs = inst[15:11];
assign ft = inst[20:16];
assign fd = inst[10:6];

// output of register file
wire[31:0] qa, qb;

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
wire i_lwc1    = (op == 6'b110001); // load word coprocessor 1 (fpu)
wire i_swc1    = (op == 6'b111001); // store word coprocessor 1 (fpu)
wire i_fadd    = (op == 6'b010001) & (op1 == 5'b10000) & (func == 6'b000000); // float add
wire i_fsub    = (op == 6'b010001) & (op1 == 5'b10000) & (func == 6'b000001); // float sub
wire i_fmul    = (op == 6'b010001) & (op1 == 5'b10000) & (func == 6'b000010); // float mul
wire i_fdiv    = (op == 6'b010001) & (op1 == 5'b10000) & (func == 6'b000011); // float div
wire i_fsqrt   = (op == 6'b010001) & (op1 == 5'b10000) & (func == 6'b000100); // float div
wire i_itof    = (op == 6'b010001) & (op1 == 5'b10100) & (func == 6'b100000); // integer to float
wire i_ftoi    = (op == 6'b010001) & (op1 == 5'b10000) & (func == 6'b100000); // float to integer
wire i_mfc1    = (op == 6'b010001) & (op1 == 5'b00000) & (func == 6'b000000); // move from coprocessor1 (fpu)
wire i_mtc1    = (op == 6'b010001) & (op1 == 5'b00100) & (func == 6'b000000); // move to coprocessor1 (fpu)

// instructions that use rs
wire i_rs = i_add | i_sub | i_and | i_or | i_xor | i_jr |
            i_addi | i_andi | i_ori | i_xori | i_lw | i_sw | i_beq | i_bne |
            i_lwc1 | i_swc1;
// instructions that use rt
wire i_rt = i_add | i_sub | i_and | i_or | i_xor |
            i_sll | i_srl | i_sra | i_sw | i_beq | i_bne | i_mtc1;
// stall occurs the instruction in EXE stage is load word instruction
assign stall_lw = ewreg & em2reg & (ern != 0) & (i_rs & (ern == rs) | i_rt & (ern == rt));

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
assign wmem     = (i_sw | i_swc1) & nostall;
assign aluc[3]  = i_sra;
assign aluc[2]  = i_sub | i_or | i_srl | i_sra | i_ori | i_lui;
assign aluc[1]  = i_xor | i_sll| i_srl | i_sra | i_lui | i_xori | i_beq | i_bne;
assign aluc[0]  = i_and | i_or | i_sll | i_srl | i_sra | i_andi | i_ori;
assign shift    = i_sll | i_srl| i_sra;
assign aluimm   = i_addi | i_andi | i_ori | i_xori | i_lw | i_lui | i_sw | i_lwc1 | i_swc1;
assign pcsrc[1] = i_jr | i_j | i_jal;
assign pcsrc[0] = (i_beq & rs_rt_equal) | (i_bne & ~rs_rt_equal) | i_j | i_jal;
assign wreg     = (i_add | i_sub | i_and | i_or | i_xor | i_sll | i_srl | i_sra |
                   i_addi | i_andi | i_ori | i_xori | i_lw | i_lui | i_jal) & nostall;
wire regrt    = i_addi | i_andi | i_ori | i_xori | i_lw | i_lui | i_lwc1;
wire sext     = i_addi | i_lw | i_sw | i_beq | i_bne | i_lwc1 | i_swc1;
assign rn = (regrt ? rt : rd);

wire swfp;
wire fwdf;
reg[31:0] b0;
always @* begin
  case(fwda)
    2'b00: a = qa;
    2'b01: a = ealu;
    2'b10: a = malu;
    2'b11: a = mmo;
  endcase
  case(fwdb)
    2'b00: b0 = qb;
    2'b01: b0 = ealu;
    2'b10: b0 = malu;
    2'b11: b0 = mmo;
  endcase
  case({swfp, fwdf})
    2'b00: b = b0;
    2'b01: b = e3d;
    2'b10: b = dfb;
    2'b11: b = e3d;
  endcase
end

assign dimm = {{16{sext & imm[15]}}, imm};
assign jpc = {dpc4[31:28], addr, 2'b00};
assign bpc = dpc4 + {dimm[29:0], 2'b00};

// fpu op code
//     fop[2] fop[1] fop[0]
// add     0      0      0
// sub     0      0      1
// mul     0      1      0
// itof    0      1      1
// div     1      0      0
// ftoi    1      0      1
// sqrt    1      1      0
wire[2:0] fop;
assign fop[0] = i_fsub | i_itof | i_ftoi;
assign fop[1] = i_fmul | i_fsqrt| i_itof;
assign fop[2] = i_fdiv | i_fsqrt| i_ftoi;
wire i_fs = i_fadd | i_fsub | i_fmul | i_fdiv | i_fsqrt | i_ftoi | i_itof; // use fs
wire i_ft = i_fadd | i_fsub | i_fmul | i_fdiv;           // use ft
assign stall_fp = (e1w & (i_fs & (e1n == fs) | i_ft & (e1n == ft))) |
                  (e2w & (i_fs & (e2n == fs) | i_ft & (e2n == ft)));
assign fwdfa = e3w & (e3n == fs);   // forward fpu E3 stage to fp a
assign fwdfb = e3w & (e3n == ft);   // forward fpu E3 stage to fp b
assign wfpr  = i_lwc1 & nostall;    // fp register file port y write enable
assign fwdla = mwfpr & (mrn == fs); // forward mmo to fp a
assign fwdlb = mwfpr & (mrn == ft); // forward mmo to fp b
assign stall_lwc1 = ewfpr & (i_fs & (ern == fs) | i_ft & (ern == ft));
assign swfp = i_swc1;
assign fwdf  = swfp & e3w & (ft == e3n);
assign fwdfe = swfp & e2w & (ft == e2n);
assign stall_swc1 = swfp & e1w & (ft == e1n);
assign stall_mfc1 = i_mfc1 &
                    ((e1w & (fs == e1n)) | (e2w & (fs == e2n)) | // wait for fpu result
                     (ewreg & (ern == rt)) | (mwreg & (mrn == rt)) | (wwreg & (wrn == rt))); // wait for writing previous instructions, TODO: remove WB stage
assign stall_mtc1 = i_mtc1 &
                    ((e1w & (fs == e1n)) | (e2w & (fs == e2n)) | // wait for writing previous fpu instructions
                     (ewfpr & (fs == ern)) | (mwfpr & (fs == mrn)) | (wwfpr & (fs == wrn)) | // wait for previous lwc1
                     (ewreg & em2reg & (rt == ern))); // wait for lw
wire stall_others = stall_lw | stall_fp | stall_lwc1 | stall_swc1 | stall_mfc1 | stall_mtc1 | st;
assign nostall = ~(stall_fdiv_fsqrt | stall_others);
assign fc = fop & {3{~stall_others}};
assign wf = i_fs & nostall;
assign fasmds = i_fs;
assign fpu_wd = wwfpr ? wmo : b0;
assign fpu_rn = wwfpr ? wrn : fs;
assign fpu_we = wwfpr | (i_mtc1 & ~stall_mtc1);
wire[31:0] from_fpu = (e3w & (ft == e3n) ? e3d : dfb);
wire[31:0] regfile_wd = (i_mfc1 & ~stall_mfc1 ? from_fpu : wdi);
wire[4:0]  regfile_wn = (i_mfc1 & ~stall_mfc1 ? rt : wrn);
wire       regfile_we = wwreg | (i_mfc1 & ~stall_mfc1);

regfile regfile0(
  .rna(rs), .rnb(rt), .d(regfile_wd),
  .wn(regfile_wn), .we(regfile_we), .clk(~clk), .clrn(clrn), // invert clk for negative edge trigger
  .qa(qa), .qb(qb));

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
  input        dwfpr,
  input        dfwdfe,
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
  output reg       ewfpr,
  output reg       efwdfe
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
    ewfpr <= 0;
    efwdfe <= 0;
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
    ewfpr <= dwfpr;
    efwdfe <= dfwdfe;
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
  input        efwdfe,
  input[31:0]  e3d,
  output[4:0]  ern,
  output[31:0] ealu,
  output[31:0] eb_out
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
assign eb_out = (efwdfe ? e3d : eb);

endmodule

// EXE/MEM pipeline register
module pipelined_exe_mem_reg(
  input            ewreg,
  input            em2reg,
  input            ewmem,
  input[31:0]      ealu,
  input[31:0]      eb,
  input[4:0]       ern,
  input            ewfpr,
  input            clk,
  input            clrn,
  output reg       mwreg,
  output reg       mm2reg,
  output reg       mwmem,
  output reg[31:0] malu,
  output reg[31:0] mb,
  output reg[4:0]  mrn,
  output reg       mwfpr
);

always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    mwreg <= 0;
    mm2reg <= 0;
    mwmem <= 0;
    malu <= 0;
    mb <= 0;
    mrn <= 0;
    mwfpr <= 0;
  end
  else begin
    mwreg <= ewreg;
    mm2reg <= em2reg;
    mwmem <= ewmem;
    malu <= ealu;
    mb <= eb;
    mrn <= ern;
    mwfpr <= ewfpr;
  end
end

endmodule

// MEM stage
module pipelined_mem_stage(
  input        we,
  input[31:0]  addr,
  input[31:0]  datain,
  input        clk,
  output[31:0] dataout,
  output[31:0] vramaddr,
  output       vramwe,
  output[31:0] vramdata
);
// vram space c0000000-dfffffff
assign vramwe = addr[31] & addr[30] & ~addr[29] & we;
assign vramaddr = {3'b0, addr[28:0]};
wire[31:0] datamem_out, vram_out;
data_mem mem(addr[9:2], clk, datain, ~vramwe & we, datamem_out);
assign dataout = vramwe ? 32'b0 : datamem_out;
assign vramdata = datain;
endmodule

// MEM/WB pipeline register
module pipelined_mem_wb_reg(
  input           mwreg,
  input           mm2reg,
  input[31:0]     mmo,
  input[31:0]     malu,
  input[4:0]      mrn,
  input           mwfpr,
  input           clk,
  input           clrn,
  output reg      wwreg,
  output reg      wm2reg,
  output reg[31:0] wmo,
  output reg[31:0] walu,
  output reg[4:0]  wrn,
  output reg       wwfpr
);

always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    wwreg <= 0;
    wm2reg <= 0;
    wmo <= 0;
    walu <= 0;
    wrn <= 0;
    wwfpr <= 0;
  end
  else begin
    wwreg <= mwreg;
    wm2reg <= mm2reg;
    wmo <= mmo;
    walu <= malu;
    wrn <= mrn;
    wwfpr <= mwfpr;
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

