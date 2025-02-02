module pipelined_cpu_with_fpu(
  input clk,
  input memclk,
  input clrn,
  output[31:0] pc,
  output[31:0] inst,
  output[31:0] ealu,
  output[31:0] malu,
  output[31:0] walu,
  output[4:0]  wn,
  output[31:0] wd,
  output       ww,
  output       stall_lw,
  output       stall_fp,
  output       stall_lwc1,
  output       stall_swc1,
  output       stall_mfc1,
  output       stall_mtc1,
  output       stall,
  output[4:0]  count_fdiv,
  output[4:0]  count_fsqrt,
  output[4:0]  e1n,
  output[4:0]  e2n,
  output[4:0]  e3n,
  output[31:0] e3d,
  output       e,
  output[31:0] vramaddr,
  output       vramwe,
  output[31:0] vramdata
);

wire[31:0] qfa, qfb, fa, fb, dfa, dfb, mmo, fpu_wd;
wire[4:0]  fs, ft, fd, fpu_rn;
wire[2:0]  fc;
wire[2:0]  e1c, e2c, e3c;
wire fwdla, fwdlb, fwdfa, fwdfb, wf, fasmds, e1w, e2w, e3w, fpu_we;
integer_unit iu(
  .e1n(e1n), .e2n(e2n), .e3n(e3n), .e1w(e1w), .e2w(e2w), .e3w(e3w),
  .stall_fdiv_fsqrt(stall), .st(1'b0), .dfb(dfb), .e3d(e3d),
  .clk(clk), .clrn(clrn), .memclk(memclk),
  .fs(fs), .ft(ft), .fpu_wd(fpu_wd), .fpu_rn(fpu_rn), .fpu_we(fpu_we), .mmo(mmo),
  .fwdla(fwdla), .fwdlb(fwdlb), .fwdfa(fwdfa), .fwdfb(fwdfb),
  .fd(fd), .fc(fc), .wf(wf), .fasmds(fasmds), .pc(pc), .inst(inst),
  .ealu(ealu), .malu(malu), .walu(walu),
  .stall_lw(stall_lw), .stall_fp(stall_fp), .stall_lwc1(stall_lwc1),
  .stall_swc1(stall_swc1), .stall_mfc1(stall_mfc1), .stall_mtc1(stall_mtc1),
  .vramaddr(vramaddr), .vramwe(vramwe), .vramdata(vramdata));

regfile2w fpr(
  .rna(fs), .rnb(ft),
  .dx(wd), .wnx(wn), .wex(ww),
  .dy(fpu_wd), .wny(fpu_rn), .wey(fpu_we),
  .clk(~clk), .clrn(clrn), .qa(qfa), .qb(qfb));

assign fa = (fwdla ? mmo : qfa);
assign fb = (fwdlb ? mmo : qfb);
assign dfa = (fwdfa ? e3d : fa);
assign dfb = (fwdfb ? e3d : fb);

floating_point_unit fpu(
  .a(dfa), .b(dfb), .fc(fc), .wf(wf), .fd(fd), .ein1(1'b1), .clk(clk), .clrn(clrn),
  .ed(e3d), .wd(wd), .wn(wn), .ww(ww), .st_ds(stall),
  .e1n(e1n), .e1w(e1w), .e2n(e2n), .e2w(e2w), .e3n(e3n), .e3w(e3w),
  .e1c(e1c), .e2c(e2c), .e3c(e3c), .count_div(count_fdiv), .count_sqrt(count_fsqrt),
  .e(e), .ein2(1'b1));

endmodule
