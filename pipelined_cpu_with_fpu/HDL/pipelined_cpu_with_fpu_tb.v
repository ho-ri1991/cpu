`timescale 1ns/1ns
module pipelined_cpu_with_fpu_tb;

reg clk, memclk, clrn;
wire[31:0] pc, inst, ealu, malu, walu, wd, e3d, vramaddr, vramdata;
wire[4:0] wn, count_fdiv, count_fsqrt, e1n, e2n, e3n;
wire ww, stall_lw, stall_fp, stall_lwc1, stall_swc1, stall_mfc1, stall_mtc1, stall, e, vramwe;
pipelined_cpu_with_fpu plcpufpu(
  clk, memclk, clrn, pc, inst, ealu, malu, walu,
  wn, wd, ww, stall_lw, stall_fp, stall_lwc1, stall_swc1, stall_mfc1, stall_mtc1, stall,
  count_fdiv, count_fsqrt, e1n, e2n, e3n, e3d, e, vramaddr, vramwe, vramdata);

initial begin
  clk <= 1;
  clrn <= 0;
  memclk <= 0;
  #1 clrn <= 1;
//  #335 $finish;
end
//initial begin
//  $dumpfile("pipelined_cpu_with_fpu.vcd");
//  $dumpvars;
//end
always #1 memclk <= !memclk;
always #2 clk <= !clk;

endmodule
