`timescale 1ns/1ns
module pipelined_cpu_tb;

reg clk, clrn;
wire[31:0] pc, inst, ealu, malu, wdi;
wire wmem;
pipelined_cpu plcpu(clk, clrn, pc, inst, ealu, malu, wdi);

initial begin
  clk = 1;
  clrn = 0;
  #1 clrn = 1;
  #335 $finish;
end
initial begin
  $dumpfile("pipelined_cpu.vcd");
  $dumpvars;
end
always #2 clk = !clk;

endmodule
