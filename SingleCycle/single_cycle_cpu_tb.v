`timescale 1ns/1ns

module single_cycle_cpu_tb;

reg clk, clrn;
wire[31:0] pc, inst, aluout, memout, data;
wire wmem;
single_cycle_cpu sccpu(clk, clrn, inst, memout, pc, wmem, aluout, data);
scinstmem imem(pc, inst);
scdatamem dmem(clk, memout, data, aluout, wmem);

initial begin
  clk = 1;
  clrn = 0;
  #1 clrn = 1;
  #140 $finish;
end
initial begin
  $dumpfile("single_cycle_cpu.vcd");
  $dumpvars;
end
always #1 clk = !clk;

endmodule

