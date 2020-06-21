`timescale 1ns/1ns

module single_cycle_cpu_with_interrupt_tb;

reg clk, clrn, intr;
wire[31:0] pc, inst, aluout, memout, data;
wire wmem, inta;
single_cycle_cpu_with_interrupt sccpu(clk, clrn, inst, memout, pc, wmem, aluout, data, intr, inta);
scinstmem imem(pc, inst);
scdatamem dmem(clk, memout, data, aluout, wmem);

initial begin
  clk = 1;
  clrn = 0;
  intr = 0;
  #1 clrn = 1;
  #94 intr = 1;
  #2  intr = 0;
  #140 $finish;
end
initial begin
  $dumpfile("single_cycle_cpu_with_interrupts.vcd");
  $dumpvars;
end
always #1 clk = !clk;

endmodule

