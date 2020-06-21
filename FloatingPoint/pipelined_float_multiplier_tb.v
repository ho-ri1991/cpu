`timescale 1ns/1ns
module pipelined_float_multiplier_tb;
reg[31:0] a, b;
reg       e;
reg[1:0]  rm;
reg       clk, clrn;
wire[31:0] s;
pipelined_float_multiplier pfm(a,b,rm,s,clk,clrn,e);
initial begin
      e    = 1;
      clk  = 1;
      clrn = 0;
      rm   = 0;
  #1  a    = 32'h3fc00000;
      b    = 32'h3fc00000;
  #1  clrn = 1;
  #9  a    = 32'h00800000;
      b    = 32'h00800000;
  #10 a    = 32'h7f7fffff;
      b    = 32'h7f7fffff;
  #10 a    = 32'h00800000;
      b    = 32'h3f000000;
  #10 a    = 32'h003fffff;
      b    = 32'h40000000;
  #10 a    = 32'h7f800000;
      b    = 32'h00ffffff;
  #10 a    = 32'h7f800000;
      b    = 32'h00000000;
  #10 a    = 32'h7ff000ff;
      b    = 32'h3f80ff00;
  $finish;
end
always #5 clk = !clk;
initial begin
  $dumpfile("pipelined_float_multiplier.vcd");
  $dumpvars;
end
endmodule
