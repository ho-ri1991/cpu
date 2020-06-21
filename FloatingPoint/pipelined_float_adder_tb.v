`timescale 1ns/1ns
module pipelined_fadder_tb;
reg  [31:0] a, b;
reg         sub;
reg         e;
reg  [1:0]  rm;
reg         clk, clrn;
wire [31:0] s;
pipelined_float_adder fadder(a,b,sub,rm,s,clk,clrn,e);
initial begin
      a    = 32'h3c600011;
      b    = 32'hbe820000;
      e    = 1;
      clk  = 1;
      clrn = 0;
      rm   = 0;
      sub  = 1;
  #2  clrn = 1;
  #13 rm   = 2'h1;
  #10 rm   = 2'h2;
  #10 rm   = 2'h3;
  #10 rm   = 2'h0;
      sub  = 0;
  #10 rm   = 2'h1;
  #10 rm   = 2'h2;
  #10 rm   = 2'h3;
  #10 rm   = 2'h0;
      sub  = 1;
      a    = 32'h3fffffff;
      b    = 32'hb3800000;
  #10 rm   = 2'h1;
  #10 rm   = 2'h2;
  #10 rm   = 2'h3;
  #10 rm   = 2'h0;
      sub  = 0;
  #10 rm   = 2'h1;
  #10 rm   = 2'h2;
  #10 rm   = 2'h3;
  #10 rm   = 2'h0;
      a    = 32'h7f800000;
      b    = 32'h7f800000;
  #10 sub  = 1;
  #10 sub  = 0;
      a    = 32'h7f00ffff;
      b    = 32'h7fe0000f;
  #10 sub  = 0;
      a    = 32'h7f7fffff;
      b    = 32'h7f7fffff;
  #10 sub  = 0;
      a    = 32'h7f00ffff;
      b    = 32'h00000000;
  #10 sub  = 0;
      a    = 32'h00800000;
      b    = 32'h007fffff;
  #10 sub  = 0;
      a    = 32'h00000007;
      b    = 32'h00000008;
  #10 sub  = 1;
  $finish;
end
always #5 clk = !clk;
initial begin
  $dumpfile("pipelined_float_adder.vcd");
  $dumpvars;
end
endmodule
