`timescale 1ns/1ns
module fsqrt_newton_tb;
  reg  [31:0] d;
  reg   [1:0] rm;
  reg         fsqrt;
  reg         ena;
  reg         clk,clrn;
  wire [31:0] s;
  wire        busy;
  wire        stall;
  wire  [4:0] count;
  wire [25:0] reg_x;
  float_sqrt_newton fsn(d,rm,fsqrt,ena,clk,clrn,s,busy,stall,count,reg_x);
  initial begin
         clk   = 1;
         clrn  = 0;
         rm    = 0;
         ena   = 1;
         fsqrt = 0;
         d     = 32'h41100000;
    #2   clrn  = 1;
    #13  fsqrt = 1;
    #214 fsqrt = 0;
    #71  d     = 32'h00003200;
    #15  fsqrt = 1;
    #214 fsqrt = 0;
    $finish;
  end
  always #5 clk = !clk;
initial begin
  $dumpfile("float_sqrt_newton.vcd");
  $dumpvars;
end
endmodule
