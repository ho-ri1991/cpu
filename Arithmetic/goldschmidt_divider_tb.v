`timescale 1ns/1ns

module goldschmidt_divider_tb();
reg [31:0] a, b;
reg start, clk, clrn;
wire [31:0] q;
wire busy, ready;
wire [2:0] count;
goldschmidt_divider gs(a, b, start, clk, clrn, q, busy, ready, count);
initial begin
  a = 32'd121;
  b = 32'd17;
  start = 0;
  clk = 1;
  clrn = 0;
  #1 clrn = 1;
     start = 1;
  #1 start = 1;
  #1 start = 0;
  #16 $finish;
end
initial begin
  $dumpfile("goldschmidt_divider.vcd");
  $dumpvars;
end
always #1 clk = !clk;
endmodule

