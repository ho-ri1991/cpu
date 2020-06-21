`timescale 1ns/1ns
module integer_to_float_tb;

reg[31:0] d;
wire[31:0] a;
wire p_lost;
integer_to_float i2f(d, a, p_lost);
initial begin
     d = 32'h1fffffff;
  #1 d = 32'h00000001;
  #1 d = 32'h7fffff80;
  #1 d = 32'h7fffffc0;
  #1 d = 32'h80000000;
  #1 d = 32'h80000040;
  #1 d = 32'hffffffff;
  #1 $finish;
end
initial begin
  $dumpfile("integer_to_float.vcd");
  $dumpvars;
end

endmodule
