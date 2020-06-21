`timescale 1ns/1ns
module float_to_integer_tb;

reg[31:0] a;
wire[31:0] d;
wire p_lost, denorm, invalid;
float_to_integer f2i(a, d, p_lost, denorm, invalid);
initial begin
     a = 32'h4effffff;
  #1 a = 32'h4f000000;
  #1 a = 32'h3f800000;
  #1 a = 32'h3f000000;
  #1 a = 32'h00000001;
  #1 a = 32'h7f800000;
  #1 a = 32'h3fc00000;
  #1 a = 32'hcf000000;
  #1 a = 32'hcf000001;
  #1 a = 32'hbf800000;
  #1 a = 32'hbf7fffff;
  #1 a = 32'h80000001;
  #1 a = 32'hff800000;
  #1 a = 32'h00000000;
  #1 $finish;
end
initial begin
  $dumpfile("float_to_integer.vcd");
  $dumpvars;
end

endmodule
