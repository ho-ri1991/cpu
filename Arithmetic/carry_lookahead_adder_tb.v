`timescale 1ns/1ns
module cla32_tb;

reg[31:0]  a, b;
reg        c;
wire[31:0] s;
cla32 cla32(a, b, c, s);
initial begin
     a = 32'd1;
     b = 32'd1;
     c = 0;
  #1 a = 32'd294385;
     b = 32'd4085;
  #1 a = 32'd193857678;
     b = 32'd24895709;
  #1 a = 32'd95876;
     b = 32'd897045;
  #1 $finish;
end
initial begin
  $dumpfile("cla32.vcd");
  $dumpvars;
end

endmodule

