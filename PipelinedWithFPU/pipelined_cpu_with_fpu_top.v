module pipelined_cpu_with_fpu_top(
  input CLK, RST,

  input [8:0] SW,
  input [3:0] KEY,
  output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5,

  output [9:0] LEDR,
  inout PS2_CLK, PS2_DAT,

  output [3:0] VGA_R, VGA_G, VGA_B,
  output       VGA_HS, VGA_VS,

  output        DRAM_CLK, DRAM_CKE,
  output [12:0] DRAM_ADDR,
  output [1:0]  DRAM_BA,
  output        DRAM_CAS_N, DRAM_RAS_N,
  output        DRAM_CS_N,  DRAM_WE_N,
  output        DRAM_UDQM,  DRAM_LDQM,
  input  [15:0] DRAM_DQ,

  inout [35:0] GPIO_0
);

assign HEX0=7'h7f, HEX1=7'h7f, HEX2=7'h7f, HEX3=7'h7f, HEX4=7'h7f, HEX5=7'h7f;
wire[31:0] a = 32'd3248576;
wire[31:0] b = 32'd2038;
wire start = 1'b1;
wire[31:0] q;
wire busy, ready;
wire[2:0] count;

endmodule
