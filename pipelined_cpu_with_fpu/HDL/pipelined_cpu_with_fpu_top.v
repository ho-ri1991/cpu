module pipelined_cpu_with_fpu_top(
  input CLK, RST,

  input [8:0] SW,
  input [3:0] KEY,
  output reg[6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5,

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

//assign HEX0=7'h7f, HEX1=7'h7f, HEX2=7'h7f, HEX3=7'h7f, HEX4=7'h7f, HEX5=7'h7f;

reg cpuclk;
wire memclk = ~CLK;
always @(posedge CLK) begin
  if(RST) begin
    cpuclk <= 0;
  end
  else begin
    if(CLK)
      cpuclk <= ~cpuclk;
  end
end
wire[31:0] w1, w2, w3, w4, w5, w6, w7;
wire[4:0] x1, x2, x3, x4, x5, x6;
wire y1, y2, y3, y4, y5, y6, y7, y8, y9;
wire[31:0] vramaddr, vramin;
wire vramwe;
pipelined_cpu_with_fpu cpu(
  .clk(cpuclk), .memclk(~CLK), .clrn(~RST),
  .pc(w1), .inst(w2), .ealu(w3), .malu(w4), .walu(w5), .wn(x1), .wd(w6),
  .ww(y1), .stall_lw(y2), .stall_fp(y3), .stall_lwc1(y4), .stall_swc1(y5),
  .stall_mfc1(y6), .stall_mtc1(y7), .stall(y8),
  .count_fdiv(x2), .count_fsqrt(x3), .e1n(x4), .e2n(x5), .e3n(x6), .e3d(w7),
  .e(y9), .vramaddr(vramaddr), .vramwe(vramwe), .vramdata(vramin));

wire[31:0] vgavramaddr, vramout;
vgaif vga(
  .clk(CLK), .clrn(~RST), .vramaddr(vgavramaddr), .vramdata(vramout),
  .VGA_R(VGA_R), .VGA_G(VGA_G), .VGA_B(VGA_B), .VGA_HS(VGA_HS), .VGA_VS(VGA_VS));

VRAM VRAM(
  .clock ( CLK ),
  .data ( vramin ),
  .rdaddress ( vgavramaddr[13:2] ),
  .wraddress ( vramaddr ),
  .wren ( vramwe ),
  .q ( vramout )
);

reg [25:0] cnt;
wire en1hz = (cnt==26'd49_999_999);

always @(posedge CLK) begin
  if(RST)
    cnt <= 26'b0;
  else if(en1hz)
    cnt <= 26'b0;
  else
    cnt <= cnt + 26'b1;
end

reg [3:0] sec;

always @(posedge CLK) begin
  if(RST)
    sec <= 4'h0;
  else if(en1hz)
    if(sec==4'h9)
      sec <= 4'h0;
    else
      sec <= sec + 4'h1;
end

always @* begin
  case(sec)
    4'h0: HEX0 = 7'b1000000;
    4'h1: HEX0 = 7'b1111001;
    4'h2: HEX0 = 7'b0100100;
    4'h3: HEX0 = 7'b0110000;
    4'h4: HEX0 = 7'b0011001;
    4'h5: HEX0 = 7'b0010010;
    4'h6: HEX0 = 7'b0000010;
    4'h7: HEX0 = 7'b1011000;
    4'h8: HEX0 = 7'b0000000;
    4'h9: HEX0 = 7'b0010000;
    default: HEX0 = 7'bxxxxxxx;
  endcase
end

endmodule
