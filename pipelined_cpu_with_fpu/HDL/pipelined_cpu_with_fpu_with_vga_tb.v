`timescale 1ns/1ns
module pipelined_cpu_with_fpu_with_vga_tb;

reg clk, memclk, clrn, sysclock;
wire[31:0] pc, inst, ealu, malu, walu, wd, e3d, vramaddr, vramin;
wire[4:0] wn, count_fdiv, count_fsqrt, e1n, e2n, e3n;
wire ww, stall_lw, stall_fp, stall_lwc1, stall_swc1, stall_mfc1, stall_mtc1, stall, e, vramwe;
pipelined_cpu_with_fpu plcpufpu(
  clk, memclk, clrn, pc, inst, ealu, malu, walu,
  wn, wd, ww, stall_lw, stall_fp, stall_lwc1, stall_swc1, stall_mfc1, stall_mtc1, stall,
  count_fdiv, count_fsqrt, e1n, e2n, e3n, e3d, e, vramaddr, vramwe, vramin);

wire[3:0] VGA_R, VGA_G, VGA_B;
wire VGA_HS, VGA_VS;

wire[31:0] vgavramaddr, vramout;
vgaif vga(
  .clk(sysclock), .clrn(clrn), .vramaddr(vgavramaddr), .vramdata(vramout),
  .VGA_R(VGA_R), .VGA_G(VGA_G), .VGA_B(VGA_B), .VGA_HS(VGA_HS), .VGA_VS(VGA_VS));

VRAM VRAM(
  .clock ( sysclock ),
  .data ( vramin ),
  .rdaddress ( vgavramaddr ),
  .wraddress ( vramaddr[31:2] ),
  .wren ( vramwe ),
  .q ( vramout )
);


initial begin
  sysclock <= 1;
  clk <= 1;
  clrn <= 0;
  memclk <= 0;
  #1 clrn <= 1;
//  #335 $finish;
end
//initial begin
//  $dumpfile("pipelined_cpu_with_fpu.vcd");
//  $dumpvars;
//end
always #2 memclk <= !memclk;
always #2 sysclock <= !sysclock;
always #4 clk <= !clk;

endmodule
