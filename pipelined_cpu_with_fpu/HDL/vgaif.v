module vgaif(
  input        clk,
  input        clrn,
  output[31:0] vramaddr,
  input[31:0]  vramdata,

  // VGA output
  output [3:0] VGA_R,
  output [3:0] VGA_G,
  output [3:0] VGA_B,
  output       VGA_HS, VGA_VS
);

`include "vga_param.vh"

wire [9:0] HCNT;
wire [9:0] VCNT;
wire       PCK;

syncgen syncgen(
  .CLK   (clk),
  .clrn  (clrn),
  .PCK   (PCK),
  .VGA_HS(VGA_HS),
  .VGA_VS(VGA_VS),
  .HCNT  (HCNT),
  .VCNT  (VCNT)
);

wire [9:0] iHCNT = HCNT - HFRONT - HWIDTH - HBACK + 10'd8;
wire [9:0] iVCNT = VCNT - VFRONT - VWIDTH - VBACK - 10'd40;

wire [2:0] vdotcnt;
wire [7:0] cgout;

CGROM CGROM(
  .address({vramdata[6:0], vdotcnt}),
  .q      (cgout),
  .clock  (PCK)
);

wire [6:0] hchacnt = iHCNT[9:3];
wire [2:0] hdotcnt = iHCNT[2:0];
wire [5:0] vchacnt = iVCNT[8:3];
assign     vdotcnt = iVCNT[2:0];

assign vramaddr = (vchacnt<<6) + (vchacnt<<4) + hchacnt;

reg [7:0] sreg;
wire sregld = (hdotcnt==3'h6 && iHCNT<10'd640);

always @(posedge PCK or negedge clrn) begin
  if(!clrn)
    sreg <= 8'h00;
  else if(sregld)
    sreg <= cgout;
  else
    sreg <= {sreg[6:0], 1'b0};
end

reg [11:0] color;

always @(posedge PCK or negedge clrn) begin
  if(!clrn)
    color <= 12'h000;
  else if(sregld)
    color <= vramdata[27:16];
end

wire hdispen = (10'd7<=iHCNT && iHCNT<10'd647);
wire vdispen = (iVCNT<9'd400);

reg [11:0] vga_rgb;
always @(posedge PCK or negedge clrn) begin
  if(!clrn)
    vga_rgb <= 12'h000;
  else
    vga_rgb <= color & {12{hdispen & vdispen & sreg[7]}};
end

assign VGA_R = vga_rgb[11:8];
assign VGA_G = vga_rgb[ 7:4];
assign VGA_B = vga_rgb[ 3:0];

endmodule

