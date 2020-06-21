module float_sqrt_newton(
  input[31:0]  d,
  input[1:0]   rm,
  input        fsqrt,
  input        ena,
  input        clk,
  input        clrn,
  output[31:0] s,
  output       busy,
  output       stall,
  output[4:0]  count,
  output[25:0] reg_x
);

parameter ZERO = 32'h00000000;
parameter INF  = 32'h7f800000;
parameter NaN  = 32'h7fc00000;
wire d_expo_is_00 = ~|d[30:23];
wire d_expo_is_ff =  &d[30:23];
wire d_frac_is_00 = ~|d[22:00];
wire sign = d[31];
//                 e_d >> 1        +    63 + e_d%2
wire[7:0] exp_8 = {1'b0, d[30:24]} + 8'd63 + d[23];
wire[23:0] d_f24 = d_expo_is_00 ? {d[22:0], 1'b0} : {1'b1, d[22:0]};
wire[23:0] d_temp24 = d[23] ? {1'b0, d_f24[23:1]} : d_f24;
wire[23:0] d_frac24;
wire[4:0] shift_amount;
shift_even_bits shift_d(d_temp24, d_frac24, shift_amount);
wire[7:0] exp0 = exp_8 - {4'h0, shift_amount[4:1]};

reg e1_sign, e1_e00, e1_eff, e1_f00;
reg e2_sign, e2_e00, e2_eff, e2_f00;
reg e3_sign, e3_e00, e3_eff, e3_f00;
reg[1:0] e1_rm, e2_rm, e3_rm;
reg[7:0] e1_exp, e2_exp, e3_exp;
always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    e1_sign <= 0; e1_e00 <= 0; e1_eff <= 0; e1_f00 <= 0; e1_rm <= 0; e1_exp <= 0;
    e2_sign <= 0; e2_e00 <= 0; e2_eff <= 0; e2_f00 <= 0; e2_rm <= 0; e2_exp <= 0;
    e3_sign <= 0; e3_e00 <= 0; e3_eff <= 0; e3_f00 <= 0; e3_rm <= 0; e3_exp <= 0;
  end else if(ena) begin
    e1_sign <= sign; e1_e00 <= d_expo_is_00; e1_eff <= d_expo_is_ff; e1_f00 <= d_frac_is_00; e1_rm <= rm; e1_exp <= exp0;
    e2_sign <= e1_sign; e2_e00 <= e1_e00; e2_eff <= e1_eff; e2_f00 <= e1_f00; e2_rm <= e1_rm; e2_exp <= e1_exp;
    e3_sign <= e2_sign; e3_e00 <= e2_e00; e3_eff <= e2_eff; e3_f00 <= e2_f00; e3_rm <= e2_rm; e3_exp <= e2_exp;
  end
end

wire[31:0] frac0;
sqrt_newton24 frac_newton(d_frac24, fsqrt, ena, clk, clrn, frac0, busy, count, reg_x, stall);
wire[26:0] frac = {frac0[31:6], |frac0[5:0]};
wire frac_plus_1 = (e3_rm == 2'b00 & frac[3:0] == 4'b1100) |
                   (e3_rm == 2'b00 & frac[2] & (frac[1] | frac[0])) |
                   (e3_rm == 2'b01 & (|frac[2:0]) &  e3_sign) |
                   (e3_rm == 2'b10 & (|frac[2:0]) & ~e3_sign);
wire[24:0] frac_round = {1'b0, frac[26:3]} + frac_plus_1;
wire[7:0] expo_new = frac_round[24] ? e3_exp + 8'h1 : e3_exp;
wire[22:0] frac_new = frac_round[24] ? frac_round[23:1] : frac_round[22:0];
assign s = final_result(e3_sign,e3_e00,e3_eff,e3_f00,
                       {e3_sign,expo_new,frac_new});
function  [31:0] final_result;
  input        d_sign,d_e00,d_eff,d_f00;
  input [31:0] calc;
  casex ({d_sign,d_e00,d_eff,d_f00})
    4'b1xxx : final_result = NaN;       // -
    4'b000x : final_result = calc;      // nor
    4'b0100 : final_result = calc;      // den
    4'b0010 : final_result = NaN;       // nan
    4'b0011 : final_result = INF;       // inf
    default : final_result = ZERO;      // 0
  endcase
endfunction

endmodule

module shift_even_bits(
  input[23:0]  a,
  output[23:0] b,
  output[4:0]  shift_amount
);
wire[23:0] a5, a4, a3, a2, a1;
assign a5 = a;
assign shift_amount[4] = ~|a5[23:8];
assign a4 = shift_amount[4] ? {a5[07:00], 16'b0} : a5;
assign shift_amount[3] = ~|a4[23:16];
assign a3 = shift_amount[3] ? {a4[15:00],  8'b0} : a4;
assign shift_amount[2] = ~|a3[23:20];
assign a2 = shift_amount[2] ? {a3[19:00],  4'b0} : a3;
assign shift_amount[1] = ~|a2[23:22];
assign a1 = shift_amount[1] ? {a2[21:00],  2'b0} : a2;
assign shift_amount[0] = 0;
assign b = a1;
endmodule

module sqrt_newton24(
  input[23:0]  d,
  input        fsqrt,
  input        ena,
  input        clk,
  input        clrn,
  output reg[31:0] q,
  output reg       busy,
  output reg[4:0]  count,
  output reg[25:0] reg_x,
  output           stall
);
reg[23:0] reg_d;
wire[7:0] x0 = rom(d[23:19]);
wire[51:0] x_2, x2d, x52;
always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    q <= 0;
    busy <= 0;
    count <= 0;
    reg_x <= 0;
    reg_d <= 0;
  end else begin
    if(fsqrt & (count == 0)) begin
      count <= 5'b1;
      busy <= 1'b1;
    end else begin
      if(count == 5'h01) begin
        reg_x <= {2'b1, x0, 16'b0};
        reg_d <= d;
      end
      if(count != 0)
        count <= count + 5'b1;
      if(count == 5'h15)
        busy <= 0;
      if(count == 5'h16)
        count <= 0;
      if((count == 5'h08) ||
         (count == 5'h0f) ||
         (count == 5'h16))
         reg_x <= x52[50:25];
    end
  end
end
assign stall = fsqrt & (count == 0) | busy;
assign x_2 = {26'b0, reg_x} * {26'b0, reg_x};
assign x2d = {28'b0, reg_d} * {24'b0, x_2[51:24]};
wire[25:0] b26 = 26'h3000000 - x2d[49:24];
assign x52 = {26'b0, reg_x} * {26'b0, b26};
reg[25:0] reg_de_x;
reg[23:0] reg_de_d;
wire[49:0] m_s;
wire[49:8] m_c;
wallace_24x26 wt(reg_de_d, reg_de_x, m_s[49:8], m_c, m_s[7:0]);
reg[49:0] a_s;
reg[49:8] a_c;
wire[49:0] d_x = {1'b0, a_s} + {a_c, 8'b0};
wire[31:0] e2p = {d_x[47:17], |d_x[16:0]};
always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    reg_de_x <= 0;
    reg_de_d <=0;
    a_s <= 0;
    a_c <= 0;
    q <= 0;
  end else if(ena) begin
    reg_de_x <= x52[50:25];
    reg_de_d <= reg_d;
    a_s <= m_s;
    a_c <= m_c;
    q <= e2p;
  end
end
function [7:0] rom;                                   // about 1/d^{1/2}
  input [4:0] d;
  case (d)
    5'h08: rom = 8'hff;            5'h09: rom = 8'he1;
    5'h0a: rom = 8'hc7;            5'h0b: rom = 8'hb1;
    5'h0c: rom = 8'h9e;            5'h0d: rom = 8'h9e;
    5'h0e: rom = 8'h7f;            5'h0f: rom = 8'h72;
    5'h10: rom = 8'h66;            5'h11: rom = 8'h5b;
    5'h12: rom = 8'h51;            5'h13: rom = 8'h48;
    5'h14: rom = 8'h3f;            5'h15: rom = 8'h37;
    5'h16: rom = 8'h30;            5'h17: rom = 8'h29;
    5'h18: rom = 8'h23;            5'h19: rom = 8'h1d;
    5'h1a: rom = 8'h17;            5'h1b: rom = 8'h12;
    5'h1c: rom = 8'h0d;            5'h1d: rom = 8'h08;
    5'h1e: rom = 8'h04;            5'h1f: rom = 8'h00;
    default: rom = 8'hff;                         // 0 - 7: not used
  endcase
endfunction

endmodule
