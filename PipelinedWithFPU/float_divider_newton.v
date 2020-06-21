module float_divider_newton(
  input[31:0]  a,
  input[31:0]  b,
  input[1:0]   rm,
  input        fdiv,
  input        ena, // enable
  input        clk,
  input        clrn,
  output[31:0] s,
  output       busy,
  output       stall,
  output[4:0]  count,
  output[25:0] reg_x
);

parameter ZERO = 31'h00000000;
parameter INF  = 31'h7f800000;
parameter NaN  = 31'h7fc00000;
parameter MAX  = 31'h7f7fffff;
wire a_expo_is_00 = ~|a[30:23];
wire b_expo_is_00 = ~|b[30:23];
wire a_expo_is_ff =  &a[30:23];
wire b_expo_is_ff =  &b[30:23];
wire a_frac_is_00 = ~|a[22:0];
wire b_frac_is_00 = ~|b[22:0];
wire sign = a[31] ^ b[31];
wire[9:0] exp_10 = {2'h0, a[30:23]} - {2'h0, b[30:23]} + 10'h7f;
wire[23:0] a_temp24 = a_expo_is_00 ? {a[22:0], 1'b0} : {1'b1, a[22:0]};
wire[23:0] b_temp24 = b_expo_is_00 ? {b[22:0], 1'b0} : {1'b1, b[22:0]};
wire[23:0] a_frac24, b_frac24;
wire[4:0] shift_amount_a, shift_amount_b;
shift_to_msb_equ_1 shift_a(a_temp24, a_frac24, shift_amount_a);
shift_to_msb_equ_1 shift_b(b_temp24, b_frac24, shift_amount_b);
wire[9:0] exp10 = exp_10 - shift_amount_a + shift_amount_b;

reg e1_sign, e1_ae00, e1_aeff, e1_af00, e1_be00, e1_beff, e1_bf00;
reg e2_sign, e2_ae00, e2_aeff, e2_af00, e2_be00, e2_beff, e2_bf00;
reg e3_sign, e3_ae00, e3_aeff, e3_af00, e3_be00, e3_beff, e3_bf00;
reg[1:0] e1_rm, e2_rm, e3_rm;
reg[9:0] e1_exp10, e2_exp10, e3_exp10;
always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    // reg_e1
    e1_sign <= 0; e1_rm <= 0; e1_exp10 <= 0; e1_ae00 <= 0; e1_aeff <= 0; e1_af00 <= 0; e1_be00 <= 0; e1_beff <= 0; e1_bf00 <= 0;
    // reg_e2
    e2_sign <= 0; e2_rm <= 0; e2_exp10 <= 0; e2_ae00 <= 0; e2_aeff <= 0; e2_af00 <= 0; e2_be00 <= 0; e2_beff <= 0; e2_bf00 <= 0;
    // reg_e3
    e3_sign <= 0; e3_rm <= 0; e3_exp10 <= 0; e3_ae00 <= 0; e3_aeff <= 0; e3_af00 <= 0; e3_be00 <= 0; e3_beff <= 0; e3_bf00 <= 0;
  end else if(ena) begin
    e1_sign <= sign; e1_rm <= rm; e1_exp10 <= exp10; e1_ae00 <= a_expo_is_00; e1_aeff <= a_expo_is_ff; e1_af00 <= a_frac_is_00; e1_be00 <= b_expo_is_00; e1_beff <= b_expo_is_ff; e1_bf00 <= b_frac_is_00;
    e2_sign <= e1_sign; e2_rm <= e1_rm; e2_exp10 <= e1_exp10; e2_ae00 <= e1_ae00; e2_aeff <= e1_aeff; e2_af00 <= e1_af00; e2_be00 <= e1_be00; e2_beff <= e1_beff; e2_bf00 <= e1_bf00;
    e3_sign <= e2_sign; e3_rm <= e2_rm; e3_exp10 <= e2_exp10; e3_ae00 <= e2_ae00; e3_aeff <= e2_aeff; e3_af00 <= e2_af00; e3_be00 <= e2_be00; e3_beff <= e2_beff; e3_bf00 <= e2_bf00;
  end
end

wire[31:0] q; //a_frac24 / b_frac24 = 1.xxxx or 0.1xxxx
newton_divider24 div(a_frac24, b_frac24, fdiv, ena, clk, clrn, q, busy, count, reg_x, stall);

// normalization stage
wire[31:0] z0 = q[31] ? q : {q[30:0], 1'b0};
wire[9:0] exp_adj = q[31] ? e3_exp10 : e3_exp10 - 10'b1;
reg[9:0] exp0;
reg[31:0] frac0;
always @* begin
  if(exp_adj[9]) begin
    exp0 = 0;
    if(z0[31]) // denormalized
      frac0 = z0 >> (10'b1 - exp_adj);
    else
      frac0 = 0;
  end else if(exp_adj == 0) begin
    exp0 = 0;
    frac0 = {1'b0, z0[31:2], |z0[1:0]}; // denormalized
  end else begin
    if(exp_adj > 254) begin // inf
      exp0 = 10'hff;
      frac0 = 0;
    end else begin
      exp0 = exp_adj;
      frac0 = z0;
    end
  end
end
wire[26:0] frac = {frac0[31:6], |frac0[5:0]};
wire frac_plus_1 = (e3_rm == 2'b00 & frac[3:0] == 4'b1100) |
                   (e3_rm == 2'b00 & frac[2] & (frac[1] | frac[0])) |
                   (e3_rm == 2'b01 & (|frac[2:0]) &  e3_sign) |
                   (e3_rm == 2'b10 & (|frac[2:0]) & ~e3_sign);
wire[24:0] frac_round = {1'b0, frac[26:3]} + frac_plus_1;
wire[7:0]  exp1 = frac_round[24] ? exp0 + 10'h1 : exp0;
wire overflow = (exp1 >= 10'h0ff);
wire[7:0] exponent;
wire[22:0] fraction;
assign {exponent, fraction} = final_result(
  overflow, e3_rm, e3_sign,
  e3_ae00, e3_aeff, e3_af00,
  e3_be00, e3_beff, e3_bf00, {exp1[7:0], frac_round[22:0]});
assign s = {e3_sign, exponent, fraction};
function [30:0] final_result;
  input overflow;
  input[1:0] e3_rm;
  input      e3_sign;
  input      a_e00, a_eff, a_f00, b_e00, b_eff, b_f00;
  input[30:0] calc;
  casex ({overflow,e3_rm,e3_sign,a_e00,a_eff,a_f00,b_e00,b_eff,b_f00})
    10'b100x_xxx_xxx : final_result = INF;    // overflow
    10'b1010_xxx_xxx : final_result = MAX;    // overflow
    10'b1011_xxx_xxx : final_result = INF;    // overflow
    10'b1100_xxx_xxx : final_result = INF;    // overflow
    10'b1101_xxx_xxx : final_result = MAX;    // overflow
    10'b111x_xxx_xxx : final_result = MAX;    // overflow
    10'b0xxx_010_xxx : final_result = NaN;    // NaN / any
    10'b0xxx_011_010 : final_result = NaN;    // inf / NaN
    10'b0xxx_100_010 : final_result = NaN;    // den / NaN
    10'b0xxx_101_010 : final_result = NaN;    //   0 / NaN
    10'b0xxx_00x_010 : final_result = NaN;    // nor / NaN
    10'b0xxx_011_011 : final_result = NaN;    // inf / inf
    10'b0xxx_100_011 : final_result = ZERO;   // den / inf
    10'b0xxx_101_011 : final_result = ZERO;   //   0 / inf
    10'b0xxx_00x_011 : final_result = ZERO;   // nor / inf
    10'b0xxx_011_101 : final_result = INF;    // inf / 0
    10'b0xxx_100_101 : final_result = INF;    // den / 0
    10'b0xxx_101_101 : final_result = NaN;    //   0 / 0
    10'b0xxx_00x_101 : final_result = INF;    // nor / 0
    10'b0xxx_011_100 : final_result = INF;    // inf / den
    10'b0xxx_100_100 : final_result = calc;   // den / den
    10'b0xxx_101_100 : final_result = ZERO;   //   0 / den
    10'b0xxx_00x_100 : final_result = calc;   // nor / den
    10'b0xxx_011_00x : final_result = INF;    // inf / nor
    10'b0xxx_100_00x : final_result = calc;   // den / nor
    10'b0xxx_101_00x : final_result = ZERO;   //   0 / nor
    10'b0xxx_00x_00x : final_result = calc;   // nor / nor
    default          : final_result = ZERO;
  endcase
endfunction

endmodule

module shift_to_msb_equ_1(
  input[23:0]  a,
  output[23:0] b,
  output[4:0]  shift_amount
);
wire[23:0] a5, a4, a3, a2, a1, a0;
assign a5 = a;
assign shift_amount[4] = ~|a5[23:08];
assign a4 = shift_amount[4] ? {a5[07:00], 16'b0} : a5;
assign shift_amount[3] = ~|a4[23:16];
assign a3 = shift_amount[3] ? {a4[15:00],  8'b0} : a4;
assign shift_amount[2] = ~|a3[23:20];
assign a2 = shift_amount[2] ? {a3[19:00],  4'b0} : a3;
assign shift_amount[1] = ~|a2[23:22];
assign a1 = shift_amount[1] ? {a2[21:00],  2'b0} : a2;
assign shift_amount[0] = ~a1[23];
assign a0 = shift_amount[0] ? {a1[22:00],  1'b0} : a1;
assign b = a0;
endmodule

module newton_divider24(
  input[23:0]  a, 
  input[23:0]  b,
  input        fdiv,
  input        ena,
  input        clk,
  input        clrn,
  output reg[31:0] q,
  output reg       busy,
  output reg[4:0]  count, // 3 iteration * 5 clock, however, this implementation does not do pipelining in each iteration (xi * (2 - xi * b))
  output reg[25:0] reg_x,
  output           stall
);
reg[23:0] reg_a, reg_b;
wire[49:0] bxi;
wire[51:0] x52;
wire[49:0] d_x;
wire[31:0] e2p;
wire[7:0]  x0 = rom(b[22:19]);
always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    busy <= 0;
    count <= 0;
    reg_x <= 0;
    reg_a = 0;
    reg_b <= 0;
  end else begin
    if(fdiv & (count == 0)) begin
      count <= 5'b1;
      busy <= 1'b1;
    end else begin
      if(count == 5'h01) begin
        reg_a <= a;
        reg_b <= b;
        reg_x <= {2'b1, x0, 16'b0};
      end
      if(count != 0)
        count <= count + 5'b1;
      if(count == 5'h0f)
        busy <= 0;
      if(count == 5'h10)
        count <= 5'b0;
      if((count == 5'h06) || // 1st iteration
         (count == 5'h0b) || // 2nd iteration
         (count == 5'h10))   // 3rd iteration
         reg_x <= x52[50:25];
    end
  end
end
assign stall = fdiv & (count == 0) | busy;
assign bxi = {26'h0, reg_b} * {24'h0, reg_x}; // xi * b
wire[25:0] b26 = ~bxi[48:23] + 1'b1; // 2 - xi * b
assign x52 = {26'h0, reg_x} * {26'h0, b26}; // xi * (2 - xi * b)
reg[25:0] reg_de_x;
reg[23:0] reg_de_a;
wire[49:0] m_s; // sum
wire[49:8] m_c; // carry
wallace_24x26 wt(reg_de_a, reg_de_x, m_s[49:8], m_c, m_s[7:0]);
reg[49:0] a_s;
reg[49:8] a_c;
assign d_x = {1'b0, a_s} + {a_c, 8'b0};
assign e2p = {d_x[48:18], |d_x[17:0]};
always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    reg_de_x <= 0;
    reg_de_a <= 0;
    a_s <= 0;
    a_c <= 0;
    q <= 0;
  end else if(ena) begin
    reg_de_x <= x52[50:25];
    reg_de_a <= reg_a;
    a_s <= m_s;
    a_c <= m_c;
    q <= e2p;
  end
end
function  [7:0] rom;                                      // a rom table
  input [3:0] b;
  case (b)
    4'h0: rom = 8'hff;            4'h1: rom = 8'hdf;
    4'h2: rom = 8'hc3;            4'h3: rom = 8'haa;
    4'h4: rom = 8'h93;            4'h5: rom = 8'h7f;
    4'h6: rom = 8'h6d;            4'h7: rom = 8'h5c;
    4'h8: rom = 8'h4d;            4'h9: rom = 8'h3f;
    4'ha: rom = 8'h33;            4'hb: rom = 8'h27;
    4'hc: rom = 8'h1c;            4'hd: rom = 8'h12;
    4'he: rom = 8'h08;            4'hf: rom = 8'h00;
  endcase
endfunction

endmodule
