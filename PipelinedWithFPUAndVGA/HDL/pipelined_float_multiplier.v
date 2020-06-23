module pipelined_float_multiplier(
  input[31:0]  a,
  input[31:0]  b,
  input[1:0]   rm,
  output[31:0] s,
  input        clk,
  input        clrn,
  input        e
);

wire m_sign, m_is_nan, m_is_inf;
wire[9:0] m_exp10;
wire[22:0] m_inf_nan_frac;
wire[39:0] m_sum, m_carry;
wire[7:0]  m_z8;
float_multiplier_mul_stage mul_stage(
  a, b, m_sign, m_exp10, m_is_nan, m_is_inf, m_inf_nan_frac, m_sum, m_carry, m_z8);

wire a_sign, a_is_nan, a_is_inf;
wire[1:0] a_rm;
wire[9:0] a_exp10;
wire[22:0] a_inf_nan_frac;
wire[39:0] a_sum, a_carry, a_z40;
wire[7:0]  a_z8;
float_multiplier_mul_add_reg mul_add_reg(
  rm, m_sign, m_exp10, m_is_nan, m_is_inf, m_inf_nan_frac, m_sum, m_carry, m_z8,
  clk, clrn, e,
  a_rm, a_sign, a_exp10, a_is_nan, a_is_inf, a_inf_nan_frac, a_sum, a_carry, a_z8);

float_multiplier_add_stage add_stage(a_sum, a_carry, a_z40);

wire n_sign, n_is_nan, n_is_inf;
wire[1:0] n_rm;
wire[9:0] n_exp10;
wire[22:0] n_inf_nan_frac;
wire[47:0] n_z48;
float_multiplier_add_norm_reg add_norm_reg(
  a_rm, a_sign, a_exp10, a_is_nan, a_is_inf, a_inf_nan_frac, {a_z40, a_z8},
  clk, clrn, e,
  n_rm, n_sign, n_exp10, n_is_nan, n_is_inf, n_inf_nan_frac, n_z48);

float_multiplier_norm_stage norm_stage(
  n_rm, n_sign, n_exp10, n_is_nan, n_is_inf, n_inf_nan_frac, n_z48, s);

endmodule

module float_multiplier_mul_stage(
  input[31:0]  a,
  input[31:0]  b,
  output       sign,
  output[9:0]  exp10,
  output       is_nan,
  output       is_inf,
  output[22:0] inf_nan_frac,
  output[39:0] z_sum,   // output of wallace tree
  output[39:0] z_carry, // output of wallace tree
  output[7:0]  z8       // output of wallace tree
);

wire a_expo_is_00 = ~|a[30:23];
wire b_expo_is_00 = ~|b[30:23];
wire a_expo_is_ff =  &a[30:23];
wire b_expo_is_ff =  &b[30:23];
wire a_frac_is_00 = ~|a[22:0];
wire b_frac_is_00 = ~|b[22:0];
wire a_is_inf = a_expo_is_ff &  a_frac_is_00;
wire b_is_inf = b_expo_is_ff &  b_frac_is_00;
wire a_is_nan = a_expo_is_ff & ~a_frac_is_00;
wire b_is_nan = b_expo_is_ff & ~b_frac_is_00;
wire a_is_0   = a_expo_is_00 &  a_frac_is_00;
wire b_is_0   = b_expo_is_00 &  b_frac_is_00;
assign is_inf = a_is_inf | b_is_inf;
assign is_nan = a_is_nan | (a_is_inf & b_is_0) |
                b_is_nan | (b_is_inf & a_is_0);
wire[22:0] nan_frac = (a[21:0] > b[21:0]) ? {1'b1, a[21:0]} : {1'b1, b[21:0]};
assign inf_nan_frac = is_nan ? nan_frac : 23'h0;
assign sign = a[31] ^ b[31];
assign exp10 = {2'h0, a[30:23]} + {2'h0, b[30:23]} - 10'h7f + a_expo_is_00 + b_expo_is_00;
wire[23:0] a_frac24 = {~a_expo_is_00, a[22:0]};
wire[23:0] b_frac24 = {~b_expo_is_00, b[22:0]};
wallace_24x24 wt24(a_frac24, b_frac24, z_sum, z_carry, z8);

endmodule

module float_multiplier_mul_add_reg(
  input[1:0]  m_rm,
  input       m_sign,
  input[9:0]  m_exp10,
  input       m_is_nan,
  input       m_is_inf,
  input[22:0] m_inf_nan_frac,
  input[39:0] m_sum,
  input[39:0] m_carry,
  input[7:0]  m_z8,
  input       clk,
  input       clrn,
  input       e,
  output reg[1:0]  a_rm,
  output reg       a_sign,
  output reg[9:0]  a_exp10,
  output reg       a_is_nan,
  output reg       a_is_inf,
  output reg[22:0] a_inf_nan_frac,
  output reg[39:0] a_sum,
  output reg[39:0] a_carry,
  output reg[7:0]  a_z8
);
always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    a_rm <= 0;
    a_sign <= 0;
    a_exp10 <= 0;
    a_is_nan <= 0;
    a_is_inf <= 0;
    a_inf_nan_frac <= 0;
    a_sum <= 0;
    a_carry <= 0;
    a_z8 <= 0;
  end else if(e) begin
    a_rm <= m_rm;
    a_sign <= m_sign;
    a_exp10 <= m_exp10;
    a_is_nan <= m_is_nan;
    a_is_inf <= m_is_inf;
    a_inf_nan_frac <= m_inf_nan_frac;
    a_sum <= m_sum;
    a_carry <= m_carry;
    a_z8 <= m_z8;
  end
end
endmodule

module float_multiplier_add_stage(
  input[39:0]  z_sum,
  input[39:0]  z_carry,
  output[47:8] z
);
assign z = z_sum + z_carry;
endmodule

module float_multiplier_add_norm_reg(
  input[1:0]  a_rm,
  input       a_sign,
  input[9:0]  a_exp10,
  input       a_is_nan,
  input       a_is_inf,
  input[22:0] a_inf_nan_frac,
  input[47:0] a_z48,
  input       clk,
  input       clrn,
  input       e,
  output reg[1:0]  n_rm,
  output reg       n_sign,
  output reg[9:0]  n_exp10,
  output reg       n_is_nan,
  output reg       n_is_inf,
  output reg[22:0] n_inf_nan_frac,
  output reg[47:0] n_z48
);
always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    n_rm <= 0;
    n_sign <= 0;
    n_exp10 <= 0;
    n_is_nan <= 0;
    n_is_inf <= 0;
    n_inf_nan_frac <= 0;
    n_z48 = 0;
  end else if(e) begin
    n_rm <= a_rm;
    n_sign <= a_sign;
    n_exp10 <= a_exp10;
    n_is_nan <= a_is_nan;
    n_is_inf <= a_is_inf;
    n_inf_nan_frac <= a_inf_nan_frac;
    n_z48 <= a_z48;
  end
end
endmodule

module float_multiplier_norm_stage(
  input[1:0]   rm,
  input        sign,
  input[9:0]   exp10,
  input        is_nan,
  input        is_inf,
  input[22:0]  inf_nan_frac,
  input[47:0]  z,
  output[31:0] s
);
wire[46:0] z5, z4, z3, z2, z1, z0;
wire[5:0]  zeros;
assign zeros[5] = ~|z[46:15];
assign z5 = zeros[5] ? {z[14:0], 32'b0} : z[46:0];
assign zeros[4] = ~|z5[46:31];
assign z4 = zeros[4] ? {z5[30:0], 16'b0} : z5;
assign zeros[3] = ~|z4[46:39];
assign z3 = zeros[3] ? {z4[38:0], 8'b0} : z4;
assign zeros[2] = ~|z3[46:43];
assign z2 = zeros[2] ? {z3[42:0], 4'b0} : z3;
assign zeros[1] = ~|z2[46:45];
assign z1 = zeros[1] ? {z2[44:0], 2'b0} : z2;
assign zeros[0] = ~z1[46];
assign z0 = zeros[0] ? {z1[45:0], 1'b0} : z1;
reg[46:0] frac0;
reg[9:0]  exp0;
always @* begin
  if(z[47]) begin // z = 1x.xxxxxx
    exp0 = exp10 + 10'h1;
    frac0 = z[47:1];
  end else begin  // z = 1.xxxxxx
    if(!exp10[9] & (exp10[8:0] > zeros) && z0[46]) begin
      exp0 = exp10 - zeros;
      frac0 = z[47:1];
    end else begin
      exp0 = 0;
      if(!exp10[9] && (exp10 != 0))
        frac0 = z[46:0] << (exp10 - 10'h1);
      else
        frac0 = z[46:0] >> (10'h1 - exp10);
    end
  end
end
wire[26:0] frac = {frac0[46:21], |frac0[20:0]};
wire frac_plus_1 = (rm == 2'b00 & frac0[3:0] == 4'b1100) |
                   (rm == 2'b00 & frac0[2] & (frac0[1] | frac0[0])) |
                   (rm == 2'b01 & (|frac0[2:0]) & sign) |
                   (rm == 2'b10 & (|frac0[2:0]) & ~sign);
wire[24:0] frac_round = {1'b0, frac[26:3]} + frac_plus_1;
wire[7:0]  exp1 = frac_round[24] ? exp0 + 10'h1 : exp0;
wire overflow = (exp0 >= 10'h0ff) | (exp1 >= 10'h0ff);
assign s = final_result(overflow, rm, sign, is_nan, is_inf, exp1[7:0], frac_round[22:0], inf_nan_frac);
function [31:0] final_result;
  input overflow;
  input[1:0] rm;
  input sign, is_nan, is_inf;
  input[7:0] exponent;
  input[22:0] fraction, inf_nan_frac;
  casex({overflow, rm, sign, is_nan, is_inf})
    6'b1_00_x_0_x : final_result = {sign,8'hff,23'h000000};   // inf
    6'b1_01_0_0_x : final_result = {sign,8'hfe,23'h7fffff};   // max
    6'b1_01_1_0_x : final_result = {sign,8'hff,23'h000000};   // inf
    6'b1_10_0_0_x : final_result = {sign,8'hff,23'h000000};   // inf
    6'b1_10_1_0_x : final_result = {sign,8'hfe,23'h7fffff};   // max
    6'b1_11_x_0_x : final_result = {sign,8'hfe,23'h7fffff};   // max
    6'b0_xx_x_0_0 : final_result = {sign,exponent,fraction};  // nor
    6'bx_xx_x_1_x : final_result = {1'b1,8'hff,inf_nan_frac}; // nan
    6'bx_xx_x_0_1 : final_result = {sign,8'hff,inf_nan_frac}; // inf
    default       : final_result = {sign,8'h00,23'h000000};   // 0
  endcase
endfunction
endmodule

