module pipelined_float_adder(
  input[31:0]  a,
  input[31:0]  b,
  input        sub,
  input[1:0]   rm,
  output[31:0] s,
  input        clk,
  input        clrn,
  input        e
);

wire[26:0] a_small_frac;
wire[23:0] a_large_frac;
wire[22:0] a_inf_nan_frac;
wire[7:0]  a_exp;
wire       a_is_nan, a_is_inf, a_sign, a_op_sub;

float_adder_align_stage align_stage(
  a, b, sub, a_is_nan, a_is_inf, a_inf_nan_frac, a_sign,
  a_exp, a_op_sub, a_large_frac, a_small_frac);

wire[1:0]  c_rm;
wire[26:0] c_small_frac;
wire[27:0] c_frac;
wire[23:0] c_large_frac;
wire[22:0] c_inf_nan_frac;
wire[7:0]  c_exp;
wire c_is_nan, c_is_inf, c_sign, c_op_sub;

float_adder_align_cal_reg align_cal_reg(
  rm, a_is_nan, a_is_inf, a_inf_nan_frac,
  a_sign, a_exp, a_op_sub, a_large_frac, a_small_frac,
  clk, clrn, e,
  c_rm, c_is_nan, c_is_inf, c_inf_nan_frac,
  c_sign, c_exp, c_op_sub, c_large_frac, c_small_frac);

float_adder_cal_stage cal_stage(
  c_op_sub, c_large_frac, c_small_frac, c_frac);

wire[1:0] n_rm;
wire[22:0] n_inf_nan_frac;
wire[7:0] n_exp;
wire[27:0] n_frac;
wire n_is_nan, n_is_inf, n_sign;

float_adder_cal_norm_reg cal_norm_reg(
  c_rm, c_is_nan, c_is_inf, c_inf_nan_frac, c_sign, c_exp, c_frac,
  clk, clrn, e,
  n_rm, n_is_nan, n_is_inf, n_inf_nan_frac, n_sign, n_exp, n_frac);

float_adder_norm_stage norm_stage(
  n_rm, n_is_nan, n_is_inf, n_inf_nan_frac, n_sign, n_exp, n_frac, s);

endmodule

module float_adder_align_stage(
  input[31:0]  a,
  input[31:0]  b,
  input        sub,
  output       a_is_nan,
  output       a_is_inf,
  output[22:0] a_inf_nan_frac,
  output       a_sign,
  output[7:0]  a_exp,
  output       a_op_sub,
  output[23:0] a_large_frac,
  output[26:0] a_small_frac
);
wire exchange = (b[30:0] > a[30:0]);
wire[31:0] fp_large = exchange ? b : a;
wire[31:0] fp_small = exchange ? a : b;
wire       fp_large_hidden_bit = |fp_large[30:23];
wire       fp_small_hidden_bit = |fp_small[30:23];
wire[23:0] large_frac24 = {fp_large_hidden_bit, fp_large[22:0]};
wire[23:0] small_frac24 = {fp_small_hidden_bit, fp_small[22:0]};
assign a_exp = fp_large[30:23];
assign a_sign = exchange ? sub ^ b[31] : a[31];
assign a_op_sub = sub ^ fp_large[31] ^ fp_small[31];
wire fp_large_expo_is_ff = &fp_large[30:23]; // exp == 0xff
wire fp_small_expo_is_ff = &fp_small[30:23];
wire fp_large_frac_is_00 = ~|fp_large[22:0]; // frac == 0x0
wire fp_small_frac_is_00 = ~|fp_small[22:0];
wire fp_large_is_inf = fp_large_expo_is_ff & fp_large_frac_is_00;
wire fp_small_is_inf = fp_small_expo_is_ff & fp_small_frac_is_00;
wire fp_large_is_nan = fp_large_expo_is_ff & ~fp_large_frac_is_00;
wire fp_small_is_nan = fp_small_expo_is_ff & ~fp_small_frac_is_00;
assign a_is_inf = fp_large_is_inf | fp_small_is_inf;
assign a_is_nan = fp_large_is_nan | fp_small_is_nan |
                  ((sub ^ fp_small[31] ^ fp_large[31]) & fp_large_is_inf & fp_small_is_inf);
wire[22:0] nan_frac = (a[21:0] > b[21:0]) ? {1'b1, a[21:0]} : {1'b1, b[21:0]};
assign a_inf_nan_frac = a_is_nan ? nan_frac : 23'h0;
wire[7:0] exp_diff = fp_large[30:23] - fp_small[30:23];
wire      small_den_only = (fp_large[30:23] != 0) & (fp_small[30:23] == 0);
wire[7:0] shift_amount = small_den_only ? exp_diff - 8'h1 : exp_diff;
wire[49:0] small_frac50 = (shift_amount >= 26) ? {26'h0, small_frac24} : {small_frac24, 26'h0} >> shift_amount;
assign a_large_frac = large_frac24;
assign a_small_frac = {small_frac50[49:24], |small_frac50[23:0]};
endmodule

module float_adder_align_cal_reg(
  input[1:0]  rm,
  input       a_is_nan,
  input       a_is_inf,
  input[22:0] a_inf_nan_frac,
  input       a_sign,
  input[7:0]  a_exp,
  input       a_op_sub,
  input[23:0] a_large_frac,
  input[26:0] a_small_frac,
  input       clk,
  input       clrn,
  input       e,
  output reg[1:0]  c_rm,
  output reg       c_is_nan,
  output reg       c_is_inf,
  output reg[22:0] c_inf_nan_frac,
  output reg       c_sign,
  output reg[7:0]  c_exp,
  output reg       c_op_sub,
  output reg[23:0] c_large_frac,
  output reg[26:0] c_small_frac
);
always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    c_rm <= 0;
    c_is_nan <= 0;
    c_is_inf <= 0;
    c_inf_nan_frac <= 0;
    c_sign <= 0;
    c_exp <= 0;
    c_op_sub <= 0;
    c_large_frac <= 0;
    c_small_frac <= 0;
  end else if(e) begin
    c_rm <= rm;
    c_is_nan <= a_is_nan;
    c_is_inf <= a_is_inf;
    c_inf_nan_frac <= a_inf_nan_frac;
    c_sign <= a_sign;
    c_exp <= a_exp;
    c_op_sub <= a_op_sub;
    c_large_frac <= a_large_frac;
    c_small_frac <= a_small_frac;
  end
end
endmodule

module float_adder_cal_stage(
  input        op_sub,
  input [23:0] large_frac,
  input [26:0] small_frac,
  output[27:0] cal_frac
);
wire[27:0] aligned_large_frac = {1'b0, large_frac, 3'b000};
wire[27:0] aligned_small_frac = {1'b0, small_frac};
assign cal_frac = op_sub ?
                  aligned_large_frac - aligned_small_frac :
                  aligned_large_frac + aligned_small_frac;
endmodule

module float_adder_cal_norm_reg(
  input[1:0]  c_rm,
  input       c_is_nan,
  input       c_is_inf,
  input[22:0] c_inf_nan_frac,
  input       c_sign,
  input[7:0]  c_exp,
  input[27:0] c_frac,
  input       clk,
  input       clrn,
  input       e,
  output reg[1:0]  n_rm,
  output reg       n_is_nan,
  output reg       n_is_inf,
  output reg[22:0] n_inf_nan_frac,
  output reg       n_sign,
  output reg[7:0]  n_exp,
  output reg[27:0] n_frac
);
always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    n_rm <= 0;
    n_is_nan <= 0;
    n_is_inf <= 0;
    n_inf_nan_frac <= 0;
    n_sign <= 0;
    n_exp <= 0;
    n_frac <= 0;
  end else if(e) begin
    n_rm <= c_rm;
    n_is_nan <= c_is_nan;
    n_is_inf <= c_is_inf;
    n_inf_nan_frac <= c_inf_nan_frac;
    n_sign <= c_sign;
    n_exp <= c_exp;
    n_frac <= c_frac;
  end
end
endmodule

module float_adder_norm_stage(
  input[1:0]  n_rm,
  input       n_is_nan,
  input       n_is_inf,
  input[22:0] n_inf_nan_frac,
  input       n_sign,
  input[7:0]  n_exp,
  input[27:0] n_frac,
  output[31:0] s
);
wire[26:0] f4, f3, f2, f1, f0;
wire[4:0] zeros;
assign zeros[4] = ~|n_frac[26:11]; // 16 bit shift
assign f4 = zeros[4] ? {n_frac[10:0], 16'b0} : n_frac[26:0];
assign zeros[3] = ~|f4[26:19];     // 8 bit shift
assign f3 = zeros[3] ? {f4[18:0], 8'b0} : f4;
assign zeros[2] = ~|f3[26:23];     // 4 bit shift
assign f2 = zeros[2] ? {f3[22:0], 4'b0} : f3;
assign zeros[1] = ~|f2[26:25];     // 2 bit shift
assign f1 = zeros[1] ? {f2[24:0], 2'b0} : f2;
assign zeros[0] = ~f1[26];         // 1 bit shift
assign f0 = zeros[0] ? {f1[25:0], 1'b0} : f1;
reg[26:0] frac0;
reg[7:0]  exp0;
always @* begin
  if(n_frac[27]) begin // 1x.xxxxxxxx
    frac0 = n_frac[27:0];
    exp0 = n_exp + 8'h1;
  end else begin
    if((n_exp > zeros) && (f0[26])) begin // 01.xxxxxx
      exp0 = n_exp - zeros;
      frac0 = f0;
    end else begin // denormalized or zero
      exp0 = 0;
      if(n_exp != 0)
        frac0 = n_frac[26:0] << (n_exp - 8'h1);
      else
        frac0 = n_frac[26:0];
    end
  end
end
wire frac_plus_1 = (n_rm == 2'b00 & frac0[3:0] == 4'b1100) |
                   (n_rm == 2'b00 & frac0[2] & (frac0[1] | frac0[0])) |
                   (n_rm == 2'b01 & (|frac0[2:0]) & n_sign) |
                   (n_rm == 2'b10 & (|frac0[2:0]) & ~n_sign);
wire[24:0] frac_round = {1'b0, frac0[26:3]} + frac_plus_1;
wire[7:0]  exponent = frac_round[24] ? exp0 + 8'h1 : exp0;
wire overflow = &exp0 | &exponent;
assign s = final_result(overflow, n_rm, n_sign, n_is_nan, n_is_inf, exponent, frac_round[22:0], n_inf_nan_frac);
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

