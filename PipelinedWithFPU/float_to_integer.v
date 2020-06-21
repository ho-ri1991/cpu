module float_to_integer(
  input[31:0]      a,
  output reg[31:0] d,
  output reg       p_lost,
  output           denorm,
  output reg       invalid
);

wire hidden_bit = |a[30:23]; // 0.XXX if exponent is zero otherwire, 1.XXX
wire frac_is_not_0 = |a[22:0];
assign denorm = ~hidden_bit & frac_is_not_0;
wire is_zero = ~hidden_bit & ~frac_is_not_0;
wire sign = a[31];
wire[8:0] shift_right_bits = 9'd158 - {1'b0, a[30:23]};
wire[55:0] frac0 = {hidden_bit, a[22:0], 32'h0};
wire[55:0] f_abs = (shift_right_bits > 9'd32) ? frac0 >> 6'd32 : frac0 >> shift_right_bits;
wire lost_bits = |f_abs[23:0];
wire[31:0] int32 = sign ? ~f_abs[55:24] + 32'd1 : f_abs[55:24];
always @* begin
  if(denorm) begin
    p_lost = 1;
    invalid = 0;
    d = 32'h0;
  end else begin
    if(shift_right_bits[8]) begin // too big
      p_lost = 0;
      invalid = 1;
      d = 32'h80000000;
    end else begin
      if(shift_right_bits[7:0] > 8'h1f) begin // too small
        if(is_zero)
          p_lost = 0;
        else
          p_lost = 1;
        invalid = 0;
        d = 32'h0;
      end else begin
        if(sign != int32[31]) begin
          p_lost = 0;
          invalid = 1;
          d = 32'h80000000;
        end else begin
          if(lost_bits)
            p_lost = 1;
          else
            p_lost = 0;
          invalid = 0;
          d = int32;
        end
      end
    end
  end
end

endmodule
