module integer_to_float(
  input[31:0]  d,
  output[31:0] a,
  output       p_lost
);

wire sign = d[31];
wire[31:0] f5 = sign ? -d : d;
wire[31:0] f4, f3, f2, f1, f0;
wire[4:0] sa;
assign sa[4] = ~|f5[31:16]; // 16-bit shift
assign f4 = sa[4] ? {f5[15:0], 16'b0} : f5;
assign sa[3] = ~|f4[31:24]; // 8-bit shift
assign f3 = sa[3] ? {f4[23:0], 8'b0} : f4;
assign sa[2] = ~|f3[31:28]; // 4-bit shift
assign f2 = sa[2] ? {f3[27:0], 4'b0} : f3;
assign sa[1] = ~|f2[31:30]; // 2-bit shift
assign f1 = sa[1] ? {f2[29:0], 2'b0} : f2;
assign sa[0] = ~f1[31];     // 1-bit shift
assign f0 = sa[0] ? {f1[30:0], 1'b0} : f1;
assign p_lost = |f0[7:0];
wire[22:0] fraction = f0[30:8];
wire[7:0] exponent = 8'h9e - {3'h0, sa};
assign a = (d == 0) ? 0 : {sign, exponent, fraction};

endmodule
