module goldschmidt_divider(
  input[31:0]  a, b,
  input        start, clk, clrn,
  output reg[31:0] q,
  output       busy, ready,
  output[2:0]  count
);
// TODO: handle a == 1, a == 0, b == 1, a < b

wire[4:0] msb_a, msb_b;
most_significant_bit most_significant_bit_a(a, msb_a);
most_significant_bit most_significant_bit_b(b, msb_b);

wire[31:0] a_in, b_in;
assign a_in = a << (5'd31 - msb_a);
assign b_in = b << (5'd31 - msb_b);
//assign a_in = a << ((5'd31 - msb_a) + 5'd1);
//assign b_in = b << ((5'd31 - msb_b) + 5'd1);

wire[31:0] q_out;
goldschmidt goldschmidt0(a_in, b_in, start, clk, clrn, q_out, busy, ready, count);

//assign q = q_out >> (5'd31 - (msb_a - msb_b));

always @* begin
  if(a == 0 || a < b)
    q <= 32'd0;
  else if(a == 1) begin
    if(b == 1)
      q <= 32'd1;
    else
      q <= 32'd0;
  end
  else begin
    if(a_in == b_in)
      q <= 1 << (msb_a - msb_b);
    else
      q <= q_out >> (5'd31 - (msb_a - msb_b));
  end
end

endmodule

module most_significant_bit(
  input[31:0]     a,
  output reg[4:0] b
);

always @* begin
  casex(a)
    32'b1xxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx: b = 5'd31;
    32'b01xx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx: b = 5'd30;
    32'b001x_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx: b = 5'd29;
    32'b0001_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx: b = 5'd28;
    32'b0000_1xxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx: b = 5'd27;
    32'b0000_01xx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx: b = 5'd26;
    32'b0000_001x_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx: b = 5'd25;
    32'b0000_0001_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx: b = 5'd24;
    32'b0000_0000_1xxx_xxxx_xxxx_xxxx_xxxx_xxxx: b = 5'd23;
    32'b0000_0000_01xx_xxxx_xxxx_xxxx_xxxx_xxxx: b = 5'd22;
    32'b0000_0000_001x_xxxx_xxxx_xxxx_xxxx_xxxx: b = 5'd21;
    32'b0000_0000_0001_xxxx_xxxx_xxxx_xxxx_xxxx: b = 5'd20;
    32'b0000_0000_0000_1xxx_xxxx_xxxx_xxxx_xxxx: b = 5'd19;
    32'b0000_0000_0000_01xx_xxxx_xxxx_xxxx_xxxx: b = 5'd18;
    32'b0000_0000_0000_001x_xxxx_xxxx_xxxx_xxxx: b = 5'd17;
    32'b0000_0000_0000_0001_xxxx_xxxx_xxxx_xxxx: b = 5'd16;
    32'b0000_0000_0000_0000_1xxx_xxxx_xxxx_xxxx: b = 5'd15;
    32'b0000_0000_0000_0000_01xx_xxxx_xxxx_xxxx: b = 5'd14;
    32'b0000_0000_0000_0000_001x_xxxx_xxxx_xxxx: b = 5'd13;
    32'b0000_0000_0000_0000_0001_xxxx_xxxx_xxxx: b = 5'd12;
    32'b0000_0000_0000_0000_0000_1xxx_xxxx_xxxx: b = 5'd11;
    32'b0000_0000_0000_0000_0000_01xx_xxxx_xxxx: b = 5'd10;
    32'b0000_0000_0000_0000_0000_001x_xxxx_xxxx: b = 5'd9;
    32'b0000_0000_0000_0000_0000_0001_xxxx_xxxx: b = 5'd8;
    32'b0000_0000_0000_0000_0000_0000_1xxx_xxxx: b = 5'd7;
    32'b0000_0000_0000_0000_0000_0000_01xx_xxxx: b = 5'd6;
    32'b0000_0000_0000_0000_0000_0000_001x_xxxx: b = 5'd5;
    32'b0000_0000_0000_0000_0000_0000_0001_xxxx: b = 5'd4;
    32'b0000_0000_0000_0000_0000_0000_0000_1xxx: b = 5'd3;
    32'b0000_0000_0000_0000_0000_0000_0000_01xx: b = 5'd2;
    32'b0000_0000_0000_0000_0000_0000_0000_001x: b = 5'd1;
    32'b0000_0000_0000_0000_0000_0000_0000_0001: b = 5'd0;
    default:                                     b = 5'b0;
  endcase
end
endmodule

module goldschmidt(
  input [31:0] a, b,
  input start, clk, clrn,
  output [31:0] q,
  output reg busy, ready,
  output reg [2:0] count
);
reg [63:0] reg_a, reg_b;
wire [63:0] two_minus_yi = ~reg_b + 1'b1;
wire [127:0] xi = reg_a * two_minus_yi;
wire [127:0] yi = reg_b * two_minus_yi;
assign q = reg_a[63:32] + |reg_a[31:29];
assign yn = reg_b[62:31];
always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    busy <= 0;
    ready <= 0;
  end else begin
    if(start) begin
      reg_a <= {1'b0,a,31'b0};
      reg_b <= {1'b0,b,31'b0};
      busy <= 1;
      ready <= 0;
      count <= 0;
    end else begin
      reg_a <= xi[126:63];
      reg_b <= yi[126:63];
      count <= count + 3'b1;
      if(count == 3'h4) begin
        busy <= 0;
        ready <= 1;
      end
    end
  end
end
endmodule

