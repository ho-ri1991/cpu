module cla32(
  input[31:0]  a, b,
  input        c_in,
  output[31:0] s
);
wire g, p;
cla_32 add0(a, b, c_in, g, p, s);
endmodule

module cla_32(
  input[31:0]  a, b,
  input        c_in,
  output       g_out, p_out,
  output[31:0] s
);
wire[1:0] g, p;
wire      c;
cla_16 add0(a[15:0],  b[15:0],  c_in, g[0], p[0], s[15:0]);
cla_16 add1(a[31:16], b[31:16], c,    g[1], p[1], s[31:16]);
gp    gp0(g, p, c_in, g_out, p_out, c);
endmodule

module cla_16(
  input[15:0]  a, b,
  input        c_in,
  output       g_out, p_out,
  output[15:0] s
);
wire[1:0] g, p;
wire      c;
cla_8 add0(a[7:0],  b[7:0],  c_in, g[0], p[0], s[7:0]);
cla_8 add1(a[15:8], b[15:8], c,    g[1], p[1], s[15:8]);
gp    gp0(g, p, c_in, g_out, p_out, c);
endmodule

module cla_8(
  input[7:0]  a, b,
  input       c_in,
  output      g_out, p_out,
  output[7:0] s
);
wire[1:0] g, p;
wire      c;
cla_4 add0(a[3:0], b[3:0], c_in, g[0], p[0], s[3:0]);
cla_4 add1(a[7:4], b[7:4], c,    g[1], p[1], s[7:4]);
gp    gp0(g, p, c_in, g_out, p_out, c);
endmodule

module cla_4(
  input[3:0]  a, b,
  input       c_in,
  output      g_out, p_out,
  output[3:0] s
);
wire[1:0] g, p;
wire      c;
cla_2 add0(a[1:0], b[1:0], c_in, g[0], p[0], s[1:0]);
cla_2 add1(a[3:2], b[3:2], c,    g[1], p[1], s[3:2]);
gp    gp0(g, p, c_in, g_out, p_out, c);
endmodule

module cla_2(
  input[1:0]  a, b,
  input       c_in,
  output      g_out, p_out,
  output[1:0] s
);
wire[1:0] g, p;
wire      c;
add add0(a[0], b[0], c_in, g[0], p[0], s[0]);
add add1(a[1], b[1], c,    g[1], p[1], s[1]);
gp  gp0(g, p, c_in, g_out, p_out, c);
endmodule

module gp(
  input[1:0] g, p,
  input      c_in,
  output     g_out, p_out, c_out
);
assign g_out = g[1] | p[1] & g[0];
assign p_out = p[1] & p[0];
assign c_out = g[0] | p[0] & c_in;
endmodule

module add(
  input  a, b, c,
  output g, p, s
);
assign g = a & b;
assign p = a | b;
assign s = a ^ b ^ c;
endmodule

