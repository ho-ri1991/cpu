module floating_point_unit(
  input[31:0]  a,
  input[31:0]  b,
  input[2:0]   fc, // fp control
  input        wf, // write fpu register file (from ID stage)
  input[4:0]   fd, // write register number for fpu register file (from ID stage)
  input        ein1, // cache stall
  input        clk,
  input        clrn,
  output reg[31:0] ed, // output of E3 stage of fpu (fadd or fmul or fdiv or fsqrt)
  output reg[31:0] wd, // write data for fpu register file
  output reg[4:0]  wn, // write register number for fpu register file
  output reg       ww, // write enable for fpu register file
  output           st_ds, // stall caused by fdiv or fsqrt
  output reg[4:0]  e1n,   // write port num (E1 stage)
  output           e1w,   // write enable (E1 stage)
  output reg[4:0]  e2n,   // write port num (E2 state)
  output reg       e2w,   // write enable (E2 stage)
  output reg[4:0]  e3n,   // write port num (E3 stage)
  output reg       e3w,   // write enable (E3 stage)
  output reg[2:0]  e1c,   // for testing, fp control (E1 stage)
  output reg[2:0]  e2c,   // for testing, fp control (E2 stage)
  output reg[2:0]  e3c,   // for testing, fp control (E3 stage)
  output[4:0]      count_div,
  output[4:0]      count_sqrt,
  output           e,     // ein1 & ~st_ds
  input            ein2   // for canceling E1 instruction
);

reg[31:0] efa, efb;
reg e1w0, sub;
wire[25:0] reg_x_div, reg_x_sqrt;
wire[31:0] s_add, s_mul, s_div, s_sqrt, s_ftoi, s_itof, s_conv;
reg[31:0] conv1, conv2, conv3;
wire busy_div, stall_div, busy_sqrt, stall_sqrt;
wire fdiv  = (fc == 3'b100);
wire fsqrt = (fc == 3'b110);
assign e = ein1 & ~st_ds;
assign e1w = e1w0 & ein2;

pipelined_float_adder f_add(efa, efb, sub, 2'b0, s_add, clk, clrn, e);
pipelined_float_multiplier f_mul(efa, efb, 2'b0, s_mul, clk, clrn, e);
float_divider_newton f_div(a, b, 2'b0, fdiv, e, clk, clrn, s_div, busy_div, stall_div, count_div, reg_x_div);
float_sqrt_newton f_sqrt(a, 2'b0, fsqrt, e, clk, clrn, s_sqrt, busy_sqrt, stall_sqrt, count_sqrt, reg_x_sqrt);
wire dummy1, dummy2, dummy3, dummy4;
float_to_integer f_to_i(efa, s_ftoi, dummy1, dummy2, dummy3);
integer_to_float i_to_f(efa, s_itof, dummy4);
assign st_ds = stall_div | stall_sqrt;
always @* begin
  case(e3c)
    3'b000: ed = s_add;
    3'b001: ed = s_add;
    3'b010: ed = s_mul;
    3'b100: ed = s_div;
    3'b110: ed = s_sqrt;
    3'b011: ed = conv3;
    3'b101: ed = conv3;
    default: ed = 0;
  endcase
end

always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    sub <= 0; efa  <= 0; efb <= 0;
    e1c <= 0; e1w0 <= 0; e1n <= 0;
    e2c <= 0; e2w  <= 0; e2n <= 0; conv2 <= 0;
    e3c <= 0; e3w  <= 0; e3n <= 0; conv3 <= 0;
    wd  <= 0; ww   <= 0; wn  <= 0;
  end else if(e) begin
    sub <= fc[0];   efa  <= a;   efb <= b;
    e1c <= fc[2:0]; e1w0 <= wf;  e1n <= fd;
    e2c <= e1c;     e2w  <= e1w; e2n <= e1n; conv2 <= (e1c == 3'b011 ? s_itof : s_ftoi);
    e3c <= e2c;     e3w  <= e2w; e3n <= e2n; conv3 <= conv2;
    wd  <= ed;      ww   <= e3w; wn  <= e3n;
  end
end

endmodule
