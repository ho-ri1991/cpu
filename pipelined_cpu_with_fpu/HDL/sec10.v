module SEC10(
  input CLK, RST,
  output reg[6:0] HEX0
);

reg [25:0] cnt;
wire en1hz = (cnt==26'd49_999_999);

always @(posedge CLK) begin
  if(RST)
    cnt <= 26'b0;
  else if(en1hz)
    cnt <= 26'b0;
  else
    cnt <= cnt + 26'b1;
end

reg [3:0] sec;

always @(posedge CLK) begin
  if(RST)
    sec <= 4'h0;
  else if(en1hz)
    if(sec==4'h9)
      sec <= 4'h0;
    else
      sec <= sec + 4'h1;
end

always @* begin
  case(sec)
    4'h0: HEX0 = 7'b1000000;
    4'h1: HEX0 = 7'b1111001;
    4'h2: HEX0 = 7'b0100100;
    4'h3: HEX0 = 7'b0110000;
    4'h4: HEX0 = 7'b0011001;
    4'h5: HEX0 = 7'b0010010;
    4'h6: HEX0 = 7'b0000010;
    4'h7: HEX0 = 7'b1011000;
    4'h8: HEX0 = 7'b0000000;
    4'h9: HEX0 = 7'b0010000;
    default: HEX0 = 7'bxxxxxxx;
  endcase
end

endmodule

