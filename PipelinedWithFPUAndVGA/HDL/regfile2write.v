module regfile2w(
  input[4:0]  rna,
  input[4:0]  rnb,
  input[31:0]  dx,
  input[4:0]   wnx,
  input        wex,
  input[31:0]  dy,
  input[4:0]   wny,
  input        wey,
  input        clk,
  input        clrn,
  output[31:0] qa,
  output[31:0] qb
);

reg[31:0] register[0:31];
assign qa = register[rna];
assign qb = register[rnb];
integer i;
always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    for(i = 0; i < 32; i = i + 1)
      register[i] <= 0;
  end else begin
    if(wey)
      register[wny] <= dy; // write port y, y has higher priority than x
    if(wex && (!wey || (wnx != wny)))
      register[wnx] <= dx;
  end
end

endmodule

