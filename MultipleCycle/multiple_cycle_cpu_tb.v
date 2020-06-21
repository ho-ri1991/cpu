`timescale 1ns/1ns
module multiple_cycle_cpu_tb;
    reg         clk,clrn,memclk;
    wire [31:0] a,b,alu,adr,tom,fromm,pc,ir;
    wire [2:0] state;
    wire wmem;
    multiple_cycle_cpu cpu(clk, clrn, fromm, pc, ir, a, b, alu, wmem, adr, tom, state);
    mcmem mem (
      .address(adr[9:2]),
      .clock(memclk),
      .data(tom),
      .wren(wmem),
      .q(fromm));
    initial begin
              clrn   <= 0;
              memclk <= 0;
              clk    <= 1;
        #1    clrn   <= 1;
        #1099 $finish;
    end
    always #1 memclk <= !memclk;
    always #2 clk  <= !clk;
endmodule

