module single_cycle_cpu(
  input        clk, clrn,
  input[31:0]  inst,
  input[31:0]  mem,
  output[31:0] pc,
  output       wmem,
  output[31:0] alu, data
);

// instruction
wire[5:0] op    = inst[31:26];
wire[4:0] rs    = inst[25:21];
wire[4:0] rt    = inst[20:16];
wire[4:0] rd    = inst[15:11];
wire[5:0] func  = inst[5:0];
wire[15:0] imm  = inst[15:0];
wire[25:0] addr = inst[25:0];
// ALU
reg[31:0] alu_out;
wire[31:0] alu_in_a, alu_in_b;
wire[31:0] shift_amount = {27'b0, inst[10:6]};
wire alu_out_zero = (alu_out == 32'b0);
assign alu = alu_out;
// control signals
wire[3:0] aluc;
wire[1:0] pcsrc;
wire wmem, wreg, regrt, m2reg, shift, aluimm, jal, sext;
// program counter
reg[31:0] program_counter;
wire[31:0] pc_plus_4 = program_counter + 32'd4;
wire[15:0] imm_sign = {16{sext&imm[15]}};
wire[31:0] branch_pc = pc_plus_4 + {imm_sign[13:0], imm, 2'b00}; 
wire[31:0] jump_pc = {pc_plus_4[31:28], addr, 2'b00};
assign pc = program_counter;
// register file
wire[31:0] qa, qb; // output
wire[4:0]  reg_write_port = (jal ? {5{jal}} : (regrt ? rt : rd));
wire[31:0] reg_write_data = (jal ? pc_plus_4 : (m2reg ? mem : alu_out));
regfile regfile0(rs, rt, reg_write_data, reg_write_port, wreg, clk, clrn, qa, qb);
assign data = qb;

// program counter
always @(posedge clk or negedge clrn) begin
  if(!clrn)
    program_counter <= 32'b0;
  else
    if(pcsrc == 2'd0)
      program_counter <= pc_plus_4;
    else if(pcsrc == 2'd1)
      program_counter <= branch_pc;
    else if(pcsrc == 2'd2)
      program_counter <= qa;
    else if(pcsrc == 2'd3)
      program_counter <= jump_pc;
end

// ALU
assign alu_in_a = (shift ? shift_amount : qa);
assign alu_in_b = (aluimm ? {imm_sign, imm} : qb);
always @* begin
  casex(aluc)
    4'bx000: alu_out <= alu_in_a + alu_in_b;
    4'bx100: alu_out <= alu_in_a - alu_in_b;
    4'bx001: alu_out <= alu_in_a & alu_in_b;
    4'bx101: alu_out <= alu_in_a | alu_in_b;
    4'bx010: alu_out <= alu_in_a ^ alu_in_b;
    4'bx110: alu_out <= {alu_in_b[15:0], 16'b0};
    4'b0011: alu_out <= alu_in_b << shift_amount;
    4'b0111: alu_out <= alu_in_b >> shift_amount;
    4'b1111: alu_out <= $signed(alu_in_b) >>> shift_amount;
    default: alu_out <= 32'b0;
  endcase
end

// control unit
single_cycle_control_unit control_unit(
  .op(op), .func(func), .z(alu_out_zero),
  .wmem(wmem), .wreg(wreg), .regrt(regrt),
  .m2reg(m2reg), .aluc(aluc), .shift(shift),
  .aluimm(aluimm), .pcsrc(pcsrc), .jal(jal), .sext(sext));

endmodule


module single_cycle_control_unit(
  input[5:0]  op, func,
  input       z,      // ALU zero flag
  output      wmem,   // write enable of Data memory
  output      wreg,   // write enable of register file
  output      regrt,  // use rt as a destination register
  output      m2reg,  // select Data memory out instead of ALU out
  output[3:0] aluc,   // ALU control signal
  output      shift,  // use inst[10:6] instead of rs as a ALU input (shift amount)
  output      aluimm, // use inst[15:0] instead of rd as a ALU input (immediate value)
  output[1:0] pcsrc,  // selector of program counter source
  output      jal,    // jump and link (function call, $31 <= PC; PC <= target) instruction
  output      sext    // sign extension of immediate value
);

// op decode
wire i_add  = (op == 6'b000000) & (func == 6'b100000); // add
wire i_sub  = (op == 6'b000000) & (func == 6'b100010); // sub
wire i_and  = (op == 6'b000000) & (func == 6'b100100); // and
wire i_or   = (op == 6'b000000) & (func == 6'b100101); // or
wire i_xor  = (op == 6'b000000) & (func == 6'b100110); // xor
wire i_sll  = (op == 6'b000000) & (func == 6'b000000); // shift left logical
wire i_srl  = (op == 6'b000000) & (func == 6'b000010); // shift right logical
wire i_sra  = (op == 6'b000000) & (func == 6'b000011); // shift right arithmetic
wire i_jr   = (op == 6'b000000) & (func == 6'b001000); // jump register
wire i_addi = (op == 6'b001000); // add immediate
wire i_andi = (op == 6'b001100); // and immediate
wire i_ori  = (op == 6'b001101); // or immediate
wire i_xori = (op == 6'b001110); // xor immediate
wire i_lw   = (op == 6'b100011); // load word
wire i_sw   = (op == 6'b101011); // store word
wire i_beq  = (op == 6'b000100); // branch equal
wire i_bne  = (op == 6'b000101); // branch not equal
wire i_lui  = (op == 6'b001111); // load upper immediate
wire i_j    = (op == 6'b000010); // jump
wire i_jal  = (op == 6'b000011); // jump and link

assign regrt    = i_addi | i_andi | i_ori | i_xori | i_lw | i_lui;
assign jal      = i_jal;
assign m2reg    = i_lw;
assign wmem     = i_sw;
assign aluc[3]  = i_sra;
assign aluc[2]  = i_sub | i_or | i_srl | i_sra | i_ori | i_lui;
assign aluc[1]  = i_xor | i_sll| i_srl | i_sra | i_lui | i_xori | i_beq | i_bne;
assign aluc[0]  = i_and | i_or | i_sll | i_srl | i_sra | i_andi | i_ori;
assign shift    = i_sll | i_srl| i_sra;
assign aluimm   = i_addi | i_andi | i_ori | i_xori | i_lw | i_lui | i_sw;
assign sext     = i_addi | i_lw | i_sw | i_beq | i_bne;
assign pcsrc[1] = i_jr | i_j | i_jal;
assign pcsrc[0] = i_beq & z | i_bne & ~z | i_j | i_jal;
assign wreg     = i_add | i_sub | i_and | i_or | i_xor | i_sll | i_srl | i_sra |
                  i_addi | i_andi | i_ori | i_xori | i_lw | i_lui | i_jal;

endmodule





module scinstmem (a,inst);               // instruction memory, rom
    input  [31:0] a;                     // address
    output [31:0] inst;                  // instruction
    wire   [31:0] rom [0:31];            // rom cells: 32 words * 32 bits
    // rom[word_addr] = instruction      // (pc) label   instruction
    assign rom[5'h00] = 32'h3c010000;    // (00) main:   lui  $1, 0
    assign rom[5'h01] = 32'h34240050;    // (04)         ori  $4, $1, 80
    assign rom[5'h02] = 32'h20050004;    // (08)         addi $5, $0,  4
    assign rom[5'h03] = 32'h0c000018;    // (0c) call:   jal  sum
    assign rom[5'h04] = 32'hac820000;    // (10)         sw   $2, 0($4)
    assign rom[5'h05] = 32'h8c890000;    // (14)         lw   $9, 0($4)
    assign rom[5'h06] = 32'h01244022;    // (18)         sub  $8, $9, $4
    assign rom[5'h07] = 32'h20050003;    // (1c)         addi $5, $0,  3
    assign rom[5'h08] = 32'h20a5ffff;    // (20) loop2:  addi $5, $5, -1
    assign rom[5'h09] = 32'h34a8ffff;    // (24)         ori  $8, $5, 0xffff
    assign rom[5'h0A] = 32'h39085555;    // (28)         xori $8, $8, 0x5555
    assign rom[5'h0B] = 32'h2009ffff;    // (2c)         addi $9, $0, -1
    assign rom[5'h0C] = 32'h312affff;    // (30)         andi $10,$9, 0xffff
    assign rom[5'h0D] = 32'h01493025;    // (34)         or   $6, $10, $9
    assign rom[5'h0E] = 32'h01494026;    // (38)         xor  $8, $10, $9
    assign rom[5'h0F] = 32'h01463824;    // (3c)         and  $7, $10, $6
    assign rom[5'h10] = 32'h10a00001;    // (40)         beq  $5, $0, shift
    assign rom[5'h11] = 32'h08000008;    // (44)         j    loop2
    assign rom[5'h12] = 32'h2005ffff;    // (48) shift:  addi $5, $0, -1
    assign rom[5'h13] = 32'h000543c0;    // (4c)         sll  $8, $5, 15
    assign rom[5'h14] = 32'h00084400;    // (50)         sll  $8, $8, 16
    assign rom[5'h15] = 32'h00084403;    // (54)         sra  $8, $8, 16
    assign rom[5'h16] = 32'h000843c2;    // (58)         srl  $8, $8, 15
    assign rom[5'h17] = 32'h08000017;    // (5c) finish: j    finish
    assign rom[5'h18] = 32'h00004020;    // (60) sum:    add  $8, $0, $0
    assign rom[5'h19] = 32'h8c890000;    // (64) loop:   lw   $9, 0($4)
    assign rom[5'h1A] = 32'h20840004;    // (68)         addi $4, $4,  4
    assign rom[5'h1B] = 32'h01094020;    // (6c)         add  $8, $8, $9
    assign rom[5'h1C] = 32'h20a5ffff;    // (70)         addi $5, $5, -1
    assign rom[5'h1D] = 32'h14a0fffb;    // (74)         bne  $5, $0, loop
    assign rom[5'h1E] = 32'h00081000;    // (78)         sll  $2, $8, 0
    assign rom[5'h1F] = 32'h03e00008;    // (7c)         jr   $31
    assign inst = rom[a[6:2]];           // use word address to read rom
endmodule

module scdatamem (clk,dataout,datain,addr,we);           // data memory, ram
    input         clk;                   // clock
    input         we;                    // write enable
    input  [31:0] datain;                // data in (to memory)
    input  [31:0] addr;                  // ram address
    output [31:0] dataout;               // data out (from memory)
    reg    [31:0] ram [0:31];            // ram cells: 64 words * 32 bits
    assign dataout = ram[addr[6:2]];     // use word address to read ram
    always @ (posedge clk)
        if (we) ram[addr[6:2]] = datain; // use word address to write ram
    integer i;
    initial begin                        // initialize memory
        for (i = 0; i < 32; i = i + 1)
            ram[i] = 0;
        // ram[word_addr] = data         // (byte_addr) item in data array
        ram[5'h14] = 32'h000000a3;       // (50)  data[0]    0 +  A3 =  A3
        ram[5'h15] = 32'h00000027;       // (54)  data[1]   a3 +  27 =  ca
        ram[5'h16] = 32'h00000079;       // (58)  data[2]   ca +  79 = 143
        ram[5'h17] = 32'h00000115;       // (5c)  data[3]  143 + 115 = 258
        // ram[5'h18] should be 0x00000258, the sum stored by sw instruction
    end
endmodule
