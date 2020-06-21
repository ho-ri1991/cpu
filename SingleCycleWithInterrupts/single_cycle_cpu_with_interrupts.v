module single_cycle_cpu_with_interrupt(
  input        clk, clrn,
  input[31:0]  inst,
  input[31:0]  mem,
  output[31:0] pc,
  output       wmem,
  output[31:0] alu, data,
  input        intr,
  output       inta
);

parameter BASE = 32'h00000008; // exception/interrupt handler entry point
parameter ZERO = 32'h00000000;

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
reg       alu_overflow;
wire[31:0] alu_in_a, alu_in_b;
wire[31:0] shift_amount = {27'b0, inst[10:6]};
wire alu_out_zero = (alu_out == 32'b0);
assign alu = alu_out;
// control signals
wire[3:0] aluc;
wire[1:0] pcsrc, selpc, mfc0;
wire[31:0] cause;
wire wmem, wreg, regrt, m2reg, shift, aluimm, jal, sext, aluoverflow, exc, wsta, wcau, wepc, mtc0;
// program counter
reg[31:0] program_counter, next_pc;
wire[31:0] pc_plus_4 = program_counter + 32'd4;
wire[15:0] imm_sign = {16{sext&imm[15]}};
wire[31:0] branch_pc = pc_plus_4 + {imm_sign[13:0], imm, 2'b00}; 
wire[31:0] jump_pc = {pc_plus_4[31:28], addr, 2'b00};
assign pc = program_counter;
// register file
wire[31:0] qa, qb; // output
reg[31:0] c0_status, c0_cause, c0_epc; // c0 register
wire[4:0]  reg_write_port = (jal ? {5{jal}} : (regrt ? rt : rd));
reg[31:0] reg_write_data;
regfile regfile0(rs, rt, reg_write_data, reg_write_port, wreg, clk, clrn, qa, qb);
always @* begin
  if(jal)
    reg_write_data <= pc_plus_4;
  else begin
    case(mfc0)
      2'b00: reg_write_data <= m2reg ? mem : alu_out;
      2'b01: reg_write_data <= c0_status;
      2'b10: reg_write_data <= c0_cause;
      2'b11: reg_write_data <= c0_epc;
    endcase
  end
end
assign data = qb;
always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    c0_status <= 32'b0;
    c0_cause  <= 32'b0;
    c0_epc    <= 32'b0;
  end
  else begin
    if(wcau)
      c0_cause <= (mtc0 ? data : cause);
    if(wsta)
      c0_status <= (mtc0 ? data : (exc ? (c0_status << 4) : (c0_status >> 4)));
    if(wepc)
      c0_epc <= (mtc0 ? data : (inta ? next_pc : program_counter));
  end
end

// program counter
always @* begin
  case(pcsrc)
    2'b00: next_pc <= pc_plus_4;
    2'b01: next_pc <= branch_pc;
    2'b10: next_pc <= qa;
    2'b11: next_pc <= jump_pc;
  endcase
end
always @(posedge clk or negedge clrn) begin
  if(!clrn)
    program_counter <= 32'b0;
  else
    if(selpc == 2'b00)
      program_counter <= next_pc;
    else if(selpc == 2'b01)
      program_counter <= c0_epc;
    else if(selpc == 2'b10)
      program_counter <= BASE;
    else
      program_counter <= ZERO;
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
always @*begin
  casex(aluc)
    4'bx000: alu_overflow <= (~alu_in_a[31]&~alu_in_b[31]& alu_out[31]) | ( alu_in_a[31]& alu_in_b[31]&~alu_out[31]);
    4'bx100: alu_overflow <= (~alu_in_a[31]& alu_in_b[31]&~alu_out[31]) | ( alu_in_a[31]&~alu_in_b[31]&~alu_out[31]);
    default: alu_overflow <= 1'b0;
  endcase
end

// control unit
single_cycle_control_unit control_unit(
  .op(op), .op1(rs), .rd(rd), .func(func), .z(alu_out_zero),
  .wmem(wmem), .wreg(wreg), .regrt(regrt),
  .m2reg(m2reg), .aluc(aluc), .shift(shift),
  .aluimm(aluimm), .pcsrc(pcsrc), .jal(jal), .sext(sext),
  .intr(intr), .inta(inta), .v(alu_overflow),
  .sta(c0_status), .cause(cause), .exc(exc),
  .wsta(wsta), .wcau(wcau), .wepc(wepc),
  .mtc0(mtc0), .mfc0(mfc0), .selpc(selpc));

endmodule


module single_cycle_control_unit(
  input[5:0]   op,
  input[4:0]   op1,
  input[4:0]   rd,
  input[5:0]   func,
  input        z,      // ALU zero flag
  output       wmem,   // write enable of Data memory
  output       wreg,   // write enable of register file
  output       regrt,  // use rt as a destination register
  output       m2reg,  // select Data memory out instead of ALU out
  output[3:0]  aluc,   // ALU control signal
  output       shift,  // use inst[10:6] instead of rs as a ALU input (shift amount)
  output       aluimm, // use inst[15:0] instead of rd as a ALU input (immediate value)
  output[1:0]  pcsrc,  // selector of program counter source (pc+4, beq/bne, jr, jump/jal)
  output       jal,    // jump and link (function call, $31 <= PC; PC <= target) instruction
  output       sext,   // sign extension of immediate value
  input        intr,   // interrupt request
  output       inta,   // interrupt ack
  input        v,      // alu signed add/sub overflow
  input[31:0]  sta,    // c0 status register
  output[31:0] cause,  // c0 cause register
  output       exc,    // exception or interrupt occured
  output       wsta,   // write enable of c0 status
  output       wcau,   // write enable of c0 cause
  output       wepc,   // write enable of c0 epc
  output       mtc0,   // move to c0 regs
  output[1:0]  mfc0,   // selector of alu/mem, sta, cau, epc
  output[1:0]  selpc   // select for program counter (pc from pcsrc, epc, base)
);

// op decode
wire i_add     = (op == 6'b000000) & (func == 6'b100000); // add
wire i_sub     = (op == 6'b000000) & (func == 6'b100010); // sub
wire i_and     = (op == 6'b000000) & (func == 6'b100100); // and
wire i_or      = (op == 6'b000000) & (func == 6'b100101); // or
wire i_xor     = (op == 6'b000000) & (func == 6'b100110); // xor
wire i_sll     = (op == 6'b000000) & (func == 6'b000000); // shift left logical
wire i_srl     = (op == 6'b000000) & (func == 6'b000010); // shift right logical
wire i_sra     = (op == 6'b000000) & (func == 6'b000011); // shift right arithmetic
wire i_jr      = (op == 6'b000000) & (func == 6'b001000); // jump register
wire i_addi    = (op == 6'b001000); // add immediate
wire i_andi    = (op == 6'b001100); // and immediate
wire i_ori     = (op == 6'b001101); // or immediate
wire i_xori    = (op == 6'b001110); // xor immediate
wire i_lw      = (op == 6'b100011); // load word
wire i_sw      = (op == 6'b101011); // store word
wire i_beq     = (op == 6'b000100); // branch equal
wire i_bne     = (op == 6'b000101); // branch not equal
wire i_lui     = (op == 6'b001111); // load upper immediate
wire i_j       = (op == 6'b000010); // jump
wire i_jal     = (op == 6'b000011); // jump and link
wire i_mfc0    = (op == 6'b010000) & (op1 == 5'b00000); // move from c0 register
wire i_mtc0    = (op == 6'b010000) & (op1 == 5'b00100); // move to c0 register
wire i_eret    = (op == 6'b010000) & (op1 == 5'b10000) & (func == 6'b011000); // return from exception handler
wire i_syscall = (op == 6'b000000) & (func == 6'b001100); // syscall

wire unimplemented_inst =
  ~(i_add|i_sub|i_and|i_or|i_xor|i_sll|i_srl|i_sra|
    i_jr|i_addi|i_andi|i_ori|i_xori|i_lw|i_sw|
    i_beq|i_bne|i_lui|i_j|i_jal|i_mfc0|i_mtc0|i_eret|i_syscall);
wire overflow = v & (i_add|i_sub|i_addi);
wire exc = (sta[0]&intr) | (sta[1]&i_syscall) | (sta[2]&unimplemented_inst) | (sta[3]&overflow);

assign inta = (sta[0] & intr);
// exccode: 0 0 : intr
//          0 1 : i_syscall
//          1 0 : unimplemented_instruction
//          1 1 : ALU overflow
assign cause = {28'h0, unimplemented_inst|overflow, i_syscall|overflow, 2'b00};
parameter C0_STATUS = 5'd12;
parameter C0_CAUSE  = 5'd13;
parameter C0_EPC    = 5'd14;
wire rd_is_c0_status = rd == C0_STATUS;
wire rd_is_c0_cause  = rd == C0_CAUSE;
wire rd_is_c0_epc    = rd == C0_EPC;
// mfc0:    0 0 : alu_mem
//          0 1 : sta
//          1 0 : cau
//          1 1 : epc
assign mfc0[0] = (i_mfc0 & rd_is_c0_status) | (i_mfc0 & rd_is_c0_epc);
assign mfc0[1] = (i_mfc0 & rd_is_c0_cause)  | (i_mfc0 & rd_is_c0_epc);
// selpc:   0 0 : pc from pcsrc
//          0 1 : epc
//          1 0 : exc_base
//          1 1 : unused
assign selpc[0] = i_eret;
assign selpc[1] = exc;
assign mtc0 = i_mtc0;
assign wsta = exc | (mtc0&(rd == C0_STATUS)) | i_eret;
assign wcau = exc | (mtc0&(rd == C0_CAUSE));
assign wepc = exc | (mtc0&(rd == C0_EPC));

assign regrt    = i_addi | i_andi | i_ori | i_xori | i_lw | i_lui | i_mfc0;
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
                  i_addi | i_andi | i_ori | i_xori | i_lw | i_lui | i_jal | i_mfc0;

endmodule



module scinstmem (a,inst);               // instruction memory, rom
    input  [31:0] a;                     // address
    output [31:0] inst;                  // instruction
    wire   [31:0] rom [0:49];            // rom cells: 32 words * 32 bits
    // rom[word_addr] = instruction      // (pc) label   instruction
    assign rom[6'h00] = 32'h0800001d; // (00)    j    start             # entry on reset            %
    assign rom[6'h01] = 32'h00000000; // (04)    nop                    #                           %
    assign rom[6'h02] = 32'h401a6800; // (08)    mfc0 $26, c0_cause     # read cp0 cause reg        %
    assign rom[6'h03] = 32'h335b000c; // (0c)    andi $27, $26, 0xc     # get exccode, 2 bits here  %
    assign rom[6'h04] = 32'h8f7b0020; // (10)    lw   $27, j_table($27) # get address from table    %
    assign rom[6'h05] = 32'h00000000; // (14)    nop                    #                           %
    assign rom[6'h06] = 32'h03600008; // (18)    jr   $27               # jump to that address      %
    assign rom[6'h07] = 32'h00000000; // (1c)    nop                    #                           %
    assign rom[6'h08] = 32'h00000000; // (20)
    assign rom[6'h09] = 32'h00000000; // (24)
    assign rom[6'h0A] = 32'h00000000; // (28)
    assign rom[6'h0B] = 32'h00000000; // (28)
    assign rom[6'h0C] = 32'h00000000; // (30)    nop                    # deal with interrupt here  %
    assign rom[6'h0D] = 32'h42000018; // (34)    eret                   # retrun from interrupt     %
    assign rom[6'h0E] = 32'h00000000; // (38)    nop                    #                           %
    assign rom[6'h0F] = 32'h00000000; // (3c)    nop                    # do something here         %
    assign rom[6'h10] = 32'h401a7000; // (40)    mfc0  $26, c0_epc      # get epc                   %
    assign rom[6'h11] = 32'h235a0004; // (44)    addi  $26, $26, 4      # epc + 4                   %
    assign rom[6'h12] = 32'h409a7000; // (48)    mtc0  $26, c0_epc      # epc <- epc + 4            %
    assign rom[6'h13] = 32'h42000018; // (4c)    eret                   # retrun from exception     %
    assign rom[6'h14] = 32'h00000000; // (50)    nop                    #                           %
    assign rom[6'h15] = 32'h00000000; // (54)    nop                    # do something here         %
    assign rom[6'h16] = 32'h08000010; // (58)    j     epc_plus4        # return                    %
    assign rom[6'h17] = 32'h00000000; // (5c)    nop                    #                           %
    assign rom[6'h18] = 32'h00000000; // (60)
    assign rom[6'h19] = 32'h00000000; // (64)
    assign rom[6'h1A] = 32'h00000000; // (68)    nop                    # do something here         %
    assign rom[6'h1B] = 32'h08000010; // (6c)    j     epc_plus4        # return                    %
    assign rom[6'h1C] = 32'h00000000; // (70)    nop                    #                           %
    assign rom[6'h1D] = 32'h2008000f; // (74)    addi $8, $0, 0xf       # im[3:0] <- 1111           %
    assign rom[6'h1E] = 32'h40886000; // (78)    mtc0 $8, c0_status     # exc/intr enable           %
    assign rom[6'h1F] = 32'h8c080048; // (7c)    lw   $8, 0x48($0)      # try overflow exception    %
    assign rom[6'h20] = 32'h8c09004c; // (80)    lw   $9, 0x4c($0)      # caused by add             %
    assign rom[6'h21] = 32'h01094020; // (84)    add  $9, $9, $8        # overflow                  %
    assign rom[6'h22] = 32'h00000000; // (88)    nop                    #                           %
    assign rom[6'h23] = 32'h0000000c; // (8c)    syscall                # system call               %
    assign rom[6'h24] = 32'h00000000; // (90)    nop                    #                           %
    assign rom[6'h25] = 32'h0128001a; // (94)    div  $9, $8            # div, but not implemented  %
    assign rom[6'h26] = 32'h00000000; // (98)    nop                    #                           %
    assign rom[6'h27] = 32'h34040050; // (9c)    ori  $4, $1, 0x50      # address of data[0]        %
    assign rom[6'h28] = 32'h20050004; // (a0)    addi $5, $0,  4        # counter                   %
    assign rom[6'h29] = 32'h00004020; // (a4)    add  $8, $0, $0        # sum <- 0                  %
    assign rom[6'h2A] = 32'h8c890000; // (a8)    lw   $9, 0($4)         # load data                 %
    assign rom[6'h2B] = 32'h20840004; // (ac)    addi $4, $4,  4        # address + 4               %
    assign rom[6'h2C] = 32'h01094020; // (b0)    add  $8, $8, $9        # sum                       %
    assign rom[6'h2D] = 32'h20a5ffff; // (b4)    addi $5, $5, -1        # counter - 1               %
    assign rom[6'h2E] = 32'h14a0fffb; // (b8)    bne  $5, $0, loop      # finish?                   %
    assign rom[6'h2F] = 32'h00000000; // (bc)    nop                    #                           %
    assign rom[6'h30] = 32'h08000030; // (c0)    j    finish            # dead loop                 %
    assign inst = rom[a[7:2]];           // use word address to read rom
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
        ram[5'h08] = 32'h00000030;
        ram[5'h09] = 32'h0000003c;
        ram[5'h0A] = 32'h00000054;
        ram[5'h0B] = 32'h00000068;
        ram[5'h12] = 32'h00000002;
        ram[5'h13] = 32'h7FFFFFFF;
        ram[5'h14] = 32'h000000a3;       // (50)  data[0]    0 +  A3 =  A3
        ram[5'h15] = 32'h00000027;       // (54)  data[1]   a3 +  27 =  ca
        ram[5'h16] = 32'h00000079;       // (58)  data[2]   ca +  79 = 143
        ram[5'h17] = 32'h00000115;       // (5c)  data[3]  143 + 115 = 258
        // ram[5'h18] should be 0x00000258, the sum stored by sw instruction
    end
endmodule

//module scinstmem (a,inst);               // instruction memory, rom
//    input  [31:0] a;                     // address
//    output [31:0] inst;                  // instruction
//    wire   [31:0] rom [0:31];            // rom cells: 32 words * 32 bits
//    // rom[word_addr] = instruction      // (pc) label   instruction
//    assign rom[5'h00] = 32'h3c010000;    // (00) main:   lui  $1, 0
//    assign rom[5'h01] = 32'h34240050;    // (04)         ori  $4, $1, 80
//    assign rom[5'h02] = 32'h20050004;    // (08)         addi $5, $0,  4
//    assign rom[5'h03] = 32'h0c000018;    // (0c) call:   jal  sum
//    assign rom[5'h04] = 32'hac820000;    // (10)         sw   $2, 0($4)
//    assign rom[5'h05] = 32'h8c890000;    // (14)         lw   $9, 0($4)
//    assign rom[5'h06] = 32'h01244022;    // (18)         sub  $8, $9, $4
//    assign rom[5'h07] = 32'h20050003;    // (1c)         addi $5, $0,  3
//    assign rom[5'h08] = 32'h20a5ffff;    // (20) loop2:  addi $5, $5, -1
//    assign rom[5'h09] = 32'h34a8ffff;    // (24)         ori  $8, $5, 0xffff
//    assign rom[5'h0A] = 32'h39085555;    // (28)         xori $8, $8, 0x5555
//    assign rom[5'h0B] = 32'h2009ffff;    // (2c)         addi $9, $0, -1
//    assign rom[5'h0C] = 32'h312affff;    // (30)         andi $10,$9, 0xffff
//    assign rom[5'h0D] = 32'h01493025;    // (34)         or   $6, $10, $9
//    assign rom[5'h0E] = 32'h01494026;    // (38)         xor  $8, $10, $9
//    assign rom[5'h0F] = 32'h01463824;    // (3c)         and  $7, $10, $6
//    assign rom[5'h10] = 32'h10a00001;    // (40)         beq  $5, $0, shift
//    assign rom[5'h11] = 32'h08000008;    // (44)         j    loop2
//    assign rom[5'h12] = 32'h2005ffff;    // (48) shift:  addi $5, $0, -1
//    assign rom[5'h13] = 32'h000543c0;    // (4c)         sll  $8, $5, 15
//    assign rom[5'h14] = 32'h00084400;    // (50)         sll  $8, $8, 16
//    assign rom[5'h15] = 32'h00084403;    // (54)         sra  $8, $8, 16
//    assign rom[5'h16] = 32'h000843c2;    // (58)         srl  $8, $8, 15
//    assign rom[5'h17] = 32'h08000017;    // (5c) finish: j    finish
//    assign rom[5'h18] = 32'h00004020;    // (60) sum:    add  $8, $0, $0
//    assign rom[5'h19] = 32'h8c890000;    // (64) loop:   lw   $9, 0($4)
//    assign rom[5'h1A] = 32'h20840004;    // (68)         addi $4, $4,  4
//    assign rom[5'h1B] = 32'h01094020;    // (6c)         add  $8, $8, $9
//    assign rom[5'h1C] = 32'h20a5ffff;    // (70)         addi $5, $5, -1
//    assign rom[5'h1D] = 32'h14a0fffb;    // (74)         bne  $5, $0, loop
//    assign rom[5'h1E] = 32'h00081000;    // (78)         sll  $2, $8, 0
//    assign rom[5'h1F] = 32'h03e00008;    // (7c)         jr   $31
//    assign inst = rom[a[6:2]];           // use word address to read rom
//endmodule
//
//module scdatamem (clk,dataout,datain,addr,we);           // data memory, ram
//    input         clk;                   // clock
//    input         we;                    // write enable
//    input  [31:0] datain;                // data in (to memory)
//    input  [31:0] addr;                  // ram address
//    output [31:0] dataout;               // data out (from memory)
//    reg    [31:0] ram [0:31];            // ram cells: 64 words * 32 bits
//    assign dataout = ram[addr[6:2]];     // use word address to read ram
//    always @ (posedge clk)
//        if (we) ram[addr[6:2]] = datain; // use word address to write ram
//    integer i;
//    initial begin                        // initialize memory
//        for (i = 0; i < 32; i = i + 1)
//            ram[i] = 0;
//        // ram[word_addr] = data         // (byte_addr) item in data array
//        ram[5'h14] = 32'h000000a3;       // (50)  data[0]    0 +  A3 =  A3
//        ram[5'h15] = 32'h00000027;       // (54)  data[1]   a3 +  27 =  ca
//        ram[5'h16] = 32'h00000079;       // (58)  data[2]   ca +  79 = 143
//        ram[5'h17] = 32'h00000115;       // (5c)  data[3]  143 + 115 = 258
//        // ram[5'h18] should be 0x00000258, the sum stored by sw instruction
//    end
//endmodule
