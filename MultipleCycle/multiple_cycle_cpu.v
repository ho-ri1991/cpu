module multiple_cycle_cpu(
  input        clk,
  input        clrn,
  input[31:0]  frommem, // data from memory
  output[31:0] pc,      // program counter
  output[31:0] inst,    // instruction register
  output[31:0] alua, 
  output[31:0] alub, 
  output[31:0] alu,
  output       wmem,
  output[31:0] madr,    // memory address
  output[31:0] tomem,   // data to memory
  output[2:0]  state
);

// instruction
wire[5:0] op    = inst[31:26];
wire[4:0] rs    = inst[25:21];
wire[4:0] rt    = inst[20:16];
wire[4:0] rd    = inst[15:11];
wire[5:0] func  = inst[5:0];
wire[15:0] imm  = inst[15:0];
wire[25:0] addr = inst[25:0];

// control signals
wire[3:0] aluc;
wire[1:0] pcsrc;
wire wpc, wir, iord, selpc, wreg, regrt, m2reg, shift, aluimm, jal, sext;
wire[1:0] alusrcb;
reg[31:0] ir, dr, reg_a, reg_b, reg_c;
assign inst = ir;
// ALU
reg[31:0] alu_out;
wire[31:0] alu_in_a;
reg[31:0]  alu_in_b;
wire[31:0] shift_amount = {27'b0, inst[10:6]};
wire alu_out_zero = (alu_out == 32'b0);
wire[15:0] imm_sign = {16{sext&imm[15]}};
wire[31:0] immediate = {imm_sign, imm};
assign alua = alu_in_a;
assign alub = alu_in_b;
assign alu = alu_out;
// program counter
reg[31:0] program_counter;
reg[31:0] npc;
assign pc = program_counter;
// register file
wire[31:0] qa, qb; // output
wire[4:0]  reg_write_port = (jal ? {5{jal}} : (regrt ? rt : rd));
wire[31:0] reg_write_data = (jal ? program_counter : (m2reg ? dr : reg_c));
regfile regfile0(rs, rt, reg_write_data, reg_write_port, wreg, clk, clrn, qa, qb);

// program counter
always @(posedge clk or negedge clrn) begin
  if(!clrn)
    program_counter <= 32'h0;
  else if(wpc)
    program_counter <= npc;
end
always @(posedge clk or negedge clrn) begin
  if(!clrn) begin
    ir <= 32'h0;
    dr <= 32'h0;
    reg_a <= 32'h0;
    reg_b <= 32'h0;
    reg_c <= 32'h0;
  end else begin
    if(wir)
      ir <= frommem;
    dr <= frommem;
    reg_a <= qa;
    reg_b <= qb;
    reg_c <= alu_out;
  end
end

assign madr = iord ? reg_c : program_counter;
assign tomem = reg_b;
assign alu_in_a = (selpc ? program_counter : (shift ? shift_amount : reg_a));
always @* begin
  case(alusrcb)
    2'b00: alu_in_b <= reg_b;
    2'b01: alu_in_b <= 32'h4;
    2'b10: alu_in_b <= immediate;
    2'b11: alu_in_b <= immediate << 2;
  endcase
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
  case(pcsrc)
    2'b00: npc <= alu_out;                        // pc + 4
    2'b01: npc <= reg_c;                          // beq, bne
    2'b10: npc <= qa;                             // jr
    2'b11: npc <= {program_counter, addr, 2'b00}; // j, jal
  endcase
end

// control unit
multiple_cycle_cpu_control_unit control_unit(
  .op(op), .func(func), .z(alu_out_zero),
  .clk(clk), .clrn(clrn), .wpc(wpc), .wir(wir),
  .wmem(wmem), .wreg(wreg), .iord(iord), .regrt(regrt),
  .m2reg(m2reg), .aluc(aluc), .shift(shift),
  .alusrca(selpc), .alusrcb(alusrcb), .pcsrc(pcsrc),
  .jal(jal), .sext(sext), .state(state));

endmodule

module multiple_cycle_cpu_control_unit(
  input[5:0] op,
  input[5:0] func,
  input      z, // alu zero flag
  input      clk,
  input      clrn,
  output reg wpc,  // pc write enable
  output reg wir,  // instructino register write enable
  output reg wmem, // memory write enable
  output reg wreg, // register file write enable
  output reg iord, // instruction or data memory address selector
  output reg regrt, // use rt as a destination reg instead of rd
  output reg m2reg, // instruction is load word
  output reg[3:0] aluc,  // alu control signal
  output reg      shift, // instruction is sll, srl, sra
  output reg      alusrca,
  output reg[1:0] alusrcb,
  output reg[1:0] pcsrc,
  output reg      jal,
  output reg      sext,
  output reg[2:0] state
);

// dfa state
parameter[2:0] sif = 3'b000, sid = 3'b001, sexe = 3'b010, smem = 3'b011, swb = 3'b100;
reg[2:0] next_state;
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
wire i_shift = i_sll | i_srl| i_sra;

always @*begin
  // default values
  wpc <= 0;
  wir <= 0;
  wmem <= 0;
  wreg <= 0;
  iord <= 0;
  aluc <= 4'bx000;
  alusrca <= 0;
  alusrcb <= 2'h0;
  regrt <= 0;
  m2reg <= 0;
  shift <= 0;
  pcsrc <= 2'h0;
  jal <= 0;
  sext <= 1;
  case(state)
    sif: begin
      wpc <= 1;
      wir <= 1;
      alusrca <= 1;
      alusrcb <= 2'h1;
      next_state <= sid;
    end
    sid: begin
      if(i_j) begin
        pcsrc <= 2'h3;
        wpc <= 1;
        next_state <= sif;
      end else if(i_jal) begin
        pcsrc <= 2'h3;
        wpc <= 1;
        jal <= 1;
        wreg <= 1;
        next_state <= sif;
      end else if(i_jr) begin
        pcsrc <= 2'h2;
        wpc <= 1;
        next_state <= sif;
      end else begin
        aluc <= 4'bx000; // use alu to calculate branch address
        alusrca <= 1;
        alusrcb <= 2'h3;
        next_state <= sexe;
      end
    end
    sexe: begin
      aluc[3]  <= i_sra;
      aluc[2]  <= i_sub | i_or | i_srl | i_sra | i_ori | i_lui;
      aluc[1]  <= i_xor | i_sll| i_srl | i_sra | i_lui | i_xori | i_beq | i_bne;
      aluc[0]  <= i_and | i_or | i_sll | i_srl | i_sra | i_andi | i_ori;
      if(i_beq || i_bne) begin
        pcsrc <= 2'h1;
        wpc <= i_beq & z | i_bne & ~z;
        next_state <= sif;
      end else begin
        if(i_lw || i_sw) begin
          alusrcb <= 2'h2;
          next_state <= smem;
        end else begin
          if(i_shift) shift <= 1;
          if(i_addi || i_andi || i_ori || i_xori || i_lui)
            alusrcb <= 2'h2;
          if(i_andi || i_ori || i_xori)
            sext <= 0;
          next_state <= swb;
        end
      end
    end
    smem: begin
      iord <= 1;
      if(i_lw)
        next_state <= swb;
      else begin
        wmem <= 1;
        next_state <= sif;
      end
    end
    swb: begin
      if(i_lw) m2reg <= 1;
      if(i_lw || i_addi || i_andi || i_ori || i_xori || i_lui)
        regrt <= 1;
      wreg <= 1;
      next_state <= sif;
    end
    default:
      next_state <= sif;
  endcase
end
always @(posedge clk or negedge clrn) begin
  if(!clrn)
    state <= sif;
  else
    state <= next_state;
end

endmodule


