`timescale 1ns / 1ps

module Controller (
    //Input
    input logic [6:0] Opcode,
    //7-bit opcode field from the instruction

    //Outputs
    output logic ALUSrc,
    output logic MemtoReg,
    output logic RegWrite,
    output logic MemRead,
    output logic MemWrite,
    output logic [1:0] ALUOp,
    output logic Branch,
    output logic Jump,
    output logic [1:0] JumpType,
    output logic Halt      
);

  logic [6:0] R_TYPE, LOAD, STORE, BR, I_TYPE, JAL, JALR, HALT_OP;

  assign R_TYPE = 7'b0110011;  //add,and,sub,slt,xor,or
  assign I_TYPE = 7'b0010011;  //addi,slti,slli,srli,srai
  assign LOAD   = 7'b0000011;  //lw, lh, lb, lbu, lhu
  assign STORE  = 7'b0100011;  //sw, sh, sb 
  assign BR     = 7'b1100011;  //beq, bne, blt, bge
  assign JAL    = 7'b1101111;  //jal
  assign JALR   = 7'b1100111;  //jalr
  assign HALT_OP = 7'b1111111; //halt 

  assign ALUSrc = (Opcode == LOAD || Opcode == STORE || Opcode == I_TYPE || Opcode == JALR);
  assign MemtoReg = (Opcode == LOAD);
  assign RegWrite = (Opcode == R_TYPE || Opcode == LOAD || Opcode == I_TYPE || Opcode == JAL || Opcode == JALR);
  assign MemRead = (Opcode == LOAD);
  assign MemWrite = (Opcode == STORE);
  assign ALUOp[0] = (Opcode == BR);
  assign ALUOp[1] = (Opcode == R_TYPE || Opcode == I_TYPE);
  assign Branch = (Opcode == BR);
  assign Jump = (Opcode == JAL || Opcode == JALR);
  assign Halt = (Opcode == HALT_OP);  // NOVO - Detecta HALT
  
  // JumpType: 00: não jump, 01: JAL, 10: JALR
  assign JumpType = (Opcode == JAL) ? 2'b01 : 
                    (Opcode == JALR) ? 2'b10 : 2'b00;
endmodule