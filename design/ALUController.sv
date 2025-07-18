`timescale 1ns / 1ps

module ALUController (
    //Inputs
    input logic [1:0] ALUOp,  // 2-bit opcode field from the Controller--00: LW/SW; 01:Branch; 10: Rtype/Itype
    input logic [6:0] Funct7,  // bits 25 to 31 of the instruction
    input logic [2:0] Funct3,  // bits 12 to 14 of the instruction

    //Output
    output logic [3:0] Operation  // operation selection for ALU
);

  always_comb begin
    case (ALUOp)
      2'b00: Operation = 4'b0010;  // LW/SW: ADD
      
      2'b01: begin  // Branch operations
      case (Funct3)
        3'b000: Operation = 4'b1000;  // BEQ (Equal)
        3'b001: Operation = 4'b1001;  // BNE (Not Equal)
        3'b100: Operation = 4'b1100;  // BLT (Less Than)
        3'b101: Operation = 4'b1101;  // BGE (Greater Equal)
      endcase
      end
      
      2'b10: begin  // R-type and I-type operations
        case (Funct3)
          3'b000: begin  // ADD/ADDI or SUB
            if (Funct7 == 7'b0100000)  // SUB R-type específico
              Operation = 4'b0110;  // SUB
            else
              Operation = 4'b0010;  // ADD/ADDI (padrão para I-type)
          end
          
          3'b001: Operation = 4'b0011;  // SLL/SLLI (Shift Left Logical)
          
          3'b010: Operation = 4'b1100;  // SLT/SLTI (Set Less Than)
          
          3'b100: Operation = 4'b0100;  // XOR/XORI
          
          3'b101: begin  // SRL/SRLI or SRA/SRAI
            // Correção similar para shifts
            if (Funct7 == 7'b0100000)  // SRA R-type específico
              Operation = 4'b1010;  // SRA
            else
              Operation = 4'b0101;  // SRL/SRLI (padrão)
          end
          
          3'b110: Operation = 4'b0001;  // OR/ORI
          
          3'b111: Operation = 4'b0000;  // AND/ANDI
          
          default: Operation = 4'b0000;
        endcase
      end
      
      default: Operation = 4'b0000;
    endcase
  end

endmodule
