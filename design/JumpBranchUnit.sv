`timescale 1ns / 1ps

module JumpBranchUnit #(
    parameter PC_W = 9
) (
    input logic [PC_W-1:0] Cur_PC,
    input logic [31:0] Imm,
    input logic [31:0] SrcA,        // Para JALR (conteúdo do registrador)
    input logic Branch,
    input logic Jump,
    input logic [1:0] JumpType,     // 00: não jump, 01: JAL, 10: JALR
    input logic [31:0] AluResult,
    output logic [31:0] PC_Imm,
    output logic [31:0] PC_Four,
    output logic [31:0] BrPC,
    output logic [31:0] PC_Plus4_Out,  // PC+4 para salvar no registrador
    output logic PcSel
);

  logic Branch_Sel;
  logic Jump_Sel;
  logic [31:0] PC_Full;
  logic [31:0] JALR_Target;

  assign PC_Full = {23'b0, Cur_PC};
  assign PC_Plus4_Out = PC_Full + 32'd4;  // PC+4 para salvar no rd

  assign PC_Imm = PC_Full + Imm;
  assign PC_Four = PC_Full + 32'd4;
  
  // Para JALR: (rs1 + imm) & ~1 (zera o bit menos significativo)
  assign JALR_Target = (SrcA + Imm) & 32'hFFFFFFFE;
  
  assign Branch_Sel = Branch && AluResult[0];  // Branch é tomado
  assign Jump_Sel = Jump;                      // Jump sempre é tomado
  
  // Lógica de seleção do próximo PC
  always_comb begin
    if (Jump_Sel) begin
      case (JumpType)
        2'b01: BrPC = PC_Imm;        // JAL: PC + imm
        2'b10: BrPC = JALR_Target;   // JALR: (rs1 + imm) & ~1
        default: BrPC = PC_Four;
      endcase
    end else if (Branch_Sel) begin
      BrPC = PC_Imm;                 // Branch: PC + imm
    end else begin
      BrPC = PC_Four;                // Sequencial: PC + 4
    end
  end

  assign PcSel = Branch_Sel || Jump_Sel;  // 1: branch/jump tomado; 0: sequencial

endmodule