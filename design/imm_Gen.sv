module imm_Gen (
    input  logic [31:0] inst_code,
    output logic [31:0] Imm_out
);

  always_comb
    case (inst_code[6:0])
      7'b0000011:  /*I-type load*/
        Imm_out = {{20{inst_code[31]}}, inst_code[31:20]};

      7'b0010011: begin /*I-type arithmetic/logic operations*/
        case(inst_code[14:12]) // funct3
          3'b001: // SLLI - shift amount (unsigned, only 5 bits)
            Imm_out = {27'b0, inst_code[24:20]}; 
          3'b101: // SRLI/SRAI - shift amount (unsigned, only 5 bits)
            Imm_out = {27'b0, inst_code[24:20]}; 
          default: // ADDI, SLTI - need sign extension for negative numbers
            Imm_out = {{20{inst_code[31]}}, inst_code[31:20]};
        endcase
      end

      7'b0100011:  /*S-type*/
        Imm_out = {{20{inst_code[31]}}, inst_code[31:25], inst_code[11:7]};

      7'b1100011:  /*B-type*/
        Imm_out = {
          {19{inst_code[31]}},
          inst_code[31],
          inst_code[7],
          inst_code[30:25],
          inst_code[11:8],
          1'b0
        };

      default: 
        Imm_out = 32'b0;
    endcase
endmodule