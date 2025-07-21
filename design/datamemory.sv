module datamemory #(
    parameter DM_ADDRESS = 9,
    parameter DATA_W = 32
) (
    input logic clk,
    input logic MemRead,
    input logic MemWrite,
    input logic [DM_ADDRESS - 1:0] a,
    input logic [DATA_W - 1:0] wd,
    input logic [2:0] Funct3,
    output logic [DATA_W - 1:0] rd
);
    logic [31:0] raddress;
    logic [31:0] waddress;
    logic [31:0] Datain;
    logic [31:0] Dataout;
    logic [3:0] Wr;
    
    Memoria32Data mem32 (
        .raddress(raddress),
        .waddress(waddress),
        .Clk(~clk),
        .Datain(Datain),
        .Dataout(Dataout),
        .Wr(Wr)
    );
    
    // Lógica combinacional para leitura
    always_comb begin
        // Endereço sempre alinhado em palavra (remove 2 LSBs)
        raddress = {{22{1'b0}}, a[8:2], 2'b00};
        waddress = {{22{1'b0}}, a[8:2], 2'b00};
        
        // Default values
        Datain = wd;
        Wr = 4'b0000;
        rd = 32'b0;
        
        if (MemRead) begin
            case (Funct3)
                3'b000: begin // LB (Load Byte with Sign Extension)
                    case (a[1:0])
                        2'b00: rd = {{24{Dataout[7]}}, Dataout[7:0]};
                        2'b01: rd = {{24{Dataout[15]}}, Dataout[15:8]};
                        2'b10: rd = {{24{Dataout[23]}}, Dataout[23:16]};
                        2'b11: rd = {{24{Dataout[31]}}, Dataout[31:24]};
                    endcase
                end
                3'b001: begin // LH (Load Halfword with Sign Extension)
                    case (a[1])
                        1'b0: rd = {{16{Dataout[15]}}, Dataout[15:0]};
                        1'b1: rd = {{16{Dataout[31]}}, Dataout[31:16]};
                    endcase
                end
                3'b010: begin // LW (Load Word)
                    rd = Dataout;
                end
                3'b100: begin // LBU (Load Byte Unsigned)
                    case (a[1:0])
                        2'b00: rd = {24'b0, Dataout[7:0]};
                        2'b01: rd = {24'b0, Dataout[15:8]};
                        2'b10: rd = {24'b0, Dataout[23:16]};
                        2'b11: rd = {24'b0, Dataout[31:24]};
                    endcase
                end
                3'b101: begin // LHU (Load Halfword Unsigned)
                    case (a[1])
                        1'b0: rd = {16'b0, Dataout[15:0]};
                        1'b1: rd = {16'b0, Dataout[31:16]};
                    endcase
                end
                default: rd = Dataout;
            endcase
        end 
        else if (MemWrite) begin
            case (Funct3)
                3'b000: begin // SB (Store Byte)
                    case (a[1:0])
                        2'b00: begin
                            Wr = 4'b0001;
                            Datain = {Dataout[31:8], wd[7:0]};
                        end
                        2'b01: begin
                            Wr = 4'b0010;
                            Datain = {Dataout[31:16], wd[7:0], Dataout[7:0]};
                        end
                        2'b10: begin
                            Wr = 4'b0100;
                            Datain = {Dataout[31:24], wd[7:0], Dataout[15:0]};
                        end
                        2'b11: begin
                            Wr = 4'b1000;
                            Datain = {wd[7:0], Dataout[23:0]};
                        end
                    endcase
                end
                3'b001: begin // SH (Store Halfword)
                    case (a[1])
                        1'b0: begin
                            Wr = 4'b0011;
                            Datain = {Dataout[31:16], wd[15:0]};
                        end
                        1'b1: begin
                            Wr = 4'b1100;
                            Datain = {wd[15:0], Dataout[15:0]};
                        end
                    endcase
                end
                3'b010: begin // SW (Store Word)
                    Wr = 4'b1111;
                    Datain = wd;
                end
            endcase
        end
    end
endmodule
