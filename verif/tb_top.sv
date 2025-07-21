`timescale 1ns / 1ps

module tb_top;
    // Clock and reset signal declaration
    logic tb_clk, reset;
    logic [31:0] tb_WB_Data;
    logic [4:0] reg_num;
    logic [31:0] reg_data;
    logic reg_write_sig;
    logic wr, rd;
    logic [8:0] addr;
    logic [31:0] wr_data;
    logic [31:0] rd_data;
    
    localparam CLKPERIOD = 10;
    localparam CLKDELAY = CLKPERIOD / 2;
    
    riscv riscV (
        .clk(tb_clk),
        .reset(reset),
        .WB_Data(tb_WB_Data),
        .reg_num(reg_num),
        .reg_data(reg_data),
        .reg_write_sig(reg_write_sig),
        .wr(wr),
        .rd(rd),
        .addr(addr),
        .wr_data(wr_data),
        .rd_data(rd_data)
    );
    
    initial begin
        tb_clk = 0;
        reset = 1;
        $display("========================================");
        $display("    RISC-V PROCESSOR TESTBENCH START   ");
        $display("========================================");
        
        #(CLKPERIOD);
        reset = 0;
        $display("Time %0t: RESET released - Execution started", $time);
        $display("----------------------------------------");
        
        #(CLKPERIOD * 100); // Mais tempo para ver todas as operações
        
        $display("========================================");
        $display("    SIMULATION COMPLETED - Time: %0t", $time);
        $display("========================================");
        $stop;
    end
    
    // Monitor COMPLETO - captura TODAS as operações
    always @(posedge tb_clk) begin
        if (!reset) begin
            // Monitor de registrador - captura QUALQUER write
            if (reg_write_sig && reg_num != 0) begin // x0 sempre é 0
                $display("Time %0t: REG[x%02d] <= 0x%08X (%10d) [BIN: %032b]", 
                        $time, reg_num, reg_data, reg_data, reg_data);
            end
            
            // Monitor de memória - captura QUALQUER acesso
            if (wr) begin
                $display("Time %0t: MEM[%03d] <= 0x%08X (%10d) [WRITE]", 
                        $time, addr, wr_data, wr_data);
            end
            
            if (rd) begin
                $display("Time %0t: MEM[%03d] => 0x%08X (%10d) [READ]", 
                        $time, addr, rd_data, rd_data);
            end
        end
    end
    
    // Monitor alternativo - captura mudanças em QUALQUER momento
    always @(*) begin
        if (!reset) begin
            // Detecta qualquer mudança nos sinais de controle
            if (reg_write_sig && reg_num != 0) begin
                $display("Time %0t: [ALT] REG[x%02d] = 0x%08X (%10d)", 
                        $time, reg_num, reg_data, reg_data);
            end
        end
    end
    
    //clock generator
    always #(CLKDELAY) tb_clk = ~tb_clk;
    
endmodule