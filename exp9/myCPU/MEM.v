module MEM(
    input [31:0]    clk,
    input [31:0]    reset,
    //from EXE
    input           ready_go_exe,
    output          allow_in,
    input [31:0]    inst_from_exe,
    input [31:0]    pc_from_exe,
    input           reg_en_from_exe,
    input           mem_ld_from_exe,
    input [4:0]     dest_from_exe,
    input [31:0]    alu_result_from_exe,
    //to WB
    output          ready_go,
    input           WB_allow_in,
    output [31:0]   inst_mem,
    output [31:0]   pc_mem,
    output [31:0]   data_to_reg,
    output          reg_en,
    output [4:0]    dest,
    //to dram
    input  [31:0] data_sram_rdata,
    //valid
    output  reg       valid,
    //bypass
    output [31:0] forward_data_mem
);  
    //valid Part
    always @(posedge clk) begin
        if (reset) begin
            valid <= 1'b0;
        end
        else if (ready_go_exe &allow_in) begin
            valid <= 1'b1;
        end
        else if (ready_go &WB_allow_in) begin
            valid <= 1'b0;
        end
    end

    reg [31:0] inst_reg;
    reg [31:0] pc_reg;
    reg        reg_en_reg;
    reg        mem_ld_reg;
    reg [4:0]  dest_reg;
    reg [31:0] alu_result_reg;
    always @(posedge clk) begin
        if (reset) begin
            inst_reg       <= 32'b0;
            pc_reg         <= 32'b0;
            reg_en_reg     <= 1'b0;
            mem_ld_reg     <= 1'b0;
            dest_reg       <= 5'b0;
            alu_result_reg <= 32'b0;
        end
        else if (ready_go_exe &allow_in) begin
            inst_reg       <= inst_from_exe;
            pc_reg         <= pc_from_exe;
            reg_en_reg     <= reg_en_from_exe;
            mem_ld_reg     <= mem_ld_from_exe;
            dest_reg       <= dest_from_exe;
            alu_result_reg <= alu_result_from_exe;
        end
    end
    assign reg_en     = reg_en_reg;
    assign dest       = dest_reg;
    assign allow_in   = ~valid|(WB_allow_in&&ready_go);
    assign ready_go   = valid;
    assign inst_mem   = inst_reg;
    assign pc_mem     = pc_reg;
    assign data_to_reg= mem_ld_reg ? data_sram_rdata : alu_result_reg;
    assign forward_data_mem = data_to_reg;
endmodule
