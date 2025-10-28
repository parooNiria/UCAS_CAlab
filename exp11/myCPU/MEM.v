module MEM(
    input [31:0]    clk,
    input [31:0]    reset,
    //from EXE
    input           ready_go_exe,
    output          allow_in,
    input [31:0]    inst_from_exe,
    input [31:0]    pc_from_exe,
    input           reg_en_from_exe,
    input [4:0]     mem_ld_from_exe,
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
    output [31:0] forward_data_mem,
    output        forward_en_mem
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
    reg [4:0]  mem_ld_reg;
    reg [4:0]  dest_reg;
    reg [31:0] alu_result_reg;
    always @(posedge clk) begin
        if (reset) begin
            inst_reg       <= 32'b0;
            pc_reg         <= 32'b0;
            reg_en_reg     <= 1'b0;
            mem_ld_reg     <= 5'b0;
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
    assign data_to_reg= ld_inst ? sram_rdata : alu_result_reg;
    assign forward_data_mem = alu_result_reg;
    assign forward_en_mem   = ~ld_inst;
    wire   ld_inst;
    assign ld_inst = mem_ld_reg != 5'b0;
    wire   ld_b;
    wire   ld_h;
    wire   ld_bu;
    wire   ld_hu;
    wire   ld_w;
    assign ld_bu = mem_ld_reg[0];
    assign ld_hu = mem_ld_reg[1];
    assign ld_b  = mem_ld_reg[2];
    assign ld_h  = mem_ld_reg[3];
    assign ld_w  = mem_ld_reg[4];
    wire [31:0] sram_rdata;
    wire [31:0] data_hu;
    assign data_hu = alu_result_reg[1] ? {16'b0, data_sram_rdata[31:16]} : {16'b0, data_sram_rdata[15:0]};
    wire [31:0] data_bu;
    assign data_bu = alu_result_reg[1:0]==2'b11 ? {24'b0, data_sram_rdata[31:24]} :
                     alu_result_reg[1:0]==2'b10 ? {24'b0, data_sram_rdata[23:16]} :
                     alu_result_reg[1:0]==2'b01 ? {24'b0, data_sram_rdata[15:8]}  :
                                                   {24'b0, data_sram_rdata[7:0]} ;  
    wire [31:0] data_h;
    assign data_h = alu_result_reg[1] ?{{16{data_sram_rdata[31]}},data_sram_rdata[31:16]} :
                                        {{16{data_sram_rdata[15]}},data_sram_rdata[15:0]} ;
    wire [31:0] data_b;
    assign data_b = alu_result_reg[1:0]==2'b11 ? {{24{data_sram_rdata[31]}},data_sram_rdata[31:24]} :
                    alu_result_reg[1:0]==2'b10 ? {{24{data_sram_rdata[23]}},data_sram_rdata[23:16]} :
                    alu_result_reg[1:0]==2'b01 ? {{24{data_sram_rdata[15]}},data_sram_rdata[15:8]}  :
                                                 {{24{data_sram_rdata[7]}}, data_sram_rdata[7:0]} ;
    assign sram_rdata = ld_w ? data_sram_rdata :
                        ld_hu  ? data_hu :
                        ld_bu  ? data_bu :
                        ld_h   ? data_h :
                        ld_b   ? data_b :
                                  32'b0;
endmodule
