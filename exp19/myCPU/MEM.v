module MEM(
    input [31:0]    clk,
    input [31:0]    reset,
    //from EXE
    input           ready_go_exe,
    output          allow_in,
    input [31:0]    inst_from_exe,
    input [31:0]    pc_from_exe,
    input           reg_en_from_exe,
    input [6:0]     mem_from_exe,
    input [4:0]     dest_from_exe,
    input [31:0]    alu_result_from_exe,
    input [32:0]    timer_cnt_global_value_and_en,
    //to WB
    output          ready_go,
    input           WB_allow_in,
    output [31:0]   inst_mem,
    output [31:0]   pc_mem,
    output [31:0]   data_to_reg,
    output          reg_en,
    output [4:0]    dest,
    output [31:0]   dram_addr,
    //to dram
    input  [31:0] data_sram_rdata,
    input       data_sram_data_ok,
    //valid
    output  reg       valid,
    //bypass
    output [31:0] forward_data_mem,
    output        forward_en_mem,
    //exception related
    input  [87:0] exception_message_from_exe,
    output [87:0] exception_message_to_wb,
    input         ertn_flush,
    input         wb_ex,
    output  [1:0] exception_message_to_exe,
    //tlb related
    input        vaddr_sign_from_wb,
    output       vaddr_sign,
    input  [8:0] exception_state_tlb_exe,
    input        vaddr_about_inst_happen_in_wb,
    input  [4:0] tlb_related_inst_exe,

    output [8:0] exception_state_tlb,
    output [4:0] tlb_related_inst
);  
    //valid Part
    always @(posedge clk) begin
        if (reset) begin
            valid <= 1'b0;
        end
        else if(wb_ex || ertn_flush) begin
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
    reg [6:0]  mem_message_reg;
    reg [4:0]  dest_reg;
    reg [31:0] alu_result_reg;
    reg [87:0] exception_message_reg;
    reg [32:0] timer_cnt_global_value_and_en_reg;
    reg [8:0]  exception_state_tlb_reg;
    reg [4:0]  tlb_related_inst_reg;
    always @(posedge clk) begin
        if (reset) begin
            inst_reg       <= 32'b0;
            pc_reg         <= 32'b0;
            reg_en_reg     <= 1'b0;
            mem_message_reg    <= 7'b0;
            dest_reg       <= 5'b0;
            alu_result_reg <= 32'b0;
            exception_message_reg <= 87'b0;
            timer_cnt_global_value_and_en_reg <= 33'b0;
            exception_state_tlb_reg <= 9'b0;
            tlb_related_inst_reg <= 5'b0;
        end
        else if (ready_go_exe &allow_in) begin
            inst_reg       <= inst_from_exe;
            pc_reg         <= pc_from_exe;
            reg_en_reg     <= reg_en_from_exe;
            mem_message_reg     <= mem_from_exe;
            dest_reg       <= dest_from_exe;
            alu_result_reg <= alu_result_from_exe;
            exception_message_reg <= exception_message_from_exe;
            timer_cnt_global_value_and_en_reg <= timer_cnt_global_value_and_en;
            exception_state_tlb_reg <= exception_state_tlb_exe;
            tlb_related_inst_reg <= tlb_related_inst_exe;
        end
    end
    wire inst_rdcntv;
    assign inst_rdcntv = timer_cnt_global_value_and_en_reg[0];
    wire [31:0] timer_cnt_global_value;
    assign timer_cnt_global_value = timer_cnt_global_value_and_en_reg[32:1];
    assign reg_en     = reg_en_reg;
    assign dest       = dest_reg;
    assign allow_in   = ~valid|(WB_allow_in&&ready_go);
    assign ready_go   = valid&((~ld_inst&~st_inst)|data_sram_data_ok|(exception_state)|vaddr_sign | vaddr_sign_from_wb);
    assign inst_mem   = inst_reg;
    assign pc_mem     = pc_reg;
    assign data_to_reg= ld_inst ? sram_rdata : alu_result_reg;
    assign forward_data_mem = inst_rdcntv?timer_cnt_global_value:alu_result_reg;
    assign forward_en_mem   = ~ld_inst & ~csr_re;

//to dram
    wire   ld_inst;
    assign ld_inst = mem_message_reg[6];
    wire   st_inst;
    assign st_inst = mem_message_reg[5];
    wire   mem_type;
    assign mem_type = ld_inst | st_inst;
    wire   ld_b;
    wire   ld_h;
    wire   ld_bu;
    wire   ld_hu;
    wire   ld_w;
    assign ld_bu = mem_message_reg[0];
    assign ld_hu = mem_message_reg[1];
    assign ld_b  = mem_message_reg[2];
    assign ld_h  = mem_message_reg[3];
    assign ld_w  = mem_message_reg[4];
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
    assign dram_addr = alu_result_reg;

//exception related signals
    wire exception_state;
    assign exception_state = exception_message_reg[87];

    wire csr_re;
    assign csr_re = exception_message_reg[79];
    assign exception_message_to_wb = exception_message_reg;

    wire inst_ertn;
    assign inst_ertn = exception_message_reg[81];
    assign exception_message_to_exe = {inst_ertn&valid, exception_state&valid};
    
    assign vaddr_sign = valid & exception_state_tlb_reg[0];
    assign exception_state_tlb = valid ? exception_state_tlb_reg : 10'b0;
    assign tlb_related_inst = valid ? tlb_related_inst_reg : 5'b0;
endmodule
