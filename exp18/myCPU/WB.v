`define ECODE_INT 6'h00
`define ECODE_ADE 6'h08  
`define ECODE_ALE 6'h09   
`define ECODE_SYS 6'h0B
`define ECODE_BRK 6'h0C   
`define ECODE_INE 6'h0D
`define ECODE_TLBR 6'h3F
`define ECODE_PIL 6'h01  // LOAD页无效例外
`define ECODE_PIS 6'h02  // STORE页无效例外
`define ECODE_PIF 6'h03  // FETCH页无效例外
`define ECODE_PME 6'h04  // 页修改例外
`define ECODE_PPI 6'h07  // 页特权等级不合规例外

`define ESUBCODE_ADEF 9'b00    
`define ESUBCODE_ADEM 9'b01     

module WB(
    input          clk,
    input          reset,
    //from MEM
    input          ready_go_mem,
    output         allow_in,
    input  [31:0]  inst_from_mem,
    input  [31:0]  pc_from_mem,
    input  [31:0]  data_to_reg_from_mem,
    input          reg_en_from_mem,
    input  [4:0]   dest_from_mem,
    input  [31:0]  dram_addr,
    //to regfile
    output [4:0]   waddr,
    output [31:0]  wdata,
    output         we,
    //to trace
    output reg [31:0]  inst,
    output reg [31:0]  pc,  
    //valid
    output reg       valid,
    //bypass
    output [31:0] forward_data_wb,
    //exception related
    input  [87:0] exception_message_from_mem,
    output        ertn_flush,
    output        wb_ex,
    output        csr_we,
    output [13:0] csr_num,
    output [31:0] csr_wdata,
    output [31:0] csr_wmask,
    output        csr_re,
    input  [31:0] csr_rdata,
    output [5:0]  wb_ecode,
    output [8:0]  wb_esubcode,
    output [31:0] wb_pc,
    output [31:0] wb_vaddr,
    //tlb related
    input  [8:0] exception_state_tlb_mem,
    input  [5:0] tlb_related_inst_mem,
    output       inst_tlbwr_happen,
    output       inst_tlbrd_happen,
    output       inst_tlbfill_happen,
    output       wrong_addr_is_pc,
    output       vaddr_sign,
    output       vaddr_about_inst_happen_in_wb,
    output [31:0] vaddr_about_inst_pc,
    input  [4:0]  tlbsrch_csr_message_from_mem,
    output [3:0]  tlbsrch_index,
    output        tlbsrch_hit,
    output        inst_tlbsrch_in_wb_happen,
    output        stall_tlbsrch_wb
);  
    //valid part
    always@(posedge clk) begin
        if (reset) begin
            valid <= 1'b0;
        end
        else if (ready_go_mem &allow_in) begin
            valid <= 1'b1;
        end
        else 
            valid <= 1'b0;
    end
    //Save reg
    reg [31:0] data_reg;
    reg [4:0]  dest_reg;
    reg      reg_en_reg;
    reg [87:0] exception_message_reg;
    reg [31:0] vaddr_reg;
    reg [8:0]  exception_state_tlb_reg;
    reg [5:0]  tlb_related_inst_reg;
    reg [4:0]  tlbsrch_csr_message_reg;
    always @(posedge clk) begin
        if (reset) begin
            data_reg   <= 32'b0;
            dest_reg   <= 5'b0;
            reg_en_reg <= 1'b0;
            exception_message_reg <= 83'b0;
            vaddr_reg  <= 32'b0;
            exception_state_tlb_reg <= 9'b0;
            tlb_related_inst_reg <= 6'b0;
            tlbsrch_csr_message_reg <= 5'b0;
        end
        else if (ready_go_mem &allow_in) begin
            data_reg   <= data_to_reg_from_mem;
            dest_reg   <= dest_from_mem;
            reg_en_reg <= reg_en_from_mem;
            exception_message_reg <= exception_message_from_mem;
            vaddr_reg  <= dram_addr;
            exception_state_tlb_reg <= exception_state_tlb_mem;
            tlb_related_inst_reg <= tlb_related_inst_mem;
            tlbsrch_csr_message_reg <= tlbsrch_csr_message_from_mem;
        end
    end

    assign allow_in = ((~wb_ex & ~ertn_flush)&valid) | (~valid);
    assign we       = valid & reg_en_reg &~exception_state;
    assign waddr    = dest_reg;
    assign wdata    = csr_re? csr_rdata : data_reg;
    always @(posedge clk) begin
        if (reset) begin
            inst <= 32'b0;
            pc   <= 32'h1bfffffc;
        end
        else if (ready_go_mem & allow_in) begin
            inst <= inst_from_mem;
            pc   <= pc_from_mem;
        end
    end
    assign forward_data_wb = wdata; 
//exception related signals
    wire   inst_ertn;
    wire   inst_syscall;
    wire exception_state;
    assign exception_state = exception_message_reg[87];
    wire exception_int;
    wire exception_adef;
    wire exception_ine;
    wire exception_ale;
    wire inst_break;
    wire exception_tlbr_FETCH;
    wire exception_pif;
    wire exception_ppi_fetch;
    wire exception_tlbr_DATA;
    wire exception_PIL;
    wire exception_PIS;
    wire exception_PME;
    wire exception_PPI_DATA;
    assign exception_tlbr_FETCH = exception_state_tlb_reg[8];
    assign exception_pif        = exception_state_tlb_reg[7];
    assign exception_ppi_fetch  = exception_state_tlb_reg[6];
    assign exception_tlbr_DATA  = exception_state_tlb_reg[5];
    assign exception_PIL       = exception_state_tlb_reg[4];
    assign exception_PIS       = exception_state_tlb_reg[3];
    assign exception_PME       = exception_state_tlb_reg[2];
    assign exception_PPI_DATA  = exception_state_tlb_reg[1];
    assign vaddr_sign = exception_state_tlb_reg[0]& valid;

    wire  inst_tlbwr;
    wire  inst_tlbrd;
    wire  inst_tlbfill;

    assign inst_tlbwr = tlb_related_inst_reg[2];
    assign inst_tlbrd = tlb_related_inst_reg[3];
    assign inst_tlbfill = tlb_related_inst_reg[1];

    assign inst_tlbwr_happen  = inst_tlbwr & valid & ~wb_ex & ~ertn_flush & ~exception_state;
    assign inst_tlbrd_happen  = inst_tlbrd & valid & ~wb_ex & ~ertn_flush & ~exception_state;
    assign inst_tlbfill_happen= inst_tlbfill & valid & ~wb_ex & ~ertn_flush & ~exception_state;
    assign wrong_addr_is_pc   = exception_tlbr_FETCH | exception_pif | exception_ppi_fetch | exception_adef;
    assign vaddr_about_inst_happen_in_wb = vaddr_sign;
    assign exception_int  = exception_message_reg[86];
    assign exception_adef = exception_message_reg[85];
    assign exception_ine  = exception_message_reg[84];
    assign exception_ale  = exception_message_reg[83];
    assign inst_break    = exception_message_reg[82];
    assign inst_ertn    = exception_message_reg[81];
    assign inst_syscall = exception_message_reg[80];
    assign csr_re    = valid&exception_message_reg[79];
    assign csr_we     = valid&exception_message_reg[78];
    assign csr_wmask  = exception_message_reg[77:46];
    assign csr_num = exception_message_reg[45:32];
    assign csr_wdata  = exception_message_reg[31:0];
    assign ertn_flush = inst_ertn & valid;
    assign wb_ex      = exception_state & valid;
    assign wb_pc      = pc;
    assign wb_ecode   = exception_int ? 6'h00 :
                        exception_adef ? 6'h08 :
                        exception_ine  ? 6'h0d :
                        exception_ale  ? 6'h09 :
                        inst_break     ? 6'h0c :
                        inst_syscall   ? 6'h0b :
                        exception_tlbr_FETCH | exception_tlbr_DATA ? 6'h3f :
                        exception_pif ? `ECODE_PIF :
                        exception_ppi_fetch | exception_PPI_DATA ? `ECODE_PPI :
                        exception_PIL ? `ECODE_PIL :
                        exception_PIS ? `ECODE_PIS :
                        exception_PME ? `ECODE_PME :
                        6'h0;
    assign wb_esubcode= 9'h0;
    assign wb_vaddr   = vaddr_reg;
    assign vaddr_about_inst_pc = pc;
    assign tlbsrch_index = tlbsrch_csr_message_reg[3:0];
    assign tlbsrch_hit   = tlbsrch_csr_message_reg[4];
   
    assign stall_tlbsrch_wb = tlb_related_inst_reg[5] & valid;
    assign inst_tlbsrch_in_wb_happen = tlb_related_inst_reg[4] & valid & ~wb_ex & ~ertn_flush & ~exception_state & ~wb_vaddr;
endmodule