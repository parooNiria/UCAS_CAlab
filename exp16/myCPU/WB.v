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
    output [31:0] wb_vaddr
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
    always @(posedge clk) begin
        if (reset) begin
            data_reg   <= 32'b0;
            dest_reg   <= 5'b0;
            reg_en_reg <= 1'b0;
            exception_message_reg <= 83'b0;
            vaddr_reg  <= 32'b0;
        end
        else if (ready_go_mem &allow_in) begin
            data_reg   <= data_to_reg_from_mem;
            dest_reg   <= dest_from_mem;
            reg_en_reg <= reg_en_from_mem;
            exception_message_reg <= exception_message_from_mem;
            vaddr_reg  <= dram_addr;
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
                        6'h0;
    assign wb_esubcode= 9'h0;
    assign wb_vaddr   = vaddr_reg;
endmodule