module mycpu_top(
    input  wire        clk,
    input  wire        resetn,
    // inst sram interface
    output wire        inst_sram_en,
    output wire [3:0]  inst_sram_we,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,
    // data sram interface
    output wire        data_sram_en,
    output wire [3:0]  data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire [31:0] data_sram_rdata,
    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);
wire         reset;
assign reset = ~resetn;
wire flush;
wire [31:0] inst_if;
wire [31:0] pc_if;
wire ready_go_if;
IF IF_PART(
    .clk            (clk            ),
    .reset          (reset          ),
    // inst sram interface
    .inst_sram_en   (inst_sram_en   ),
    .inst_sram_wen  (inst_sram_we   ),
    .inst_sram_addr (inst_sram_addr ),
    .inst_sram_wdata(inst_sram_wdata),
    .inst_sram_rdata(inst_sram_rdata), 

    //to ID
    .ready_go       (ready_go_if   ),
    .inst_if        (inst_if       ),
    .pc_if          (pc_if         ),
    .allow_in       (allow_in_id  ),

    .flush          (flush         ),
    .newpc          (newpc    )               
);

wire        allow_in_id;
wire [31:0] newpc;
wire ready_go_id;
wire [31:0] inst_id;
wire [31:0] pc_id;
wire [31:0] src1_to_exe;
wire [31:0] src2_to_exe;
wire [11:0] alu_op_to_exe;
wire [4:0]  dest_to_exe;
wire        reg_en_to_exe;
wire [1:0]  mem_en_to_exe;
wire [31:0] rdata1;
wire [31:0] rdata2;
wire [4:0]  raddr1;
wire [4:0]  raddr2;
wire [31:0] rdata2_to_exe;
wire [31:0] rdata1_to_exe;
ID ID_PART(
    .clk            (clk            ),
    .reset          (reset          ),
    //IF_ID
    .if_ready       (ready_go_if   ),
    .allow_in       (allow_in_id   ),
    .inst_from_if   (inst_if       ),
    .pc_from_if     (pc_if         ),
    .flush          (flush         ),
    .newpc          (newpc         ),
    //to EX
    .ready_go       (ready_go_id   ),
    .EX_allow_in    (allow_in_exe  ),
    .inst_id        (inst_id       ),
    .pc_id          (pc_id         ),
    .alu_op_id      (alu_op_to_exe),
    .dest           (dest_to_exe   ),
    .src1           (src1_to_exe   ),
    .src2           (src2_to_exe   ),
    .reg_en        (reg_en_to_exe ),
    .mem_en         (mem_en_to_exe ),
    .rdata2_to_exe  (rdata2_to_exe ), 
    .rdata1_to_exe  (rdata1_to_exe),    
    //to regfile
    .raddr1         (raddr1        ),
    .rdata1         (rdata1        ),
    .raddr2         (raddr2        ),
    .rdata2         (rdata2        ),
    //data conflict
    .ex_reg_en_valid(ex_reg_en_valid),
    .mem_reg_en_valid(mem_reg_en_valid),
    .wb_reg_en_valid(wb_reg_en_valid),
    .ex_dest        (dest_to_mem       ),
    .mem_dest       (dest_to_wb      ),
    .wb_dest        (dest           ),
    //bypass
    .forward_data_from_exe(forward_data_exe),
    .forward_data_from_mem(forward_data_mem),
    .forward_data_from_wb(forward_data_wb),
    .forward_en_from_exe(forward_en_from_exe)
);

wire        allow_in_exe;
wire        ready_go_exe;
wire [31:0] inst_exe;
wire [31:0] pc_exe;
wire [31:0] alu_result_exe;
wire        reg_en_to_mem;
wire        mem_ld_exe;
wire [4:0]  dest_to_mem;
wire        exe_valid;
wire        ex_reg_en_valid;
assign      ex_reg_en_valid = exe_valid & reg_en_to_mem;
wire [31:0] forward_data_exe;
wire        forward_en_from_exe;
EXE EXE_PART(
    .clk            (clk            ),
    .reset          (reset          ),
    //from ID
    .ready_go_id    (ready_go_id   ),
    .allow_in       (allow_in_exe  ),
    .inst_from_id   (inst_id       ),
    .pc_from_id     (pc_id         ),
    .src1_from_id   (src1_to_exe   ),
    .src2_from_id   (src2_to_exe   ),
    .alu_op_from_id (alu_op_to_exe),
    .dest_from_id   (dest_to_exe   ),
    .reg_en_from_id (reg_en_to_exe),
    .mem_en_from_id (mem_en_to_exe),
    .rdata2_from_id (rdata2_to_exe ),
    .rdata1_from_id (rdata1_to_exe ),
    //to MEM
    .ready_go       (ready_go_exe  ),
    .MEM_allow_in   (allow_in_mem  ),
    .inst_exe       (inst_exe      ),
    .pc_exe         (pc_exe        ),
    .alu_result     (alu_result_exe),
    .reg_en        (reg_en_to_mem ),
    .mem_ld         (mem_ld_exe    ),
    .dest           (dest_to_mem   ),
    //to dram
    .data_sram_en   (data_sram_en  ),
    .data_sram_we   (data_sram_we  ),
    .data_sram_addr (data_sram_addr),
    .data_sram_wdata(data_sram_wdata),
    //valid
    .valid         (exe_valid     ),
    //bypass
    .forward_data_exe(forward_data_exe),
    .forward_en_exe(forward_en_from_exe)
);

wire ready_go_mem;
wire        allow_in_mem;
wire [31:0] inst_mem;
wire [31:0] pc_mem;
wire [31:0] data_to_reg_mem;
wire        reg_en_to_wb;
wire [4:0]  dest_to_wb;
wire        MEM_valid;
wire        mem_reg_en_valid;
assign mem_reg_en_valid = MEM_valid & reg_en_to_wb;
wire [31:0] forward_data_mem;
MEM MEM_PART(
    .clk            (clk            ),
    .reset          (reset          ),
    //from EXE
    .ready_go_exe  (ready_go_exe   ),
    .allow_in       (allow_in_mem   ),
    .inst_from_exe (inst_exe       ),
    .pc_from_exe   (pc_exe         ),
    .reg_en_from_exe(reg_en_to_mem ),
    .mem_ld_from_exe(mem_ld_exe    ),
    .dest_from_exe (dest_to_mem    ),
    .alu_result_from_exe(alu_result_exe),
    //to WB
    .ready_go       (ready_go_mem   ),
    .WB_allow_in    (allow_in_wb    ),
    .inst_mem       (inst_mem       ),
    .pc_mem         (pc_mem         ),
    .data_to_reg    (data_to_reg_mem),
    .reg_en        (reg_en_to_wb   ),
    .dest           (dest_to_wb     ),
    //to dram
    .data_sram_rdata(data_sram_rdata),
    //valid
    .valid         (MEM_valid     ),
    //bypass
    .forward_data_mem(forward_data_mem)
);

wire allow_in_wb;
wire [31:0] inst_wb;
wire [31:0] pc;
wire [31:0] wdata;
wire        rf_we;
wire [4:0]  dest;
wire        WB_valid;
wire        wb_reg_en_valid;
assign wb_reg_en_valid = WB_valid & rf_we;
wire [31:0] forward_data_wb;
WB WB_PART(
    .clk            (clk            ),
    .reset          (reset          ),
    //from MEM
    .ready_go_mem   (ready_go_mem   ),
    .allow_in       (allow_in_wb    ),
    .inst_from_mem  (inst_mem       ),
    .pc_from_mem    (pc_mem         ),
    .data_to_reg_from_mem(data_to_reg_mem),
    .reg_en_from_mem(reg_en_to_wb   ),
    .dest_from_mem  (dest_to_wb     ),
    //to regfile
    .waddr          (dest           ),
    .wdata          (wdata          ),
    .we             (rf_we          ),
    //to trace
    .inst           (inst_wb        ),
    .pc             (pc             ),
    .valid         (WB_valid      ),
    //bypass
    .forward_data_wb(forward_data_wb)
);

regfile regfile_PART(
    .clk    (clk      ),
    .raddr1 (raddr1   ),
    .rdata1 (rdata1   ),
    .raddr2 (raddr2   ),
    .rdata2 (rdata2   ),
    .we     (rf_we    ),
    .waddr  (dest     ),
    .wdata  (wdata    )
);
// debug info generate
assign debug_wb_pc       = pc;
assign debug_wb_rf_we   = {4{rf_we}};
assign debug_wb_rf_wnum  = dest;
assign debug_wb_rf_wdata = wdata;

endmodule
