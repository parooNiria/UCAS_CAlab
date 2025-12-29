module mycpu_core(
    input  wire        clk,
    input  wire        resetn,

    output               	icache_rd_req,
    output   	[ 2:0]      icache_rd_type,
    output   	[31:0]      icache_rd_addr,
    input              	    icache_rd_rdy,		
    input              	    icache_ret_valid,	
	input					icache_ret_last,
    input  	[31:0]          icache_ret_data,
    // dcache rd interface
	output               	dcache_rd_req,
    output   	[ 2:0]      dcache_rd_type,
    output   	[31:0]      dcache_rd_addr,
    input              	    dcache_rd_rdy,		
    input              	    dcache_ret_valid,	
	input					dcache_ret_last,
    input  	[31:0]          dcache_ret_data,
	// dcache wr interface
	output               	dcache_wr_req,
    output   	[ 2:0]      dcache_wr_type,
    output   	[31:0]      dcache_wr_addr,
    output   	[ 3:0]      dcache_wr_wstrb,
	output	   [127:0]		dcache_wr_data,
	input					dcache_wr_rdy,
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
wire exception_adef;
wire exception_pif_if;
wire exception_tlbr_fetch_if;
wire exception_ppi_if;
wire vaddr_sign_id;
wire vaddr_sign_exe;
wire vaddr_sign_mem;
wire vaddr_sign_wb;

wire vaddr_about_inst_happen_in_wb;
wire invalidtlb_happen_in_ex;
wire [1:0]  cacop_op;
wire        cacop_req_icache;
wire        cacop_req_dcache;
wire [31:0] cacop_va;
wire        cacop_finish_icache;
wire        cacop_finish_dcache;
wire        cacop_finish_in_exe;
wire        cacop_inst_in_exe;
wire        cacop_inst_in_id;

wire [31:0] invalidtlb_pc;
wire [31:0] vaddr_about_inst_pc;
wire [1:0]  datf_csr;
wire [1:0]  datm_csr;
wire [1:0]  csr_dmw0_mat_state;
wire [1:0]  csr_dmw1_mat_state;
wire [1:0]  s0_mat;
wire [1:0]  s1_mat;
wire [1:0]  mem_type_if;
wire [1:0]  mem_type_exe;
wire        inst_sram_req;
wire        inst_sram_wr;
wire [1:0]  inst_sram_size;
wire [3:0]  inst_sram_wstrb;
wire [31:0] inst_sram_addr;
wire [31:0] inst_sram_wdata;
wire        inst_sram_addr_ok;
wire        inst_sram_data_ok;
wire [31:0] inst_sram_rdata;
wire        data_sram_req;
wire        data_sram_wr;
wire [1:0]  data_sram_size;
wire [3:0]  data_sram_wstrb;
wire [31:0] data_sram_addr;
wire [31:0] data_sram_wdata;
wire        data_sram_addr_ok;
wire        data_sram_data_ok;
wire [11:0] inst_addr_vrtl;
wire [11:0] data_addr_vrtl;
wire [31:0] data_sram_rdata;
IF IF_PART(
    .clk            (clk            ),
    .reset          (reset          ),
    // inst sram interface
    .inst_sram_req   (inst_sram_req   ),
    .inst_sram_wr    (inst_sram_wr    ),
    .inst_sram_addr  (inst_sram_addr  ),
    .inst_sram_wdata (inst_sram_wdata ),
    .inst_sram_rdata (inst_sram_rdata ),
    .inst_sram_addr_ok(inst_sram_addr_ok),
    .inst_sram_data_ok(inst_sram_data_ok),
    .inst_sram_wstrb(inst_sram_wstrb),
    .inst_sram_size (inst_sram_size ),
    .inst_addr_vrtl(inst_addr_vrtl),
    .mem_type_if  (mem_type_if  ),

    //to ID
    .ready_go       (ready_go_if   ),
    .inst_if        (inst_if       ),
    .pc_if          (pc_if         ),
    .allow_in_id    (allow_in_id   ),
    .exception_adef (exception_adef ),
    .flush          (flush         ),
    .newpc          (newpc          ),
    .ertn_flush     (ertn_flush     ),
    .wb_ex          (wb_ex          ),
    .ex_entry       (ex_entry       ),
    .ertn_entry     (ertn_entry     ),
    .exception_pif  (exception_pif_if),
    .exception_tlbr_fetch(exception_tlbr_fetch_if),
    .exception_ppi  (exception_ppi_if),

    .s0_vppn    (s0_vppn    ),
    .s0_va_bit12(s0_va_bit12),
    .s0_found   (s0_found   ),
    .s0_index   (s0_index   ),
    .s0_ppn     (s0_ppn     ),
    .s0_ps      (s0_ps      ),
    .s0_plv     (s0_plv     ),
    .s0_v       (s0_v       ),
    .s0_mat     (s0_mat     ),
    .crmd_plv  (crmd_plv_state  ),

    .csr_dmw0_plv0(csr_dmw0_plv0_state),
    .csr_dmw0_plv3(csr_dmw0_plv3_state),
    .csr_dmw0_pseg(csr_dmw0_pseg_state),
    .csr_dmw0_vseg(csr_dmw0_vseg_state),
    .csr_dmw0_mat_state(csr_dmw0_mat_state),

    .csr_dmw1_plv0(csr_dmw1_plv0_state),
    .csr_dmw1_plv3(csr_dmw1_plv3_state),
    .csr_dmw1_pseg(csr_dmw1_pseg_state),
    .csr_dmw1_vseg(csr_dmw1_vseg_state),
    .csr_dmw1_mat_state(csr_dmw1_mat_state),

    .csr_addr_mode(csr_addr_mode),
    .datf_csr    (datf_csr    ),

    .vaddr_sign_from_id(vaddr_sign_id),
    .vaddr_sign_from_ex(vaddr_sign_exe),
    .vaddr_sign_from_mem(vaddr_sign_mem),
    .vaddr_sign_from_wb(vaddr_sign_wb),

    .invalidtlb_happen_in_ex(invalidtlb_happen_in_ex),
    .invalidtlb_pc(invalidtlb_pc),
    .vaddr_about_inst_happen_in_wb(vaddr_about_inst_happen_in_wb),
    .vaddr_about_inst_pc(vaddr_about_inst_pc),
    .cacop_finish_in_exe(cacop_finish_in_exe),
    .cacop_inst_in_id(cacop_inst_in_id),
    .cacop_inst_in_exe(cacop_inst_in_exe)
);

wire        allow_in_id;
wire [31:0] newpc;
wire ready_go_id;
wire [31:0] inst_id;
wire [31:0] pc_id;
wire [31:0] src1_to_exe;
wire [31:0] src2_to_exe;
wire [18:0] alu_op_to_exe;
wire [4:0]  dest_to_exe;
wire        reg_en_to_exe;
wire [4:0]  mem_en_to_exe;
wire [31:0] rdata1;
wire [31:0] rdata2;
wire [4:0]  raddr1;
wire [4:0]  raddr2;
wire [31:0] rdata2_to_exe;
wire [31:0] rdata1_to_exe;
wire        div_en_to_exe;
wire [1:0]  inst_rdcntv;
wire [3:0]  exception_state_tlb_id;
wire [5:0]  tlb_related_inst_id;

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
    .exception_adef_if (exception_adef ),
    .exception_tlbr_fetch_if(exception_tlbr_fetch_if),
    .exception_ppi_if(exception_ppi_if),
    .exception_pif_if(exception_pif_if),
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
    .div_en        (div_en_to_exe ),
    .mem_en         (mem_en_to_exe ),
    .rdata2_to_exe  (rdata2_to_exe ), 
    .rdata1_to_exe  (rdata1_to_exe),
    .inst_rdcntv    (inst_rdcntv  ),
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
    .forward_en_from_exe(forward_en_from_exe),
    .forward_en_from_mem(forward_en_mem),
    //exception related
    .exception_message(exception_message_from_id),
    .ertn_flush     (ertn_flush     ),
    .wb_ex          (wb_ex          ),
    .has_int       (has_int       ),

    .vaddr_sign    (vaddr_sign_id),
    .exception_state_tlb(exception_state_tlb_id),
    .tlb_related_inst(tlb_related_inst_id),
    .invalidtlb_happen_in_ex(invalidtlb_happen_in_ex),
    .vaddr_about_inst_happen_in_wb(vaddr_about_inst_happen_in_wb),
    .cacop_finish_in_exe(cacop_finish_in_exe),
    .cacop_inst_in_id(cacop_inst_in_id)
);

wire        allow_in_exe;
wire        ready_go_exe;
wire [31:0] inst_exe;
wire [31:0] pc_exe;
wire [31:0] alu_result_exe;
wire        reg_en_to_mem;
wire [6:0]  mem_message;
wire [4:0]  dest_to_mem;
wire        exe_valid;
wire        ex_reg_en_valid;
assign      ex_reg_en_valid = exe_valid & reg_en_to_mem;
wire [31:0] forward_data_exe;
wire        forward_en_from_exe;
wire [87:0] exception_message_from_id;
wire [32:0] timer_cnt_global_value_and_en;
wire [8:0]  exception_state_tlb_exe;
wire [5:0]  tlb_related_inst_exe;
wire [4:0]  tlbsrch_csr_message_exe;
wire        stall_tlbsrch_mem;
wire        stall_tlbsrch_wb;
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
    .div_en_from_id (div_en_to_exe ),
    .mem_en_from_id (mem_en_to_exe),
    .rdata2_from_id (rdata2_to_exe ),
    .rdata1_from_id (rdata1_to_exe ),
    .inst_rdcntv    (inst_rdcntv   ),
    //to MEM
    .ready_go       (ready_go_exe  ),
    .MEM_allow_in   (allow_in_mem  ),
    .inst_exe       (inst_exe      ),
    .pc_exe         (pc_exe        ),
    .alu_result     (alu_result_exe),
    .reg_en        (reg_en_to_mem ),
    .mem_message         (mem_message   ),
    .dest           (dest_to_mem   ),
    .timer_cnt_global_value_and_en(timer_cnt_global_value_and_en),
    //to dram
    .data_sram_req   (data_sram_req  ),
    .data_sram_wr   (data_sram_wr  ),
    .data_sram_size (data_sram_size),
    .data_sram_wstrb(data_sram_wstrb),
    .data_sram_addr (data_sram_addr),
    .data_sram_addr_ok(data_sram_addr_ok),
    .data_sram_wdata(data_sram_wdata),
    .mem_type_exe (mem_type_exe),
    .data_addr_vrtl(data_addr_vrtl),
    //valid
    .valid         (exe_valid     ),
    //bypass
    .forward_data_exe(forward_data_exe),
    .forward_en_exe(forward_en_from_exe),
    //exception related
    .exception_message_from_id(exception_message_from_id),
    .exception_message_to_mem(exception_message_to_mem),
    .ertn_flush     (ertn_flush     ),
    .wb_ex          (wb_ex          ),
    .exception_message_from_mem(exception_message_from_mem),

    .exception_state_tlb_id(exception_state_tlb_id),
    .tlb_related_inst_id(tlb_related_inst_id),
    .vaddr_sign (vaddr_sign_exe),
    .vaddr_sign_from_mem(vaddr_sign_mem),
    .vaddr_sign_from_wb(vaddr_sign_wb),
    .exception_state_tlb_exe(exception_state_tlb_exe),
    .tlb_related_inst(tlb_related_inst_exe),
    .stall_srch_from_mem(stall_tlbsrch_mem),
    .stall_srch_from_wb(stall_tlbsrch_wb),
    .invalidtlb_happen_in_ex(invalidtlb_happen_in_ex),
    .invalidtlb_pc(invalidtlb_pc),
    .vaddr_about_inst_happen_in_wb(vaddr_about_inst_happen_in_wb),

    .invtlb_op (invtlb_op ),
    .inst_invtlb_happen(invtlb_happen),
    .inst_tlbsrch_happen(tlbsrch_happen),
    .s1_vppn    (s1_vppn    ),
    .s1_va_bit12(s1_va_bit12),
    .s1_asid    (s1_asid    ),
    .s1_found   (s1_found   ),
    .s1_index   (s1_index   ),
    .s1_ppn     (s1_ppn     ),
    .s1_ps      (s1_ps      ),
    .s1_plv     (s1_plv     ),
    .s1_d       (s1_d       ),
    .s1_v       (s1_v       ),
    .s1_mat     (s1_mat     ),

    .csr_dmw0_plv0(csr_dmw0_plv0_state),
    .csr_dmw0_plv3(csr_dmw0_plv3_state),
    .csr_dmw0_pseg(csr_dmw0_pseg_state),
    .csr_dmw0_vseg(csr_dmw0_vseg_state),
    .csr_dmw0_mat_state(csr_dmw0_mat_state),

    .csr_dmw1_plv0(csr_dmw1_plv0_state),
    .csr_dmw1_plv3(csr_dmw1_plv3_state),
    .csr_dmw1_pseg(csr_dmw1_pseg_state),
    .csr_dmw1_vseg(csr_dmw1_vseg_state),
    .csr_dmw1_mat_state(csr_dmw1_mat_state),
    .csr_addr_mode(csr_addr_mode),
    .crmd_plv  (crmd_plv_state  ),
    .csr_asid  (csr_asid_state  ),
    .tlbehi_vppn (tlbehi_vppn),
    .tlbsrch_csr_message(tlbsrch_csr_message_exe),
    .datm_csr    (datm_csr    ),
    .cacop_op  (cacop_op  ),
    .cacop_req_icache(cacop_req_icache),
    .cacop_req_dcache(cacop_req_dcache),
    .cacop_va  (cacop_va),
    .cacop_finish_dcache(cacop_finish_dcache),
    .cacop_finish_icache(cacop_finish_icache),
    .cacop_inst_in_exe(cacop_inst_in_exe),
    .cacop_finish_in_exe(cacop_finish_in_exe),
    .cacop_inst_in_id(cacop_inst_in_id)
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
wire forward_en_mem;
wire [87:0] exception_message_to_mem;
wire [1:0] exception_message_from_mem; 
wire [31:0] dram_addr;
wire [8:0] exception_state_tlb_mem;
wire [5:0] tlb_related_inst_mem;
wire [4:0] tlbsrch_csr_message_to_wb;
MEM MEM_PART(
    .clk            (clk            ),
    .reset          (reset          ),
    //from EXE
    .ready_go_exe  (ready_go_exe   ),
    .allow_in       (allow_in_mem   ),
    .inst_from_exe (inst_exe       ),
    .pc_from_exe   (pc_exe         ),
    .reg_en_from_exe(reg_en_to_mem ),
    .mem_from_exe(mem_message),
    .dest_from_exe (dest_to_mem    ),
    .alu_result_from_exe(alu_result_exe),
    .timer_cnt_global_value_and_en(timer_cnt_global_value_and_en),
    //to WB
    .ready_go       (ready_go_mem   ),
    .WB_allow_in    (allow_in_wb    ),
    .inst_mem       (inst_mem       ),
    .pc_mem         (pc_mem         ),
    .data_to_reg    (data_to_reg_mem),
    .reg_en        (reg_en_to_wb   ),
    .dest           (dest_to_wb     ),
    .dram_addr      (dram_addr      ),
    //to dram
    .data_sram_rdata(data_sram_rdata),
    .data_sram_data_ok(data_sram_data_ok),
    //valid
    .valid         (MEM_valid     ),
    //bypass
    .forward_data_mem(forward_data_mem),
    .forward_en_mem(forward_en_mem),

    //exception related
    .exception_message_from_exe(exception_message_to_mem),
    .exception_message_to_wb(exception_message_to_wb),
    .ertn_flush     (ertn_flush     ),
    .wb_ex          (wb_ex          ),
    .exception_message_to_exe(exception_message_from_mem),
    .exception_state_tlb(exception_state_tlb_mem),
    .tlb_related_inst(tlb_related_inst_mem),
    .vaddr_sign_from_wb(vaddr_sign_wb),
    .vaddr_sign    (vaddr_sign_mem),
    .exception_state_tlb_exe(exception_state_tlb_exe),
    .vaddr_about_inst_happen_in_wb(vaddr_about_inst_happen_in_wb),
    .tlb_related_inst_exe(tlb_related_inst_exe),
    .stall_tlbsrch_mem(stall_tlbsrch_mem),
    .tlbsrch_csr_message_from_exe(tlbsrch_csr_message_exe),
    .tlbsrch_csr_message_to_wb(tlbsrch_csr_message_to_wb)
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
wire [87:0] exception_message_to_wb;
wire inst_tlbsrch_in_wb_happen;
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
    .dram_addr      (dram_addr      ),
    //to regfile
    .waddr          (dest           ),
    .wdata          (wdata          ),
    .we             (rf_we          ),
    //to trace
    .inst           (inst_wb        ),
    .pc             (pc             ),
    .valid         (WB_valid      ),
    //bypass
    .forward_data_wb(forward_data_wb),
    //exception related
    .ertn_flush     (ertn_flush     ),
    .wb_ex          (wb_ex          ),
    .csr_we         (csr_we         ),
    .csr_num        (csr_num        ),
    .csr_wdata      (csr_wdata      ),
    .csr_wmask      (csr_wmask      ),
    .csr_re         (csr_re         ),
    .csr_rdata      (csr_rvalue     ),
    .wb_ecode       (wb_ecode       ),
    .wb_esubcode    (wb_esubcode    ),
    .wb_pc          (wb_pc          ),
    .exception_message_from_mem(exception_message_to_wb),
    .wb_vaddr       (wb_vaddr       ),

    .exception_state_tlb_mem(exception_state_tlb_mem),
    .tlb_related_inst_mem(tlb_related_inst_mem),
    .inst_tlbwr_happen(tlbwr_happen),
    .inst_tlbrd_happen(tlbrd_happen),
    .inst_tlbfill_happen(tlbfill_happen),
    .wrong_addr_is_pc(wrong_addr_is_pc),
    .vaddr_sign    (vaddr_sign_wb),
    .vaddr_about_inst_happen_in_wb(vaddr_about_inst_happen_in_wb),
    .vaddr_about_inst_pc(vaddr_about_inst_pc),
    .tlbsrch_csr_message_from_mem(tlbsrch_csr_message_to_wb),
    .tlbsrch_index (tlbsrch_index ),
    .tlbsrch_hit   (tlbsrch_hit   ),
    .inst_tlbsrch_in_wb_happen(inst_tlbsrch_in_wb_happen),
    .stall_tlbsrch_wb(stall_tlbsrch_wb)
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

wire csr_re;
wire [13:0] csr_num;
wire [31:0] csr_rvalue;
wire csr_we;
wire [31:0] csr_wdata;
wire [31:0] csr_wmask;
wire wb_ex;
wire ertn_flush;
wire [31:0] wb_pc;
wire [5:0] wb_ecode;
wire [8:0] wb_esubcode;
wire [31:0] ex_entry;
wire [31:0] ertn_entry;
wire has_int;
wire [31:0] wb_vaddr;
wire ipi_ini_in;
wire [7:0] hw_int_in;
assign ipi_ini_in = 1'b0;
assign hw_int_in = 8'b0;
wire         csr_addr_mode;  //1表示当前为虚地址模式，0表示为物理地址模式
wire [1:0]   crmd_plv_state;         //当前特权级别
wire [9:0]   csr_asid_state;        //地址空间标识符
wire [3:0]   csr_tlbidx_index_state;   //tlb索引寄存器的index域

wire         csr_dmw0_plv0_state;   //用户级别0
wire         csr_dmw0_plv3_state;   //用户级别3
wire [2:0]   csr_dmw0_pseg_state;   //直接映射的物理地址段
wire [2:0]   csr_dmw0_vseg_state;   //直接映射的虚拟地址段
 
wire         csr_dmw1_plv0_state;   
wire         csr_dmw1_plv3_state;
wire [2:0]   csr_dmw1_pseg_state;
wire [2:0]   csr_dmw1_vseg_state;


wire         tlbsrch_hit;
wire [18:0]  tlbehi_vppn;
wire         wrong_addr_is_pc;
wire [3:0]   tlbsrch_index;

csr csr_PART(
    .clk        (clk        ),
    .reset      (reset      ),
    .csr_re     (csr_re     ),
    .csr_num    (csr_num    ),
    .csr_rvalue (csr_rvalue ),
    .csr_we     (csr_we     ),
    .csr_wdata  (csr_wdata  ),
    .csr_wmask  (csr_wmask  ),
    .wb_ex      (wb_ex      ),
    .ertn_flush (ertn_flush ),
    .wb_pc      (wb_pc      ),
    .wb_ecode   (wb_ecode   ),
    .wb_esubcode(wb_esubcode),
    .ex_entry   (ex_entry   ),
    .ertn_entry (ertn_entry ),
    .has_int    (has_int    ),
    .wb_vaddr   (wb_vaddr   ),
    .ipi_ini_in (ipi_ini_in ),
    .hw_int_in  (hw_int_in  ),

    .csr_addr_mode (csr_addr_mode ),
    .crmd_plv_state    (crmd_plv_state   ),
    .csr_asid_state   (csr_asid_state  ),
    .csr_tlbidx_index_state (csr_tlbidx_index_state ),
    .datf_csr        (datf_csr       ),
    .datm_csr        (datm_csr       ),
    .csr_dmw0_plv0_state   (csr_dmw0_plv0_state ),
    .csr_dmw0_plv3_state   (csr_dmw0_plv3_state ),
    .csr_dmw0_pseg_state   (csr_dmw0_pseg_state ),
    .csr_dmw0_vseg_state   (csr_dmw0_vseg_state ),
    .csr_dmw0_mat_state    (csr_dmw0_mat_state  ),

    .csr_dmw1_plv0_state   (csr_dmw1_plv0_state ),
    .csr_dmw1_plv3_state   (csr_dmw1_plv3_state ),
    .csr_dmw1_pseg_state   (csr_dmw1_pseg_state ),
    .csr_dmw1_vseg_state   (csr_dmw1_vseg_state ),
    .csr_dmw1_mat_state    (csr_dmw1_mat_state  ),

    .inst_tlbsrch_in_wb_happen   (inst_tlbsrch_in_wb_happen   ),
    .inst_tlbrd_happen    (tlbrd_happen    ),
    .inst_tlbfill_happen   (tlbfill_happen   ),
    .inst_tlbwr_happen    (tlbwr_happen    ),
    .inst_tlbinvalid_happen   (invtlb_happen   ),

    .r_tlb_e              (r_e             ),
    .r_tlb_ps            (r_ps            ),
    .r_tlb_vppn           (r_vppn          ),
    .r_tlb_asid           (r_asid          ),
    .r_tlb_g              (r_g             ),
    .r_tlb_ppn0           (r_ppn0          ),
    .r_tlb_plv0           (r_plv0          ),
    .r_tlb_mat0           (r_mat0          ),
    .r_tlb_d0             (r_d0            ),
    .r_tlb_v0             (r_v0            ),

    .r_tlb_ppn1           (r_ppn1          ),
    .r_tlb_plv1           (r_plv1          ),
    .r_tlb_mat1           (r_mat1          ),
    .r_tlb_d1             (r_d1            ),
    .r_tlb_v1             (r_v1            ),

    .w_tlb_e              (w_e             ),
    .w_tlb_vppn           (w_vppn          ),
    .w_tlb_asid           (w_asid          ),
    .w_tlb_ps             (w_ps            ),
    .w_tlb_g              (w_g             ),
    .w_tlb_ppn0           (w_ppn0          ),
    .w_tlb_plv0           (w_plv0          ),
    .w_tlb_mat0           (w_mat0          ),
    .w_tlb_d0             (w_d0            ),
    .w_tlb_v0             (w_v0            ),
    .w_tlb_ppn1           (w_ppn1          ),
    .w_tlb_plv1           (w_plv1          ),
    .w_tlb_mat1           (w_mat1          ),
    .w_tlb_d1             (w_d1            ),
    .w_tlb_v1             (w_v1            ),

    .tlbsrch_hit          (tlbsrch_hit     ),
    .tlbehi_vppn         (tlbehi_vppn     ),
    .wrong_addr_is_pc     (wrong_addr_is_pc),
    .tlbsrch_index        (tlbsrch_index    )
);

    wire [18:0]  s0_vppn;
    wire         s0_va_bit12;
    wire [9:0]   s0_asid;
    wire         s0_found;
    wire [3:0]   s0_index;
    wire [19:0]  s0_ppn;
    wire [5:0]   s0_ps;
    wire [1:0]   s0_plv;
    wire         s0_d;
    wire         s0_v;

    wire [18:0]  s1_vppn;
    wire         s1_va_bit12;
    wire [9:0]   s1_asid;
    wire         s1_found;
    wire [3:0]   s1_index;
    wire [19:0]  s1_ppn;
    wire [5:0]   s1_ps;
    wire [1:0]   s1_plv;
    wire         s1_d;
    wire         s1_v;

    wire  invtlb_happen;
    wire  tlbwr_happen;
    wire  tlbfill_happen;
    wire   tlbrd_happen;
    wire   tlbsrch_happen;
    wire  [4:0] invtlb_op;

    wire         w_e;
    wire [18:0]  w_vppn;
    wire [9:0]   w_asid;
    wire [5:0]   w_ps;
    wire         w_g;
    wire [19:0]  w_ppn0;
    wire [1:0]   w_plv0;
    wire [1:0]   w_mat0;
    wire         w_d0;
    wire         w_v0;
    wire [19:0]  w_ppn1;
    wire [1:0]   w_plv1;
    wire [1:0]   w_mat1;
    wire         w_d1;
    wire         w_v1;

    wire         r_e;
    wire [18:0]  r_vppn;
    wire [9:0]   r_asid;
    wire         r_g;
    wire [19:0]  r_ppn0;
    wire [1:0]   r_plv0;
    wire [1:0]   r_mat0;
    wire         r_d0;
    wire         r_v0;
    wire [19:0]  r_ppn1;
    wire [1:0]   r_plv1;
    wire [1:0]   r_mat1;
    wire         r_d1;
    wire         r_v1;

wire [5:0] r_ps;
tlb tlb_PART(
    .clk        (clk        ),
 
    .s0_vppn    (s0_vppn    ),
    .s0_va_bit12(s0_va_bit12),
    .s0_asid    (csr_asid_state   ),
    .s0_found   (s0_found   ),
    .s0_index   (s0_index   ),
    .s0_ppn     (s0_ppn     ),
    .s0_ps      (s0_ps      ),
    .s0_plv     (s0_plv     ),
    .s0_d       (s0_d       ),
    .s0_v       (s0_v       ),
    .s0_mat     (s0_mat     ),

    .s1_vppn    (s1_vppn    ),
    .s1_va_bit12(s1_va_bit12),
    .s1_asid    (s1_asid  ),
    .s1_found   (s1_found   ),
    .s1_index   (s1_index   ),
    .s1_ppn     (s1_ppn     ),
    .s1_ps      (s1_ps      ),
    .s1_plv     (s1_plv     ),
    .s1_d       (s1_d       ),
    .s1_v       (s1_v       ),
    .s1_mat     (s1_mat     ),

    .invtlb_valid (invtlb_happen),
    .invtlb_op  (invtlb_op  ),

    .we         (tlbwr_happen | tlbfill_happen),
    .w_index    (csr_tlbidx_index_state),
    .w_e        (w_e        ),
    .w_vppn     (w_vppn     ),
    .w_asid     (w_asid     ),
    .w_ps       (w_ps       ),
    .w_g        (w_g        ),
    .w_ppn0     (w_ppn0     ),
    .w_plv0     (w_plv0     ),
    .w_mat0     (w_mat0     ),
    .w_d0       (w_d0       ),
    .w_v0       (w_v0       ),
    .w_ppn1     (w_ppn1     ),
    .w_plv1     (w_plv1     ),
    .w_mat1     (w_mat1     ),
    .w_d1       (w_d1       ),
    .w_v1       (w_v1       ),

    .r_index    (csr_tlbidx_index_state),
    .r_e        (r_e        ),
    .r_vppn     (r_vppn     ),
    .r_asid     (r_asid     ),
    .r_g        (r_g        ),
    .r_ppn0     (r_ppn0     ),
    .r_plv0     (r_plv0     ),
    .r_mat0     (r_mat0     ),
    .r_d0       (r_d0       ),
    .r_v0       (r_v0       ),
    .r_ppn1     (r_ppn1     ),
    .r_plv1     (r_plv1     ),
    .r_mat1     (r_mat1     ),
    .r_d1       (r_d1       ),
    .r_v1       (r_v1       ),
    .r_ps       (r_ps       )
);


wire icache_wr_rdy;
assign icache_wr_rdy = 1'b1;//icache不会真正要写sram，置1没有关系
wire icache_wr_req;
wire [2:0] icache_wr_type;
wire [31:0] icache_wr_addr;
wire [3:0] icache_wr_strb;
wire [127:0] icache_wr_data;
wire [11:0] icache_addr_vrtl;
wire [19:0] icache_addr_tag;
wire cacop_req_hit_invalid;
assign cacop_req_hit_invalid = cacop_req_icache & (cacop_op == 2'b10);
assign icache_addr_vrtl = cacop_req_hit_invalid ? data_addr_vrtl : inst_addr_vrtl;
assign icache_addr_tag = cacop_req_hit_invalid ? data_sram_addr[31:12] : inst_sram_addr[31:12];
cache icache(
    .clk    (clk                       ),
    .resetn (resetn                    ),
    .valid  (inst_sram_req              ),
    .op     (inst_sram_wr               ),
    .index  (icache_addr_vrtl[11:4]       ),
    .tag    (icache_addr_tag              ),
    .offset (icache_addr_vrtl[3:0]        ),
    .wstrb  (inst_sram_wstrb            ),
    .wdata  (inst_sram_wdata            ),
    .addr_ok(inst_sram_addr_ok            ),
    .data_ok(inst_sram_data_ok             ),
    .rdata  (inst_sram_rdata               ),

    .rd_req (icache_rd_req              ),
    .rd_type(icache_rd_type             ),
    .rd_addr(icache_rd_addr             ),

    .rd_rdy   (icache_rd_rdy            ),
    .ret_valid(icache_ret_valid         ),
    .ret_last (icache_ret_last          ),
    .ret_data (icache_ret_data          ),
    .mem_type     (mem_type_if),


    .wr_req (icache_wr_req              ),
    .wr_type(icache_wr_type             ),
    .wr_addr(icache_wr_addr             ),
    .wr_wstrb(icache_wr_strb             ),
    .wr_data(icache_wr_data             ),
    .wr_rdy (icache_wr_rdy              ),

    .cacop_req (cacop_req_icache),
    .cacop_op  (cacop_op),
    .cacop_va  (cacop_va),
    .cacop_finish(cacop_finish_icache)
);

cache dcache(
    .clk    (clk                       ),
    .resetn (resetn                    ),
    .valid  (data_sram_req              ),
    .op     (data_sram_wr               ),
    .index  (data_addr_vrtl[11:4]       ),
    .tag    (data_sram_addr[31:12]      ),
    .offset (data_addr_vrtl[3:0]        ),
    .wstrb  (data_sram_wstrb            ),
    .wdata  (data_sram_wdata            ),
    .addr_ok(data_sram_addr_ok             ),
    .data_ok(data_sram_data_ok             ),
    .rdata  (data_sram_rdata               ),

    .rd_req (dcache_rd_req              ),
    .rd_type(dcache_rd_type             ),
    .rd_addr(dcache_rd_addr             ),

    .rd_rdy   (dcache_rd_rdy            ),
    .ret_valid(dcache_ret_valid         ),
    .ret_last (dcache_ret_last          ),
    .ret_data (dcache_ret_data          ),
    .mem_type    (mem_type_exe),//dcache的datm由外部传入

    .wr_req (dcache_wr_req              ),
    .wr_type(dcache_wr_type             ),
    .wr_addr(dcache_wr_addr             ),
    .wr_wstrb(dcache_wr_wstrb            ),
    .wr_data(dcache_wr_data             ),
    .wr_rdy (dcache_wr_rdy              ),

    .cacop_req (cacop_req_dcache),
    .cacop_op  (cacop_op),
    .cacop_va  (cacop_va),
    .cacop_finish(cacop_finish_dcache)
);
endmodule
