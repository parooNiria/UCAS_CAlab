module EXE(
    input           clk,
    input           reset,
    //from ID
    input           ready_go_id,
    output          allow_in,
    input [31:0]    inst_from_id,
    input [31:0]    pc_from_id,
    input [31:0]    src1_from_id,
    input [31:0]    src2_from_id,
    input [18:0]    alu_op_from_id,
    input [4:0]     dest_from_id,
    input           reg_en_from_id,
    input [4:0]     mem_en_from_id,
    input           div_en_from_id,
    input [31:0]    rdata2_from_id,
    input [31:0]    rdata1_from_id,
    input [1:0]     inst_rdcntv,
    //to MEM
    output          ready_go,
    input           MEM_allow_in,
    output [31:0]   inst_exe,
    output [31:0]   pc_exe,
    output [31:0]   alu_result,
    output [32:0]   timer_cnt_global_value_and_en,
    output          reg_en,
    output [6:0]    mem_message,
    output [4:0]    dest,
    //to dram
    output wire        data_sram_req,
    output wire        data_sram_wr,
    output wire [1:0]  data_sram_size,
    output wire [3:0]  data_sram_wstrb,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire        data_sram_addr_ok,
    //valid
    output reg       valid,
    //bypass
    output [31:0] forward_data_exe,
    output        forward_en_exe,
    //exception related
    input  [87:0] exception_message_from_id,
    output [87:0] exception_message_to_mem,
    input         ertn_flush,
    input         wb_ex,
    input  [1:0]  exception_message_from_mem,
    //tlb related
    input  [3:0]  exception_state_tlb_id,
    input  [5:0]  tlb_related_inst_id,
    output        vaddr_sign,
    input         vaddr_sign_from_mem,
    input         vaddr_sign_from_wb,
    output  [8:0] exception_state_tlb_exe,
    output        invalidtlb_happen_in_ex,  //这个也可以直接当invtlb送入csr中
    output        [31:0] invalidtlb_pc,
    input         vaddr_about_inst_happen_in_wb,
    output  [5:0] tlb_related_inst,
    input         stall_srch_from_mem,
    input         stall_srch_from_wb,
    //tlb s1端口
    output wire [ 4:0] invtlb_op,
    output wire        inst_invtlb_happen,
    output wire        inst_tlbsrch_happen,
    output wire [18:0] s1_vppn,
    output wire        s1_va_bit12,
    output wire [ 9:0] s1_asid,   //这个地方在if中没有，是直接由csr送过去的，但是此处由于invalid要复用，不能直接送出

    input         s1_found,
    input  [ 3:0] s1_index,
    input  [19:0] s1_ppn,
    input  [ 5:0] s1_ps,
    input  [ 1:0] s1_plv,

    input         s1_d,
    input         s1_v,

    // DMW0 
    input  wire        csr_dmw0_plv0,   //用户级别0
    input  wire        csr_dmw0_plv3,   //用户级别3
    input  wire [ 2:0] csr_dmw0_pseg,   //直接映射的物理地址段
    input  wire [ 2:0] csr_dmw0_vseg,   //直接映射的虚拟地址段
    // DMW1
    input  wire        csr_dmw1_plv0,   
    input  wire        csr_dmw1_plv3,
    input  wire [ 2:0] csr_dmw1_pseg,
    input  wire [ 2:0] csr_dmw1_vseg,   //mat似乎没有用到

    // output        tlbsrch_hit,
    // output [3:0]  tlbsrch_index,
    //修改后这个应该传给MEM级，直到到达wb级才更新
    output [4:0]  tlbsrch_csr_message,
    input  [ 1:0] crmd_plv, 
    input  [9:0]  csr_asid,
    input  [18:0]  tlbehi_vppn,
    input          csr_addr_mode
);
    always @(posedge clk) begin
        if (reset) begin
            valid <= 1'b0;
        end
        else if (wb_ex || ertn_flush || vaddr_about_inst_happen_in_wb) begin
            valid <= 1'b0;
        end
        else if (ready_go_id &allow_in) begin
            valid <= 1'b1;
        end
        else if (ready_go &MEM_allow_in) begin
            valid <= 1'b0;
        end
    end
    wire inst_rdcntvh_w = inst_rdcntv_reg[1];
    wire inst_rdcntvl_w = inst_rdcntv_reg[0];
    reg [63:0] timer_cnt_global;
    always @(posedge clk) begin
        if (reset) begin
            timer_cnt_global <= 64'b0;
        end
        else begin
            timer_cnt_global <= timer_cnt_global + 64'b1;
        end
    end
    assign timer_cnt_global_value_and_en = {{32{inst_rdcntvh_w}}&timer_cnt_global[63:32]
                                  |{32{inst_rdcntvl_w}}&timer_cnt_global[31:0],inst_rdcntvh_w|inst_rdcntvl_w};
    wire [31:0] alu_result_without_div_mul;
    wire [31:0] alu_result_with_div;
    alu u_alu(
    .alu_op     (alu_op_reg[11:0]   ),
    .alu_src1   (src1_reg   ),
    .alu_src2   (src2_reg   ),
    .alu_result (alu_result_without_div_mul     )
    );
    //mul part
    wire [63:0] unsigned_mul;
    wire [63:0] signed_mul;
    assign unsigned_mul = (src1_reg * src2_reg);
    assign signed_mul = ($signed(src1_reg) * $signed(src2_reg));
    wire [31:0] alu_result_mul;
    wire [31:0] mul_w_result;
    wire [31:0] mulh_w_result;
    wire [31:0] mulh_wu_result;
    assign mul_w_result = unsigned_mul[31:0];
    assign mulh_w_result = signed_mul[63:32];
    assign mulh_wu_result = unsigned_mul[63:32];
    assign alu_result_mul = ({32{alu_op_reg[12]}} & mul_w_result)
                          | ({32{alu_op_reg[13]}} & mulh_w_result)
                          | ({32{alu_op_reg[14]}} & mulh_wu_result);
    //div part
    reg [3:0]     current_state;
    always @(posedge clk) begin
        if (reset) begin
            current_state <= 4'b0001;
        end
        else if(wb_ex || ertn_flush || vaddr_about_inst_happen_in_wb) begin
            current_state <= 4'b0001;
        end
        else if (div_en_from_id & allow_in & ready_go_id) begin
            current_state <= 4'b0010;
        end
        else if(current_state[1]&commit) begin
            current_state <= 4'b0100;
        end
        else if(current_state[2]&div_result_ready&ready_go&MEM_allow_in) begin
            current_state <= 4'b0001;
        end
        else if(current_state[2]&div_result_ready) begin
            current_state <= 4'b1000;
        end
        else if(current_state[3]&ready_go&MEM_allow_in) begin
            current_state <= 4'b0001;
        end
    end
    //div result reg
    reg [31:0] div_result_reg;
    always @(posedge clk) begin
        if (reset) begin
            div_result_reg <= 32'b0;
        end
        else if (current_state[2]&div_result_ready) begin
            div_result_reg <= alu_result_with_div;
        end
    end 
    wire [31:0] alu_result_div_part;
    assign alu_result_div_part = (current_state[3]) ? div_result_reg : alu_result_with_div;
    wire signed_div;
    assign signed_div = alu_op_reg[15]|alu_op_reg[16];
    wire tvalid_div_signed;
    wire tvalid_div_unsigned;
    wire tready_divisor_signed;
    wire tready_dividend_signed;
    wire tready_dividend_unsigned;
    wire tready_divisor_unsigned;
    wire tready_result_signed;
    wire tready_result_unsigned;
    assign tvalid_div_signed = current_state[1] & signed_div &  ~vaddr_sign_from_mem & ~vaddr_sign_from_wb;
    assign tvalid_div_unsigned = current_state[1] & ~signed_div & ~vaddr_sign_from_mem & ~vaddr_sign_from_wb;  //突然发现这个地方也是不能发出开启除法操作信号的，这是之前留下的一个bug，现在改一下
    wire commit_signed;
    wire commit_unsigned;
    wire commit;
    wire div_result_ready;
    assign div_result_ready = signed_div ? tready_result_signed : tready_result_unsigned;
    assign commit_signed = tvalid_div_signed & tready_divisor_signed & tready_dividend_signed;
    assign commit_unsigned = tvalid_div_unsigned & tready_divisor_unsigned & tready_dividend_unsigned;
    assign commit = signed_div ? commit_signed : commit_unsigned;
    wire [63:0] alu_result_signed;
    wire [63:0] alu_result_unsigned;
    div_signed u_div_signed(
        .aclk(clk),
        .s_axis_dividend_tvalid(tvalid_div_signed),
        .s_axis_dividend_tready(tready_dividend_signed),
        .s_axis_dividend_tdata(src1_reg),
        .s_axis_divisor_tvalid(tvalid_div_signed),
        .s_axis_divisor_tready(tready_divisor_signed),
        .s_axis_divisor_tdata(src2_reg),
        .m_axis_dout_tvalid(tready_result_signed),
        .m_axis_dout_tdata(alu_result_signed)
    );

    div_unsigned u_div_unsigned(
        .aclk(clk),
        .s_axis_dividend_tvalid(tvalid_div_unsigned),
        .s_axis_dividend_tready(tready_dividend_unsigned),
        .s_axis_dividend_tdata(src1_reg),
        .s_axis_divisor_tvalid(tvalid_div_unsigned),
        .s_axis_divisor_tready(tready_divisor_unsigned),
        .s_axis_divisor_tdata(src2_reg),
        .m_axis_dout_tvalid(tready_result_unsigned),
        .m_axis_dout_tdata(alu_result_unsigned)
    );
    wire [31:0] div_result_signed;
    wire [31:0] div_result_unsigned;
    wire [31:0] mod_result_signed;
    wire [31:0] mod_result_unsigned;
    assign div_result_signed = alu_result_signed[63:32];
    assign mod_result_signed = alu_result_signed[31:0];
    assign div_result_unsigned = alu_result_unsigned[63:32];
    assign mod_result_unsigned = alu_result_unsigned[31:0];
    assign alu_result_with_div = ({32{alu_op_reg[15]}} & div_result_signed)
                              | ({32{alu_op_reg[16]}} & mod_result_signed)
                              | ({32{alu_op_reg[17]}} & div_result_unsigned)
                              | ({32{alu_op_reg[18]}} & mod_result_unsigned);
    assign alu_result = (mul_type & valid) ? alu_result_mul :
                    (div_en_reg & valid) ? alu_result_div_part : alu_result_without_div_mul;

    assign reg_en     = reg_en_reg;
    reg [31:0] rdata2_reg;
    reg [31:0] rdata1_reg;
    reg [31:0] inst_reg;
    reg [31:0] pc_reg;
    reg [31:0] src1_reg;
    reg [31:0] src2_reg;
    reg [18:0] alu_op_reg;
    reg [4:0]  dest_reg;
    reg        reg_en_reg;
    reg [4:0]  mem_en_reg;
    reg        div_en_reg;
    reg [87:0] exception_message_reg;
    reg [1:0]  inst_rdcntv_reg;
    reg [3:0]  exception_state_tlb_reg;
    reg [5:0]  tlb_related_inst_reg;
    always @(posedge clk) begin
        if (reset) begin
            inst_reg   <= 32'b0;
            pc_reg     <= 32'b0;
            src1_reg   <= 32'b0;
            src2_reg   <= 32'b0;
            alu_op_reg <= 12'b0;
            dest_reg   <= 5'b0;
            reg_en_reg <= 1'b0;
            mem_en_reg <= 5'b0;
            rdata2_reg <= 32'b0;
            rdata1_reg <= 32'b0;
            div_en_reg <= 1'b0;
            exception_message_reg <= 87'b0;
            inst_rdcntv_reg <= 2'b0;
            exception_state_tlb_reg <= 4'b0;
            tlb_related_inst_reg <= 6'b0;
        end
        else if (ready_go_id &allow_in) begin
            inst_reg   <= inst_from_id;
            pc_reg     <= pc_from_id;
            src1_reg   <= src1_from_id;
            src2_reg   <= src2_from_id;
            alu_op_reg <= alu_op_from_id;
            dest_reg   <= dest_from_id;
            reg_en_reg <= reg_en_from_id;
            mem_en_reg <= mem_en_from_id;
            rdata2_reg <= rdata2_from_id;
            rdata1_reg <= rdata1_from_id;
            div_en_reg <= div_en_from_id;
            exception_message_reg <= exception_message_from_id;
            inst_rdcntv_reg <= inst_rdcntv;
            exception_state_tlb_reg <= exception_state_tlb_id;
            tlb_related_inst_reg <= tlb_related_inst_id;
        end
    end
    wire inst_tlbsrch = tlb_related_inst_reg[4];
    wire inst_tlbrd   = tlb_related_inst_reg[3];
    wire inst_tlbwr   = tlb_related_inst_reg[2];
    wire inst_tlbfill = tlb_related_inst_reg[1];
    wire inst_invtlb  = tlb_related_inst_reg[0];
    
    wire mul_type;
    assign mul_type = alu_op_reg[12]|alu_op_reg[13]|alu_op_reg[14];
    assign dest       = dest_reg;
    assign allow_in   = ~valid | (ready_go&&MEM_allow_in);
    assign ready_go   = valid&((!div_en_reg) | (current_state[2]&&div_result_ready) | current_state[3] | ex_stall_mem_store | vaddr_sign_from_mem | vaddr_sign_from_wb)
    &((~mem_type)|(data_sram_addr_ok&data_sram_req)|mem_req_ready|ex_stall_mem_store | vaddr_sign_from_mem | vaddr_sign_from_wb)
    &((~inst_tlbsrch)|(~stall_srch_from_mem & ~stall_srch_from_wb)); //阻塞
    assign inst_exe   = inst_reg;
    assign pc_exe     = pc_reg;
    assign forward_data_exe = alu_result_without_div_mul;
    assign forward_en_exe = (~mem_ld)&&(div_en_reg==1'b0)&&(mul_type==1'b0)&&(~csr_re);
//mem deal part
    reg    mem_req_ready;
    always @(posedge clk) begin
        if (reset) begin
            mem_req_ready <= 1'b0;
        end
        else if (ready_go & MEM_allow_in) begin
            mem_req_ready <= 1'b0;
        end        
        else if (data_sram_addr_ok&data_sram_req&valid) begin
            mem_req_ready <= 1'b1;
        end
    end
    wire   mem_type;
    wire   mem_st;
    wire   mem_ld;
    wire   mem_b;
    wire   mem_h;
    wire   mem_bu;
    wire   mem_hu;
    wire   mem_w;
    assign mem_type = mem_en_reg[4];
    assign mem_ld = mem_type & mem_en_reg[3];
    assign mem_st = mem_type & ~mem_en_reg[3];
    assign mem_b  = mem_type & mem_en_reg[1:0]==2'b01 & ~mem_en_reg[2];
    assign mem_h  = mem_type & mem_en_reg[1:0]==2'b10 & ~mem_en_reg[2];
    assign mem_bu = mem_type & mem_en_reg[1:0]==2'b01 & mem_en_reg[2];
    assign mem_hu = mem_type & mem_en_reg[1:0]==2'b10 & mem_en_reg[2];
    assign mem_w  = mem_type & mem_en_reg[1:0]==2'b11;
    assign mem_message = {mem_ld,mem_st,mem_w,mem_h,mem_b,mem_hu,mem_bu};
    wire   [3:0]wen_half;
    assign wen_half = {4{alu_result[1]}}&4'b1100 | {4{~alu_result[1]}}&4'b0011;
    wire   [3:0]wen_b;
    assign wen_b = 4'b0001 << alu_result[1:0];
    //地址翻译
    wire DMW0_hit = (csr_dmw0_plv0 & (crmd_plv == 2'b0) | csr_dmw0_plv3 & (crmd_plv == 2'b11)) &
                     (alu_result[31:29] == csr_dmw0_vseg)&(csr_addr_mode == 1'b1);
    wire DMW1_hit = (csr_dmw1_plv0 & (crmd_plv == 2'b0) | csr_dmw1_plv3 & (crmd_plv == 2'b11)) &
                     (alu_result[31:29] == csr_dmw1_vseg)&(csr_addr_mode == 1'b1);
    wire [31:0] DMW0_addr_translated;
    wire [31:0] DMW1_addr_translated;
    assign DMW0_addr_translated = {csr_dmw0_pseg, alu_result[28:0]};
    assign DMW1_addr_translated = {csr_dmw1_pseg, alu_result[28:0]};
    //TLB映射
    assign {s1_vppn, s1_va_bit12} = inst_invtlb_happen ? src2_reg[31:12] :      //rk中来
                                    inst_tlbsrch_happen ? {tlbehi_vppn,1'b0} :  //这时bit12无意义
                                    alu_result[31:12]; 
    assign s1_asid = inst_invtlb_happen ? src1_reg[9:0]:csr_asid;
    wire [31:0] tlb_addr_translated;
    assign tlb_addr_translated = (s1_ps == 6'd22) ? {s1_ppn[19:10], alu_result[21:0]} : {s1_ppn, alu_result[11:0]};
    //最终的地址选择
    assign data_sram_addr = DMW0_hit ? DMW0_addr_translated :
                            DMW1_hit ? DMW1_addr_translated :
                            (csr_addr_mode == 1'b0) ? alu_result : //物理地址模式
                            tlb_addr_translated; //tlb地址转换
    //由此产生异常
    wire   exception_TLBR_DATA = valid & mem_type & csr_addr_mode & ~DMW0_hit & ~DMW1_hit & ~s1_found;
    wire   exception_PIL = valid & mem_type & csr_addr_mode & ~DMW0_hit & ~DMW1_hit & s1_found & ~mem_st & ~s1_v;
    wire   exception_PIS = valid & mem_type & csr_addr_mode & ~DMW0_hit & ~DMW1_hit & s1_found & mem_st & ~s1_v;
    wire   exception_PME = valid & mem_type & csr_addr_mode & ~DMW0_hit & ~DMW1_hit & s1_found & s1_v & mem_st & (crmd_plv <= s1_plv) &!s1_d;
    wire   exception_PPI_DATA = valid & mem_type & csr_addr_mode & ~DMW0_hit & ~DMW1_hit & s1_found & s1_v &(crmd_plv > s1_plv);

    assign data_sram_req    = valid&~mem_req_ready&mem_type&~ex_stall_mem_store & ~vaddr_sign_from_mem & ~vaddr_sign_from_wb;          //由异常阻止和虚实地址改变阻止
    assign data_sram_wr     = mem_st;
    assign data_sram_size   = (mem_w?2'b10:
                              (mem_h|mem_hu)?2'b01:
                              (mem_b|mem_bu)?2'b00:2'b10
                            );
    assign data_sram_wstrb    = (mem_w?4'b1111:
                              mem_h?wen_half:
                              mem_b?wen_b:4'b0000
                            )&{4{valid}}&{4{mem_st}};
    assign data_sram_wdata = mem_b ? {4{rdata2_reg[7:0]}} :
                             mem_h ? {2{rdata2_reg[15:0]}} :
                            rdata2_reg;
//exception related
    wire csr_re;
    assign csr_re = exception_message_reg[79];

    wire exception_state_exe_now;
    assign exception_state_exe_now = (exception_int|exception_adef|exception_ine|exception_ale|inst_break|inst_syscall
                                     | exception_tlbr_FETCH | exception_pif | exception_PPI_FETCH|
                                     exception_TLBR_DATA | exception_PIL | exception_PIS | exception_PME | 
                                     exception_PPI_DATA)&valid;

    wire exception_int;
    wire exception_adef;
    wire exception_ine;
    wire exception_ale;
    wire inst_break;
    wire inst_syscall;
    wire inst_ertn;
    assign exception_int = exception_message_reg[86];
    assign exception_adef = exception_message_reg[85];
    assign exception_ine = exception_message_reg[84];
    assign exception_ale = exception_message_reg[83]|(((mem_w & alu_result[1:0]!=2'b00)|((mem_hu|mem_h)&alu_result[0]))&~exception_adef);
    wire   exception_tlbr_FETCH = valid & exception_state_tlb_reg[3];
    wire   exception_pif = exception_state_tlb_reg[2];
    wire   exception_PPI_FETCH = exception_state_tlb_reg[1];
    assign vaddr_sign = valid & exception_state_tlb_reg[0];
    
    assign inst_break = exception_message_reg[82];
    assign inst_ertn = exception_message_reg[81];
    assign inst_syscall = exception_message_reg[80];
    wire exception_state_mem;
    wire ertn_mem;
    assign exception_state_mem = exception_message_from_mem[0];
    assign ertn_mem = exception_message_from_mem[1];

    wire ex_stall_mem_store;
    assign ex_stall_mem_store = exception_state_exe_now
                                |wb_ex | ertn_flush
                                |exception_state_mem | ertn_mem;

    assign exception_message_to_mem = {   exception_state_exe_now,   //88
                                          exception_int,             //87
                                          exception_adef,            //86
                                          exception_ine,             //85
                                          exception_ale,             //84
                                          inst_break,                //83
                                          inst_ertn,                 //82
                                          inst_syscall,              //80
                                          exception_message_reg[79:0]
                                        };
    wire vaddr_sign_next = vaddr_sign & ~inst_invtlb_happen;

    assign exception_state_tlb_exe = {exception_tlbr_FETCH,
                                      exception_pif,
                                      exception_PPI_FETCH,
                                      exception_TLBR_DATA,
                                        exception_PIL,
                                        exception_PIS,
                                        exception_PME,
                                        exception_PPI_DATA,
                                      vaddr_sign_next
                                     };
    assign invalidtlb_happen_in_ex = inst_invtlb & valid & ~wb_ex & ~ertn_flush & ~exception_state_mem & ~ertn_mem
                                     & ~vaddr_sign_from_mem & ~vaddr_sign_from_wb&
                                     (~exception_int&~exception_adef&~exception_ine&~exception_ale&~inst_break&~inst_syscall&
                                     ~exception_tlbr_FETCH&~exception_PPI_FETCH&~exception_pif) //此前没有导致的异常
                                     &~stall_srch_from_mem & ~stall_srch_from_wb; //前面没有在mem和wb级引起tlbsrch指令的stall
    assign inst_invtlb_happen = invalidtlb_happen_in_ex;
    //这个地方形成一个组合环
    assign inst_tlbsrch_happen = inst_tlbsrch & valid & ~wb_ex & ~ertn_flush & ~exception_state_mem & ~ertn_mem
                                     & ~vaddr_sign_from_mem & ~vaddr_sign_from_wb;
    assign invalidtlb_pc = pc_reg;

    assign invtlb_op = {5{inst_invtlb}} & inst_exe[4:0];
    wire tlbsrch_hit = valid & inst_tlbsrch_happen & s1_found;
    wire [3:0] tlbsrch_index ;
    assign tlbsrch_index = s1_index;
    assign tlbsrch_csr_message = {tlbsrch_hit, tlbsrch_index};
    assign tlb_related_inst = tlb_related_inst_reg;
endmodule