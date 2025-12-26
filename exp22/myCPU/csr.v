module csr(
    input  wire         clk,
    input  wire         reset,
    //read port
    input  wire         csr_re,
    input  wire [13:0]  csr_num,
    output wire [31:0]  csr_rvalue,
    //write port
    input  wire         csr_we,
    input  wire [31:0]  csr_wdata,
    input  wire [31:0]  csr_wmask,
    //exception port
    input  wire         wb_ex,
    input  wire         ertn_flush,
    input  wire [31:0]  wb_pc,
    input  wire [7:0]   hw_int_in,
    input  wire         ipi_ini_in,
    input  wire [5:0]   wb_ecode,
    input  wire [8:0]   wb_esubcode,
    input  wire [31:0]  wb_vaddr,
    //地址翻译所需信号
    output wire         csr_addr_mode,  //1表示当前为虚地址模式，0表示为物理地址模式
    output wire [1:0]   crmd_plv_state,         //当前特权级别
    output wire [9:0]   csr_asid_state,        //地址空间标识符
    output wire [3:0]   csr_tlbidx_index_state,   //tlb索引寄存器的index域
    output wire [1:0]   datf_csr,               
    output wire [1:0]   datm_csr,

    output wire         csr_dmw0_plv0_state,   //用户级别0
    output wire         csr_dmw0_plv3_state,   //用户级别3
    output wire [2:0]   csr_dmw0_pseg_state,   //直接映射的物理地址段
    output wire [2:0]   csr_dmw0_vseg_state,   //直接映射的虚拟地址段
    output wire [1:0]   csr_dmw0_mat_state,    
    output wire         csr_dmw1_plv0_state,   
    output wire         csr_dmw1_plv3_state,
    output wire [2:0]   csr_dmw1_pseg_state,
    output wire [2:0]   csr_dmw1_vseg_state,
    output wire [1:0]   csr_dmw1_mat_state,


    //tlb instruction port
    input  wire         inst_tlbsrch_in_wb_happen,
    input  wire         inst_tlbrd_happen,
    input  wire         inst_tlbwr_happen,
    input  wire         inst_tlbfill_happen,
    input  wire         inst_tlbinvalid_happen,//疑似无用

    //tlb相关接口
    //首先是关于读写端口的指令tlbwr和tlbrd，还有tlbfill
    input  wire             r_tlb_e,
    input  wire [ 5:0]      r_tlb_ps,
    input  wire [18:0]      r_tlb_vppn,
    input  wire [ 9:0]      r_tlb_asid,
    input  wire             r_tlb_g,

    input  wire [19:0]      r_tlb_ppn0,
    input  wire [ 1:0]      r_tlb_plv0,
    input  wire [ 1:0]      r_tlb_mat0,
    input  wire             r_tlb_d0,
    input  wire             r_tlb_v0,

    input  wire [19:0]      r_tlb_ppn1,
    input  wire [ 1:0]      r_tlb_plv1,
    input  wire [ 1:0]      r_tlb_mat1,
    input  wire             r_tlb_d1,
    input  wire             r_tlb_v1,

    output wire             w_tlb_e,
    output wire [ 5:0]      w_tlb_ps,
    output wire [18:0]      w_tlb_vppn,
    output wire [ 9:0]      w_tlb_asid,
    output wire             w_tlb_g,

    output wire [19:0]      w_tlb_ppn0,
    output wire [ 1:0]      w_tlb_plv0,
    output wire [ 1:0]      w_tlb_mat0,
    output wire             w_tlb_d0,
    output wire             w_tlb_v0,

    output wire [19:0]      w_tlb_ppn1,
    output wire [ 1:0]      w_tlb_plv1,
    output wire [ 1:0]      w_tlb_mat1,
    output wire             w_tlb_d1,
    output wire             w_tlb_v1,

    //tlbsrch和tlbinvalid 这两都是复用了访存端口，除了知道这两个指令发生了之外，还需要知道tlbsrch的结果和tlbinvalid的索引 ，好像tlbinvalid这个信号与csr没有关系？
    input wire              tlbsrch_hit,//是否命中，决定ne
    input wire [$clog2(16)-1:0] tlbsrch_index,
    output wire [18:0]       tlbehi_vppn, //用于查询tlbsrch的地址
    input  wire              wrong_addr_is_pc,   //出错的地址可能是pc还可能是vaddr，此时需要一个信号来区分
    output wire [31:0]  ex_entry,
    output wire [31:0]  ertn_entry,
    output wire         has_int
);

    `define CSR_CRMD   14'h0000
    `define CSR_PRMD   14'h0001
    `define CSR_ECFG   14'h0004
    `define CSR_ESTAT  14'h0005
    `define CSR_ERA    14'h0006
    `define CSR_BADV   14'h0007
    `define CSR_EENTRY 14'h000c
    `define CSR_TLBIDX    14'h0010
    `define CSR_TLBEHI    14'h0011
    `define CSR_TLBELO0   14'h0012
    `define CSR_TLBELO1   14'h0013
    `define CSR_ASID      14'h0018
    `define CSR_SAVE0  14'h0030
    `define CSR_SAVE1  14'h0031
    `define CSR_SAVE2  14'h0032
    `define CSR_SAVE3  14'h0033  
    `define CSR_TID    14'h0040
    `define CSR_TCFG   14'h0041
    `define CSR_TVAL   14'h0042
    `define CSR_TICLR  14'h0044
    `define CSR_TLBRENTRY 14'h0088
    `define CSR_DMW0      14'h0180
    `define CSR_DMW1      14'h0181

    `define CSR_CRMD_PLV  1:0
    `define CSR_CRMD_PIE   2
    `define CSR_PRMD_PPLV 1:0
    `define CSR_PRMD_PIE  2
    `define CSR_CRMD_DA   3
    `define CSR_CRMD_PG   4
    `define CSR_CRMD_DATF 6:5
    `define CSR_CRMD_DATM 8:7
    `define CSR_ECFG_LIE  12:0
    `define CSR_ESTAT_IS10 1:0  
    `define CSR_ERA_PC    31:0
    `define CSR_EENTRY_VA 31:6
    `define CSR_SAVE_DATA 31:0
    `define CSR_TICLR_CLR 0
    `define CSR_TID_TID   31:0
    `define CSR_TCFG_EN   0
    `define CSR_TCFG_PERIOD 1
    `define CSR_TCFG_INITV 31:2
    `define CSR_TCFG_INITVAL 31:2
    `define CSR_TLBIDX_INDEX 3:0
    `define CSR_TLBIDX_PS 29:24
    `define CSR_TLBIDX_NE 31
    `define CSR_TLBEHI_VPPN 31:13
    `define CSR_TLBELO_V  0
    `define CSR_TLBELO_D  1
    `define CSR_TLBELO_PLV 3:2
    `define CSR_TLBELO_MAT 5:4
    `define CSR_TLBELO_G  6
    `define CSR_TLBELO_PPN 27:8
    `define CSR_ASID_ASID 9:0
    `define CSR_ASID_ASIDBITS 23:16
    `define CSR_TLBRENTRY_PA 31:6
    `define CSR_DMW_PLV0  0
    `define CSR_DMW_PLV3  3
    `define CSR_DMW_MAT   5:4
    `define CSR_DMW_PSEG  27:25
    `define CSR_DMW_VSEG  31:29

    `define ECODE_ADE    6'h08
    `define ECODE_ALE    6'h09
    `define ESUBCODE_ADEF 9'h00
    `define ECODE_SYS 6'h0B
    `define ECODE_BRK 6'h0C   
    `define ECODE_INE 6'h0D
    `define ECODE_TLBR 6'h3F
    `define ECODE_PIL 6'h01  // LOAD页无效例外
    `define ECODE_PIS 6'h02  // STORE页无效例外
    `define ECODE_PIF 6'h03  // FETCH页无效例外
    `define ECODE_PME 6'h04  // 页修改例外
    `define ECODE_PPI 6'h07  // 页特权等级不合规例外
//CSR registers
    //CRMD
    wire [31:0] csr_crmd;
    reg [1:0]  csr_crmd_plv;
    reg        csr_crmd_ie;
    reg       csr_crmd_da;
    reg       csr_crmd_pg;
    reg [1:0] csr_crmd_datf;
    reg [1:0] csr_crmd_datm;
    always @(posedge clk) begin
        if (reset) begin
            csr_crmd_plv <= 2'b0;
            csr_crmd_ie <= 1'b0;
        end
        else if(wb_ex) begin
            csr_crmd_plv <= 2'b0;
            csr_crmd_ie <= 1'b0;
        end 
        else if(ertn_flush) begin
            csr_crmd_plv <= csr_prmd_pplv;
            csr_crmd_ie <= csr_prmd_pie;
        end
        else if(csr_we && csr_num == `CSR_CRMD) begin
            csr_crmd_plv <= csr_wdata[`CSR_CRMD_PLV] & csr_wmask[`CSR_CRMD_PLV]
                            | csr_crmd_plv & ~csr_wmask[`CSR_CRMD_PLV];
            csr_crmd_ie <= csr_wdata[`CSR_CRMD_PIE] & csr_wmask[`CSR_CRMD_PIE]
                           | csr_crmd_ie & ~csr_wmask[`CSR_CRMD_PIE];
        end
    end

    // assign csr_crmd_da = 1'b1;
    // assign csr_crmd_pg = 1'b0;
        //写到19发现，datf和datm必须实现
    // assign csr_crmd_datf = 2'b0;
    // assign csr_crmd_datm = 2'b0;
    //完善上述的信号
    always @(posedge clk) begin
        if (reset)
            csr_crmd_da <= 1'b1;
        else if (wb_ex && wb_ecode==6'h3F)  //发生异常且是重填异常
            csr_crmd_da <= 1'b1;
        else if (ertn_flush && csr_estat_ecode==6'b111111)
            csr_crmd_da <= 1'b0;
        else if (csr_we && csr_num==`CSR_CRMD)
            csr_crmd_da <= csr_wmask[`CSR_CRMD_DA]&csr_wdata[`CSR_CRMD_DA]
                        | ~csr_wmask[`CSR_CRMD_DA]&csr_crmd_da;
    end       

    always @(posedge clk) begin
        if (reset)
            csr_crmd_pg <= 1'b0;
        else if (wb_ex && wb_ecode==6'h3F)
            csr_crmd_pg <= 1'b0;
        else if (ertn_flush && csr_estat_ecode==6'b111111)
            csr_crmd_pg <= 1'b1;
        else if (csr_we && csr_num==`CSR_CRMD)
            csr_crmd_pg <= csr_wmask[`CSR_CRMD_PG]&csr_wdata[`CSR_CRMD_PG]
                        | ~csr_wmask[`CSR_CRMD_PG]&csr_crmd_pg;
    end

    always @(posedge clk) begin
        if (reset)
            csr_crmd_datf <= 2'b0;
        else if (csr_we && csr_num==`CSR_CRMD)
            csr_crmd_datf <= csr_wmask[`CSR_CRMD_DATF]&csr_wdata[`CSR_CRMD_DATF]
                          | ~csr_wmask[`CSR_CRMD_DATF]&csr_crmd_datf;
    end

    always @(posedge clk) begin
        if (reset)
            csr_crmd_datm <= 2'b0; 
        else if (csr_we && csr_num==`CSR_CRMD)
            csr_crmd_datm <= csr_wmask[`CSR_CRMD_DATM]&csr_wdata[`CSR_CRMD_DATM]
                          | ~csr_wmask[`CSR_CRMD_DATM]&csr_crmd_datm;
    end
    assign datf_csr = csr_crmd_datf;
    assign datm_csr = csr_crmd_datm;
    assign csr_crmd = {23'b0,csr_crmd_datm,csr_crmd_datf,csr_crmd_pg,csr_crmd_da,
                       csr_crmd_ie,csr_crmd_plv};
    assign csr_addr_mode = csr_crmd_pg;
    assign crmd_plv_state = csr_crmd_plv;

    //PRMD
    wire [31:0] csr_prmd;
    reg [1:0]  csr_prmd_pplv;
    reg        csr_prmd_pie;
    always @(posedge clk) begin
        if(wb_ex) begin
            csr_prmd_pplv <= csr_crmd_plv;
            csr_prmd_pie <= csr_crmd_ie;
        end 
        else if(csr_we && csr_num == `CSR_PRMD) begin
            csr_prmd_pplv <= csr_wdata[`CSR_PRMD_PPLV] & csr_wmask[`CSR_PRMD_PPLV]
                             | csr_prmd_pplv & ~csr_wmask[`CSR_PRMD_PPLV];
            csr_prmd_pie <= csr_wdata[`CSR_PRMD_PIE] & csr_wmask[`CSR_PRMD_PIE]
                            | csr_prmd_pie & ~csr_wmask[`CSR_PRMD_PIE];
        end
    end

    assign csr_prmd = {29'b0,csr_prmd_pie,csr_prmd_pplv};
    
    //ESTAT
    reg  [12:0] csr_estat_is;
    reg  [5:0]  csr_estat_ecode;
    reg  [8:0]  csr_estat_esubcode;

    always@(posedge clk) begin
        if(reset) begin
            csr_estat_is[1:0] <= 2'b0;
        end
        else if(csr_we && csr_num == `CSR_ESTAT) begin
            csr_estat_is[1:0] <= (csr_wdata[`CSR_ESTAT_IS10] & csr_wmask[`CSR_ESTAT_IS10])
                                 | (csr_estat_is[1:0] & ~csr_wmask[`CSR_ESTAT_IS10]);
        end

        csr_estat_is[9:2] <= hw_int_in;
        csr_estat_is[10] <= 1'b0;

        if(reset) begin
            csr_estat_is[11] <= 1'b0;
        end
        else if(timer_cnt[31:0] == 32'b0)
            csr_estat_is[11] <= 1'b1;
        else if(csr_we && csr_num == `CSR_TICLR && csr_wmask[`CSR_TICLR_CLR] && csr_wdata[`CSR_TICLR_CLR]) begin
            csr_estat_is[11] <= 1'b0;
        end


        csr_estat_is[12] <= ipi_ini_in;
    end

    always@(posedge clk) begin
        if(wb_ex) begin
            csr_estat_ecode <= wb_ecode;
            csr_estat_esubcode <= wb_esubcode;
        end
    end
    wire [31:0] csr_estat;  
    assign csr_estat = {1'b0,csr_estat_esubcode,csr_estat_ecode,3'b0,csr_estat_is};

    //ERA
    reg [31:0] csr_era_pc;
    always@(posedge clk) begin
        if(wb_ex) 
            csr_era_pc <= wb_pc;
        else if(csr_we && csr_num == `CSR_ERA) 
            csr_era_pc <= (csr_wdata[`CSR_ERA_PC] & csr_wmask[`CSR_ERA_PC])
                        | (csr_era_pc & ~csr_wmask[`CSR_ERA_PC]);
    end

    //EENTRY
    wire [31:0] csr_eentry;
    reg [25:0] csr_eentry_va;
    always@(posedge clk) begin
        if(csr_we && csr_num == `CSR_EENTRY) 
            csr_eentry_va <= (csr_wdata[`CSR_EENTRY_VA] & csr_wmask[`CSR_EENTRY_VA])
                            | (csr_eentry_va & ~csr_wmask[`CSR_EENTRY_VA]);
    end
    assign csr_eentry = {csr_eentry_va,6'b0};

    //SAVE0~3
    reg [31:0] csr_save0_data;
    reg [31:0] csr_save1_data;
    reg [31:0] csr_save2_data;
    reg [31:0] csr_save3_data;
    always@(posedge clk) begin
        if(csr_we && csr_num == `CSR_SAVE0) 
            csr_save0_data <= (csr_wdata[`CSR_SAVE_DATA] & csr_wmask[`CSR_SAVE_DATA])
                             | (csr_save0_data & ~csr_wmask[`CSR_SAVE_DATA]);
        if(csr_we && csr_num == `CSR_SAVE1) 
            csr_save1_data <= (csr_wdata[`CSR_SAVE_DATA] & csr_wmask[`CSR_SAVE_DATA])
                             | (csr_save1_data & ~csr_wmask[`CSR_SAVE_DATA]);
        if(csr_we && csr_num == `CSR_SAVE2) 
            csr_save2_data <= (csr_wdata[`CSR_SAVE_DATA] & csr_wmask[`CSR_SAVE_DATA])
                             | (csr_save2_data & ~csr_wmask[`CSR_SAVE_DATA]);
        if(csr_we && csr_num == `CSR_SAVE3) 
            csr_save3_data <= (csr_wdata[`CSR_SAVE_DATA] & csr_wmask[`CSR_SAVE_DATA])
                             | (csr_save3_data & ~csr_wmask[`CSR_SAVE_DATA]);
    end

    

    //ECFG
    reg [12:0] csr_ecfg_lie;
    wire [31:0] csr_ecfg;
    always@(posedge clk) begin
        if(reset) begin
            csr_ecfg_lie <= 13'b0;
        end
        else if(csr_we && csr_num == `CSR_ECFG) begin
            csr_ecfg_lie <= (csr_wdata[`CSR_ECFG_LIE] & csr_wmask[`CSR_ECFG_LIE]& 13'h1bff)
                            | (csr_ecfg_lie & ~csr_wmask[`CSR_ECFG_LIE] & 13'h1bff);
        end
    end
    assign csr_ecfg = {19'b0,csr_ecfg_lie};

    //BADV
    reg [31:0] csr_badv;
    wire wb_ex_addr_err = wb_ecode==`ECODE_ADE || wb_ecode==`ECODE_ALE || wb_ecode==`ECODE_PIL || wb_ecode==`ECODE_TLBR
                          ||wb_ecode==`ECODE_PIS || wb_ecode==`ECODE_PIF || wb_ecode==`ECODE_PME || wb_ecode==`ECODE_PPI;
    always@(posedge clk) begin
        if(wb_ex && wb_ex_addr_err) begin
            csr_badv <= (wrong_addr_is_pc ? wb_pc : wb_vaddr);
        end
    end

    //TID
    reg [31:0] csr_tid_tid;
    always@(posedge clk) begin
        if(reset) begin
            csr_tid_tid <= 32'b0;
        end
        else if(csr_we && csr_num == `CSR_TID) begin
            csr_tid_tid <= (csr_wdata[`CSR_TID_TID] & csr_wmask[`CSR_TID_TID])
                           | (csr_tid_tid & ~csr_wmask[`CSR_TID_TID]);
        end
    end

    //TCFG
    reg        csr_tcfg_en;
    reg        csr_tcfg_periodic;
    reg [29:0] csr_tcfg_initval;
    wire [31:0] csr_tcfg;
    always@(posedge clk) begin
        if(reset) begin
            csr_tcfg_en <= 1'b0;
        end
        else if(csr_we && csr_num == `CSR_TCFG) begin
            csr_tcfg_en <= (csr_wdata[`CSR_TCFG_EN] & csr_wmask[`CSR_TCFG_EN])
                           | (csr_tcfg_en & ~csr_wmask[`CSR_TCFG_EN]);
        end
        
        if(csr_we && csr_num == `CSR_TCFG) begin
            csr_tcfg_periodic <= (csr_wdata[`CSR_TCFG_PERIOD] & csr_wmask[`CSR_TCFG_PERIOD])
                              | (csr_tcfg_periodic & ~csr_wmask[`CSR_TCFG_PERIOD]);
            csr_tcfg_initval <= (csr_wdata[`CSR_TCFG_INITV] & csr_wmask[`CSR_TCFG_INITV])
                              | (csr_tcfg_initval & ~csr_wmask[`CSR_TCFG_INITV]);
        end
    end
    assign csr_tcfg = {csr_tcfg_initval,csr_tcfg_periodic,csr_tcfg_en};

    //TVAL
    reg [31:0] timer_cnt;
    wire [31:0] tcfg_next_value;
    wire [31:0] csr_tval;
    assign tcfg_next_value = csr_wmask[31:0]&csr_wdata[31:0]|
                            ~csr_wmask[31:0]&{csr_tcfg_initval,csr_tcfg_periodic,csr_tcfg_en};
    always@(posedge clk) begin
        if(reset) begin
            timer_cnt <= 32'hffff_ffff;
        end
        else if(csr_we && csr_num == `CSR_TCFG && tcfg_next_value[`CSR_TCFG_EN] ) begin
            timer_cnt <= {tcfg_next_value[`CSR_TCFG_INITVAL],2'b0};
        end
        else if(csr_tcfg_en && timer_cnt != 32'hffffffff) begin
            if(timer_cnt == 32'b0 && csr_tcfg_periodic) begin
                timer_cnt <= {csr_tcfg_initval,2'b0};
            end
            else  begin
                timer_cnt <= timer_cnt - 1'b1;
            end 
        end
    end

    //TICLR
    wire csr_ticlr_clr;
    wire [31:0] csr_ticlr;
    assign csr_ticlr = {31'b0,csr_ticlr_clr};
    assign csr_ticlr_clr = 1'b0;

    //TLBIDX
    reg  [ 3:0]      csr_tlbidx_index;
    reg              csr_tlbidx_ne;
    reg [5:0] csr_tlbidx_ps;

    //index可能由写改，TLBSRCH改变
    always @(posedge clk) begin
        if (reset)
            csr_tlbidx_index <= 4'b0;
        else if (csr_we && csr_num==`CSR_TLBIDX)
            csr_tlbidx_index <= csr_wmask[`CSR_TLBIDX_INDEX]&csr_wdata[`CSR_TLBIDX_INDEX]
                             | ~csr_wmask[`CSR_TLBIDX_INDEX]&csr_tlbidx_index;
        else if (inst_tlbsrch_in_wb_happen && tlbsrch_hit)
            csr_tlbidx_index <= tlbsrch_index;
    end
    //ps TLBRD改变
    always @(posedge clk) begin
        if (reset)
            csr_tlbidx_ps <= 6'b0;
        else if (csr_we && csr_num==`CSR_TLBIDX)
            csr_tlbidx_ps <= csr_wmask[`CSR_TLBIDX_PS]&csr_wdata[`CSR_TLBIDX_PS]
                          | ~csr_wmask[`CSR_TLBIDX_PS]&csr_tlbidx_ps;
        else if (inst_tlbrd_happen && r_tlb_e)   //tlbrd有效
            csr_tlbidx_ps <= r_tlb_ps;
        else if (inst_tlbrd_happen && ~r_tlb_e)  //tlbrd无效
            csr_tlbidx_ps <= 6'b0;
    end

    //ne 多种改变
    always @(posedge clk) begin
        if (reset)
            csr_tlbidx_ne <= 1'b0;
        else if (csr_we && csr_num==`CSR_TLBIDX)
            csr_tlbidx_ne <= csr_wmask[`CSR_TLBIDX_NE]&csr_wdata[`CSR_TLBIDX_NE]
                          | ~csr_wmask[`CSR_TLBIDX_NE]&csr_tlbidx_ne;
        else if (inst_tlbsrch_in_wb_happen)
            csr_tlbidx_ne <= ~tlbsrch_hit;
        else if (inst_tlbrd_happen)
            csr_tlbidx_ne <= ~r_tlb_e;
    end

    wire [31:0] csr_tlbidx;
    assign csr_tlbidx = {csr_tlbidx_ne,1'b0,csr_tlbidx_ps,20'b0,csr_tlbidx_index};

    //TLBEHI
    //当触发TLB重填例外、load操作页无效例外、store操作页无效例外、取指操作页无
    //效例外、页写允许例外和页特权等级不合规例外时，触发例外的虚地址的[31:13]位被
    //记录到这里。
    //执行TLBRD指令时，所读取TLB表项的VPPN域的值记录到这里。
    wire tlbehi_exception_about =  wb_ecode == `ECODE_TLBR || wb_ecode == `ECODE_PIL || wb_ecode == `ECODE_PIS 
                       || wb_ecode == `ECODE_PIF  || wb_ecode == `ECODE_PME || wb_ecode == `ECODE_PPI;
    reg [18:0] csr_tlbehi_vppn;

    always @(posedge clk) begin
        if (reset)
            csr_tlbehi_vppn <= 19'b0;
        else if (csr_we && csr_num==`CSR_TLBEHI)
            csr_tlbehi_vppn <= csr_wmask[`CSR_TLBEHI_VPPN]&csr_wdata[`CSR_TLBEHI_VPPN]
                            | ~csr_wmask[`CSR_TLBEHI_VPPN]&csr_tlbehi_vppn;
        else if (wb_ex && tlbehi_exception_about)
            csr_tlbehi_vppn <= wrong_addr_is_pc ? wb_pc[`CSR_TLBEHI_VPPN] : wb_vaddr[`CSR_TLBEHI_VPPN];
        else if (inst_tlbrd_happen && r_tlb_e)
            csr_tlbehi_vppn <= r_tlb_vppn;
        else if (inst_tlbrd_happen && ~r_tlb_e)  //无效清零
            csr_tlbehi_vppn <= 19'b0;
    end

    wire [31:0] csr_tlbehi;
    assign csr_tlbehi = {csr_tlbehi_vppn,13'b0};
    assign tlbehi_vppn = csr_tlbehi_vppn;
    //TLBELO0
    // TLBELO0和TLBELO1两个寄存器包含了TLB指令操作时TLB表项低位部分物理页号等相关的信息。
    // 因龙芯架构32位精简版下TLB采用双页结构，所以TLB表项的低位信息对应奇偶两个物理页表项，其中
    // 偶数页信息在TLBELO0中，奇数页信息在TLBELO1中。
    //     执行TLBRD指令时，从TLB表项中读出的上述信息逐个写入到TLBELO0和TLBELO1两寄存器中的
    // 对应域中。

    reg  csr_tlbelo0_v;
    reg  csr_tlbelo0_d;
    reg  [1:0] csr_tlbelo0_plv;
    reg  [1:0] csr_tlbelo0_mat;
    reg  csr_tlbelo0_g;
    reg  [19:0] csr_tlbelo0_ppn;
    reg  csr_tlbelo1_v;
    reg  csr_tlbelo1_d;
    reg  [1:0] csr_tlbelo1_plv;
    reg  [1:0] csr_tlbelo1_mat;
    reg  csr_tlbelo1_g;
    reg  [19:0] csr_tlbelo1_ppn;

    always @(posedge clk) begin
        if (reset) begin
            csr_tlbelo0_v <= 1'b0;
            csr_tlbelo0_d <= 1'b0;
            csr_tlbelo0_plv <= 2'b0;
            csr_tlbelo0_mat <= 2'b0;
            csr_tlbelo0_g <= 1'b0;
            csr_tlbelo0_ppn <= 20'b0;
        end
        else if (csr_we && csr_num==`CSR_TLBELO0) begin
            csr_tlbelo0_v <= csr_wmask[`CSR_TLBELO_V]&csr_wdata[`CSR_TLBELO_V]
                          | ~csr_wmask[`CSR_TLBELO_V]&csr_tlbelo0_v;
            csr_tlbelo0_d <= csr_wmask[`CSR_TLBELO_D]&csr_wdata[`CSR_TLBELO_D]
                          | ~csr_wmask[`CSR_TLBELO_D]&csr_tlbelo0_d;
            csr_tlbelo0_plv <= csr_wmask[`CSR_TLBELO_PLV]&csr_wdata[`CSR_TLBELO_PLV]
                            | ~csr_wmask[`CSR_TLBELO_PLV]&csr_tlbelo0_plv;
            csr_tlbelo0_mat <= csr_wmask[`CSR_TLBELO_MAT]&csr_wdata[`CSR_TLBELO_MAT]
                            | ~csr_wmask[`CSR_TLBELO_MAT]&csr_tlbelo0_mat;
            csr_tlbelo0_g <= csr_wmask[`CSR_TLBELO_G]&csr_wdata[`CSR_TLBELO_G]
                            | ~csr_wmask[`CSR_TLBELO_G]&csr_tlbelo0_g;
            csr_tlbelo0_ppn <= csr_wmask[`CSR_TLBELO_PPN]&csr_wdata[`CSR_TLBELO_PPN]
                            | ~csr_wmask[`CSR_TLBELO_PPN]&csr_tlbelo0_ppn;
        end
        else if(inst_tlbrd_happen && r_tlb_e) begin
            csr_tlbelo0_v <= r_tlb_v0;
            csr_tlbelo0_d <= r_tlb_d0;
            csr_tlbelo0_plv <= r_tlb_plv0;
            csr_tlbelo0_mat <= r_tlb_mat0;
            csr_tlbelo0_g <= r_tlb_g;
            csr_tlbelo0_ppn <= r_tlb_ppn0;
        end
        else if(inst_tlbrd_happen && ~r_tlb_e) begin
            csr_tlbelo0_v <= 1'b0;
            csr_tlbelo0_d <= 1'b0;
            csr_tlbelo0_plv <= 2'b0;
            csr_tlbelo0_mat <= 2'b0;
            csr_tlbelo0_g <= 1'b0;
            csr_tlbelo0_ppn <= 20'b0;
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            csr_tlbelo1_v <= 1'b0;
            csr_tlbelo1_d <= 1'b0;
            csr_tlbelo1_plv <= 2'b0;
            csr_tlbelo1_mat <= 2'b0;
            csr_tlbelo1_g <= 1'b0;
            csr_tlbelo1_ppn <= 20'b0;
        end
        else if (csr_we && csr_num==`CSR_TLBELO1) begin
            csr_tlbelo1_v <= csr_wmask[`CSR_TLBELO_V]&csr_wdata[`CSR_TLBELO_V]
                          | ~csr_wmask[`CSR_TLBELO_V]&csr_tlbelo1_v;
            csr_tlbelo1_d <= csr_wmask[`CSR_TLBELO_D]&csr_wdata[`CSR_TLBELO_D]
                          | ~csr_wmask[`CSR_TLBELO_D]&csr_tlbelo1_d;
            csr_tlbelo1_plv <= csr_wmask[`CSR_TLBELO_PLV]&csr_wdata[`CSR_TLBELO_PLV]
                            | ~csr_wmask[`CSR_TLBELO_PLV]&csr_tlbelo1_plv;
            csr_tlbelo1_mat <= csr_wmask[`CSR_TLBELO_MAT]&csr_wdata[`CSR_TLBELO_MAT]
                            | ~csr_wmask[`CSR_TLBELO_MAT]&csr_tlbelo1_mat;
            csr_tlbelo1_g <= csr_wmask[`CSR_TLBELO_G]&csr_wdata[`CSR_TLBELO_G]
                            | ~csr_wmask[`CSR_TLBELO_G]&csr_tlbelo1_g;
            csr_tlbelo1_ppn <= csr_wmask[`CSR_TLBELO_PPN]&csr_wdata[`CSR_TLBELO_PPN]
                            | ~csr_wmask[`CSR_TLBELO_PPN]&csr_tlbelo1_ppn;
        end
        else if (inst_tlbrd_happen && r_tlb_e) begin
            csr_tlbelo1_v <= r_tlb_v1;
            csr_tlbelo1_d <= r_tlb_d1;
            csr_tlbelo1_plv <= r_tlb_plv1;
            csr_tlbelo1_mat <= r_tlb_mat1;
            csr_tlbelo1_g <= r_tlb_g;
            csr_tlbelo1_ppn <= r_tlb_ppn1;
        end
        else if (inst_tlbrd_happen && ~r_tlb_e) begin
            csr_tlbelo1_v <= 1'b0;
            csr_tlbelo1_d <= 1'b0;
            csr_tlbelo1_plv <= 2'b0;
            csr_tlbelo1_mat <= 2'b0;
            csr_tlbelo1_g <= 1'b0;
            csr_tlbelo1_ppn <= 20'b0;
        end
    end

    wire [31:0] csr_tlbelo0;
    wire [31:0] csr_tlbelo1;
    assign csr_tlbelo0 = {4'b0,csr_tlbelo0_ppn,1'b0,csr_tlbelo0_g,csr_tlbelo0_mat,csr_tlbelo0_plv,csr_tlbelo0_d,csr_tlbelo0_v};
    assign csr_tlbelo1 = {4'b0,csr_tlbelo1_ppn,1'b0,csr_tlbelo1_g,csr_tlbelo1_mat,csr_tlbelo1_plv,csr_tlbelo1_d,csr_tlbelo1_v};

    //ASID
    reg [9:0] csr_asid_asid;
    wire [7:0] csr_asid_asidbits;
    assign csr_asid_asidbits = 8'b1010;
    //ASID域的位宽。其直接等于这个域的数值。
        always @(posedge clk) begin
        if (reset)
            csr_asid_asid <= 10'b0;
        else if (csr_we && csr_num==`CSR_ASID)
            csr_asid_asid <= csr_wmask[`CSR_ASID_ASID]&csr_wdata[`CSR_ASID_ASID]
                          | ~csr_wmask[`CSR_ASID_ASID]&csr_asid_asid;
        else if (inst_tlbrd_happen && r_tlb_e)
            csr_asid_asid <= r_tlb_asid;
        else if (inst_tlbrd_happen && ~r_tlb_e)
            csr_asid_asid <= 10'b0;
    end

    wire [31:0] csr_asid;
    assign csr_asid = {8'b0,csr_asid_asidbits,6'b0,csr_asid_asid};
    assign csr_asid_state = csr_asid_asid;

    //TLBRENTRY
    // 该寄存器用于配置低半地址空间的全局目录的基址。要求全局目录的基址一定是4KB边界地址对齐的，
    // 所以该寄存器的最低12位软件不可配置，只读恒为0。
    reg [25:0] csr_tlbrentry_base;
        always @(posedge clk) begin
        if (reset)
            csr_tlbrentry_base <= 26'b0;
        else if (csr_we && csr_num==`CSR_TLBRENTRY)
            csr_tlbrentry_base <= csr_wmask[`CSR_TLBRENTRY_PA]&csr_wdata[`CSR_TLBRENTRY_PA]
                             | ~csr_wmask[`CSR_TLBRENTRY_PA]&csr_tlbrentry_base;
    end
    wire [31:0] csr_tlbrentry;
    assign csr_tlbrentry = {csr_tlbrentry_base,6'b0};

    //DMW0-1
    reg csr_dmw0_plv0;
    reg csr_dmw0_plv3;
    reg [1:0] csr_dmw0_mat;
    reg [2:0] csr_dmw0_pseg;
    reg [2:0] csr_dmw0_vseg;
    reg csr_dmw1_plv0;
    reg csr_dmw1_plv3;
    reg [1:0] csr_dmw1_mat;
    reg [2:0] csr_dmw1_pseg;
    reg [2:0] csr_dmw1_vseg;

 always @(posedge clk) begin
        if (reset) begin
            csr_dmw0_plv0 <= 1'b0;
            csr_dmw0_plv3 <= 1'b0;
            csr_dmw0_mat <= 2'b0;
            csr_dmw0_pseg <= 3'b0;
            csr_dmw0_vseg <= 3'b0;
        end
        else if (csr_we && csr_num==`CSR_DMW0) begin
            csr_dmw0_plv0 <= csr_wmask[`CSR_DMW_PLV0]&csr_wdata[`CSR_DMW_PLV0]
                          | ~csr_wmask[`CSR_DMW_PLV0]&csr_dmw0_plv0;
            csr_dmw0_plv3 <= csr_wmask[`CSR_DMW_PLV3]&csr_wdata[`CSR_DMW_PLV3]
                          | ~csr_wmask[`CSR_DMW_PLV3]&csr_dmw0_plv3;
            csr_dmw0_mat <= csr_wmask[`CSR_DMW_MAT]&csr_wdata[`CSR_DMW_MAT]
                          | ~csr_wmask[`CSR_DMW_MAT]&csr_dmw0_mat;
            csr_dmw0_pseg <= csr_wmask[`CSR_DMW_PSEG]&csr_wdata[`CSR_DMW_PSEG]
                          | ~csr_wmask[`CSR_DMW_PSEG]&csr_dmw0_pseg;
            csr_dmw0_vseg <= csr_wmask[`CSR_DMW_VSEG]&csr_wdata[`CSR_DMW_VSEG]
                          | ~csr_wmask[`CSR_DMW_VSEG]&csr_dmw0_vseg;
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            csr_dmw1_plv0 <= 1'b0;
            csr_dmw1_plv3 <= 1'b0;
            csr_dmw1_mat <= 2'b0;
            csr_dmw1_pseg <= 3'b0;
            csr_dmw1_vseg <= 3'b0;
        end
        else if (csr_we && csr_num==`CSR_DMW1) begin
            csr_dmw1_plv0 <= csr_wmask[`CSR_DMW_PLV0]&csr_wdata[`CSR_DMW_PLV0]
                          | ~csr_wmask[`CSR_DMW_PLV0]&csr_dmw1_plv0;
            csr_dmw1_plv3 <= csr_wmask[`CSR_DMW_PLV3]&csr_wdata[`CSR_DMW_PLV3]
                          | ~csr_wmask[`CSR_DMW_PLV3]&csr_dmw1_plv3;
            csr_dmw1_mat <= csr_wmask[`CSR_DMW_MAT]&csr_wdata[`CSR_DMW_MAT]
                          | ~csr_wmask[`CSR_DMW_MAT]&csr_dmw1_mat;
            csr_dmw1_pseg <= csr_wmask[`CSR_DMW_PSEG]&csr_wdata[`CSR_DMW_PSEG]
                          | ~csr_wmask[`CSR_DMW_PSEG]&csr_dmw1_pseg;
            csr_dmw1_vseg <= csr_wmask[`CSR_DMW_VSEG]&csr_wdata[`CSR_DMW_VSEG]
                          | ~csr_wmask[`CSR_DMW_VSEG]&csr_dmw1_vseg;
        end
    end
    wire [31:0] csr_dmw0;
    wire [31:0] csr_dmw1;
    assign csr_dmw0_plv0_state = csr_dmw0_plv0;
    assign csr_dmw0_plv3_state = csr_dmw0_plv3;
    assign csr_dmw0_pseg_state = csr_dmw0_pseg;
    assign csr_dmw0_vseg_state = csr_dmw0_vseg;
    assign csr_dmw0_mat_state = csr_dmw0_mat;
    assign csr_dmw1_plv0_state = csr_dmw1_plv0;
    assign csr_dmw1_plv3_state = csr_dmw1_plv3;
    assign csr_dmw1_pseg_state = csr_dmw1_pseg;
    assign csr_dmw1_vseg_state = csr_dmw1_vseg;
    assign csr_dmw1_mat_state = csr_dmw1_mat;
    assign csr_dmw0 = {csr_dmw0_vseg,1'b0,csr_dmw0_pseg,19'b0,csr_dmw0_mat,csr_dmw0_plv3,2'b0,csr_dmw0_plv0};
    assign csr_dmw1 = {csr_dmw1_vseg,1'b0,csr_dmw1_pseg,19'b0,csr_dmw1_mat,csr_dmw1_plv3,2'b0,csr_dmw1_plv0};

    //现在完成了寄存器的编写，再来完善对应的tlb读写相关端口
    assign csr_tlbidx_index_state = csr_tlbidx_index;
    //读端口，对应tlbrd指令 
    //无需多余接入输入

    //写端口，对应tlbwr和tlbfill+指令
    assign w_tlb_e = csr_estat_ecode == 6'b111111 ? 1 : ~csr_tlbidx_ne;
    assign w_tlb_ps = csr_tlbidx_ps;
    assign w_tlb_vppn = csr_tlbehi_vppn;
    assign w_tlb_asid = csr_asid_asid;
    assign w_tlb_g = csr_tlbelo0_g & csr_tlbelo1_g; 
    //执行TLBRD指令时，当所读取的TLB表项的G位为1，则TLBELO0和TLBELO1中
    // 的G位被同时置为1
    assign w_tlb_ppn0 = csr_tlbelo0_ppn;
    assign w_tlb_plv0 = csr_tlbelo0_plv;
    assign w_tlb_mat0 = csr_tlbelo0_mat;
    assign w_tlb_d0 = csr_tlbelo0_d;
    assign w_tlb_v0 = csr_tlbelo0_v;
    assign w_tlb_ppn1 = csr_tlbelo1_ppn;
    assign w_tlb_plv1 = csr_tlbelo1_plv;
    assign w_tlb_mat1 = csr_tlbelo1_mat;
    assign w_tlb_d1 = csr_tlbelo1_d;
    assign w_tlb_v1 = csr_tlbelo1_v;

assign csr_rvalue =     {32{(csr_num == `CSR_CRMD)}} & csr_crmd
                        |{32{(csr_num == `CSR_PRMD)}} & csr_prmd
                        |{32{(csr_num == `CSR_ESTAT)}} & csr_estat
                        |{32{(csr_num == `CSR_ERA)}} & csr_era_pc
                        |{32{(csr_num == `CSR_EENTRY)}} & csr_eentry
                        |{32{(csr_num == `CSR_SAVE0)}} & csr_save0_data
                        |{32{(csr_num == `CSR_SAVE1)}} & csr_save1_data
                        |{32{(csr_num == `CSR_SAVE2)}} & csr_save2_data
                        |{32{(csr_num == `CSR_SAVE3)}} & csr_save3_data
                        |{32{(csr_num == `CSR_ECFG)}} & csr_ecfg
                        |{32{(csr_num == `CSR_BADV)}} & csr_badv
                        |{32{(csr_num == `CSR_TID)}} & csr_tid_tid
                        |{32{(csr_num == `CSR_TCFG)}} & csr_tcfg
                        |{32{(csr_num == `CSR_TVAL)}} & csr_tval
                        |{32{(csr_num == `CSR_TICLR)}} & csr_ticlr
                        |{32{(csr_num == `CSR_TLBIDX)}} & csr_tlbidx
                        |{32{(csr_num == `CSR_TLBEHI)}} & csr_tlbehi
                        |{32{(csr_num == `CSR_TLBELO0)}} & csr_tlbelo0
                        |{32{(csr_num == `CSR_TLBELO1)}} & csr_tlbelo1
                        |{32{(csr_num == `CSR_ASID)}} & csr_asid
                        |{32{(csr_num == `CSR_TLBRENTRY)}} & csr_tlbrentry
                        |{32{(csr_num == `CSR_DMW0)}} & csr_dmw0
                        |{32{(csr_num == `CSR_DMW1)}} & csr_dmw1;
    assign ex_entry = (wb_ecode == 6'b111111) ? csr_tlbrentry : csr_eentry ;
    assign ertn_entry = csr_era_pc;
    assign has_int = ((csr_estat_is[12:0] & csr_ecfg_lie[12:0]) != 13'b0)
              && (csr_crmd_ie == 1'b1);

endmodule