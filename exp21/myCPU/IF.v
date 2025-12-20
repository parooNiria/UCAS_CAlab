module IF(
    input         clk,
    input         reset,
    // inst sram interface
    output wire        inst_sram_req,
    output wire        inst_sram_wr,
    output wire [1:0]  inst_sram_size,
    output wire [3:0]  inst_sram_wstrb,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire        inst_sram_addr_ok,
    input  wire        inst_sram_data_ok,
    input  wire [31:0] inst_sram_rdata,

    //to ID
    output        ready_go       ,
    input         allow_in_id    ,
    output [31:0] inst_if        ,
    output [31:0] pc_if          ,
    output        exception_adef ,
    output        exception_pif  ,
    output        exception_tlbr_fetch,
    output        exception_ppi ,

    input         flush          ,
    input  [31:0] newpc          ,

    input         ertn_flush,
    input         wb_ex, //对于tlb例外入口和普通入口，虽然二者是通过不同的寄存器导致的，但是对外部是没区别的，因此可以在csr内部中做选择
    input  [31:0] ex_entry,
    input  [31:0] ertn_entry,

    // tlb
    output wire [18:0] s0_vppn,         //虚拟页号
    output wire        s0_va_bit12,     //虚拟地址第12位
    input  wire        s0_found,        //TLB查找是否命中
    input  wire [3:0] s0_index,//命中条目索引
    input  wire [19:0] s0_ppn,          //命中的物理页号
    input  wire [ 5:0] s0_ps,           //页面大小
    input  wire [ 1:0] s0_plv,          //权限级别
    input  wire        s0_v,            //页面是否有效
    input  wire [ 1:0] crmd_plv,         //当前特权级别
    // DMW0 
    input  wire        csr_dmw0_plv0,   //用户级别0
    input  wire        csr_dmw0_plv3,   //用户级别3
    input  wire [ 2:0] csr_dmw0_pseg,   //直接映射的物理地址段
    input  wire [ 2:0] csr_dmw0_vseg,   //直接映射的虚拟地址段
    // DMW1
    input  wire        csr_dmw1_plv0,   
    input  wire        csr_dmw1_plv3,
    input  wire [ 2:0] csr_dmw1_pseg,
    input  wire [ 2:0] csr_dmw1_vseg,
    //此时需要一个信号表示此时处于什么翻译模式，选择在csr中直接产生（本来由pa和da信号共同决定）
    input  wire        csr_addr_mode,  //1表示当前为虚地址模式，0表示为物理地址模式
    //虚实地址映射标记信号
    input  wire        vaddr_sign_from_id,
    input  wire        vaddr_sign_from_ex,
    input  wire        vaddr_sign_from_mem,
    input  wire        vaddr_sign_from_wb,
    //虚实地址标记的指令到达对应级别，需要取消当前的指令的有效性的信号
    input  wire        invalidtlb_happen_in_ex,
    input  wire        vaddr_about_inst_happen_in_wb,       //此处是应该能做一些优化的，比如不是每一级都接受下面所有级的信号，而是一级一级向上传递，但是由于对于性能没有什么要求
                                                           //暂且就这样实现，主要是这样的扩展性是比较好的
    input  wire [31:0] invalidtlb_pc,
    input  wire [31:0] vaddr_about_inst_pc    //对于标记重取法的下一条指令都是产生标记的指令pc + 4,此时需要对应的pc送入IF级重新取指

);  
    reg    valid_pre_if;
//pre-if
    reg    start;
    reg   [31:0] preif_pc;
    wire  [31:0] next_preif_pc;
    wire  preif_pc_update;
    //开机逻辑
    always @(posedge clk) begin
        if (reset) begin
            start <= 1'b1;
        end
        else begin
            start <= 1'b0;
        end
    end
    
    always @(posedge clk) begin
        if (reset) begin
            valid_pre_if <= 1'b0;
        end
        else if (start) begin
            valid_pre_if <= 1'b1;
        end
    end
    //开机逻辑结束
    wire vaddr_sign = vaddr_sign_from_id | vaddr_sign_from_ex | vaddr_sign_from_mem | vaddr_sign_from_wb;
    wire cancel_because_of_vaddr = invalidtlb_happen_in_ex | vaddr_about_inst_happen_in_wb;

    always @(posedge clk) begin
        if (reset) begin
            preif_pc <= 32'h1c000000;
        end
        else if (preif_pc_update) begin
            preif_pc <= next_preif_pc;
        end
    end    //对于标记重取法的下一条指令都是产生标记的指令pc + 4
    assign preif_pc_update = (((inst_sram_addr_ok&inst_sram_req) | wb_ex | ertn_flush | flush | vaddr_about_inst_happen_in_wb | invalidtlb_happen_in_ex) & valid_pre_if);
    assign next_preif_pc = wb_ex ? ex_entry :
                           ertn_flush ? ertn_entry :
                            vaddr_about_inst_happen_in_wb ? (vaddr_about_inst_pc + 4) :
                            invalidtlb_happen_in_ex ? (invalidtlb_pc + 4) :
                           flush ? newpc :
                           preif_pc + 4;
    //这个地方需要予以改变适应地址翻译模式
    //地址翻译模块
    //直接地址映射
    wire DMW0_hit = ((csr_addr_mode == 1'b1) && 
                      ((crmd_plv == 2'b00 && csr_dmw0_plv0) || (crmd_plv == 2'b11 && csr_dmw0_plv3)) && 
                      (preif_pc[31:29] == csr_dmw0_vseg));

    wire DMW1_hit = ((csr_addr_mode == 1'b1) && 
                      ((crmd_plv == 2'b00 && csr_dmw1_plv0) || (crmd_plv == 2'b11 && csr_dmw1_plv3)) && 
                      (preif_pc[31:29] == csr_dmw1_vseg));
    wire [31:0] DMW0_addr_translated;
    wire [31:0] DMW1_addr_translated;
    assign DMW0_addr_translated = {csr_dmw0_pseg, preif_pc[28:0]};
    assign DMW1_addr_translated = {csr_dmw1_pseg, preif_pc[28:0]};
    //TLB映射
    assign {s0_vppn, s0_va_bit12} = {preif_pc[31:13], preif_pc[12]};
    wire [31:0] tlb_addr_translated;
    assign tlb_addr_translated = (s0_ps == 6'd22) ? {s0_ppn[19:10], preif_pc[21:0]} : {s0_ppn, preif_pc[11:0]};
    //最终的地址选择
    assign inst_sram_addr = DMW0_hit ? DMW0_addr_translated :
                            DMW1_hit ? DMW1_addr_translated :
                            (csr_addr_mode == 1'b0) ? preif_pc : //物理地址模式
                            tlb_addr_translated; //tlb地址转换
    //这个地方还可能出现例外，需要在后续加入PIF(取指异常),PPI（权限异常）,TLB_TLBR_FECTH（重填异常）
    //当出现这三种异常时，不需要发出取指请求，流入下一级，下一级也可直接流出
    //问题在于，从preif到if时，if需要知道此时这条指令不需要等待数据返回,主要是要与取消配合
    wire preif_exception_tlbr_fecth = valid_pre_if & csr_addr_mode & ~DMW0_hit & ~DMW1_hit & ~s0_found;
    wire preif_exception_pif = valid_pre_if & csr_addr_mode & ~DMW0_hit & ~DMW1_hit & s0_found & ~s0_v;
    wire preif_exception_ppi = valid_pre_if & csr_addr_mode & ~DMW0_hit & ~DMW1_hit & s0_found & s0_v & (crmd_plv > s0_plv);
    wire preif_exception_because_of_tlb = preif_exception_tlbr_fecth | preif_exception_pif | preif_exception_ppi;
    wire preif_excepetion_adef = valid_pre_if & (preif_pc[1:0] != 2'b00) & ~preif_exception_because_of_tlb;
    assign inst_sram_req   = valid_pre_if & allow_in_if & ~flush & ~ertn_flush & ~wb_ex & ~vaddr_sign & ~preif_exception_because_of_tlb & ~preif_excepetion_adef;
    assign inst_sram_wr    = 1'b0;
    assign inst_sram_size  = 2'b10;
    assign inst_sram_wstrb = 4'b0;
    
    wire   pre_if_ready_go;
    assign pre_if_ready_go = ((inst_sram_addr_ok| preif_exception_because_of_tlb | preif_excepetion_adef) & valid_pre_if & ~flush & ~ertn_flush & ~wb_ex & ~vaddr_sign);


//if
    reg [31:0] pc;
    always @(posedge clk) begin
        if (reset) begin
            pc <= 32'h1bfffffc;
        end
        else if (pre_if_ready_go&allow_in_if) begin
            pc <= preif_pc;
        end
    end

    reg exception_tlbr_fetch_reg    ;
    reg exception_pif_reg;
    reg exception_ppi_reg;
    reg exception_adef_reg;
    always @(posedge clk) begin
        if (reset) begin
            exception_tlbr_fetch_reg <= 1'b0;
            exception_pif_reg <= 1'b0;
            exception_ppi_reg <= 1'b0;
            exception_adef_reg <= 1'b0;
        end
        else if (pre_if_ready_go&allow_in_if) begin
            exception_tlbr_fetch_reg <= preif_exception_tlbr_fecth;
            exception_pif_reg <= preif_exception_pif;
            exception_ppi_reg <= preif_exception_ppi;
            exception_adef_reg <= preif_excepetion_adef;
        end
        else if (ready_go&allow_in_id |wb_ex|flush|ertn_flush|cancel_because_of_vaddr) begin
            exception_tlbr_fetch_reg <= 1'b0;
            exception_pif_reg <= 1'b0;
            exception_ppi_reg <= 1'b0;
            exception_adef_reg <= 1'b0;
        end
    end

    wire exception_because_of_tlb = exception_tlbr_fetch_reg | exception_pif_reg | exception_ppi_reg;
    reg valid_if;
    always @(posedge clk) begin
        if (reset) begin
            valid_if <= 1'b0;
        end
        else if(wb_ex|ertn_flush|cancel_because_of_vaddr) begin
            valid_if <= 1'b0;
        end
        else if (pre_if_ready_go&allow_in_if) begin
            valid_if <= 1'b1;
        end
        else if (ready_go&allow_in_id | wb_ex|flush|ertn_flush|cancel_because_of_vaddr) begin
            valid_if <= 1'b0;
        end
    end
//如果当前是exception_because_of_tlb是不需要等待一个多余数据返回的
    reg wait_data;
    always @(posedge clk) begin
        if(reset)
            wait_data <= 1'b0;
        else if (~get_data_state&valid_if&(wb_ex|flush|ertn_flush|cancel_because_of_vaddr)&~exception_because_of_tlb & ~exception_adef_reg)  begin
            wait_data <= 1'b1;
        end
        else if(inst_sram_data_ok)
            wait_data <= 1'b0;
    end

    reg [31:0] inst_sram_rdata_reg;
    always @(posedge clk) begin
        if (reset) begin
            inst_sram_rdata_reg <= 32'b0;
        end
        else if (inst_sram_data_ok) begin
            inst_sram_rdata_reg <= inst_sram_rdata;
        end
    end

    reg if_already_recv_inst;
    always @(posedge clk) begin
        if (reset) begin
            if_already_recv_inst <= 1'b0;
        end
        else if ( valid_if & ready_go & allow_in_id | ertn_flush | flush | wb_ex |cancel_because_of_vaddr) begin
            if_already_recv_inst <= 1'b0;
        end
        else if (inst_sram_data_ok&valid_if &~wait_data) begin
            if_already_recv_inst <= 1'b1;
        end
    end

    assign allow_in_if = ~valid_if | (ready_go & allow_in_id) | wb_ex | flush | ertn_flush | cancel_because_of_vaddr;
    wire get_data_state;
    assign get_data_state = ((~wait_data & inst_sram_data_ok)| if_already_recv_inst)&valid_if;
    assign ready_go    = valid_if & (((get_data_state|exception_because_of_tlb)&(~wb_ex)&(~flush)&(~ertn_flush))|(cancel_because_of_vaddr) | exception_adef_reg);
    assign inst_if   = (inst_sram_data_ok) ? inst_sram_rdata:inst_sram_rdata_reg;
    assign pc_if     = pc;
    assign exception_adef = exception_adef_reg;
    assign exception_tlbr_fetch = exception_tlbr_fetch_reg;
    assign exception_pif = exception_pif_reg;
    assign exception_ppi = exception_ppi_reg;

endmodule