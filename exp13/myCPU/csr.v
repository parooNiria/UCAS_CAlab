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
    `define CSR_SAVE0  14'h0030
    `define CSR_SAVE1  14'h0031
    `define CSR_SAVE2  14'h0032
    `define CSR_SAVE3  14'h0033  
    `define CSR_TID    14'h0040
    `define CSR_TCFG   14'h0041
    `define CSR_TVAL   14'h0042
    `define CSR_TICLR  14'h0044

    `define CSR_CRMD_PLV  1:0
    `define CSR_CRMD_PIE   2
    `define CSR_PRMD_PPLV 1:0
    `define CSR_PRMD_PIE  2
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

    `define ECODE_ADE    6'h08
    `define ECODE_ALE    6'h09
    `define ESUBCODE_ADEF 9'h00
//CSR registers
    //CRMD
    wire [31:0] csr_crmd;
    reg [1:0]  csr_crmd_plv;
    reg        csr_crmd_ie;
    wire       csr_crmd_da;
    wire       csr_crmd_pg;
    wire [1:0] csr_crmd_datf;
    wire [1:0] csr_crmd_datm;
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

    assign csr_crmd_da = 1'b1;
    assign csr_crmd_pg = 1'b0;
    assign csr_crmd_datf = 2'b0;
    assign csr_crmd_datm = 2'b0;
    assign csr_crmd = {23'b0,csr_crmd_datm,csr_crmd_datf,csr_crmd_pg,csr_crmd_da,
                       csr_crmd_ie,csr_crmd_plv};


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
    wire wb_ex_addr_err = wb_ecode == `ECODE_ADE || wb_ecode == `ECODE_ALE;
    always@(posedge clk) begin
        if(wb_ex && wb_ex_addr_err) begin
            csr_badv <= (wb_ecode == `ECODE_ADE && wb_esubcode == `ESUBCODE_ADEF ? wb_pc : wb_vaddr);
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
                        |{32{(csr_num == `CSR_TICLR)}} & csr_ticlr;

    assign ex_entry = csr_eentry;
    assign ertn_entry = csr_era_pc;
    assign has_int = ((csr_estat_is[12:0] & csr_ecfg_lie[12:0]) != 13'b0)
              && (csr_crmd_ie == 1'b1);

endmodule