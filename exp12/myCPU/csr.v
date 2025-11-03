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
    // input  wire [7:0]   hw_int_in,
    // input  wire         ipi_ini_in,
    input  wire [5:0]   wb_ecode,
    input  wire [8:0]   wb_esubcode,

    output wire [31:0]  ex_entry,
    output wire [31:0]  ertn_entry

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
    `define CSR_TICLR_CLR 11
    `define CSR_TID_TID   31:0
    `define CSR_TCFG_EN   0
    `define CSR_TCFG_PERIOD 1
    `define CSR_TCFG_INITV 31:2
    `define CSR_TCFG_INITVAL 31:2

    `define ECODE_ADE    6'h08
    `define ECODE_ALE    6'h09

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

        // csr_estat_is[9:2] <= hw_int_in;
        csr_estat_is[9:2] <= 8'b0;
        csr_estat_is[10] <= 1'b0;

        // if(timer_cnt[31:0] == 32'b0)
        //     csr_estat_is[11] <= 1'b1;
        // else 
        if(csr_we && csr_num == `CSR_ESTAT && csr_wmask[`CSR_TICLR_CLR] && csr_wdata[`CSR_TICLR_CLR]) begin
            csr_estat_is[11] <= 1'b0;
        end
        else begin
            csr_estat_is[11] <= 1'b0;
        end

        // csr_estat_is[12] <= ipi_ini_in;
        csr_estat_is[12] <= 1'b0;
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

    assign csr_rvalue = {32{(csr_num == `CSR_CRMD)}} & csr_crmd
                        |{32{(csr_num == `CSR_PRMD)}} & csr_prmd
                        |{32{(csr_num == `CSR_ESTAT)}} & csr_estat
                        |{32{(csr_num == `CSR_ERA)}} & csr_era_pc
                        |{32{(csr_num == `CSR_EENTRY)}} & csr_eentry
                        |{32{(csr_num == `CSR_SAVE0)}} & csr_save0_data
                        |{32{(csr_num == `CSR_SAVE1)}} & csr_save1_data
                        |{32{(csr_num == `CSR_SAVE2)}} & csr_save2_data
                        |{32{(csr_num == `CSR_SAVE3)}} & csr_save3_data;

    assign ex_entry = csr_eentry;
    assign ertn_entry = csr_era_pc;

    //TVAL
    // reg [31:0] timer_cnt;
    // always@(posedge clk) begin
    //     if(reset) begin
    //         timer_cnt <= 32'hffff_ffff;
    //     end
    //     else if(csr_we && csr_num == `CSR_TVAL) begin
    //         timer_cnt <= (csr_wdata & csr_wmask)
    //                      | (timer_cnt & ~csr_wmask);
    //     end
    //     else if(timer_cnt != 32'b0) begin
    //         timer_cnt <= timer_cnt - 1;
    //     end
    // end

endmodule