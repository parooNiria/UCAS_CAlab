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

    input         flush          ,
    input  [31:0] newpc          ,

    input         ertn_flush,
    input         wb_ex,
    input  [31:0] ex_entry,
    input  [31:0] ertn_entry 
);  
    reg    valid_pre_if;
//pre-if
    reg    start;
    reg   [31:0] preif_pc;
    wire  [31:0] next_preif_pc;
    wire  preif_pc_update;
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

    always @(posedge clk) begin
        if (reset) begin
            preif_pc <= 32'h1c000000;
        end
        else if (preif_pc_update) begin
            preif_pc <= next_preif_pc;
        end
    end
    assign preif_pc_update = (((inst_sram_addr_ok&inst_sram_req) | wb_ex | ertn_flush | flush) & valid_pre_if);
    assign next_preif_pc = wb_ex ? ex_entry :
                           ertn_flush ? ertn_entry :
                           flush ? newpc :
                           preif_pc + 4;
    assign inst_sram_addr  = {preif_pc[31:2],2'b00};
    assign inst_sram_req   = valid_pre_if&allow_in_if&~flush&~ertn_flush&~wb_ex;
    assign inst_sram_wr    = 1'b0;
    assign inst_sram_size  = 2'b10;
    assign inst_sram_wstrb = 4'b0;
    
    wire   pre_if_ready_go;
    assign pre_if_ready_go = inst_sram_addr_ok & valid_pre_if & ~flush & ~ertn_flush & ~wb_ex;


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

    reg valid_if;
    always @(posedge clk) begin
        if (reset) begin
            valid_if <= 1'b0;
        end
        else if (pre_if_ready_go&allow_in_if) begin
            valid_if <= 1'b1;
        end
        else if (ready_go&allow_in_id |wb_ex|flush|ertn_flush) begin
            valid_if <= 1'b0;
        end
    end

    reg wait_data;
    always @(posedge clk) begin
        if(reset)
            wait_data <= 1'b0;
        else if (~get_data_state&valid_if&(wb_ex|flush|ertn_flush))  begin
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
        else if ( valid_if & ready_go & allow_in_id | ertn_flush | flush | wb_ex) begin
            if_already_recv_inst <= 1'b0;
        end
        else if (inst_sram_data_ok&valid_if &~wait_data) begin
            if_already_recv_inst <= 1'b1;
        end
    end

    assign allow_in_if = ~valid_if | (ready_go & allow_in_id) | wb_ex | flush | ertn_flush;
    wire get_data_state;
    assign get_data_state = ((~wait_data & inst_sram_data_ok)| if_already_recv_inst)&valid_if;
    assign ready_go    = get_data_state&(~wb_ex)&(~flush)&(~ertn_flush);
    assign inst_if   = (inst_sram_data_ok) ? inst_sram_rdata:inst_sram_rdata_reg;
    assign pc_if     = pc;
    assign exception_adef = (pc[1:0] != 2'b0) & valid_if;

endmodule