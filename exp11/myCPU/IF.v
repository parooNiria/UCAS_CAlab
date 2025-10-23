module IF(
    input         clk,
    input         reset,
    // inst sram interface
    output        inst_sram_en   ,
    output [ 3:0] inst_sram_wen  ,
    output [31:0] inst_sram_addr ,
    output [31:0] inst_sram_wdata,
    input  [31:0] inst_sram_rdata, 

    //to ID
    output        ready_go       ,
    input         allow_in       ,
    output [31:0] inst_if        ,
    output [31:0] pc_if          ,

    input         flush          ,
    input  [31:0] newpc               
);  
    wire       handshake;
    assign handshake = ready_go & allow_in;
    //initial cpu && IF valid state Part
    reg         valid;
    reg         start;
    reg         before_first_inst;
    //simply pc = 32'h1bfffffc
    wire        to_valid;
    //start cpu
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
            before_first_inst <= 1'b0;
        end
        else if (start) begin
            before_first_inst <= 1'b1;
        end
        else 
            before_first_inst <= 1'b0;
    end
    assign to_valid = ~start;//if idram needs more cycles, could change here
    always @(posedge clk) begin
        if (reset) 
            valid <= 1'b0;
        else if(pc_update)
            valid <= to_valid;
        else if (flush)
            valid <= 1'b0;//flush not used here,just for future update
    end

    //pc part
    reg  [31:0] pc;
    wire [31:0] nextpc;
    reg  keep;
    wire pc_update;
    assign pc_update = (ready_go & allow_in)|flush|before_first_inst;
    always @(posedge clk) begin
        if (reset) begin
            pc <= 32'h1bfffffc;     //trick: to make nextpc be 0x1c000000 during reset

        end
        else if (pc_update) begin
            pc <= nextpc;
        end
    end
    //when pc update,inst from ram now needs to be kept
    //considering inst come from ram after one cycle delay than pc change
    //so we need a keep signal to tell if we need to keep the inst from last cycle
    always @(posedge clk) begin
        if (reset) begin
            keep <= 1'b0;
        end
        else if (pc_update) begin
            keep <= 1'b1;
        end
        else 
            keep <= 1'b0;
    end
    reg [31:0] inst_keep;
    always @(posedge clk) begin
        if (reset) begin
            inst_keep <= 32'b0;
        end
        else if (keep) begin
            inst_keep <= inst_sram_rdata;
        end
    end
//pre_if
    wire [31:0] seq_pc;
    assign seq_pc = pc + 4;
    assign inst_sram_en    = valid | before_first_inst;
    assign inst_sram_wen   = 4'b0;
    assign inst_sram_addr  = nextpc;
    assign inst_sram_wdata = 32'b0;
    assign nextpc = flush ? newpc : seq_pc;

//if_id
    assign ready_go = ~flush&valid;
    assign inst_if   = ~keep ? inst_keep : inst_sram_rdata;
    assign pc_if     = pc;

endmodule