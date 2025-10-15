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
    output [31:0] forward_data_wb
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
    always @(posedge clk) begin
        if (reset) begin
            data_reg   <= 32'b0;
            dest_reg   <= 5'b0;
            reg_en_reg <= 1'b0;
        end
        else if (ready_go_mem &allow_in) begin
            data_reg   <= data_to_reg_from_mem;
            dest_reg   <= dest_from_mem;
            reg_en_reg <= reg_en_from_mem;
        end
    end

    assign allow_in = 1'b1;
    assign we       = valid & reg_en_reg;
    assign waddr    = dest_reg;
    assign wdata    = data_reg;
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
endmodule