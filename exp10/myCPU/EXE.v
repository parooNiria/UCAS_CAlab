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
    input [11:0]    alu_op_from_id,
    input [4:0]     dest_from_id,
    input           reg_en_from_id,
    input [1:0]     mem_en_from_id,
    input [31:0]    rdata2_from_id,
    input [31:0]    rdata1_from_id,
    //to MEM
    output          ready_go,
    input           MEM_allow_in,
    output [31:0]   inst_exe,
    output [31:0]   pc_exe,
    output [31:0]   alu_result,
    output          reg_en,
    output          mem_ld,
    output [4:0]    dest,
    //to dram
    output wire        data_sram_en,
    output wire [3:0]  data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    //valid
    output reg       valid,
    //bypass
    output [31:0] forward_data_exe,
    output        forward_en_exe
);
    always @(posedge clk) begin
        if (reset) begin
            valid <= 1'b0;
        end
        else if (ready_go_id &allow_in) begin
            valid <= 1'b1;
        end
        else if (ready_go &MEM_allow_in) begin
            valid <= 1'b0;
        end
    end
    alu u_alu(
    .alu_op     (alu_op_reg   ),
    .alu_src1   (src1_reg   ),
    .alu_src2   (src2_reg   ),
    .alu_result (alu_result     )
    );
    assign reg_en     = reg_en_reg;
    reg [31:0] rdata2_reg;
    reg [31:0] rdata1_reg;
    reg [31:0] inst_reg;
    reg [31:0] pc_reg;
    reg [31:0] src1_reg;
    reg [31:0] src2_reg;
    reg [11:0] alu_op_reg;
    reg [4:0]  dest_reg;
    reg        reg_en_reg;
    reg [1:0]  mem_en_reg;
    always @(posedge clk) begin
        if (reset) begin
            inst_reg   <= 32'b0;
            pc_reg     <= 32'b0;
            src1_reg   <= 32'b0;
            src2_reg   <= 32'b0;
            alu_op_reg <= 12'b0;
            dest_reg   <= 5'b0;
            reg_en_reg <= 1'b0;
            mem_en_reg <= 2'b0;
            rdata2_reg <= 32'b0;
            rdata1_reg <= 32'b0;
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
        end
    end

    assign dest       = dest_reg;
    assign allow_in   = ~valid | (ready_go&&MEM_allow_in);
    assign ready_go   = valid;
    assign inst_exe   = inst_reg;
    assign pc_exe     = pc_reg;
    assign data_sram_en    = valid;
    assign data_sram_we    = (mem_en_reg==2'b11 ? 4'b1111:0)&{4{valid}};
    assign data_sram_addr  = alu_result;
    assign data_sram_wdata = rdata2_reg;
    assign mem_ld     = (mem_en_reg==2'b01);
    assign forward_data_exe = alu_result;
    assign forward_en_exe = (mem_en_reg!=2'b01);
endmodule