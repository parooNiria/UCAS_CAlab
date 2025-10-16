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
    input [1:0]     mem_en_from_id,
    input           div_en_from_id,
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
    reg [2:0]     current_state;
    always @(posedge clk) begin
        if (reset) begin
            current_state <= 3'b001;
        end
        else if (div_en_from_id & allow_in & ready_go_id) begin
            current_state <= 3'b010;
        end
        else if(current_state[1]&commit) begin
            current_state <= 3'b100;
        end
        else if(current_state[2]&div_result_ready) begin
            current_state <= 3'b001;
        end
    end
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
    assign tvalid_div_signed = current_state[1] & signed_div;
    assign tvalid_div_unsigned = current_state[1] & ~signed_div;
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
                    (div_en_reg & valid) ? alu_result_with_div : alu_result_without_div_mul;

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
    reg [1:0]  mem_en_reg;
    reg        div_en_reg;
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
            div_en_reg <= 1'b0;
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
        end
    end
    wire mul_type;
    assign mul_type = alu_op_reg[12]|alu_op_reg[13]|alu_op_reg[14];
    assign dest       = dest_reg;
    assign allow_in   = ~valid | (ready_go&&MEM_allow_in);
    assign ready_go   = valid&((!div_en_reg) | (current_state==3'b100&&div_result_ready));
    assign inst_exe   = inst_reg;
    assign pc_exe     = pc_reg;
    assign data_sram_en    = valid;
    assign data_sram_we    = (mem_en_reg==2'b11 ? 4'b1111:0)&{4{valid}};
    assign data_sram_addr  = alu_result;
    assign data_sram_wdata = rdata2_reg;
    assign mem_ld     = (mem_en_reg==2'b01);
    assign forward_data_exe = alu_result_without_div_mul;
    assign forward_en_exe = (mem_en_reg!=2'b01)&&(div_en_reg==1'b0)&&(mul_type==1'b0);
endmodule