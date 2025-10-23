module ID(
    input           clk,
    input           reset,
    //IF_ID
    input           if_ready,
    output          allow_in,
    input [31:0]    inst_from_if,
    input [31:0]    pc_from_if,
    output          flush,
    output [31:0]   newpc,
    //to EX
    output          ready_go,
    input           EX_allow_in,
    output [31:0]   inst_id,
    output [31:0]   pc_id,
    output [18:0]   alu_op_id,
    output [4:0]    dest,
    output [31:0]   src1,
    output [31:0]   src2,
    output          reg_en,
    output [1:0]    mem_en,
    output          div_en, 
    output [31:0]   rdata1_to_exe,//spare for inst store 
    output [31:0]   rdata2_to_exe, //not used now,spare for future
    //to regfile
    output [4:0]    raddr1,
    input [31:0]    rdata1,
    output [4:0]    raddr2,
    input [31:0]    rdata2,
    //data conflict
    input [4:0]     ex_dest,
    input           ex_reg_en_valid,
    input [4:0]     mem_dest,
    input           mem_reg_en_valid,
    input [4:0]     wb_dest,
    input           wb_reg_en_valid,
    //bypass
    input [31:0]   forward_data_from_exe,
    input [31:0]   forward_data_from_mem,
    input [31:0]   forward_data_from_wb,
    input          forward_en_from_exe
);

    wire [63:0] op_31_26_d;
    wire [15:0] op_25_22_d;
    wire [ 3:0] op_21_20_d;
    wire [31:0] op_19_15_d;
    wire [ 5:0] op_31_26;
    wire [ 3:0] op_25_22;
    wire [ 1:0] op_21_20;
    wire [ 4:0] op_19_15;
    wire [ 4:0] rd;
    wire [ 4:0] rj;
    wire [ 4:0] rk;
    wire [11:0] i12;
    wire [19:0] i20;
    wire [15:0] i16;
    wire [25:0] i26;
    wire        inst_add_w;
    wire        inst_sub_w;
    wire        inst_slt;
    wire        inst_sltu;
    wire        inst_nor;
    wire        inst_and;
    wire        inst_or;
    wire        inst_xor;
    wire        inst_slli_w;
    wire        inst_srli_w;
    wire        inst_srai_w;
    wire        inst_addi_w;
    wire        inst_ld_w;
    wire        inst_st_w;
    wire        inst_jirl;
    wire        inst_b;
    wire        inst_bl;
    wire        inst_beq;
    wire        inst_bne;
    wire        inst_lu12i_w;
    wire        inst_slti;
    wire        inst_sltiu;
    wire        inst_andi;
    wire        inst_ori;
    wire        inst_xori;
    wire        inst_sll_w;
    wire        inst_srl_w;
    wire        inst_sra_w;
    wire        inst_pcaddu12i;
    wire        inst_mul_w;
    wire        inst_mulh_w;
    wire        inst_mulh_wu;
    wire        inst_div_w;
    wire        inst_mod_w;
    wire        inst_div_wu;
    wire        inst_mod_wu;

    wire        need_ui5;
    wire        need_si12;
    wire        need_si16;
    wire        need_si20;
    wire        need_si26;
    wire        src2_is_4;
    wire        handshake_fd;
    wire        handshake_de;
    assign handshake_fd = if_ready & allow_in;
    assign handshake_de = ready_go & EX_allow_in;
    reg         valid;
    always@(posedge clk) begin
        if (reset) begin
            valid <= 1'b0;
        end
        else if (handshake_fd) begin
            valid <= 1'b1;
        end
        else if (handshake_de) begin
            valid <= 1'b0;
        end
    end

    reg [31:0] inst_reg;
    always @(posedge clk) begin
        if (reset) begin
            inst_reg <= 32'b0;
        end
        else if (allow_in && if_ready) begin
            inst_reg <= inst_from_if;
        end
    end
    assign inst_id = inst_reg;
    //decode part
    assign op_31_26  = inst_reg[31:26];
    assign op_25_22  = inst_reg[25:22];
    assign op_21_20  = inst_reg[21:20];
    assign op_19_15  = inst_reg[19:15];

    assign rd   = inst_reg[ 4: 0];
    assign rj   = inst_reg[ 9: 5];
    assign rk   = inst_reg[14:10];

    assign i12  = inst_reg[21:10];
    assign i20  = inst_reg[24: 5];
    assign i16  = inst_reg[25:10];
    assign i26  = {inst_reg[ 9: 0], inst_reg[25:10]};

    decoder_6_64 u_dec0(.in(op_31_26 ), .out(op_31_26_d ));
    decoder_4_16 u_dec1(.in(op_25_22 ), .out(op_25_22_d ));
    decoder_2_4  u_dec2(.in(op_21_20 ), .out(op_21_20_d ));
    decoder_5_32 u_dec3(.in(op_19_15 ), .out(op_19_15_d ));

    assign inst_add_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h00];
    assign inst_sub_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h02];
    assign inst_slt    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h04];
    assign inst_sltu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h05];
    assign inst_nor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h08];
    assign inst_and    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h09];
    assign inst_or     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0a];
    assign inst_xor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0b];
    assign inst_slli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h01];
    assign inst_srli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h09];
    assign inst_srai_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h11];
    assign inst_addi_w = op_31_26_d[6'h00] & op_25_22_d[4'ha];
    assign inst_ld_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h2];
    assign inst_st_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h6];
    assign inst_jirl   = op_31_26_d[6'h13];
    assign inst_b      = op_31_26_d[6'h14];
    assign inst_bl     = op_31_26_d[6'h15];
    assign inst_beq    = op_31_26_d[6'h16];
    assign inst_bne    = op_31_26_d[6'h17];
    assign inst_lu12i_w= op_31_26_d[6'h05] & ~inst_reg[25];
    assign inst_slti   = op_31_26_d[6'h00] & op_25_22_d[4'h8];
    assign inst_sltiu  = op_31_26_d[6'h00] & op_25_22_d[4'h9];
    assign inst_andi   = op_31_26_d[6'h00] & op_25_22_d[4'hd];
    assign inst_ori    = op_31_26_d[6'h00] & op_25_22_d[4'he];
    assign inst_xori   = op_31_26_d[6'h00] & op_25_22_d[4'hf];
    assign inst_sll_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'he];
    assign inst_srl_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'hf];
    assign inst_sra_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h10];
    assign inst_pcaddu12i = op_31_26_d[6'h7] &~inst_reg[25];
    assign inst_mul_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h18];
    assign inst_mulh_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h19];
    assign inst_mulh_wu= op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h1a];
    assign inst_div_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h0];
    assign inst_mod_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h1];
    assign inst_div_wu = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h2];
    assign inst_mod_wu = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h3];
    wire [18:0] alu_op;
    assign alu_op_id = alu_op;
    assign alu_op[ 0] = inst_add_w | inst_addi_w | inst_ld_w | inst_st_w
                    | inst_jirl | inst_bl | inst_pcaddu12i;
    assign alu_op[ 1] = inst_sub_w;
    assign alu_op[ 2] = inst_slt|inst_slti;
    assign alu_op[ 3] = inst_sltu|inst_sltiu;
    assign alu_op[ 4] = inst_and|inst_andi;
    assign alu_op[ 5] = inst_nor;
    assign alu_op[ 6] = inst_or|inst_ori;
    assign alu_op[ 7] = inst_xor|inst_xori;
    assign alu_op[ 8] = inst_slli_w|inst_sll_w;
    assign alu_op[ 9] = inst_srli_w|inst_srl_w;
    assign alu_op[10] = inst_srai_w|inst_sra_w;
    assign alu_op[11] = inst_lu12i_w;
    assign alu_op[12] = inst_mul_w;
    assign alu_op[13] = inst_mulh_w;
    assign alu_op[14] = inst_mulh_wu;
    assign alu_op[15] = inst_div_w;
    assign alu_op[16] = inst_mod_w;
    assign alu_op[17] = inst_div_wu;
    assign alu_op[18] = inst_mod_wu;
    assign div_en = (inst_div_w | inst_mod_w | inst_div_wu | inst_mod_wu);
    //to regfile 
    wire src_reg_is_rd;
    assign src_reg_is_rd = inst_beq | inst_bne | inst_st_w;
    assign raddr1 = rj;
    assign raddr2 = src_reg_is_rd ? rd : rk;

    //src
    wire src1_is_pc;
    wire src2_is_imm;
    assign src1_is_pc    = inst_jirl | inst_bl | inst_pcaddu12i;
    assign src2_is_imm   =  inst_slli_w |
                            inst_srli_w |
                            inst_srai_w |
                            inst_addi_w |
                            inst_ld_w   |
                            inst_st_w   |
                            inst_lu12i_w|
                            inst_jirl   |
                            inst_bl     |
                            inst_slti   |
                            inst_sltiu  |
                            inst_andi   |
                            inst_ori    |
                            inst_xori   |
                            inst_pcaddu12i;

    //aboat imm
    assign need_ui5   =  inst_slli_w | inst_srli_w | inst_srai_w;
    assign need_si12  =  inst_addi_w | inst_ld_w | inst_st_w| inst_slti | inst_sltiu ;
    assign need_si12_unsigned  = inst_andi | inst_ori | inst_xori ; 
    assign need_si16  =  inst_jirl | inst_beq | inst_bne;
    assign need_si20  =  inst_lu12i_w | inst_pcaddu12i;
    assign need_si26  =  inst_b | inst_bl;
    assign src2_is_4  =  inst_jirl | inst_bl;
    wire [31:0] imm;
    assign imm = src2_is_4 ? 32'h4                      :
                need_si20 ? {i20[19:0], 12'b0}         :
                need_si12 ? {{20{i12[11]}}, i12[11:0]}  :
                {{20{1'b0}}, i12[11:0]}  ;

//about flush
    wire [31:0]     rj_value;
    wire [31:0]     rd_value;
    wire [31:0]     rk_value;
    wire [31:0]     rkd_value;
    wire            rj_eq_rd;
    assign rj_value  =  (compare_rj_exe & forward_en_from_exe) ? forward_data_from_exe :
                        (compare_rj_mem) ? forward_data_from_mem :
                        (compare_rj_wb)  ? forward_data_from_wb  :
                        rdata1;;
    assign rd_value =   (compare_rd_exe) ? forward_data_from_exe :
                        (compare_rd_mem) ? forward_data_from_mem :
                        (compare_rd_wb)  ? forward_data_from_wb  :
                                        rdata2;
    assign rk_value =   (compare_rk_exe & forward_en_from_exe  ) ? forward_data_from_exe :
                        (compare_rk_mem) ? forward_data_from_mem :
                        (compare_rk_wb)  ? forward_data_from_wb  :
                                            rdata2;
    assign rkd_value = src_reg_is_rd ? rd_value : rk_value; 
    assign rj_eq_rd = (rj_value == rd_value);
    assign br_taken = (   inst_beq  &&  rj_eq_rd
                    || inst_bne  && !rj_eq_rd
                    || inst_jirl
                    || inst_bl
                    || inst_b
                    );
    //flush newpc                
    wire [31:0] br_offs;
    wire [31:0] jirl_offs;
    assign flush = valid&~conflict&br_taken;
    assign newpc = (inst_beq || inst_bne || inst_bl || inst_b) ? (pc_reg + br_offs) :
                                                   /*inst_jirl*/ (rj_value + jirl_offs);
    assign br_offs = need_si26 ? {{ 4{i26[25]}}, i26[25:0], 2'b0} :
                             {{14{i16[15]}}, i16[15:0], 2'b0} ;
    assign jirl_offs = {{14{i16[15]}}, i16[15:0], 2'b0};
    //Interactive signal
    assign pc_id = pc_reg;
    reg [31:0] pc_reg;
    always @(posedge clk) begin
        if (reset) begin
            pc_reg <= 32'b0;
        end
        else if (allow_in && if_ready) begin
            pc_reg <= pc_from_if;
        end
    end
    assign ready_go = valid & ~conflict;
    assign allow_in = ~valid | (ready_go & EX_allow_in);
    wire dst_is_r1;
    assign dst_is_r1  = inst_bl;
    assign dest = dst_is_r1 ? 5'd1 : rd;
    assign reg_en = ~inst_st_w & ~inst_beq & ~inst_bne & ~inst_b;
    assign mem_en = inst_st_w ? 2'b11 :
                    inst_ld_w ? 2'b01 : 2'b00;
    assign src1 = src1_is_pc ? pc_reg : rj_value;
    assign src2 = src2_is_imm ? imm : rkd_value;
    assign rdata1_to_exe = rj_value;
    assign rdata2_to_exe = rkd_value;
    
    //about conflict
    wire   no_rj;
    assign no_rj = inst_b | inst_lu12i_w | inst_bl | inst_pcaddu12i;
    wire   have_rk;
    assign have_rk = inst_add_w | inst_sub_w | inst_slt | inst_sltu | inst_nor | inst_and | inst_or | inst_xor | inst_mul_w | inst_mulh_w | inst_mulh_wu
                  | inst_div_w | inst_mod_w | inst_div_wu | inst_mod_wu | inst_sll_w | inst_srl_w | inst_sra_w;
    wire   have_rd;
    assign have_rd = inst_st_w | inst_beq | inst_bne;
    wire   conflict_rj;
    wire   conflict_rk;
    wire   conflict_rd;
    wire   compare_rj_exe;
    wire   compare_rj_mem;
    wire   compare_rj_wb;
    assign compare_rj_exe = (rj == ex_dest) & ex_reg_en_valid & (rj != 5'b0);
    assign compare_rj_mem = (rj == mem_dest) & mem_reg_en_valid & (rj != 5'b0);
    assign compare_rj_wb  = (rj == wb_dest)  & wb_reg_en_valid & (rj != 5'b0);
    wire   compare_rk_exe;
    wire   compare_rk_mem;
    wire   compare_rk_wb;
    assign compare_rk_exe = (rk == ex_dest) & ex_reg_en_valid & (rk != 5'b0);
    assign compare_rk_mem = (rk == mem_dest) & mem_reg_en_valid & (rk != 5'b0);
    assign compare_rk_wb  = (rk == wb_dest)  & wb_reg_en_valid & (rk != 5'b0);
    wire   compare_rd_exe;
    wire   compare_rd_mem;
    wire   compare_rd_wb;
    assign compare_rd_exe = (rd == ex_dest) & ex_reg_en_valid & (rd != 5'b0);
    assign compare_rd_mem = (rd == mem_dest) & mem_reg_en_valid & (rd != 5'b0);
    assign compare_rd_wb  = (rd == wb_dest)  & wb_reg_en_valid & (rd != 5'b0);
    assign conflict_rj = compare_rj_exe &~forward_en_from_exe & ~no_rj;
    assign conflict_rk = compare_rk_exe &~forward_en_from_exe & have_rk;
    assign conflict_rd = compare_rd_exe &~forward_en_from_exe & have_rd;
    wire   conflict;
    assign conflict = conflict_rj | conflict_rk | conflict_rd;

 

endmodule
