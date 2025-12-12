module tlb
#(
    parameter TLBNUM = 16 //TLB 表项的数量
)
(
    input wire clk,

    //搜索端口 0（用于取指）
    input  wire [              18:0] s0_vppn, // 虚拟页号，访存虚地址的 31..13 位
    input  wire                      s0_va_bit12, // 第 12 位虚拟地址
    input  wire [               9:0] s0_asid,// 地址空间 ID
    output wire                      s0_found, // 判定是否产生 TLB 重填异常
    output wire [$clog2(TLBNUM)-1:0] s0_index,// 命中 TLB 的索引
    output wire [              19:0] s0_ppn,// 物理页号
    output wire [               5:0] s0_ps,// 页大小
    output wire [               1:0] s0_plv,// 特权级
    output wire [               1:0] s0_mat, // 存储类型
    output wire                      s0_d, // 可写标志
    output wire                      s0_v,// 有效标志

    //搜索端口 1（用于访存）
    input  wire [              18:0] s1_vppn,
    input  wire                      s1_va_bit12,
    input  wire [               9:0] s1_asid,
    output wire                      s1_found,
    output wire [$clog2(TLBNUM)-1:0] s1_index,
    output wire [              19:0] s1_ppn,
    output wire [               5:0] s1_ps,
    output wire [               1:0] s1_plv,
    output wire [               1:0] s1_mat,
    output wire                      s1_d,
    output wire                      s1_v,

    // TLB 失效
    input  wire                      invtlb_valid, //是否执行 TLB 失效操作
    input  wire [               4:0] invtlb_op,  //TLB 失效操作的类型

    // 写端口
    input  wire                      we, // 写使能
    input  wire [$clog2(TLBNUM)-1:0] w_index, //要写入的 TLB 表项索引
    input  wire                      w_e, // 写e位
    input  wire [              18:0] w_vppn,
    input  wire [               5:0] w_ps,
    input  wire [               9:0] w_asid,
    input  wire                      w_g,
    input  wire [              19:0] w_ppn0,
    input  wire [               1:0] w_plv0,
    input  wire [               1:0] w_mat0,
    input  wire                      w_d0,
    input  wire                      w_v0,
    input  wire [              19:0] w_ppn1,
    input  wire [               1:0] w_plv1,
    input  wire [               1:0] w_mat1,
    input  wire                      w_d1,
    input  wire                      w_v1,

    // 读端口
    input  wire [$clog2(TLBNUM)-1:0] r_index,
    output wire                      r_e,
    output wire [              18:0] r_vppn,
    output wire [               5:0] r_ps,
    output wire [               9:0] r_asid,
    output wire                      r_g,
    output wire [              19:0] r_ppn0,
    output wire [               1:0] r_plv0,
    output wire [               1:0] r_mat0,
    output wire                      r_d0,
    output wire                      r_v0,
    output wire [              19:0] r_ppn1,
    output wire [               1:0] r_plv1,
    output wire [               1:0] r_mat1,
    output wire                      r_d1,
    output wire                      r_v1
);

//本来是一个完整的表象，但是在书上的设计中，分为不同的寄存器组。
reg  [TLBNUM-1:0] tlb_e; 
reg  [TLBNUM-1:0] tlb_ps4MB; //页大小 1:4MB, 0:4KB
reg  [      18:0] tlb_vppn    [TLBNUM-1:0]; // 虚拟页号
reg  [       9:0] tlb_asid    [TLBNUM-1:0]; // 地址空间 ID
reg               tlb_g       [TLBNUM-1:0]; // 全局页标志

reg  [      19:0] tlb_ppn0    [TLBNUM-1:0];
reg  [       1:0] tlb_plv0    [TLBNUM-1:0];
reg  [       1:0] tlb_mat0    [TLBNUM-1:0];
reg               tlb_d0      [TLBNUM-1:0];
reg               tlb_v0      [TLBNUM-1:0];

reg  [      19:0] tlb_ppn1    [TLBNUM-1:0];
reg  [       1:0] tlb_plv1    [TLBNUM-1:0];
reg  [       1:0] tlb_mat1    [TLBNUM-1:0];
reg               tlb_d1      [TLBNUM-1:0];
reg               tlb_v1      [TLBNUM-1:0];

//三组match信号
wire  [TLBNUM-1:0] match0;
wire  [TLBNUM-1:0] match1;
wire inv_match [TLBNUM-1:0];

//生成对应的match信号
genvar i;
//普通访问部分
//对于命中的匹配，需要得到对应的index信号
generate
    for(i=0; i<TLBNUM; i=i+1) begin : MATCH_GEN
        assign match0[i] = (s0_vppn[18:9] == tlb_vppn[i][18:9])
                           && (tlb_ps4MB[i] || s0_vppn[8:0] == tlb_vppn[i][8:0]) //虚拟地址比较
                           && ( (tlb_g[i]) || (s0_asid == tlb_asid[i]) )//进程标号比较
                           && tlb_e[i];   //有效判断
        assign match1[i] = (s1_vppn[18:9] == tlb_vppn[i][18:9])
                           && (tlb_ps4MB[i] || s1_vppn[8:0] == tlb_vppn[i][8:0])
                           && ( (tlb_g[i]) || (s1_asid == tlb_asid[i]) )
                           && tlb_e[i];   //有效判断
    end
endgenerate

//假设是单命中的
// assign s0_index = {$clog2(TLBNUM){match0[0]}}&{0}
//                  ||{$clog2(TLBNUM){match0[1]}}&{1}
//                     ||{$clog2(TLBNUM){match0[2]}}&{2}...

//这个地方本来是应该采用上述的编写方式，但是上述一个一个打会导致当宏定义的TLBNUM变化时，代码也需要进行相应的改动，因此采用如下的编写方式：

    wire [$clog2(TLBNUM)-1:0] match0_idx_mask [TLBNUM-1:0];
    wire [$clog2(TLBNUM)-1:0] match1_idx_mask [TLBNUM-1:0];

    generate
        for (i = 0; i < TLBNUM; i = i + 1) begin : MATCH_IDX_MASK_GEN
            assign match0_idx_mask[i] = { $clog2(TLBNUM){match0[i]} } & i[$clog2(TLBNUM)-1:0];
            assign match1_idx_mask[i] = { $clog2(TLBNUM){match1[i]} } & i[$clog2(TLBNUM)-1:0];
        end
    endgenerate

generate
  begin : INDEX_OR_BLOCK
    genvar j;

    // 定义临时信号
    wire [$clog2(TLBNUM)-1:0] s0_index_tmp [0:TLBNUM];
    wire [$clog2(TLBNUM)-1:0] s1_index_tmp [0:TLBNUM];

    // 初值
    assign s0_index_tmp[0] = 0;
    assign s1_index_tmp[0] = 0;

    // 逐级 OR 累加
    for (j = 0; j < TLBNUM; j = j + 1) begin : INDEX_OR_GEN
      assign s0_index_tmp[j+1] = s0_index_tmp[j] | match0_idx_mask[j];
      assign s1_index_tmp[j+1] = s1_index_tmp[j] | match1_idx_mask[j];
    end

    // 最终结果
    assign s0_index = s0_index_tmp[TLBNUM];
    assign s1_index = s1_index_tmp[TLBNUM];
  end
endgenerate



//失效部分
//先生成对应的cond信号
wire [3:0] cond[TLBNUM-1:0];
//这个信号是看每个表项是否满足失效条件,用的是s1端口
//cond[0]: G是否为0
//cond[1]: G是否为1
//cond[2]: ASID是否匹配
//cond[3]: VPPN是否匹配
//这个由于是要使其失效，可以不用看e位
generate
    for(i=0; i<TLBNUM; i=i+1) begin : COND_GEN
        assign cond[i][0] = ~tlb_g[i];
        assign cond[i][1] = tlb_g[i];
        assign cond[i][2] = (s1_asid == tlb_asid[i]);
        assign cond[i][3] = (s1_vppn[18:9] == tlb_vppn[i][18:9])
                      && (tlb_ps4MB[i] || s1_vppn[8:0] == tlb_vppn[i][8:0]);
    end 
endgenerate
//根据cond信号和失效操作类型生成失效匹配信号
//用或逻辑要高效一些，因此此时没有优先级关系
generate
    for(i=0; i<TLBNUM; i=i+1) begin : INV_MATCH_GEN
        assign inv_match[i] = (invtlb_op == 0 || invtlb_op == 1)  //清空所有
                              ||(invtlb_op == 2) & (cond[i][1])  //清空全局页
                              || (invtlb_op == 3) & (cond[i][0]) //清空非全局页
                              || (invtlb_op == 4) & (cond[i][0] && cond[i][2]) //清空非全局和asid匹配
                              || (invtlb_op == 5) & (cond[i][0] && cond[i][2] && cond[i][3]) //清空非全局和asid匹配和vppn匹配
                              || (invtlb_op == 6) & (cond[i][1] | cond[i][2]) & cond[i][3] ; //清空全局或asid匹配和vppn匹配，这个指令集对应描述的话过于逆天
    end
endgenerate

//s0和s1端口的输出逻辑
//此时上述的match信号还不够，由于再loognarch中是双页，还需要低位的判断
//这个判断要是根据页大小来的
//这判断的产生逻辑在对应的tlb命中之后，此时已经知道是哪个表项命中，因此不需要16组


wire page_select_0 = tlb_ps4MB[s0_index] ? s0_vppn[8] : s0_va_bit12;
wire page_select_1 = tlb_ps4MB[s1_index] ? s1_vppn[8] : s1_va_bit12;
//接下来可以输出两个端口的所有信号了
assign s0_found = | match0; //只要有一个match就表示找到了,verilog中|表示或规约
assign s1_found =  | match1;
assign s0_ps = tlb_ps4MB[s0_index] ? 6'd21 : 6'd12;
assign s1_ps = tlb_ps4MB[s1_index] ? 6'd21 : 6'd12;

assign s0_ppn = page_select_0 ? tlb_ppn1[s0_index] : tlb_ppn0[s0_index];
assign s0_plv = page_select_0 ? tlb_plv1[s0_index] : tlb_plv0[s0_index];
assign s0_mat = page_select_0 ? tlb_mat1[s0_index] : tlb_mat0[s0_index];
assign s0_d   = page_select_0 ? tlb_d1[s0_index]   : tlb_d0[s0_index];
assign s0_v   = page_select_0 ? tlb_v1[s0_index]   : tlb_v0[s0_index];

assign s1_ppn = page_select_1 ? tlb_ppn1[s1_index] : tlb_ppn0[s1_index];
assign s1_plv = page_select_1 ? tlb_plv1[s1_index] : tlb_plv0[s1_index];   
assign s1_mat = page_select_1 ? tlb_mat1[s1_index] : tlb_mat0[s1_index];
assign s1_d   = page_select_1 ? tlb_d1[s1_index]   : tlb_d0[s1_index];
assign s1_v   = page_select_1 ? tlb_v1[s1_index]   : tlb_v0[s1_index];

//现在已经完成了s0和s1端口的输出，接下来是读端口的输出,即根据r_index读出对应的表项
//这一步很简单

assign r_e      = tlb_e[r_index];
assign r_vppn   = tlb_vppn[r_index];
assign r_ps     = tlb_ps4MB[r_index] ? 6'd21 : 6'd12;
assign r_asid   = tlb_asid[r_index];
assign r_g      = tlb_g[r_index];

assign r_ppn0   = tlb_ppn0[r_index];
assign r_plv0   = tlb_plv0[r_index];
assign r_mat0   = tlb_mat0[r_index];
assign r_d0     = tlb_d0[r_index];
assign r_v0     = tlb_v0[r_index];

assign r_ppn1   = tlb_ppn1[r_index];
assign r_plv1   = tlb_plv1[r_index];
assign r_mat1   = tlb_mat1[r_index];
assign r_d1     = tlb_d1[r_index];
assign r_v1     = tlb_v1[r_index];

//现在再完成写端口的逻辑，这个端口是要修改对应tlb寄存器的内容，有意思的是在tlb没有reset信号，即不在开机时做任何初始化
//因此在上电之后，tlb的内容是随机的，需要软件进行初始化

generate
    for(i=0; i<TLBNUM; i=i+1) begin : TLB_WRITE_GEN
        always @(posedge clk) begin
            if(invtlb_valid && inv_match[i]) begin //此处应该是不需要对应的优先级关系的
                tlb_e[i] <= 1'b0; 
            end
            else if(we && (w_index == i[$clog2(TLBNUM)-1:0])) begin
                tlb_e[i]      <= w_e;
                tlb_vppn[i]   <= w_vppn;
                tlb_ps4MB[i]  <= (w_ps == 6'd21) ? 1'b1 : 1'b0;
                tlb_asid[i]   <= w_asid;
                tlb_g[i]      <= w_g;

                tlb_ppn0[i]   <= w_ppn0;
                tlb_plv0[i]   <= w_plv0;
                tlb_mat0[i]   <= w_mat0;
                tlb_d0[i]     <= w_d0;
                tlb_v0[i]     <= w_v0;

                tlb_ppn1[i]   <= w_ppn1;
                tlb_plv1[i]   <= w_plv1;
                tlb_mat1[i]   <= w_mat1;
                tlb_d1[i]     <= w_d1;
                tlb_v1[i]     <= w_v1;
            end
        end
    end
endgenerate

endmodule