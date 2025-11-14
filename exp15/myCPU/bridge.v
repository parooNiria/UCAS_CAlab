module bridge(
    input               clk,
    input               aresetn,

    // AXI Read Address Channel
    output reg  [ 3:0]  arid,
    output reg  [31:0]  araddr,
    output      [ 7:0]  arlen,
    output reg  [ 2:0]  arsize,
    output      [ 1:0]  arburst,
    output      [ 1:0]  arlock,
    output      [ 3:0]  arcache,
    output      [ 2:0]  arprot,
    output              arvalid,
    input               arready,

    // AXI Read Data Channel
    input       [ 3:0]  rid,
    input       [31:0]  rdata,
    input       [ 1:0]  rresp,
    input               rlast,
    input               rvalid,
    output      wire    rready,

    // AXI Write Address Channel
    output      [ 3:0]  awid,
    output reg  [31:0]  awaddr,
    output      [ 7:0]  awlen,
    output reg  [ 2:0]  awsize,
    output      [ 1:0]  awburst,
    output      [ 1:0]  awlock,
    output      [ 3:0]  awcache,
    output      [ 2:0]  awprot,
    output      wire    awvalid,
    input               awready,

    // AXI Write Data Channel
    output    wire      wid,
    output reg  [31:0]  wdata,
    output reg  [ 3:0]  wstrb,
    output              wlast,
    output      wire    wvalid,
    input               wready,

    // AXI Write Response Channel
    input       [ 3:0]  bid,
    input       [ 1:0]  bresp,
    input               bvalid,
    output      wire    bready,

    // Inst SRAM Interface (Read-Only)
    input               inst_sram_req,
    input       [31:0]  inst_sram_addr,
    // Unused write ports for instruction SRAM, included for interface completeness
    input               inst_sram_wr,
    input       [ 1:0]  inst_sram_size,
    input       [ 3:0]  inst_sram_wstrb,
    input       [31:0]  inst_sram_wdata,
    output      wire    inst_sram_addr_ok,
    output      wire    inst_sram_data_ok,
    output      wire [31:0] inst_sram_rdata,

    // Data SRAM Interface (Read/Write)
    input               data_sram_req,
    input               data_sram_wr,
    input       [ 1:0]  data_sram_size,
    input       [31:0]  data_sram_addr,
    input       [31:0]  data_sram_wdata,
    input       [ 3:0]  data_sram_wstrb,
    output      wire    data_sram_addr_ok,
    output      wire    data_sram_data_ok,
    output      wire [31:0] data_sram_rdata
);
    //对于固定的值，申明为wire信号，直接输出就可以了
    assign      arlen  = 8'b0;  //单次传输
    assign      arburst= 2'b01; //INCR模式
    assign      arlock = 2'b00; //正常访问
    assign      arcache= 4'b0000;
    assign      arprot = 3'b000; //正常访问

    assign      awid   = 4'b0;  //单一ID
    assign      awlen  = 8'b0;  //单次传输
    assign      awburst= 2'b01; //INCR模式
    assign      awlock = 2'b00; //正常访问
    assign      awcache= 4'b0000;
    assign      awprot = 3'b000; //正常访问

    assign      wid    = 4'b0;  //单一ID
    assign      wlast  = 1'b1;  //单次传输

    //想实现的效果是：cpu上一拍说要读/写，然后桥接模块下一拍发出请求
    //这个不用同一拍组合逻辑的原因是1.时序2.axi不可改变的问题3.简化设计

//Read Address Channel
    //首先给出三个请求信号
    wire read_req_from_inst = inst_sram_req & ~inst_sram_wr;
    wire read_req_from_data = data_sram_req & ~data_sram_wr;
    wire write_req_from_data= data_sram_req & data_sram_wr;
    //握手信号
    wire ar_handshake_done = arvalid & arready;
    wire aw_handshake_done = awvalid & awready;
    wire w_handshake_done  = wvalid & wready;
    wire r_handshake_done  = rvalid & rready;
    wire b_handshake_done  = bvalid & bready;
    
    //对于三个信号，采取如下的响应策略：
    //数据请求高于指令请求

    //由于请求分为待发送和待响应两个阶段和读和写两种请求
    //因此存在请求的并行问题
    //采取方式如下：
    //指令读和数据写可并行
    //数据读和数据写不可并行，必须等到数据写完成后才能进行数据读，反之亦然
    //读指令之间可并行，即一个读还未返回时发出下一个读信号

    //在本转接桥的实现中，没有采取先将请求发给转接桥，再由转接桥发给axi的方式（为了性能）
    //将上述的响应策略翻译一下就是：
    //每个时刻转接桥最多有两个请求，一个来自指令，一个来自数据
    //在转接桥内部我设置一个状态机，说明当前是否有数据写和数据读请求进行中
    //当数据写进行中，屏蔽来自数据读的请求
    //当数据读进行中，屏蔽来自数据写的请求
    //对于两个请求的选择，数据请求优先级更高，优先响应
    localparam  DP_IDLE       = 3'b001,
                DP_BUSY_READ  = 3'b010,
                DP_BUSY_WRITE = 3'b100;
    wire [2:0] data_path_state;

    reg [1:0] outstanding_data_reads; //待完成的读请求数
    reg [1:0] outstanding_data_writes; //待完成的写请求数

    wire data_read_starts     = ar_handshake_done && (arid[0] == 1'b1);
    wire data_read_completed  = r_handshake_done && rlast && (rid[0] == 1'b1) && exit_read_req;

    wire data_write_starts    = write_req_from_data_valid && data_sram_addr_ok;
    wire data_write_completed = b_handshake_done;
    //非常简单的实现逻辑，此处不多说
    wire [1:0] outstanding_data_reads_next;
    wire [1:0] outstanding_data_writes_next;
    assign outstanding_data_reads_next = (data_read_starts && ~data_read_completed) ? (outstanding_data_reads + 1) :
                                      (~data_read_starts && data_read_completed) ? (outstanding_data_reads - 1) :
                                      outstanding_data_reads;

    // 计算写操作的未完成数量
    assign outstanding_data_writes_next = (data_write_starts && ~data_write_completed) ? (outstanding_data_writes + 1) :
                                       (~data_write_starts && data_write_completed) ? (outstanding_data_writes - 1) :
                                       outstanding_data_writes;
    // reg [31:0] inst_addr_commit_cnt;
    // always @(posedge clk) begin
    //     if (~aresetn) begin
    //         inst_addr_commit_cnt <= 32'b0;
    //     end else if (ar_handshake_done & (arid[0] == 1'b0)) begin
    //         inst_addr_commit_cnt <= inst_addr_commit_cnt + 1;
    //     end
    // end

    // reg [31:0] inst_back_cnt;
    // always @(posedge clk) begin
    //     if (~aresetn) begin
    //         inst_back_cnt <= 32'b0;
    //     end else if (r_handshake_done & (rid[0] == 1'b0)) begin
    //         inst_back_cnt <= inst_back_cnt + 1;
    //     end
    // end

    // wire more = inst_back_cnt > inst_addr_commit_cnt;
        
    wire any_read_starts = ar_handshake_done;
    wire any_read_completed = r_handshake_done&exit_read_req;
    reg [3:0] outstanding_all_reads;
    always @(posedge clk or negedge aresetn) begin
        if (~aresetn) begin
            outstanding_all_reads <= 4'b0;
        end else if (any_read_starts && !any_read_completed) begin
            outstanding_all_reads <= outstanding_all_reads + 1;
        end else if (!any_read_starts && any_read_completed) begin
            outstanding_all_reads <= outstanding_all_reads - 1;
        end
    end
    wire exit_read_req = outstanding_all_reads != 4'b0; 

    always @(posedge clk) begin
        if (~aresetn) begin
            outstanding_data_reads <= 2'b00;
        end else begin
            outstanding_data_reads <= outstanding_data_reads_next;
        end
    end

    always @(posedge clk) begin
        if (~aresetn) begin
            outstanding_data_writes <= 2'b00;
        end else begin
            outstanding_data_writes <= outstanding_data_writes_next;
        end
    end

//转换桥一个时间点只能处理数据读，或者数据写，或者空闲
    assign data_path_state = (outstanding_data_writes > 0) ? DP_BUSY_WRITE :
                             (outstanding_data_reads > 0)  ? DP_BUSY_READ  :
                                                                DP_IDLE;

//当有读，屏蔽写，当有写，屏蔽读
//现在当看到请求，能处理就处理就行了
    wire read_req_from_data_valid = read_req_from_data & (data_path_state[0]| data_path_state[1]); 
    wire write_req_from_data_valid= write_req_from_data & (data_path_state[0]| data_path_state[2]);
    wire read_req_from_inst_valid = read_req_from_inst;

//读请求处理
    //当请求来临时，给与一个仲裁器
    //在仲裁器转换时，看valid,在仲裁保持，看keep
    //只有对于指令的读请求存在中途取消的可能性
    //值得在报告中指出的是，我通过修改cpu的取指逻辑，使得cpu如果要半途更换地址，一定会先取消再重新发出请求
    reg [2:0] arbiter_for_read;
    reg [2:0] arbiter_for_read_next;
    localparam ARB_IDLE = 3'b001,
               ARB_FROM_INST = 3'b010,
               ARB_FROM_DATA = 3'b100;
    always @(posedge clk) begin
        if (~aresetn) begin
            arbiter_for_read <= ARB_IDLE;
        end
        else begin
            arbiter_for_read <= arbiter_for_read_next;
        end
    end
    
    wire keep_read_from_inst = arbiter_for_read[1]&(inst_sram_req & ~inst_sram_wr);

    always @(*) begin
        case (arbiter_for_read)
            ARB_IDLE: begin                    //写报告时注意此处优先级的问题
                if (read_req_from_data_valid)
                    arbiter_for_read_next = ARB_FROM_DATA;
                else if (read_req_from_inst_valid)
                    arbiter_for_read_next = ARB_FROM_INST;
                else
                    arbiter_for_read_next = ARB_IDLE;
            end
            ARB_FROM_DATA: begin             //当data握手后，可以直接转入IDLE状态，或则继续下一次请求
                if (ar_handshake_done & arid[0])
                    if (read_req_from_inst_valid)
                        arbiter_for_read_next = ARB_FROM_INST;
                    else
                        arbiter_for_read_next = ARB_IDLE;
                else
                    arbiter_for_read_next = ARB_FROM_DATA;
            end
            ARB_FROM_INST: begin             //对于处理指令读请求，可能变更的条件有此时中途取消了，及握手完成
                if (~keep_read_from_inst) begin        //中途取消，可转入IDLE，或则转入DATA读
                    if(read_req_from_data_valid)
                        arbiter_for_read_next = ARB_FROM_DATA;
                    else
                        arbiter_for_read_next = ARB_IDLE;
                end
                else if(ar_handshake_done & ~arid[0]) begin//握手完成，可转入IDLE，或则转入下一次读 
                    if(read_req_from_data_valid)
                        arbiter_for_read_next = ARB_FROM_DATA;
                    else
                        arbiter_for_read_next = ARB_IDLE;
                end
                else
                    arbiter_for_read_next = ARB_FROM_INST;
            end
            default: arbiter_for_read_next = ARB_IDLE;
        endcase
    end

    //arvalid信号
    assign arvalid = (arbiter_for_read[1] & read_req_from_inst_valid) |
                     (arbiter_for_read[2] & read_req_from_data_valid);
    //arid信号
    reg  [3:0] arid_next;
    always @(posedge clk) begin
        if (~aresetn) begin
            arid <= 4'b0;
        end else begin
            arid <= arid_next;
        end
    end

    always @(*) begin
        if (arbiter_for_read_next[1]) begin
            arid_next = 4'b0; //inst read id = 0
        end else if (arbiter_for_read_next[2]) begin
            arid_next = 4'b1; //data read id = 1
        end else begin
            arid_next = 4'b0;
        end
    end

    //araddr信号
    reg [31:0] araddr_next;
    always @(posedge clk) begin
        if (~aresetn) begin
            araddr <= 32'b0;
        end else begin
            araddr <= araddr_next;
        end
    end

    always @(*) begin
        if (arbiter_for_read_next[1]) begin
            araddr_next = inst_sram_addr;
        end else if (arbiter_for_read_next[2]) begin
            araddr_next = data_sram_addr;
        end else begin
            araddr_next = 32'b0;
        end
    end

    //arsize信号
    reg [2:0] arsize_next;
    always @(posedge clk) begin
        if (~aresetn) begin
            arsize <= 3'b0;
        end else begin
            arsize <= arsize_next;
        end
    end

    always @(*) begin
        if (arbiter_for_read_next[1]) begin
            case (inst_sram_size)
                2'b00: arsize_next = 3'b000;
                2'b01: arsize_next = 3'b001;
                2'b10: arsize_next = 3'b010;
                default: arsize_next = 3'b000;
            endcase
        end else if (arbiter_for_read_next[2]) begin
            case (data_sram_size)
                2'b00: arsize_next = 3'b000;
                2'b01: arsize_next = 3'b001;
                2'b10: arsize_next = 3'b010;
                default: arsize_next = 3'b000;
            endcase
        end else begin
            arsize_next = 3'b000;
        end
    end

    //arready信号
    assign inst_sram_addr_ok = arbiter_for_read[1] & arready;


//读响应通道
    assign rready = 1'b1;
    assign inst_sram_data_ok = (rid[0] == 1'b0) & r_handshake_done & exit_read_req;
    assign data_sram_data_ok = ((rid[0] == 1'b1) & r_handshake_done & exit_read_req) | ((bid[0] == 1'b0) & b_handshake_done);
    assign inst_sram_rdata = (rid[0] == 1'b0) ? rdata : 32'b0;
    assign data_sram_rdata = (rid[0] == 1'b1) ? rdata : 32'b0;

//写请求通道
    //对于写请求，要同时握两次手，因此可以采取对于写请求，在可接受的情况下，先拿过来存在转换桥，此时让直接给cpu地址接受信号就行了
    assign data_sram_addr_ok = (arbiter_for_read[2] & arready) | (data_sram_wr & ~data_path_state[1] & write_req_state_machine[0]);

    reg [3:0] write_req_state_machine;
    reg [3:0] write_req_state_machine_next;
    localparam WS_IDLE = 4'b0001,
               WS_RECV = 4'b0010,
               WS_SEND_ADDR =4'b0100,
               WS_SEND_DATA =4'b1000;

    always @(posedge clk) begin
        if (~aresetn) begin
            write_req_state_machine <= WS_IDLE;
        end else begin
            write_req_state_machine <= write_req_state_machine_next;
        end
    end
    wire write_addr_handshake_done = (awvalid & awready);
    wire write_data_handshake_done = (wvalid & wready);
    always @(*) begin
        case (write_req_state_machine)
            WS_IDLE: begin
                if (write_req_from_data_valid)
                    write_req_state_machine_next = WS_RECV;
                else
                    write_req_state_machine_next = WS_IDLE;
            end
            WS_RECV: begin
                if (write_data_handshake_done&&write_addr_handshake_done)
                    write_req_state_machine_next = WS_IDLE;
                else if (write_addr_handshake_done)
                    write_req_state_machine_next = WS_SEND_ADDR;
                else if (write_data_handshake_done)
                    write_req_state_machine_next = WS_SEND_DATA;
                else
                    write_req_state_machine_next = WS_RECV;
            end
            WS_SEND_ADDR: begin
                if (write_data_handshake_done)
                    write_req_state_machine_next = WS_IDLE;
                else
                    write_req_state_machine_next = WS_SEND_ADDR;
            end
            WS_SEND_DATA: begin
                if (write_addr_handshake_done)
                    write_req_state_machine_next = WS_IDLE;
                else
                    write_req_state_machine_next = WS_SEND_DATA;
            end
            default: write_req_state_machine_next = WS_IDLE;
        endcase
    end

    assign bready = 1'b1;
    assign awvalid = write_req_state_machine[1] | write_req_state_machine[3];
    assign wvalid  = write_req_state_machine[1] | write_req_state_machine[2];
    reg [31:0] awaddr_next;
    always @(posedge clk) begin
        if (~aresetn) begin
            awaddr <= 32'b0;
        end else if (write_req_state_machine[0]&write_req_state_machine_next[1]) begin
            awaddr <= awaddr_next;
        end
    end

    always @(*) begin
        if (write_req_state_machine_next[1]&write_req_state_machine[0]) begin
            awaddr_next = data_sram_addr;
        end else begin
            awaddr_next = 32'b0;
        end
    end

    reg [2:0] awsize_next;
    always @(posedge clk) begin
        if (~aresetn) begin
            awsize <= 3'b0;
        end else if (write_req_state_machine[0]&write_req_state_machine_next[1]) begin
            awsize <= awsize_next;
        end
    end


    wire [31:0] wdata_next;
    always @(posedge clk) begin
        if (~aresetn) begin
            wdata <= 32'b0;
        end else if (write_req_state_machine[0]&write_req_state_machine_next[1]) begin
            wdata <= wdata_next;
        end
    end

    assign wdata_next = data_sram_wdata;

    wire [3:0] wstrb_next;
    always @(posedge clk) begin
        if (~aresetn) begin
            wstrb <= 4'b0;
        end else if (write_req_state_machine[0]&write_req_state_machine_next[1]) begin
            wstrb <= wstrb_next;
        end
    end

    assign wstrb_next = data_sram_wstrb;

    always @(*) begin
        if (write_req_state_machine_next[1]&write_req_state_machine[0]) begin
            case (data_sram_size)
                2'b00: awsize_next = 3'b000;
                2'b01: awsize_next = 3'b001;
                2'b10: awsize_next = 3'b010;
                default: awsize_next = 3'b000;
            endcase
        end else begin
            awsize_next = 3'b000;
        end
    end

endmodule