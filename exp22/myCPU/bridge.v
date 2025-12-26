module bridge(
    input               clk,
    input               aresetn,

    // AXI Read Address Channel
    output   [ 3:0]  arid,
    output   [31:0]  araddr,
    output   [ 7:0]  arlen,
    output   [ 2:0]  arsize,
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
    output   [31:0]  awaddr,
    output      [ 7:0]  awlen,
    output   [ 2:0]  awsize,
    output      [ 1:0]  awburst,
    output      [ 1:0]  awlock,
    output      [ 3:0]  awcache,
    output      [ 2:0]  awprot,
    output      wire    awvalid,
    input               awready,

    // AXI Write Data Channel
    output    wire [3:0]     wid,
    output   [31:0]  wdata,
    output   [ 3:0]  wstrb,
    output              wlast,
    output      wire    wvalid,
    input               wready,

    // AXI Write Response Channel
    input       [ 3:0]  bid,
    input       [ 1:0]  bresp,
    input               bvalid,
    output      wire    bready,
    // icache rd interface
    input               	icache_rd_req,
    input   	[ 2:0]      icache_rd_type,
    input   	[31:0]      icache_rd_addr,
    output              	icache_rd_rdy,		// icache_addr_ok
    output              	icache_ret_valid,	// icache_data_ok
	output					icache_ret_last,
    output  	[31:0]      icache_ret_data,
    // dcache rd interface
	input               	dcache_rd_req,
    input   	[ 2:0]      dcache_rd_type,
    input   	[31:0]      dcache_rd_addr,
    output              	dcache_rd_rdy,
    output              	dcache_ret_valid,
	output					dcache_ret_last,
    output  	[31:0]      dcache_ret_data,
	// dcache wr interface
	input               	dcache_wr_req,
    input   	[ 2:0]      dcache_wr_type,
    input   	[31:0]      dcache_wr_addr,
    input   	[ 3:0]      dcache_wr_wstrb,
	input	   [127:0]		dcache_wr_data,
	output					dcache_wr_rdy
); 

    //现在这个bridge模块实现cache和axi的桥接功能

    
    //性能损失也不大，但是对于地址冲突，uncache访问的顺序都能得到很好的保证，这个做法是可行的
    //并且对于读指令，此时也不存在取消请求的可能
    //为了使得对于uncache的读写能够完成有序，此处对于写操作，只有当写操作完成才能发出下一个写请求
    //梳理一下这个axi接口的应答方式
    //对于读请求，先通过ar通道发出读请求，由于此时采用是burst方式，需要设置对应的arlen
    //然后等待r通道返回数据
    //这个地方现在的实现可以简单的以id号区分icache和dcache的请求
    //对于写请求，先通过aw通道发出写地址请求，然后通过w通道发出写数据(此时是burst传输）
    //最后当b通道返回写响应时，说明写操作完成

    //对于现在axi接口，之前我一直想每次req拉高直接发出请求
    //由于现在在这个地方，这样一拍的延迟其实无关紧要
    //现在决定采用书上推荐的axi接口实现方式
    //即存在四个状态机，读请求状态机，读响应状态机，写请求和写数据状态机，写响应状态机
    //对于读请求
    //现在对于指令和数据之间的读请求仲裁不需要一个单独的状态机
    //对于读，开始IDLE，然后进入请求状态，两个状态即可
    //在请求状态中，发出请求时指令还是数据，根据对应的寄存器发出即可
    //只要控制对应寄存器更新的优先级，就可完成
    //还有一个问题是，之前我是没有设置一个读响应状态机的,这个地方也是不用的，这个状态机在我看来就是一个组合逻辑的实现，没有明确的状态流转
    //因为同时可能有多个请求在等待响应，其实也可以设置为两个读响应状态机，只是我觉得没有必要
    //并且还有一个问题是，addr_ok和data_ok的产生不能直接利用valid和ready信号
    //这是书上的要求，之前的实现其实是不规范的，现在改正为用一个寄存器存下当前的数据，下一拍再返回
    //虽然会增加一个周期的延迟，但是只有cache不命中才会有这个延迟，因此影响也不是很大
    assign      arburst= 2'b01; //INCR模式
    assign      arlock = 2'b00; //正常访问
    assign      arcache= 4'b0000;
    assign      arprot = 3'b000; //正常访问

    assign      awid   = 4'b0;  //单一ID
    assign      awburst= 2'b01; //INCR模式
    assign      awlock = 2'b00; //正常访问
    assign      awcache= 4'b0000;
    assign      awprot = 3'b000; //正常访问

    assign      wid    = 4'b0;  //单一ID
    wire        reset;
    assign     reset = ~aresetn;

//读处理
//状态机
reg [1:0] ar_cstate;
reg [1:0] ar_nstate;
//用于存储发送请求信息的寄存器
reg [3:0] arid_reg;
reg [31:0] araddr_reg;
reg [7:0] arlen_reg;
reg [2:0] arsize_reg;
wire      read_block;
wire      dcache_rd_req_valid;
assign    dcache_rd_req_valid = dcache_rd_req & (~read_block);
//阻塞相同地址在完成写之前不能读
//完成读之前不能写由cache的请求机制保证
//状态转移
parameter AR_IDLE = 2'b01,
        AR_REQ  = 2'b10;

always @(posedge clk) begin
    if (reset) begin
        ar_cstate <= AR_IDLE;
    end else begin
        ar_cstate <= ar_nstate;
    end
end

//对于这个状态转移，有一个问题是，我返回给icache和dcache的rd_rdy信号
//是不能够直接由两个cache的rd_req信号决定的
//这样做极其不规范，在两个模块之间产生一个连接
//一不注意就会导致组合逻辑环
//此前我采取的做法是，先上一拍cpu请求发出,下一拍根据上一拍的请求情况，设置对应的仲裁，让给出应答信号
//这地方采取一样的实现方式，IDLE时，根据这一拍的请求情况，设置对应的寄存器
//然后下一拍转入REQ发出应答信号，并同时发出axi请求
//由于此时一个请求是不会被取消，因此下一拍转入REQ时，一拍绝对可以与请求握手
//这个地方的道理与cache中MISS转入REPLACE是一样的
always @(*) begin
    case (ar_cstate)
        AR_IDLE: begin
            if (icache_rd_req || dcache_rd_req_valid) begin
                ar_nstate = AR_REQ;
            end else begin
                ar_nstate = AR_IDLE;
            end
        end
        AR_REQ: begin
            if (arvalid && arready) begin   //这个地方如果优化的话，可以直接转入下一次REQ，但是会带来一个问题，就先这样了
                ar_nstate = AR_IDLE;
            end else begin
                ar_nstate = AR_REQ;
            end
        end
        default: begin
            ar_nstate = AR_IDLE;
        end
    endcase
end

localparam WRITE_WORD     = 3'b010;
localparam WRITE_BLOCK    = 3'b100;
//相关寄存器的更新
always @(posedge clk) begin
    if (reset) begin
        arid_reg   <= 4'b0;
        araddr_reg <= 32'b0;
        arlen_reg  <= 8'b0;
        arsize_reg <= 3'b010; //默认字传输
    end else if(ar_cstate[0] && (icache_rd_req || dcache_rd_req_valid)) begin
        arid_reg <= {3'b0, dcache_rd_req_valid}; //dcache_rd_req优先级高
        araddr_reg <= dcache_rd_req_valid ? {dcache_rd_addr[31:2], 2'b00} : {icache_rd_addr[31:2], 2'b00};
        arsize_reg <= 3'b010;  //固定32位传输
        arlen_reg  <= dcache_rd_req_valid ? (dcache_rd_type == WRITE_BLOCK ? 8'b11 : 8'b0) : (icache_rd_type == WRITE_BLOCK ? 8'b11 : 8'b0);
    end
end
//axi端应答
assign arvalid = ar_cstate[1];
assign arid = arid_reg;
assign araddr = araddr_reg;
assign arlen = arlen_reg;
assign arsize = arsize_reg;

//两个cache端应答,只能在刚刚进入时拉高
reg first_into_req;
always @(posedge clk) begin
    if (reset) begin
        first_into_req <= 1'b0;
    end else if (ar_cstate[0] && (icache_rd_req || dcache_rd_req_valid)) begin
        first_into_req <= 1'b1;
    end else begin
        first_into_req <= 1'b0;
    end
end
assign icache_rd_rdy = ar_cstate[1] && (arid_reg == 4'b0) && first_into_req;
assign dcache_rd_rdy = ar_cstate[1] && (arid_reg == 4'b1) && first_into_req;
//读响应通道
//此时对于数据返回，会先存储到本地的寄存器，下一拍再返回给cache
reg [31:0] rdata_temp_icache;
reg [31:0] rdata_temp_dcache;
reg        icache_data_valid;
reg        dcache_data_valid;
reg        icache_ret_last_r;
reg        dcache_ret_last_r;
always @(posedge clk) begin
    if (reset) begin
        rdata_temp_icache <= 32'b0;
        rdata_temp_dcache <= 32'b0;
    end else if (rvalid && rready) begin
        if (rid == 4'b0) begin
            rdata_temp_icache <= rdata;
        end else if (rid == 4'b1) begin
            rdata_temp_dcache <= rdata;
        end
    end
end
always @(posedge clk) begin
    if (reset) begin
        icache_data_valid <= 1'b0;
    end else if (rvalid && rready && (rid == 4'b0)) begin
        icache_data_valid <= 1'b1;
    end else begin
        icache_data_valid <= 1'b0;
    end
end
always @(posedge clk) begin
    if (reset) begin
        dcache_data_valid <= 1'b0;
    end else if (rvalid && rready && (rid == 4'b1)) begin
        dcache_data_valid <= 1'b1;
    end else begin
        dcache_data_valid <= 1'b0;
    end
end
always @(posedge clk) begin
    if (reset) begin
        icache_ret_last_r <= 1'b0;
    end else if (rvalid && rready && (rid == 4'b0)) begin
        icache_ret_last_r <= rlast;
    end else begin
        icache_ret_last_r <= 1'b0;
    end
end
always @(posedge clk) begin
    if (reset) begin
        dcache_ret_last_r <= 1'b0;
    end else if (rvalid && rready && (rid == 4'b1)) begin
        dcache_ret_last_r <= rlast;
    end else begin
        dcache_ret_last_r <= 1'b0;
    end
end
assign rready = 1'b1; //始终准备好接收数据
assign icache_ret_data = rdata_temp_icache;
assign dcache_ret_data = rdata_temp_dcache;
assign icache_ret_valid = icache_data_valid;
assign dcache_ret_valid = dcache_data_valid;
assign icache_ret_last = icache_ret_last_r;
assign dcache_ret_last = dcache_ret_last_r;

//写处理
//写请求&写数据通道，状态机一共5个状态
//由于现在要保证一个时间只处理一个写请求
//加入一个请求已发出，未完成响应状态
localparam WRITE_IDLE    = 5'b00001,
           WRITE_REQ     = 5'b00010,
           WRITE_SEND_DATA    = 5'b00100,
           WRITE_SEND_ADDR    = 5'b01000,
           WRITE_WAIT    = 5'b10000; //等待写响应完成，才能发出下一个写请求
//状态机
reg [4:0] w_cstate;
reg [4:0] w_nstate;
//保存写请求信息的寄存器
reg [31:0] awaddr_reg;
reg [2:0]  awsize_reg;
reg [7:0]  awlen_reg;
reg [3:0]  wstrb_reg;
reg [127:0] wdata_reg;
reg [4:0] write_data_cnt; //用于burst传输数据计数

assign read_block = (dcache_rd_addr == awaddr_reg) & (~w_cstate[0]); // 读写地址相同且有写操作且数据未写入
//状态转移
always @(posedge clk) begin
    if (reset) begin
        w_cstate <= WRITE_IDLE;
    end else begin
        w_cstate <= w_nstate;
    end
end

always @(*) begin
    case (w_cstate)
        WRITE_IDLE: begin
            if (dcache_wr_req) begin
                w_nstate = WRITE_REQ;
            end else begin
                w_nstate = WRITE_IDLE;
            end
        end
        WRITE_REQ: begin
            if (awvalid & awready & wvalid & wready & wlast) begin
                w_nstate = WRITE_WAIT;
            end else if(awvalid & awready) begin
                w_nstate = WRITE_SEND_DATA;
            end else if(wvalid & wready & wlast) begin
                w_nstate = WRITE_SEND_ADDR;
            end else begin
                w_nstate = WRITE_REQ;
            end
        end
        WRITE_SEND_DATA: begin
            if (wvalid && wready & wlast) begin
                w_nstate = WRITE_WAIT;
            end else begin
                w_nstate = WRITE_SEND_DATA;
            end
        end
        WRITE_SEND_ADDR: begin
            if (awvalid & awready) begin
                w_nstate = WRITE_WAIT;
            end else begin
                w_nstate = WRITE_SEND_ADDR;
            end
        end
        WRITE_WAIT: begin
            if (bvalid & bready) begin
                w_nstate = WRITE_IDLE;
            end else begin
                w_nstate = WRITE_WAIT;
            end
        end
        default: begin
            w_nstate = WRITE_IDLE;
        end
    endcase
end

assign awvalid = w_cstate[1] | w_cstate[3]; //WRITE_REQ | WRITE_SEND_ADDR
always @(posedge clk) begin
    if (reset) begin
        awaddr_reg <= 32'b0;
        awsize_reg <= 3'b010;
        awlen_reg  <= 8'b0;    
    end else if (w_cstate[0] && dcache_wr_req) begin //写请求状态机为空闲状态，更新数据
        awaddr_reg <= dcache_wr_addr;
        awsize_reg <= 3'b010;
        awlen_reg  <= dcache_wr_type == WRITE_BLOCK ? 8'b11 : 8'b0;
    end
end
assign awaddr = awaddr_reg;
assign awsize = awsize_reg;
assign awlen  = awlen_reg;
//cache交互端寄存器更新
always @(posedge clk) begin
    if (reset) begin
        wdata_reg <= 128'b0;
        wstrb_reg <= 4'b0;
    end else if (w_cstate[0] && dcache_wr_req) begin
        wdata_reg <= dcache_wr_data;
        wstrb_reg <= dcache_wr_wstrb;
    end
end
assign wvalid = w_cstate[1] | w_cstate[2]; //WRITE_REQ | WRITE_SEND_DATA
assign dcache_wr_rdy = w_cstate[0]; //WRITE_IDLE

always @(posedge clk) begin
    if (reset) begin
        write_data_cnt <= 5'b0;
    end else if (wvalid && wready && wlast) begin
        write_data_cnt <= 5'b0;
    end else if (wvalid && wready) begin
        write_data_cnt <= write_data_cnt + 1;
    end
end

assign wdata = wdata_reg[32*write_data_cnt +: 32];
assign wlast = (write_data_cnt == awlen_reg);
assign wstrb = wstrb_reg;
assign bready = w_cstate[4]; //WRITE_WAIT
endmodule