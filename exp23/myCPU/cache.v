module cache(
    input wire        clk,
    input wire        resetn,

    // cache与CPU的交互接口
    input wire        valid,  // CPU 访问cache 请求的有效信号
    input wire        op,     // 读或写
    input wire [ 7:0] index,  // vaddr[11:4] 索引
    input wire [19:0] tag,    // paddr[31:12] 标签
    input wire [ 3:0] offset, // vaddr[3:0] 偏移量
    input wire [ 3:0] wstrb,  // 字节写使能
    input wire [31:0] wdata,  // 写数据
    
    output wire        addr_ok, // 地址传输完成信号
    output wire        data_ok, // 数据传输完成信号
    output wire [31:0] rdata,   // cache读数据
    input  wire [1:0]  mem_type, //存储控制类型，表示：0——强序非缓存，1——一致可缓存

    // cache与总线的交互接口
    output wire        rd_req,   // 读请求有效信号
    output wire [ 2:0] rd_type,  // 读请求类型
    output wire [31:0] rd_addr,  // 读请求起始地址

    input  wire        rd_rdy,   // 读请求是否被内存接收
    input  wire        ret_valid,// 返回数据有效
    input  wire        ret_last, // 读请求的最后一个返回数据
    input  wire [31:0] ret_data, // 读返回数据

    output wire        wr_req,   // 写请求有效信号
    output wire [ 2:0] wr_type,  // 写请求类型
    output wire [31:0] wr_addr,  // 写请求起始地址
    output wire [ 3:0] wr_wstrb,  // 写操作字节掩码，仅在 WRITE_BYTE, WRITE_HALFWORD, WRITE_WORD下有意义
    output wire [127:0] wr_data, // 写数据

    input  wire        wr_rdy,    // 写请求能否被接收的握手信号

    //cacop接口
    input  wire        cacop_req,
    output wire        cacop_finish,
    input  wire [1:0]  cacop_op,
    input  wire [31:0] cacop_va
    );
//在cache中有四种信号，根据状态机理解，hitwrite是单独的操作（请务必理解这个地方的每一句话）

//1.当IDLE到LOOKUP,和LOOKUP到LOOUP时，需要根据发来的地址index进行查找，就是lookup
//2.当LOOKUP阶段发现不命中，需要转入MISS阶段等待wr_rdy拉高，当wr_rdy拉高后，进入REPLACE阶段
//在这个转变刚刚发生的那个周期，需要进行replace操作，这就是一个读操作，将需要替换的地方的数据读出
//3.当REPLACE阶段发现rd_rdy拉高，发出读请求（就是请求对应的cache miss处的数据），进入REFILL阶段，接受对应的数据
//在REFILL阶段，根据ret_valid和ret_last信号，将数据写入到之前replace读出的地方，这就是refill操作
//对于hitwrite操作，只在一个时候发生，就是cache hit的情况下进行写操作（cache miss时替换的时候refill直接综合读出数据和待写数据写入即可）
//4.这个hitwrite操作，就是LOOKUP时，执行store，发现cache hit将待写的数据存入write buffer中，等到下一个周期，发出hitwrite信号，进行写操作
//这个时候存在两个冲突问题，
//（1）就是LOOKUP时，执行store，发现cache hit，就可以转入IDLE状态，等待下一个请求
//然后下面这一拍，主状态机IDLE，发出lookup,writebuf状态机WRITE，进行hitwrite操作,这二者都是在对内部的ram操作
//但是内部ram是单端口的，不能同时进行读写操作，所以会产生冲突
//（2）LOOKUP时，执行store，发现cache hit，这个时候来了下一个load请求，这个load请求的地址和store的地址是一样的
//按照主状态机的设计，这个load请求的数据会在下一拍返回，但是下一拍writebuf状态机发出hitwrite操作，进行写操作，还没有完成
//这个时候就产生了数据相关问题
//5.为了解决这个冲突问题，需要阻塞主状态机的状态转换，采取的解决方式如下：
//（1）对于第一个冲突问题，当写状态机处于WRITE状态时，如果有一个新的读请求，主状态机不能从IDLE转入LOOKUP，保持在IDLE状态
//（2）对于第二个冲突问题，当写命中时，如果来了一个LOOKUP请求，不能直接转入下一个LOOKUP状态，需要先转入IDLE状态，再处理
//这个时候就有一个问题需要额外指出，就是对于store指令，只会阻塞相关的load指令，而对于相关的store指令，由于都是需要利用write buffer进行写操作
//因此不会发生冲突问题
    wire lookup; 
    wire hitwrite;
    wire replace;
    wire refill;
//我发现这种定义将其声明为参数可以使得代码可读性更高
//op_type
localparam READ  = 1'b0;
localparam WRITE = 1'b1;

//rd_type
localparam READ_BYTE     = 3'b000; //1字节
localparam READ_HALFWORD = 3'b001; //2字节
localparam READ_WORD     = 3'b010; //4字节
localparam READ_BLOCK    = 3'b100; 

//wr_type
localparam WRITE_BYTE     = 3'b000;
localparam WRITE_HALFWORD = 3'b001;
localparam WRITE_WORD     = 3'b010;
localparam WRITE_BLOCK    = 3'b100;

wire reset = ~resetn;

//请理解每一句话！！！
//这个地方需要定制对应的ram结构
//这个地方接受一下对应的表需要什么操作
//{tag,v}表，每一行21位，256行
//对于这个表，会在lookup操作时进行读操作，在replace时读出，refill时写入
//data表，每一行128位，256行
//对于这个表，会在lookup时读出，hitwrite时写入，replace时读出，refill时写入
//上述可能同时的操作只有lookup和hitwrite，所以有相关的冲突信号
//D表，只在replace时读出，在refill时写入和hitwrite时写入
//D表用寄存器实现
//上述还有一个问题：
//就是现在对于D表是将其分成了不同的bank进行存储，只有bank数相同的读写才会冲突
//那么对于store后面一条load的情况
//如果在IDLE是lookup和hitwrite同时进行时，只要操作的是不同的bank，就可避免冲突，这个时候的地址也应该是不同的
//在此处应该分四组信号，对应四组bank
//如果在LOOKUP时，来了load，只要地址不同就可同时操作
//对于第二种冲突，是数据的写后读冲突，可以通过前递解决，但是我觉得此处由于可能存在uncache的问题，如果采用前递策略
//可能给后续实验造成一点麻烦
//对于第一种冲突，主要是结构上的冲突，无法避免
//就两个冲突，需要比对的地址部分是不一样的

//现在加入了非缓存的部分，再来考虑一下状态机的操作,对于读操作
//IDLE获得请求，查表，此时就算是非缓存请求，查表也不会导致错误的后果
//在LOOKUP时，如果是非缓存请求，直接将cache_miss信号拉高，进入MISS状态
//进入miss，如果是非缓存，不用等到wr_rdy拉高，直接进入REPLACE状态
//此时对于replace的读取操作，也可不用进行，同时刚进入REPLACE状态时的写请求也不用发
//进入replace，需要发出axi上的读请求，这个地方需要修改对应的信号
//进入refill，等到数据返回，直接返回给cpu即可。
//对于写操作，进入miss，如果是非缓存，等到wr_rdy拉高，进入REPLACE状态
//进入REPLACE状态，发出对应的写请求，之后空转一圈即可

//这个地方看出一点不同了，wr_rdy为高时，是不能在没有接受写请求由于其他原因再拉低的
//因此转接桥需要重新设计

//对于相关交互信号的影响：
//addr_ok信号：当处于LOOKUP状态时，如果此时是非缓冲，由于直接将cache_miss拉高，因此此时不会拉高addr_ok
//data_ok信号：对于非缓存读操作，当处于REFILL状态时，等到数据返回，直接拉高data_ok信号
//对于非缓存写操作，直接再Lookup时拉高，无修改
//rd_req信号：无修改
//wr_req信号：非缓存读不用发出，非缓冲写直接在REPLACE时发出
wire [ 7:0] tagv_addr;
wire [20:0] tagv_wdata;
wire [20:0] tagv_w0_rdata, tagv_w1_rdata;
wire        tagv_w0_en, tagv_w1_en;
wire        tagv_w0_we, tagv_w1_we;

wire [ 7:0] data_addr[3:0];
wire [31:0] data_wdata;
wire [31:0] data_w0_rdata[3:0];
wire [31:0] data_w1_rdata[3:0];
wire        data_w0_en[3:0], data_w1_en[3:0];
wire [ 3:0] data_w0_we[3:0], data_w1_we[3:0];

//cacop相关信号
wire cacop_store_tag;
wire cacop_Index_Invalidate;
wire cacop_Hit_Invalidate;
assign cacop_store_tag = (cacop_op == 2'b00) && cacop_req;
assign cacop_Index_Invalidate = (cacop_op == 2'b01) && cacop_req;
assign cacop_Hit_Invalidate = (cacop_op == 2'b10) && cacop_req;

wire cacop_store_tag_valid;
wire cacop_Index_Invalidate_valid;
wire cacop_Hit_Invalidate_valid;
assign cacop_store_tag_valid = cacop_store_tag && (current_state == IDLE) && (writebuf_cstate == WRITEBUF_IDLE);
assign cacop_Index_Invalidate_valid = cacop_Index_Invalidate && (current_state == IDLE) && (writebuf_cstate == WRITEBUF_IDLE);
assign cacop_Hit_Invalidate_valid = cacop_Hit_Invalidate && (current_state == IDLE) && (writebuf_cstate == WRITEBUF_IDLE);

tagv_ram tagv_way0(
    .addra(tagv_addr),
    .clka(clk),
    .dina(tagv_wdata),
    .douta(tagv_w0_rdata),
    .ena(tagv_w0_en),
    .wea(tagv_w0_we)
);
tagv_ram tagv_way1(
    .addra(tagv_addr),
    .clka(clk),
    .dina(tagv_wdata),
    .douta(tagv_w1_rdata),
    .ena(tagv_w1_en),
    .wea(tagv_w1_we)
);


data_bank_ram data_way0_bank0(
    .addra(data_addr[0]),
    .clka(clk),
    .dina(data_wdata),
    .douta(data_w0_rdata[0]),
    .ena(data_w0_en[0]),
    .wea(data_w0_we[0])
);
data_bank_ram data_way0_bank1(
    .addra(data_addr[1]),
    .clka(clk),
    .dina(data_wdata),
    .douta(data_w0_rdata[1]),
    .ena(data_w0_en[1]),
    .wea(data_w0_we[1])
);
data_bank_ram data_way0_bank2(
    .addra(data_addr[2]),
    .clka(clk),
    .dina(data_wdata),
    .douta(data_w0_rdata[2]),
    .ena(data_w0_en[2]),
    .wea(data_w0_we[2])
);
data_bank_ram data_way0_bank3(
    .addra(data_addr[3]),
    .clka(clk),
    .dina(data_wdata),
    .douta(data_w0_rdata[3]),
    .ena(data_w0_en[3]),
    .wea(data_w0_we[3])
);
data_bank_ram data_way1_bank0(
    .addra(data_addr[0]),
    .clka(clk),
    .dina(data_wdata),
    .douta(data_w1_rdata[0]),
    .ena(data_w1_en[0]),
    .wea(data_w1_we[0])
);
data_bank_ram data_way1_bank1(
    .addra(data_addr[1]),
    .clka(clk),
    .dina(data_wdata),
    .douta(data_w1_rdata[1]),
    .ena(data_w1_en[1]),
    .wea(data_w1_we[1])
);
data_bank_ram data_way1_bank2(
    .addra(data_addr[2]),
    .clka(clk),
    .dina(data_wdata),
    .douta(data_w1_rdata[2]),
    .ena(data_w1_en[2]),
    .wea(data_w1_we[2])
);
data_bank_ram data_way1_bank3(
    .addra(data_addr[3]),
    .clka(clk),
    .dina(data_wdata),
    .douta(data_w1_rdata[3]),
    .ena(data_w1_en[3]),
    .wea(data_w1_we[3])
);

//D表
reg [255:0] dirty_way0;
reg [255:0] dirty_way1;

//state machine parameters
localparam IDLE    = 5'b00001,
           LOOKUP  = 5'b00010,
           MISS    = 5'b00100,
           REPLACE = 5'b01000,
           REFILL  = 5'b10000;

reg [4:0] current_state;
reg [4:0] next_state;

// write_buffer
localparam WRITEBUF_IDLE  = 2'b01,
           WRITEBUF_WRITE = 2'b10;

//write buffer state machine
reg [1:0] writebuf_cstate;
reg [1:0] writebuf_nstate;

//write buffer registers
//write buffer用于存储hitwrite操作的数据
//包括index,路号，offset,写使能，数据(tag,v不用，请思考一下为什么)
reg        write_way;   //写入的路
reg [ 1:0] write_bank;  //写入的缓存块
reg [ 7:0] write_index; //索引
reg [ 3:0] write_strb; //写使能信号
reg [31:0] write_data; //写入的数据

//request buffer
//request buffer用于存储当前的请求信息
//看一眼接口信号（请一定要看）
//包括op,index,tag,offset,wstrb,wdata
reg        reg_valid;
reg        reg_op;
reg [ 7:0] reg_index;
reg [19:0] reg_tag;
reg [ 3:0] reg_offset;
reg [ 3:0] reg_wstrb;
reg [31:0] reg_wdata;
reg [1:0]  reg_mem_type;
wire       uncache_type;

//tag compare parts
//当lookup完成时，这个时候得到对应index的tag和v,进行比较
//得出cache hit/miss的信息
//此时采用的是单周期返回的同步ram，即上一拍发出地址，这一拍得到数据
//而比较也是这样，上一拍发出lookup信号，这一拍得到tag和v，进行比较
wire        way0_v, way1_v;
wire [19:0] way0_tag, way1_tag;
wire        way0_hit, way1_hit;
wire        way0_hit_cacop,way1_hit_cacop;
wire        cache_hit_cacop;
wire        cache_hit;

assign {way0_tag, way0_v} = tagv_w0_rdata;
assign {way1_tag, way1_v} = tagv_w1_rdata;
assign way0_hit = (way0_tag == reg_tag) && way0_v && (~uncache_type);
assign way1_hit = (way1_tag == reg_tag) && way1_v && (~uncache_type);
assign cache_hit = way0_hit || way1_hit;
assign way0_hit_cacop = (way0_tag == reg_tag) && way0_v;
assign way1_hit_cacop = (way1_tag == reg_tag) && way1_v;
assign cache_hit_cacop = way0_hit_cacop || way1_hit_cacop;

//DATA select parts
//这个选出需要返回给cpu的数据，和选出需要执行replace时送入data ram的数据
wire [127:0] way0_data, way1_data;//用于拼接到一起
wire [31:0] way0_load_data, way1_load_data;
wire [31:0] load_data;
wire replace_way;//用于选择替换哪一路
wire [31:0] replace_data;
assign way0_data = {data_w0_rdata[3], data_w0_rdata[2], data_w0_rdata[1], data_w0_rdata[0]};
assign way1_data = {data_w1_rdata[3], data_w1_rdata[2], data_w1_rdata[1], data_w1_rdata[0]};
assign way0_load_data = way0_data[reg_offset[3:2]*32 +: 32]; //从拼接的缓存块中选择一个特定的 32 位数据单元
assign way1_load_data = way1_data[reg_offset[3:2]*32 +: 32];
assign load_data = {32{way0_hit}} & way0_load_data
                   |{32{way1_hit}} & way1_load_data
                   |{32{current_state == REFILL}} & ret_data;
                   //最后一行是cache不命中时的数据返回，虽然返回的ret_data有多个周期，但是我只要能在合适的时间将data_ok拉高，就能对

//MISS buffer
//当发生cache miss时，需要存储一些信息
//包括index,tag,offset,wstrb,wdata
//但是一个经济的考虑就是，miss时，request buffer不变了，可以用来保存对应的请求信息
//因此对于提交替换的信息是不需要额外保存
//对于返回的信息，如果每返回一个32位都及时写入，也是不需要额外保存的
//因此我最多记录一下此时返回了几个32位的数据
reg [1:0] miss_return_count;

//LFSR部分，这玩意就是一个移位寄存器，每位代表要替换的路数，最右侧是这一次使用的
//再用内部的状态，生成一下个进入寄存器的状态
reg [7:0] lfsr_reg;
wire lfsr_feedback;

assign lfsr_feedback = lfsr_reg[7] ^ lfsr_reg[3] ^ lfsr_reg[2] ^ lfsr_reg[1];
always @(posedge clk) begin
    if (reset) begin
        lfsr_reg <= 8'b10101010;
    end else if (ret_valid == 1 & ret_last == 1) begin 
        lfsr_reg <= {lfsr_feedback, lfsr_reg[7:1]};
    end
end
assign replace_way = lfsr_reg[0];

//state machine
//这个地方还需要一些额外的信息
//冲突相关
wire conflict_writing; //表示当前write buffer正在进行写操作,而来了一个load请求要读相同的bank,这个时候主状态机不一定在IDLE，例如连续两条store的情况
wire conflict_load_same_addr; //当处于LOOKUP时，来了一个load请求，地址和现在request buffer里面相同
assign conflict_writing =   (writebuf_cstate == WRITEBUF_WRITE) &
                            (op == READ & valid) &
                            (write_bank == reg_offset[3:2]);
assign conflict_load_same_addr =    (current_state == LOOKUP) &
                                    (reg_op == WRITE & valid) &
                                    (valid && (op == READ))   &
                                   {tag, index, offset[3:2]} == {reg_tag, reg_index, offset[3:2]}; 

//替换块是否为脏的信息,这个是直接从寄存器中读出即可，这个地方其实还有一个问题，就是此时选出的替换块可能是无效的，但是仔细想，只有有效时才会写D，因此不用判断
//这个地方存在一些无效的操作，只是为了简化逻辑
wire replace_part_is_dirty;
assign replace_part_is_dirty = (replace_way == 1'b0) && dirty_way0[reg_index] 
                            || (replace_way == 1'b1) && dirty_way1[reg_index];
wire cacop_part_is_dirty;
assign cacop_part_is_dirty =  (cacop_Hit_Invalidate & ((way0_hit_cacop & dirty_way0[reg_index])|(way1_hit_cacop & dirty_way1[reg_index]))) |
                             (cacop_Index_Invalidate & (cacop_va[0] ?dirty_way0[cacop_va[11:4]] : dirty_way1[cacop_va[11:4]]));

always @(posedge clk) begin
    if (reset) begin
        current_state <= IDLE;
    end else begin
        current_state <= next_state;
    end
end
//请仔细理解当中的状态转换关系
always @(*) begin
    case (current_state)
        IDLE: begin  //当处于IDLE，如果此时有请求，且不与write_buffer冲突，转入LOOKUP
                     //对于cacop，现转入LOOKUP态，再处理，storetag直接处理就可以了
                     //这个地方需要指出，只有当writebuf处于IDLE时，才能转入LOOKUP
            if((valid & ~conflict_writing)|| (cacop_Hit_Invalidate_valid | cacop_Index_Invalidate_valid)) begin
                next_state = LOOKUP;
            end else begin
                next_state = IDLE;
            end
        end
        LOOKUP: begin //cache hit时,如果此时没有冲突，有请求继续LOOKUP，否则IDLE
                      //cache miss时，转入MISS态即可
                      //对于cacop，到这个状态得到对应的tagv数据后，如果是dirty，转入miss态，否则转回IDLE
            if(~reg_valid & cacop_part_is_dirty)
                next_state = MISS;
            else if(~reg_valid & ~cacop_part_is_dirty)
                next_state = IDLE;
            else if(~cache_hit & reg_valid) 
                next_state = MISS;
            else if((~valid) || conflict_writing || conflict_load_same_addr)
                next_state = IDLE;
            else
                next_state = LOOKUP;        
        end
        MISS: begin  //当处于MISS态时，需要等待wr_rdy为高，方便一进入就对其进行写回操作
                     //这个地方还有一个问题，就是如果替换的块是干净的，就不需要写回，直接REPLACE
                     //对于uncache的情况，也是直接REPLACE（此处未实现）
            if((((((wr_rdy == 1) || ~replace_part_is_dirty) && ~uncache_type) || (uncache_type && wr_rdy == 1))&reg_valid)
                || (~reg_valid &&wr_rdy == 1))
                next_state = REPLACE;
            else
                next_state = MISS;
        end
        REPLACE: begin  //处于这个状态，需要把对应脏块写回，并发出读请求
                        //由于是在wr_rdy == 1时进入的这个状态，因此如果要写回脏块一定可以立刻满足
                        //uncache的写可以直接写然后返回
            if(~reg_valid)
                next_state = IDLE;
            else if(uncache_type && (reg_op == WRITE))
                next_state = IDLE;
            else if(rd_rdy == 1)
                next_state = REFILL;
            else
                next_state = REPLACE;
        end
        REFILL: begin //当处于这个状态时，等待数据返回并写入内部ram，可能要在某些时候返回给CPU数据
            if(ret_valid == 1 && ret_last == 1)
                next_state = IDLE;
            else
                next_state = REFILL;
        end
endcase
end

//write buffer state machine
always @(posedge clk) begin
    if (reset) begin
        writebuf_cstate <= WRITEBUF_IDLE;
    end else begin
        writebuf_cstate <= writebuf_nstate;
    end
end

always @(*) begin
    case (writebuf_cstate)
        WRITEBUF_IDLE:begin  //处于LOOKUP,cache写命中转入写状态
            if((current_state == LOOKUP) && (reg_op == WRITE) && cache_hit && reg_valid)
                writebuf_nstate = WRITEBUF_WRITE;
            else
                writebuf_nstate = WRITEBUF_IDLE;
        end
        WRITEBUF_WRITE:begin  //写操作完成后，转入空闲状态，也可直接接受下一次写
            if((current_state == LOOKUP) && (reg_op == WRITE) && cache_hit)
                writebuf_nstate = WRITEBUF_WRITE;
            else
                writebuf_nstate = WRITEBUF_IDLE;
        end
    endcase
end

//现在已经完成了状态机的设计，接下来进行各个信号的设计
//先生成出对应的四种操作信号
//我觉得最好别用nextstate，容易引入组合逻辑环
assign lookup = ((current_state == IDLE) && (valid & ~conflict_writing)) ||
                ((current_state == LOOKUP) && cache_hit&&(valid & ~conflict_writing & ~conflict_load_same_addr));
//这个地方的lookup信号时不能直接控制ram读的，因为使用了cache_hit，这要依赖于ram的读出结果
//对于控制ram读的信号，最好再定义一个信号，上述的用于更新相关的寄存器即可
wire   lookup_ram_en;
assign lookup_ram_en = ((current_state == IDLE) && (valid & ~conflict_writing)) ||
                        ((current_state == LOOKUP)&&(valid & ~conflict_writing & ~conflict_load_same_addr)) && (~uncache_type);
//请一定要理解使用lookup_ram_en控制ram读，有时候会多读，但是为什么不会导致错误的原因
//以及为什么一定要多定义一个信号的原因

assign hitwrite = (writebuf_cstate == WRITEBUF_WRITE);
assign replace = (current_state == MISS) && (reg_valid) &&
                 ((wr_rdy == 1) || (~replace_part_is_dirty));
wire   replace_cacop;
assign replace_cacop = (current_state == MISS) && (~reg_valid) && (wr_rdy == 1);
wire   cacop_invalid_way;
assign cacop_invalid_way = ((cacop_Index_Invalidate | cacop_store_tag) & (cacop_va[0]));
//这个地方的refill就有一点麻烦，按照书上的设计，应该用于更新data数据和tagv数据
//但是现在分为4个bank进行存储，因此此处的refill信号只能表示当前处于refill状态，具体更新哪个bank
//还需要临时决定
assign refill = (current_state == REFILL);
                
//request buffer
always @(posedge clk) begin
    if(reset) begin
        reg_op     <= 1'b0;
        reg_index  <= 8'b0;
        reg_tag    <= 20'b0;
        reg_offset <= 4'b0;
        reg_wstrb  <= 4'b0;
        reg_wdata  <= 32'b0;
        reg_mem_type <= 2'b0;
        reg_valid  <= 1'b0;
    end else if(lookup || (cacop_Hit_Invalidate_valid | cacop_Index_Invalidate_valid)) begin
        reg_op     <= op;
        reg_index  <= index;
        reg_tag    <= tag;
        reg_offset <= offset;
        reg_wstrb  <= wstrb;
        reg_wdata  <= wdata;
        reg_mem_type <= mem_type;
        reg_valid  <= 1'b1; 
    end
end
assign uncache_type = (reg_mem_type == 2'b00);

// write buffer
always @(posedge clk)begin
    if(reset)begin
        write_way <= 1'b0;
        write_bank <= 2'b0;
        write_index <= 8'b0;
        write_strb <= 4'b0;
        write_data <= 32'b0;
    end
    else if((current_state == LOOKUP) && (reg_op == WRITE) && cache_hit)begin
        write_way <= way1_hit;
        write_bank <= reg_offset[3:2];
        write_index <= reg_index;
        write_strb <= reg_wstrb;
        write_data <= reg_wdata;
    end
end

//MISS buffer
always @(posedge clk) begin
    if(reset) begin
        miss_return_count <= 2'b0;
    end else if(replace) begin
        miss_return_count <= 2'b0;
    end else if(refill && ret_valid) begin
        miss_return_count <= miss_return_count + 2'b1;
    end
end

//D表的操作
//当hitwrite,refill时需要写操作
always @(posedge clk)begin
    if(reset)begin
        dirty_way0 <= 256'b0;
        dirty_way1 <= 256'b0;
    end
    else if(cacop_store_tag_valid)begin
        if(~cacop_va[0])
            dirty_way0[cacop_va[11:4]] <= 1'b0;
        else
            dirty_way1[cacop_va[11:4]] <= 1'b0;
    end
    else if(current_state == LOOKUP && ~reg_valid && cacop_Index_Invalidate)begin
        if(~cacop_va[0])
            dirty_way0[cacop_va[11:4]] <= 1'b0;
        else 
            dirty_way1[cacop_va[11:4]] <= 1'b0;
    end
    else if(current_state == LOOKUP && ~reg_valid &&cacop_Hit_Invalidate &&cache_hit_cacop) begin
        if(way0_hit_cacop)
            dirty_way[reg_index] <= 1'b0;
        else 
            dirty_way[reg_index] <= 1'b0; 
    end
    else if(hitwrite)begin
        if(way0_hit)
            dirty_way0[write_index] <= 1'b1;
        else if(way1_hit)
            dirty_way1[write_index] <= 1'b1;
    end
    else if(refill && ret_valid && ret_last && ~uncache_type)begin //uncache的读最后返回时不用更新
        if(replace_way == 1'b0)
            dirty_way0[reg_index] <= 1'b0;
        else if(replace_way == 1'b1)
            dirty_way1[reg_index] <= 1'b0;
    end
end

//{tag,v}表的操作
//在lookup,replace时读，在refill时写，可以重复写多次，也可以就在replace切到refill时写一次
//采取后一种实现,在最后一个数返回时写一次就够
assign tagv_addr =  {8{(cacop_store_tag | cacop_Index_Invalidate)&~reg_valid} & cacop_va[11:4]} |
                    {8{cacop_Hit_Invalidate & ~reg_valid} & reg_index} |
                    {8{lookup_ram_en}} & index |
                    {8{replace}} & reg_index |
                    {8{refill}} & reg_index;
assign tagv_wdata = {21{refill}} & {reg_tag, 1'b1};
assign tagv_w0_en = lookup_ram_en || ((replace || refill)) && (replace_way == 1'b0)
                    || (cacop_store_tag_valid | cacop_Hit_Invalidate_valid | cacop_Index_Invalidate_valid) //IDLE时，store tag写，另外两个读
                    || (current_state == LOOKUP && ~reg_valid && ~cacop_invalid_way && cacop_Index_Invalidate)
                    || (current_state == LOOKUP && ~reg_valid && way0_hit_cacop && cacop_Hit_Invalidate);  //LOOKUP时，修改为无效
assign tagv_w1_en = lookup_ram_en || ((replace || refill)) && (replace_way == 1'b1)
                    || (cacop_store_tag_valid | cacop_Hit_Invalidate_valid | cacop_Index_Invalidate_valid)
                    || (current_state == LOOKUP && ~reg_valid && cacop_invalid_way && cacop_Index_Invalidate)
                    || (current_state == LOOKUP && ~reg_valid && way1_hit_cacop && cacop_Hit_Invalidate);
assign tagv_w0_we = refill&& (replace_way == 1'b0)&& ret_valid && ret_last && (~uncache_type)
                    || (cacop_store_tag_valid && ~cacop_invalid_way)
                    || (current_state == LOOKUP && ~reg_valid && ~cacop_invalid_way && cacop_Index_Invalidate)
                    || (current_state == LOOKUP && ~reg_valid && way0_hit_cacop && cacop_Hit_Invalidate);  
assign tagv_w1_we = refill&& (replace_way == 1'b1)&& ret_valid && ret_last && (~uncache_type)
                     || (cacop_store_tag_valid && cacop_invalid_way)
                     || (current_state == LOOKUP && ~reg_valid && cacop_invalid_way && cacop_Index_Invalidate)
                     || (current_state == LOOKUP && ~reg_valid && way1_hit_cacop && cacop_Hit_Invalidate);

//data表的操作
//在lookup,replace时读，在hitwrite,refill时写
//当中hitwrite和refill可能同时进行
//因此需要阻塞
//为了优化性能
//采取不同的bank块进行存储，从而减少冲突
assign data_w0_en[0] =  lookup_ram_en && (offset[3:2] == 2'b00) ||
                        hitwrite && (write_bank == 2'b00) && (write_way == 1'b0) ||
                        (replace) && (replace_way == 1'b0) ||
                        refill && (replace_way == 1'b0)&& (~uncache_type)
                        || (replace_cacop); //这个地方讲道理应该是当对应bank数据返回再片选，但是也可以一直片选，不是对应路就读，也没有什么额外的问题
assign data_w0_en[1] =  lookup_ram_en && (offset[3:2] == 2'b01) ||
                        hitwrite && (write_bank == 2'b01) && (write_way == 1'b0) ||
                        (replace) && (replace_way == 1'b0) ||
                        refill && (replace_way == 1'b0)&& (~uncache_type)
                        || (replace_cacop);
assign data_w0_en[2] =  lookup_ram_en && (offset[3:2] == 2'b10) ||
                        hitwrite && (write_bank == 2'b10) && (write_way == 1'b0) ||
                        (replace) && (replace_way == 1'b0) ||
                        refill && (replace_way == 1'b0)&& (~uncache_type)
                        || (replace_cacop);
assign data_w0_en[3] =  lookup_ram_en && (offset[3:2] == 2'b11) ||
                        hitwrite && (write_bank == 2'b11) && (write_way == 1'b0) ||
                        (replace) && (replace_way == 1'b0) ||
                        refill && (replace_way == 1'b0) && (~uncache_type)
                        || (replace_cacop);
assign data_w1_en[0] =  lookup_ram_en && (offset[3:2] == 2'b00) ||
                        hitwrite && (write_bank == 2'b00) && (write_way == 1'b1) ||
                        (replace) && (replace_way == 1'b1) ||
                        refill && (replace_way == 1'b1) && (~uncache_type)
                        || (replace_cacop);
assign data_w1_en[1] =  lookup_ram_en && (offset[3:2] == 2'b01) ||
                        hitwrite && (write_bank == 2'b01) && (write_way == 1'b1) ||
                        (replace) && (replace_way == 1'b1) ||
                        refill && (replace_way == 1'b1) && (~uncache_type)
                        || (replace_cacop);
assign data_w1_en[2] =  lookup_ram_en && (offset[3:2] == 2'b10) ||
                        hitwrite && (write_bank == 2'b10) && (write_way == 1'b1) ||
                        (replace) && (replace_way == 1'b1) ||
                        refill && (replace_way == 1'b1) && (~uncache_type)
                        || (replace_cacop);
assign data_w1_en[3] =  lookup_ram_en && (offset[3:2] == 2'b11) ||
                        hitwrite && (write_bank == 2'b11) && (write_way == 1'b1) ||
                        (replace) && (replace_way == 1'b1) ||
                        refill && (replace_way == 1'b1) && (~uncache_type)
                        || (replace_cacop);
//这个地方的we是一个四位信号，兼具表示是读写操作和写使能的作用
assign data_w0_we[0] = {4{hitwrite && (write_bank == 2'b00) && (write_way == 1'b0)}} & write_strb |
                        {4{refill && (replace_way == 1'b0) && ret_valid && miss_return_count == 2'b00}} & 4'b1111;
assign data_w0_we[1] = {4{hitwrite && (write_bank == 2'b01) && (write_way == 1'b0)}} & write_strb |
                        {4{refill && (replace_way == 1'b0) && ret_valid && miss_return_count == 2'b01}} & 4'b1111;
assign data_w0_we[2] = {4{hitwrite && (write_bank == 2'b10) && (write_way == 1'b0)}} & write_strb |
                        {4{refill && (replace_way == 1'b0) && ret_valid && miss_return_count == 2'b10}} & 4'b1111;
assign data_w0_we[3] = {4{hitwrite && (write_bank == 2'b11) && (write_way == 1'b0)}} & write_strb |
                        {4{refill && (replace_way == 1'b0) && ret_valid && miss_return_count == 2'b11}} & 4'b1111;
assign data_w1_we[0] = {4{hitwrite && (write_bank == 2'b00) && (write_way == 1'b1)}} & write_strb |
                        {4{refill && (replace_way == 1'b1) && ret_valid && miss_return_count == 2'b00}} & 4'b1111;
assign data_w1_we[1] = {4{hitwrite && (write_bank == 2'b01) && (write_way == 1'b1)}} & write_strb |
                        {4{refill && (replace_way == 1'b1) && ret_valid && miss_return_count == 2'b01}} & 4'b1111;
assign data_w1_we[2] = {4{hitwrite && (write_bank == 2'b10) && (write_way == 1'b1)}} & write_strb |
                        {4{refill && (replace_way == 1'b1) && ret_valid && miss_return_count == 2'b10}} & 4'b1111;
assign data_w1_we[3] = {4{hitwrite && (write_bank == 2'b11) && (write_way == 1'b1)}} & write_strb |
                        {4{refill && (replace_way == 1'b1) && ret_valid && miss_return_count == 2'b11}} & 4'b1111;

//这个地方选择送入的数据
//对于hitwrite,送入write buffer的数据
//对于refill可能送入从内存返回的数据或者待写入的数据
//这个地方有一个麻烦的问题
//即对于写入从cpu来的数据，可能存在部分写入的情况，即写一个字节或者两个字节
//因此需要对数据进行处理
wire [31:0] refill_data;
wire cpu_write_op;
wire [31:0] mix_data;
assign cpu_write_op = (miss_return_count == reg_offset[3:2]) && (reg_op == WRITE);
assign mix_data = {{reg_wstrb[3]} ? reg_wdata[31:24] : ret_data[31:24],
                   {reg_wstrb[2]} ? reg_wdata[23:16] : ret_data[23:16],
                   {reg_wstrb[1]} ? reg_wdata[15:8]  : ret_data[15:8],
                   {reg_wstrb[0]} ? reg_wdata[7:0]   : ret_data[7:0]};
assign refill_data = {32{cpu_write_op}} ?mix_data :
                     ret_data;
assign data_wdata = {32{hitwrite}} & write_data |
                    {32{refill}} & refill_data;
//对于地址的生成，由于可能同时一个操作写一个bank，另一个读另外一个bank
//因此四组bank需要四组地址
//四组操作时都需要地址
//replace时，地址来自replace buffer
//refill时，地址来自request buffer
//lookup时，地址来自index
//hitwrite时，地址来自write buffer
//对于下面两个信号，可能同时存在，要依据读的bank予以选择
assign data_addr[0] = (replace || refill) ? reg_index :
                      (hitwrite && (write_bank == 2'b00)) ? write_index :
                      index;
assign data_addr[1] = (replace || refill) ? reg_index :
                      (hitwrite && (write_bank == 2'b01)) ? write_index :
                        index;
assign data_addr[2] = (replace || refill) ? reg_index :
                      (hitwrite && (write_bank == 2'b10)) ? write_index :
                        index;
assign data_addr[3] = (replace || refill) ? reg_index :
                      (hitwrite && (write_bank == 2'b11)) ? write_index :
                        index;

//cache与总线的交互接口
//对于uncache的写不用发出读请求
assign rd_req = (current_state == REPLACE)&& ~(uncache_type && (reg_op == WRITE));
//对于uncache的读请求，只需要一个32字节，cache读小于32字节也是32字节
assign rd_type = {3{~uncache_type}} & READ_BLOCK
                |{3{ uncache_type}} & READ_WORD;
assign rd_addr =  {32{~uncache_type}} & {reg_tag, reg_index, 4'b0000}
                |{32{ uncache_type}} & {reg_tag, reg_index, reg_offset[3:2], 2'b00};


//wr_req在刚刚进入REPLACE时拉高
//请注意这一块的实现
//对于uncache的读请求，不用发出写请求
reg wr_req_reg;
always @(posedge clk) begin
    if (reset) begin
        wr_req_reg <= 1'b0;
    end else if (current_state == MISS && next_state == REPLACE && 
    ((reg_valid && (replace_part_is_dirty||uncache_type) && ~(uncache_type && (reg_op == READ)))
    ||(~reg_valid))) begin
        wr_req_reg <= 1'b1;
    end else if(wr_rdy)begin
        wr_req_reg <= 1'b0;
    end
end
reg [19:0] reg_cacop_tag; //由于在LOOKUP时，会将tag清零，因此需要保存写回的tag
always @(posedge clk) begin
    if(reset)
        reg_cacop_tag <= 20'b0;
    else if(cacop_va[0])
        reg_cacop_tag <= tagv_w1_rdata[20:1];
    else if(cacop_va[1])
        reg_cacop_tag <= tagv_w0_rdata[20:1];
end
assign wr_req = wr_req_reg;
assign wr_type = {3{~uncache_type && reg_op && reg_valid}} & WRITE_BLOCK
                |{3{ uncache_type && reg_valid}} & WRITE_WORD
                |{3{~reg_valid} & WRITE_BLOCK }; 
assign wr_addr = {32{uncache_type & reg_valid}} & {reg_tag, reg_index, reg_offset}
                |{32{~uncache_type & reg_valid}} &
                ({32{replace_way == 1'b0}} & {way0_tag, reg_index, 4'b0000} |
                 {32{replace_way == 1'b1}} & {way1_tag, reg_index, 4'b0000})
                | {32{~reg_valid & cacop_Index_Invalidate}} & {reg_cacop_tag,cacop_va[11:4],4'b0000}
                | {32{~reg_valid & cacop_Hit_Invalidate}} & {cacop_va[31:4],4'b0000};
assign wr_wstrb = {4{ uncache_type & reg_valid}} & reg_wstrb
                | {4{~uncache_type & reg_valid}} &4'b1111; 
                | {4{~reg_valid}} & 4'b1111;
assign wr_data = {128{uncache_type & reg_valid}} & {96'b0, reg_wdata}
                |{128{~uncache_type & reg_valid}} &
                ({128{replace_way == 1'b0}} & way0_data |
                 {128{replace_way == 1'b1}} & way1_data) 
                | {128{~reg_valid}} & 
                (cacop_va[0] ? way1_data : way0_data);
//cpu接口信号
//此处有一个问题，就是如果addr_ok依赖于valid，那么在cpu内存应该注意req的发出不能依赖于addr_ok;
//这个地方addr_ok不用改变
assign addr_ok = (current_state == IDLE) && valid && (~conflict_writing)||
                (current_state == LOOKUP) && cache_hit && valid && (~conflict_writing & ~conflict_load_same_addr);
//这个地方data_ok,对于读操作的uncache，最后refill拉高，对于写操作的uncache，在进入REPLACE时拉高
assign data_ok = (current_state == LOOKUP) && (cache_hit || (reg_op == WRITE && ~uncache_type)) &&reg_valid ||
                (current_state == REPLACE) && (reg_op == WRITE && uncache_type&& reg_valid) ||
                (current_state == REFILL) && ret_valid && ((miss_return_count == reg_offset[3:2] && ~uncache_type)|| (ret_last && uncache_type)) && (reg_op == READ);
//此处有一个细节，就是store操作即使不命中，也是直接data_ok拉高
//这样当refill时，只有读才需要拉高data_ok
assign rdata = load_data;

endmodule