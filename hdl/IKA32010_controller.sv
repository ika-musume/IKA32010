module IKA32010_controller
(
    //chip clock
    input   wire            i_EMUCLK, //emulator master clock
    input   wire            i_CLKIN_PCEN_n, //CLKIN positive edge enable

    //clock out
    output  wire            o_CLKOUT,
    output  wire            o_CLKOUT_PCEN_n,
    output  wire            o_CLKOUT_NCEN_n,

    //chip reset
    input   wire            i_RS_n,

    //bus control
    output  reg             o_MEN_n, //external instruction read
    output  reg             o_DEN_n, //IN instruction
    output  reg             o_WE_n, //OUT instruction

    output  wire    [11:0]  o_AOUT,
    input   wire    [15:0]  i_DIN,
    output  reg     [15:0]  o_DOUT,
    output  reg             o_DOUT_OE_n,

    //flag
    input   reg             i_BIO_n,

    //interrupt
    input   reg             i_INT_n,

    input   wire    [15:0]  i_DATABUS_READ,
    output  wire    [15:0]  o_DATABUS_WRITE
);

`define IKA32010_DISASSEMBLY
`define IKA32010_DISASSEMBLY_SHOWID
`define IKA32010_DEVICE_ID "ikakawa"


///////////////////////////////////////////////////////////
//////  Clock
////

//master cycle counter
reg     [1:0]   cyclecntr;
always @(posedge i_EMUCLK) begin
    if(!i_CLKIN_PCEN_n) begin
        if(!i_RS_n) cyclecntr <= 2'd0;
        else begin
            if(cyclecntr == 2'd3) cyclecntr <= 2'd0;
            else cyclecntr <= cyclecntr + 2'd1;
        end
    end
end

//divided clock
assign  o_CLKOUT = cyclecntr[1];
assign  o_CLKOUT_NCEN_n = ~(cyclecntr == 2'd3) | i_CLKIN_PCEN_n;
assign  o_CLKOUT_PCEN_n = ~(cyclecntr == 2'd1) | i_CLKIN_PCEN_n;

assign  ncen_n = o_CLKOUT_NCEN_n;
assign  pcen_n = o_CLKOUT_PCEN_n;



///////////////////////////////////////////////////////////
//////  I/O status pin
////

//sampled at every positive edge?
reg             bio_n;
always @(posedge i_EMUCLK) if(!pcen_n) begin
    bio_n <= i_BIO_n;
end



///////////////////////////////////////////////////////////
//////  Opcode register
////

//opcode register
reg     [15:0]  if_opcodereg;
reg             if_opcodereg_force_zero;



///////////////////////////////////////////////////////////
//////  Peripheral address register
////

reg     [2:0]   ex_addr_peripheral; //address for peripheral(PA)
reg             ex_addr_peripheral_ld;
always @(posedge i_EMUCLK) if(!ncen_n) begin
    ex_addr_peripheral <= (ex_addr_peripheral_ld) ? if_opcodereg[10:8] : ex_addr_peripheral;
end



///////////////////////////////////////////////////////////
//////  Program counter
////

reg     [15:0]  register_wrbus;

reg     [11:0]  if_pc; //program counter
reg     [2:0]   if_pc_modesel;
wire    [11:0]  if_pc_next = (if_pc == 12'hFFF) ? 12'd000 : if_pc + 12'h001;

localparam  PC_HOLD             = 3'd0;
localparam  PC_INCREASE         = 3'd1;
localparam  PC_LOAD_IMMEDIATE   = 3'd2;
localparam  PC_LOAD_INTERRUPT   = 3'd3;
localparam  PC_LOAD_WRBUS       = 3'd4;
localparam  PC_RESET            = 3'd5;
localparam  DO_RESET            = 1'b0;
localparam  DO_INCREASE         = 1'b1;


always @(posedge i_EMUCLK) begin
    if(!i_RS_n) begin
        if_pc <= 12'h000;
    end
    else begin
        if(!o_CLKOUT_NCEN_n) begin
            case(if_pc_modesel)
                PC_HOLD          : if_pc <= if_pc;
                PC_INCREASE      : if_pc <= if_pc_next;
                PC_LOAD_IMMEDIATE: if_pc <= i_DIN[11:0];
                PC_LOAD_INTERRUPT: if_pc <= 12'h002;
                PC_LOAD_WRBUS    : if_pc <= register_wrbus[11:0];
                PC_RESET         : if_pc <= 12'h000;
                default: if_pc <= if_pc;
            endcase
        end
    end
end



///////////////////////////////////////////////////////////
//////  Bus controller
////

//current mode latch
reg     [3:0]   busctrl_mode; 

//address output
localparam  BUSCTRL_ADDR_PC         = 1'd0;
localparam  BUSCTRL_ADDR_PERIPHERAL = 1'd1;

reg             busctrl_addr_muxsel; //control signal from the microcode
reg     [11:0]  busctrl_addr; //MUX, controlled by the microcode
assign o_AOUT = busctrl_addr;

always @(*) begin
    case(busctrl_mode[3])
        1'b0: busctrl_addr <= if_pc; //program counter + data offset
        1'b1: busctrl_addr <= {9'b0000_0000_0, ex_addr_peripheral}; //PA0 1 2
    endcase
end


//outlatch/inlatch
reg     [15:0]  busctrl_outlatch, busctrl_inlatch;
assign  if_pc_immediate_addr = busctrl_inlatch[11:0]; //to program counter

//microcode should reset these things
reg     [2:0]   busctrl_req; //0 = stop, 1 = instruction read, 
                             //2 = table read, 3= table write, 
                             //4 = IN, 5 = OUT

localparam  BUSCTRL_STOP        = 3'd0;
localparam  OPCODE_READ         = 3'd1;
localparam  DATA_READ          = 3'd2;
localparam  DATA_WRITE         = 3'd3;
localparam  COMMAND_IN          = 3'd4;
localparam  COMMAND_OUT         = 3'd5;

always @(*) begin
    busctrl_mode[2:0] <= busctrl_req; //latch request type
    busctrl_mode[3] <= busctrl_addr_muxsel;
end

always @(posedge i_EMUCLK) begin
    if(!i_RS_n) begin
        busctrl_outlatch <= 16'h0000;
        busctrl_inlatch <= 16'h0000;

        if_opcodereg <= 16'h0000;
    end
    else begin
        if(!i_CLKIN_PCEN_n) begin
            if(cyclecntr == 2'd3) begin
                if(if_opcodereg_force_zero) if_opcodereg <= 16'h0000;
                else begin
                    if(busctrl_mode[2:0] == 3'd1) if_opcodereg <= i_DIN;
                end
            end

            //no transaction
            if(busctrl_mode[2:0] == 3'd0) begin
                case(cyclecntr)
                    2'd0: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                    2'd1: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                    2'd2: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                    2'd3: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                endcase
            end

            //instruction read
            else if(busctrl_mode[2:0] == 3'd1) begin
                case(cyclecntr)
                    2'd0: begin o_MEN_n <= 1'b0; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                    2'd1: begin o_MEN_n <= 1'b0; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                    2'd2: begin o_MEN_n <= 1'b0; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                    2'd3: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                endcase
            end

            //table read
            else if(busctrl_mode[2:0] == 3'd2) begin
                case(cyclecntr)
                    2'd0: begin o_MEN_n <= 1'b0; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                    2'd1: begin o_MEN_n <= 1'b0; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                    2'd2: begin o_MEN_n <= 1'b0; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                    2'd3: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1;
                            busctrl_inlatch <= i_DIN; end
                endcase
            end

            //table write
            else if(busctrl_mode[2:0] == 3'd3) begin
                case(cyclecntr)
                    2'd0: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                    2'd1: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b0;
                            o_DOUT <= busctrl_outlatch; end
                    2'd2: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b1; o_WE_n <= 1'b0; o_DOUT_OE_n <= 1'b0; end
                    2'd3: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                endcase
            end

            //IN instruction
            else if(busctrl_mode[2:0] == 3'd4) begin
                case(cyclecntr)
                    2'd0: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b0; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                    2'd1: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b0; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                    2'd2: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b0; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                    2'd3: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1;
                            busctrl_inlatch <= i_DIN; end
                endcase
            end

            //OUT instruction
            else if(busctrl_mode[2:0] == 3'd5) begin
                case(cyclecntr)
                    2'd0: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                    2'd1: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b0;
                            o_DOUT <= busctrl_outlatch; end
                    2'd2: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b1; o_WE_n <= 1'b0; o_DOUT_OE_n <= 1'b0; end
                    2'd3: begin o_MEN_n <= 1'b1; o_DEN_n <= 1'b1; o_WE_n <= 1'b1; o_DOUT_OE_n <= 1'b1; end
                endcase
            end

            //not defined
            else begin

            end
        end
    end
end








///////////////////////////////////////////////////////////
//////  Internal write bus control
////

//sources
reg     [15:0]  shb_output;
wire    [15:0]  ram_output;
wire    [15:0]  ar_data_output;
wire    [11:0]  stk_output;

//select write bus data
localparam  WRBUS_SOURCE_SHB    = 3'd0;
localparam  WRBUS_SOURCE_RAM    = 3'd1;
localparam  WRBUS_SOURCE_AR     = 3'd2;
localparam  WRBUS_SOURCE_STK    = 3'd3;
localparam  WRBUS_SOURCE_INST   = 3'd4;
reg     [2:0]   register_wrbus_source_sel; //0 SHIFTER-B, 1 RAM, 2 AR, 3 INSTRUCTION

always @(*) begin
    case(register_wrbus_source_sel)
        WRBUS_SOURCE_SHB : register_wrbus = shb_output;
        WRBUS_SOURCE_RAM : register_wrbus = ram_output;
        WRBUS_SOURCE_AR  : register_wrbus = ar_data_output;
        WRBUS_SOURCE_STK : register_wrbus = {4'h0, stk_output};
        WRBUS_SOURCE_INST: register_wrbus = {8'h00, if_opcodereg[7:0]};
        default          : register_wrbus = 16'h0000;
    endcase
end









///////////////////////////////////////////////////////////
//////  System registers
////

//alu overflow mode bit
reg             reg_ovm; //0 = overflow disabled, 1 = overflow enabled 
reg             reg_ovm_set, reg_ovm_rst;
always @(posedge i_EMUCLK) begin
    if(!i_RS_n) reg_ovm <= 1'b0;
    else begin
        if(!ncen_n) begin
            case({reg_ovm_set, reg_ovm_rst})
                2'b01: reg_ovm <= 1'b0;
                2'b10: reg_ovm <= 1'b1;
                default: reg_ovm <= reg_ovm;
            endcase
        end
    end
end

//alu overflow mode bit
reg             reg_intm; //0 = interrupt enabled, 1 = interrupt disabled
reg             reg_intm_en, reg_intm_dis;
always @(posedge i_EMUCLK) if(!ncen_n) begin
    case({reg_intm_en, reg_intm_dis})
        2'b00: reg_intm <= reg_intm;
        2'b01: reg_intm <= 1'b1;
        2'b10: reg_intm <= 1'b0;
        2'b11: reg_intm <= reg_intm;
    endcase
end



//auxillary register pointer and register
reg             reg_arp_ld;
reg             reg_arp;

reg             reg_ar_ld; //AR reg load
reg             reg_ar_inc, reg_ar_dec; //increase decrease AR
reg     [15:0]  reg_ar[0:1];

assign          ar_data_output = reg_ar[if_opcodereg[8]]; //used to save AR data 
wire    [7:0]   ar_addr_output = reg_ar[reg_arp][7:0]; //use AR data as RAM address


always @(posedge i_EMUCLK) begin
    if(!i_RS_n) begin
        reg_arp <= 1'b0;
        reg_ar[0] <= 16'h0000;
        reg_ar[1] <= 16'h0000;
    end
    else begin
        if(!ncen_n) begin
            //auxillary register pointer
            reg_arp <= (reg_arp_ld) ? register_wrbus[0] : reg_arp;

            //auxillary register
            if(reg_ar_ld) begin
                reg_ar[if_opcodereg[8]] <= register_wrbus;
            end
            else begin
                case({reg_ar_inc, reg_ar_dec})
                    2'b00: reg_ar[reg_arp] <= reg_ar[reg_arp];
                    2'b01: reg_ar[reg_arp] <= reg_ar[reg_arp] - 16'd1;
                    2'b10: reg_ar[reg_arp] <= reg_ar[reg_arp] + 16'd1;
                    2'b11: reg_ar[reg_arp] <= reg_ar[reg_arp];
                endcase
            end
        end
    end
end

//data memory page pointer
reg             reg_dp, reg_dp_ld;
always @(posedge i_EMUCLK) begin
    if(i_RS_n) begin
        reg_dp <= 1'b0;
    end
    else begin
        if(!ncen_n) reg_dp <= (reg_dp_ld) ? register_wrbus[0] : reg_dp;
    end
end




///////////////////////////////////////////////////////////
//////  Interrupt 
////

reg             int_ack;
reg             int_n_z, int_n_zz, int_n_zzz, int_latched;
wire            int_rq = int_latched; //& ~reg_intm;
always @(posedge i_EMUCLK) begin
    if(!i_RS_n) begin
        int_n_z <= 1'b1;
        int_n_zz <= 1'b1;
        int_n_zzz <= 1'b1;
    end
    else begin
        if(!ncen_n) int_n_z <= i_INT_n;
        if(!pcen_n) int_n_zz <= int_n_z;
        if(!ncen_n) int_n_zzz <= int_n_zz;
    end

    if(!i_RS_n) begin
        int_latched <= 1'b0;
    end
    else begin
        if(!ncen_n) begin
            if(int_ack) int_latched <= 1'b0;
            else begin
                if(~int_n_zz & int_n_zzz) int_latched <= 1'b1;
            end
        end
    end
end




///////////////////////////////////////////////////////////
//////  Misc control bits
////

reg             reg_t_ld; //T reg load
reg             reg_p_ld; //P reg load

reg     [31:0]  reg_p;




///////////////////////////////////////////////////////////
//////  Stack
////

reg             stk_data_sel; //0 = ACC, 1 = PC
reg             stk_pop, stk_push;
localparam  STACK_DATA_ACC  = 1'b0;
localparam  STACK_DATA_PC   = 1'b1;

IKA32010_stack u_stack (
    .i_EMUCLK(i_EMUCLK), .i_CEN_n(ncen_n), .i_RST_n(i_RS_n),
    .i_PUSH(stk_push), .i_POP(stk_pop),
    .i_DIN(stk_data_sel ? if_pc : register_wrbus[11:0]), .o_DOUT(stk_output)
);



///////////////////////////////////////////////////////////
//////  ALU-ACC control
////

//shifter-A(ALU input)
reg     [4:0]   sha_amt; //ALU input shifter control
reg     [31:0]  sha_output;
always @(*) begin
    sha_output = {{16{register_wrbus[15]}}, register_wrbus}; //sign extension
    sha_output = sha_output << sha_amt; //do arithmetic shift
end


//define ALU mode/control parameters
reg     [3:0]   alu_modesel; //000 AND, 001 OR,  010 XOR, 011 ABS
                             //100 ADD, 101 SUB, 110 SUBC,
localparam  ALU_AND             = 3'd0;
localparam  ALU_OR              = 3'd1;
localparam  ALU_XOR             = 3'd2;
localparam  ALU_ABS             = 3'd3;
localparam  ALU_ADD             = 3'd4;
localparam  ALU_SUB             = 3'd5;
localparam  ALU_SUBC            = 3'd6;

reg             alu_paz; //force alu port A zero
reg             alu_pbz; //force alu port B zero
reg     [1:0]   alu_pbdata;
localparam  ALU_PBDATA_LONGWORD = 2'd0;
localparam  ALU_PBDATA_HIGHWORD = 2'd1;
localparam  ALU_PBDATA_LOWWORD  = 2'd2;
localparam  ALU_PBDATA_BYTE     = 2'd3;  

reg             alu_pbsel; //alu port B source select(0 = shifter, 1 = multiplier)
localparam  ALU_SOURCE_SHFT     = 1'b0;
localparam  ALU_SOURCE_MUL      = 1'b1;

reg             alu_acc_ld;
wire    [31:0]  alu_acc_output;
wire            alu_flag_zero, alu_flag_neg, alu_flag_ovfl;

IKA32010_alu u_alu (
    .i_EMUCLK(i_EMUCLK), .i_CEN_n(ncen_n), .i_RST_n(i_RS_n),
    .i_ALU_OVM(reg_ovm), .i_ALU_MODESEL(alu_modesel), .i_ALU_PAZ(alu_paz), .i_ALU_PBZ(alu_pbz), .i_ALU_PBDATA(alu_pbdata),
    .i_ALU_PA(alu_acc_output), .i_ALU_PB(alu_pbsel ? reg_p : sha_output),
    .i_ALU_ACC_LD(alu_acc_ld), 
    .o_ALU_ACC_OUTPUT(alu_acc_output),
    .o_Z(alu_flag_zero), .o_N(alu_flag_neg), .o_V(alu_flag_ovfl)
);


//shifter-B(accumulator output)
reg             shb_mux;
reg     [2:0]   shb_amt; //ALU output shifter control

wire    [15:0]  shb_hilo = shb_mux ? alu_acc_output[31:16] : alu_acc_output[15:0];
always @(*) begin
    case(shb_amt)
        3'd0: shb_output = shb_hilo;
        3'd1: shb_output = shb_hilo << 1;
        3'd4: shb_output = shb_hilo << 4;
        default: shb_output = shb_hilo;
    endcase
end



///////////////////////////////////////////////////////////
//////  Data RAM
////

//RAM address source select(0 = direct, 1 = indirect)
wire    [7:0]   ram_addr = if_opcodereg[7] ? ar_addr_output : {reg_dp, if_opcodereg[6:0]};
reg             ram_rd, ram_wr;

IKA32010_ram u_ram (
    .i_EMUCLK(i_EMUCLK), .i_CEN_n(pcen_n),
    .i_WE(ram_wr), .i_ADDR(ram_addr), .i_DIN(register_wrbus), .o_DOUT(ram_output)
);



///////////////////////////////////////////////////////////
//////  Microcode
////

localparam  YES = 1'b1;
localparam  NO  = 1'b0;
localparam  HIGH = 1'b1;
localparam  LOW = 1'b0;

//processor state
reg     [2:0]   ex_state;
/*
    000: reset low
    001: prepare to start 
    010: normal operation
*/


//instruction cycle counter
reg     [1:0]   ex_inst_cycle;
reg             ex_inst_cycle_rst;
always @(posedge i_EMUCLK) begin
    if(!o_CLKOUT_NCEN_n) begin
        ex_inst_cycle <= (ex_inst_cycle_rst) ? 2'd0 : ex_inst_cycle + 2'd1;
    end
end


//interrupt control
reg             ex_int_start = 1'b0;

//reset delay
reg             rs_n_z, rs_n_zz;
always @(posedge i_EMUCLK) begin
    if(!i_RS_n) begin
        rs_n_z <= 1'b0; //reset state
    end
    else begin
        if(!ncen_n) begin
            rs_n_z <= i_RS_n;
            rs_n_zz <= rs_n_z;
        end
    end
end

//processor state
always @(posedge i_EMUCLK) begin
    if(!i_RS_n) begin
        //state
        ex_state <= 3'b000; //reset state
    end
    else begin
        if(!o_CLKOUT_NCEN_n) begin
            //reset state
            if(ex_state == 3'b000) if((~rs_n_zz & rs_n_z) == 1'b1) ex_state <= 3'b001;

            //normal operation
            else if(ex_state == 3'b001) begin

            end
        end
    end
end

//disassembly message
`ifdef IKA32010_DISASSEMBLY
string  disasm, num_data;

//NOP, ABS
function void disasm_type0;
    input   string  mnemonic;
    input   [11:0]  pc;
    disasm = "";
    `ifdef IKA32010_DISASSEMBLY_SHOWID
        disasm = {"IKA32010_", `IKA32010_DEVICE_ID, ": "};
    `endif
    $sformat(num_data, " PC=0x%3h |", {pc-1}[11:0]);
    disasm = {disasm, num_data};
    disasm = {disasm, " ", mnemonic};
    disasm = {disasm, "\n"};
    $display(disasm);
endfunction

//ADD, LAC, SUB...
function void disasm_type1;
    input   string  mnemonic;
    input   [15:0]  opcodereg;
    input   [11:0]  pc;
    disasm = "";
    `ifdef IKA32010_DISASSEMBLY_SHOWID
        disasm = {"IKA32010_", `IKA32010_DEVICE_ID, ": "};
    `endif
    $sformat(num_data, " PC=0x%h |", {pc-1}[11:0]);
    disasm = {disasm, num_data};
    disasm = {disasm, " ", mnemonic};
    if(!opcodereg[7]) begin
        $sformat(num_data, " DAT0x%h, %d", opcodereg[6:0], opcodereg[11:8]);
        disasm = {disasm, num_data};
    end
    else begin
             if(opcodereg[5:4] == 2'b00) disasm = {disasm, " *"}; //inc/dec
        else if(opcodereg[5:4] == 2'b01) disasm = {disasm, " *-"};
        else if(opcodereg[5:4] == 2'b10) disasm = {disasm, " *+"};
        $sformat(num_data, ", %d", opcodereg[11:8]);
        disasm = {disasm, num_data};
        if(opcodereg[3]) begin
            num_data = "";
            $sformat(num_data, ", %b", opcodereg[0]);
            disasm = {disasm, num_data};
        end
    end
    disasm = {disasm, "\n"};
    $display(disasm);
endfunction

//ADDH, AND, OR...
function void disasm_type2;
    input   string  mnemonic;
    input   [15:0]  opcodereg;
    input   [11:0]  pc;
    input           aux;
    disasm = "";
    `ifdef IKA32010_DISASSEMBLY_SHOWID
        disasm = {"IKA32010_", `IKA32010_DEVICE_ID, ": "};
    `endif
    $sformat(num_data, " PC=0x%h |", {pc-1}[11:0]);
    disasm = {disasm, num_data};
    disasm = {disasm, " ", mnemonic};
    if(aux) begin
        $sformat(num_data, " AR%b", opcodereg[8]);
        disasm = {disasm, num_data};
    end
    if(!opcodereg[7]) begin
        $sformat(num_data, " DAT0x%h, %d", opcodereg[6:0], opcodereg[11:8]);
        disasm = {disasm, num_data};
    end
    else begin
             if(opcodereg[5:4] == 2'b00) disasm = {disasm, " *"}; //inc/dec
        else if(opcodereg[5:4] == 2'b01) disasm = {disasm, " *-"};
        else if(opcodereg[5:4] == 2'b10) disasm = {disasm, " *+"};
        if(opcodereg[3]) begin
            num_data = "";
            $sformat(num_data, ", %b", opcodereg[0]);
            disasm = {disasm, num_data};
        end
    end
    disasm = {disasm, "\n"};
    $display(disasm);
endfunction

//LACK
function void disasm_type3;
    input   string  mnemonic;
    input   [15:0]  opcodereg;
    input   [11:0]  pc;
    input           aux;
    disasm = "";
    `ifdef IKA32010_DISASSEMBLY_SHOWID
        disasm = {"IKA32010_", `IKA32010_DEVICE_ID, ": "};
    `endif
    $sformat(num_data, " PC=0x%h |", {pc-1}[11:0]);
    disasm = {disasm, num_data};
    disasm = {disasm, " ", mnemonic};
    if(aux) begin
        $sformat(num_data, " AR%b", opcodereg[8]);
        disasm = {disasm, num_data};
    end
    $sformat(num_data, " 0x%h", opcodereg[7:0]);
    disasm = {disasm, num_data};
    disasm = {disasm, "\n"};
    $display(disasm);
endfunction

//B
function void disasm_type4;
    input   string  mnemonic;
    input   [11:0]  pc;
    input   [1:0]   cycle;
    input   [15:0]  branch;
    input           ret;
    if(cycle == 2'd0) begin
        disasm = "";
        `ifdef IKA32010_DISASSEMBLY_SHOWID
            disasm = {"IKA32010_", `IKA32010_DEVICE_ID, ": "};
        `endif
        $sformat(num_data, " PC=0x%h |", {pc-1}[11:0]);
        disasm = {disasm, num_data};
        disasm = {disasm, " ", mnemonic};
    end
    else if(cycle == 2'd1) begin
        if(ret == 1'b0) begin
            $sformat(num_data, " 0x%h", branch[11:0]);
            disasm = {disasm, num_data};
            disasm = {disasm, "\n"};
            $display(disasm);
        end
        else begin
            disasm = {disasm, "\n"};
            $display(disasm);
        end
    end
endfunction

//LARP, LDPK
function void disasm_type5;
    input   string  mnemonic;
    input   [15:0]  opcodereg;
    input   [11:0]  pc;
    disasm = "";
    `ifdef IKA32010_DISASSEMBLY_SHOWID
        disasm = {"IKA32010_", `IKA32010_DEVICE_ID, ": "};
    `endif
    $sformat(num_data, " PC=0x%h |", {pc-1}[11:0]);
    disasm = {disasm, num_data};
    disasm = {disasm, " ", mnemonic};
    $sformat(num_data, " 0x%b", opcodereg[0]);
    disasm = {disasm, num_data};
    disasm = {disasm, "\n"};
    $display(disasm);
endfunction
`endif


//microcode
always @(*) begin
    //next bus transaction type
    busctrl_req = BUSCTRL_STOP; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;

    //next program counter operation
    if_pc_modesel = PC_RESET;
    
    //reset instruction cycle?
    ex_inst_cycle_rst = YES;
    
    //force next opcode nop?
    if_opcodereg_force_zero = YES; //flush!
    
    //ALU operation
    alu_modesel = ALU_ADD; alu_paz = NO; alu_pbz = NO; alu_pbdata = ALU_PBDATA_LONGWORD; alu_pbsel = ALU_SOURCE_SHFT;

    //load peripheral address?
    ex_addr_peripheral_ld = NO;

    //overflow mode, interrupt mode
    reg_ovm_set = NO; reg_ovm_rst = NO;
    reg_intm_en = NO; reg_intm_dis = NO;

    //AR register
    reg_arp_ld = NO;
    reg_ar_ld = NO;
    reg_ar_inc = NO; reg_ar_dec = NO;

    //T, P, ACC
    reg_t_ld = NO; reg_p_ld = NO;
    alu_acc_ld = NO;

    //RAM write
    ram_wr = NO;

    //stack
    stk_data_sel = STACK_DATA_PC;
    stk_pop = NO; stk_push = NO;

    //shifter enable
    sha_amt = 5'd0;
    shb_amt = 3'd0;
    shb_mux = LOW;

    //read source
    register_wrbus_source_sel = WRBUS_SOURCE_RAM;

    //interrupt acknowledge
    int_ack = 1'b0;

    //disassembly message
    `ifdef IKA32010_DISASSEMBLY
    num_data = "";
    `endif

    if(ex_state == 3'b000) begin
        //No change
    end

    else if(ex_state == 3'b001) begin
        //interrupt check
        if_opcodereg_force_zero = (int_rq) ? YES : NO;
        if_pc_modesel =           (int_rq) ? PC_LOAD_INTERRUPT : PC_INCREASE;
        stk_push =                (int_rq) ? YES : NO;
        stk_data_sel =            (int_rq) ? STACK_DATA_PC : STACK_DATA_ACC;

        casez(if_opcodereg)

            //
            //  CONTROL INSTRUCTIONS
            //

            //INTERNAL SPECIAL INSTRUCTION: IDK
            16'b0000_0000_0000_0000: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                if_pc_modesel = PC_INCREASE;

                //acknowledge interrupt
                if_opcodereg_force_zero = NO;
                int_ack = (int_rq) ? YES : NO;
                stk_push = NO; stk_data_sel = STACK_DATA_ACC;
                `ifdef IKA32010_DISASSEMBLY 
                    if(int_rq) $display("IKA32010_", `IKA32010_DEVICE_ID, ": IRQ RECEIVED\n");
                `endif
            end

            //NOP
            16'b0111_1111_1000_0000: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type0("NOP", if_pc);
                `endif
            end



            //
            //  ACCUMULATOR INSTRUCTIONS
            //

            //ABS - Absolute value of accumulator
            16'b0111_1111_1000_1000: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                alu_modesel = ALU_ABS; alu_pbz = YES; //disable port B
                alu_acc_ld = YES;

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type0("ABS", if_pc);
                `endif
            end

            //ADD - Add to accumulator with shift
            16'b0000_????_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                alu_modesel = ALU_ADD; //load from port B
                alu_acc_ld = YES;
                sha_amt = {1'b0, if_opcodereg[11:8]}; 
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type1("ADD", if_opcodereg, if_pc);
                `endif
            end

            //ADDH - Add to high-order accumulator bits
            16'b0110_0000_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                alu_modesel = ALU_ADD; alu_pbdata = ALU_PBDATA_HIGHWORD; //load from port B, low bits masked
                alu_acc_ld = YES;
                sha_amt = 5'd16;
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end
                
                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type2("ADDH", if_opcodereg, if_pc, 0);
                `endif
            end

            //ADDS - Add to accumulator with no sign extension
            16'b0110_0001_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                alu_modesel = ALU_ADD; alu_pbdata = ALU_PBDATA_LOWWORD; //load from port B
                alu_acc_ld = YES;
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type2("ADDS", if_opcodereg, if_pc, 0);
                `endif
            end

            //AND - AND with accumulator
            16'b0111_1001_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                alu_modesel = ALU_AND; alu_pbdata = ALU_PBDATA_LOWWORD; //load from port B
                alu_acc_ld = YES;
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type2("AND", if_opcodereg, if_pc, 0);
                `endif
            end

            //LAC - Load accumulator with shift
            16'b0111_1001_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                alu_modesel = ALU_ADD; alu_paz = YES; //block acc feedback
                alu_acc_ld = YES;
                sha_amt = {1'b0, if_opcodereg[11:8]};
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type1("LAC", if_opcodereg, if_pc);
                `endif
            end

            //LACK - Load accumulator immediate
            16'b0111_1110_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                alu_modesel = ALU_ADD; alu_paz = YES; alu_pbdata = ALU_PBDATA_BYTE; //block acc feedback
                alu_acc_ld = YES;
                register_wrbus_source_sel = WRBUS_SOURCE_INST;

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type3("LACK", if_opcodereg, if_pc, 0);
                `endif
            end

            //OR - OR with accumulator
            16'b0111_1010_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                alu_modesel = ALU_OR; alu_pbdata = ALU_PBDATA_LOWWORD; //load from port B
                alu_acc_ld = YES;
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type2("OR", if_opcodereg, if_pc, 0);
                `endif
            end

            //SACH - Store high-order accumulator bits with shift
            16'b0101_1???_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                register_wrbus_source_sel = WRBUS_SOURCE_SHB;
                ram_wr = YES;
                shb_amt = if_opcodereg[10:8]; shb_mux = HIGH;
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end
            end

            //SACL - Store low-order accumulator bits
            16'b0101_0???_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                register_wrbus_source_sel = WRBUS_SOURCE_SHB;
                ram_wr = YES;
                shb_mux = LOW;
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type2("SACL", if_opcodereg, if_pc, 0);
                `endif
            end

            //SUB - Subtract from accumulator with shift
            16'b0001_????_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                alu_modesel = ALU_SUB; //load from port B
                alu_acc_ld = YES;
                sha_amt = {1'b0, if_opcodereg[11:8]};
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type1("SUB", if_opcodereg, if_pc);
                `endif
            end

            //SUBC - Conditional subtract (for divide)
            16'b0110_0100_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC; //next instruction cannot use the ACC
                alu_modesel = ALU_SUBC;
                sha_amt = 5'd15;
                //ACC will be loaded next cycle
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type2("SUBC", if_opcodereg, if_pc, 0);
                `endif
            end

            //SUBH - Subtract from High-Order Accumulator
            16'b0110_0010_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                alu_modesel = ALU_SUB; alu_pbdata = ALU_PBDATA_HIGHWORD; //load from port B, low bits masked
                alu_acc_ld = YES;
                sha_amt = 5'd16;
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type2("SUBH", if_opcodereg, if_pc, 0);
                `endif
            end

            //SUBS - Subtract from Low Accumulator with Sign-Extension Suppressed
            16'b0110_0011_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                alu_modesel = ALU_SUB; alu_pbdata = ALU_PBDATA_LOWWORD; //load from port B
                alu_acc_ld = YES;
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type2("SUBS", if_opcodereg, if_pc, 0);
                `endif
            end

            //XOR - XOR with accumulator
            16'b0111_1000_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                alu_modesel = ALU_XOR; alu_pbdata = ALU_PBDATA_LOWWORD; //load from port B
                alu_acc_ld = YES;
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type2("XOR", if_opcodereg, if_pc, 0);
                `endif
            end

            //ZAC - Zero Accumulator
            16'b0111_1111_1000_1001: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                alu_modesel = ALU_ADD; alu_paz = YES; alu_pbz = YES; //load from port B
                alu_acc_ld = YES;

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type0("ZAC", if_pc);
                `endif
            end

            //ZALH - Zero Accumulator and Load High
            16'b0110_0101_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                alu_modesel = ALU_ADD; alu_paz = YES; alu_pbdata = ALU_PBDATA_HIGHWORD;
                alu_acc_ld = YES;
                sha_amt = 5'd16;
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type2("ZALH", if_opcodereg, if_pc, 0);
                `endif
            end

            //ZALS - Zero Accumulator and Load Low with Sign-Extension Suppressed
            16'b0110_0110_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                alu_modesel = ALU_ADD; alu_paz = YES; alu_pbdata = ALU_PBDATA_LOWWORD;
                alu_acc_ld = YES;
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type2("ZALS", if_opcodereg, if_pc, 0);
                `endif
            end



            //
            //  AUXILLARY REGISTER AND DATA POINTER INSTRUCTIONS
            //

            //LAR - Load Auxillary Register
            16'b0011_100?_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                reg_ar_ld = YES;
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type2("LAR", if_opcodereg, if_pc, 1);
                `endif
            end

            //LARK - Load Auxillary Register Immediate
            16'b0111_000?_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                register_wrbus_source_sel = WRBUS_SOURCE_INST;
                reg_ar_ld = YES;

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type3("LARK", if_opcodereg, if_pc, 1);
                `endif
            end

            //LARP - Load Auxillary Register Pointer Immediate
            16'b0110_1000_1000_000?: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                register_wrbus_source_sel = WRBUS_SOURCE_INST;
                reg_arp_ld = YES;

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type5("LARP", if_opcodereg, if_pc);
                `endif
            end

            //LDP - Load Data Memory Page Pointer
            16'b0110_1111_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                reg_dp_ld = YES;
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type2("LDP", if_opcodereg, if_pc, 0);
                `endif
            end

            //LDPK - Load Data Memory Page Pointer Immediate
            16'b0110_1110_0000_000?: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                register_wrbus_source_sel = WRBUS_SOURCE_INST;
                reg_dp_ld = YES;

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type5("LDPK", if_opcodereg, if_pc);
                `endif
            end

            //MAR - Modify auxiliary register and pointer
            16'b0110_1000_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type2("LDP", if_opcodereg, if_pc, 0);
                `endif
            end

            //SAR - Store auxiliary register
            16'b0011_000?_????_????: begin
                busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                register_wrbus_source_sel = WRBUS_SOURCE_AR;
                ram_wr = YES;
                if(if_opcodereg[7]) begin 
                    reg_ar_inc = if_opcodereg[5]; reg_ar_dec = if_opcodereg[4]; reg_arp_ld = ~if_opcodereg[3]; //AR register
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type2("SAR", if_opcodereg, if_pc, 1);
                `endif
            end




            //
            //  BRANCH INSTRUCTIONS
            //

            //B - Branch unconditionally
            16'b1111_1001_0000_0000: begin
                if(ex_inst_cycle == 2'd0) begin
                    busctrl_req = DATA_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                    if_pc_modesel = PC_LOAD_IMMEDIATE; //deny interrupt request
                    ex_inst_cycle_rst = NO;

                    //deny interrupt request
                    if_opcodereg_force_zero = NO; 
                    stk_push = NO; stk_data_sel = STACK_DATA_ACC;
                end
                else if(ex_inst_cycle == 2'd1) begin
                    busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                    ex_inst_cycle_rst = YES;
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type4("B", if_pc, ex_inst_cycle, busctrl_inlatch, 0);
                `endif
            end

            //BANZ - Branch on auxillary register not zero
            16'b1111_0100_0000_0000: begin
                if(ex_inst_cycle == 2'd0) begin
                    busctrl_req = DATA_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                    if_pc_modesel = (reg_ar[reg_arp] != 16'h0000) ? PC_LOAD_IMMEDIATE : PC_INCREASE; //deny interrupt request
                    ex_inst_cycle_rst = NO;

                    //deny interrupt request
                    if_opcodereg_force_zero = NO; 
                    stk_push = NO; stk_data_sel = STACK_DATA_ACC;
                end
                else if(ex_inst_cycle == 2'd1) begin
                    busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type4("BANZ", if_pc, ex_inst_cycle, busctrl_inlatch, 0);
                `endif
            end


            //BGEZ - Branch if Accumulator Greater Than or Equal to Zero 
            16'b1111_1101_0000_0000: begin
                if(ex_inst_cycle == 2'd0) begin
                    busctrl_req = DATA_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                    if_pc_modesel = (alu_flag_neg != 1'b1) ? PC_LOAD_IMMEDIATE : PC_INCREASE; //deny interrupt request
                    ex_inst_cycle_rst = NO;

                    //deny interrupt request
                    if_opcodereg_force_zero = NO; 
                    stk_push = NO; stk_data_sel = STACK_DATA_ACC;
                end
                else if(ex_inst_cycle == 2'd1) begin
                    busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type4("BGEZ", if_pc, ex_inst_cycle, busctrl_inlatch, 0);
                `endif
            end

            //BGZ - Branch if Accumulator Greater Than Zero
            16'b1111_1100_0000_0000: begin
                if(ex_inst_cycle == 2'd0) begin
                    busctrl_req = DATA_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                    if_pc_modesel = ((alu_flag_neg != 1'b1) && (alu_flag_zero != 1'b1)) ? PC_LOAD_IMMEDIATE : PC_INCREASE; //deny interrupt request
                    ex_inst_cycle_rst = NO;

                    //deny interrupt request
                    if_opcodereg_force_zero = NO; 
                    stk_push = NO; stk_data_sel = STACK_DATA_ACC;
                end
                else if(ex_inst_cycle == 2'd1) begin
                    busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type4("BGZ", if_pc, ex_inst_cycle, busctrl_inlatch, 0);
                `endif
            end

            
            //BIOZ - Branch on I/O Status Equal to Zero
            16'b1111_0110_0000_0000: begin
                if(ex_inst_cycle == 2'd0) begin
                    busctrl_req = DATA_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                    if_pc_modesel = (bio_n == 1'b0) ? PC_LOAD_IMMEDIATE : PC_INCREASE; //deny interrupt request
                    ex_inst_cycle_rst = NO;

                    //deny interrupt request
                    if_opcodereg_force_zero = NO; 
                    stk_push = NO; stk_data_sel = STACK_DATA_ACC;
                end
                else if(ex_inst_cycle == 2'd1) begin
                    busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type4("BIOZ", if_pc, ex_inst_cycle, busctrl_inlatch, 0);
                `endif
            end

            
            //BLEZ - Branch if Accumulator Less Than or Equal to Zero 
            16'b1111_1011_0000_0000: begin
                if(ex_inst_cycle == 2'd0) begin
                    busctrl_req = DATA_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                    if_pc_modesel = (alu_flag_neg == 1'b1 || alu_flag_zero == 1'b1) ? PC_LOAD_IMMEDIATE : PC_INCREASE; //deny interrupt request
                    ex_inst_cycle_rst = NO;

                    //deny interrupt request
                    if_opcodereg_force_zero = NO; 
                    stk_push = NO; stk_data_sel = STACK_DATA_ACC;
                end
                else if(ex_inst_cycle == 2'd1) begin
                    busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type4("BLEZ", if_pc, ex_inst_cycle, busctrl_inlatch, 0);
                `endif
            end

            //BLZ - Branch if Accumulator Less Than Zero 
            16'b1111_1010_0000_0000: begin
                if(ex_inst_cycle == 2'd0) begin
                    busctrl_req = DATA_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                    if_pc_modesel = (alu_flag_neg == 1'b1) ? PC_LOAD_IMMEDIATE : PC_INCREASE; //deny interrupt request
                    ex_inst_cycle_rst = NO;

                    //deny interrupt request
                    if_opcodereg_force_zero = NO; 
                    stk_push = NO; stk_data_sel = STACK_DATA_ACC;
                end
                else if(ex_inst_cycle == 2'd1) begin
                    busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type4("BLZ", if_pc, ex_inst_cycle, busctrl_inlatch, 0);
                `endif
            end

            //BNZ - Branch if Accumulator Not Equal to Zero
            16'b1111_1110_0000_0000: begin
                if(ex_inst_cycle == 2'd0) begin
                    busctrl_req = DATA_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                    if_pc_modesel = (alu_flag_zero != 1'b1) ? PC_LOAD_IMMEDIATE : PC_INCREASE; //deny interrupt request
                    ex_inst_cycle_rst = NO;

                    //deny interrupt request
                    if_opcodereg_force_zero = NO; 
                    stk_push = NO; stk_data_sel = STACK_DATA_ACC;
                end
                else if(ex_inst_cycle == 2'd1) begin
                    busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type4("BNZ", if_pc, ex_inst_cycle, busctrl_inlatch, 0);
                `endif
            end

            //BV - Branch on Overflow
            16'b1111_0101_0000_0000: begin
                if(ex_inst_cycle == 2'd0) begin
                    busctrl_req = DATA_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                    if_pc_modesel = (alu_flag_ovfl == 1'b1) ? PC_LOAD_IMMEDIATE : PC_INCREASE; //deny interrupt request
                    ex_inst_cycle_rst = NO;

                    //deny interrupt request
                    if_opcodereg_force_zero = NO; 
                    stk_push = NO; stk_data_sel = STACK_DATA_ACC;
                end
                else if(ex_inst_cycle == 2'd1) begin
                    busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type4("BV", if_pc, ex_inst_cycle, busctrl_inlatch, 0);
                `endif
            end

            //BZ - Branch if Accumulator Equals Zero
            16'b1111_1111_0000_0000: begin
                if(ex_inst_cycle == 2'd0) begin
                    busctrl_req = DATA_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                    if_pc_modesel = (alu_flag_zero == 1'b1) ? PC_LOAD_IMMEDIATE : PC_INCREASE; //deny interrupt request
                    ex_inst_cycle_rst = NO;

                    //deny interrupt request
                    if_opcodereg_force_zero = NO; 
                    stk_push = NO; stk_data_sel = STACK_DATA_ACC;
                end
                else if(ex_inst_cycle == 2'd1) begin
                    busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type4("BZ", if_pc, ex_inst_cycle, busctrl_inlatch, 0);
                `endif
            end

            //CALA - Call Subroutine Indirect(from Accumulator)
            16'b0111_1111_1000_1100: begin
                if(ex_inst_cycle == 2'd0) begin
                    busctrl_req = BUSCTRL_STOP; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                    if_pc_modesel = PC_LOAD_WRBUS; //deny interrupt request
                    register_wrbus_source_sel = WRBUS_SOURCE_SHB; shb_mux = LOW;//PC write source is SHB
                    ex_inst_cycle_rst = NO;

                    //deny interrupt request
                    if_opcodereg_force_zero = NO; 
                    stk_push = YES; stk_data_sel = STACK_DATA_PC;
                end
                else if(ex_inst_cycle == 2'd1) begin
                    busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type4("CALA", if_pc, ex_inst_cycle, alu_acc_output[15:0], 0);
                `endif
            end

            //CALL - Call Subroutine Direct
            16'b1111_1000_0000_0000: begin
                if(ex_inst_cycle == 2'd0) begin
                    busctrl_req = DATA_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                    if_pc_modesel = PC_LOAD_IMMEDIATE; //deny interrupt request
                    ex_inst_cycle_rst = NO;

                    //deny interrupt request
                    if_opcodereg_force_zero = NO; 
                    stk_push = YES; stk_data_sel = STACK_DATA_PC;
                end
                else if(ex_inst_cycle == 2'd1) begin
                    busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type4("CALL", if_pc, ex_inst_cycle, busctrl_inlatch, 0);
                `endif
            end
            
            //RET - Return from Subroutine
            16'b0111_1111_1000_1101: begin
                if(ex_inst_cycle == 2'd0) begin
                    busctrl_req = BUSCTRL_STOP; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                    if_pc_modesel = PC_LOAD_WRBUS; //deny interrupt request
                    register_wrbus_source_sel = WRBUS_SOURCE_STK;
                    ex_inst_cycle_rst = NO;

                    //deny interrupt request
                    if_opcodereg_force_zero = NO; 
                    stk_push = NO; stk_pop = YES; stk_data_sel = STACK_DATA_ACC;
                end
                else if(ex_inst_cycle == 2'd1) begin
                    busctrl_req = OPCODE_READ; busctrl_addr_muxsel = BUSCTRL_ADDR_PC;
                end

                `ifdef IKA32010_DISASSEMBLY 
                    disasm_type4("RET", if_pc, ex_inst_cycle, busctrl_inlatch, 1);
                `endif
            end



        endcase
    end


    else begin

    end
end



endmodule


module IKA32010_alu (
    input   wire            i_EMUCLK,
    input   wire            i_CEN_n,
    input   wire            i_RST_n,

    input   wire            i_ALU_OVM,
    input   wire    [3:0]   i_ALU_MODESEL,
    input   wire            i_ALU_PAZ, //force alu port A zero
    input   wire            i_ALU_PBZ, //force alu port B zero
    input   wire    [1:0]   i_ALU_PBDATA,

    input   wire    [31:0]  i_ALU_PA, i_ALU_PB,

    input   wire            i_ALU_ACC_LD,
    output  reg     [31:0]  o_ALU_ACC_OUTPUT,

    output  reg             o_Z, o_N, o_V //Zero, Negative, oVerflow
);

localparam  ALU_AND             = 3'd0;
localparam  ALU_OR              = 3'd1;
localparam  ALU_XOR             = 3'd2;
localparam  ALU_ABS             = 3'd3;
localparam  ALU_ADD             = 3'd4;
localparam  ALU_SUB             = 3'd5;
localparam  ALU_SUBC            = 3'd6;

localparam  ALU_PBDATA_LONGWORD = 2'd0;
localparam  ALU_PBDATA_HIGHWORD = 2'd1;
localparam  ALU_PBDATA_LOWWORD  = 2'd2;
localparam  ALU_PBDATA_BYTE     = 2'd3;  

//subc related
reg             prev_subc, subc_divided;
reg     [31:0]  prev_adder;

//PORT A/B input generation
reg     [31:0]  port_a, port_b;
reg             adder_cin;
always @(*) begin
    //adder's carry in
    adder_cin = 1'b0; //default carry
    if(subc_divided) adder_cin = 1'b1; //add carry if divided
    else begin
        if(i_ALU_MODESEL == ALU_SUB || i_ALU_MODESEL == ALU_SUBC) adder_cin = 1'b1; //make 2's complement
        else if(i_ALU_MODESEL == ALU_ABS) adder_cin = i_ALU_PA[31]; //make 2's complement if the ACC is negative, to take abs value
    end

    //port A(feedback from the accumulator)
    if(prev_subc) port_a = subc_divided ? 32'h0000_0000 : i_ALU_PA << 1;
    else begin
        if(i_ALU_PAZ) port_a = 32'h0000_0000;
        else begin
            if(i_ALU_MODESEL == ALU_ABS) port_a = i_ALU_PA[31] ? ~i_ALU_PA : i_ALU_PA; //for abs
            else port_a = i_ALU_PA;
        end
    end

    //port B
    if(prev_subc) port_b = subc_divided ? prev_adder << 1 : 32'h0000_0000;
    else begin
        if(i_ALU_PBZ) port_b = 32'h0000_0000;
        else begin
            if(i_ALU_MODESEL == ALU_SUB || i_ALU_MODESEL == ALU_SUBC) port_b = ~i_ALU_PB;
            else port_b = i_ALU_PB;

            case(i_ALU_PBDATA)
                ALU_PBDATA_LONGWORD: port_b = i_ALU_PB;
                ALU_PBDATA_HIGHWORD: port_b = {i_ALU_PB[31:16], 16'h0000}; //32bit arithmetic: with sign bit(32nd bit)
                ALU_PBDATA_LOWWORD : port_b = {16'h0000, i_ALU_PB[15:0]}; //32bit arithmetic: ignore sign bit
                ALU_PBDATA_BYTE    : port_b = {24'h0000_00, i_ALU_PB[7:0]};
            endcase
        end
    end
end

//ALU adder
wire    [31:0]  alu_adder31 = port_a[30:0] + port_b[30:0] + adder_cin;
wire    [1:0]   alu_adder1 = port_a[31] + port_b[31] + alu_adder31[31];
wire    [31:0]  alu_adder = {alu_adder1[0], alu_adder31[30:0]};
reg     [31:0]  alu_output;

//ALU saturation
wire            alu_ovfl = alu_adder31[31] ^ alu_adder1[1];

//ALU operation select
always @(*) begin
    if(prev_subc) alu_output = port_a + port_b + adder_cin;
    else begin   
        case(i_ALU_MODESEL)
            ALU_AND : alu_output = port_a & port_b;
            ALU_OR  : alu_output = port_a | port_b;
            ALU_XOR : alu_output = port_a ^ port_b;
            ALU_ABS : alu_output = alu_adder;
            ALU_ADD : alu_output = i_ALU_OVM ? alu_ovfl ? {~alu_adder31[31], {31{alu_adder31[31]}}} : alu_adder : alu_adder; //saturation
            ALU_SUB : alu_output = i_ALU_OVM ? alu_ovfl ? {~alu_adder31[31], {31{alu_adder31[31]}}} : alu_adder : alu_adder; //saturation
            ALU_SUBC: alu_output = i_ALU_OVM ? alu_ovfl ? {~alu_adder31[31], {31{alu_adder31[31]}}} : alu_adder : alu_adder; //saturation
            default: alu_output = 32'h0000_0000;
        endcase
    end
end

//SUBC control, EX unit takes 1 cycle to process SUBC, but ALU doesn't
always @(posedge i_EMUCLK) if(!i_CEN_n) begin
    if(i_ALU_MODESEL == ALU_SUBC && alu_output[31] == 1'b0) subc_divided <= 1'b1;
    else subc_divided <= 1'b0;

    prev_subc <= i_ALU_MODESEL == ALU_SUBC;
    prev_adder <= alu_output;
end

//accumulator control
wire            alu_acc_ld = prev_subc | i_ALU_ACC_LD;
always @(posedge i_EMUCLK) begin
    if(!i_RST_n) o_ALU_ACC_OUTPUT <= 32'h0000_0000;
    else begin
        if(!i_CEN_n) if(alu_acc_ld) o_ALU_ACC_OUTPUT <= alu_output;
    end
end

//flags
always @(posedge i_EMUCLK) begin
    if(!i_RST_n) begin
        o_Z <= 1'b1; o_N <= 1'b0; o_V <= 1'b0;
    end
    else begin
        if(!i_CEN_n) if(alu_acc_ld) begin
            o_Z <= alu_output == 32'h0000_0000;
            o_N <= alu_output[31];
            o_V <= alu_adder31[31] ^ alu_adder1[1];
        end
    end
end

endmodule


module IKA32010_ram (
    input   wire            i_EMUCLK,
    input   wire            i_CEN_n,

    input   wire            i_WE,
    input   wire    [7:0]   i_ADDR,
    input   wire    [15:0]  i_DIN,
    output  reg     [15:0]  o_DOUT
);

reg     [15:0]  RAM[0:255];

always @(posedge i_EMUCLK) if(!i_CEN_n) begin
    if(i_WE) RAM[i_ADDR] <= i_DIN;
    else o_DOUT <= RAM[i_ADDR];
end

//initialize 
/*
integer i;
initial begin
    for(i=0; i<255; i=i+1) begin
        RAM[i] <= 16'h0000;
    end
end
*/
endmodule


module IKA32010_stack (
    input   wire            i_EMUCLK,
    input   wire            i_CEN_n,
    input   wire            i_RST_n,

    input   wire            i_PUSH,
    input   wire            i_POP,

    input   wire    [11:0]  i_DIN,
    output  wire    [11:0]  o_DOUT
);

reg     [11:0]  stack[0:3];
assign  o_DOUT = stack[0];

always @(posedge i_EMUCLK) begin
    if(!i_RST_n) begin
        stack[0] <= 12'h000;
        stack[1] <= 12'h000;
        stack[2] <= 12'h000;
        stack[3] <= 12'h000;
    end
    else begin
        if(!i_CEN_n) begin
            case({i_PUSH, i_POP})
                2'b10: begin
                    stack[0] <= i_DIN;    stack[1] <= stack[0]; stack[2] <= stack[1]; stack[3] <= stack[2]; //push
                end
                2'b01: begin
                    stack[0] <= stack[1]; stack[1] <= stack[2]; stack[2] <= stack[3]; stack[3] <= stack[3]; //pop
                end
                default: begin
                    stack[0] <= stack[0]; stack[1] <= stack[1]; stack[2] <= stack[2]; stack[3] <= stack[3]; //hold
                end
            endcase
        end
    end
end

endmodule