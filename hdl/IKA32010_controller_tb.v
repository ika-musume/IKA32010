`timescale 10ps/10ps
module IKA32010_tb;

reg             EMUCLK = 1'b1;
reg             RS_n = 1'b1;
reg             INT_n = 1'b1;

always #1 EMUCLK = ~EMUCLK;

reg     [1:0]   divider = 2'd0;
always @(posedge EMUCLK) divider <= divider + 2'd1;
wire            cen_n = ~(divider == 2'd3);
wire            refclk = ~divider[1];

initial begin
    #30 RS_n <= 1'b0;
    #200 RS_n <= 1'b1;

    //#320 INT_n <= 1'b0;
    //#63 INT_n <= 1'b1;
end

wire            MEN_n, DEN_n, WE_n;
wire    [11:0]  ADDR;
reg     [15:0]  DIN;

IKA32010_controller main (
    .i_EMUCLK               (EMUCLK                     ),
    .i_CLKIN_PCEN_n         (cen_n                      ),

    .o_CLKOUT               (                           ),
    .o_CLKOUT_PCEN_n        (                           ),
    .o_CLKOUT_NCEN_n        (                           ),

    .i_RS_n                 (RS_n                       ),

    .o_MEN_n                (MEN_n                      ),
    .o_DEN_n                (DEN_n                      ),
    .o_WE_n                 (WE_n                       ),

    .o_AOUT                 (ADDR                       ),
    .i_DIN                  (DIN                        ),
    .o_DOUT                 (                           ),
    .o_DOUT_OE_n            (                           ),

    .i_BIO_n                (1'b0                       ),
    .i_INT_n                (INT_n                      )
);

always @(*) begin
    case(ADDR)
        12'h000: DIN = (!MEN_n) ? 16'h7F89 : 16'hZZZZ; //ZAC
        12'h001: begin DIN = (!MEN_n) ? 16'h7EE0 : 16'hZZZZ; DIN = (!DEN_n) ? 16'h4E71 : 16'hZZZZ; end//LACK 0xE0
        12'h002: DIN = (!MEN_n) ? 16'h7F9C : 16'hZZZZ; //PUSH
        12'h003: DIN = (!MEN_n) ? 16'h7F89 : 16'hZZZZ; //ZAC
        12'h004: DIN = (!MEN_n) ? 16'h7F9D : 16'hZZZZ; //POP
        12'h005: DIN = (!MEN_n) ? 16'h7F8B : 16'hZZZZ; //SOVM
        12'h006: DIN = (!MEN_n) ? 16'h6E01 : 16'hZZZZ; //LDP 1
        12'h007: DIN = (!MEN_n) ? 16'h7C00 : 16'hZZZZ; //SSR DAT0
        12'h008: DIN = (!MEN_n) ? 16'h7F8A : 16'hZZZZ; //ROVM
        12'h009: DIN = (!MEN_n) ? 16'h7B00 : 16'hZZZZ; //LST DAT0
        12'h00A: DIN = (!MEN_n) ? 16'h70FF : 16'hZZZZ; //LARP AR0, FF
        12'h00B: DIN = (!MEN_n) ? 16'h71EE : 16'hZZZZ; //LARP AR1, EE

        12'h00C: DIN = (!MEN_n) ? 16'h68A1 : 16'hZZZZ; //MAR *+, 1

        12'h00D: DIN = (!MEN_n) ? 16'h4101 : 16'hZZZZ; //IN
        12'h00E: DIN = (!MEN_n) ? 16'h4800 : 16'hZZZZ; //OUT

        12'h00F: DIN = (!MEN_n) ? 16'h7E06 : 16'hZZZZ; //LACK 0x06
        12'h010: DIN = (!MEN_n) ? 16'h6702 : 16'hZZZZ; //TBLR DAT2

        12'h011: DIN = (!MEN_n) ? 16'h7E42 : 16'hZZZZ; //LACK 0x42
        12'h012: DIN = (!MEN_n) ? 16'h7D02 : 16'hZZZZ; //TBLW DAT2
        
        12'h013: DIN = (!MEN_n) ? 16'hF900 : 16'hZZZZ; //B
        12'h014: DIN = (!MEN_n) ? 16'h0000 : 16'hZZZZ; //0000

        default: DIN = (!MEN_n) ? 16'h7F80 : 16'hZZZZ;
    endcase
end

endmodule