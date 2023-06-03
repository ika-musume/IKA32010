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

    #320 INT_n <= 1'b0;
    #63 INT_n <= 1'b1;
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
    .i_INT_n                (INT_n                      ),
    .i_DATABUS_READ         (                           ),
    .o_DATABUS_WRITE        (                           )
);

always @(*) begin
    case(ADDR)
        12'h000: DIN <= (!MEN_n) ? 16'h7F80 : 16'hZZZZ;
        12'h001: DIN <= (!MEN_n) ? 16'h7F80 : 16'hZZZZ;
        12'h002: DIN <= (!MEN_n) ? 16'h7EA5 : 16'hZZZZ;
        12'h003: DIN <= (!MEN_n) ? 16'h7F88 : 16'hZZZZ;

        12'h006: DIN <= (!MEN_n) ? 16'hF900 : 16'hZZZZ;
        12'h007: DIN <= (!MEN_n) ? 16'h00A0 : 16'hZZZZ;
        12'h008: DIN <= (!MEN_n) ? 16'h7F80 : 16'hZZZZ; //NOP
        12'h009: DIN <= (!MEN_n) ? 16'h7F80 : 16'hZZZZ; //NOP
        12'h00A: DIN <= (!MEN_n) ? 16'h7F89 : 16'hZZZZ; //ZAC
        12'h00B: DIN <= (!MEN_n) ? 16'h7EE0 : 16'hZZZZ; //LACK 0xE0
        12'h00C: DIN <= (!MEN_n) ? 16'h7F8C : 16'hZZZZ; //CALA
        12'h00D: DIN <= (!MEN_n) ? 16'h7F8D : 16'hZZZZ; //RET

        12'h0A0: DIN <= (!MEN_n) ? 16'h7EE7 : 16'hZZZZ; //ABS
        12'h0A1: DIN <= (!MEN_n) ? 16'hF400 : 16'hZZZZ; //BANZ 0x0008
        12'h0A2: DIN <= (!MEN_n) ? 16'h0008 : 16'hZZZZ;
        12'h0A3: DIN <= (!MEN_n) ? 16'hF900 : 16'hZZZZ; //Z 0x0008
        12'h0A4: DIN <= (!MEN_n) ? 16'h0008 : 16'hZZZZ;

        12'h0E0: DIN <= (!MEN_n) ? 16'h7F80 : 16'hZZZZ; //NOP
        12'h0E1: DIN <= (!MEN_n) ? 16'h7F8D : 16'hZZZZ; //RET
        default: DIN <= (!MEN_n) ? 16'h7F80 : 16'hZZZZ;
    endcase
end

endmodule