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
    #100 RS_n <= 1'b1;

    #200 RS_n <= 1'b0;
    #120 RS_n <= 1'b1;

    #420 INT_n <= 1'b0;
    #63 INT_n <= 1'b1;
end

wire            MEN_n, DEN_n, WE_n;
wire    [11:0]  ADDR;
wire    [15:0]  DBUS;

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
    .i_DIN                  (DBUS                        ),
    .o_DOUT                 (                           ),
    .o_DOUT_OE_n            (                           ),

    .i_BIO_n                (1'b1                       ),
    .i_INT_n                (INT_n                      )
);





reg     [15:0]  in = 16'h0000;
assign  DBUS = (DEN_n) ? 16'hZZZZ : in;





reg     [7:0]   dsp_hi[0:2047];
reg     [7:0]   dsp_lo[0:2047];
assign  DBUS = (MEN_n) ? 16'hZZZZ : {dsp_hi[ADDR], dsp_lo[ADDR]};
initial begin
    $readmemh("D:/PROCESSOR/IKA32010/IKA32010/rom/dsp_hi.txt", dsp_hi);
    $readmemh("D:/PROCESSOR/IKA32010/IKA32010/rom/dsp_lo.txt", dsp_lo);
end








endmodule