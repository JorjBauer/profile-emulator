// top_r2: the r2 board (hw-r2/) wrapped around the same top as the ULX3S.
// r2 added pins that the ULX3S does not have, and this is where they're
// handled.
//
//   u3_oe_n      U3's /OE (U1 pin 76, R53 pulls it high). U3 buffers the
//                host's control lines. It stays disabled until the FPGA is
//                configured, because U1 pins 97/99 and 102/104 are A/B
//                pairs that must not be driven apart before POR (Lattice
//                FPGA-DS-02012 3.6). Once we are running, it is on for good.
//
//   xcvr_oe_n    U2's /OE. PD0 to PD7 have no pull resistors, so with no host
//                on the cable U2's B side floats. Hold it disabled until the
//                host has asserted /PCMD once, and after that do whatever the
//                core asks (which today is "always on").
//
//   fpga_reboot  U1 pin 67, to PROGRAMN through R48. Pulled LOW reboots the
//                part, and it must never be driven HIGH. top drives it
//                through an explicit pad, open drain, when an SD firmware
//                update has finished. Don't replace that with an output
//                assigned 1'bz: nextpnr turns that into a driven pin, and a
//                driven low here reboots us over and over.
//
//   flash_*      The configuration flash, which the SD update rewrites. Its
//                clock is reached through USRMCLK inside top.
//
// Every port here is constrained in ../constraints/profile-drive-r2.lpf.

`default_nettype none

module top_r2 (
               input  wire        clk,
               input  wire        rst_n,

               inout  wire [7:0]  pd,
               output wire        xcvr_dir,
               output wire        xcvr_oe_n,
               output wire        pparity,
               input  wire        pstrb_n,
               input  wire        pcmd_n,
               input  wire        prw,
               output wire        pbsy_n,
               input  wire        pres_n,
               output wire        pchk,
               input  wire        ph0,
               input  wire        aux7,

               output wire        sd_clk,
               output wire        sd_cs_n,
               output wire        sd_mosi,
               input  wire        sd_miso,
               input  wire        sd_cd_n,
               input  wire        sd_wp_n,

               output wire [7:0]  led,
               output wire        usb_tx,
               input  wire        usb_rx,
               output wire        usb_tx_alt,

               output wire        u3_oe_n,
               inout  wire        fpga_reboot,  // see above
               output wire        flash_csn,
               output wire        flash_mosi,
               input  wire        flash_miso
               );

   wire core_oe_n;

   // /PCMD comes in through U3, which R49 to R52 hold high with no host, so
   // the first low we see is a host talking to us.
   reg [1:0] cmd_s     = 2'b11;
   reg       host_seen = 1'b0;
   always @(posedge clk) begin
      cmd_s <= {cmd_s[0], pcmd_n};
      if (!cmd_s[1]) host_seen <= 1'b1;
   end

   assign u3_oe_n     = 1'b0;
   assign xcvr_oe_n   = core_oe_n | ~host_seen;

   top core (.clk(clk), .rst_n(rst_n),
             .pd(pd), .xcvr_dir(xcvr_dir), .xcvr_oe_n(core_oe_n),
             .pparity(pparity), .pstrb_n(pstrb_n), .pcmd_n(pcmd_n),
             .prw(prw), .pbsy_n(pbsy_n), .pres_n(pres_n), .pchk(pchk),
             .ph0(ph0), .aux7(aux7),
             .sd_clk(sd_clk), .sd_cs_n(sd_cs_n), .sd_mosi(sd_mosi),
             .sd_miso(sd_miso), .sd_cd_n(sd_cd_n), .sd_wp_n(sd_wp_n),
             .led(led), .usb_tx(usb_tx), .usb_rx(usb_rx),
             .usb_tx_alt(usb_tx_alt),
             .flash_csn(flash_csn), .flash_mosi(flash_mosi),
             .flash_miso(flash_miso), .fpga_reboot(fpga_reboot));

endmodule
