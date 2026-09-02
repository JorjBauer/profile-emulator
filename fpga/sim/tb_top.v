// tb_top: first simulation of the ProFile FPGA target.
//
// Two things are being checked here, in this order of importance:
//
//   1. BUS CONTENTION SAFETY. The data '245's DIR pin is driven by xcvr_dir and
//      its /OE is tied low, so the buffer is always enabled and the FPGA decides
//      which way it points. If xcvr_dir is ever high while the host is driving
//      the DB-25 side, the '245 drives A->B into the host and two outputs fight
//      on a 5V net. That is the only failure mode that can damage the board,
//      the interface card, or both, so it is checked continuously rather than
//      at the end.
//
//   2. That a get-status read of $FFFFFF actually completes: the three phases,
//      the six command bytes, the four status bytes, the synthesized spare
//      table, and the //e's abandon-by-/PCMD ending.
//
// The host model is the FORGIVING one deliberately: strobe timings here are the
// reader's (~16 us/byte, ../../profile-dumper.ino), not the //e's 355 ns. A
// design that fails against this model cannot possibly work against the //e.
//
//   iverilog -g2012 -o /tmp/tb -s tb_top sim/tb_top.v src/*.v && /tmp/tb

`timescale 1ns / 1ps
`default_nettype none

module tb_top;

   localparam integer TCK = 10;              // 100 MHz
   localparam integer PCMD_HOLD = 4000;      // > the 3 us debounce in profile_target

   reg clk = 0, rst_n = 0;
   always #(TCK/2) clk = ~clk;

   // --- host-driven lines ---
   reg pstrb_n = 1, pcmd_n = 1, prw = 1, pres_n = 1;
   reg ph0 = 0, aux7 = 0, sd_miso = 1, sd_cd_n = 1, sd_wp_n = 1;

   // --- the data bus, with the host as the other driver ---
   wire [7:0] pd;
   reg  [7:0] host_drv = 8'hFF;
   reg        host_oe  = 0;
   assign pd = host_oe ? host_drv : 8'bz;

   wire xcvr_dir, xcvr_oe_n, pparity, pbsy_n, pchk;
   wire sd_clk, sd_cs_n, sd_mosi;
   wire [7:0] led;
   wire usb_tx, usb_tx_alt;

   top #(.FAMILY("ECP5"), .USE_PLL(1'b0)) dut (
                                               .clk(clk), .rst_n(rst_n),
                                               .pd(pd), .xcvr_dir(xcvr_dir), .xcvr_oe_n(xcvr_oe_n), .pparity(pparity),
                                               .pstrb_n(pstrb_n), .pcmd_n(pcmd_n), .prw(prw),
                                               .pbsy_n(pbsy_n), .pres_n(pres_n), .pchk(pchk),
                                               .ph0(ph0), .aux7(aux7),
                                               .sd_clk(sd_clk), .sd_cs_n(sd_cs_n), .sd_mosi(sd_mosi), .sd_miso(sd_miso),
                                               .sd_cd_n(sd_cd_n), .sd_wp_n(sd_wp_n),
                                               .led(led), .usb_tx(usb_tx), .usb_rx(1'b1), .usb_tx_alt(usb_tx_alt));

   integer errors = 0;

   // ---- 1. contention monitor, live for the whole run ----------------------
   always @(posedge clk) begin
      if (rst_n && host_oe && xcvr_dir) begin
         $display("*** CONTENTION at %0t: host is driving PD and xcvr_dir is high", $time);
         errors = errors + 1;
      end
      // /OE is tied low in the design; if that ever changes, the check above is
      // no longer sufficient on its own, so notice.
      if (rst_n && xcvr_oe_n !== 1'b0) begin
         $display("*** xcvr_oe_n is not low at %0t; revisit the contention check", $time);
         errors = errors + 1;
      end
   end

   // ---- host model ---------------------------------------------------------
   task host_write_byte(input [7:0] b);
      begin
         prw = 1'b0;                 // host writes
         @(posedge clk);
         host_drv = b; host_oe = 1'b1;
         #1000;                      // setup, deliberately generous
         pstrb_n = 1'b0;
         #4000;                      // the reader's strobe is microseconds wide
         pstrb_n = 1'b1;
         #1000;
         host_oe = 1'b0;
         #1000;
      end
   endtask

   task host_read_byte(output [7:0] b);
      begin
         prw = 1'b1;                 // host reads, drive drives
         host_oe = 1'b0;
         #1000;
         pstrb_n = 1'b0;
         #2000;
         b = pd;                     // sample while the strobe is low
         #2000;
         pstrb_n = 1'b1;
         #1000;
      end
   endtask

   task host_cmd(input v);         // 1 = assert /PCMD, held past the debounce
      begin
         pcmd_n = ~v;
         #PCMD_HOLD;
      end
   endtask

   // The ack is NOT strobed. The reader parks the byte on the bus, THEN raises
   // /PCMD, and the drive is expected to have read its port in between
   // (profile-dumper.ino steps [5] and [6]; the Z8 does `ld R0,P1` in
   // SetResponse). Modeling it as a strobed byte is wrong twice over: the
   // drive never sees the ack, and the extra strobe is eaten as a command byte.
   task host_ack(input [7:0] b);
      begin
         prw = 1'b0;                 // turnaround: this is what releases the drive
         #2000;
         host_drv = b; host_oe = 1'b1;
         #4000;                      // the drive samples somewhere in here
         pcmd_n = 1'b1;              // [6] raise /PCMD: handshake over
         #PCMD_HOLD;
         host_oe = 1'b0;
         #1000;
      end
   endtask

   task expect_byte(input [7:0] got, input [7:0] want, input [127:0] what);
      begin
         if (got !== want) begin
            $display("*** %0s: got $%02h, expected $%02h", what, got, want);
            errors = errors + 1;
         end else begin
            $display("    %0s = $%02h  ok", what, got);
         end
      end
   endtask

   // ---- 2. the transaction -------------------------------------------------
   reg [7:0] b;
   integer i;
   reg [7:0] table_bytes [0:15];

   initial begin
      $dumpfile("/tmp/tb_top.vcd");
      $dumpvars(0, tb_top);

      repeat (10) @(posedge clk);
      rst_n = 1;
      repeat (10) @(posedge clk);

      if (pbsy_n !== 1'b1) begin
         $display("*** BSY is asserted at idle"); errors = errors + 1;
      end
      if (xcvr_dir !== 1'b0) begin
         $display("*** the bus points at the host at idle"); errors = errors + 1;
      end

      $display("-- command phase");
      host_cmd(1);
      host_read_byte(b);
      expect_byte(b, 8'h01, "phase-1 response");
      if (pbsy_n !== 1'b0) begin
         $display("*** BSY did not drop for the response"); errors = errors + 1;
      end
      host_ack(8'h55);

      $display("-- command frame: read block $FFFFFF");
      host_write_byte(8'h00);              // opcode: read
      host_write_byte(8'hFF);              // block hi
      host_write_byte(8'hFF);              // block mid
      host_write_byte(8'hFF);              // block lo
      host_write_byte(8'h64);              // retry count
      host_write_byte(8'h14);              // sparing threshold

      $display("-- confirmation phase");
      host_cmd(1);
      host_read_byte(b);
      expect_byte(b, 8'h02, "phase-2 response");
      host_ack(8'h55);

      $display("-- data phase: 4 status bytes");
      for (i = 0; i < 4; i = i + 1) begin
         host_read_byte(b);
         $display("    status%0d = $%02h", i, b);
      end

      $display("-- data phase: first 16 bytes of the spare table");
      for (i = 0; i < 16; i = i + 1) begin
         host_read_byte(b);
         table_bytes[i] = b;
      end
      $write("    ");
      for (i = 0; i < 16; i = i + 1) $write("%02h ", table_bytes[i]);
      $display("");
      $write("    ascii: ");
      for (i = 0; i < 13; i = i + 1)
        $write("%0s", (table_bytes[i] >= 8'h20 && table_bytes[i] < 8'h7F)
               ? table_bytes[i] : "?");
      $display("");

      $display("-- host abandons by re-asserting /PCMD");
      host_cmd(1);
      host_cmd(0);
      repeat (100) @(posedge clk);

      if (pbsy_n !== 1'b1) begin
         $display("*** BSY still asserted after the abandon"); errors = errors + 1;
      end
      if (xcvr_dir !== 1'b0) begin
         $display("*** bus not released after the abandon"); errors = errors + 1;
      end

      $display("");
      if (errors == 0) $display("PASS: no contention, transaction completed");
      else             $display("FAIL: %0d problem(s)", errors);
      $finish;
   end

   initial begin
      #5_000_000;
      $display("*** TIMEOUT");
      $finish;
   end

endmodule

`default_nettype wire
