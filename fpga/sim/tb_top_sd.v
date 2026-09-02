// tb_top_sd: the whole design, with a model SD card and NO host.
//
// With nothing on the DB-25, nothing drives the ProFile bus. What this checks
// is that the FPGA brings the card up and reads a sector, with the eight LEDs
// as the only output channel. It is the first thing to run on new hardware.
//
// Expected LED result, which is what to look for on the hardware:
//
//     led[7] = 1   card initialized
//     led[6] = 1   self-test finished
//     led[5] = 1   sector 0 read and carried $55 $AA at 510/511
//     led[4:0]= 0  no error code
//
// i.e. all of 7, 6, 5 lit and the low five dark: $E0.
//
//   iverilog -g2012 -o /tmp/tb -s tb_top_sd sim/tb_top_sd.v sim/sd_model.v src/*.v

`timescale 1ns / 1ps
`default_nettype none

module tb_top_sd;

   reg clk = 0, rst_n = 0;
   always #5 clk = ~clk;

   // No host: the bus lines sit at their idle levels, which is what the pull
   // resistors on the board do with no cable attached.
   wire [7:0] pd;
   reg  pstrb_n = 1, pcmd_n = 1, prw = 1, pres_n = 1, ph0 = 0, aux7 = 0;
   reg  sd_cd_n = 0, sd_wp_n = 1;

   wire xcvr_dir, xcvr_oe_n, pparity, pbsy_n, pchk;
   wire sd_clk, sd_cs_n, sd_mosi, sd_miso;
   wire [7:0] led;
   wire usb_tx, usb_tx_alt;

   // SD_DIV_WRITE is shortened here for the same reason as SD_DIV_INIT: the
   // real value is the slow one the card needs for writes, and
   // simulating 512 bytes at that divider takes longer than the timeout.
   top #(.FAMILY("ECP5"),
         .SD_DIV_INIT(6), .SD_DIV_FAST(2), .SD_DIV_WRITE(6), .USE_PLL(1'b0),
         // This bench IS the self-test's test, so it turns the scan back on.
         // The shipping build compiles it out: fat32 and the verifier cover
         // it.
         .SELFTEST_FAT(1'b1),
         // AUTOPICK on: this bench checks that an image is RESOLVED and mapped.
         // The shipping default refuses to choose one without a configured
         // selection, which sim-fat32-noauto covers separately.
         .FS_AUTOPICK(1'b1)) dut (
                                  .clk(clk), .rst_n(rst_n),
                                  .pd(pd), .xcvr_dir(xcvr_dir), .xcvr_oe_n(xcvr_oe_n), .pparity(pparity),
                                  .pstrb_n(pstrb_n), .pcmd_n(pcmd_n), .prw(prw),
                                  .pbsy_n(pbsy_n), .pres_n(pres_n), .pchk(pchk),
                                  .ph0(ph0), .aux7(aux7),
                                  .sd_clk(sd_clk), .sd_cs_n(sd_cs_n), .sd_mosi(sd_mosi), .sd_miso(sd_miso),
                                  .sd_cd_n(sd_cd_n), .sd_wp_n(sd_wp_n),
                                  .led(led), .usb_tx(usb_tx), .usb_tx_alt(usb_tx_alt));

   sd_model #(.FS_IMAGE(1'b1)) card (.sd_clk(sd_clk), .sd_cs_n(sd_cs_n),
                                     .sd_mosi(sd_mosi), .sd_miso(sd_miso));

   integer errors = 0;
   task check(input cond, input [255:0] what);
      begin
         if (cond) $display("    ok    %0s", what);
         else begin $display("*** FAIL  %0s", what); errors = errors + 1; end
      end
   endtask

   initial begin
      repeat (5) @(posedge clk);
      rst_n = 1;

      // The bus must stay released for the whole run: with no host attached,
      // anything that turns the '245 around is pointing it at an open
      // connector.
      fork
         forever @(posedge clk) if (rst_n && xcvr_dir !== 1'b0) begin
            $display("*** the bus was driven with no host attached"); errors = errors + 1;
         end
      join_none

      wait (dut.selftest_done === 1'b1);
      repeat (20) @(posedge clk);

      $display("-- FAT scan");
      $display("    part=%0d rsvd=%0d spc=%0d nfat=%0d fatsz=%0d",
               dut.part_lba, dut.bpb_rsvd, dut.bpb_spc, dut.bpb_nfat, dut.bpb_fatsz);
      $display("    fat=%0d data=%0d free=%0d first=%0d target=%0d",
               dut.fat_start, dut.data_start, dut.free_count,
               dut.first_free, dut.target_lba);
      check(dut.part_lba   === 32'd64, "partition LBA from the MBR");
      check(dut.bpb_spc    === 8'd1,   "sectors per cluster from the BPB");
      check(dut.fat_start  === 32'd66, "FAT start computed");
      check(dut.data_start === 32'd67, "data start computed");
      // Clusters 2 to 8 are in use: root, /HD, the four image clusters and
      // profile.cfg, so the first free one is 9.
      check(dut.first_free === 32'd9,  "first free cluster is 9, past the config");
      check(dut.target_lba === 32'd74, "target LBA for that cluster");
      check(dut.wtest_ok   === 1'b1,   "write, read back and compare");

      // ---- the filesystem layer ----
      wait (dut.fs_ready === 1'b1);
      repeat (4) @(posedge clk);
      $display("-- filesystem resolver");
      $display("    ok=%0b err=%0d image_blocks=%0d clusters=%0d data_start=%0d",
               dut.fs_ok, dut.fs_err, dut.fs_image_blocks, dut.fs_nclusters,
               dut.fs_data_start);
      check(dut.fs_ok === 1'b1, "an image file was found in /HD");
      check(dut.fs_image_blocks === 32'd4, "its size came from the directory entry");
      check(dut.fs_nclusters === 32'd4, "and its chain is four clusters long");
      // The image's blocks must reach the SD as the CHAIN says (4,6,5,7), not
      // as a contiguous guess would (4,5,6,7). Geometry sees the real size too.
      check(dut.claim_blocks === 16'd4, "geometry claims the file's real size");

      $display("-- self-test finished, LEDs = %02h  (%08b)", led, led);
      check(led[7] === 1'b1, "led7: card initialized");
      check(led[6] === 1'b1, "led6: self-test finished");
      check(led[5] === 1'b1, "led5: sector 0 carried $55 $AA");
      check(led[4:0] === 5'd0, "led4:0: no error code");
      check(led === 8'hE0, "LED pattern is $E0");
      check(pbsy_n === 1'b1, "BSY never asserted with no host");

      $display("");
      if (errors == 0) $display("PASS: card up, sector 0 read and verified, bus untouched");
      else             $display("FAIL: %0d problem(s)", errors);
      $finish;
   end

   initial begin
      #40_000_000;
      $display("*** TIMEOUT: sd state %0d, err_code %0d, ready %0b, LEDs %08b",
               dut.sd_dbg, dut.sd_err_code, dut.sd_ready, led);
      $finish;
   end

endmodule

`default_nettype wire
