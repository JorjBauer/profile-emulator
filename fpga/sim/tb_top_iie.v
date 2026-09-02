// tb_top_iie: the design against a model of the APPLE //e, not the reader.
//
// sim/tb_top.v already runs the protocol against the reader's host sequence,
// and that model is deliberately FORGIVING: microseconds of setup, a settling
// delay after PR/-W rises, generous strobes. A drive that fails there cannot
// possibly work. The converse is not true, and ../../PROTOCOL.md section 3 says
// so in as many words: "the Apple /// / Lisa host is forgiving here, which is
// why a slow drive works with the reader but the same drive can still fail
// against the //e. Test against the //e timing, not just the reader loopback."
//
// This is that test. Three things are modeled that tb_top.v does not:
//
//   1. NO SETTLING DELAY. The card raises PR/-W and clocks the first byte
//      immediately. Byte 0 must ALREADY be on the bus when PR/-W rises. A drive
//      that turns the bus around after seeing PR/-W loses byte 0, the card
//      reads a floating bus, and the boot fails in a way that looks like bad
//      data rather than bad timing.
//   2. A 355 ns STROBE. That is the //e's shortest window (README), against
//      16 us in the reader model: a factor of 45.
//   3. THE ABANDON. The card takes 4 status + 512 data and then re-asserts
//      /PCMD. It never takes the 20 tag bytes. A drive that insists on sending
//      532 stalls at byte 512.
//
// And on the write side, the 6-byte trailer of section 9.3, including the check
// that /PPARITY is generated across it. The drive is responsible for parity on
// bytes the HOST drives (section 9.4). Getting that wrong shows up on real
// hardware as a parity error that sets "by itself" and a ProDOS $27 on a block
// that was in fact written correctly.
//
//   iverilog -g2012 -o /tmp/tb -s tb_top_iie sim/tb_top_iie.v sim/sd_model.v src/*.v

`timescale 1ns / 1ps
`default_nettype none

module tb_top_iie #(
                    // The //e's shortest strobe window, and its byte pacing.
                    // Both are overridable so the margin can be squeezed
                    // deliberately (see sim-iie-fast in the Makefile) rather
                    // than only ever tested at the nominal figure.
                    parameter integer STROBE_NS = 355,
                    // Drive the byte this many ns AFTER /PSTRB falls instead of
                    // 100 ns before it. 0 is the well-behaved host. Non-zero
                    // reproduces what the //e's card actually does on the FIRST
                    // command byte: the 374's output enable (Q3, $C503) and the
                    // write that strobes it land together, so the bus is still
                    // floating when the strobe arrives.
                    parameter integer LATE_NS = 0,
                    parameter integer BYTE_NS   = 9000,
                    // Overridable so the decoupled-geometry band can be
                    // exercised through the host's own path, not just in the
                    // geometry unit test.
                    parameter integer IMG_BLOCKS = 32'h0000FE00,
                    parameter         SERVE_FULL = 1'b1
                    );

   localparam integer TCK = 10;              // 100 MHz oscillator
   localparam integer PCMD_HOLD = 4000;      // past the 3 us debounce

   reg clk = 0, rst_n = 0;
   always #(TCK/2) clk = ~clk;

   reg pstrb_n = 1, pcmd_n = 1, prw = 1, pres_n = 1;
   reg ph0 = 0, aux7 = 0, sd_cd_n = 0, sd_wp_n = 1;

   wire [7:0] pd;
   // The real bus idles HIGH: RPA pulls it up on the interface board, and a
   // byte sampled before anything drives reads $FF, not x. Without this the
   // bench cannot reproduce the hardware fault at all.
   pullup pd_pu [7:0] (pd);
   reg  [7:0] host_drv = 8'hFF;
   reg        host_oe  = 0;
   assign pd = host_oe ? host_drv : 8'bz;

   wire xcvr_dir, xcvr_oe_n, pparity, pbsy_n, pchk;
   wire sd_clk, sd_cs_n, sd_mosi, sd_miso;
   wire [7:0] led;
   wire usb_tx, usb_tx_alt;

   top #(.FAMILY("ECP5"),
         .SD_DIV_INIT(6), .SD_DIV_FAST(2), .SD_DIV_WRITE(6), .USE_PLL(1'b0),
         // Raw-image card: this bench is about the ProFile protocol and uses
         // block 77, while the model card's image file is only four blocks
         // long. Testing the filesystem here would be testing two things at
         // once.
         .FS_ENABLE(1'b0),
         .IMAGE_BLOCKS(IMG_BLOCKS), .SERVE_FULL_IMAGE(SERVE_FULL)) dut (
                                                                        .clk(clk), .rst_n(rst_n),
                                                                        .pd(pd), .xcvr_dir(xcvr_dir), .xcvr_oe_n(xcvr_oe_n), .pparity(pparity),
                                                                        .pstrb_n(pstrb_n), .pcmd_n(pcmd_n), .prw(prw),
                                                                        .pbsy_n(pbsy_n), .pres_n(pres_n), .pchk(pchk),
                                                                        .ph0(ph0), .aux7(aux7),
                                                                        .sd_clk(sd_clk), .sd_cs_n(sd_cs_n), .sd_mosi(sd_mosi), .sd_miso(sd_miso),
                                                                        .sd_cd_n(sd_cd_n), .sd_wp_n(sd_wp_n),
                                                                        .led(led), .usb_tx(usb_tx), .usb_rx(1'b1), .usb_tx_alt(usb_tx_alt));

   sd_model #(.FS_IMAGE(1'b1)) card (.sd_clk(sd_clk), .sd_cs_n(sd_cs_n),
                                     .sd_mosi(sd_mosi), .sd_miso(sd_miso));

   integer errors = 0;

   // What the router last asked the card for. Checking the LBA is how a read of
   // a high block is shown to have reached the right sector: the model card
   // returns zeros outside its small synthetic image, so the data cannot say.
   reg [31:0] last_rd_lba = 32'hFFFF_FFFF;
   reg [31:0] last_wr_lba = 32'hFFFF_FFFF;
   always @(posedge clk) if (rst_n) begin
      if (dut.sd_rd) last_rd_lba <= dut.sd_lba;
      if (dut.sd_wr) last_wr_lba <= dut.sd_lba;
   end

   task check(input cond, input [511:0] what);
      begin
         if (cond) $display("    ok    %0s", what);
         else begin $display("*** FAIL  %0s", what); errors = errors + 1; end
      end
   endtask

   // ---- contention monitor, live for the whole run -------------------------
   // The only failure mode that can damage hardware, so it is checked
   // continuously rather than at the end.
   always @(posedge clk) begin
      if (rst_n && host_oe && xcvr_dir) begin
         $display("*** CONTENTION at %0t: host driving PD while xcvr_dir is high", $time);
         errors = errors + 1;
      end
   end

   // ---- parity monitor ------------------------------------------------------
   // Odd parity across whatever is on the wire, in BOTH directions
   // (PROTOCOL.md 9.4). Sampled in the middle of every strobe, whoever is
   // driving, so the host-driven bytes and the write trailer are covered too.
   integer par_checked = 0, par_bad = 0;
   task check_parity;
      begin
         par_checked = par_checked + 1;
         if (pparity !== ~(^pd)) begin
            if (par_bad < 5)
              $display("*** parity wrong at %0t: bus $%02h, /PPARITY %0b", $time, pd, pparity);
            par_bad = par_bad + 1;
         end
      end
   endtask

   // ---- the //e's host model ------------------------------------------------
   integer pad_checked = 0, pad_bad = 0;

   task iie_write_byte(input [7:0] b);
      begin
         prw = 1'b0;
         host_drv = b; host_oe = 1'b1;
         #100;                              // the card's setup is not generous
         // PAD-LEVEL contention check, which is a different question from the
         // xcvr_dir monitor above. That one asks whether the '245 is pointed
         // the wrong way; this one asks whether the FPGA's own pad drivers let
         // go. If a pad is still driving while the host drives,
         // the net resolves to x here and the byte will not read back as what
         // was put on it. On real hardware that is two drivers on a 5V net.
         pad_checked = pad_checked + 1;
         if (pd !== b) begin
            if (pad_bad < 5)
              $display("*** pad contention at %0t: host drove $%02h, bus reads $%02h",
                       $time, b, pd);
            pad_bad = pad_bad + 1;
         end
         if (LATE_NS == 0) begin
            pstrb_n = 1'b0;
            #(STROBE_NS/2); check_parity; #(STROBE_NS/2);
            pstrb_n = 1'b1;
         end else begin
            host_oe = 1'b0;                  // bus floats: the 374 is not on yet
            #1;
            pstrb_n = 1'b0;                  // strobe falls over a floating bus
            #(LATE_NS);
            host_drv = b; host_oe = 1'b1;    // ...and the byte appears mid-strobe
            #((STROBE_NS - LATE_NS)/2); check_parity; #((STROBE_NS - LATE_NS)/2);
            pstrb_n = 1'b1;                  // still valid at the trailing edge
         end
         #(BYTE_NS - STROBE_NS - 100);
      end
   endtask

   // NO settling delay before the strobe. This is the whole point.
   task iie_read_byte(output [7:0] b);
      begin
         host_oe = 1'b0;
         pstrb_n = 1'b0;
         #(STROBE_NS/2);
         b = pd;
         check_parity;
         #(STROBE_NS/2);
         pstrb_n = 1'b1;
         #(BYTE_NS - STROBE_NS);
      end
   endtask

   task iie_cmd(input v);
      begin
         pcmd_n = ~v;
         #PCMD_HOLD;
      end
   endtask

   // The ack is NOT strobed: the card parks the byte, then raises /PCMD, and
   // the drive is expected to have sampled its port in between.
   task iie_ack(input [7:0] b);
      begin
         prw = 1'b0;
         #500;
         host_drv = b; host_oe = 1'b1;
         #2000;
         pcmd_n = 1'b1;
         #PCMD_HOLD;
         host_oe = 1'b0;
         #200;
      end
   endtask

   // The handshake, both phases. Returns the drive's response byte.
   task iie_handshake(output [7:0] resp);
      begin
         iie_cmd(1);
         // PR/-W first, THEN wait for BSY. The drive holds off driving the bus
         // until the host has pointed the transceiver at it, so a host that
         // waits for BSY before turning the bus around deadlocks against a
         // correct drive.
         prw = 1'b1;
         wait (pbsy_n === 1'b0);
         iie_read_byte(resp);
         iie_ack(8'h55);
      end
   endtask

   reg [7:0] b, resp;
   reg [7:0] got_status [0:3];
   reg [7:0] got_data [0:511];
   integer i, bad;

   // A full read transaction at //e timing, ending in the abandon.
   task iie_read_block(input [23:0] blk);
      begin
         iie_handshake(resp);
         if (resp !== 8'h01) begin
            $display("*** phase-1 response was $%02h, expected $01", resp);
            errors = errors + 1;
         end
         iie_write_byte(8'h00);             // READ
         iie_write_byte(blk[23:16]);
         iie_write_byte(blk[15:8]);
         iie_write_byte(blk[7:0]);
         iie_write_byte(8'h64);
         iie_write_byte(8'h14);

         iie_handshake(resp);
         if (resp !== 8'h02) begin
            $display("*** phase-2 response was $%02h, expected $02", resp);
            errors = errors + 1;
         end

         // THE STRICT PART. PR/-W goes high and the first strobe follows with
         // no settling delay whatsoever.
         prw = 1'b1;
         for (i = 0; i < 4; i = i + 1)   iie_read_byte(got_status[i]);
         for (i = 0; i < 512; i = i + 1) iie_read_byte(got_data[i]);

         // ...and the card walks away rather than taking the 20 tag bytes.
         iie_cmd(1);
         iie_cmd(0);
         #PCMD_HOLD;
      end
   endtask

   // A full WRITE at //e timing, including the 6-byte trailer the card emits
   // after the data and before it asks for its parity check (PROTOCOL.md 9.3).
   // A drive that consumes only 512 leaves those six bytes strobed with nobody
   // generating /PPARITY, and the ROM returns $27 on a block it wrote
   // correctly.
   task iie_write_block(input [23:0] blk, input [7:0] seed);
      begin
         iie_handshake(resp);
         if (resp !== 8'h01) begin
            $display("*** write phase-1 response $%02h, expected $01", resp);
            errors = errors + 1;
         end
         iie_write_byte(8'h01);             // WRITE
         iie_write_byte(blk[23:16]);
         iie_write_byte(blk[15:8]);
         iie_write_byte(blk[7:0]);
         iie_write_byte(8'h64);
         iie_write_byte(8'h14);

         iie_handshake(resp);
         if (resp !== 8'h03) begin
            $display("*** write phase-2 response $%02h, expected $03 (receive data)", resp);
            errors = errors + 1;
         end

         for (i = 0; i < 512; i = i + 1) iie_write_byte(seed ^ i[7:0]);

         // The trailer: $00, $47, $46, $FF, $FF EOR $47, $46 EOR $FF, where
         // $47/$46 are the high and low bytes of the block number.
         iie_write_byte(8'h00);
         iie_write_byte(blk[15:8]);
         iie_write_byte(blk[7:0]);
         iie_write_byte(8'hFF);
         iie_write_byte(8'hFF ^ blk[15:8]);
         iie_write_byte(blk[7:0] ^ 8'hFF);

         // Second confirmation: the drive says $06, "perform the write".
         iie_handshake(resp);
         if (resp !== 8'h06) begin
            $display("*** write phase-3 response $%02h, expected $06 (perform)", resp);
            errors = errors + 1;
         end

         prw = 1'b1;
         for (i = 0; i < 4; i = i + 1) iie_read_byte(got_status[i]);
         iie_cmd(1);
         iie_cmd(0);
         #PCMD_HOLD;
      end
   endtask


   initial begin
      repeat (10) @(posedge clk);
      rst_n = 1;

      // Let the SD self-test finish first: on the real board it runs to
      // completion long before a host could ask for anything, and overlapping
      // the two would be testing something that never happens.
      wait (dut.selftest_done === 1'b1);
      repeat (50) @(posedge clk);

      check(pbsy_n === 1'b1, "BSY released at idle");
      check(xcvr_dir === 1'b0, "bus points at the host at idle");

      // ---- 1. the get-status read, which is what every boot starts with
      // -------
      $display("-- get-status read of $FFFFFF at //e timing (%0d ns strobe, %0d ns/byte)",
               STROBE_NS, BYTE_NS);
      iie_read_block(24'hFFFFFF);

      // The FIRST command after power-up reports "drive was reset" in status 2.
      $display("    status bytes: %02h %02h %02h %02h",
               got_status[0], got_status[1], got_status[2], got_status[3]);
      check(got_status[2] === 8'h80, "first command reports drive-was-reset ($80)");

      $write("    name: ");
      for (i = 0; i < 13; i = i + 1)
        $write("%0s", (got_data[i] >= 8'h20 && got_data[i] < 8'h7F) ? got_data[i] : "?");
      $display("");
      check(got_data[0] === "P" && got_data[1] === "R" && got_data[2] === "O"
            && got_data[3] === "F", "device name survived the strict read");

      // Byte 0 is the one that gets lost if it was not pre-staged, so say so
      // explicitly rather than letting it hide inside the name check.
      check(got_data[0] !== 8'hxx && got_data[0] === "P",
            "byte 0 was already on the bus when PR/-W rose");

      // ---- 2. the geometry claim, read back through the host's own path
      // ------- geometry.v is unit-tested on its own; this checks the number
      // actually reaches offset $13/$14 of the table the //e reads.
      $display("-- the claimed block count, as the //e sees it");
      $display("    offset $12..$14 = %02h %02h %02h",
               got_data[16'h12], got_data[16'h13], got_data[16'h14]);
      check(got_data[16'h13] === dut.claim_blocks[15:8]
            && got_data[16'h14] === dut.claim_blocks[7:0],
            "spare table reports the geometry claim");
      check((got_data[16'h14] & 8'h03) === 8'h00,
            "claimed count is a multiple of 4, so the ROM will hand off");
      check({got_data[16'h13], got_data[16'h14]} <= 16'hFFFC,
            "claimed count is not above $FFFC");

      // ---- 3. parity, including on host-driven bytes
      // --------------------------
      $display("-- parity across %0d strobed bytes", par_checked);
      check(par_bad == 0, "odd parity correct on every byte, both directions");

      // ---- 4. the drive let go
      // ------------------------------------------------
      check(pbsy_n === 1'b1, "BSY released after the abandon");
      check(xcvr_dir === 1'b0, "bus released after the abandon");

      // ---- 5. a second transaction, to check the abandon left it usable
      // The checks above do not cover a drive that the first read left in a
      // subtly wrong state. Such a drive never answers again.
      $display("-- a second get-status read, after the abandon");
      iie_read_block(24'hFFFFFF);
      check(got_data[0] === "P" && got_data[1] === "R",
            "the drive still answers after being abandoned once");
      check(par_bad == 0, "parity still clean on the second transaction");
      check(got_status[2] === 8'h00, "and does NOT report the reset a second time");

      // ---- 5b. a block we do not serve
      // --------------------------------------- $010000 = 65536, which is above
      // BOTH the aligned default ($FE00) and the decoupled maximum ($FFFF), and
      // still well below the magic window. A block number that is only out of
      // range in one of the two geometry builds makes this test pass or fail
      // depending on which variant runs it.
      $display("-- reading a block outside the served range");
      // Lenient reads: trimmed images (NOX) read absolute blocks past their own
      // EOF and expect zeros, the way every software emulator answers. Only
      // WRITES beyond the range are refused.
      iie_read_block(24'h010000);
      check(got_status[2][6] === 1'b0, "beyond-range read is lenient: no error bit");
      bad = 0;
      for (i = 0; i < 512; i = i + 1)
        if (got_data[i] !== 8'h00) bad = bad + 1;
      check(bad == 0, "and it serves 512 zeros");
      iie_read_block(24'hFFFFFF);
      check(got_status[2][6] === 1'b0, "and the next command is clean too");

      // ---- 5d. the A2E-check knock: runtime menu entry
      // ------------------------ With the window closed, the info block is not
      // served ($FFFFC0 answers as a polite no-op). After the knock, it serves
      // for real: byte 4 is the $A5 emulator signature. A deliberately WRONG
      // knock must not open it.
      $display("-- the A2E-check knock");
      iie_read_block(24'hFFFF41);
      iie_read_block(24'hFFFF32);
      iie_read_block(24'hFFFF45);
      iie_read_block(24'hFFFF43);              // WRONG last byte: no knock
      check(dut.window_live === 1'b0, "a wrong knock leaves the window closed");
      iie_read_block(24'hFFFF41);
      iie_read_block(24'hFFFF32);
      iie_read_block(24'hFFFF45);
      iie_read_block(24'hFFFF44);              // the real knock
      check(dut.window_live === 1'b1, "the knock opens the window");
      check(dut.knock_menu === 1'b1, "and forces the menu");
      iie_read_block(24'hFFFFC0);
      check(got_data[4] === 8'hA5, "info block serves the emulator signature");
      iie_read_block(24'hFFFFC1);
      check(got_data[0] === dut.g_sel.u_sel.boot_rom[0]
            && got_data[1] === dut.g_sel.u_sel.boot_rom[1],
            "the boot blob is fetchable through the window");
      // A SELECT closes the window and clears the forced menu, as at boot.
      // With FS_ENABLE=0 img_count is 0 and no SELECT can strobe, so window
      // closure is exercised by the boot-claim path in section 5c instead.

      // ---- 5c. the drive's own scratch buffer, block $FFFFFE
      // ------------------ The point of the check is the read of block 0 in the
      // MIDDLE. Sharing the block buffer passes a write-then-read test and
      // fails this one.
      $display("-- block $FFFFFE, the RAM scratch buffer");
      // Geometry overloading: on a full-size image (the decoupled variant of
      // this bench) the closed window serves $FFFFFE as the image's top block,
      // and the scratch buffer only exists during the menu phase. Enter it
      // the way a boot does: status then block 0 is a boot, and the blob's
      // claim opens the window. The aligned variant never aliases and passes
      // either way.
      iie_read_block(24'hFFFFFF);
      iie_read_block(24'd0);
      iie_write_block(24'hFFFFFE, 8'h3C);
      iie_read_block(24'd0);                        // clobbers the block buffer
      iie_read_block(24'hFFFFFE);
      bad = 0;
      for (i = 0; i < 512; i = i + 1)
        if (got_data[i] !== (8'h3C ^ i[7:0])) bad = bad + 1;
      check(bad == 0, "the scratch buffer survived an intervening block read");
      if (bad) $display("    %0d byte(s) differ, first: got $%02h want $%02h",
                        bad, got_data[0], 8'h3C);

      // ---- 6. a write, trailer and all
      // ---------------------------------------
      $display("-- a WRITE of block 77 at //e timing, with the 6-byte trailer");
      iie_write_block(24'd77, 8'h5A);
      check(par_bad == 0, "parity generated across the data AND the trailer");
      check(pbsy_n === 1'b1, "BSY released after the write");

      $display("-- reading block 77 back");
      iie_read_block(24'd77);
      bad = 0;
      for (i = 0; i < 512; i = i + 1)
        if (got_data[i] !== (8'h5A ^ i[7:0])) bad = bad + 1;
      check(last_wr_lba === 32'd77, "the write went to sector 77 of the card");
      check(bad == 0, "every byte read back matches what was written");
      if (bad)
        $display("    %0d byte(s) differ, first: got $%02h want $%02h",
                 bad, got_data[0], 8'h5A);

      // ---- 6b. a BURST of writes ----
      // Four back-to-back writes to consecutive blocks with distinct data;
      // every one must commit. sd_model only remembers the last two written
      // sectors, so the read-back check below covers only the final block; the
      // CMD24/COMMIT trace is what reveals a dropped write.
      $display("-- BURST: 4 back-to-back writes to blocks 100..103");
      iie_write_block(24'd100, 8'hA0);
      iie_write_block(24'd101, 8'hA1);
      iie_write_block(24'd102, 8'hA2);
      iie_write_block(24'd103, 8'hA3);
      $display("-- reading block 103 back (last of the burst)");
      iie_read_block(24'd103);
      bad = 0;
      for (i = 0; i < 512; i = i + 1)
        if (got_data[i] !== (8'hA3 ^ i[7:0])) bad = bad + 1;
      check(bad == 0, "burst: last block (103) reads back correct");
      if (bad) $display("    103 mismatch: got $%02h want $%02h", got_data[0], 8'hA3);
      check(last_wr_lba === 32'd103, "burst: last write LBA was 103");

      // ---- 7. the geometry claim reaches the host, and the served range
      // ------ With a decoupled image this reads a block ABOVE the claim, which
      // is the entire reason the two counts are separate. With the default
      // image it is an ordinary in-range read and still has to work.
      $display("-- claim $%04h, served $%04h, decoupled %0b",
               dut.claim_blocks, dut.served_blocks, dut.geo_decoupled);
      if (dut.geo_decoupled) begin
         $display("-- reading block $FFFE, which is above the claim");
         iie_read_block(24'h00FFFE);
         check(dut.u_router.out_of_range === 1'b0,
               "a block above the claim is still inside the served range");
         check(last_rd_lba === 32'h0000FFFE,
               "and the card was asked for that sector, not a clamped one");
      end else begin
         check(dut.served_blocks === dut.claim_blocks,
               "an aligned image serves exactly what it claims");
      end

      // ---- 8. the telemetry the DB-25 test will be read through ----
      // A counter that is wired up but never increments looks exactly like a
      // idle, working bus.
      $display("-- transaction telemetry");
      $display("    last op $%02h  last blk $%06h  cmds %0d  reads %0d  writes %0d",
               dut.last_op, dut.last_blk, dut.n_cmd, dut.n_read_ok, dut.n_write_ok);
      $display("    naks %0d  abandons %0d  bad opcodes %0d  out-of-range %0d",
               dut.n_nak, dut.n_abandon, dut.n_badop, dut.n_oor);
      check(dut.n_cmd      >  8'd0, "commands were counted");
      check(dut.n_read_ok  >  8'd0, "completed reads were counted");
      check(dut.n_write_ok >  8'd0, "completed writes were counted");
      check(dut.n_abandon  >  8'd0, "host abandons were counted");
      check(dut.n_oor      == 8'd1, "the single out-of-range block was counted once");
      // the lenient read still shows in telemetry, with the block that asked
      check(dut.last_oor_blk === 24'h010000, "and its block number was recorded");
      check(dut.n_badop    == 8'd0, "no bad opcode was sent, and none was counted");
      // Which block was last depends on the geometry build: the decoupled
      // variant reads $FFFE above the claim, the aligned one stops at the
      // burst's block-103 read back (section 6b). Hard-coding either makes the
      // test pass in one build and fail in the other, so the check has to cover
      // both orders.
      check(dut.last_blk === (dut.geo_decoupled ? 24'h00FFFE : 24'd103),
            "the last block number was recorded");
      check(dut.last_op  === 8'h00,  "and the last opcode (a read) with it");

      // ---- 9. an opcode the drive must refuse
      // --------------------------------- $03 is Force Sparing: destructive on
      // a real drive. Refused before the confirmation, so the host never gets
      // to start a data phase.
      $display("-- sending opcode $03, which must be refused");
      iie_handshake(resp);
      iie_write_byte(8'h03);
      iie_write_byte(8'h00); iie_write_byte(8'h00); iie_write_byte(8'h01);
      iie_write_byte(8'h64); iie_write_byte(8'h14);
      repeat (200) @(posedge clk);
      check(dut.n_badop == 8'd1, "the bad opcode was counted");
      check(pbsy_n === 1'b1, "and the drive released BSY instead of answering");
      pcmd_n = 1'b1; #PCMD_HOLD;

      // Reported at the END so it covers the write's 518 host-driven bytes as
      // well as the command frames, rather than just the first six.
      $display("-- pad drivers released across %0d host-driven bytes", pad_checked);
      check(pad_bad == 0, "the FPGA's pads let go whenever the host drives");

      $display("");
      $display("    %0d bytes strobed at %0d ns, %0d parity failures",
               par_checked, STROBE_NS, par_bad);
      if (errors == 0)
        $display("PASS: the //e's timing is met, byte 0 included");
      else
        $display("FAIL: %0d problem(s)", errors);
      $finish;
   end

   initial begin
      #160_000_000;
      $display("*** TIMEOUT at //e timing: sd state %0d, selftest_done %0b",
               dut.sd_dbg, dut.selftest_done);
      $display("    a stall here is the drive failing to keep up with the host");
      $finish;
   end

endmodule

`default_nettype wire
