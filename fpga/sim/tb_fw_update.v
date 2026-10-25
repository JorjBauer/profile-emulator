`timescale 1ns/1ps
// tb_fw_update: the SD firmware updater against a hand-built FAT32 image and
// a behavioral W25Q128.
//
// The card: 4 sectors per cluster, the FAT at LBA 100, data at LBA 200. The
// root directory is clusters 2 -> 3, and the update file is in cluster 3, so
// finding it means following the FAT. Ahead of it in the directory are a
// volume label, a long-name piece, a deleted entry and a directory, all
// carrying the same 8.3 name, and all of which have to be skipped. The file
// itself is fragmented: clusters 10 -> 7 -> 12, and long enough to reach
// all three. The rest of the FAT sector is other files' entries, so reading
// the wrong entry gives a wrong cluster, and the card answers a read past its
// end with an error, the way a real one does.
//
// The flash is 256 KB here, with the golden image at 128 KB. Each case
// checks that nothing at or above that address is ever written.
//
// Cases, each from reset:
//   1. a good file: written, read back, and reboot requested
//   2. the same file again, now matching BUILD_ID: nothing happens
//   2b. an older file: nothing happens either
//   2c. a file with an old-style month-day-hour-minute ID: older, ignored
//   3. a corrupted payload: refused on CRC, flash untouched
//   4. a file for another board: refused, flash untouched
//   5. no file at all: nothing happens
//   6. a flash that drops a bit: refused on verify
//   7. a good file, but the drive never goes idle: nothing starts
//   8. host activity after the write holds the reboot off
//   9. the slot headers: which slot we came from, the build IDs, EMPTY and
//      UNKNOWN, and a header in the older "PEV:<build>:<version>" form

`default_nettype none

module tb_fw_update;

   localparam [31:0] RUNNING = 32'h80068B27;   // the BUILD_ID we "are"
   localparam [31:0] NEWER   = 32'h80068C00;   // the file's BUILD_ID
   localparam [31:0] OTHER   = 32'h80001000;   // some other build
   localparam integer PAYLOAD_SECTORS = 9;
   localparam integer FLASH_BYTES = 262144;
   localparam [23:0] GOLDEN = 24'h020000;

   reg clk = 0;
   always #10 clk = ~clk;
   reg rst = 1;

   // ---- the card ------------------------------------------------------------
   reg [7:0] disk [0:512*320-1];
   integer   i;

   task put32(input integer at, input [31:0] v);
      begin
         disk[at] = v[7:0]; disk[at+1] = v[15:8];
         disk[at+2] = v[23:16]; disk[at+3] = v[31:24];
      end
   endtask

   task dirent(input integer at, input [87:0] name, input [7:0] attr,
               input [31:0] clus, input [31:0] size);
      integer b;
      begin
         for (b = 0; b < 11; b = b + 1) disk[at+b] = name[87-8*b -: 8];
         disk[at+11] = attr;
         for (b = 12; b < 32; b = b + 1) disk[at+b] = 8'h00;
         disk[at+20] = clus[23:16]; disk[at+21] = clus[31:24];
         disk[at+26] = clus[7:0];   disk[at+27] = clus[15:8];
         put32(at+28, size);
      end
   endtask

   function integer clus_byte(input integer c, input integer s);
      clus_byte = (200 + (c - 2) * 4 + s) * 512;
   endfunction

   // payload sector n lives in file sector n+1 (sector 0 is the header)
   function integer file_byte(input integer fs);
      integer c;
      begin
         c = (fs < 4) ? 10 : (fs < 8) ? 7 : 12;
         file_byte = clus_byte(c, fs % 4);
      end
   endfunction

   function [31:0] crc_byte(input [31:0] c, input [7:0] b);
      integer n;
      reg [31:0] x;
      begin
         x = c ^ {24'd0, b};
         for (n = 0; n < 8; n = n + 1)
           x = x[0] ? ((x >> 1) ^ 32'hEDB88320) : (x >> 1);
         crc_byte = x;
      end
   endfunction

   // A bitstream's header, as the Makefile writes it, 256 bytes into a
   // slot: $FF $00 "PEV:<build>" $00. Builds from before that wrote
   // "PEV:<build>:1.0" (old = 1).
   function [7:0] hex_digit(input [3:0] n);
      hex_digit = (n < 10) ? "0" + n : "A" + n - 10;
   endfunction

   function [7:0] header_byte(input integer i, input [31:0] build, input old);
      begin
         if (i == 0)       header_byte = 8'hFF;
         else if (i == 1)  header_byte = 8'h00;
         else if (i < 6)   header_byte = "PEV:" >> (8 * (5 - i));
         else if (i < 14)  header_byte = hex_digit(build[31 - 4*(i-6) -: 4]);
         else if (old && i < 18) header_byte = ":1.0" >> (8 * (17 - i));
         else              header_byte = 8'h00;
      end
   endfunction

   // a build ID as the eight hex digits the updater reports
   function [63:0] id_text(input [31:0] build);
      integer d;
      begin
         for (d = 0; d < 8; d = d + 1)
           id_text[63 - 8*d -: 8] = hex_digit(build[31 - 4*d -: 4]);
      end
   endfunction

   // the new image's header
   function [7:0] payload(input integer at);
      if (at >= 256 && at < 256 + 24) payload = header_byte(at - 256, NEWER, 0);
      else payload = (at * 7 + (at >> 9) * 13 + 8'h5A) & 8'hFF;
   endfunction

   reg [31:0] good_crc;

   task build_card(input integer with_file, input [31:0] board, input integer corrupt);
      integer n, s;
      reg [31:0] c;
      begin
         for (n = 0; n < 512*320; n = n + 1) disk[n] = 8'h00;
         // Every other FAT entry in the sector belongs to some other file,
         // with a nonzero top byte, the way a real FAT has. Reading the
         // wrong entry then gives a wrong cluster instead of a lucky zero.
         for (n = 0; n < 128; n = n + 1) put32(100*512 + n*4, 32'h0A000000 + n);
         // FAT: 2 -> 3 -> end (the directory), 10 -> 7 -> 12 -> end (the file)
         put32(100*512 + 2*4, 32'h00000003);
         put32(100*512 + 3*4, 32'h0FFFFFFF);
         put32(100*512 + 10*4, 32'h00000007);
         put32(100*512 + 7*4,  32'h0000000C);
         put32(100*512 + 12*4, 32'h0FFFFFFF);
         // directory, cluster 2: the decoys fill it, so the scan has to
         // carry on into cluster 3
         dirent(clus_byte(2,0) + 0,  "PROFILE    ", 8'h08, 0, 0);          // volume label
         dirent(clus_byte(2,0) + 32, "R2UPDATEBIN", 8'h0F, 0, 0);          // long-name piece
         dirent(clus_byte(2,0) + 64, "R2UPDATEBIN", 8'h20, 10, 99);
         disk[clus_byte(2,0) + 64] = 8'hE5;                                // deleted
         dirent(clus_byte(2,0) + 96, "R2UPDATEBIN", 8'h10, 5, 0);          // a directory
         for (s = 0; s < 4; s = s + 1)
           for (n = (s == 0 ? 4 : 0); n < 16; n = n + 1)
             dirent(clus_byte(2,s) + n*32, "OTHER   TXT", 8'h20, 0, 10);
         // cluster 3: the file, then the end of the directory
         if (with_file)
           dirent(clus_byte(3,0) + 0, "R2UPDATEBIN", 8'h20, 10, 512*(PAYLOAD_SECTORS+1));
         // the payload, and its CRC
         c = 32'hFFFFFFFF;
         for (s = 0; s < PAYLOAD_SECTORS; s = s + 1)
           for (n = 0; n < 512; n = n + 1) begin
              disk[file_byte(s+1) + n] = payload(s*512 + n);
              c = crc_byte(c, payload(s*512 + n));
           end
         good_crc = c ^ 32'hFFFFFFFF;
         if (corrupt) disk[file_byte(3) + 17] = disk[file_byte(3) + 17] ^ 8'h01;
         // the header
         disk[file_byte(0)+0] = "P"; disk[file_byte(0)+1] = "E";
         disk[file_byte(0)+2] = "F"; disk[file_byte(0)+3] = "W";
         for (n = 0; n < 4; n = n + 1) disk[file_byte(0)+4+n] = board[31-8*n -: 8];
         put32(file_byte(0)+8,  512*PAYLOAD_SECTORS);
         put32(file_byte(0)+12, good_crc);
         put32(file_byte(0)+16, NEWER);
      end
   endtask

   // ---- the SD side: serve a sector the way sd_spi does ---------------------
   wire        req;
   wire [31:0] lba;
   reg         fire = 0, sd_done = 0, sd_buf_we = 0, sd_err = 0;
   reg [7:0]   sd_buf_wdata;
   reg [9:0]   sd_buf_addr;
   integer     sd_reads = 0;
   reg         busy_sd = 0;

   always @(posedge clk) begin
      fire <= 1'b0;
      if (req && !fire && !busy_sd) fire <= 1'b1;
   end
   always @(posedge clk) if (fire && lba >= 320) begin
      // past the end of the card: the read fails after a while
      busy_sd = 1;
      repeat (50) @(posedge clk);
      sd_err <= 1'b1; sd_done <= 1'b1; @(posedge clk);
      sd_err <= 1'b0; sd_done <= 1'b0;
      busy_sd = 0;
   end else if (fire) begin
      busy_sd = 1;
      sd_reads = sd_reads + 1;
      repeat (20) @(posedge clk);
      for (i = 0; i < 512; i = i + 1) begin
         sd_buf_addr  <= i[9:0];
         sd_buf_wdata <= disk[lba*512 + i];
         sd_buf_we    <= 1'b1;
         @(posedge clk);
         sd_buf_we    <= 1'b0;
         @(posedge clk);
      end
      repeat (5) @(posedge clk);
      sd_done <= 1'b1; @(posedge clk); sd_done <= 1'b0;
      busy_sd = 0;
   end

   // ---- the flash -----------------------------------------------------------
   wire flash_csn, flash_sck, flash_mosi;
   wire flash_miso;
   integer writes_above_golden;

   flash_model #(.BYTES(FLASH_BYTES), .GOLDEN(GOLDEN)) u_fm (
                                                             .csn(flash_csn), .sck(flash_sck), .mosi(flash_mosi), .miso(flash_miso));

   // What the flash holds before each case. A slot is one of:
   //   0  our own BUILD_ID                  1  another build
   //   2  no header comment of ours         3  erased
   //   4  our own BUILD_ID, older header form
   task slot_setup(input [23:0] base, input integer kind);
      integer n;
      begin
         if (kind == 0)
           for (n = 0; n < 24; n = n + 1)
             u_fm.mem[base + 256 + n] = header_byte(n, RUNNING, 0);
         else if (kind == 1)
           for (n = 0; n < 24; n = n + 1)
             u_fm.mem[base + 256 + n] = header_byte(n, OTHER, 0);
         else if (kind == 4)
           for (n = 0; n < 24; n = n + 1)
             u_fm.mem[base + 256 + n] = header_byte(n, RUNNING, 1);
         else if (kind == 3)
           for (n = 0; n < 512; n = n + 1) u_fm.mem[base + n] = 8'hFF;
      end
   endtask

   task flash_setup(input integer pri, input integer gld);
      begin
         u_fm.fill_old();
         slot_setup(24'd0, pri);
         slot_setup(GOLDEN, gld);
         u_fm.snap();
      end
   endtask

   // ---- the updater ---------------------------------------------------------
   reg         start = 0;
   reg         idle = 1;
   wire        running, reboot;
   wire [3:0]  err;
   wire [15:0] progress, total;
   wire [1:0]  phase;
   wire [31:0] file_build;
   wire [4:0]  dbg_state;
   wire [63:0] pri_id, gld_id;
   wire [7:0]  run_slot;

   fw_update #(.CLK_HZ(50_000_000), .BUILD_ID(RUNNING), .GOLDEN_ADDR(GOLDEN),
               .REBOOT_HOLD(100)) dut (
                                       .clk(clk), .rst(rst), .start(start), .idle(idle),
                                       .root_clus(32'd2), .fat_start(32'd100), .data_start(32'd200),
                                       .spc_log(4'd2),
                                       .req(req), .lba(lba), .fire(fire), .sd_done(sd_done), .sd_err(sd_err),
                                       .sd_buf_we(sd_buf_we), .sd_buf_wdata(sd_buf_wdata),
                                       .sd_buf_addr(sd_buf_addr),
                                       .running(running), .reboot(reboot), .err(err),
                                       .progress(progress), .total(total), .phase(phase),
                                       .file_build(file_build), .dbg_state(dbg_state),
                                       .pri_id(pri_id), .gld_id(gld_id), .run_slot(run_slot),
                                       .flash_csn(flash_csn), .flash_sck(flash_sck),
                                       .flash_mosi(flash_mosi), .flash_miso(flash_miso));

   integer fails = 0;
   task check(input cond, input [511:0] what);
      begin
         if (!cond) begin
            fails = fails + 1;
            $display("    FAIL  %0s", what);
         end else
           $display("    ok    %0s", what);
      end
   endtask

   // run until the updater settles (or reboots), at most `limit` clocks
   reg was_running;
   task run_case(input integer limit);
      integer t;
      begin
         rst = 1; start = 0; was_running = 0;
         repeat (5) @(posedge clk);
         rst = 0;
         repeat (5) @(posedge clk);
         start = 1;
         t = 0;
         while (t < limit && !reboot
                && !(dbg_state == 5'd21 || dbg_state == 5'd22)) begin   // S_FAIL, S_QUIET
            @(posedge clk);
            if (running) was_running = 1;
            t = t + 1;
         end
         repeat (5) @(posedge clk);
      end
   endtask

   // reset, then wait for the header reads to finish (S_IDLE)
   task probe_only;
      begin
         rst = 1; start = 0;
         repeat (5) @(posedge clk);
         rst = 0;
         repeat (5) @(posedge clk);
         while (dbg_state != 5'd0) @(posedge clk);
         repeat (5) @(posedge clk);
      end
   endtask

   function flash_matches(input integer dummy);
      integer n;
      begin
         flash_matches = 1;
         for (n = 0; n < 512*PAYLOAD_SECTORS; n = n + 1)
           if (u_fm.mem[n] !== payload(n)) flash_matches = 0;
      end
   endfunction

   function flash_untouched(input integer dummy);
      integer n;
      begin
         flash_untouched = 1;
         for (n = 0; n < 512*PAYLOAD_SECTORS; n = n + 1)
           if (u_fm.mem[n] !== u_fm.orig[n]) flash_untouched = 0;
      end
   endfunction

   initial begin
      $display("-- 1. a good update file");
      build_card(1, "R2  ", 0);
      flash_setup(0, 1);
      run_case(5_000_000);
      check(was_running, "ran the update");
      check(err == 4'd0, "no error");
      check(total == PAYLOAD_SECTORS, "nine payload sectors");
      check(flash_matches(0), "the primary holds the payload");
      check(u_fm.golden_ok(0), "the golden image is untouched");
      check(u_fm.over_golden == 0, "nothing was written at or above the golden address");
      check(reboot, "reboot requested");
      check(file_build == NEWER, "read the file's BUILD_ID");
      check(run_slot == "P", "running from the primary");
      check(pri_id == id_text(NEWER), "the primary reads as the new build");
      check(gld_id == id_text(OTHER), "the golden copy's build");

      $display("-- 2. the same file, already running it");
      put32(file_byte(0)+16, RUNNING);
      flash_setup(0, 1);
      run_case(5_000_000);
      check(!was_running && !reboot && err == 4'd0, "nothing happens");

      $display("-- 2b. an older file");
      put32(file_byte(0)+16, RUNNING - 1);
      flash_setup(0, 1);
      run_case(5_000_000);
      check(!was_running && !reboot && err == 4'd0 && flash_untouched(0),
            "nothing happens: newer files only");

      $display("-- 2c. a file with an old-style ID");
      put32(file_byte(0)+16, 32'h10251745);
      flash_setup(0, 1);
      run_case(5_000_000);
      check(!was_running && !reboot && err == 4'd0 && flash_untouched(0),
            "nothing happens: it sorts below every new ID");

      $display("-- 3. a corrupted payload");
      build_card(1, "R2  ", 1);
      flash_setup(0, 1);
      run_case(5_000_000);
      check(err == 4'd4, "refused on CRC (E_CRC)");
      check(flash_untouched(0), "flash untouched");
      check(!running && !reboot, "stopped, no reboot");

      $display("-- 4. a file for another board");
      build_card(1, "ULX3", 0);
      flash_setup(0, 1);
      run_case(5_000_000);
      check(err == 4'd3, "refused (E_BOARD)");
      check(flash_untouched(0) && !was_running, "flash untouched, never ran");

      $display("-- 5. no update file");
      build_card(0, "R2  ", 0);
      flash_setup(0, 1);
      run_case(5_000_000);
      check(err == 4'd0 && !was_running && !reboot, "nothing happens");
      check(dbg_state == 5'd22, "settles quietly");

      $display("-- 6. a flash that drops a bit");
      build_card(1, "R2  ", 0);
      flash_setup(0, 1);
      u_fm.stuck_bit = 1;
      run_case(5_000_000);
      u_fm.stuck_bit = 0;
      check(err == 4'd9, "refused on verify (E_VERIFY)");
      check(!running && !reboot, "stopped, no reboot");
      check(u_fm.golden_ok(0), "the golden image is untouched");

      $display("-- 7. a good file, but the drive never goes idle");
      build_card(1, "R2  ", 0);
      flash_setup(0, 1);
      idle = 0;
      run_case(200_000);
      check(dbg_state == 5'd0 && !req, "waits without touching the card");
      check(flash_untouched(0) && !reboot, "flash untouched, no reboot");

      $display("-- 8. the host gets busy after the write");
      // Start it with idle high, then drop idle for the rest of the
      // write and hold the reboot off with short idle spells.
      rst = 1; start = 0;
      repeat (5) @(posedge clk);
      rst = 0; start = 1; idle = 1;
      // past the header reads (S_PROBE, S_PROBE_W) and out of S_IDLE
      while (dbg_state == 5'd0 || dbg_state == 5'd23 || dbg_state == 5'd24) @(posedge clk);
      idle = 0;
      // until S_DONE or S_FAIL
      while (dbg_state != 5'd19 && dbg_state != 5'd21) @(posedge clk);
      repeat (1000) @(posedge clk);
      check(flash_matches(0) && !reboot, "written in the background, no reboot while busy");
      idle = 1;
      repeat (50) @(posedge clk);
      idle = 0;
      repeat (1000) @(posedge clk);
      check(!reboot, "a short idle spell does not reboot");
      idle = 1;
      repeat (200) @(posedge clk);
      check(reboot, "reboots once idle lasts");

      $display("-- 9. the slot headers");
      idle = 0;   // so nothing starts
      flash_setup(0, 1);
      probe_only;
      check(run_slot == "P" && pri_id == id_text(RUNNING) && gld_id == id_text(OTHER),
            "ours in the primary: P, and both build IDs");
      flash_setup(1, 0);
      probe_only;
      check(run_slot == "G" && pri_id == id_text(OTHER) && gld_id == id_text(RUNNING),
            "ours in the golden copy: G");
      flash_setup(1, 1);
      probe_only;
      check(run_slot == "J", "ours in neither: J (a JTAG load)");
      flash_setup(3, 2);
      probe_only;
      check(pri_id == {"EMPTY", 24'd0} && gld_id == {"UNKNOWN", 8'd0} && run_slot == "J",
            "an erased primary and a golden copy with no build ID");
      flash_setup(1, 4);
      probe_only;
      check(run_slot == "G" && gld_id == id_text(RUNNING),
            "the older header form still reads as its build");
      check(u_fm.writes == 0, "reading the headers writes nothing");

      $display("");
      if (fails == 0) $display("PASS: the updater writes only good files, only below the golden image");
      else            $display("FAIL: %0d problem(s)", fails);
      $finish;
   end

   initial begin
      #2_000_000_000;
      $display("FAIL: timeout");
      $finish;
   end

endmodule

// A W25Q128JV, the parts the updater uses: 9Fh, 06h, 05h, D8h, 02h, 03h.
// Mode 0. Busy lasts a few microseconds instead of milliseconds.
module flash_model #(parameter integer BYTES = 262144,
                     parameter [23:0] GOLDEN = 24'h020000) (
                                                            input  wire csn,
                                                            input  wire sck,
                                                            input  wire mosi,
                                                            output reg  miso
                                                            );
   reg [7:0] mem [0:BYTES-1];
   reg [7:0] orig [0:BYTES-1];  // the contents before the case
   reg [7:0] cmd, sr, outb;
   reg [23:0] a;
   integer    nbit, nbyte, over_golden, busy_until, stuck_bit, writes;
   reg        wel, busy;
   reg [7:0]  inb;

   initial begin
      miso = 1'b1; wel = 0; busy = 0; over_golden = 0; stuck_bit = 0; writes = 0;
   end

   task fill_old;
      integer n;
      begin
         for (n = 0; n < BYTES; n = n + 1) mem[n] = (n >= GOLDEN) ? 8'h60 + (n & 15) : 8'hC3;
         over_golden = 0; writes = 0;
      end
   endtask

   task snap;
      integer n;
      begin
         for (n = 0; n < BYTES; n = n + 1) orig[n] = mem[n];
      end
   endtask

   function golden_ok(input integer dummy);
      integer n;
      begin
         golden_ok = 1;
         for (n = GOLDEN; n < BYTES; n = n + 1)
           if (mem[n] !== orig[n]) golden_ok = 0;
      end
   endfunction

   always @(negedge csn) begin nbit = 0; nbyte = 0; end

   always @(posedge csn) begin
      if (cmd == 8'h06) wel = 1;
      if ((cmd == 8'hD8 || cmd == 8'h02) && wel) begin
         wel = 0; busy = 1; writes = writes + 1;
         #3000 busy = 0;
      end
   end

   always @(posedge sck) if (!csn) begin
      inb = {inb[6:0], mosi};
      nbit = nbit + 1;
      if (nbit == 8) begin
         nbit = 0;
         if (nbyte == 0) cmd = inb;
         else if (nbyte <= 3 && (cmd == 8'hD8 || cmd == 8'h02 || cmd == 8'h03)) begin
            a = {a[15:0], inb};
            if (nbyte == 3 && cmd == 8'hD8 && wel) begin : erase
               integer n;
               for (n = 0; n < 65536; n = n + 1)
                 if (({a[23:16], 16'd0} + n) < BYTES) begin
                    if (({a[23:16], 16'd0} + n) >= GOLDEN) over_golden = over_golden + 1;
                    mem[{a[23:16], 16'd0} + n] = 8'hFF;
                 end
            end
         end else if (cmd == 8'h02 && wel) begin
            if (a >= GOLDEN) over_golden = over_golden + 1;
            if (a < BYTES) mem[a] = mem[a] & inb;
            a[7:0] = a[7:0] + 1'b1;                 // wraps inside the page
         end
         nbyte = nbyte + 1;
      end
   end

   // output changes on the falling edge, for the byte after the opcode/address
   integer obit;
   always @(negedge sck) if (!csn) begin
      if (nbit == 0) begin
         // a new byte is starting: load what this byte position returns
         case (cmd)
           8'h9F: outb = (nbyte == 1) ? 8'hEF : (nbyte == 2) ? 8'h40 : 8'h18;
           8'h05: outb = {7'd0, busy};
           8'h03: if (nbyte >= 4) begin
              outb = (a < BYTES) ? mem[a] : 8'hFF;
              if (stuck_bit && a == 24'd700) outb = outb ^ 8'h04;
              a = a + 1;
           end
           default: outb = 8'hFF;
         endcase
         obit = 7;
      end
      if (nbyte >= 1) begin miso = outb[obit]; obit = obit - 1; end
   end

   always @(posedge csn) miso = 1'b1;

endmodule

`default_nettype wire
