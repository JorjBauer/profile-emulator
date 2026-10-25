`timescale 1ns/1ps
// tb_fw_update: the SD firmware updater against a hand-built FAT32 image and
// a behavioral W25Q128.
//
// The card: 4 sectors per cluster, the FAT at LBA 100, data at LBA 200. The
// root directory is clusters 2 -> 3, and the update file is in cluster 3, so
// finding it means following the FAT. Ahead of it in the directory are a
// volume label, a long-name piece, a deleted entry and a directory, all
// carrying the same 8.3 name, and all of which have to be skipped. The file
// itself is fragmented: clusters 10 -> 7 -> 12.
//
// The flash is 256 KB here, with the golden image at 128 KB. Each case
// checks that nothing at or above that address is ever written.
//
// Cases, each from reset:
//   1. a good file: written, read back, and reboot requested
//   2. the same file again, now matching BUILD_ID: nothing happens
//   3. a corrupted payload: refused on CRC, flash untouched
//   4. a file for another board: refused, flash untouched
//   5. no file at all: nothing happens
//   6. a flash that drops a bit: refused on verify

`default_nettype none

module tb_fw_update;

   localparam [31:0] RUNNING = 32'h10250900;   // the BUILD_ID we "are"
   localparam [31:0] NEWER   = 32'h10251200;   // the file's BUILD_ID
   localparam integer PAYLOAD_SECTORS = 6;
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

   function [7:0] payload(input integer at);
      payload = (at * 7 + (at >> 9) * 13 + 8'h5A) & 8'hFF;
   endfunction

   reg [31:0] good_crc;

   task build_card(input integer with_file, input [31:0] board, input integer corrupt);
      integer n, s;
      reg [31:0] c;
      begin
         for (n = 0; n < 512*320; n = n + 1) disk[n] = 8'h00;
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
   reg         fire = 0, sd_done = 0, sd_buf_we = 0;
   reg [7:0]   sd_buf_wdata;
   reg [9:0]   sd_buf_addr;
   integer     sd_reads = 0;
   reg         busy_sd = 0;

   always @(posedge clk) begin
      fire <= 1'b0;
      if (req && !fire && !busy_sd) fire <= 1'b1;
   end
   always @(posedge clk) if (fire) begin
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

   // ---- the updater ---------------------------------------------------------
   reg         start = 0;
   wire        active, reboot;
   wire [3:0]  err;
   wire [15:0] progress, total;
   wire [1:0]  phase;
   wire [31:0] file_build;
   wire [4:0]  dbg_state;

   fw_update #(.CLK_HZ(50_000_000), .BUILD_ID(RUNNING), .GOLDEN_ADDR(GOLDEN),
               .REBOOT_DELAY(100)) dut (
                                        .clk(clk), .rst(rst), .start(start),
                                        .root_clus(32'd2), .fat_start(32'd100), .data_start(32'd200),
                                        .spc_log(4'd2),
                                        .req(req), .lba(lba), .fire(fire), .sd_done(sd_done), .sd_err(1'b0),
                                        .sd_buf_we(sd_buf_we), .sd_buf_wdata(sd_buf_wdata),
                                        .sd_buf_addr(sd_buf_addr),
                                        .active(active), .reboot(reboot), .err(err),
                                        .progress(progress), .total(total), .phase(phase),
                                        .file_build(file_build), .dbg_state(dbg_state),
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
   reg was_active;
   task run_case(input integer limit);
      integer t;
      begin
         rst = 1; start = 0; was_active = 0;
         repeat (5) @(posedge clk);
         rst = 0;
         repeat (5) @(posedge clk);
         start = 1;
         t = 0;
         while (t < limit && !reboot
                && !(dbg_state == 5'd21 || dbg_state == 5'd22)) begin   // S_FAIL, S_QUIET
            @(posedge clk);
            if (active) was_active = 1;
            t = t + 1;
         end
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
           if (u_fm.mem[n] !== 8'hC3) flash_untouched = 0;
      end
   endfunction

   initial begin
      $display("-- 1. a good update file");
      build_card(1, "R2  ", 0);
      u_fm.fill_old();
      run_case(5_000_000);
      check(was_active, "took the drive while updating");
      check(err == 4'd0, "no error");
      check(total == PAYLOAD_SECTORS, "six payload sectors");
      check(flash_matches(0), "the primary holds the payload");
      check(u_fm.golden_ok(0), "the golden image is untouched");
      check(u_fm.over_golden == 0, "nothing was written at or above the golden address");
      check(reboot, "reboot requested");
      check(file_build == NEWER, "read the file's BUILD_ID");

      $display("-- 2. the same file, already running it");
      put32(file_byte(0)+16, RUNNING);
      u_fm.fill_old();
      run_case(5_000_000);
      check(!was_active && !reboot && err == 4'd0, "nothing happens");

      $display("-- 3. a corrupted payload");
      build_card(1, "R2  ", 1);
      u_fm.fill_old();
      run_case(5_000_000);
      check(err == 4'd4, "refused on CRC (E_CRC)");
      check(flash_untouched(0), "flash untouched");
      check(!active && !reboot, "drive given back, no reboot");

      $display("-- 4. a file for another board");
      build_card(1, "ULX3", 0);
      u_fm.fill_old();
      run_case(5_000_000);
      check(err == 4'd3, "refused (E_BOARD)");
      check(flash_untouched(0) && !was_active, "flash untouched, drive never taken");

      $display("-- 5. no update file");
      build_card(0, "R2  ", 0);
      u_fm.fill_old();
      run_case(5_000_000);
      check(err == 4'd0 && !was_active && !reboot, "nothing happens");
      check(dbg_state == 5'd22, "settles quietly");

      $display("-- 6. a flash that drops a bit");
      build_card(1, "R2  ", 0);
      u_fm.fill_old();
      u_fm.stuck_bit = 1;
      run_case(5_000_000);
      u_fm.stuck_bit = 0;
      check(err == 4'd9, "refused on verify (E_VERIFY)");
      check(!active && !reboot, "drive given back, no reboot");
      check(u_fm.golden_ok(0), "the golden image is untouched");

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
   reg [7:0] cmd, sr, outb;
   reg [23:0] a;
   integer    nbit, nbyte, over_golden, busy_until, stuck_bit;
   reg        wel, busy;
   reg [7:0]  inb;

   initial begin
      miso = 1'b1; wel = 0; busy = 0; over_golden = 0; stuck_bit = 0;
   end

   task fill_old;
      integer n;
      begin
         for (n = 0; n < BYTES; n = n + 1) mem[n] = (n >= GOLDEN) ? 8'h60 + (n & 15) : 8'hC3;
         over_golden = 0;
      end
   endtask

   function golden_ok(input integer dummy);
      integer n;
      begin
         golden_ok = 1;
         for (n = GOLDEN; n < BYTES; n = n + 1)
           if (mem[n] !== (8'h60 + (n & 15))) golden_ok = 0;
      end
   endfunction

   always @(negedge csn) begin nbit = 0; nbyte = 0; end

   always @(posedge csn) begin
      if (cmd == 8'h06) wel = 1;
      if ((cmd == 8'hD8 || cmd == 8'h02) && wel) begin
         wel = 0; busy = 1;
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
