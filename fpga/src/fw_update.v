// fw_update: rewrite the configuration flash from a file on the SD card,
// then reboot into it. r2 only: it needs the flash pins and the FPGA's
// own line to PROGRAMN.
//
// THE FLASH HOLDS TWO IMAGES. The primary at address 0 is the only one
// this ever writes. A golden copy sits at GOLDEN_ADDR and a jump record
// in the flash's last 256 bytes points at it. If the primary fails to
// configure (a write that lost power halfway, say), the ECP5 falls back
// to the golden image on its own (Lattice FPGA-TN-02203). Every image
// carries this updater, so the golden one can finish the job.
//
// THE FILE. NAME in the root of the card: a 512-byte header, then the
// bytes to write at flash address 0. `make r2-update` builds it. The
// header is little-endian:
//
//    0  "PEFW"
//    4  BOARD, so a file for some other board is refused
//    8  payload length, a multiple of 512, at most GOLDEN_ADDR
//   12  CRC-32 of the payload (the zlib one)
//   16  the payload's BUILD_ID
//
// WHEN IT RUNS. Once, after the filesystem is up and the drive has first
// gone idle (top decides what idle means), so it never slows a boot. If
// there is no file, or its BUILD_ID isn't newer than the one we're running,
// it stops there. BUILD_ID only goes up (minutes since 2026 with the top bit
// set, from the Makefile; the top bit puts it above the older
// month-day-hour-minute IDs), so an older file left on the card is ignored,
// and the new build ignores its own file after the reboot, which is what
// keeps an update from looping. Going back to an older build takes JTAG.
// Otherwise it runs two passes in the background, as the SD card's
// lowest-priority user, while the drive keeps serving. The first reads the
// whole payload and checks its CRC, and touches nothing. The second erases,
// programs and reads back each sector. Then it waits until the drive has
// stayed idle for REBOOT_HOLD more and pulls PROGRAMN. If the host never
// goes quiet for that long, the new image waits for the next power-up.
//
// WHAT'S IN EACH SLOT. Every bitstream carries a comment in its header, put
// there by the Makefile: "PEV:<BUILD_ID in hex>". (Builds from before that
// had ":<version>" after it, which is ignored.) The header starts after the
// slot's 256 bytes of $FF, as $FF $00 and then the comment, so it sits at a
// fixed place. At reset, before anything else, we read both slots' headers.
// The build IDs go to the menu, and whichever slot carries our own BUILD_ID
// is the one we were loaded from (neither means a JTAG load into SRAM). The
// primary is read again after it's rewritten, so the menu shows what's
// waiting for the reboot; the slot we're running from doesn't change.
//
// A failure before the second pass leaves the flash as it was. A
// failure during it leaves the primary half written: the running image
// is unaffected, and the next power-up falls back to the golden image,
// which finds the same file and tries again.

`default_nettype none

module fw_update #(
                   parameter integer CLK_HZ      = 50_000_000,
                   parameter [31:0]  BUILD_ID    = 32'h0,
                   parameter [31:0]  BOARD       = "R2  ",
                   parameter [87:0]  NAME        = "R2UPDATEBIN",
                   parameter [23:0]  GOLDEN_ADDR = 24'h100000,
                   parameter [23:0]  FLASH_ID    = 24'hEF4018,
                   // How long idle has to stay high, on top of whatever
                   // top needed to raise it, before the reboot. A
                   // testbench shortens it.
                   parameter integer REBOOT_HOLD = CLK_HZ * 4
                   ) (
                      input  wire        clk,
                      input  wire        rst,

                      // The filesystem's geometry, valid once start is high.
                      input  wire        start,
                      input  wire        idle,       // the drive is idle
                      input  wire [31:0] root_clus,
                      input  wire [31:0] fat_start,
                      input  wire [31:0] data_start,
                      input  wire [3:0]  spc_log,

                      // SD request face (sd_arb's updater face).
                      output reg         req,
                      output reg  [31:0] lba,
                      input  wire        fire,
                      input  wire        sd_done,
                      input  wire        sd_err,
                      input  wire        sd_buf_we,
                      input  wire [7:0]  sd_buf_wdata,
                      input  wire [9:0]  sd_buf_addr,

                      output reg         running,    // an update is under way
                      output reg         reboot,     // pull PROGRAMN low
                      output reg  [3:0]  err,
                      output reg  [15:0] progress,   // payload sectors done
                      output reg  [15:0] total,      // payload sectors in all
                      output reg  [1:0]  phase,      // 0 idle, 1 check, 2 write, 3 done
                      output reg  [31:0] file_build,
                      output wire [4:0]  dbg_state,

                      // The two slots' build IDs as eight hex digits in
                      // ASCII, from the top byte down; "EMPTY" for an erased
                      // slot, "UNKNOWN" for one with no ID in its header.
                      output reg  [63:0] pri_id,
                      output reg  [63:0] gld_id,
                      // where we were loaded from: "P", "G", or "J" for
                      // neither (a JTAG load); $00 until the headers are read
                      output reg  [7:0]  run_slot,
                      // the file cursor, for the JTAG debug register in top:
                      // {ret, fat_ret, sec, cur_clus, file_clus, file_size}
                      output wire [127:0] dbg_regs,

                      output wire        flash_csn,
                      output wire        flash_sck,
                      output wire        flash_mosi,
                      input  wire        flash_miso
                      );

   localparam [3:0]
                   E_NONE    = 4'd0,
                   E_SD      = 4'd1,   // an SD read failed
                   E_HEADER  = 4'd2,   // not "PEFW", or a bad length
                   E_BOARD   = 4'd3,   // the file is for another board
                   E_CRC     = 4'd4,   // the payload doesn't match its CRC
                   E_SHORT   = 4'd5,   // the file ends before the payload does
                   E_FLASHID = 4'd6,   // the flash isn't the W25Q128 we expect
                   E_ERASE   = 4'd7,   // an erase timed out
                   E_PROG    = 4'd8,   // a page program timed out
                   E_VERIFY  = 4'd9;   // a sector read back differently

   localparam [2:0] OP_ID = 3'd1, OP_ERASE = 3'd2, OP_PROG = 3'd3, OP_READ = 3'd4;

   localparam [4:0]
                   S_IDLE    = 5'd0,
                   S_RD      = 5'd1,  S_RD_W    = 5'd2,
                   S_DIR     = 5'd3,  S_DIR_SC  = 5'd4,  S_DIR_NXT = 5'd5,
                   S_FAT     = 5'd6,  S_FAT_GET = 5'd7,
                   S_HDR_SC  = 5'd8,  S_HDR_CHK = 5'd9,
                   S_P1      = 5'd10, S_P1_CRC  = 5'd11,
                   S_ID_W    = 5'd12,
                   S_P2      = 5'd13, S_ERASE_W = 5'd14, S_NEXT    = 5'd15,
                   S_P2_PROG = 5'd16, S_PROG_W  = 5'd17, S_VER_W   = 5'd18,
                   S_DONE    = 5'd19, S_REBOOT  = 5'd20,
                   S_FAIL    = 5'd21, S_QUIET   = 5'd22,
                   S_PROBE   = 5'd23, S_PROBE_W = 5'd24,
                   S_FAT_USE = 5'd25;

   reg [4:0] st, ret, fat_ret;
   assign dbg_state = st;

   // ---- one sector of SD data ---------------------------------------------
   // Filled straight from the SD stream; read back one byte a clock, with q
   // a clock behind ra.
   reg [7:0] sbuf [0:511];
   reg [8:0] ra;
   reg [7:0] q;
   always @(posedge clk) begin
      if (sd_buf_we) sbuf[sd_buf_addr[8:0]] <= sd_buf_wdata;
      q <= sbuf[ra];
   end

   // ---- the flash ----------------------------------------------------------
   reg  [2:0]  f_op;
   reg         f_go;
   reg  [23:0] f_addr;
   reg  [9:0]  f_len;
   wire        f_busy, f_done, f_ok, f_rd_stb;
   wire [8:0]  f_idx;
   wire [7:0]  f_rdata;
   wire [23:0] f_jedec;
   reg         f_page;   // which half of sbuf is being programmed

   spi_flash #(.CLK_HZ(CLK_HZ)) u_flash (
                                         .clk(clk), .rst(rst),
                                         .op(f_op), .go(f_go), .addr(f_addr), .len(f_len),
                                         .busy(f_busy), .done(f_done), .ok(f_ok),
                                         .idx(f_idx), .wdata(q),
                                         .rd_stb(f_rd_stb), .rdata(f_rdata), .jedec(f_jedec),
                                         .csn(flash_csn), .sck(flash_sck), .mosi(flash_mosi),
                                         .miso(flash_miso));

   // ---- CRC-32 (zlib's), one byte at a time --------------------------------
   function [31:0] crc_byte(input [31:0] c, input [7:0] b);
      integer i;
      reg [31:0] x;
      begin
         x = c ^ {24'd0, b};
         for (i = 0; i < 8; i = i + 1)
           x = x[0] ? ((x >> 1) ^ 32'hEDB88320) : (x >> 1);
         crc_byte = x;
      end
   endfunction

   function [7:0] name_byte(input [3:0] i);
      name_byte = NAME[87 - 8*i -: 8];
   endfunction

   function [31:0] clus_lba(input [31:0] c);
      clus_lba = data_start + ((c - 32'd2) << spc_log);
   endfunction

   wire [8:0] spc = 9'd1 << spc_log;

   // ---- the slot headers ----------------------------------------------------
   // What a slot's header has to start with, and our BUILD_ID as the eight
   // hex digits it appears as there.
   function [7:0] hex_digit(input [3:0] n);
      hex_digit = (n < 4'd10) ? "0" + n : "A" + n - 4'd10;
   endfunction

   function [7:0] want_byte(input [4:0] i);
      case (i)
        5'd0:  want_byte = 8'hFF;
        5'd1:  want_byte = 8'h00;
        5'd2:  want_byte = "P";
        5'd3:  want_byte = "E";
        5'd4:  want_byte = "V";
        5'd5:  want_byte = ":";
        default: want_byte = hex_digit(BUILD_ID[31 - 4*(i - 5'd6) -: 4]);
      endcase
   endfunction

   reg        pr_slot;        // 0 the primary, 1 the golden copy
   reg        pr_again;       // a re-read after the write: back to S_DONE
   reg [4:0]  pr_i;           // header byte index
   reg        pr_hdr, pr_ours, pr_empty;
   reg [63:0] pr_id;
   wire [63:0] pr_result = pr_hdr   ? pr_id
               : pr_empty ? {"EMPTY", 24'd0}
               : {"UNKNOWN", 8'd0};

   // ---- state ---------------------------------------------------------------
   reg [31:0] cur_clus;        // the file cursor: this cluster...
   reg [8:0]  sec;             // ...and this sector of it
   reg [31:0] file_clus, file_size;
   reg [27:0] fat_val;         // a FAT entry as it arrives

   assign dbg_regs = {3'd0, ret, 3'd0, fat_ret, 7'd0, sec, cur_clus, file_clus, file_size};

   reg [31:0] hdr_magic, hdr_board, hdr_len, hdr_crc, crc;
   reg [15:0] k;               // payload sector index
   reg        found, ent_end, started, match;

   // sbuf scans: ra walks 0..last while q and jq trail it by a clock
   reg [9:0]  j;
   reg [8:0]  jq;
   reg        scan_v;
   reg        name_ok;
   reg [7:0]  ent_first, ent_attr;
   reg [31:0] ent_clus, ent_size;
   reg [31:0] wait_cnt;

   always @(posedge clk) begin
      f_go   <= 1'b0;
      jq     <= ra;
      scan_v <= 1'b0;
      if (rst) begin
         st <= S_PROBE; req <= 1'b0; running <= 1'b0; reboot <= 1'b0;
         err <= E_NONE; progress <= 16'd0; total <= 16'd0; phase <= 2'd0;
         started <= 1'b0; file_build <= 32'd0; ra <= 9'd0; j <= 10'd0;
         pr_slot <= 1'b0; pr_again <= 1'b0; run_slot <= 8'h00;
         pri_id <= 64'd0; gld_id <= 64'd0;
      end else begin
         case (st)
           // ---- read a slot's header ---------------------------------------
           S_PROBE: begin
              f_op <= OP_READ; f_addr <= (pr_slot ? GOLDEN_ADDR : 24'd0) + 24'h100;
              f_len <= 10'd14; f_go <= 1'b1;
              pr_i <= 5'd0; pr_hdr <= 1'b1; pr_ours <= 1'b1; pr_empty <= 1'b1;
              pr_id <= 64'd0;
              st <= S_PROBE_W;
           end
           S_PROBE_W: begin
              if (f_rd_stb) begin
                 pr_i <= pr_i + 1'b1;
                 if (pr_i < 5'd2 && f_rdata != 8'hFF) pr_empty <= 1'b0;
                 if (pr_i < 5'd6) begin
                    if (f_rdata != want_byte(pr_i)) pr_hdr <= 1'b0;
                 end else begin
                    // the build ID's eight digits, kept as text for the menu
                    if (f_rdata != want_byte(pr_i)) pr_ours <= 1'b0;
                    pr_id[63 - 8*(pr_i - 5'd6) -: 8] <= f_rdata;
                 end
              end
              if (f_done) begin
                 if (!pr_slot) pri_id <= pr_result;
                 else          gld_id <= pr_result;
                 if (pr_again) begin
                    st <= S_DONE;
                 end else if (!pr_slot) begin
                    if (pr_hdr && pr_ours) run_slot <= "P";
                    pr_slot <= 1'b1; st <= S_PROBE;
                 end else begin
                    if (run_slot == 8'h00) run_slot <= (pr_hdr && pr_ours) ? "G" : "J";
                    st <= S_IDLE;
                 end
              end
           end

           S_IDLE: if (start && idle && !started) begin
              started  <= 1'b1;
              cur_clus <= root_clus; sec <= 9'd0; found <= 1'b0;
              lba <= clus_lba(root_clus); ret <= S_DIR; st <= S_RD;
           end

           // ---- read the sector at lba into sbuf, then go to ret ----------
           S_RD: begin req <= 1'b1; st <= S_RD_W; end
           S_RD_W: begin
              if (fire) req <= 1'b0;
              if (sd_done) begin
                 j <= 10'd0; ra <= 9'd0;
                 if (sd_err) begin err <= E_SD; st <= S_FAIL; end
                 else st <= ret;
              end
           end

           // ---- the root directory, a sector at a time ---------------------
           S_DIR: begin ent_end <= 1'b0; st <= S_DIR_SC; end
           S_DIR_SC: begin
              if (!j[9]) begin ra <= j[8:0]; j <= j + 1'b1; end
              if (j != 10'd0) scan_v <= 1'b1;
              if (scan_v) begin
                 case (jq[4:0])
                   5'd0:  begin ent_first <= q; name_ok <= (q == name_byte(4'd0)); end
                   5'd11: ent_attr <= q;
                   5'd20: ent_clus[23:16] <= q;
                   5'd21: ent_clus[31:24] <= q;
                   5'd26: ent_clus[7:0]   <= q;
                   5'd27: ent_clus[15:8]  <= q;
                   5'd28: ent_size[7:0]   <= q;
                   5'd29: ent_size[15:8]  <= q;
                   5'd30: ent_size[23:16] <= q;
                   5'd31: ent_size[31:24] <= q;
                   default: if (jq[4:0] < 5'd11 && q != name_byte(jq[3:0])) name_ok <= 1'b0;
                 endcase
                 if (jq[4:0] == 5'd31 && !found && !ent_end) begin
                    // 00 ends the directory, E5 is a deleted entry, attr 0F
                    // is a piece of a long name, and 10 and 08 are a
                    // directory and the volume label
                    if (ent_first == 8'h00) begin
                       ent_end <= 1'b1;
                    end else if (ent_first != 8'hE5 && ent_attr != 8'h0F
                                 && (ent_attr & 8'h18) == 8'h00 && name_ok) begin
                       found     <= 1'b1;
                       file_clus <= {4'd0, ent_clus[27:0]};
                       file_size <= {q, ent_size[23:0]};
                    end
                 end
                 if (jq == 9'd511) st <= S_DIR_NXT;
              end
           end
           S_DIR_NXT: begin
              if (found) begin
                 cur_clus <= file_clus; sec <= 9'd0;
                 lba <= clus_lba(file_clus);
                 ret <= S_HDR_SC; st <= S_RD;
              end else if (ent_end) begin
                 st <= S_QUIET;                       // no update file
              end else if (sec + 1'b1 < spc) begin
                 sec <= sec + 1'b1; lba <= lba + 1'b1; ret <= S_DIR; st <= S_RD;
              end else begin
                 fat_ret <= S_DIR; st <= S_FAT;         // the directory's next cluster
              end
           end

           // ---- follow the FAT from cur_clus, read sector 0 of the next
           // cluster, and go to fat_ret. The end of the chain means no file
           // for a directory and a short file anywhere else. --------------
           S_FAT: begin
              lba <= fat_start + (cur_clus >> 7);
              ret <= S_FAT_GET; st <= S_RD;
           end
           S_FAT_GET: begin
              // Four bytes, little-endian, at (cur_clus & 127) * 4. They
              // gather in fat_val, not cur_clus: ra is still being worked
              // out from cur_clus while they arrive, and overwriting it
              // early points the last byte at some other entry.
              j  <= j + 1'b1;
              ra <= {cur_clus[6:0], 2'b00} + j[1:0];
              case (j)
                10'd2: fat_val[7:0]   <= q;
                10'd3: fat_val[15:8]  <= q;
                10'd4: fat_val[23:16] <= q;
                10'd5: begin fat_val[27:24] <= q[3:0]; st <= S_FAT_USE; end
                default: ;
              endcase
           end
           // A clock of its own: straight off the block RAM into the
           // compare and clus_lba's subtract, shift and add is too long a
           // path for 50 MHz.
           S_FAT_USE: begin
              if (fat_val >= 28'hFFFFFF8 || fat_val < 28'd2) begin
                 if (fat_ret == S_DIR) st <= S_QUIET;
                 else begin err <= E_SHORT; st <= S_FAIL; end
              end else begin
                 cur_clus <= {4'd0, fat_val}; sec <= 9'd0;
                 lba <= clus_lba({4'd0, fat_val});
                 ret <= fat_ret; st <= S_RD;
              end
           end

           // ---- the header --------------------------------------------------
           S_HDR_SC: begin
              if (j < 10'd20) begin ra <= j[8:0]; j <= j + 1'b1; end
              if (j != 10'd0) scan_v <= 1'b1;
              if (scan_v) begin
                 case (jq[4:2])
                   3'd0: hdr_magic  <= {q, hdr_magic[31:8]};
                   3'd1: hdr_board  <= {q, hdr_board[31:8]};
                   3'd2: hdr_len    <= {q, hdr_len[31:8]};
                   3'd3: hdr_crc    <= {q, hdr_crc[31:8]};
                   3'd4: file_build <= {q, file_build[31:8]};
                   default: ;
                 endcase
                 if (jq == 9'd19) st <= S_HDR_CHK;
              end
           end
           S_HDR_CHK: begin
              // the bytes arrive first to last, so "PEFW" reads back as "WFEP"
              if (hdr_magic != "WFEP") begin
                 err <= E_HEADER; st <= S_FAIL;
              end else if (hdr_board != {BOARD[7:0], BOARD[15:8], BOARD[23:16], BOARD[31:24]}) begin
                 err <= E_BOARD; st <= S_FAIL;
              end else if (hdr_len == 32'd0 || hdr_len[8:0] != 9'd0
                           || hdr_len > {8'd0, GOLDEN_ADDR}) begin
                 err <= E_HEADER; st <= S_FAIL;
              end else if (file_size < hdr_len + 32'd512) begin
                 err <= E_SHORT; st <= S_FAIL;
              end else if (file_build <= BUILD_ID) begin
                 st <= S_QUIET;                       // not newer than this one
              end else begin
                 running <= 1'b1; phase <= 2'd1;
                 total <= hdr_len[24:9]; k <= 16'd0; progress <= 16'd0;
                 crc <= 32'hFFFFFFFF;
                 st <= S_P1;
              end
           end

           // ---- pass 1: read the whole payload and check its CRC ------------
           // The cursor starts on the header sector; each step moves it one
           // sector along and reads.
           S_P1: begin
              if (k == total) begin
                 if ((crc ^ 32'hFFFFFFFF) != hdr_crc) begin
                    err <= E_CRC; st <= S_FAIL;
                 end else begin
                    phase <= 2'd2; k <= 16'd0; progress <= 16'd0;
                    f_op <= OP_ID; f_go <= 1'b1; st <= S_ID_W;
                 end
              end else if (sec + 1'b1 < spc) begin
                 sec <= sec + 1'b1; lba <= lba + 1'b1; ret <= S_P1_CRC; st <= S_RD;
              end else begin
                 fat_ret <= S_P1_CRC; st <= S_FAT;
              end
           end
           S_P1_CRC: begin
              if (!j[9]) begin ra <= j[8:0]; j <= j + 1'b1; end
              if (j != 10'd0) scan_v <= 1'b1;
              if (scan_v) begin
                 crc <= crc_byte(crc, q);
                 if (jq == 9'd511) begin
                    k <= k + 1'b1; progress <= k + 1'b1; st <= S_P1;
                 end
              end
           end

           // ---- the flash's ID, before anything is erased -------------------
           S_ID_W: if (f_done) begin
              if (f_jedec != FLASH_ID) begin
                 err <= E_FLASHID; st <= S_FAIL;
              end else begin
                 cur_clus <= file_clus; sec <= 9'd0;   // back to the header
                 lba <= clus_lba(file_clus);
                 st <= S_P2;
              end
           end

           // ---- pass 2: erase, program and verify, a sector at a time -------
           S_P2: begin
              if (k == total) begin
                 // read the new primary's build ID, then wait for the reboot
                 phase <= 2'd3; wait_cnt <= 32'd0;
                 pr_slot <= 1'b0; pr_again <= 1'b1; st <= S_PROBE;
              end else if (k[6:0] == 7'd0) begin
                 // the first sector of a 64 KB block erases the block
                 f_op <= OP_ERASE; f_addr <= {k[14:7], 16'd0}; f_go <= 1'b1;
                 st <= S_ERASE_W;
              end else begin
                 st <= S_NEXT;
              end
           end
           S_ERASE_W: if (f_done) begin
              if (!f_ok) begin err <= E_ERASE; st <= S_FAIL; end
              else st <= S_NEXT;
           end
           S_NEXT: begin
              if (sec + 1'b1 < spc) begin
                 sec <= sec + 1'b1; lba <= lba + 1'b1; ret <= S_P2_PROG; st <= S_RD;
              end else begin
                 fat_ret <= S_P2_PROG; st <= S_FAT;
              end
           end
           S_P2_PROG: begin
              // the first 256 bytes; S_PROG_W starts the second half
              f_page <= 1'b0;
              f_op <= OP_PROG; f_addr <= {k[14:0], 9'd0}; f_len <= 10'd256; f_go <= 1'b1;
              st <= S_PROG_W;
           end
           S_PROG_W: begin
              ra <= {f_page, f_idx[7:0]};
              if (f_done) begin
                 if (!f_ok) begin
                    err <= E_PROG; st <= S_FAIL;
                 end else if (!f_page) begin
                    f_page <= 1'b1;
                    f_op <= OP_PROG; f_addr <= {k[14:0], 9'd256}; f_go <= 1'b1;
                 end else begin
                    // read the whole sector back and compare
                    j <= 10'd0; ra <= 9'd0; match <= 1'b1;
                    f_op <= OP_READ; f_addr <= {k[14:0], 9'd0}; f_len <= 10'd512; f_go <= 1'b1;
                    st <= S_VER_W;
                 end
              end
           end
           S_VER_W: begin
              if (f_rd_stb) begin
                 if (f_rdata != q) match <= 1'b0;
                 j <= j + 1'b1; ra <= j[8:0] + 1'b1;
              end
              if (f_done) begin
                 if (!match) begin
                    err <= E_VERIFY; st <= S_FAIL;
                 end else begin
                    k <= k + 1'b1; progress <= k + 1'b1; st <= S_P2;
                 end
              end
           end

           // Any host activity starts the count over.
           S_DONE: begin
              wait_cnt <= idle ? wait_cnt + 1'b1 : 32'd0;
              if (wait_cnt >= REBOOT_HOLD) begin reboot <= 1'b1; st <= S_REBOOT; end
           end
           S_REBOOT: reboot <= 1'b1;                  // held until PROGRAMN takes us down

           S_FAIL:  begin req <= 1'b0; running <= 1'b0; end
           S_QUIET: req <= 1'b0;
           default: st <= S_IDLE;
         endcase
      end
   end

endmodule

`default_nettype wire
