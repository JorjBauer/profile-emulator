// selector: the boot-time image menu, drive side.
//
// The //e's interface card ROM (the ROM this was developed against, 341-0299-B
// in a //e; other ROMs may differ) boots by reading block 0 into $0800 and
// jumping to $0801, so whoever supplies block 0 supplies the boot program.
// Before a selection has been made this supplies a synthetic loader instead of
// the image's own boot block. That loader pulls in the menu through a window
// of magic block numbers. After a choice is made the real boot block is
// served and chained to, with no reboot.
//
// See ../../selector/SELECTOR.md for the 6502 side. The two blobs served
// here are BUILT BY THAT PROJECT: tools/import_blobs.js generates
// src/blobs/sel_boot.hex and sel_body.hex from its selector_blobs.h at build
// time. Nothing here interprets them.
//
// THE WINDOW. The card ROM maps a 16-bit ProDOS block of $FFxx onto ProFile
// block $FFFFxx, not $00FFxx: it tests $47 against $FF at $CB2D and treats
// that as the drive's high special range. So the //e asks for $FFxx and this
// sees $FFFFxx. The window stops at $FFFFFD, just below the two real special
// blocks, and a volume tops out at $FFFC blocks, so it can never collide with
// data.
//
//   $FFFF00 + i   SELECT image i (0..63). Returns a status block.
//   $FFFF40 + p   selector body, page p
//   $FFFF80 + p   catalog page p, eight 64-byte records
//   $FFFFC0       info block
//
// On a real ProFile every one of these is out of range: the range check at
// $0306 sets an error bit before any seek, so nothing moves and nothing is
// written. And a real drive serves its own block 0, so the menu never runs.

`default_nettype none

module selector #(
                  parameter integer MAX_IMAGES = 64  // UNUSED: the SELECT bound is img_count; nothing reads this
                  ) (
                     input  wire        clk,

                     // the block being served, and which byte of it
                     input  wire [23:0] blk_addr,
                     input  wire        blk_rd,
                     input  wire [9:0]  buf_addr,

                     // state the menu displays
                     input  wire [7:0]  img_count,
                     input  wire [7:0]  img_current,
                     input  wire        force_menu,   // no valid saved selection
                     input  wire        armed,        // no selection made yet this power-up
                     // The $FFFF00-FD window belongs to the menu ONLY while the
                     // menu phase is live (no selection made since the last
                     // arm). Once an OS runs, the card driver maps ProDOS
                     // blocks $FFxx to $FFFFxx on the wire. A 100%-full
                     // 65535-block volume (Total Replay) stores real data
                     // there, and the router serves those as image blocks when
                     // this is low.
                     input  wire        window_live,

                     // catalog RAM, filled by the scanner
                     output wire [11:0] cat_addr,
                     input  wire [7:0]  cat_data,

                     // a SELECT arrived
                     output reg  [5:0]  sel_idx,
                     output reg         sel_stb,

                     output wire        hit,          // this block belongs to the selector
                     // The blob's block-0 claim, exported so top can OPEN the
                     // window on it. Only a boot produces the
                     // status-then-block-0 sequence, so this is the only safe
                     // "menu phase begins" signal: a runtime STATUS call
                     // re-arms sel_armed (by design, so the NEXT boot gets the
                     // menu) but must not reopen the window over a running OS's
                     // top blocks.
                     output wire        boot_hit,
                     output reg  [7:0]  rdata         // registered, to match the block buffer
                     );

   // ---- the window ---------------------------------------------------------
   wire in_win  = (blk_addr[23:8] == 16'hFFFF) && (blk_addr[7:0] <= 8'hFD);
   wire is_sel  = in_win && (blk_addr[7:6] == 2'b00);
   wire is_body = in_win && (blk_addr[7:6] == 2'b01);
   wire is_cat  = in_win && (blk_addr[7:6] == 2'b10);
   wire is_info = in_win && (blk_addr[7:0] == 8'hC0);
   // The boot blob, readable THROUGH the window. At boot the card ROM plants
   // it at $0800; the runtime selector client (the A2E-check knock) fetches
   // this block to $0800 itself and jumps to it, reusing the whole menu.
   wire is_bootpg = in_win && (blk_addr[7:0] == 8'hC1);

   // Block 0 is ours only while armed AND only as part of a boot: see want_boot
   // in block_router, which qualifies it with "the previous block read was the
   // spare table". Reading block 0 at any other time gets the image's own boot
   // block, which is what a running system expects. The claim must OUTLIVE the
   // arm. `armed` includes prev_blk == $FFFFFF, and prev_blk is updated by the
   // very block-0 read being served: the claim is true at request time (the
   // router latches sel_hit from it) but false by the data phase, when this mux
   // is consulted per byte. Without the latch the else-clause serves $00 for
   // all 512 bytes and the //e reads a zeroed boot block. So latch the claim
   // while block 0 is the address; any other address ends it.
   reg  boot_held = 1'b0;   // no rst port here; explicit power-on value
   always @(posedge clk) begin
      if (blk_addr == 24'd0 && armed) boot_held <= 1'b1;
      else if (blk_addr != 24'd0)     boot_held <= 1'b0;
   end
   wire is_boot = (blk_addr == 24'd0) && (armed || boot_held);

   assign hit = (in_win && window_live) || is_boot;
   assign boot_hit = is_boot;

   // ---- the blobs, built by ../selector ------------------------------------
   reg [7:0] boot_rom [0:511];
   reg [7:0] body_rom [0:2047];
   initial begin
      $readmemh("src/blobs/sel_boot.hex", boot_rom);
      $readmemh("src/blobs/sel_body.hex", body_rom);
   end

   wire [1:0]  body_page = blk_addr[1:0];
   wire        body_ok   = (blk_addr[5:2] == 4'd0);   // only pages 0 to 3 exist
   wire [10:0] body_a    = {body_page, buf_addr[8:0]};

   // A catalog page is a 512-byte block and the store holds eight of them, so
   // the address is 3 page bits and NINE byte bits: buf_addr[7:0] alone would
   // wrap every page at 256 bytes.
   assign cat_addr = {blk_addr[2:0], buf_addr[8:0]};

   // ---- the info block -----------------------------------------------------
   // Layout from SELECTOR.md. $A5 is how the 6502 side tells it is talking to
   // an emulator at all, and bit 0 of byte 5 is what makes the menu appear
   // whether or not open-apple is held.
   function [7:0] info_byte(input [9:0] a);
      case (a)
        10'd0: info_byte = img_count;
        10'd1: info_byte = img_current;
        10'd2: info_byte = 8'd8;                 // catalog pages
        10'd3: info_byte = 8'd4;                 // body pages
        10'd4: info_byte = 8'hA5;
        10'd5: info_byte = {7'b0, force_menu};
        default: info_byte = 8'h00;
      endcase
   endfunction

   reg [7:0] boot_q, body_q;
   always @(posedge clk) begin
      boot_q <= boot_rom[buf_addr[8:0]];
      body_q <= body_ok ? body_rom[body_a] : 8'h00;
   end

   // Registered, so this arrives with the same timing as the block buffer's
   // read and block_router can mux the two without a special case.
   always @(posedge clk) begin
      if (is_boot || is_bootpg) rdata <= boot_q;
      else if (is_body) rdata <= body_q;
      else if (is_cat)  rdata <= cat_data;
      else if (is_info) rdata <= info_byte(buf_addr);
      else if (is_sel)
        // The status block a SELECT returns: $00 accepted, $FF out of range.
        // The bound is img_count, the number of images actually cataloged, not
        // a fixed maximum. WIDTH MATTERS: the index is 6 bits and img_count is
        // 8 and can exceed 63, so the compare is done at 8 bits. A 6-bit
        // compare against a limit of 64 truncates that limit to zero and
        // rejects every index.
        rdata <= (buf_addr == 10'd0)
          ? (({2'b00, blk_addr[5:0]} < img_count) ? 8'h00 : 8'hFF)
            : 8'h00;
      else rdata <= 8'h00;
   end

   // ---- the SELECT itself --------------------------------------------------
   // A pulse, on the request rather than on any byte of the reply, so switching
   // images happens once however many times the host reads the status block.
   always @(posedge clk) begin
      sel_stb <= 1'b0;
      // window_live: with the window closed, $FFFF00-3F are the OS's own top
      // blocks: serving them as data is the router's job, and a data read must
      // never fire a reselect.
      if (blk_rd && is_sel && window_live
          && ({2'b00, blk_addr[5:0]} < img_count)) begin
         sel_idx <= blk_addr[5:0];
         sel_stb <= 1'b1;
      end
   end

endmodule

`default_nettype wire
