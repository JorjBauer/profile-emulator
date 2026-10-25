// tb_selector: the magic blocks the boot menu is made of.
//
// The 6502 side of this lives in ../../selector and is not simulated
// here. What is checked is that the drive answers the block numbers that side
// asks for, with the bytes it expects. If any of these are wrong the //e loads
// a menu that crashes, and the failure will look like a 6502 bug.
//
// Checked, in the order the //e does it:
//
//   1. the info block ($FFFFC0) carries the $A5 signature, the image count,
//      the force-menu bit and the three build IDs,
//   2. the selector body ($FFFF40 + p) matches the blob byte for byte,
//   3. block 0 is the synthetic loader while armed and belongs to nobody once a
//      selection has been made,
//   4. SELECT ($FFFF00 + i) accepts a valid index and refuses one past the end,
//   5. the window stops at $FFFFFD, below the two real special blocks.
//
//   iverilog -g2012 -o /tmp/tb -s tb_selector sim/tb_selector.v src/selector.v

`timescale 1ns / 1ps
`default_nettype none

module tb_selector;

   reg clk = 0;
   always #5 clk = ~clk;

   reg  [23:0] blk_addr = 24'd0;
   reg         blk_rd   = 1'b0;
   reg  [9:0]  buf_addr = 10'd0;
   reg  [7:0]  img_count = 8'd3;
   reg  [7:0]  img_current = 8'd1;
   reg         force_menu = 1'b1;
   reg         armed = 1'b1;
   wire [11:0] cat_addr;
   wire [5:0]  sel_idx;
   wire        sel_stb, hit;
   wire [7:0]  rdata;

   selector dut (
                 .clk(clk), .blk_addr(blk_addr), .blk_rd(blk_rd), .buf_addr(buf_addr),
                 .img_count(img_count), .img_current(img_current),
                 .force_menu(force_menu), .armed(armed), .window_live(1'b1),
                 .boot_hit(),
                 .build_id("80068B67"), .pri_id("80068B70"), .gld_id({"EMPTY", 24'd0}),
                 .run_slot("G"),
                 .cat_addr(cat_addr), .cat_data(8'hCC),
                 .sel_idx(sel_idx), .sel_stb(sel_stb), .hit(hit), .rdata(rdata));

   // The same blobs the design serves, read independently.
   reg [7:0] boot_ref [0:511];
   reg [7:0] body_ref [0:2047];
   initial begin
      $readmemh("src/blobs/sel_boot.hex", boot_ref);
      $readmemh("src/blobs/sel_body.hex", body_ref);
   end

   integer errors = 0;
   task check(input cond, input [511:0] what);
      begin
         if (cond) $display("    ok    %0s", what);
         else begin $display("*** FAIL  %0s", what); errors = errors + 1; end
      end
   endtask

   // Present a block and a byte offset, then let the registered read settle.
   task fetch(input [23:0] blk, input [9:0] off, output [7:0] b);
      begin
         blk_addr = blk; buf_addr = off;
         @(posedge clk); @(posedge clk); @(posedge clk);
         b = rdata;
      end
   endtask

   reg [7:0] b;
   integer i, bad;
   // the build IDs as the info block should carry them, written out
   reg [63:0] want_run = {"8", "0", "0", "6", "8", "B", "6", "7"};
   reg [63:0] want_pri = {"8", "0", "0", "6", "8", "B", "7", "0"};
   reg [63:0] want_gld = {"E", "M", "P", "T", "Y", 8'h00, 8'h00, 8'h00};

   initial begin
      @(posedge clk);

      $display("-- the info block, $FFFFC0");
      blk_addr = 24'hFFFFC0;
      check(hit === 1'b1, "the info block is claimed by the selector");
      fetch(24'hFFFFC0, 10'd0, b); check(b === 8'd3,  "byte 0 is the image count");
      fetch(24'hFFFFC0, 10'd1, b); check(b === 8'd1,  "byte 1 is the current index");
      fetch(24'hFFFFC0, 10'd3, b); check(b === 8'd4,  "byte 3 is the body page count");
      fetch(24'hFFFFC0, 10'd4, b); check(b === 8'hA5,
                                         "byte 4 is $A5, which is how the 6502 knows it is talking to us");
      fetch(24'hFFFFC0, 10'd5, b); check(b[0] === 1'b1, "byte 5 bit 0 forces the menu");
      force_menu = 1'b0;
      fetch(24'hFFFFC0, 10'd5, b); check(b[0] === 1'b0, "and clears when a choice is valid");
      force_menu = 1'b1;
      bad = 0;
      for (i = 0; i < 8; i = i + 1) begin
         fetch(24'hFFFFC0, 10'd16 + i, b); if (b !== want_run[63 - 8*i -: 8]) bad = bad + 1;
         fetch(24'hFFFFC0, 10'd32 + i, b); if (b !== want_pri[63 - 8*i -: 8]) bad = bad + 1;
         fetch(24'hFFFFC0, 10'd40 + i, b); if (b !== want_gld[63 - 8*i -: 8]) bad = bad + 1;
      end
      check(bad == 0, "bytes 16, 32 and 40 are the running, primary and golden builds");
      fetch(24'hFFFFC0, 10'd24, b); check(b === "G", "byte 24 is the slot we came from");

      $display("-- the selector body, $FFFF40 + p");
      bad = 0;
      for (i = 0; i < 512; i = i + 1) begin
         fetch(24'hFFFF40, i[9:0], b);
         if (b !== body_ref[i]) bad = bad + 1;
      end
      check(bad == 0, "page 0 matches the blob byte for byte");
      bad = 0;
      for (i = 0; i < 512; i = i + 1) begin
         fetch(24'hFFFF41, i[9:0], b);
         if (b !== body_ref[512 + i]) bad = bad + 1;
      end
      check(bad == 0, "page 1 is the NEXT 512 bytes, not page 0 again");
      // Past the end of the blob the //e must get zeros, not wrapped data.
      fetch(24'hFFFF4F, 10'd0, b);
      check(b === 8'h00, "a page past the end of the body reads as zero");

      $display("-- block 0, the synthetic loader");
      blk_addr = 24'd0;
      check(hit === 1'b1, "block 0 is claimed while armed");
      bad = 0;
      for (i = 0; i < 512; i = i + 1) begin
         fetch(24'd0, i[9:0], b);
         if (b !== boot_ref[i]) bad = bad + 1;
      end
      check(bad == 0, "and serves the loader blob");
      armed = 1'b0;
      // The boot_held latch keeps the claim while blk_addr sits at 0 (that
      // is its whole purpose: the claim must outlive the arm through the
      // data phase). On real hardware the SELECT read itself moves blk_addr
      // off 0, which is what releases the hold: model that.
      blk_addr = 24'hFFFF05;
      @(posedge clk); @(posedge clk);
      blk_addr = 24'd0;
      @(posedge clk);
      check(hit === 1'b0,
            "once a selection is made block 0 belongs to the image again");
      armed = 1'b1;

      $display("-- SELECT, $FFFF00 + i");
      blk_addr = 24'hFFFF02; buf_addr = 10'd0;
      @(posedge clk); blk_rd = 1'b1; @(posedge clk); blk_rd = 1'b0;
      @(posedge clk);
      check(sel_idx === 6'd2, "the index comes from the low bits of the block");
      fetch(24'hFFFF02, 10'd0, b);
      check(b === 8'h00, "a valid index returns $00 in byte 0");
      // Three images exist, so index 5 does not.
      fetch(24'hFFFF05, 10'd0, b);
      check(b === 8'hFF, "an index past the end returns $FF");
      blk_addr = 24'hFFFF05;
      @(posedge clk); blk_rd = 1'b1; @(posedge clk); blk_rd = 1'b0;
      @(posedge clk); @(posedge clk);
      check(sel_stb === 1'b0, "and does not switch images");

      $display("-- the window's edges");
      blk_addr = 24'hFFFFFD; @(posedge clk);
      check(hit === 1'b1, "$FFFFFD is the last magic block");
      blk_addr = 24'hFFFFFE; @(posedge clk);
      check(hit === 1'b0, "$FFFFFE is the drive's RAM buffer, not ours");
      blk_addr = 24'hFFFFFF; @(posedge clk);
      check(hit === 1'b0, "$FFFFFF is the spare table, not ours");
      blk_addr = 24'h00FF40; @(posedge clk);
      check(hit === 1'b0,
            "$00FF40 is an ordinary data block: the window is $FFFFxx only");

      $display("");
      if (errors == 0) $display("PASS: the menu's blocks answer as the 6502 expects");
      else             $display("FAIL: %0d problem(s)", errors);
      $finish;
   end

endmodule

`default_nettype wire
