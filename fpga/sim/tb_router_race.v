`timescale 1ns/1ps
// The new-selection parked-read race, at the block_router seam.
//
// The sequence, exactly as a menu selection produces it:
//   1. a magic-window read (the SELECT) answers instantly: blk_ready <= 1
//   2. the resolver goes busy (fat32 rescanning the new image)
//   3. the boot blob reads block 0; the router PARKS it (pend_rd)
//
// The parking branch must clear blk_ready. profile_target's S_CONFIRM is
// level-sensitive. A stale 1 would confirm the read at once, and the data
// phase would serve the stale output mux (sel_sel still latched from the
// SELECT: zeros for block 0) instead of the image's boot block.
//
// PASS criteria: blk_ready must be LOW from the moment the block-0 read is
// parked until the replayed request actually fetches the block, and the
// fetch must go to the SD with the right LBA once the resolver returns.
module tb_router_race;
   reg clk = 0; always #10 clk = ~clk;

   reg  [15:0] claim = 16'hFE00, served = 16'hFFFF;
   reg         fs_ok = 1, fs_ready = 1, sel_hit = 0;
   reg  [23:0] blk_addr = 0;
   reg         blk_rd = 0, blk_wr = 0;
   reg         sd_done = 0;
   wire        blk_ready, sd_rd, sd_wr, out_of_range, write_refused;
   wire [31:0] sd_lba;
   wire [15:0] map_idx;

   reg window_live = 1;
   block_router #(.FS_ENABLE(1'b1)) dut (
                                         .clk(clk), .claim_blocks(claim), .served_blocks(served),
                                         .no_media(1'b0), .write_protect(1'b0), .write_refused(write_refused),
                                         .sel_hit(sel_hit), .sel_data(8'h00), .window_live(window_live),
                                         .fs_ok(fs_ok), .fs_ready(fs_ready),
                                         .fs_data_start(32'd9224), .fs_spc_log(4'd6),
                                         .fs_map_idx(map_idx), .fs_map_cluster(32'd2),
                                         .blk_addr(blk_addr), .blk_rd(blk_rd), .blk_wr(blk_wr),
                                         .blk_ready(blk_ready),
                                         .buf_addr(10'd0), .buf_rdata(), .buf_wdata(8'h00), .buf_we(1'b0),
                                         .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_lba(sd_lba),
                                         .sd_done(sd_done), .sd_err(1'b0),
                                         .ram_addr(), .ram_rdata(8'h00), .ram_wdata(), .ram_we(),
                                         .out_of_range(out_of_range));

   integer fails = 0;
   task check(input cond, input [255:0] what);
      if (!cond) begin
         fails = fails + 1;
         $display("*** FAIL %0s at t=%0t (blk_ready=%b)", what, $time, blk_ready);
      end
   endtask

   integer i;
   reg saw_low;
   initial begin
      repeat (4) @(posedge clk);

      // 1: the SELECT, a magic-window read the selector answers from ROM
      blk_addr = 24'hFFFF0B; sel_hit = 1;
      blk_rd <= 1; @(posedge clk); blk_rd <= 0;
      @(posedge clk); @(posedge clk);
      check(blk_ready === 1'b1, "select answered ready");

      // 2: the resolver goes busy (fat32 reselect after sel_stb)
      fs_ready = 0; fs_ok = 0; sel_hit = 0;
      @(posedge clk);

      // 3: the blob reads block 0 while the resolver is busy
      blk_addr = 24'h000000;
      blk_rd <= 1; @(posedge clk); blk_rd <= 0;
      @(posedge clk); @(posedge clk);
      check(blk_ready === 1'b0,
            "parked read must NOT look ready (the zero-serve race)");

      // ready must STAY low the whole busy window
      saw_low = 1;
      for (i = 0; i < 50; i = i + 1) begin
         @(posedge clk);
         if (blk_ready !== 1'b0) saw_low = 0;
      end
      check(saw_low, "ready stayed low across the busy window");

      // 4: resolver returns: the parked request must replay into an SD read
      fs_ready = 1; fs_ok = 1;
      saw_low = 0;
      for (i = 0; i < 20 && !sd_rd; i = i + 1) @(posedge clk);
      check(sd_rd === 1'b1, "parked read replayed to the SD");
      check(sd_lba === 32'd9224, "replayed read maps block 0 to the image LBA");
      check(blk_ready === 1'b0, "still not ready until the SD fetch lands");

      // 5: the fetch completes; only NOW may ready rise
      sd_done <= 1; @(posedge clk); sd_done <= 0;   // nonblocking: the clear must not race the DUT sample
      @(posedge clk); @(posedge clk);
      check(blk_ready === 1'b1, "ready after the data actually arrived");

      // ---- GEOMETRY OVERLOADING: the closed window serves the top blocks ---
      // 6: window closed, a read of $FFFF42 is image block $00FF42
      window_live = 0;
      blk_addr = 24'hFFFF42;
      blk_rd <= 1; @(posedge clk); blk_rd <= 0;
      for (i = 0; i < 20 && !sd_rd; i = i + 1) @(posedge clk);
      check(sd_rd === 1'b1, "closed-window read went to the SD");
      // block $FF42: map_idx = $FF42>>6 = $3FD, sector-in-cluster = 2, and the
      // stub chain answers cluster 2 for every index: 9224 + 0 + 2
      check(sd_lba === 32'd9226, "aliased to the image's top range");
      check(out_of_range === 1'b0, "top block is in the served range");
      sd_done <= 1; @(posedge clk); sd_done <= 0;
      @(posedge clk); @(posedge clk);
      check(blk_ready === 1'b1, "aliased read completed");

      // 7: the spare table is permanent, window open or closed
      blk_addr = 24'hFFFFFF;
      blk_rd <= 1; @(posedge clk); blk_rd <= 0;
      @(posedge clk); @(posedge clk);
      check(blk_ready === 1'b1, "spare table answers with the window closed");
      check(sd_rd === 1'b0, "spare table never touches the SD");

      // 8: window open, the same $FFFF42 is magic again (no SD, no range error)
      window_live = 1;
      blk_addr = 24'hFFFF42;
      blk_rd <= 1; @(posedge clk); blk_rd <= 0;
      @(posedge clk); @(posedge clk);
      check(blk_ready === 1'b1, "open-window magic answers immediately");
      check(sd_rd === 1'b0, "open-window magic never touches the SD");
      check(out_of_range === 1'b0, "open-window magic is not a range error");

      // 9a: a beyond-range READ is lenient: instant zeros, no SD, no failure
      window_live = 1; served = 16'h0B40;
      blk_addr = 24'h00208A;              // the NOX probe block, verbatim
      blk_rd <= 1; @(posedge clk); blk_rd <= 0;
      @(posedge clk); @(posedge clk);
      check(blk_ready === 1'b1, "beyond-range read answers instantly");
      check(sd_rd === 1'b0, "beyond-range read never touches the SD");
      check(out_of_range === 1'b0, "beyond-range READ is not a failure");
      // 9b: a beyond-range WRITE is still refused (one-cycle pulse: latch it)
      saw_low = 0;
      blk_wr <= 1; @(posedge clk); blk_wr <= 0;
      for (i = 0; i < 4; i = i + 1) begin
         @(posedge clk);
         if (write_refused === 1'b1) saw_low = 1;
      end
      check(saw_low === 1, "beyond-range WRITE is refused");
      served = 16'hFFFF;

      // 9: a normal-size image never aliases: scratch stays scratch even
      // with the window closed
      window_live = 0; served = 16'h0080;
      blk_addr = 24'hFFFFFE;
      blk_rd <= 1; @(posedge clk); blk_rd <= 0;
      @(posedge clk); @(posedge clk);
      check(blk_ready === 1'b1, "small image: scratch answers instantly");
      check(sd_rd === 1'b0, "small image: scratch never touches the SD");
      served = 16'hFFFF; window_live = 1;

      if (fails == 0)
        $display("PASS: a parked read never looks ready before its data exists");
      else
        $display("FAIL: %0d problem(s)", fails);
      $finish;
   end

   always @(posedge clk)
     if ($time > 1200)
       $display("t=%0t rdy=%b sd_rd=%b sd_done=%b look=%b pend=%b lba=%0d",
                $time, blk_ready, sd_rd, sd_done, dut.look, dut.pend_rd, sd_lba);

   initial begin #100000; $display("TIMEOUT"); $finish; end
endmodule
