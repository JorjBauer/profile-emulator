// tb_fat32_lru: the LRU-2 chain cache. Two images on the model card; an
// ephemeral reselect between two recently-used images must switch INSTANTLY
// (no SD reads) and serve the correct, independently-fragmented chain.
//
//   image 0 "longname.img"  clusters 4 6 5 7   (idx 0, named by profile.cfg)
//   image 1 "two.img"       clusters 9 11      (idx 1)
//
//   iverilog -g2012 -o /tmp/tb -s tb_fat32_lru sim/tb_fat32_lru.v \
//       sim/sd_model.v src/fat32.v src/sd_spi.v

`timescale 1ns / 1ps
`default_nettype none

module tb_fat32_lru;
   localparam integer TCK = 10;
   reg clk = 0, rst = 1;
   always #(TCK/2) clk = ~clk;

   wire        sd_clk, sd_cs_n, sd_mosi, sd_miso;
   wire        sd_done, sd_err, sd_ready, sd_buf_we;
   wire [7:0]  sd_buf_wdata;
   wire [4:0]  sd_dbg;
   wire [3:0]  sd_errc;

   wire        f_req, f_ready, f_ok;
   wire [31:0] f_lba, f_image_blocks, f_data_start, f_nclusters;
   wire [3:0]  f_err, f_spc_log;
   wire        f_need_select;
   wire [4:0]  f_state;
   reg  [15:0] map_idx = 0;
   reg         resel = 1'b0;
   reg  [7:0]  resel_idx = 8'd0;
   reg         nopersist = 1'b1;         // all selects ephemeral: the fast path
   wire [7:0]  f_img_count, f_img_index, f_wr_data;
   wire        f_req_wr;
   wire [9:0]  sd_wbuf_addr;
   wire [31:0] map_cluster;

   localparam [4:0] SD_S_READY = 5'd9;
   wire f_fire = f_req && (sd_dbg == SD_S_READY);

   sd_spi #(.DIV_INIT(8), .DIV_FAST(2), .TOKEN_TIMEOUT_CYC(200_000)) u_sd (
                                                                           .clk(clk), .rst(rst), .wspeed(8'd63),
                                                                           .rd(f_fire && !f_req_wr), .wr(f_fire && f_req_wr), .lba(f_lba),
                                                                           .done(sd_done), .err(sd_err),
                                                                           .buf_addr(sd_wbuf_addr), .buf_wdata(sd_buf_wdata), .buf_we(sd_buf_we),
                                                                           .buf_rdata(f_wr_data),
                                                                           .sd_clk(sd_clk), .sd_cs_n(sd_cs_n), .sd_mosi(sd_mosi), .sd_miso(sd_miso),
                                                                           .ready(sd_ready), .err_code(sd_errc), .dbg_state(sd_dbg));

   sd_model #(.FS_IMAGE(1'b1), .TWO_IMG(1'b1)) card (
                                                     .sd_clk(sd_clk), .sd_cs_n(sd_cs_n), .sd_mosi(sd_mosi), .sd_miso(sd_miso));

   fat32 #(.MAX_CLUSTERS(64), .AUTOPICK(1'b1)) dut (
                                                    .clk(clk), .rst(rst), .start(sd_ready),
                                                    .reselect(resel), .resel_idx(resel_idx), .nopersist(nopersist),
                                                    .req(f_req), .fire(f_fire), .sd_done(sd_done), .sd_err(sd_err),
                                                    .sd_buf_we(sd_buf_we), .sd_buf_wdata(sd_buf_wdata), .lba(f_lba),
                                                    .sd_buf_addr(sd_wbuf_addr), .wr_data(f_wr_data), .req_wr(f_req_wr),
                                                    .ready(f_ready), .ok(f_ok), .err(f_err),
                                                    .image_blocks(f_image_blocks), .data_start(f_data_start),
                                                    .spc_log(f_spc_log), .nclusters(f_nclusters), .need_select(f_need_select),
                                                    .img_count(f_img_count), .img_index(f_img_index),
                                                    .cat_raddr(12'd0), .cat_rdata(),
                                                    .map_idx(map_idx), .map_cluster(map_cluster),
                                                    .dbg_state(f_state));

   integer errors = 0;
   task check(input cond, input [511:0] what);
      begin
         if (cond) $display("    ok    %0s", what);
         else begin $display("*** FAIL  %0s", what); errors = errors + 1; end
      end
   endtask

   // Free-running count of actual SD operations, so a reselect's SD traffic can
   // be measured: a cache hit must add ZERO.
   integer fire_count = 0;
   always @(posedge clk) if (f_fire) fire_count = fire_count + 1;

   // Pulse a reselect, then wait until the SD engine has been quiet for a while
   // (a miss does reads; a hit does none) and the resolver reports ready.
   task do_reselect(input [7:0] idx);
      integer idle;
      begin
         resel_idx = idx;
         @(posedge clk); resel = 1'b1; @(posedge clk); resel = 1'b0;
         idle = 0;
         while (idle < 800) begin
            @(posedge clk);
            if (f_fire || !f_ready) idle = 0; else idle = idle + 1;
         end
      end
   endtask

   reg [31:0] want [0:3];
   integer i, snap;
   task verify_chain(input integer n, input [255:0] label);
      integer k; integer bad;
      begin
         bad = 0;
         for (k = 0; k < n; k = k + 1) begin
            map_idx = k[15:0];
            @(posedge clk); @(posedge clk);
            if (map_cluster !== want[k]) begin
               bad = bad + 1;
               $display("      chain[%0d] = %0d, wanted %0d", k, map_cluster, want[k]);
            end
         end
         check(bad == 0, label);
      end
   endtask

   initial begin
      repeat (5) @(posedge clk);
      rst = 0;

      // ---- boot resolve: profile.cfg names longname.img -> image 0 ----
      wait (f_ready);
      $display("-- boot resolve (should be image 0, longname.img)");
      check(f_ok === 1'b1,          "boot resolved an image");
      check(f_img_count === 8'd2,   "two images on the card");
      check(f_img_index === 8'd0,   "boot serves image 0");
      check(f_nclusters === 32'd4,  "image 0 has 4 clusters");
      want[0]=4; want[1]=6; want[2]=5; want[3]=7;
      verify_chain(4, "image 0 chain 4,6,5,7");

      // ---- reselect to image 1: a MISS, must read the SD and serve two.img
      $display("-- reselect to image 1 (MISS: fills slot 1)");
      snap = fire_count;
      do_reselect(8'd1);
      check(f_ok === 1'b1,          "reselect 1 ok");
      check(f_img_index === 8'd1,   "now serving image 1");
      check(f_nclusters === 32'd2,  "image 1 has 2 clusters");
      want[0]=9; want[1]=11;
      verify_chain(2, "image 1 chain 9,11");
      check(fire_count - snap > 0,  "a miss did SD reads");

      // ---- reselect back to image 0: a HIT, must be instant (no SD) ----
      $display("-- reselect to image 0 (HIT: slot 0, no SD)");
      snap = fire_count;
      do_reselect(8'd0);
      check(f_ok === 1'b1,          "reselect 0 ok");
      check(f_img_index === 8'd0,   "back to image 0");
      check(f_nclusters === 32'd4,  "image 0 has 4 clusters again");
      want[0]=4; want[1]=6; want[2]=5; want[3]=7;
      verify_chain(4, "image 0 chain restored from cache");
      check(fire_count - snap === 0, "cache HIT did ZERO SD reads");

      // ---- reselect to image 1 again: a HIT on slot 1 ----
      $display("-- reselect to image 1 (HIT: slot 1, no SD)");
      snap = fire_count;
      do_reselect(8'd1);
      check(f_img_index === 8'd1,   "image 1 again");
      check(f_nclusters === 32'd2,  "image 1 has 2 clusters");
      want[0]=9; want[1]=11;
      verify_chain(2, "image 1 chain from cache");
      check(fire_count - snap === 0, "cache HIT (slot 1) did ZERO SD reads");

      // ---- and once more back to 0, still a hit ----
      $display("-- reselect to image 0 (HIT again)");
      snap = fire_count;
      do_reselect(8'd0);
      check(f_img_index === 8'd0,   "image 0 again");
      want[0]=4; want[1]=6; want[2]=5; want[3]=7;
      verify_chain(4, "image 0 chain from cache");
      check(fire_count - snap === 0, "cache HIT (slot 0) did ZERO SD reads");

      $display("");
      if (errors == 0) $display("PASS: LRU-2 cache serves both images, hits are free");
      else             $display("FAIL: %0d problem(s)", errors);
      $finish;
   end

   initial begin
      #40_000_000;
      $display("*** TIMEOUT (state %0d, ready %0b, ok %0b, idx %0d)",
               f_state, f_ready, f_ok, f_img_index);
      $finish;
   end
endmodule

`default_nettype wire
