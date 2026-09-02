// block_router: decides where a block number is served from.
//
//   $FFFFFF  the synthesized spare/status table (read only on a real 4.04 drive)
//   $FFFFFE  the drive's 1 KB RAM buffer, readable and writable
//   $FFFF00..$FFFFFD  the boot-time image selector's window, while the menu
//                     phase is live; ordinary blocks otherwise
//   anything else     the SD card, if it is inside the SERVED range
//
// The served range is not necessarily the claimed range. See
// geometry.v: a full 32MB volume claims $FFFC blocks because the
// //e's boot path demands a multiple of 4, and serves $FFFF of them
// because the image really has that many.
//
// See PROTOCOL.md (profile-dumper) section 3 for the special block
// numbers and section 9.2 for why the volume size matters.

`default_nettype none

module block_router #(
                      parameter FS_ENABLE = 1'b1
                      ) (
                         input wire        clk,

                         // Geometry, from geometry.v. TWO counts,
                         // deliberately separate: what we tell the
                         // host and what we will actually serve are
                         // not the same number for a full 32MB
                         // volume. See geometry.v for why.
                         input wire [15:0] claim_blocks,
                         input wire [15:0] served_blocks,

                         // No usable card. Rather than serving
                         // whatever the block buffer happens to hold,
                         // serve a self-verifying pattern: the Teensy
                         // build does the same, and it is what makes
                         // the DB-25 testable with nothing in the SD
                         // socket at all.
                         input wire        no_media,

                         // Refuse writes. Either a build-time setting
                         // or the socket's write-protect contact; the
                         // caller decides which, this just obeys.
                         input wire        write_protect,
                         output reg        write_refused,

                         // ---- filesystem mapping (src/fat32.v) ---
                         // When FS_ENABLE, a ProFile block is an
                         // offset into an image FILE, and its sector
                         // is found through the file's cluster
                         // chain. The chain lookup has a cycle of
                         // latency, which is why the request path
                         // below gained a wait.  The selector claims
                         // block 0 during a boot, and the whole
                         // $FFFF00 window.
                         input wire        sel_hit,
                         input wire [7:0]  sel_data,

                         input wire        fs_ok, // an image was located

                         // High once the resolver has
                         // finished. Distinct from fs_ok: a SELECT
                         // pulses fat32's `reselect` and the resolver
                         // goes NOT-READY while it finds the new
                         // image. During that window fs_ok is also
                         // low, and treating that as "no image"
                         // answers every read with out-of-range. The
                         // host reads straight through a re-resolve,
                         // so it would collect a run of out-of-range
                         // answers and give up. A real drive just
                         // stays busy.

                         input wire        fs_ready,
                         // Menu phase (selection not yet made since
                         // the last arm). While high the $FFFFxx
                         // window is the selector's; while low it
                         // aliases to the image's top 256 blocks
                         // (see window_live in selector.v).
                         input wire        window_live,
                         input wire [31:0] fs_data_start,
                         input wire [3:0]  fs_spc_log,
                         output reg [15:0] fs_map_idx,
                         input wire [31:0] fs_map_cluster,
                         input wire [23:0] blk_addr,
                         input wire        blk_rd,
                         input wire        blk_wr,
                         output reg        blk_ready,

                         // buffer face presented upward to profile_target
                         input wire [9:0]  buf_addr,
                         output wire [7:0] buf_rdata,
                         input wire [7:0]  buf_wdata,
                         input wire        buf_we,

                         // SD face
                         output reg        sd_rd,
                         output reg        sd_wr,
                         output reg [31:0] sd_lba,
                         input wire        sd_done,
                         input wire        sd_err,

                         // block buffer face (dual-port BRAM lives in top)
                         output wire [9:0] ram_addr,
                         input wire [7:0]  ram_rdata,
                         output wire [7:0] ram_wdata,
                         output wire       ram_we,

                         output reg        out_of_range,
                         // A read beyond the served range, answered
                         // with zeros rather than failed. Counted for
                         // telemetry, never an error. Writes stay
                         // refused: lenient reads are
                         // inconsequential, but a lenient write
                         // would report success and discard the
                         // data.
                         output reg        oor_read
                         );

   wire is_spare = (blk_addr == 24'hFFFFFF);
   // GEOMETRY OVERLOADING:
   // the //e card maps ProDOS $FFxx to wire $FFFFxx unconditionally,
   // so the top 256 blocks of a full-size volume land in the magic
   // window. While the menu phase is over (window_live low) those
   // addresses translate to image blocks $00FFxx. The spare table at
   // $FFFFFF is permanent. A 65535-block volume's top index is $FFFE,
   // so ProDOS never wants $FFFF.  The rambuf at $FFFFFE is
   // menu-phase-only for the same reason.  The alias applies only when
   // the aliased block actually EXISTS in the image. On any
   // normal-size image the top range is beyond the served count, so
   // the alias never fires. The scratch buffer and the window's no-op
   // answers then keep their classic behavior. Only a full-size volume
   // trades them for data.
   wire win_alias = !window_live && (blk_addr[23:8] == 16'hFFFF) && !is_spare
        && ({8'hFF, blk_addr[7:0]} < served_blocks);
   wire [23:0] eff_addr = win_alias ? {8'h00, 8'hFF, blk_addr[7:0]} : blk_addr;
   wire        is_rambuf= (blk_addr == 24'hFFFFFE) && !win_alias;
   wire        is_magic = (blk_addr >= 24'hFFFF00) && (blk_addr <= 24'hFFFFFD)
               && !win_alias;
   // Range-checked against what we SERVE, not what we claim. On a
   // decoupled 32MB volume the blocks above the claim must still be
   // readable and writable.
   wire        in_vol   = (eff_addr < {8'h00, served_blocks});

   // The resolver is busy, which is not an error. Hold the request
   // until the resolver comes back. blk_addr is stable for the whole
   // transaction (the target holds it while it waits for blk_ready),
   // so the request can simply be replayed when we are ready.
   wire        fs_busy  = FS_ENABLE && !fs_ready;

   wire [7:0]  spare_byte;
   spare_table u_spare (.blocks_claim(claim_blocks), 
                        .addr(buf_addr),
                        .data(spare_byte));
   
   // ------ the drive's own 1 KB scratch buffer, block $FFFFFE ------
   // A real ProFile has a RAM buffer that the host can read and write
   // without touching the platter, and it is SEPARATE
   // storage. Pointing $FFFFFE at the shared block buffer very nearly
   // works and then does not: the next ordinary block read overwrites
   // it, so a host that stashes something there and reads it back
   // later gets the last block read instead. So this needs special
   // handling.
   
   reg [7:0] rambuf [0:1023];
   reg [7:0] rambuf_q;
   reg       sel_spare, sel_pattern, sel_sel, sel_zero;
   reg [1:0] look;
   reg       look_rd, look_wr;
   reg [31:0] look_base, look_sec;

   // is_rambuf, NOT a latched select. blk_wr does not pulse until
   // after the host's 512 bytes are already in the buffer, so a
   // select latched on blk_wr is still the PREVIOUS command's answer
   // for the whole data phase: scratch writes land in the block
   // buffer and, worse, the next ordinary write finds its own buffer
   // writes suppressed and pushes stale bytes at the card.  blk_addr
   // is stable from S_DECODE onward, so decode it directly.
   always @(posedge clk) begin
      if (buf_we && is_rambuf) rambuf[buf_addr] <= buf_wdata;
      rambuf_q <= rambuf[buf_addr];
   end

   // ---- the no-media pattern --------------------------------------
   // $A5, then the three block-number bytes, then a running sum. Self
   // verifying: it checks the protocol AND that the block number
   // reached us, because a drive that ignored the address would
   // return the same bytes every time.
   wire [7:0] pat_byte = (buf_addr == 10'd0) ? 8'hA5
              : (buf_addr == 10'd1) ? blk_addr[23:16]
              : (buf_addr == 10'd2) ? blk_addr[15:8]
              : (buf_addr == 10'd3) ? blk_addr[7:0]
              : (blk_addr[7:0] + buf_addr[7:0]);

   assign buf_rdata = sel_sel     ? sel_data
                      : sel_spare   ? spare_byte
                      : is_rambuf   ? rambuf_q
                      : sel_pattern ? pat_byte
                      : sel_zero    ? 8'h00
                      : ram_rdata;
   assign ram_addr  = buf_addr;
   assign ram_wdata = buf_wdata;
   // Writes aimed at the scratch buffer must not also land in the
   // block buffer, or the next flush would push them at the card.
   assign ram_we    = buf_we && !is_rambuf;

   reg pend_rd = 1'b0, pend_wr = 1'b0;   // explicit: this module has no rst
   // A parked request replays the instant the resolver is ready
   // again.
   wire do_rd = (blk_rd | pend_rd) & ~fs_busy;
   wire do_wr = (blk_wr | pend_wr) & ~fs_busy;

   always @(posedge clk) begin
      sd_rd <= 1'b0;
      sd_wr <= 1'b0;
      // Pulses, not levels. Held high until the next request, these
      // would re-set the status failure bit on the cycle after it was
      // cleared, so one bad block number would mark every command
      // after it as failed too.
      out_of_range  <= 1'b0;
      oor_read      <= 1'b0;
      write_refused <= 1'b0;
      // Park a request that arrives mid-reselect rather than failing
      // it. The magic window and the scratch buffer do not depend on
      // the filesystem, so they are answered immediately either way.
      if ((blk_rd || blk_wr) && fs_busy && !(is_spare || is_rambuf || is_magic)) begin
         pend_rd <= blk_rd;
         pend_wr <= blk_wr;
         // The answer is NOT ready. blk_ready must drop here: left
         // at the value the previous transaction set (1 after a
         // magic-window answer), profile_target's level-sensitive
         // S_CONFIRM would fire at once and the data phase would
         // serve the stale output mux instead of the block being
         // fetched.
         blk_ready <= 1'b0;
      end else if (do_rd || do_wr) begin
         pend_rd <= 1'b0;
         pend_wr <= 1'b0;
         sel_spare     <= is_spare && !sel_hit;
         sel_sel       <= sel_hit;
         sel_pattern   <= (no_media || (FS_ENABLE && !fs_ok))
           && !is_spare && !is_rambuf;
         // Lenient reads, strict writes: see oor_read above.
         sel_zero      <= do_rd && !(is_spare || is_rambuf || is_magic || in_vol
                                     || sel_hit);
         oor_read      <= do_rd && !(is_spare || is_rambuf || is_magic || in_vol);
         out_of_range  <= do_wr && !(is_spare || is_rambuf || is_magic || in_vol);
         // A write to the spare table or to a block we do not serve
         // is refused, and so is any write at all while
         // write-protected. The scratch buffer stays writable: it is
         // RAM, not media.
         write_refused <= do_wr && (write_protect || is_spare
                                    || (!is_rambuf && (no_media || !in_vol
                                                       || (FS_ENABLE && !fs_ok))));
         blk_ready     <= 1'b0;
         if (sel_hit) begin
            blk_ready <= 1'b1;              // the selector answers from ROM
         end else if (is_spare || is_rambuf) begin
            blk_ready <= 1'b1;              // nothing to fetch
         end else if (no_media || (FS_ENABLE && !fs_ok)) begin
            blk_ready <= 1'b1;              // the pattern needs no fetching either
         end else if (in_vol && !(do_wr && write_protect)) begin
            if (FS_ENABLE) begin
               // Blocks live in a file, so the sector has to come out
               // of the cluster chain. Start the lookup and wait; see
               // LOOKUP below.
               fs_map_idx <= eff_addr >> fs_spc_log;
               look_sec   <= eff_addr & ((32'd1 << fs_spc_log) - 32'd1);
               look_rd    <= do_rd;
               look_wr    <= do_wr;
               look       <= 2'd3;
            end else begin
               sd_lba <= {8'h00, eff_addr};  // raw: 1 block == 1 SD sector
               sd_rd  <= do_rd;
               sd_wr  <= do_wr;
            end
         end else begin
            blk_ready <= 1'b1;              // magic, out of range, or refused
         end
      end else if (look != 2'd0) begin
         // The chain RAM answers a cycle after fs_map_idx is
         // presented, and one more cycle of slack costs nothing and
         // keeps this off the critical path. The arithmetic is spread
         // over two cycles rather than one: in a single cycle the
         // 32-bit variable shift and both adds land on the same path.
         // The wait for the chain RAM is needed anyway, so the extra
         // cycle is free.
         look <= look - 2'd1;
         if (look == 2'd2) begin
            look_base <= (fs_map_cluster - 32'd2) << fs_spc_log;
         end else if (look == 2'd1) begin
            sd_lba <= fs_data_start + look_base + look_sec;
            sd_rd  <= look_rd;
            sd_wr  <= look_wr;
         end
      end else if (sd_done) begin
         blk_ready <= 1'b1;
      end
   end

endmodule

`default_nettype wire
