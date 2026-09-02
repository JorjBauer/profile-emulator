// block_stream: put a real block from the SD card onto the DB-25, with no host.
//
// The pin walk in bus_exercise.v checks the wiring: every PD bit
// reaches the right pin, parity is generated, BSY moves. It checks
// nothing about the PATH, because the thirteen bytes it drives are
// constants generated three gates away from the pads.
//
// This drives the real thing instead. It asks the block router for a
// block, waits for it, and clocks all 512 bytes onto the bus at a
// chosen pace. The sniffer captures them and tools/check_stream.js
// compares them against the same block read from the card image on
// the host. That covers, on real hardware and end to end:
//
//     SD card -> sd_spi -> block buffer -> block_router -> profile_bus
//             -> TRELLIS_IO pads -> 74LVC245 -> DB-25 -> sniffer
//
// which is the entire read path a boot depends on, minus the
// handshake and the host's strobe pacing. Those need a host;
// everything else is here.
//
// It free-runs rather than being strobe-paced, because there is no
// host to strobe. So it is a DATA PATH test, not a timing test,
// though the sniffer's nanosecond timestamps make the byte period
// measurable as a side effect.

`default_nettype none

module block_stream #(
                      parameter integer CLK_HZ = 50_000_000,
                      parameter integer DWELL_US = 20, // per byte, must beat the sniffer's
                      // sample loop but stay well under
                      // its 30 ms idle timeout
                      parameter integer GAP_US = 500,  // bus released between repeats, so
                      // the capture can be segmented
                      parameter [23:0]  BLOCK = 24'd0,
                      parameter integer NBYTES = 512
                      ) (
                         input wire        clk,
                         input wire        rst,
                         input wire        enable,

                         // block face, borrowed from profile_target (held in
                         // reset in this build)
                         output reg [23:0] blk_addr,
                         output reg        blk_rd,
                         input wire        blk_ready,
                         output reg [9:0]  buf_addr,
                         input wire [7:0]  buf_rdata,

                         // bus face
                         output reg        drive_en,
                         output reg [7:0]  drive_data,

                         output wire       active, // owns the block buffer's read port
                         output wire [3:0] dbg_state
                         );

   localparam integer DWELL = (CLK_HZ / 1_000_000) * DWELL_US;
   localparam integer GAP   = (CLK_HZ / 1_000_000) * GAP_US;
   
   localparam [2:0]   
                      S_IDLE  = 3'd0,
                      S_REQ   = 3'd1,   // ask the router for the block
                      S_WAIT  = 3'd2,   // it is fetching from the card
                      S_SETUP = 3'd3,   // address presented, waiting on the registered read
                      S_HOLD  = 3'd4,   // byte on the bus for a full dwell
                      S_GAP   = 3'd5;   // released, so repeats can be told apart

   reg [2:0]          st;
   reg [31:0]         t;
   reg [9:0]          idx;
   reg [1:0]          lat;

   // High from the moment the block is requested until the last byte
   // is off the bus, so nothing else borrows the buffer's read port
   // underneath us.
   assign active    = (st != S_IDLE) && (st != S_GAP);
   assign dbg_state = {1'b0, st};

   always @(posedge clk) begin
      blk_rd <= 1'b0;

      if (rst || !enable) begin
         st <= S_IDLE; t <= 0; idx <= 0; lat <= 0;
         drive_en <= 1'b0; drive_data <= 8'h00;
         buf_addr <= 10'd0; blk_addr <= BLOCK;
      end else case (st)

                 S_IDLE: begin
                    drive_en <= 1'b0;
                    blk_addr <= BLOCK;
                    blk_rd   <= 1'b1;
                    st       <= S_REQ;
                 end

                 // blk_rd is a pulse and blk_ready is deasserted a
                 // cycle later, so waiting for "ready" immediately
                 // would see the PREVIOUS transfer's ready and stream
                 // whatever the buffer happened to hold.
                 S_REQ: st <= S_WAIT;

                 S_WAIT: if (blk_ready) begin
                    idx      <= 10'd0;
                    buf_addr <= 10'd0;
                    lat      <= 2'd0;
                    st       <= S_SETUP;
                 end

                 // The block buffer's read is REGISTERED: buf_rdata
                 // is one cycle behind buf_addr. Two cycles of slack
                 // costs nothing here and means the byte on the bus
                 // is the byte at the address, not its neighbor.
                 S_SETUP: if (lat == 2'd2) begin
                    drive_data <= buf_rdata;
                    drive_en   <= 1'b1;
                    t          <= 0;
                    st         <= S_HOLD;
                 end else lat <= lat + 2'd1;

                 S_HOLD: if (t == DWELL[31:0]) begin
                    if (idx == NBYTES[9:0] - 10'd1) begin
                       drive_en <= 1'b0;
                       t        <= 0;
                       st       <= S_GAP;
                    end else begin
                       idx      <= idx + 10'd1;
                       buf_addr <= idx + 10'd1;
                       lat      <= 2'd0;
                       st       <= S_SETUP;
                    end
                 end else t <= t + 32'd1;

                 S_GAP: if (t == GAP[31:0]) begin
                    t  <= 0;
                    st <= S_IDLE;                 // fetch it again and repeat
                 end else t <= t + 32'd1;

                 default: st <= S_IDLE;
               endcase
   end

endmodule

`default_nettype wire
