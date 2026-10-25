// sd_arb: ownership arbitration between the SD masters that can overlap.
//
// The block_router (host I/O) and fat32 (the deferred persist) both
// drive the single SD engine, and the persist runs during live OS
// sessions, so the two really do overlap. Without arbitration a host
// read issued while a persist is in flight is dropped by the busy
// engine. The persist's completion then raises blk_ready, and the host
// is served a stale block buffer. That corrupts data with no error
// reported, because the host then read-modify-writes a block it never
// actually read.
//
// Rules enforced here:
//   - Router pulses (sd_rd/sd_wr) are LATCHED until dispatched, never
//     dropped.
//   - One operation in flight at a time. The router has priority:
//     fat32 only fires when the router has nothing pending. fat32's
//     multi-op sequences tolerate interleaving by design (each req is
//     self-contained).
//   - The firmware updater is last: it only fires when neither the
//     router nor fat32 wants the engine. It runs in the background
//     while a host may be talking, so a waiting host read always goes
//     first; at worst it waits out the one updater read in flight.
//   - sd_done and sd_err are routed to the OWNER of the in-flight op
//     only.
//   - The owner flag keys the write-data source and buffer-write
//     gating in top: fat32-owned reads must not touch the host block
//     buffer, and host-owned reads must not fill fat32's sector buffer.
//
// The pre-host masters (self-test, verifier, streamer) have their own
// direct paths in top: they run before a host exists and never overlap
// the router or the persist.

`default_nettype none

module sd_arb (
               input wire  clk,
               input wire  rst,

               // engine status
               input wire  engine_ready, // SD FSM idle, can accept a command
               input wire  sd_done,
               input wire  sd_err,

               // router face (pulses in, routed completion out)
               input wire  r_rd,
               input wire  r_wr,
               output wire r_fire_rd,
               output wire r_fire_wr,
               output wire r_done,
               output wire r_err,

               // fat32 face (req/fire handshake)
               input wire  f_req,
               input wire  f_req_wr,
               output wire f_fire,
               output wire f_done,
               output wire f_err,

               // updater face (req/fire handshake, reads only)
               input wire  u_req,
               output wire u_fire,
               output wire u_done,
               output wire u_err,

               // who owns the op in flight (keys data-source and
               // buffer gating in top)
               output reg  own_fs,
               output reg  own_up
               );

   reg rd_p, wr_p;
   wire r_pend = rd_p | wr_p | r_rd | r_wr;
   wire r_fire = r_pend && engine_ready;
   assign r_fire_rd = r_fire && (rd_p | r_rd);
   assign r_fire_wr = r_fire && (wr_p | r_wr);
   assign f_fire    = f_req && engine_ready && !r_pend;
   assign u_fire    = u_req && engine_ready && !r_pend && !f_req;

   always @(posedge clk) begin
      if (rst) begin
         rd_p <= 1'b0; wr_p <= 1'b0; own_fs <= 1'b0; own_up <= 1'b0;
      end else begin
         if (r_fire) begin
            rd_p <= 1'b0; wr_p <= 1'b0;
         end else begin
            if (r_rd) rd_p <= 1'b1;
            if (r_wr) wr_p <= 1'b1;
         end
         if (r_fire)      begin own_fs <= 1'b0; own_up <= 1'b0; end
         else if (f_fire) begin own_fs <= 1'b1; own_up <= 1'b0; end
         else if (u_fire) begin own_fs <= 1'b0; own_up <= 1'b1; end
      end
   end

   assign r_done = sd_done && !own_fs && !own_up;
   assign r_err  = sd_err  && !own_fs && !own_up;
   assign f_done = sd_done &&  own_fs;
   assign f_err  = sd_err  &&  own_fs;
   assign u_done = sd_done &&  own_up;
   assign u_err  = sd_err  &&  own_up;

endmodule

`default_nettype wire
