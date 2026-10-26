// sd_sweep: measure the SD clock ceilings on this board, for reads and for
// writes. A bench tool (SWEEP_ENABLE in top.v), never a shipping build: it
// drives the card at rates that are expected to fail.
//
// WHERE IT WRITES. Writes need sectors nobody uses, so we take them from the
// gap between the MBR and the first partition. A card formatted by the SD
// Association's formatter starts its partition at sector 8192, and the gap
// holds nothing. We use the 33 sectors that end two before the partition:
// one signature sector, then 32 data sectors. Before the first write we
// check that all 33 read back as zeros. If any of them doesn't, we refuse to
// write. Once the region is ours, the signature sector says so, and a later
// run accepts the region on the strength of that. That's what lets a run
// that died part way (a card hung at a failing rate) be run again. At the
// end we put zeros back, data first and the signature last.
//
// A GPT card (partition type $EE) or a partition that starts below sector 64
// has no gap worth the name, and we don't write at all.
//
// THE RATES. Eight dividers, each a half-period of the 50 MHz clock, so the
// bit rate is 50e6 / (2 * (DIV + 1)):
//
//   index   0     1     2      3      4      5      6       7
//   DIV    63    32    16      8      4      2      1       0
//   rate  391k  758k  1.47M  2.78M  5.0M   8.33M  12.5M   25M
//
// WRITES go first. At each rate we write all 32 data sectors with a pattern
// unique to that rate, then read each one back at the safe rate (DIV 63,
// the old harness's write rate) and compare all 512 bytes. A write that
// doesn't land fails too, since the previous rate's pattern differs in every
// byte.
//
// READS go second. We write one more pattern at the safe rate, then read the
// 32 sectors four times over at each rate, comparing every byte. sd_spi
// doesn't check the data CRC on a read, so the compare is the only thing
// that catches a flipped bit.
//
// A rate runs all its sectors unless a request fails, so the bad-byte count
// says how bad it is. Each sweep stops after the first rate that returned
// an error or a wrong byte, because a card that fails mid-transfer can stay
// hung until it's reinserted, and every rate after it would only measure
// the hang. A rate that needed retries but came back right is marked failed
// and the sweep carries on: sd_spi's retries hide marginal links from the
// //e, and we want to see them here.
//
// RESULTS, one word per rate, for reads and for writes:
//
//   [31] tested  [30] passed  [27:24] SD error code  [23:16] retries
//   [15:0] bad bytes (saturating)
//
// and a status word: [31:28] state, [27:24] why we stopped (F_ below),
// [20] zeros restored, [19:16] write rates passed, [11:8] read rates
// passed, [0] done.

`default_nettype none

module sd_sweep #(
                  parameter [7:0] SAFE_DIV = 8'd63
                  ) (
                     input  wire         clk,
                     input  wire         rst,
                     input  wire         start,

                     // the SD engine, which we own while `active` is high
                     input  wire         engine_ready,
                     input  wire         sd_done,
                     input  wire         sd_err,
                     input  wire [3:0]   sd_err_code,
                     input  wire [7:0]   sd_retries,
                     output wire         rd,
                     output wire         wr,
                     output reg  [31:0]  lba,
                     output reg  [7:0]   speed,
                     output reg  [7:0]   wspeed,

                     input  wire         buf_we,
                     input  wire [9:0]   buf_addr,
                     input  wire [7:0]   buf_wdata,
                     output wire [7:0]   wdata,

                     output reg          active,
                     output reg          done,
                     output wire [31:0]  status,
                     output reg  [31:0]  part_lba,
                     output wire [255:0] wres,
                     output wire [255:0] rres
                     );

   localparam integer NSEC   = 32;     // data sectors
   localparam integer PASSES = 4;      // read passes per rate

   // Why we stopped short, if we did.
   localparam [3:0] F_NONE  = 4'd0,
                    F_MBR   = 4'd1,    // sector 0 unreadable or no $55 $AA
                    F_NOGAP = 4'd2,    // GPT, empty slot 1, or no room
                    F_INUSE = 4'd3,    // the region isn't all zeros
                    F_SIGW  = 4'd4,    // the signature didn't write
                    F_PREP  = 4'd5;    // the read pattern didn't write

   localparam [3:0] S_IDLE  = 4'd0,  S_OP    = 4'd1,  S_MBR   = 4'd2,
                    S_SIG   = 4'd3,  S_ZSIG  = 4'd4,  S_ZERO  = 4'd5,
                    S_SIGW  = 4'd6,  S_SIGV  = 4'd7,  S_WW    = 4'd8,
                    S_WV    = 4'd9,  S_PW    = 4'd10, S_PV    = 4'd11,
                    S_RD    = 4'd12, S_CLR   = 4'd13, S_CLRS  = 4'd14,
                    S_END   = 4'd15;

   // What a read compares against, and what a write sends.
   localparam [1:0] M_ZERO = 2'd0, M_SIG = 2'd1, M_PAT = 2'd2, M_NONE = 2'd3;

   localparam [7:0] SEED_R = 8'hA5;    // the read sweep's pattern

   function [7:0] div_of(input [2:0] i);
      case (i)
        3'd0: div_of = 8'd63; 3'd1: div_of = 8'd32;
        3'd2: div_of = 8'd16; 3'd3: div_of = 8'd8;
        3'd4: div_of = 8'd4;  3'd5: div_of = 8'd2;
        3'd6: div_of = 8'd1;  default: div_of = 8'd0;
      endcase
   endfunction

   // The write sweep's pattern for rate i. None of them is SEED_R.
   function [7:0] seed_w(input [2:0] i);
      seed_w = {i, 5'b10110};
   endfunction

   // Every byte value appears twice in a sector, so a pattern is never all
   // zeros and never a constant. Two seeds differ in every byte.
   function [7:0] pat(input [7:0] seed, input [4:0] s, input [8:0] a);
      pat = a[7:0] ^ {a[8], s, 2'b01} ^ seed;
   endfunction

   // The signature: "PESWEEP1" and then zeros.
   function [7:0] sig(input [8:0] a);
      case (a)
        9'd0: sig = "P"; 9'd1: sig = "E"; 9'd2: sig = "S"; 9'd3: sig = "W";
        9'd4: sig = "E"; 9'd5: sig = "E"; 9'd6: sig = "P"; 9'd7: sig = "1";
        default: sig = 8'h00;
      endcase
   endfunction

   function [7:0] want_byte(input [1:0] m, input [7:0] seed, input [4:0] s,
                            input [8:0] a);
      case (m)
        M_SIG:   want_byte = sig(a);
        M_PAT:   want_byte = pat(seed, s, a);
        default: want_byte = 8'h00;
      endcase
   endfunction

   reg [3:0]   st, cont;
   reg [3:0]   why;
   reg         cleaned;
   reg [3:0]   wpassed, rpassed;
   reg [255:0] wres_r, rres_r;

   // the operation in flight
   reg         want, op_wr;
   reg [1:0]   op_mode;            // compare mode on a read, source on a write
   reg [7:0]   op_seed;
   reg [4:0]   op_sec;
   reg [9:0]   op_bad;             // wrong bytes in this sector
   reg         op_err;
   reg [3:0]   op_code;            // the SD error code, if it failed

   // where the sweep is
   reg [31:0]  sig_lba;
   reg [2:0]   ri;                 // rate index
   reg [4:0]   sec;
   reg [1:0]   pass;
   reg [15:0]  bad;                // wrong bytes at this rate so far
   reg [7:0]   retry0;             // sd_retries when this rate started
   reg [7:0]   ptype;
   reg         sig_lo, sig_hi;

   wire        fire = want && engine_ready;
   assign rd = fire && !op_wr;
   assign wr = fire && op_wr;
   assign wdata = want_byte(op_mode, op_seed, op_sec, buf_addr[8:0]);
   assign wres = wres_r;
   assign rres = rres_r;

   assign status = {st, why, 3'd0, cleaned, wpassed, 4'd0, rpassed, 7'd0,
                    done};

   // The rate's result, counting the sector that just finished.
   wire [7:0]  retried = sd_retries - retry0;
   wire [15:0] bad_sum = bad + {6'd0, op_bad};
   wire [15:0] bad_sat = (bad_sum < bad) ? 16'hFFFF : bad_sum;
   wire        rate_ok = !op_err && bad_sat == 16'd0 && retried == 8'd0;
   wire        rate_stop = op_err || bad_sat != 16'd0;
   wire [31:0] res_now = {1'b1, rate_ok, 2'd0, op_code, retried, bad_sat};

   wire [31:0] data0 = sig_lba + 32'd1;   // the first data sector

   task issue(input w, input [31:0] l, input [7:0] spd, input [7:0] wspd,
              input [1:0] m, input [7:0] seed, input [4:0] s,
              input [3:0] next);
      begin
         want    <= 1'b1;
         op_wr   <= w;
         lba     <= l;
         speed   <= spd;
         wspeed  <= wspd;
         op_mode <= m;
         op_seed <= seed;
         op_sec  <= s;
         op_bad  <= 10'd0;
         cont    <= next;
         st      <= S_OP;
      end
   endtask

   // Reads at the safe rate, comparing against mode m.
   task safe_read(input [31:0] l, input [1:0] m, input [7:0] seed,
                  input [4:0] s, input [3:0] next);
      issue(1'b0, l, SAFE_DIV, SAFE_DIV, m, seed, s, next);
   endtask

   task safe_write(input [31:0] l, input [1:0] m, input [7:0] seed,
                   input [4:0] s, input [3:0] next);
      issue(1'b1, l, SAFE_DIV, SAFE_DIV, m, seed, s, next);
   endtask

   // A new rate starts with nothing counted.
   task new_rate(input [2:0] i);
      begin
         ri      <= i;
         sec     <= 5'd0;
         pass    <= 2'd0;
         bad     <= 16'd0;
         retry0  <= sd_retries;
         op_code <= 4'd0;
      end
   endtask

   task write_rate(input [2:0] i);
      begin
         new_rate(i);
         issue(1'b1, data0, SAFE_DIV, div_of(i), M_PAT, seed_w(i), 5'd0,
               S_WW);
      end
   endtask

   task read_rate(input [2:0] i);
      begin
         new_rate(i);
         issue(1'b0, data0, div_of(i), SAFE_DIV, M_PAT, SEED_R, 5'd0, S_RD);
      end
   endtask

   task prep;
      begin
         sec <= 5'd0;
         safe_write(data0, M_PAT, SEED_R, 5'd0, S_PW);
      end
   endtask

   task clear;
      begin
         sec <= 5'd0;
         safe_write(data0, M_ZERO, 8'd0, 5'd0, S_CLR);
      end
   endtask

   always @(posedge clk) begin
      if (fire) want <= 1'b0;

      // Compare as the sector streams in. Sector 0 is a capture instead.
      if (st == S_OP && !op_wr && buf_we) begin
         if (op_mode == M_NONE) begin
            case (buf_addr)
              10'h1C2: ptype <= buf_wdata;
              10'h1C6: part_lba[7:0]   <= buf_wdata;
              10'h1C7: part_lba[15:8]  <= buf_wdata;
              10'h1C8: part_lba[23:16] <= buf_wdata;
              10'h1C9: part_lba[31:24] <= buf_wdata;
              10'h1FE: sig_lo <= (buf_wdata == 8'h55);
              10'h1FF: sig_hi <= (buf_wdata == 8'hAA);
              default: ;
            endcase
         end else if (buf_wdata != want_byte(op_mode, op_seed, op_sec,
                                             buf_addr[8:0])) begin
            if (op_bad != 10'h3FF) op_bad <= op_bad + 10'd1;
         end
      end

      if (rst) begin
         st <= S_IDLE; cont <= S_IDLE; why <= F_NONE; cleaned <= 1'b0;
         wpassed <= 4'd0; rpassed <= 4'd0; active <= 1'b0; done <= 1'b0;
         wres_r <= 256'd0; rres_r <= 256'd0;
         want <= 1'b0; op_wr <= 1'b0; op_mode <= M_NONE; op_seed <= 8'd0;
         op_sec <= 5'd0; op_bad <= 10'd0; op_err <= 1'b0; op_code <= 4'd0;
         lba <= 32'd0; speed <= SAFE_DIV; wspeed <= SAFE_DIV;
         part_lba <= 32'd0; sig_lba <= 32'd0; ri <= 3'd0; sec <= 5'd0;
         pass <= 2'd0; bad <= 16'd0; retry0 <= 8'd0; ptype <= 8'd0;
         sig_lo <= 1'b0; sig_hi <= 1'b0;
      end else begin
         case (st)
           S_IDLE: if (start) begin
              active <= 1'b1;
              safe_read(32'd0, M_NONE, 8'd0, 5'd0, S_MBR);
           end

           // Wait for the engine, then carry on where the caller said.
           S_OP: if (sd_done) begin
              op_err <= sd_err;
              if (sd_err) op_code <= sd_err_code;
              st <= cont;
           end

           // ---- claim the region
           S_MBR: begin
              if (op_err || !sig_lo || !sig_hi) begin
                 why <= F_MBR; st <= S_END;
              end else if (ptype == 8'hEE || ptype == 8'h00
                           || part_lba < 32'd64) begin
                 why <= F_NOGAP; st <= S_END;
              end else begin
                 sig_lba <= part_lba - 32'd34;
                 safe_read(part_lba - 32'd34, M_SIG, 8'd0, 5'd0, S_SIG);
              end
           end

           // Ours from an earlier run, or blank and needing a check of every
           // sector before we write anything?
           S_SIG: begin
              if (op_err)                 begin why <= F_MBR; st <= S_END; end
              else if (op_bad == 10'd0)   write_rate(3'd0);
              else safe_read(sig_lba, M_ZERO, 8'd0, 5'd0, S_ZSIG);
           end

           S_ZSIG: begin
              if (op_err || op_bad != 10'd0) begin
                 why <= F_INUSE; st <= S_END;
              end else begin
                 sec <= 5'd0;
                 safe_read(data0, M_ZERO, 8'd0, 5'd0, S_ZERO);
              end
           end

           S_ZERO: begin
              if (op_err || op_bad != 10'd0) begin
                 why <= F_INUSE; st <= S_END;
              end else if (sec == NSEC[4:0] - 5'd1) begin
                 safe_write(sig_lba, M_SIG, 8'd0, 5'd0, S_SIGW);
              end else begin
                 sec <= sec + 5'd1;
                 safe_read(data0 + {27'd0, sec + 5'd1}, M_ZERO, 8'd0, 5'd0,
                           S_ZERO);
              end
           end

           S_SIGW: begin
              if (op_err) begin why <= F_SIGW; st <= S_END; end
              else safe_read(sig_lba, M_SIG, 8'd0, 5'd0, S_SIGV);
           end

           S_SIGV: begin
              if (op_err || op_bad != 10'd0) begin
                 why <= F_SIGW; st <= S_END;
              end else write_rate(3'd0);
           end

           // ---- the write sweep
           // Sector `sec` has gone out at rate ri. Read it back at the safe
           // rate.
           S_WW: begin
              if (op_err) begin
                 wres_r[32*ri +: 32] <= res_now;
                 prep;
              end else begin
                 safe_read(data0 + {27'd0, sec}, M_PAT, seed_w(ri), sec,
                           S_WV);
              end
           end

           S_WV: begin
              if (op_err || sec == NSEC[4:0] - 5'd1) begin
                 wres_r[32*ri +: 32] <= res_now;
                 if (rate_ok) wpassed <= wpassed + 4'd1;
                 if (rate_stop || ri == 3'd7) prep;
                 else write_rate(ri + 3'd1);
              end else begin
                 bad <= bad_sat;
                 sec <= sec + 5'd1;
                 issue(1'b1, data0 + {27'd0, sec + 5'd1}, SAFE_DIV,
                       div_of(ri), M_PAT, seed_w(ri), sec + 5'd1, S_WW);
              end
           end

           // ---- the read pattern, written and checked at the safe rate
           S_PW: begin
              if (op_err) begin why <= F_PREP; clear; end
              else safe_read(data0 + {27'd0, sec}, M_PAT, SEED_R, sec, S_PV);
           end

           S_PV: begin
              if (op_err || op_bad != 10'd0) begin
                 why <= F_PREP; clear;
              end else if (sec == NSEC[4:0] - 5'd1) begin
                 read_rate(3'd0);
              end else begin
                 sec <= sec + 5'd1;
                 safe_write(data0 + {27'd0, sec + 5'd1}, M_PAT, SEED_R,
                            sec + 5'd1, S_PW);
              end
           end

           // ---- the read sweep
           S_RD: begin
              if (op_err || (sec == NSEC[4:0] - 5'd1
                             && pass == PASSES[1:0] - 2'd1)) begin
                 rres_r[32*ri +: 32] <= res_now;
                 if (rate_ok) rpassed <= rpassed + 4'd1;
                 if (rate_stop || ri == 3'd7) clear;
                 else read_rate(ri + 3'd1);
              end else begin
                 bad <= bad_sat;
                 if (sec == NSEC[4:0] - 5'd1) pass <= pass + 2'd1;
                 sec <= sec + 5'd1;   // wraps to 0 for the next pass
                 issue(1'b0, data0 + {27'd0, sec + 5'd1}, div_of(ri),
                       SAFE_DIV, M_PAT, SEED_R, sec + 5'd1, S_RD);
              end
           end

           // ---- put the zeros back, the signature last
           S_CLR: begin
              if (op_err) st <= S_END;
              else if (sec == NSEC[4:0] - 5'd1)
                safe_write(sig_lba, M_ZERO, 8'd0, 5'd0, S_CLRS);
              else begin
                 sec <= sec + 5'd1;
                 safe_write(data0 + {27'd0, sec + 5'd1}, M_ZERO, 8'd0,
                            sec + 5'd1, S_CLR);
              end
           end

           S_CLRS: begin
              cleaned <= !op_err;
              st      <= S_END;
           end

           default: begin              // S_END, where we stay
              active <= 1'b0;
              done   <= 1'b1;
           end
         endcase
      end
   end

endmodule

`default_nettype wire
