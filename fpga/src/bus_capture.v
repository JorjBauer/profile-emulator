// bus_capture: an in-fabric event recorder for the DB-25.
//
// On every CHANGE of {bus, /PSTRB, PR/-W, /PCMD, drive_en} it stores
// one 32-bit event: a 16-bit timestamp delta in 100 ns ticks
// (capped), the bus byte, and the five line states. Recording arms at
// reset release and starts at the first /PCMD assertion, which on a
// boot is the first thing the card does at the drive. 4096 events is
// far more than a command phase needs.
//
// When the store fills, or half a second passes with no events after
// at least one was taken, the whole store streams out the UART once
// as hex lines:
//
//     CAP <n>
//     <dt16> <bus8> <flags>      flags bit4..0 = pstrb,prw,pcmd,drive_en,hstb
//     ...
//     END
//
// `dumping` is high for the duration so the top level can mux the pin
// away from the regular reporter. The dump happens after the boot has
// already failed, when the reporter has nothing new to say anyway.

`default_nettype none

module bus_capture #(
                     parameter integer CLK_HZ = 50_000_000,
                     parameter integer DEPTH_LOG2 = 12
                     ) (
                        input  wire       clk,
                        input  wire       rst,
                        input  wire [7:0] bus_in,      // already synchronized, from profile_bus
                        input  wire       pstrb_n,     // raw pins; synchronized locally
                        input  wire       prw,
                        input  wire       pcmd_n,
                        input  wire       drive_en,
                        input  wire       host_stb,
                        input  wire       arm,          // re-arm after boot to aim at a later event
                        output wire       dumping,
                        output wire       tx
                        );
   localparam integer DEPTH = 1 << DEPTH_LOG2;

   // ---- local 2-flop sync of the raw pins ----------------------------------
   reg [1:0]          stb_s, prw_s, cmd_s;
   always @(posedge clk) begin
      stb_s <= {stb_s[0], pstrb_n};
      prw_s <= {prw_s[0], prw};
      cmd_s <= {cmd_s[0], pcmd_n};
   end
   wire [12:0] now = {bus_in, stb_s[1], prw_s[1], cmd_s[1], drive_en, host_stb};

   // ---- event store ---------------------------------------------------------
   reg [31:0]  mem [0:DEPTH-1];
   reg [DEPTH_LOG2-1:0] waddr;
   reg [31:0]           mem_q;
   // ONE synchronous read port, always through raddr. A second read
   // expression anywhere (prefetching mem[raddr+1], say) makes this a
   // three-port memory that no BRAM implements, so the whole 131072
   // bits fall back to flip-flops and the design stops placing. The
   // fetch flag below adds the single idle cycle a port needs.
   // The UART is five orders of magnitude slower anyway.
   reg                  fetch;

   // ---- 100 ns tick and per-event delta ------------------------------------
   reg [2:0]            pre = 3'd0;   // explicit: this counter is outside the rst block,
   // and an unreset reg is X in simulation forever
   reg [15:0]           dt;
   wire                 tick = (pre == 3'd4);
   always @(posedge clk) pre <= tick ? 3'd0 : pre + 3'd1;

   // ---- recorder ------------------------------------------------------------
   localparam S_WAIT = 2'd0, S_REC = 2'd1, S_DUMP = 2'd2, S_DONE = 2'd3;
   reg [1:0]  st;
   reg [12:0] last;
   reg [27:0] quiet;                 // event-free time; also the arm qualifier
   reg        have;
   reg        trig;      // stage 1: eight latched bytes = real protocol ran
   reg [8:0]  post;
   reg [2:0]  nstb;
   reg        wrapped;
   reg [17:0] noserve;   // 100 ns ticks since we last drove the bus
   reg        dump_go;

   // ---- dump engine ----
   reg [7:0]  ch;
   reg        ch_stb;
   wire       busy;
   uart_tx #(.CLK_HZ(CLK_HZ)) u_tx (
                                    .clk(clk), .rst(rst), .data(ch), .stb(ch_stb), .busy(busy), .tx(tx));
   assign dumping = (st == S_DUMP);

   function [7:0] hex(input [3:0] n);
      hex = (n < 4'd10) ? (8'h30 + n) : (8'h41 + n - 4'd10);
   endfunction

   reg [DEPTH_LOG2-1:0] raddr;
   reg [3:0]            col;                    // position within one dumped line
   reg [2:0]            hdr;                    // header/trailer progress

   always @(posedge clk) begin
      ch_stb <= 1'b0;
      if (rst) begin
         st <= S_WAIT; waddr <= 0; dt <= 0; quiet <= 0; have <= 0;
         raddr <= 0; col <= 0; hdr <= 0; last <= 13'h0; fetch <= 1'b0;
         trig <= 0; post <= 0; nstb <= 0; wrapped <= 0; noserve <= 0; dump_go <= 0;
      end else if (arm) begin
         // Re-arm: throw away the boot capture and start recording
         // again, so the one-shot can be aimed at an on-demand event
         // (a write burst) that happens long after boot triggered it.
         st <= S_REC; waddr <= 0; dt <= 0; quiet <= 0; have <= 0;
         raddr <= 0; col <= 0; hdr <= 0; last <= now; fetch <= 1'b0;
         trig <= 0; post <= 0; nstb <= 0; wrapped <= 0; noserve <= 0; dump_go <= 0;
      end else begin
         if (tick && dt != 16'hFFFF) dt <= dt + 16'd1;

         case (st)
           // CIRCULAR PRE-TRIGGER capture. The ring records
           // free-running and the TRIGGER does the aiming, so it
           // cannot be drowned by the pre-boot phase where PR/-W
           // flickers at CPU rate under 320 us /PSTRB-low stretches
           // (4096 events in ~2 ms). The trigger is the 8th host_stb,
           // which only fires for qualified latched host bytes. So
           // the handshake and command frame always sit at the ring's
           // recent end, and the flicker only occupies its old end.
           // 256 post-trigger events finish the capture. Then the ring
           // dumps in order, oldest first.
           S_WAIT: begin
              last <= now;
              st   <= S_REC;
           end

           S_REC: begin
              if (now != last) begin
                 mem[waddr] <= {dt, now[12:5], 3'b000, now[4:0]};
                 waddr <= waddr + 1'b1;
                 if (waddr == {DEPTH_LOG2{1'b1}}) wrapped <= 1'b1;
                 last  <= now;
                 dt    <= 0;
                 quiet <= 0;
                 have  <= 1'b1;
                 if (dump_go) begin
                    post <= post - 9'd1;
                    if (post == 9'd1) st <= S_DUMP;
                 end
              end else begin
                 quiet <= quiet + 28'd1;
                 // Fallback: real protocol ran but the air trigger
                 // never came.  Gated on trig, NOT on mere activity:
                 // a floating bus with the //e powered off generates
                 // events endlessly, and dumping on those would keep
                 // the UART hijacked and starve the reporter.
                 if (trig && quiet[27]) st <= S_DUMP;
              end
              if (host_stb && !trig) begin
                 if (nstb == 3'd7) trig <= 1'b1;
                 else              nstb <= nstb + 3'd1;
              end
              // Stage 2: 20 ms with NOTHING SERVED after real
              // protocol ran. A crashed host only pokes the bus
              // occasionally. Its CPU-activity flicker floods the ring
              // first, so an air-strobe trigger arrives too late to
              // catch the evidence. Silence of the DRIVE is the
              // anchor instead. When we have not driven for 20 ms
              // after having served real reads, the crash is at most
              // 20 ms old, and the ring still holds the last served
              // phase.
              if (drive_en) noserve <= 18'd0;
              else if (tick && !(&noserve)) noserve <= noserve + 18'd1;
              // Level, not edge: noserve can pass 200000 before trig
              // is even set (it counts from reset), then saturate. An
              // equality would never fire again. The saturation guard
              // keeps >= true once reached.
              if (trig && !dump_go && noserve >= 18'd200000) begin
                 dump_go <= 1'b1;
                 post    <= 9'd8;             // almost nothing more to wait for
              end
           end

           // One event per line: DDDD BB FF, 12 characters with the
           // separators and CRLF. Hex throughout. The decoder
           // reassembles it.
           S_DUMP: if (fetch) begin
              mem_q <= mem[raddr];
              fetch <= 1'b0;
           end else if (!busy && !ch_stb) begin
              if (hdr < 3'd4) begin
                 case (hdr)
                   3'd0: ch <= "C";
                   3'd1: ch <= "A";
                   3'd2: ch <= "P";
                   3'd3: ch <= "\n";
                 endcase
                 ch_stb <= 1'b1; hdr <= hdr + 3'd1;
                 // A wrapped ring's oldest event lives at waddr;
                 // unwrapped, at 0.
                 raddr <= wrapped ? waddr : {DEPTH_LOG2{1'b0}};
                 fetch <= 1'b1;
              end else begin
                 case (col)
                   4'd0: ch <= hex(mem_q[31:28]);
                   4'd1: ch <= hex(mem_q[27:24]);
                   4'd2: ch <= hex(mem_q[23:20]);
                   4'd3: ch <= hex(mem_q[19:16]);
                   4'd4: ch <= " ";
                   4'd5: ch <= hex(mem_q[15:12]);
                   4'd6: ch <= hex(mem_q[11:8]);
                   4'd7: ch <= " ";
                   4'd8: ch <= hex({3'b000, mem_q[4]});
                   4'd9: ch <= hex(mem_q[3:0]);
                   4'd10: ch <= "\n";
                   default: ch <= "\n";
                 endcase
                 ch_stb <= 1'b1;
                 if (col == 4'd10) begin
                    col <= 4'd0;
                    if (raddr == waddr - 1'b1) begin   // full circle: oldest..newest
                       st <= S_DONE; quiet <= 0;
                    end else begin
                       raddr <= raddr + 1'b1;
                       fetch <= 1'b1;
                    end
                 end else begin
                    col <= col + 4'd1;
                 end
              end
           end

           // Re-arm rather than holding. A reflash reconfigures the
           // FPGA while the //e sits in its failed state. A one-shot
           // recorder would arm against that stale state, dump a
           // capture of an idle bus, and then miss the boot that
           // matters.
           // Cycling means the LAST dump in the serial log is always
           // the most recent burst of bus activity, which after a
           // power cycle is the boot attempt itself.
           S_DONE: begin
              quiet <= quiet + 28'd1;
              if (quiet[24]) begin       // ~0.3 s pause, then listen again
                 st <= S_WAIT; waddr <= 0; raddr <= 0; col <= 0; hdr <= 0;
                 have <= 0; quiet <= 0; dt <= 0;
                 trig <= 0; post <= 0; nstb <= 0; wrapped <= 0; noserve <= 0; dump_go <= 0;
              end
           end
         endcase
      end
   end

endmodule
