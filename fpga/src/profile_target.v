// profile_target: the three-phase ProFile command protocol, drive side.
//
// Written from PROTOCOL.md (profile-dumper): section 2 for the
// handshake, section 3 for the command frame and the response bytes,
// section 9.1 and 9.3 for what the Apple //e interface card actually
// does rather than what the spec says.
//
// Phase structure:
//   1 command      host asserts /PCMD -> we present $01 and release /PBSY ->
//                  host acks $55 -> host clocks in 6 command bytes
//   2 confirmation host asserts /PCMD -> we present the intent byte -> host
//                  acks $55 (anything else is a NAK; the //e sends $AA)
//   3 data         read: 4 status + 512 data, then the host ABANDONS by
//                  re-asserting /PCMD (it never takes the 20 tag bytes)
//                  write: 512 data + a 6-byte trailer, then a second
//                  confirmation ($06), then 4 status
//
// Deliberate non-goals, matching the real drive (PROTOCOL.md
// (profile-dumper) section 9.3): we do not count the host's
// data-phase bytes as a correctness check. A real ProFile cannot,
// because the host clocks bytes straight into its buffer RAM through
// a free-running address counter.
//
// Simulated against the reader's host sequence (../sim/tb_top.v) and,
// more importantly, against the //e's stricter one
// (../sim/tb_top_iie.v). Ordinary blocks come off the card through
// the FAT32 resolver and $FFFFFF, $FFFFFE and the selector window
// are synthesized.

`default_nettype none

module profile_target #(
                        parameter [7:0]   ACK = 8'h55,
                        parameter [7:0]   NAK = 8'hAA,
                        parameter [7:0]   RSP_CMD = 8'h01,  // "send me the command frame"
                        parameter [7:0]   RSP_READ = 8'h02, // read confirm
                        parameter [7:0]   RSP_WR = 8'h03,   // write: receive data
                        parameter [7:0]   RSP_WRV = 8'h04,  // write/verify: receive data
                        parameter [7:0]   RSP_GO = 8'h06,   // second write confirm: perform it
                        parameter integer DATA_BYTES = 512,
                        parameter integer TAG_BYTES = 20,   // UNUSED: the //e abandons before the tag
                        // bytes and the phase ends on /PCMD, so no
                        // count of them is ever needed
                        parameter integer TRAILER_BYTES = 6,
                        parameter integer STATUS_BYTES = 4,
                        // Give up on a data phase the host has walked away
                        // from.
                        //
                        // The phases below end on /PCMD, which is
                        // correct when the host starts its next
                        // transaction. It is NOT enough when the host
                        // simply stops strobing: the //e's card takes
                        // 4 status + 512 data and then stops. Without
                        // this the drive sits in S_RD_DATA holding
                        // drive_en, not listening, still owning the
                        // bus.
                        //
                        // 50_000 cycles is 1 ms at 50 MHz: two orders
                        // of magnitude beyond the //e's ~9 us between
                        // bytes, so it can never fire mid-transfer.
                        parameter integer PHASE_TIMEOUT = 50_000,
                        // A NAK aborts the transaction, which is what
                        // a real drive does and what keeps us in step
                        // with the card ROM's retry: it restarts at
                        // phase 1 ($CBB2 BNE $CB80). The ROM this was
                        // developed against is 341-0299-B in a //e.
                        // The ROM addresses cited throughout this
                        // file are from it. Carrying on into the data
                        // phase instead makes the retry's write
                        // strobes get counted as read bytes. 0
                        // carries on regardless, matching the Teensy
                        // build's 'n' switch.
                        parameter         NAK_ABORTS = 1'b1,
                        // Drop /PBSY as soon as /PCMD is seen,
                        // WITHOUT waiting for PR/-W to say the host
                        // has turned the bus around.
                        //
                        // /PBSY and the data bus are two different
                        // signals. Driving the '245 into a host that
                        // is still driving is a bus fight, so the
                        // DATA bus waits for PR/-W (S_RESP). /PBSY
                        // is our own pin and driving it low fights
                        // nobody, so it can answer at once. A host
                        // that waits for /PBSY before turning the bus
                        // around would otherwise deadlock with a
                        // drive waiting for the turnaround. The host
                        // breaks that deadlock by abandoning the
                        // command and retrying. 0 waits for PR/-W
                        // before touching /PBSY as well.
                        parameter         EARLY_BSY = 1'b1
                        ) (
                           input  wire        clk,
                           input  wire        rst,

                           // --- pin layer ---
                           input  wire        pcmd_n,
                           output reg         pbsy_n,
                           output reg         drive_en,
                           output reg  [7:0]  drive_data,
                           input  wire [7:0]  host_data,
                           input  wire        host_stb,
                           input  wire        read_stb,
                           input  wire [7:0]  bus_in,      // unstrobed view of the bus, for the ack
                           input  wire        host_read,   // synchronized PR/-W, 1 = host reading

                           // --- block plumbing ---
                           output reg  [23:0] blk_addr,
                           output reg         blk_rd,        // pulse: fetch blk_addr into the buffer
                           output reg         blk_wr,        // pulse: commit the buffer to blk_addr
                           input  wire        blk_ready,     // buffer holds/accepted the block
                           output reg  [9:0]  buf_addr,
                           input  wire [7:0]  buf_rdata,
                           output reg  [7:0]  buf_wdata,
                           output reg         buf_we,

                           input  wire [7:0]  status0, status1, status2, status3,

                           // High while a transaction is in progress,
                           // so the rest of the design knows to keep
                           // off the shared block-buffer read port.
                           output wire        tgt_busy,

                           // The opcode of the frame just received,
                           // valid while ev_cmd pulses. Taken
                           // straight from the frame rather than from
                           // the registered `opcode`, which is only
                           // being assigned on that same cycle and so
                           // still holds the previous command's value
                           // when a listener samples it.
                           output wire [7:0]  blk_op,

                           // --- telemetry ---
                           output reg         ev_nak,
                           output reg         ev_badop,      // an opcode we do not implement
                           output reg         ev_cmd,        // a command frame was decoded
                           output reg         ev_abandon,
                           output reg         ev_read_ok,
                           output reg         ev_write_ok,

                           // The six command-frame bytes exactly as
                           // they were latched, before any
                           // decode. When badops rise this is the
                           // only thing that separates a bit-order
                           // fault from a timing fault from a framing
                           // fault: the counters say only that the
                           // frame was bad, and this says how.
                           output wire [47:0] dbg_frame,
                           input  wire [7:0]  host_data_lead,
                           output reg  [7:0]  dbg_f0_lead,   // byte 0 as seen at the LEADING edge
                           output wire [2:0]  dbg_cmd_idx,
                           // Cycles from the command phase opening to
                           // its FIRST strobe.
                           output reg  [15:0] dbg_t0,
                           // The last five FSM transitions, newest in
                           // the low byte. One byte per transition:
                           // {new state[3:0], pcmd_asserted,
                           // host_read, stb_idle, drive_en} captured
                           // as the state is entered, so the walk
                           // need not be reconstructed from counters.
                           output reg  [39:0] dbg_walk,
                           // The LAST BAD frame, latched whole at the
                           // moment ev_badop fires. dbg_frame is
                           // re-seeded to $EE at every /PCMD assert,
                           // so by the time a report line goes out,
                           // the frame that produced a badop has
                           // already been wiped. This register
                           // survives: it holds the six bytes exactly
                           // as the decode saw them, which is the
                           // difference between knowing THAT frames
                           // decode as $FF and knowing WHAT was in
                           // them.
                           output reg  [47:0] dbg_badframe,
                           // Microseconds between the six counted
                           // strobes of that same bad frame: five
                           // gaps, 8 bits each, capped at 255, oldest
                           // gap in the top byte. The ROM's
                           // inter-byte gap is ~15 us; ack-to-opcode
                           // is ~39-53 us. So the gap PATTERN
                           // identifies which strobes were actually
                           // counted:
                           //   [40+ 15 15 15 15]  the ACK strobe was
                           //                      counted as byte 0
                           //   [15 15 15 15 BIG]  the opcode strobe was
                           //                      MISSED; the count
                           //                      starts at byte 1
                           //   [15 15 15 15 15]   all six frame strobes
                           //                      counted; the values
                           //                      are the issue
                           output reg  [39:0] dbg_dt,
                           input  wire        stb_idle,
                           output wire        trace_arm
                           );

   localparam [3:0]
                   S_IDLE      = 4'd0,   // waiting for /PCMD
                   S_RESP      = 4'd1,   // /PCMD seen: wait for PR/-W high before driving
                   S_RESP_PRE  = 4'd12,  // byte presented, waiting for the host to take it
                   S_ACK       = 4'd2,   // bus released, sampling the ack until /PCMD rises
                   S_CMDBYTES  = 4'd3,   // six command bytes inbound
                   S_CMDDRAIN  = 4'd14,  // ...but not until /PSTRB is idle
                   S_DECODE    = 4'd4,
                   S_CONFIRM   = 4'd5,
                   S_RD_STATUS = 4'd6,   // 4 status bytes outbound
                   S_RD_DATA   = 4'd7,   // 512 data bytes outbound, then abandon
                   S_WR_DATA   = 4'd8,   // 512 + 6 trailer inbound
                   S_WR_GO     = 4'd9,   // second confirmation, response $06
                   S_WR_STATUS = 4'd10,
                   S_NAKED     = 4'd11,
                   S_TURN      = 4'd13;  // wait for the host to hand the bus back

   reg [3:0]       st;

   // Declared after st and the state constants, before first use. yosys
   // accepts a forward reference without a warning. iverilog rejects it.
   assign tgt_busy = (st != S_IDLE);
   reg [2:0]  cmd_idx;
   reg [16:0] to_cnt;      // cycles since the host last strobed
   reg [15:0] t0_cnt;      // cycles since the command phase opened
   reg        t0_run;
   reg [7:0]  cmd_frame [0:5];       // op, blk hi/mid/lo, retry, sparing threshold

   // Declared after cmd_frame, not before it.
   assign blk_op = cmd_frame[0];
   assign dbg_cmd_idx = cmd_idx;
   // Trace the WHOLE command phase, drain included, so a strobe arriving
   // before byte 0 is counted rather than being stored as byte 0.
   assign trace_arm = (st == S_CMDBYTES) || (st == S_CMDDRAIN);
   assign dbg_frame = {cmd_frame[0], cmd_frame[1], cmd_frame[2],
                       cmd_frame[3], cmd_frame[4], cmd_frame[5]};
   reg [10:0] xfer;                  // data-phase byte counter
   reg [7:0]  pending_rsp;
   reg [7:0]  opcode;
   reg [7:0]  ack_byte;
   reg        turn_to_wr;               // S_TURN's destination: 0 = read, 1 = write status
   // A write must not report "done" until the SD commit that blk_wr
   // started has actually finished. A real ProFile holds the host off
   // (BSY) through its internal platter write and only then answers
   // the "perform it" confirm.  Firing blk_wr and returning status
   // immediately lets the next write in a burst race the
   // still-in-flight commit over the single shared block buffer. One
   // of them is then dropped with no error. wr_saw_busy latches once
   // blk_ready has dropped after blk_wr, so S_WR_GO waits for the
   // FRESH blk_ready rather than the stale one the previous
   // transaction left high.
   //
   // wr_settle exists because not every write reaches the SD. A
   // scratch-buffer ($FFFFFE), magic-window, or refused write is
   // answered by block_router with blk_ready held HIGH and never
   // dropped. So "wait for drop then rise" would hang forever. A real
   // SD commit ALWAYS drops blk_ready within a couple of cycles of
   // blk_wr, so if it is still high after this many cycles there is
   // nothing to wait for and the write may complete at once. Short
   // enough to be invisible on a real write, long enough to cover
   // block_router's request latency.
   reg        wr_saw_busy;
   reg [4:0]  wr_settle;
   localparam [4:0] WR_SETTLE = 5'd16;

   // /PCMD synchronizer. A real command holds /PCMD asserted for
   // milliseconds, while the card's '259 can glitch-latch it
   // (PROTOCOL.md (profile-dumper) section 9.1), so this is debounced
   // rather than edge-detected raw. At 50 MHz, 6 us = 300.
   localparam integer PCMD_DEBOUNCE = 300;
   reg [8:0]          pcmd_cnt;
   reg [1:0]          pcmd_sync;
   reg                pcmd_stable;
   always @(posedge clk) begin
      // RESET TO THE IDLE LEVEL, not to zero. /PCMD is active low, so
      // an unreset pcmd_stable comes out of configuration at 0, which
      // reads as "the host is commanding". The FSM leaves S_IDLE
      // immediately, asserts BSY, and turns the '245 around to drive a
      // bus it does not own. It only discovers its mistake
      // PCMD_DEBOUNCE cycles later. That is a 6 us contention window at
      // every power-up.
      if (rst) begin
         pcmd_sync   <= 2'b11;
         pcmd_stable <= 1'b1;      // deasserted
         pcmd_cnt    <= 0;
      end else begin
         pcmd_sync <= {pcmd_sync[0], pcmd_n};
         if (pcmd_sync[1] == pcmd_stable) begin
            pcmd_cnt <= 0;
         end else if (pcmd_cnt == PCMD_DEBOUNCE-1) begin
            pcmd_stable <= pcmd_sync[1];
            pcmd_cnt    <= 0;
         end else begin
            pcmd_cnt <= pcmd_cnt + 1'b1;
         end
      end
   end
   wire pcmd_asserted = ~pcmd_stable;

   // Strobe-gap recorder; see dbg_dt above. A 1 us tick, a per-gap
   // counter capped at 255, shifted in at each counted frame byte.
   reg [5:0] us_pre;
   reg [7:0] gap_us;
   reg [39:0] dt_acc;
   always @(posedge clk) begin
      if (rst) begin
         us_pre <= 6'd0; gap_us <= 8'd0; dt_acc <= 40'd0;
      end else begin
         us_pre <= (us_pre == 6'd49) ? 6'd0 : us_pre + 6'd1;
         if (us_pre == 6'd49 && gap_us != 8'hFF) gap_us <= gap_us + 8'd1;
         if (st == S_CMDBYTES && host_stb) begin
            // one gap per counted byte; the first byte's "gap" (since
            // phase start) is shifted through and falls off the top
            // by byte 5
            dt_acc <= {dt_acc[31:0], gap_us};
            gap_us <= 8'd0;
         end
         if (st == S_IDLE) begin
            gap_us <= 8'd0;
         end
      end
   end

   // FSM walk recorder; see dbg_walk above.
   reg [3:0] walk_st_q;
   always @(posedge clk) begin
      if (rst) begin
         dbg_walk  <= 40'h0;
         walk_st_q <= 4'd0;
      end else begin
         walk_st_q <= st;
         if (st != walk_st_q)
           dbg_walk <= {dbg_walk[31:0],
                        st, pcmd_asserted, host_read, stb_idle, drive_en};
      end
   end

   always @(posedge clk) begin
      // one-cycle telemetry pulses
      ev_nak <= 1'b0; ev_abandon <= 1'b0; ev_read_ok <= 1'b0; ev_write_ok <= 1'b0;
      ev_badop <= 1'b0; ev_cmd <= 1'b0;
      blk_rd <= 1'b0; blk_wr <= 1'b0; buf_we <= 1'b0;

      if (rst) begin
         st <= S_IDLE; drive_en <= 1'b0; pbsy_n <= 1'b1; cmd_idx <= 0; xfer <= 0;
         to_cnt <= 17'd0; wr_saw_busy <= 1'b0; wr_settle <= 5'd0;
         t0_cnt <= 16'd0; t0_run <= 1'b0;
         dbg_f0_lead <= 8'h00;
         dbg_badframe <= 48'h0;
         drive_data <= 8'h00; buf_addr <= 0;
      end else case (st)

                 // ---- command phase ----
                 S_IDLE: begin
                    drive_en <= 1'b0;
                    pbsy_n   <= 1'b1;                        // not busy
                    if (pcmd_asserted) begin
                       pending_rsp <= RSP_CMD;
                       // $EE sentinel into every frame slot. After
                       // the attempt, a slot still reading $EE means
                       // NO strobe was ever counted for it, which is
                       // different information from $FF (a strobe WAS
                       // counted and the bus read undriven).  The
                       // received-frame dump depends on the
                       // difference: it is what separates a
                       // one-strobe shift from a missing byte.
                       cmd_frame[0] <= 8'hEE; cmd_frame[1] <= 8'hEE; cmd_frame[2] <= 8'hEE;
                       cmd_frame[3] <= 8'hEE; cmd_frame[4] <= 8'hEE; cmd_frame[5] <= 8'hEE;
                       // Answer on /PBSY straight away. This
                       // assignment comes after the pbsy_n <= 1'b1
                       // above and so overrides it.  The data bus
                       // stays parked until S_RESP sees PR/-W: only
                       // /PBSY moves early.
                       if (EARLY_BSY) pbsy_n <= 1'b0;
                       st          <= S_RESP;
                    end
                 end

                 // ---- the response handshake ----
                 // Three waits, mirroring the Teensy build's
                 // respondAndAck. Two things a host does not do: it
                 // does not free the bus the moment it asserts /PCMD,
                 // so the response is not driven until PR/-W says it
                 // has turned around; and it does not strobe the ack,
                 // so the ack is sampled, never waited for as a strobe.
                 //
                 //   1 S_RESP do not touch the DATA bus until PR/-W
                 //                 says the host has turned around to
                 //                 read. Driving before that points
                 //                 the '245 at a host that is still
                 //                 driving. /PBSY is exempt: it is
                 //                 our own pin, it fights nobody, and
                 //                 with EARLY_BSY it has already
                 //                 answered back in S_IDLE. See the
                 //                 EARLY_BSY parameter for why that
                 //                 matters.
                 //   2 S_RESP_PRE byte and BSY out; wait for PR/-W to
                 //                 fall, which is the host taking the
                 //                 byte and turning around to
                 //                 answer. If /PCMD releases while
                 //                 PR/-W is still high the host has
                 //                 ABANDONED the command without
                 //                 reading; drop it at once rather
                 //                 than sitting on the bus.
                 //   3 S_ACK release the bus, sample the ack
                 //                 CONTINUOUSLY (the host parks $55
                 //                 and never strobes it), and finish
                 //                 when /PCMD rises. Do NOT gate on
                 //                 the value: the //e's DMA ack is
                 //                 not a clean $55 to a simple
                 //                 sampler, and BSY is what keeps the
                 //                 phases in step. ev_nak is
                 //                 telemetry only.
                 S_RESP: if (host_read) begin
                    drive_data <= pending_rsp;
                    drive_en   <= 1'b1;                      // byte is on the bus BEFORE
                    pbsy_n     <= 1'b0;                      // BSY drops: no race, ever
                    ack_byte   <= 8'hFF;
                    st         <= S_RESP_PRE;
                 end else if (!pcmd_asserted) begin
                    st <= S_IDLE;                            // withdrawn before we answered
                 end

                 S_RESP_PRE: begin
                    if (!host_read) begin
                       drive_en <= 1'b0;                      // host has it; turn around
                       st       <= S_ACK;
                    end else if (!pcmd_asserted) begin
                       ev_abandon <= 1'b1;
                       drive_en   <= 1'b0;
                       pbsy_n     <= 1'b1;
                       st         <= S_IDLE;
                    end
                 end

                 S_ACK: begin
                    if (!host_read) ack_byte <= bus_in;      // no strobe: just watch it
                    if (!pcmd_asserted) begin
                       pbsy_n <= 1'b1;                        // release BSY: the host waits
                       ev_nak <= (ack_byte != ACK);
                       // Abort ONLY on a positively-read $AA, exactly
                       // as the Teensy build does
                       // (emulator.ino:751: `if (lastAck == 0xAA)`),
                       // NOT on anything that is not $55.
                       //
                       // The difference matters on this card, and the
                       // card map says why: the ack write is "$CEB8:
                       // STA $C080 then $C503": the strobe fires
                       // BEFORE $C503 enables the 374, so at the
                       // strobe the $55 is not driven at all. It only
                       // appears on the bus in the few-microsecond
                       // window between $C503 landing and byte 0's
                       // write. Continuous sampling here can miss
                       // that window, so the ack can read $FF.
                       // Aborting on that would end a transaction the
                       // host considers fine. The host proceeds to
                       // the confirm phase, which a drive that has
                       // gone back to S_IDLE would misparse as a NEW
                       // command phase.
                       //
                       // An UNREADABLE ack is therefore normal on
                       // this card.  Only the //e's deliberate $AA
                       // (the $CDAC mismatch write) aborts. $AA
                       // gets the same post-$C503 visibility window
                       // that $55 does, so when the ROM means NAK we
                       // can actually read it.
                       if (NAK_ABORTS && ack_byte == NAK) begin
                          drive_en <= 1'b0;
                          st       <= S_IDLE;
                       end else
                         case (pending_rsp)
                           // Byte 0 must begin on a FRESH falling
                           // edge, so go through S_CMDDRAIN rather
                           // than straight to S_CMDBYTES. The card
                           // can strobe before byte 0 exists: the bus
                           // reads $FF right through that pulse, past
                           // the end of a 355 ns strobe, which means
                           // the pulse is not byte 0's strobe at all.
                           // Draining it is what keeps the frame from
                           // being taken one byte early.
                           RSP_CMD: begin cmd_idx <= 0;
                              t0_cnt <= 16'd0; t0_run <= 1'b1;
                              st <= S_CMDDRAIN; end
                           RSP_READ: begin xfer <= 0; turn_to_wr <= 1'b0; st <= S_TURN; end
                           RSP_WR, RSP_WRV: begin xfer <= 0; st <= S_WR_DATA; end
                           // blk_wr already fired when the data phase
                           // ended (S_WR_DATA) and the commit has
                           // been waited out in S_WR_GO, so here just
                           // send status.
                           RSP_GO: begin xfer <= 0; turn_to_wr <= 1'b1; st <= S_TURN; end
                           default: st <= S_IDLE;
                         endcase
                    end
                 end

                 // Byte 0 must begin on a FRESH falling edge. The
                 // host's ack write pulses /PSTRB while /PCMD is
                 // still asserted, and if that pulse is still low
                 // when the handshake ends, the first edge this state
                 // sees belongs to the ack, not to the frame: every
                 // byte then lands one place early and the opcode is
                 // whatever was on the bus during the ack. The
                 // working Teensy build drains the same strobe for
                 // the same reason (emulator.ino, recvBytes, "the
                 // op=$55 artifact"). Wait for a clean idle strobe
                 // line, and only then start the frame: stb_idle
                 // refuses to start in the middle of a strobe that is
                 // already in progress.
                 S_CMDDRAIN: begin
                    if (stb_idle) st <= S_CMDBYTES;
                 end

                 S_CMDBYTES: if (host_stb) begin
                    // Accept EVERY strobed byte, $FF included, as the
                    // Teensy build does. On this card the FIRST
                    // handshake of a block can never complete
                    // cleanly, because the ack is invisible (no PR/-W
                    // drop: Q1 gates FF2; no data: $CEB8 strobes
                    // before $C503 enables the 374). The reference
                    // implementation boots exclusively through the
                    // retry cascade: one abandon, one $FF-badop
                    // ABORT, then the third open succeeds. The badop
                    // abort IS the resynchronization step. Filtering
                    // leading $FF bytes as phantoms would remove the
                    // abort that realigns the phases and rotate real
                    // bytes into the frame. Do not filter by value.
                    cmd_frame[cmd_idx] <= host_data;
                    if (cmd_idx == 3'd0) dbg_f0_lead <= host_data_lead;
                    if (cmd_idx == 3'd5) st <= S_DECODE;
                    else                 cmd_idx <= cmd_idx + 1'b1;
                 end

                 // --------------------------------------------------
                 // confirmation
                 S_DECODE: begin
                    opcode   <= cmd_frame[0];
                    blk_addr <= {cmd_frame[1], cmd_frame[2], cmd_frame[3]};
                    ev_cmd   <= 1'b1;                        // clears "drive was reset"
                    case (cmd_frame[0])
                      8'h00: begin pending_rsp <= RSP_READ; blk_rd <= 1'b1; st <= S_CONFIRM; end
                      8'h01: begin pending_rsp <= RSP_WR;  st <= S_CONFIRM; end
                      8'h02: begin pending_rsp <= RSP_WRV; st <= S_CONFIRM; end
                      // Anything else is abandoned without answering,
                      // which is what the Teensy build does. $03 in
                      // particular is Force Sparing: undocumented,
                      // DESTRUCTIVE, and emphatically not a dry
                      // run. It allocates a spare and writes to
                      // cylinder 77 on a real drive. Answering it
                      // politely and failing it in status would still
                      // have entered a write data phase, so it is
                      // refused here, before the confirmation.
                      default: begin
                         ev_badop <= 1'b1;
                         dbg_badframe <= {cmd_frame[0], cmd_frame[1], cmd_frame[2],
                                          cmd_frame[3], cmd_frame[4], cmd_frame[5]};
                         dbg_dt       <= dt_acc;
                         drive_en <= 1'b0;
                         pbsy_n   <= 1'b1;
                         st       <= S_IDLE;
                      end
                    endcase
                 end

                 // The intent byte is presented when the block is
                 // READY, and a parked read (resolver mid-rescan
                 // after a menu SELECT) can hold that off for 50-400
                 // ms. The host's confirm wait is shorter: it
                 // abandons and runs its retry cascade. This state
                 // must LEAVE when that happens. Staying here until
                 // blk_ready would miss the $FF abort frame and every
                 // retry's $01 request. The ROM would then use up its
                 // whole 10-retry budget against a drive that never
                 // answers.
                 //
                 // ABANDON DETECTION. It is NOT enough to bail when
                 // /PCMD was seen asserted and then released. On
                 // the real //e that fires on EVERY read, because the
                 // ROM's confirm wait is a POLL. It releases and
                 // re-asserts /PCMD while waiting for the intent
                 // byte, so /PCMD dropping is NOT a reliable "the
                 // host gave up" signal (the sim host holds it, which
                 // is why benches do not show this). The signature
                 // that does mean it is the host DRIVING bytes at us:
                 // a confirm wait only ever reads (PR/-W high), so a
                 // write strobe here is the retry cascade's next
                 // frame already in flight. Yield to it.
                 S_CONFIRM: begin
                    if (pcmd_asserted && (opcode != 8'h00 || blk_ready)) begin
                       st <= S_RESP;                          // S_RESP presents pending_rsp
                    end else if (host_stb && !host_read) begin
                       ev_abandon <= 1'b1;
                       st         <= S_IDLE;                  // answer the new frame, alive
                    end
                 end

                 // The ack ends when /PCMD rises, but the host is
                 // STILL DRIVING the bus at that moment: the reader
                 // holds its ack byte until releaseBus() several
                 // microseconds later (profile-dumper.ino step [16]),
                 // and the //e is no different. Driving the data
                 // phase the instant the handshake ends points the
                 // '245 into a live host driver. Wait for PR/-W to
                 // say the bus is ours.
                 //
                 // ONCE, here, and not again: from S_RD_STATUS onward
                 // drive_en is held regardless of PR/-W, because
                 // PR/-W carries sub-microsecond skew on every read
                 // strobe and following it per byte is what loses
                 // byte 0 (see profile_bus.v and PROTOCOL.md
                 // (profile-dumper) section 3).
                 S_TURN: if (host_read) st <= turn_to_wr ? S_WR_STATUS : S_RD_STATUS;

                 // ---- read data phase ----
                 S_RD_STATUS: begin
                    drive_en <= 1'b1;
                    case (xfer[1:0])
                      2'd0: drive_data <= status0;
                      2'd1: drive_data <= status1;
                      2'd2: drive_data <= status2;
                      default: drive_data <= status3;
                    endcase
                    if (read_stb) begin
                       if (xfer == STATUS_BYTES-1) begin xfer <= 0; buf_addr <= 0; st <= S_RD_DATA; end
                       else                              xfer <= xfer + 1'b1;
                    end
                 end

                 // The host abandons after DATA_BYTES by re-asserting
                 // /PCMD and never takes the TAG_BYTES. That is
                 // NORMAL: end the phase on /PCMD, not on a byte
                 // count and not on a timeout (PROTOCOL.md
                 // (profile-dumper) section 9.1).
                 S_RD_DATA: begin
                    drive_en   <= 1'b1;
                    drive_data <= buf_rdata;
                    if (read_stb) begin
                       buf_addr <= buf_addr + 1'b1;
                       xfer     <= xfer + 1'b1;
                    end
                    if (pcmd_asserted) begin
                       ev_abandon <= 1'b1;
                       ev_read_ok <= (xfer >= DATA_BYTES);
                       drive_en   <= 1'b0;
                       st         <= S_IDLE;
                    end
                 end

                 // ---- write data phase ----
                 // DATA_BYTES then TRAILER_BYTES. The trailer is a
                 // block-number echo the card emits before its own parity
                 // check, and the phase has to consume it too (PROTOCOL.md
                 // (profile-dumper) section 9.3).
                 S_WR_DATA: if (host_stb) begin
                    if (xfer < DATA_BYTES) begin
                       buf_wdata <= host_data;
                       buf_addr  <= xfer[9:0];
                       buf_we    <= 1'b1;
                    end
                    if (xfer == DATA_BYTES + TRAILER_BYTES - 1) begin
                       // The full block is in the buffer (only bytes
                       // 0..511 are stored; the trailer is
                       // not). Start the SD commit NOW and go wait it
                       // out before telling the host the write is
                       // done.
                       blk_wr      <= 1'b1;
                       wr_saw_busy <= 1'b0;
                       wr_settle   <= 5'd0;
                       pending_rsp <= RSP_GO;
                       st          <= S_WR_GO;
                    end else begin
                       xfer <= xfer + 1'b1;
                    end
                 end

                 // Hold here, NOT driving, until the SD commit blk_wr
                 // started has finished. The ROM's "perform it" poll
                 // ($CDAC with A=$06, whose $CE16 waits on the BSY
                 // sense) sits spinning on $C082 through this whole
                 // window, exactly as it would against a real drive
                 // mid-write. pbsy_n stays high (not ready) because
                 // this state drives nothing. blk_ready must be seen
                 // LOW first (this commit started) and then HIGH (it
                 // finished): keying on the bare level would accept
                 // the 1 the previous transaction left behind and
                 // defeat the wait entirely.
                 S_WR_GO: begin
                    if (!blk_ready)            wr_saw_busy <= 1'b1;
                    if (wr_settle != WR_SETTLE) wr_settle  <= wr_settle + 5'd1;
                    // Ready to answer when the host is asking (pcmd)
                    // AND the commit is done: either it visibly ran
                    // (blk_ready dropped and is back up) or it was a
                    // no-SD write that block_router answered
                    // instantly (blk_ready never dropped, so fall
                    // through once the settle window has passed).
                    if (pcmd_asserted && blk_ready &&
                        (wr_saw_busy || wr_settle == WR_SETTLE)) begin
                       pending_rsp <= RSP_GO;
                       st          <= S_RESP;
                    end
                 end

                 S_WR_STATUS: begin
                    drive_en <= 1'b1;
                    case (xfer[1:0])
                      2'd0: drive_data <= status0;
                      2'd1: drive_data <= status1;
                      2'd2: drive_data <= status2;
                      default: drive_data <= status3;
                    endcase
                    if (read_stb) begin
                       if (xfer == STATUS_BYTES-1) begin
                          ev_write_ok <= 1'b1;
                          drive_en    <= 1'b0;
                          st          <= S_IDLE;
                       end else xfer <= xfer + 1'b1;
                    end
                 end

                 S_NAKED: begin
                    drive_en <= 1'b0;
                    pbsy_n   <= 1'b1;
                    if (!pcmd_asserted) st <= S_IDLE;
                 end

                 default: st <= S_IDLE;
               endcase

      // ---- when did the first command-phase strobe arrive? -----------------
      if (t0_run) begin
         if (host_stb) begin dbg_t0 <= t0_cnt; t0_run <= 1'b0; end
         else if (t0_cnt != 16'hFFFF) t0_cnt <= t0_cnt + 16'd1;
      end

      // ---- the host walked away -------------------------------------------
      // Any strobe, in either direction, means it is still
      // there. Silence for PHASE_TIMEOUT in a phase where we own the
      // bus means it is not, and holding drive_en any longer makes us
      // deaf to its next command frame.  Placed after the case so it
      // wins: releasing the bus is never wrong here.
      if (host_stb || read_stb || st == S_IDLE) to_cnt <= 17'd0;
      else if (to_cnt != PHASE_TIMEOUT[16:0])   to_cnt <= to_cnt + 17'd1;

      if (to_cnt == PHASE_TIMEOUT[16:0] &&
          (st == S_RD_STATUS || st == S_RD_DATA ||
           st == S_WR_DATA   || st == S_WR_STATUS || st == S_TURN)) begin
         ev_abandon <= 1'b1;
         drive_en   <= 1'b0;
         pbsy_n     <= 1'b1;
         to_cnt     <= 17'd0;
         st         <= S_IDLE;
      end
   end

endmodule

`default_nettype wire
