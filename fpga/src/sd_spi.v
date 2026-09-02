// sd_spi: SD card in SPI mode. Initialization, CMD17 single-block
// reads and CMD24 single-block writes.
//
// BOTH CARD GENERATIONS. An SDHC/SDXC card addresses by block, so an
// LBA is a sector number and no arithmetic is needed. A card that
// rejects CMD8 is taken for v1: ACMD41 goes out without HCS, CCS is
// read from the OCR, and the CMD17 and CMD24 arguments are scaled by
// 512. `make sim-v1` and `make sim-v1skew` cover that path.
//
// It also recovers byte alignment when a card's replies arrive out of
// phase, which the real one does. `make sim-sweep` checks that recovery
// over all eight phases.
//
// No filesystem here: src/fat32.v does the MBR, the BPB, the
// directory walk and the cluster chain in hardware, above this layer,
// and turns a ProFile block into a sector before it arrives. LBA_OFF
// remains for serving a raw image at a fixed offset with the
// filesystem compiled out.
//
// Clocking. One divider, several speeds, all set from top.v rather
// than from the defaults on the parameters below: 100 kHz while the
// card is brought up, then 8.33 MHz for reads and 391 kHz for
// writes. Those two are measured ceilings on this harness, which
// should be fixed on a production PCB. Re-measure with SWEEP_ENABLE
// before changing either.
//
// SPI mode 0: MOSI is set up while SCLK is low, the card samples on
// the rising edge, and we sample MISO on that same rising edge.

`default_nettype none

module sd_spi #(
                parameter integer CLK_HZ   = 50_000_000,
                parameter integer LBA_OFF  = 0,     // sector offset of the image
                parameter integer DIV_INIT = 250,   // half-period in clocks: 100 kHz at 50 MHz
                parameter integer DIV_FAST = 2,     // UNUSED: the post-init rate arrives on the
                // `speed` port at run time (top.v drives it from
                // SD_DIV_FAST).  Nothing here reads this. Writes get
                // their own divider. A read puts six command bytes on
                // MOSI. A write puts 512 back to back, so MOSI is
                // exercised far harder and may fail at a rate that
                // works for reads.
                parameter integer DIV_WRITE = 63,
                // How long to wait for the $FE data token, in CLOCK
                // CYCLES rather than in byte times. The SD spec
                // allows a card up to 100 ms of read access time
                // (Nac), and a byte-counted budget does not mean a
                // fixed amount of that: at the 100 kHz init rate 255
                // polls is 20.5 ms, but at 8.33 MHz the very same 255
                // polls is 0.245 ms, which is 400x short of what the
                // card is entitled to take. A card that occasionally
                // pauses for internal housekeeping is then not waited
                // for long enough, and only at the fast clock. The
                // failure is rare and transient, and it looks like a
                // signal-integrity problem. 5_000_000 cycles = 100 ms
                // at 50 MHz.
                parameter integer TOKEN_TIMEOUT_CYC = 5_000_000,
                // How many times to re-issue a transfer that failed
                // before giving up on it. A real drive retries, and
                // the ProFile command frame even carries a retry
                // count of its own (PROTOCOL.md section 3, byte 4,
                // usually $64). A single transient must not be
                // allowed to reach the //e as a bad block.
                parameter integer RETRY_MAX = 3,
                // WHERE TO SAMPLE MISO. Mode 0 says the card drives
                // on the falling edge and the host samples on the
                // rising one, which is SAMPLE_LATE=0 and the setting
                // used.
                //
                // The edge matters because MISO carries a pull-up and
                // its rising edge is slow. Sampled too early, every
                // bit reads as the PREVIOUS one: a uniform one-bit
                // lag. The bit-level search that finds R1 is
                // lag-invariant and survives that. The R7 bytes after
                // it are read by byte counting and do not, so $1AA
                // arrives as $355, shifted left one bit, and no clock
                // phase inserted before them can fix it.
                //
                // SAMPLE_LATE=1 samples just before the FALLING edge instead,
                // giving the line the whole clock-high period to settle.
                parameter         SAMPLE_LATE = 1'b0
                ) (
                   input  wire        clk,
                   input  wire        rst,

                   input  wire        rd,              // pulse: read `lba` into the buffer
                   input  wire        wr,              // pulse: write the buffer to `lba`
                   input  wire [31:0] lba,
                   input  wire [7:0]  speed,        // SPI half-period once the card is up
                   output reg         done,            // one-cycle pulse: request finished
                   output reg         err,             // sticky for this request; see err_code

                   output reg  [9:0]  buf_addr,
                   output reg  [7:0]  buf_wdata,
                   output reg         buf_we,
                   input  wire [7:0]  buf_rdata,

                   output reg         sd_clk,
                   output reg         sd_cs_n,
                   output wire        sd_mosi,
                   input  wire        sd_miso,

                   // --- observability ---
                   output reg         ready,           // card initialized and addressable
                   output reg  [3:0]  err_code,
                   output wire [4:0]  dbg_state,
                   output wire [5:0]  dbg_cmd,         // the command in flight
                   output wire [7:0]  dbg_last,        // the most recent byte clocked in
                   output wire [7:0]  dbg_r1,          // the R1 that was accepted
                   output wire [7:0]  dbg_r1a,         // CMD0's R1, kept separately
                   output wire        dbg_v1,
                   output wire        dbg_ccs,
                   output wire [7:0]  dbg_crc_errs,
                   output wire [7:0]  dbg_cmds,
                   output wire [7:0]  dbg_retries,   // transfers re-issued since reset
                   output wire [7:0]  dbg_r55,
                   output wire [7:0]  dbg_r41,
                   output wire [2:0]  dbg_k,
                   output wire [2:0]  dbg_ph,
                   output wire [2:0]  dbg_phok,
                   output wire        dbg_phfound,
                   output wire        dbg_fast,
                   output wire [7:0]  dbg_khist,
                   output wire [31:0] dbg_r7,          // the four bytes after it (R7 / OCR)
                   output wire [127:0] dbg_cap         // first 16 bytes clocked in after a command
                   );

   // Error codes. Distinct per failure so a stuck bring-up says WHICH step.
   localparam [3:0]
                   E_NONE      = 4'd0,
                   E_CMD0      = 4'd1,   // no response to GO_IDLE_STATE: wiring, or no card
                   E_CMD8      = 4'd2,   // v2 check pattern came back wrong
                   E_ACMD41    = 4'd3,   // card never left idle
                   E_CMD58     = 4'd4,   // OCR unreadable
                   E_NOTSDHC   = 4'd5,   // UNUSED: byte addressing is supported
                   E_CMD17     = 4'd6,   // read command rejected
                   E_TOKEN     = 4'd7,   // no $FE data token
                   E_NOTREADY  = 4'd8,   // request arrived before init finished
                   E_WRITE     = 4'd9,   // CMD24 itself was rejected by the card
                   E_NOR1      = 4'd10,  // a command got no R1 at all within the poll window
                   E_WDATA     = 4'd11,  // the card rejected the written data block
                   E_WBUSY     = 4'd12;  // the card never released busy after a write

   localparam [4:0] 
                    S_RST      = 5'd0,  S_PWRCLK   = 5'd1,  S_CMD0     = 5'd2,
                    S_CMD8     = 5'd3,  S_CMD55    = 5'd5,
                    S_ACMD41   = 5'd6,  S_CMD58    = 5'd7,
                    S_READY    = 5'd9,  S_CMD17    = 5'd10, S_TOKEN    = 5'd11,
                    S_DATA     = 5'd12, S_CRC      = 5'd13, S_TAIL     = 5'd14,
                    S_DONE     = 5'd15, S_ERR      = 5'd16,
                    S_SEND     = 5'd17, S_DRAIN    = 5'd18,
                    S_R1       = 5'd20, S_R1B      = 5'd23,
                    S_R7       = 5'd21, S_CRCC     = 5'd24,
                    S_NCC      = 5'd25, S_CMD24    = 5'd26,
                    S_WTOKEN   = 5'd27, S_WPREP    = 5'd28,
                    S_WDATA    = 5'd29, S_WCRC     = 5'd30,
                    S_WRESP    = 5'd31;

   reg [4:0]        st, ret;
   assign dbg_state = st;

   // ---- byte-level SPI engine ---------------------------------------------
   reg  [7:0]  shift_out, shift_in;
   reg [3:0]   bit_cnt;
   reg [7:0]   div;                 // current half-period
   reg [7:0]   div_cnt;
   reg         spi_busy, spi_phase, spi_done;
   reg         spi_start;
   reg [7:0]   spi_tx;
   reg [3:0]   spi_nbits;   // bits in the next transfer; 8 except when resyncing

   assign sd_mosi = shift_out[7];
   wire tick = (div_cnt == 0);

   // The guard "the byte engine is free", as a REGISTERED FLAG rather than a
   // combinational test. Nearly every state of the sequencer tests it, so as a
   // wire it would add a LUT level to a dozen terms that all converge on
   // spi_start's next-state logic, which is the sequencer's critical path.
   //
   // It is cleared by spi_send and set by spi_done, BOTH from the sequencer's
   // own always block below. A delayed copy of `!spi_busy && !spi_start` looks
   // equivalent and is not: it stays high for the cycle after a send is issued,
   // and every state that issues from a level-sensitive guard then issues
   // twice.
   reg  spi_idle;

   always @(posedge clk) begin
      spi_done <= 1'b0;
      if (rst) begin
         spi_busy <= 1'b0; spi_phase <= 1'b0; sd_clk <= 1'b0;
         shift_out <= 8'hFF; shift_in <= 8'hFF; bit_cnt <= 0; div_cnt <= 0;
      end else if (spi_start && !spi_busy) begin
         shift_out <= spi_tx;
         bit_cnt   <= spi_nbits;
         spi_busy  <= 1'b1;
         spi_phase <= 1'b0;
         sd_clk    <= 1'b0;
         div_cnt   <= div;
      end else if (spi_busy) begin
         if (!tick) begin
            div_cnt <= div_cnt - 1'b1;
         end else begin
            div_cnt <= div;
            if (!spi_phase) begin
               sd_clk    <= 1'b1;                             // rising edge
               if (!SAMPLE_LATE) shift_in <= {shift_in[6:0], sd_miso};
               spi_phase <= 1'b1;
            end else begin
               sd_clk    <= 1'b0;                             // falling edge
               if (SAMPLE_LATE)  shift_in <= {shift_in[6:0], sd_miso};
               spi_phase <= 1'b0;
               if (bit_cnt == 4'd1) begin
                  spi_busy <= 1'b0;
                  spi_done <= 1'b1;
               end else begin
                  shift_out <= {shift_out[6:0], 1'b1};
                  bit_cnt   <= bit_cnt - 1'b1;
               end
            end
         end
      end
   end

   // ---- command sequencer --------------------------------------------------
   reg [7:0]  cmd_idx;
   reg [5:0]  cmd_num;
   reg [31:0] cmd_arg;
   reg [7:0]  cmd_crc;
   reg [7:0]  r1;
   // The VERDICT on R1, registered alongside it. The states that follow a
   // command test one bit instead of an eight-bit compare, which keeps the
   // cone from r1 through the state decode into cmd_num's clock enable off
   // the critical path.
   reg        r1_is00, r1_is01;
   reg [7:0]  poll;
   reg [9:0]  data_cnt;
   reg [3:0]  ocr_idx;
   reg [2:0]  r1_k;                 // bit position of R1's top bit within a byte
   reg [7:0]  r1_p;                 // the byte it started in
   wire [7:0] r1_join = (r1_p << r1_k) | (shift_in >> (4'd8 - r1_k));

   // First 0 bit, counted from the MSB. 8'hFF never reaches here.
   function [2:0] first_zero(input [7:0] b);
      casez (b)
        8'b0???????: first_zero = 3'd0;
        8'b10??????: first_zero = 3'd1;
        8'b110?????: first_zero = 3'd2;
        8'b1110????: first_zero = 3'd3;
        8'b11110???: first_zero = 3'd4;
        8'b111110??: first_zero = 3'd5;
        8'b1111110?: first_zero = 3'd6;
        default:     first_zero = 3'd7;
      endcase
   endfunction
   reg        got_r7;
   reg        is_v1;                // card rejected CMD8: treat as SD v1

   // CRC7 over the 40 bits of a command frame, generated rather than tabulated.
   //
   // The spec says SPI mode ignores the CRC except for CMD0 and CMD8, but real
   // cards do not all agree: sending a dummy CRC gets ACMD41 answered with $09,
   // idle plus COM_CRC_ERROR, and initialization then loops until it times out.
   // So compute it for every command. That costs a 7-bit shift register and 40
   // cycles per command, nothing next to the 48 bit-times the frame takes.
   //
   // Polynomial x^7 + x^3 + 1, and the byte on the wire is {crc, 1'b1}.
   // Link quality, and robustness against it. On a marginal link many
   // commands come back $09, idle plus COM_CRC_ERROR, which is a wiring
   // problem rather than a protocol one. A host that treats the first reply
   // as authoritative draws a firm conclusion from a corrupted one. A
   // single bad CMD8 would latch the design into SD v1 mode.
   //
   // So: resend on CRC error, and count. The counters are the number that says
   // whether a wiring fix worked.
   localparam integer CMD_RETRIES = 8;
   reg [3:0]          cmd_retry;
   reg [7:0]          crc_errs;             // saturating
   reg [7:0]          cmd_count;            // saturating
   reg [3:0]          cmd8_tries;
   reg [7:0]          r55, r41;   // R1 of the last CMD55 and ACMD41, kept apart
   // Which bit phases have ever been seen. If the card really is byte-aligned
   // and the recovery is inventing offsets, this shows it: a clean card gives
   // $01 (only phase 0), anything else means real or spurious skew.
   reg [7:0]          k_seen;

   // PHASE SWEEP. The offset needed to line our byte boundary up with the
   // card's is measured, not derived: a framing that is one bit late shows up
   // as R7 echoing $355 instead of $1AA, and the correction is found by trying
   // each phase in turn rather than by reasoning about where the bit went.
   //
   // So measure it. Each failed initialization advances phase_try, and that
   // many extra bits are clocked after CMD0's reply before anything else is
   // read. The phase that makes CMD8 echo $1AA is the right one, and the report
   // says which it was. Eight attempts covers every possibility.
   reg [2:0]          phase_try;
   reg [2:0]          phase_ok;              // the phase that produced a good R7
   reg                phase_found;
   reg                phase_applied;

   reg [6:0]          crc7;
   reg [5:0]          crc_i;
   reg [39:0]         crc_frame;
   reg                ccs;
   reg [15:0]         acmd_tries;
   reg [31:0]         req_lba;
   reg                pending_wr;
   reg [25:0]         retry_t;              // ~1 s at 50 MHz between init attempts

   // Declared here, after their sources exist, and before use: yosys accepts
   // a forward reference by inventing an implicit wire. iverilog refuses it.
   // `make sim-all` gates anything that reaches hardware.
   reg [39:0]         r7raw;                // five raw bytes following R1
   reg [2:0]          r7_off;               // bit offset that made the check pattern fit
   reg                r7_locked;
   reg                ncc_done;

   // ADAPTIVE SPEED. The first read after initialization can fail at any rate
   // the sweep called good, because the sweep exercises a card already
   // streaming rather than one just out of ACMD41. So the fast rate is not
   // assumed. Stay at the initialization rate until a read has actually
   // succeeded, then step up, and drop back permanently if one ever fails.
   reg                fast_en;
   reg                saw_busy;
   // The busy WAIT gets its own wide counter. poll is 8 bits, and 255 poll
   // bytes is ~163 us at the write clock, but a card committing a block can
   // hold busy for 1-10 ms routinely and HUNDREDS of ms during internal
   // garbage collection. If the window expires early the retry re-sends CMD24
   // into a card that is still busy, and the command stream never recovers: the
   // card stays stuck until it is power cycled. 2^20 poll bytes is 1-2 seconds
   // at any write clock, and past that the card really is hung.
   reg [19:0]         bpoll;
   reg [3:0]          relcnt;
   reg [15:0]         settle;
   reg [22:0]         tok_t;
   reg [3:0]          retries;      // attempts left for the transfer in flight
   reg                retry_pend;   // S_TAIL should re-issue rather than finish
   reg [7:0]          retry_n;      // saturating count, for telemetry
   localparam [15:0]  SETTLE_CYCLES = (CLK_HZ / 1000) > 65535 ? 16'd65535
                      : (CLK_HZ / 1000);

   // r7 selected at the winning offset. Offset 0 is the textbook case.
   wire [31:0]        r7_at [0:7];
   assign r7_at[0] = r7raw[39:8];
   assign r7_at[1] = r7raw[38:7];
   assign r7_at[2] = r7raw[37:6];
   assign r7_at[3] = r7raw[36:5];
   assign r7_at[4] = r7raw[35:4];
   assign r7_at[5] = r7raw[34:3];
   assign r7_at[6] = r7raw[33:2];
   assign r7_at[7] = r7raw[32:1];
   wire [31:0] r7 = r7_at[r7_off];
   reg [7:0]   r1a;

   // RAW CAPTURE. Every byte clocked in from the moment a command frame ends,
   // sixteen of them, reported verbatim: the actual sequence says what a card
   // is doing where a single decoded R1 cannot. Armed when S_SEND finishes,
   // frozen when full.
   reg [127:0] cap;
   reg [4:0]   cap_i;
   assign dbg_cap = cap;
   assign dbg_r1a  = r1a;
   assign dbg_v1   = is_v1;
   assign dbg_ccs  = ccs;
   assign dbg_crc_errs = crc_errs;
   assign dbg_cmds     = cmd_count;
   assign dbg_r55      = r55;
   assign dbg_r41      = r41;
   assign dbg_k        = r1_k;
   assign dbg_ph       = phase_try;
   assign dbg_phok     = r7_off;
   assign dbg_phfound  = phase_found;
   assign dbg_fast     = fast_en;
   assign dbg_khist    = k_seen;
   assign dbg_cmd  = cmd_num;
   assign dbg_retries = retry_n;
   assign dbg_last = shift_in;
   assign dbg_r1   = r1;
   assign dbg_r7   = r7;

   task spi_send(input [7:0] b);
      begin
         spi_tx    <= b;
         spi_nbits <= 4'd8;
         spi_start <= 1'b1;
         spi_idle  <= 1'b0;
      end
   endtask

   // A short transfer, used once, to slide our byte boundary onto the card's.
   task spi_send_bits(input [3:0] n);
      begin
         spi_tx    <= 8'hFF;
         spi_nbits <= n;
         spi_start <= 1'b1;
         spi_idle  <= 1'b0;
      end
   endtask

   always @(posedge clk) begin
      spi_start <= 1'b0;
      done      <= 1'b0;
      buf_we    <= 1'b0;
      if (spi_done) begin
         spi_idle <= 1'b1;                  // a later spi_send in the case wins
         if (cap_i != 5'd16) begin
            cap   <= {cap[119:0], shift_in};
            cap_i <= cap_i + 1'b1;
         end
      end

      if (rst) begin
         st <= S_RST; ret <= S_RST;
         sd_cs_n <= 1'b1; div <= DIV_INIT[7:0];
         ready <= 1'b0; err <= 1'b0; err_code <= E_NONE;
         cmd_idx <= 0; poll <= 0; acmd_tries <= 0; ccs <= 1'b0; crc_i <= 0; crc7 <= 0;
         got_r7 <= 1'b0; spi_nbits <= 4'd8; r1_k <= 0; r1_p <= 8'hFF; is_v1 <= 1'b0;
         fast_en <= 1'b0; saw_busy <= 1'b0; relcnt <= 4'd0; settle <= 16'd0;
         bpoll <= 20'd0;
         tok_t <= 23'd0; retries <= 4'd0; retry_pend <= 1'b0; retry_n <= 8'd0;
         cmd_retry <= 0; crc_errs <= 0; cmd_count <= 0; cmd8_tries <= 0;
         r55 <= 8'hFF; r41 <= 8'hFF; k_seen <= 0;
         phase_try <= 0; phase_ok <= 0; phase_found <= 1'b0; phase_applied <= 1'b0;
         r7raw <= 0; r7_off <= 0; r7_locked <= 1'b0; ncc_done <= 1'b0;
         r1 <= 8'hFF; r1_is00 <= 1'b0; r1_is01 <= 1'b0; spi_idle <= 1'b1; r1a <= 8'hFF; cap <= 0; cap_i <= 5'd16;
         buf_addr <= 0; data_cnt <= 0; pending_wr <= 1'b0; retry_t <= 0;
      end else case (st)

                 // The spec's minimum is 74 clocks with CS high and MOSI high.
                 // Send 640.
                 //
                 // Ten bytes is enough to wake a card that has just been
                 // powered on, and not enough to rescue one that was
                 // interrupted mid-block. A read that stopped before its data
                 // token leaves the card still trying to send 512 bytes, and it
                 // ignores everything until they are clocked out. Eighty bytes
                 // is 640 clocks, more than a stuck block plus its CRC, and
                 // costs 6.4 ms at the initialization rate.
                 S_RST: begin
                    sd_cs_n <= 1'b1;
                    cmd_idx <= 0;
                    st      <= S_PWRCLK;
                    spi_send(8'hFF);
                 end

                 S_PWRCLK: if (spi_done) begin
                    if (cmd_idx == 8'd79) begin
                       sd_cs_n <= 1'b0;
                       cmd_num <= 6'd0; cmd_arg <= 32'h0000_0000; cmd_crc <= 8'h95;
                       ret     <= S_CMD0;
                       cmd_idx <= 0;
                       st      <= S_SEND;
                    end else begin
                       cmd_idx <= cmd_idx + 1'b1;
                       spi_send(8'hFF);
                    end
                 end

                 // Compute the frame's CRC7 before sending it, 40 bits at a
                 // time.
                 S_CRCC: begin
                    if (crc_i == 6'd0) begin
                       crc_frame <= {2'b01, cmd_num, cmd_arg};
                       crc7      <= 7'd0;
                       crc_i     <= 6'd1;
                    end else if (crc_i == 6'd41) begin
                       cmd_crc <= {crc7, 1'b1};
                       cmd_idx <= 0;
                       crc_i   <= 6'd0;
                       st      <= S_NCC;
                    end else begin
                       crc7      <= {crc7[5:0], 1'b0}
                                    ^ ((crc_frame[39] ^ crc7[6]) ? 7'h09 : 7'h00);
                       crc_frame <= {crc_frame[38:0], 1'b0};
                       crc_i     <= crc_i + 6'd1;
                    end
                 end

                 // Ncc: AT LEAST 8 CLOCK CYCLES between a response and the next
                 // command.
                 //
                 // Going straight from the R1 byte into the next command frame
                 // with no clocks in between violates that, and a card run
                 // outside its timing hangs: it answers the first handful of
                 // commands perfectly, with no CRC errors, then stops
                 // responding until it is power cycled.
                 //
                 // One idle byte is 8 clocks, which is the minimum, and it
                 // costs 80 us at the init rate.
                 S_NCC: if (spi_idle) begin
                    if (ncc_done) begin
                       ncc_done <= 1'b0;
                       st       <= S_SEND;
                    end else begin
                       ncc_done <= 1'b1;
                       spi_send(8'hFF);
                    end
                 end

                 // ---- generic: six command bytes, then poll for R1 ----
                 S_SEND: if (spi_idle) begin
                    case (cmd_idx)
                      8'd0: spi_send({2'b01, cmd_num});
                      8'd1: spi_send(cmd_arg[31:24]);
                      8'd2: spi_send(cmd_arg[23:16]);
                      8'd3: spi_send(cmd_arg[15:8]);
                      8'd4: spi_send(cmd_arg[7:0]);
                      default: spi_send(cmd_crc);
                    endcase
                    cmd_idx <= cmd_idx + 1'b1;
                    if (cmd_idx == 8'd5) begin
                       cap   <= 0;
                       cap_i <= 0;
                       poll  <= 8'd0;
                       st    <= S_R1;
                    end
                 end

                 // ---- find the reply, at the bit level ----
                 // Poll for the first byte that is not all ones, then locate
                 // the first 0 WITHIN it. That bit is R1's top bit, always:
                 // every R1 has bit 7 clear and the line idles high between
                 // them.
                 //
                 // Why not simply take the first byte with bit 7 clear, as a
                 // host normally would? Because a card need not byte-align its
                 // replies to our framing. One answers CMD8 with FF FF C1 7F
                 // FF... Read byte-wise, that yields $7F, every error bit set
                 // at once. Read bit-wise, it is one clean $05. Recovering the
                 // bit position costs a priority encoder and is exactly
                 // equivalent when the card does align, so there is no mode and
                 // no special case.
                 //
                 // Done PER COMMAND rather than once, because the phase can
                 // move again whenever CS is deasserted, which happens after
                 // every block read.
                 S_R1: if (spi_idle) begin
                    if (poll != 8'd0 && shift_in != 8'hFF) begin
                       r1_k   <= first_zero(shift_in);
                       r1_p   <= shift_in;
                       k_seen <= k_seen | (8'd1 << first_zero(shift_in));
                       if (first_zero(shift_in) == 3'd0) begin
                          r1      <= shift_in;
                          r1_is00 <= (shift_in == 8'h00);
                          r1_is01 <= (shift_in == 8'h01);
                          cmd_idx <= 0;
                          if (cmd_count != 8'hFF) cmd_count <= cmd_count + 1'b1;
                          if (shift_in[3]) begin                       // COM_CRC_ERROR
                             if (crc_errs != 8'hFF) crc_errs <= crc_errs + 1'b1;
                             if (cmd_retry != CMD_RETRIES[3:0]) begin
                                cmd_retry <= cmd_retry + 1'b1;
                                st        <= S_CRCC;                     // send it again
                             end else begin
                                cmd_retry <= 0;
                                st        <= ret;
                             end
                          end else begin
                             cmd_retry <= 0;
                             st        <= ret;
                          end
                       end else begin
                          st <= S_R1B;                 // R1 straddles this byte and the next
                          spi_send(8'hFF);
                       end
                    end else if (poll == 8'd200) begin
                       err_code <= E_NOR1;
                       st       <= S_ERR;
                    end else begin
                       poll <= poll + 1'b1;
                       spi_send(8'hFF);
                    end
                 end

                 // Assemble R1 from the tail of one byte and the head of the
                 // next, then clock the leftover bits so everything after this
                 // is aligned. The token and the 512 data bytes then arrive on
                 // our byte boundaries with no further handling.
                 S_R1B: if (spi_idle) begin
                    r1      <= r1_join;
                    r1_is00 <= (r1_join == 8'h00);
                    r1_is01 <= (r1_join == 8'h01);
                    cmd_idx <= 0;
                    if (cmd_count != 8'hFF) cmd_count <= cmd_count + 1'b1;
                    if (r1_join[3]) begin
                       if (crc_errs != 8'hFF) crc_errs <= crc_errs + 1'b1;
                       if (cmd_retry != CMD_RETRIES[3:0]) begin
                          cmd_retry <= cmd_retry + 1'b1;
                          st        <= S_CRCC;
                       end else begin
                          cmd_retry <= 0;
                          st        <= ret;
                       end
                    end else begin
                       cmd_retry <= 0;
                       st        <= ret;
                    end
                    // Insert k bits, NOT 8-k. Our boundaries sit at 0, 8,
                    // 16...; the card's at k, k+8, k+16... Having consumed two
                    // of our bytes we are at 16, and the next card boundary is
                    // k+16, so k more bits land us on it. Inserting 8-k instead
                    // moves the offset to (8-2k) mod 8, which for the observed
                    // k=2 turns a 2-bit skew into a 4-bit one (k=2 at CMD0
                    // becomes k=4 at CMD8).
                    spi_send_bits({1'b0, r1_k});
                 end

                 // Four more bytes after R1: R7's echo for CMD8, the OCR for
                 // CMD58.
                 //
                 // Capture on entries 1..4, which are the four bytes returned
                 // by the four sends issued on entries 0..3. Terminating at 4
                 // WITHOUT capturing there loses the last byte, which for CMD8
                 // is the check pattern and for CMD58 is the low OCR byte. The
                 // card re-aligns its reply to each command frame, so a
                 // correction applied before the command is discarded and
                 // cannot help. The offset therefore belongs HERE, between R1
                 // and the bytes that follow it. Read FIVE bytes, not four,
                 // and pick the alignment afterward.
                 //
                 // A framing one bit late puts the four bytes after R1 at $355
                 // rather than $1AA, and no inserted clock phase or choice of
                 // sampling edge changes that. So take one spare byte and try
                 // all eight alignments. The R7 whose last byte is the $AA we
                 // asked for is the right one, and the card validates the
                 // choice for us.
                 //
                 // The offset that wins is remembered in r7_off and reused for
                 // the OCR, which has no check pattern of its own.
                 S_R7: if (spi_idle) begin
                    if (ocr_idx != 4'd0) r7raw <= {r7raw[31:0], shift_in};
                    if (ocr_idx == 4'd5) begin
                       cmd_idx <= 0;
                       st      <= ret;
                    end else begin
                       ocr_idx <= ocr_idx + 1'b1;
                       spi_send(8'hFF);
                    end
                 end

                 // ---- initialization ----
                 S_CMD0: begin
                    r1a <= r1;                       // keep it: CMD8 is about to overwrite r1

                    if (!r1_is01) begin err_code <= E_CMD0; st <= S_ERR; end
                    else begin
                       // CMD8: 2.7-3.6V, check pattern $AA. CRC matters for
                       // CMD0 and CMD8 only. After that the card ignores it in
                       // SPI mode.
                       cmd_num <= 6'd8; cmd_arg <= 32'h0000_01AA; cmd_crc <= 8'h87;
                       ret <= S_CMD8; st <= S_CRCC;
                    end
                 end

                 // R7: R1 then four more bytes; the last echoes the check
                 // pattern $AA. CMD8 has three legitimate outcomes, and only
                 // one of them is a failure.
                 //
                 //   R1 bit 2 set   the card does not know CMD8, which per the spec means
                 //                  treat it as SD v1: no HCS bit in ACMD41, and capacity
                 //                  comes from the OCR afterward. NOT an error.
                 //   R1 = $01       v2 or later; read R7 and check the $AA echo.
                 //   anything else  a real failure.
                 //
                 // A card that answers $05, illegal command plus idle, does so
                 // consistently on every retry and at every clock rate. That is
                 // a v1 card saying so, not a link fault, and it must not be
                 // rejected.
                 S_CMD8: begin
                    if (r1[2] && cmd8_tries != 4'd3) begin
                       // A rejection that arrived alongside a CRC error, or the
                       // first few rejections on a noisy link, are not
                       // evidence. Ask again.
                       cmd8_tries <= cmd8_tries + 1'b1;
                       st         <= S_CRCC;
                    end else if (r1[2]) begin
                       is_v1      <= 1'b1;
                       got_r7     <= 1'b0;
                       acmd_tries <= 0;
                       cmd_num <= 6'd55; cmd_arg <= 32'h0; cmd_crc <= 8'hFF;
                       ret <= S_CMD55; st <= S_CRCC;
                    end else if (!r1_is01) begin
                       err_code <= E_CMD8; st <= S_ERR;
                    end else if (!got_r7) begin
                       got_r7 <= 1'b1; ocr_idx <= 0; phase_applied <= 1'b0; ret <= S_CMD8; st <= S_R7;
                    end else if (!r7_locked) begin
                       // try each alignment in turn until the echo matches
                       if (r7_at[r7_off][7:0] == 8'hAA && r7_at[r7_off][31:16] == 16'h0000) begin
                          r7_locked <= 1'b1;
                       end else if (r7_off == 3'd7) begin
                          err_code <= E_CMD8; st <= S_ERR;
                       end else begin
                          r7_off <= r7_off + 3'd1;
                       end
                    end else begin
                       got_r7      <= 1'b0;
                       phase_ok    <= phase_try;      // this phase produced a correct echo
                       phase_found <= 1'b1;
                       acmd_tries  <= 0;
                       cmd_num <= 6'd55; cmd_arg <= 32'h0; cmd_crc <= 8'hFF;
                       ret <= S_CMD55; st <= S_CRCC;
                    end
                 end

                 S_CMD55: begin
                    r55 <= r1;
                    // ACMD41 with HCS set: "I understand high capacity". HCS
                    // only means anything to a v2 card. A v1 card must see
                    // zero.
                    cmd_num <= 6'd41;
                    cmd_arg <= is_v1 ? 32'h0000_0000 : 32'h4000_0000;
                    cmd_crc <= 8'hFF;
                    ret <= S_ACMD41; st <= S_CRCC;
                 end

                 S_ACMD41: begin
                    r41 <= r1;
                    if (r1_is00) begin
                       cmd_num <= 6'd58; cmd_arg <= 32'h0; cmd_crc <= 8'hFF;
                       ret <= S_CMD58; st <= S_CRCC;
                       // BE PATIENT. A card may answer CMD0, CMD8 and ACMD41
                       // only on the first attempt after power-up and ignore
                       // every retry. So the first attempt is the only one
                       // that counts and must not be cut short. ACMD41 gets
                       // ~18 s here. A card that needs a second or two to come
                       // ready would otherwise be abandoned and then left hung
                       // by the reset.
                    end else if (acmd_tries[14]) begin
                       err_code <= E_ACMD41; st <= S_ERR;
                    end else begin
                       acmd_tries <= acmd_tries + 1'b1;
                       cmd_num <= 6'd55; cmd_arg <= 32'h0; cmd_crc <= 8'hFF;
                       ret <= S_CMD55; st <= S_CRCC;
                    end
                 end

                 // OCR bit 30 is CCS: 1 = block addressed, 0 = byte addressed.
                 S_CMD58: begin
                    if (!r1_is00) begin err_code <= E_CMD58; st <= S_ERR; end
                    else if (!got_r7) begin got_r7 <= 1'b1; ocr_idx <= 0; phase_applied <= 1'b0; ret <= S_CMD58; st <= S_R7; end
                    else begin
                       // OCR bit 30 is CCS. Set means block addressing, clear
                       // means the argument is a BYTE address. Both are
                       // supported.
                       ccs     <= r7[30];
                       got_r7  <= 1'b0;
                       div     <= DIV_INIT[7:0];      // the first read stays slow
                       sd_cs_n <= 1'b1;
                       ready   <= 1'b1;
                       st      <= S_READY;
                    end
                 end

                 // OCR, four bytes. Bit 30 of the first is CCS: 1 = block
                 // addressed.
                 //
                 S_READY: begin
                    err <= 1'b0;
                    div <= fast_en ? speed : DIV_INIT[7:0];   // changed only while idle
                    if (rd || wr) begin
                       req_lba    <= lba + LBA_OFF[31:0];
                       pending_wr <= wr;
                       retries    <= RETRY_MAX[3:0];
                       retry_pend <= 1'b0;
                       if (wr) begin
                          div     <= fast_en ? DIV_WRITE[7:0] : DIV_INIT[7:0];
                          sd_cs_n <= 1'b0;
                          cmd_num <= 6'd24;
                          cmd_arg <= ccs ? (lba + LBA_OFF[31:0])
                            : ((lba + LBA_OFF[31:0]) << 9);
                          cmd_crc <= 8'hFF;
                          ret <= S_CMD24; st <= S_CRCC;
                       end else begin
                          sd_cs_n <= 1'b0;
                          cmd_num <= 6'd17;
                          // Block address on a high-capacity card, byte address
                          // otherwise.
                          cmd_arg <= ccs ? (lba + LBA_OFF[31:0])
                            : ((lba + LBA_OFF[31:0]) << 9);
                          cmd_crc <= 8'hFF;
                          ret <= S_CMD17; st <= S_CRCC;
                       end
                    end
                 end

                 // ---- write path, CMD24 ----
                 // R1 = $00, then at least one idle byte (Nwr), the $FE start
                 // token, 512 data bytes, two CRC bytes the card ignores in SPI
                 // mode, then a data response whose low five bits are $05 if
                 // it was accepted, then the card holds MISO LOW until the
                 // write has actually landed.
                 S_CMD24: if (spi_idle) begin
                    if (!r1_is00) begin if (retries != 4'd0) begin
                       retries    <= retries - 4'd1;
                       retry_pend <= 1'b1;
                       if (retry_n != 8'hFF) retry_n <= retry_n + 8'd1;
                       st         <= S_TAIL;
                    end else begin err_code <= E_WRITE; st <= S_ERR; end end
                    else begin
                       st <= S_WTOKEN;
                       spi_send(8'hFF);              // Nwr: one idle byte before the token
                    end
                 end

                 S_WTOKEN: if (spi_idle) begin
                    buf_addr <= 10'd0;
                    data_cnt <= 10'd0;
                    st       <= S_WPREP;
                    spi_send(8'hFE);
                 end

                 // One cycle for the block RAM's registered read to present
                 // byte 0.
                 S_WPREP: if (spi_idle) st <= S_WDATA;

                 S_WDATA: if (spi_idle) begin
                    spi_send(buf_rdata);
                    if (data_cnt == 10'd511) begin
                       ocr_idx <= 0;
                       st      <= S_WCRC;
                    end else begin
                       data_cnt <= data_cnt + 1'b1;
                       buf_addr <= data_cnt + 1'b1;   // fetch the next byte while this one goes
                    end
                 end

                 S_WCRC: if (spi_idle) begin
                    if (ocr_idx == 4'd2) begin
                       poll    <= 0;
                       ocr_idx <= 0;
                       st      <= S_WRESP;
                    end else begin
                       ocr_idx <= ocr_idx + 1'b1;
                       spi_send(8'hFF);
                    end
                 end

                 // The data response, then the busy wait. A card that is
                 // writing holds MISO low for as long as it needs. Releasing it
                 // is the only signal that the block is committed, so this must
                 // not be skipped. Two phases, ocr_idx picking between them so
                 // this needs no extra state.
                 //
                 //   0  wait for the data response. It does NOT arrive in the byte right
                 //      after the CRC: the card is still consuming that one, so the first
                 //      read back is $FF. Poll until something else turns up.
                 //   1  wait out the busy period. The card holds MISO at $00 while it
                 //      commits the block, and releasing it is the only signal that the
                 //      write actually landed.
                 S_WRESP: if (spi_idle) begin
                    if (ocr_idx == 4'd0) begin
                       if (poll != 8'd0 && shift_in != 8'hFF) begin
                          if ((shift_in & 8'h1F) == 8'h05) begin
                             ocr_idx <= 4'd1;
                             poll    <= 8'd0;
                             bpoll   <= 20'd0;
                             spi_send(8'hFF);
                          end else begin
                             if (retries != 4'd0) begin
                                retries    <= retries - 4'd1;
                                retry_pend <= 1'b1;
                                if (retry_n != 8'hFF) retry_n <= retry_n + 8'd1;
                                st         <= S_TAIL;
                             end else begin err_code <= E_WDATA; st <= S_ERR; end
                          end
                       end else if (poll == 8'd16) begin
                          if (retries != 4'd0) begin
                             retries    <= retries - 4'd1;
                             retry_pend <= 1'b1;
                             if (retry_n != 8'hFF) retry_n <= retry_n + 8'd1;
                             st         <= S_TAIL;
                          end else begin err_code <= E_WDATA; st <= S_ERR; end
                       end else begin
                          poll <= poll + 1'b1;
                          spi_send(8'hFF);
                       end
                    end else begin
                       // Wait until busy has actually been SEEN, then wait for
                       // it to end. Exiting on the first sample that is not
                       // $00 would leave on the very first byte if the card
                       // has not asserted busy yet: the card is still writing,
                       // the next command goes into a card that is ignoring
                       // the bus, and the read after it fails with E_TOKEN.
                       //
                       // If busy never appears at all within the window, take
                       // the card at its word and carry on rather than
                       // stalling. Busy must be SEEN, and its release must be
                       // SUSTAINED.
                       //
                       // One non-zero sample is not enough. A card can let MISO
                       // float high for a moment mid-write, and how likely that
                       // is to be sampled depends entirely on the clock. At 100
                       // kHz a poll byte is 80 us and the glitch is invisible.
                       // At 8.33 MHz it is 1 us and gets caught. Acting on it
                       // issues the next command into a card that is still
                       // writing, and the read after it fails with E_TOKEN,
                       // which looks like a write speed limit and is not.
                       //
                       // Eight consecutive non-zero bytes is 8 us at 8.33 MHz,
                       // and merely 640 us at the init rate, so the strict
                       // version costs nothing.
                       if (saw_busy) begin
                          if (shift_in != 8'h00) begin
                             if (relcnt == 4'd7) begin
                                saw_busy <= 1'b0; relcnt <= 4'd0;
                                st       <= S_TAIL;
                             end else relcnt <= relcnt + 4'd1;
                             spi_send(8'hFF);
                          end else begin
                             relcnt <= 4'd0;                 // still busy, start the count over
                             // NO retry on a busy timeout: re-sending CMD24
                             // into a card that is still committing desyncs
                             // the stream and leaves the card stuck. At this
                             // window a timeout means the card is gone.
                             if (bpoll == 20'hFFFFF) begin
                                err_code <= E_WBUSY; st <= S_ERR;
                             end else begin bpoll <= bpoll + 20'd1; spi_send(8'hFF); end
                          end
                       end else if (shift_in == 8'h00) begin
                          saw_busy <= 1'b1; relcnt <= 4'd0;
                          spi_send(8'hFF);
                       end else if (poll == 8'd255) begin
                          st <= S_TAIL;                     // busy never appeared; carry on
                          spi_send(8'hFF);
                       end else begin
                          poll <= poll + 1'b1;
                          spi_send(8'hFF);
                       end
                    end
                 end

                 S_CMD17: begin
                    if (!r1_is00) begin if (retries != 4'd0) begin
                       retries    <= retries - 4'd1;
                       retry_pend <= 1'b1;
                       if (retry_n != 8'hFF) retry_n <= retry_n + 8'd1;
                       st         <= S_TAIL;
                    end else begin err_code <= E_CMD17; st <= S_ERR; end end
                    else begin poll <= 0; tok_t <= TOKEN_TIMEOUT_CYC[22:0]; st <= S_TOKEN; end
                 end

                 // The card may make us wait. $FE opens the data block, and
                 // anything with the top bits clear is an error token.
                 S_TOKEN: begin
                    // Real time, not byte times. See TOKEN_TIMEOUT_CYC above.
                    if (tok_t != 0) tok_t <= tok_t - 1'b1;
                    if (spi_idle) begin
                       if (poll != 8'd0 && shift_in == 8'hFE) begin
                          data_cnt <= 0; buf_addr <= 0; st <= S_DATA;
                          spi_send(8'hFF);
                       end else if (tok_t == 0) begin
                          if (retries != 4'd0) begin
                             retries    <= retries - 4'd1;
                             retry_pend <= 1'b1;
                             if (retry_n != 8'hFF) retry_n <= retry_n + 8'd1;
                             st         <= S_TAIL;
                          end else begin err_code <= E_TOKEN; st <= S_ERR; end
                       end else begin
                          // SATURATE rather than wrap. poll only exists to
                          // check that at least one byte has been clocked
                          // before a $FE is believed. Left to wrap, it
                          // returns to zero every 256 polls and the check
                          // throws away a perfectly good token once per wrap.
                          if (poll != 8'hFF) poll <= poll + 1'b1;
                          spi_send(8'hFF);
                       end
                    end
                 end

                 S_DATA: if (spi_done) begin
                    buf_addr  <= data_cnt;
                    buf_wdata <= shift_in;
                    buf_we    <= 1'b1;
                    if (data_cnt == 10'd511) begin
                       ocr_idx <= 0; st <= S_CRC;
                    end else begin
                       data_cnt <= data_cnt + 1'b1;
                    end
                    spi_send(8'hFF);
                 end

                 // Two CRC bytes, discarded: SPI mode does not require us to
                 // check them, and the card has already checked its own on the
                 // way out.
                 S_CRC: if (spi_done) begin
                    if (ocr_idx == 4'd1) st <= S_TAIL;
                    else begin ocr_idx <= ocr_idx + 1'b1; spi_send(8'hFF); end
                 end

                 // After a write, sit still for a millisecond before letting
                 // the next command go out.
                 //
                 // Everything else in this engine is counted in BYTE TIMES,
                 // which means the real-time gap after a write scales with the
                 // clock: about 300 us at the 100 kHz init rate but only about
                 // 4 us at 8.33 MHz. A card that wants a settle in between
                 // therefore looks like it works at slow clocks and fails at
                 // fast ones, which is precisely the shape of "writes have a
                 // lower ceiling than reads" that this chased for a while.
                 // Reads were verified byte-correct at 8.33 MHz throughout. CS
                 // is high here, so the card is free to finish whatever it is
                 // doing internally.
                 S_TAIL: begin
                    sd_cs_n <= 1'b1;
                    if (spi_idle) begin
                       // A retry settles here too, not just a write:
                       // deselecting the card and leaving it alone for a
                       // millisecond is the whole point of retrying, since
                       // whatever made the first attempt fail needs time to
                       // pass.
                       if (!(pending_wr || retry_pend) || settle == SETTLE_CYCLES) begin
                          settle <= 16'd0;
                          if (retry_pend) begin
                             retry_pend <= 1'b0;
                             sd_cs_n    <= 1'b0;
                             cmd_num    <= pending_wr ? 6'd24 : 6'd17;
                             // req_lba already carries LBA_OFF, so it must NOT
                             // be added again.
                             cmd_arg    <= ccs ? req_lba : (req_lba << 9);
                             cmd_crc    <= 8'hFF;
                             ret        <= pending_wr ? S_CMD24 : S_CMD17;
                             st         <= S_CRCC;
                          end else begin
                             st <= S_DONE;
                             spi_send(8'hFF);               // eight idle clocks with CS high
                          end
                       end else settle <= settle + 16'd1;
                    end
                 end

                 S_DONE: if (spi_done) begin
                    done    <= 1'b1;
                    err     <= 1'b0;
                    fast_en <= 1'b1;                 // a transfer completed: fast rate OK
                    st      <= S_READY;
                 end

                 // If a read failed, hand the failure up and go back to READY
                 // so the next request can still be tried.
                 //
                 // If the card never came up at all, RETRY THE WHOLE
                 // INITIALIZATION about once a second instead of latching the
                 // failure forever. On a hand-soldered board that is the
                 // difference between "fix the wire and watch it come up" and
                 // "fix the wire, then reflash to find out". It also covers a
                 // card pulled and reinserted, which is the job the socket's
                 // absent CD contact would otherwise have done. Before giving
                 // up, DRAIN THE RESPONSE. The decision that brought us here
                 // was made on one byte. The capture buffer holds sixteen, and
                 // the tail of a reply says far more than its head. Keep CS
                 // asserted and keep clocking until the buffer is full, then
                 // release and retry.
                 S_ERR: if (cap_i != 5'd16) begin
                    if (spi_idle) spi_send(8'hFF);
                 end else begin
                    sd_cs_n <= 1'b1;
                    err     <= 1'b1;
                    fast_en <= 1'b0;                 // and lost again on any failure
                    if (ready) begin
                       done <= 1'b1;
                       st   <= S_READY;
                    end else if (retry_t == 26'd50_000_000) begin
                       retry_t  <= 0;
                       err_code <= E_NONE;
                       spi_idle <= 1'b1;
                       // Next attempt, next phase, unless one has already
                       // worked.
                       if (!phase_found) phase_try <= phase_try + 3'd1;
                       phase_applied <= 1'b0;
                       st       <= S_RST;
                    end else retry_t <= retry_t + 1'b1;
                 end

                 default: st <= S_ERR;
               endcase

      // A request that arrives before the card is up must not be dropped
      // without an error code.
      if (!rst && !ready && (rd || wr) && st != S_ERR) begin
         err_code <= E_NOTREADY;
      end
   end

endmodule

`default_nettype wire
