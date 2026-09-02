// top: ProFile emulator, ULX3S (ECP5). FAMILY selects the pad and PLL
// primitives; the iCE40 and XC7 arms are unsupported reference code.
//
// Wiring only. The block buffer is one 1 KB dual-port BRAM (532 bytes are used;
// 1 KB is the natural inference and leaves room for the $FFFFFE RAM buffer to
// share the primitive later).
//
// Every port here is constrained in ../constraints/ulx3s.lpf, wired to the
// interface board as described in ../carrier/ULX3S-ADAPTER.md. Port names and
// constraint names must agree exactly or nextpnr refuses the design, so do not
// rename one alone.
//
// The whole drive is here: the SD stack, the FAT32 resolver, the boot menu and
// the serial report.

`default_nettype none

module top #(
             // Which FPGA this is being built for. It selects two vendor
             // primitives and nothing else: the PLL, and the data-bus pads in
             // src/profile_bus.v. Everything between them is portable.
             //   "ECP5"   ULX3S, 25 MHz in, 50 MHz out. The supported target.
             // The iCE40 and XC7 arms are unsupported reference code.
             parameter FAMILY = "ECP5",
             // The SD clock dividers: half-periods of the 50 MHz system
             // clock. Exposed so a testbench can run the whole initialization
             // in a reasonable number of simulated cycles. The defaults are
             // the real ones. 100 kHz to bring the card up, not the 400 kHz
             // the spec allows: the limit is the wiring, not the card. It can
             // be raised on a board that is shown to run at the higher rate.
             parameter integer SD_DIV_INIT = 250,
             // The ceiling is DIV=2, 8.33 MHz: the sweep's indices 0..5 return
             // correct data and 6 and 7 do not. That is 492 us for a 512-byte
             // block against the //e's ~6.6 ms. The card is thirteen times
             // faster than the bus can consume it, so there is nothing to gain
             // by pushing further. 8.33 MHz is the ceiling: the on-board sweep
             // verifies a known sector's checksum at each rate and finds 99.6k,
             // 391k, 1.47M, 2.78M, 5.0M and 8.33M all good, 12.5M and 25M bad.
             //
             // Re-measure with SWEEP_ENABLE before changing this number. Bad
             // reads at speed are usually not a speed limit: a missing Ncc idle
             // byte, a card left stuck by a previous run, or a busy wait that
             // exits before the card has asserted busy all look identical to
             // one.
             parameter integer SD_DIV_FAST = 2,
             // Writes get their own divider because they do NOT run as fast
             // as reads on this board. Measured with the write test (write a
             // known-free block, read it back, compare all 512 bytes, restore
             // it):
             //
             //   758 kHz (DIV 32) pass      1.47 MHz (DIV 16) fail
             //   391 kHz (DIV 63) pass      2.78 MHz (DIV  8) fail
             //                              5.00 MHz (DIV  4) fail
             //
             // so the write ceiling sits between 758 kHz and 1.47 MHz, roughly
             // a decade below the read ceiling. A read puts six command bytes
             // on MOSI, and a write puts 512 back to back. So a write is the
             // only thing that really exercises MOSI integrity. That makes
             // this a wiring limit rather than a protocol one.
             //
             // 391 kHz rather than the 758 kHz that also passed: writes are
             // rare here and margin is worth more than write throughput.
             parameter integer SD_DIV_WRITE = 63,
             // Set 0 in simulation to bypass the PLL, which iverilog does not
             // model.
             parameter         USE_PLL     = 1'b1,
             // The clock sweep is a MEASUREMENT TOOL, not something to run
             // every boot. It deliberately drives the card at rates that fail,
             // and a read that dies mid-block leaves the card stuck until it
             // is physically reinserted. Its result is recorded on SD_DIV_FAST
             // above; turn this back on only to re-measure after a wiring
             // change.
             parameter         SWEEP_ENABLE = 1'b0,

             // Blocks the image file holds. 65024 ($FE00, 31.75MB) is the size
             // the Teensy build is tested against. geometry.v turns this into
             // the two counts the drive actually needs. See there before
             // changing it. In particular, do NOT hand-align it, because
             // rounding is geometry.v's job.
             parameter integer IMAGE_BLOCKS = 32'h0000FE00,
             // 0 stops serving at the claim, so a suspect failure on a
             // decoupled volume can be A/B'd against the aligned claim.
             parameter         SERVE_FULL_IMAGE = 1'b1,

             // ---- whole-volume verification (src/verifier.v) ----
             // Off by default: it holds the SD engine for as long as it takes
             // to read the range, which is not something a build meant to
             // serve a host should ever do. Turn it on to answer "does every
             // block match the image", then turn it off again.
             // tools/verify_image.js is the other half.
             parameter         VERIFY_ENABLE = 1'b0,
             parameter [31:0]  VERIFY_START  = 32'd0,
             parameter [31:0]  VERIFY_COUNT  = 32'd65536,
             parameter [31:0]  VERIFY_STRIDE = 32'd1,

             // The SD self-test's FAT scan and write test. Bring-up
             // scaffolding; the filesystem layer and the verifier cover the
             // same ground. Benches turn it back on.
             parameter         SELFTEST_FAT = 1'b0,

             // ---- filesystem (src/fat32.v) ----
             // On: blocks are offsets into an image FILE on a FAT32 card. Off:
             // the card is a raw image and block N is sector N. FS_DIR is the
             // 8.3 directory to search, space padded; all spaces = root.
             parameter         FS_ENABLE = 1'b1,
             parameter [87:0]  FS_DIR    = "HD         ",
             // See AUTOPICK in src/fat32.v: off means "do not invent a
             // default".
             parameter         FS_AUTOPICK = 1'b0,
             // See RECURSE in src/fat32.v. Off scans FS_DIR only.
             parameter         FS_RECURSE  = 1'b1,
             // UTC MMDDHHMM build stamp, injected by the build flow (chparam
             // for yosys). Reported on the SD line,
             // word 2, so the running firmware identifies itself. 0 = a build
             // that bypassed the flow.
             parameter [31:0]  BUILD_ID = 32'h0,
             // The system clock this build actually runs at. 50 MHz is the
             // design point. Every protocol constant is in clock cycles, so a
             // different CLK_HZ stretches them in real time. Blocks that need
             // real time (UART baud, SD divider, tick prescalers, the bus
             // latch delay) take CLK_HZ explicitly.
             parameter integer CLK_HZ = 50_000_000,
             // 0 on the ULX3S. rst_n is FIRE1, which idles LOW (PULLMODE=DOWN).
             // The reset term is USE_RSTBTN & ~rst_n, so honoring it holds
             // the whole design in reset forever. POR still resets at
             // configuration.
             parameter         USE_RSTBTN = 1'b0,
             // Clock bring-up probe: drive usb_tx with clk_sys/2^22 (a ~12 Hz
             // square at 50 MHz). At the serial port a slow square reads as a
             // countable trickle of framing bytes, so "is the PLL alive and at
             // the right frequency" is answerable remotely with no LEDs and no
             // working reporter. Debug flow only; never set in a release build.
             parameter         CLK_PROBE = 1'b0,
             // Clocks after /PSTRB falls at which we decide "is this a real
             // host write?", by sampling PR/-W.
             //
             // The decision only works AT the edge. A real byte has PR/-W
             // already low when the strobe falls. A phantom fires its strobe
             // BEFORE the direction flip lands, with PR/-W dropping mid-strobe.
             // Sample 120 ns in and the phantom's PR/-W has already dropped, so
             // the phantom is accepted as real and its undriven bus reads as
             // $FF: a bogus opcode, once a block.
             //
             // 1 is the SMALLEST USABLE VALUE, not a tuning choice. 0 loads
             // arm_cnt with 0, the countdown branch (arm_cnt != 0) never runs,
             // stb_armed never sets, and nothing is ever latched. 1 decides 20
             // ns after the fall.
             parameter integer BUS_ARM_DELAY = 1,
             // Clocks after the strobe's FALL at which a host byte is sampled,
             // inside the 5-15 us valid window described in src/profile_bus.v.
             // The short-strobe guard latches at the rise instead.
             //
             // TIME-based, not cycle-based, and that matters. The working
             // value is 4 us from the write-strobe fall. A fixed cycle count
             // sampled at a different CLK_HZ lands before the host drives the
             // bus, and byte 0 reads $FF again. profile_bus's lat_cnt is 8
             // bits, so this scales safely to CLK_HZ ~63 MHz.
             parameter integer BUS_LATCH_DELAY = 4 * (CLK_HZ / 1_000_000),
             // The boot-time image menu. See src/selector.v and
             // ../../selector/SELECTOR.md.
             parameter         SELECTOR_ENABLE = 1'b1,
             parameter integer FS_MAX_CLUSTERS = 1024,

             // Refuse every write. The socket's write-protect contact is a
             // separate thing and is off by default; see write_protect in the
             // body for why.
             parameter         WRITE_PROTECT = 1'b0,
             parameter         HONOR_WP      = 1'b0,

             // Read one chosen sector at the end of the self-test so the
             // reporter dumps it as hex. A diagnostic, not a feature: off
             // unless asked for.
             parameter         DUMP_ENABLE   = 1'b0,
             parameter [31:0]  DUMP_LBA      = 32'd0,

             // Walk a known pattern onto the bus with no host attached, so the
             // sniffer has something to watch. See src/bus_exercise.v. The
             // dwell is in MICROseconds and must stay well under the sniffer's
             // 100 us idle timeout, or its capture ends before the pattern's
             // first step.
             parameter         EXERCISE_ENABLE   = 1'b0,
             parameter integer EXERCISE_DWELL_US = 20,

             // Stream a real block from the card onto the bus, so the whole
             // read path is checked on hardware and not just the pin mapping.
             // See block_stream.v.
             parameter         STREAM_ENABLE   = 1'b0,
             parameter [23:0]  STREAM_BLOCK    = 24'd0,
             parameter integer STREAM_DWELL_US = 20
             ) (
                input  wire        clk,
                input  wire        rst_n,

                inout  wire [7:0]  pd,
                output wire        xcvr_dir,
                output wire        xcvr_oe_n,
                output wire        pparity,
                input  wire        pstrb_n,
                input  wire        pcmd_n,
                input  wire        prw,
                output wire        pbsy_n,
                input  wire        pres_n,
                output wire        pchk,

                // SmartPort-only host lines. They exist on this board so one
                // DB-25 can serve either protocol; ProFile does not use them.
                // Brought in and displayed rather than left unconnected, so the
                // pins stay constrained and the levels are observable on a
                // board that has never been powered.
                input  wire        ph0,          // DB-25 1
                input  wire        aux7,         // DB-25 7

                output wire        sd_clk,
                output wire        sd_cs_n,
                output wire        sd_mosi,
                input  wire        sd_miso,
                input  wire        sd_cd_n,      // socket card-detect contact, polarity UNVERIFIED
                input  wire        sd_wp_n,      // socket write-protect contact, polarity UNVERIFIED

                output wire [7:0]  led,          // the dev board's own LEDs; see below
                output wire        usb_tx,       // to the board's USB serial, 115200 8N1
                input  wire        usb_rx,       // the other direction: bench command channel
                output wire        usb_tx_alt    // the other candidate ball; see below
                );

   // Internal versions of every pin the diagnostic can take over. Declared
   // here, before first use: yosys accepts a forward reference by inventing
   // an implicit wire. iverilog refuses, which is why `make sim-all` is the
   // gate before flashing.
   wire pbsy_n_i, pparity_i, xcvr_dir_i;
   wire pchk_i = 1'b0;              // grounded in a real drive: "I am here"

   // ---- clock ---------------------------------------------------------------
   // THE DESIGN RUNS AT 50 MHz. Nothing here needs more: the //e's shortest
   // strobe window is 355 ns, which is 17 clocks at 50 MHz, and a host-paced
   // byte is some 640 clocks (PROTOCOL.md (profile-dumper) section 3). The SD
   // command sequencer's next-state logic is the critical path, and 50 MHz
   // gives it comfortable margin.
   wire clk_sys, pll_locked;

   generate if (USE_PLL && FAMILY == "XC7") begin : g_pll7
      // Unsupported reference arm. Artix-7: 100 MHz in, 50 MHz out. 100 / 1 *
      // 10 = 1000 MHz VCO, then / 20. The BUFG is explicit: nextpnr-xilinx
      // can place exactly ONE global buffer.
      wire pll_fb, pll_out;
      MMCME2_BASE #(
                    .CLKIN1_PERIOD(10.0), .DIVCLK_DIVIDE(1),
                    .CLKFBOUT_MULT_F(10.0),
                    .CLKOUT0_DIVIDE_F((CLK_HZ == 40_000_000) ? 25.0 : 20.0)
                    ) u_pll (
                             .CLKIN1(clk), .CLKFBIN(pll_fb), .CLKFBOUT(pll_fb),
                             .CLKOUT0(pll_out), .LOCKED(pll_locked),
                             .RST(1'b0), .PWRDWN(1'b0));
      BUFG u_bufg (.I(pll_out), .O(clk_sys));
   end else if (USE_PLL && FAMILY == "ECP5") begin : g_pll5
      // ecppll -i 25 -o 50: CLKI_DIV 1, CLKFB_DIV 2, CLKOP_DIV 12, giving a
      // 600 MHz VCO from the ULX3S's 25 MHz oscillator.
      EHXPLLL #(
                .CLKI_DIV(1), .CLKFB_DIV(2), .CLKOP_DIV(12), .CLKOP_CPHASE(5),
                .CLKOP_ENABLE("ENABLED"), .FEEDBK_PATH("CLKOP"),
                .CLKOP_FPHASE(0)
                ) u_pll (
                         .CLKI(clk),
                         .CLKFB(clk_sys),
                         .CLKOP(clk_sys),
                         .LOCK(pll_locked),
                         .RST(1'b0), .STDBY(1'b0),
                         .PHASESEL0(1'b0), .PHASESEL1(1'b0), .PHASEDIR(1'b0),
                         .PHASESTEP(1'b0), .PHASELOADREG(1'b0),
                         .PLLWAKESYNC(1'b0), .ENCLKOP(1'b0));
   end else if (USE_PLL) begin : g_pll
      // Unsupported reference arm (iCE40). icepll -i 100 -o 50: DIVR 0,
      // DIVF 7, DIVQ 4, FILTER_RANGE 5.
      SB_PLL40_CORE #(
                      .FEEDBACK_PATH("SIMPLE"),
                      .DIVR(4'd0), .DIVF(7'd7), .DIVQ(3'd4), .FILTER_RANGE(3'b101)
                      ) u_pll (
                               .REFERENCECLK(clk),
                               .PLLOUTCORE(clk_sys),
                               .LOCK(pll_locked),
                               .BYPASS(1'b0),
                               .RESETB(1'b1));
   end else if (FAMILY == "XC7") begin : g_div4
      // Unsupported reference arm (XC7 without an MMCM): a fabric
      // divide-by-4 from the 100 MHz oscillator through a BUFG. 25 MHz
      // doubles every cycle-denominated constant in real time. CLK_HZ
      // gives the UART and SD the real clock rate.
      reg [1:0] div4 = 2'd0;
      always @(posedge clk) div4 <= div4 + 2'd1;
      // A real BUFG: unbuffered fabric clocking causes hold violations
      // between BRAMs and fabric flops.
      BUFG u_divbuf (.I(div4[1]), .O(clk_sys));
      assign pll_locked = 1'b1;
   end else begin : g_nopll
      assign clk_sys    = clk;
      assign pll_locked = 1'b1;
   end endgenerate

   // ---- reset --------------------------------------------------------------
   // POWER-ON RESET, and it is not optional. Flops leave configuration at
   // zero. Several of this design's synchronizers hold ACTIVE-LOW bus lines,
   // so zero means "asserted". Without a POR the target wakes up believing
   // the host is commanding it, and drives the bus before it knows better. A
   // counter starting at zero IS deterministic out of configuration, so we
   // use that.
   reg [7:0] por_cnt = 8'd0;
   wire      por = ~por_cnt[7] | ~pll_locked;
   always @(posedge clk_sys) if (!por_cnt[7]) por_cnt <= por_cnt + 1'b1;

   // HONOR_PRES: does the host's /PRES line reset the target?
   //   1  faithful: a host reset resets the drive, which is what a real ProFile
   //      does and what the reader's 'R' command expects.
   //   0  the Teensy build's setting. On some //e setups /PRES idles asserted,
   //      which with HONOR_PRES=1 pins the target in reset forever and looks
   //      exactly like a board that is not working.
   // led[3] shows /PRES live and is NOT gated by reset, so if the board seems
   // not to be working, look there first before changing this. DEFAULT 0, the
   // Teensy build's setting. With no DB-25 cable attached, DB-25 pin 21 floats
   // and U2 drives an indeterminate level into pres_n. With HONOR_PRES=1 the
   // entire design then sits in reset: no LEDs, no UART, nothing. That is
   // indistinguishable from a board that is not working. Set it to 1 only once
   // a host is actually on the other end and its /PRES has been observed
   // idling high on led[3].
   localparam HONOR_PRES = 1'b0;
   // ...and the protocol layer does not honor it either. The //e interface
   // card idles /PRES LOW and holds it low even while issuing commands, so
   // treating /PRES-low as a reset makes the drive ignore every command. The
   // n_pres counter stays, because seeing that the line is pulsed and left
   // asserted is useful; acting on it is not.
   localparam HONOR_PRES_TGT = 1'b0;

   // USE_RSTBTN: see the parameter. Nothing in this design needs the button.
   wire       rst = por | (USE_RSTBTN & ~rst_n) | (HONOR_PRES & ~pres_n);

   wire [7:0] host_data, bus_in;
   // Diagnostic: the same byte at the strobe's LEADING edge. If it differs
   // from host_data, the host put the byte on the bus part way through its
   // own strobe, which no counter can show.
   wire [7:0] host_data_lead;
   wire       stb_idle, trace_arm;
   wire [7:0] tr0, tr1, tr2, tr3;
   wire [3:0] tr_prw;   // PR/-W at each traced strobe; bit N = strobe N
   wire [7:0] tr_d8, tr_d32, tr_d128;  // bus re-read after strobe 0
   wire [39:0] rise_vals;  // bus_in at each latched byte's qualifying rise
   wire [3:0]  tr_nstb;
   wire [15:0] dbg_t0;
   wire        host_stb, read_stb, host_read;

   // The block face is muxed the same way the data bus is: the target owns it
   // normally, the streamer owns it in the stream build where the target is
   // held in reset. See src/block_stream.v.
   wire [23:0] tgt_blk_addr, str_blk_addr;
   wire        tgt_blk_rd,   str_blk_rd;
   wire [9:0]  tgt_buf_addr, str_buf_addr;
   wire [7:0]  str_data;
   wire        str_drive_en, str_active;
   wire [3:0]  str_state;

   wire [23:0] blk_addr = STREAM_ENABLE ? str_blk_addr : tgt_blk_addr;
   wire        blk_rd   = STREAM_ENABLE ? str_blk_rd   : tgt_blk_rd;
   wire [9:0]  buf_addr = STREAM_ENABLE ? str_buf_addr : tgt_buf_addr;

   // ---- bus exerciser ------------------------------------------------------
   // With no host, the drive correctly sits idle and drives nothing, and the
   // sniffer is read-only, so neither device ever puts anything on the wire.
   // This makes the drive talk first so the sniffer has something to see. It is
   // generated away entirely when off. It drops the bus the instant a host
   // appears; see src/bus_exercise.v.
   wire [7:0]  tgt_data;
   wire        tgt_drive_en;
   wire [7:0]  exr_data;
   wire        exr_drive_en, exr_bsy_n;
   wire [3:0]  exr_step;

   generate
      if (EXERCISE_ENABLE) begin : g_exercise
         // NOT gated on host_seen here, and the target is held in reset below.
         //
         // With no host the host-driven lines FLOAT, because U2 is fixed DB-25
         // to FPGA and nothing is driving its inputs. Floating CMOS reads low,
         // and low on an active-low line means asserted. So the drive sees a
         // permanent phantom /PCMD and latches host_seen. It then parks in
         // S_RESP waiting for PR/-W and holds tgt_busy forever. Gating the
         // exerciser on those signals would mean it never drives anything at
         // all.
         bus_exercise #(.CLK_HZ(CLK_HZ), .DWELL_US(EXERCISE_DWELL_US)) u_exr (
                                                                              .clk(clk_sys), .rst(rst), .enable(1'b1),
                                                                              .inhibit(1'b0),
                                                                              .drive_en(exr_drive_en), .drive_data(exr_data),
                                                                              .bsy_n(exr_bsy_n), .step_out(exr_step));
         // Same build, second stage: after the pin walk has checked the
         // wiring, stream a real block so the DATA PATH is checked too.
      end else begin : g_no_exercise
         assign exr_drive_en = 1'b0;
         assign exr_data     = 8'h00;
         assign exr_bsy_n    = 1'b1;
         assign exr_step     = 4'd0;
      end
   endgenerate

   generate
      if (STREAM_ENABLE) begin : g_stream
         block_stream #(.CLK_HZ(CLK_HZ), .DWELL_US(STREAM_DWELL_US),
                        .BLOCK(STREAM_BLOCK)) u_str (
                                                     .clk(clk_sys), .rst(rst), .enable(selftest_done & selftest_ok),
                                                     .blk_addr(str_blk_addr), .blk_rd(str_blk_rd), .blk_ready(blk_ready),
                                                     .buf_addr(str_buf_addr), .buf_rdata(buf_rdata),
                                                     .drive_en(str_drive_en), .drive_data(str_data),
                                                     .active(str_active), .dbg_state(str_state));
      end else begin : g_no_stream
         assign str_blk_addr = 24'd0;
         assign str_blk_rd   = 1'b0;
         assign str_buf_addr = 10'd0;
         assign str_drive_en = 1'b0;
         assign str_active   = 1'b0;
         assign str_data     = 8'h00;
         assign str_state    = 4'd0;
      end
   endgenerate

   // The target always wins, then the streamer, then the pin walk. Only one of
   // the diagnostics is ever built in, so the priority only documents intent.
   wire [7:0] drive_data = tgt_drive_en ? tgt_data
              : str_drive_en ? str_data
              : exr_data;
   wire       drive_en   = tgt_drive_en | str_drive_en | exr_drive_en;

   // drive_en/drive_data come from the MUX below, not from tgt_* directly:
   // the mux is the consumer, and feeding it the target's raw outputs would
   // mean the exerciser drives nothing at all.
   profile_bus #(.FAMILY(FAMILY),
                 .ARM_DELAY(BUS_ARM_DELAY),
                 .LATCH_DELAY(BUS_LATCH_DELAY)) u_bus (
                                                       .clk(clk_sys), .rst(rst),
                                                       .pd(pd), .xcvr_dir(xcvr_dir_i), .xcvr_oe_n(xcvr_oe_n), .pparity(pparity_i),
                                                       .pstrb_n(pstrb_n), .pcmd_n(pcmd_n), .prw(prw),
                                                       .drive_en(drive_en), .drive_data(drive_data),
                                                       .host_data(host_data), .host_stb(host_stb), .read_stb(read_stb),
                                                       .host_data_lead(host_data_lead), .stb_idle(stb_idle),
                                                       .trace_arm(trace_arm), .tr0(tr0), .tr1(tr1), .tr2(tr2), .tr3(tr3),
                                                       .tr_prw(tr_prw), .tr_d8(tr_d8), .tr_d32(tr_d32), .tr_d128(tr_d128),
                                                       .rise_vals(rise_vals),
                                                       .tr_nstb(tr_nstb),
                                                       .bus_in(bus_in), .host_read(host_read));

   wire        blk_wr, blk_ready;
   wire [7:0]  buf_rdata, buf_wdata;
   wire        buf_we;
   wire        ev_nak, ev_abandon, ev_read_ok, ev_write_ok, ev_badop, ev_cmd;
   wire [7:0]  blk_op;
   wire        write_refused;
   // Hoisted above the status logic below, which reads all three. Declaring
   // them where they are first ASSIGNED instead puts them after their first
   // use, and yosys accepts that with no warning while iverilog does not.
   wire        out_of_range;
   wire        oor_read;
   wire        sd_ready;
   wire        sd_err;
   localparam [4:0] SD_S_READY = 5'd9;
   wire [4:0]       sd_dbg;
   // ---- filesystem resolver -------------------------------------------------
   wire             fs_req, fs_ready, fs_ok;
   wire [31:0]      fs_lba, fs_image_blocks, fs_data_start, fs_nclusters;
   wire [31:0]      fs_dbg_c1, fs_dbg_c2, fs_dbg_fat_start, fs_dbg_persist;
   wire [63:0]      fs_dbg_cfg;
   wire [7:0]       fs_dbg_plen;
   wire [3:0]       fs_err, fs_spc_log;
   wire [4:0]       fs_state;
   wire             fs_need_select;
   wire [7:0]       fs_img_count, fs_img_index, fs_cat_data, fs_wr_data;
   wire             fs_req_wr;

   // ---- the boot-time selector ---------------------------------------------
   wire             sel_hit;
   wire [7:0]       sel_data;
   wire [11:0]      sel_cat_addr;
   wire [5:0]       sel_idx;
   wire             sel_stb;
   reg              sel_armed;
   wire             sel_want;   // driven below; dbg_txn reads it
   wire             sel_boot_hit;
   reg              window_live = 1'b0;   // menu phase; see the overloading note
   reg              knock_hit = 1'b0;     // the A2E-check knock; see the detector
   reg              knock_menu = 1'b0;
   reg [23:0]       prev_blk;
   wire [15:0]      fs_map_idx;
   wire [31:0]      fs_map_cluster;
   // Ownership arbitration between the router and fat32: see src/sd_arb.v
   // for the corruption this prevents. fs_fire keeps its name so downstream
   // wiring reads unchanged.
   wire             fs_fire, sd_own_fs;
   wire             r_fire_rd, r_fire_wr, r_done, r_err, f_done, f_err;
   // (the sd_arb instance lives below, after sd_rd/sd_wr are declared)
   wire [7:0]       sd_retries;

   // ---- the four status bytes ----------------------------------------------
   // Sent ahead of the data on every read and after the data on every write
   // (PROTOCOL.md section 3). Only two bits are ours to set, and both live in
   // byte 2:
   //
   //   $80  the drive was reset since the last command. Reported by the FIRST
   //        command after power-up and then not again.
   //   $40  the operation failed: a block number outside what we serve, a write
   //        that was refused, or an SD error underneath.
   //
   // The reset bit has to be LATCHED when the command is decoded rather than
   // read live when the status bytes go out. Clearing it at decode time and
   // reporting it at send time are several states apart, so doing both to one
   // flag means the single command that should report it never does.
   // Declared here, not beside dbg_txn: it is USED by the profile_target
   // instantiation above, and a wire declared after its use is an error.
   wire [47:0]      dbg_frame;
   wire [39:0]      dbg_walk;   // last five FSM transitions, newest low
   wire [47:0]      dbg_badframe;  // the last frame that decoded as a bad opcode
   wire [39:0]      dbg_dt;        // us gaps between that frame's counted strobes
   wire [7:0]       dbg_f0_lead;
   wire [2:0]       dbg_cmd_idx;
   // Declared here and driven far below, beside the input synchronizers:
   // dbg_txn uses them, and a wire declared after its use is an error.
   reg [6:0]        n_pres;
   wire             pres_lvl;
   // The HOST-driven half of the loom, which the bus exerciser cannot reach:
   // it only drives what the DRIVE owns. Live levels plus edge counts, so a
   // line can be checked from the monitor by touching the card's soft
   // switches and watching the count move.
   reg [7:0]        n_pcmd, n_pstrb, n_prw;
   wire             pcmd_lvl, pstrb_lvl, prw_lvl;

   reg              was_reset, rep_reset, op_failed;
   always @(posedge clk_sys) begin
      if (rst) begin
         was_reset <= 1'b1; rep_reset <= 1'b0; op_failed <= 1'b0;
      end else begin
         if (ev_cmd) begin
            rep_reset <= was_reset;      // what THIS command will report
            was_reset <= 1'b0;
            op_failed <= 1'b0;           // every command starts clean
         end
         if (out_of_range || write_refused || r_err) op_failed <= 1'b1;
      end
   end
   wire [7:0] status2 = {rep_reset, op_failed, 6'b000000};

   // ---- write protect -------------------------------------------------------
   // WRITE_PROTECT is the build-time switch. The socket's own contact is NOT
   // trusted by default: the constraints files record that these are bare
   // switch contacts with no standardized polarity. Neither has been
   // metered on this board. Honoring an unmetered contact would either protect
   // nothing or refuse every write, and both look like firmware faults.
   wire       write_protect = WRITE_PROTECT | (HONOR_WP & ~sd_wp_n);

   // ---- no media ------------------------------------------------------------
   // The card never came up. Serve the self-verifying pattern instead of the
   // block buffer's leftovers, exactly as the Teensy build does with no image
   // open. This is also what makes DB-25 bring-up possible with an empty
   // socket.
   wire       no_media = ~sd_ready;

   // High whenever the ProFile side is mid-transaction. Declared here, above
   // its use: yosys accepts a forward reference with no warning. iverilog
   // rejects it.
   wire       tgt_busy;

   // Held in reset in the exercise build. That build is by definition a no-host
   // bench. The phantom /PCMD above would otherwise have it responding to a
   // host that is not there. Two drivers inside the same FPGA fighting over
   // the bus is not a useful diagnostic. It also means tgt_busy stays low,
   // which is what lets the exerciser own the pins. /PRES resets the PROTOCOL,
   // not the whole design.
   //
   // On some setups /PRES idles DEASSERTED, so the reason HONOR_PRES defaults
   // to 0 does not apply. The line is still pulsed many times during a boot
   // attempt and left ASSERTED afterward. That is the card's drive-reset
   // one-shot being re-triggered from the card ROM's retry path (259 Q4,
   // $C50C/$C508; docs/APPLE2_ProFile_card_map.md section 8 step 4). The ROM
   // this was developed against is 341-0299-B in a //e; the ROM behavior
   // described in this file is that ROM's. A real ProFile resets when it
   // fires. Ignoring it means the ROM retries against a drive that is still
   // mid-transaction from the attempt it just gave up on, and the two can
   // never resynchronize.
   //
   // Routed to tgt_rst, not rst. A ProFile reset means "start the protocol
   // over". Tearing down the SD engine and the FAT32 resolve as well would
   // take far longer than the ROM waits, turning a recoverable retry into a
   // drive that does not answer. The block buffer and the resolved image
   // survive.
   wire       pres_rst = 1'b0;   // see HONOR_PRES_TGT
   wire       tgt_rst = rst | pres_rst | EXERCISE_ENABLE | STREAM_ENABLE;

   profile_target u_target (
                            .clk(clk_sys), .rst(tgt_rst), .dbg_frame(dbg_frame), .dbg_walk(dbg_walk), .dbg_badframe(dbg_badframe),
                            .dbg_dt(dbg_dt),
                            .host_data_lead(host_data_lead), .dbg_f0_lead(dbg_f0_lead),
                            .dbg_cmd_idx(dbg_cmd_idx), .stb_idle(stb_idle), .dbg_t0(dbg_t0),
                            .trace_arm(trace_arm),
                            .pcmd_n(pcmd_n), .pbsy_n(pbsy_n_i),
                            .drive_en(tgt_drive_en), .drive_data(tgt_data),
                            .host_data(host_data), .host_stb(host_stb), .read_stb(read_stb),
                            .bus_in(bus_in), .host_read(host_read),
                            .blk_addr(tgt_blk_addr), .blk_rd(tgt_blk_rd), .blk_wr(blk_wr), .blk_ready(blk_ready),
                            .buf_addr(tgt_buf_addr), .buf_rdata(buf_rdata), .buf_wdata(buf_wdata), .buf_we(buf_we),
                            .status0(8'h00), .status1(8'h00), .status2(status2), .status3(8'h00),
                            .tgt_busy(tgt_busy),
                            .ev_nak(ev_nak), .ev_abandon(ev_abandon), .ev_badop(ev_badop),
                            .ev_cmd(ev_cmd), .blk_op(blk_op),
                            .ev_read_ok(ev_read_ok), .ev_write_ok(ev_write_ok));

   wire [9:0] ram_addr, sd_buf_addr;
   wire [7:0] ram_rdata, ram_wdata, sd_buf_wdata;
   wire       ram_we, sd_rd, sd_wr, sd_done, sd_buf_we;
   wire [31:0] sd_lba;

   // ---- geometry -----------------------------------------------------------
   // How big the image is, and therefore what we claim and what we serve.
   //
   // Off the card when there is a filesystem (the FAT32 directory entry
   // carries the file size), from the parameter otherwise.
   //
   // REGISTERED, and that matters. Left combinational, this is the critical
   // path of the whole design: fs_ok feeds the mux, the mux feeds geometry's
   // comparators and alignment, and geometry feeds the 256-bit load of the
   // reporter's shift register, all in one cycle. These values change ONCE,
   // when the image is resolved at startup, so a cycle of latency is free.
   reg [31:0]  image_blocks;
   always @(posedge clk_sys)
     image_blocks <= (FS_ENABLE && fs_ok) ? fs_image_blocks : IMAGE_BLOCKS[31:0];

   wire [15:0] claim_w, served_w;
   wire        geo_dec_w;

   geometry u_geo (
                   .image_blocks(image_blocks), .serve_full(SERVE_FULL_IMAGE),
                   .claim_blocks(claim_w), .served_blocks(served_w),
                   .decoupled(geo_dec_w));

   // Registered for the same reason as image_blocks above: these feed the spare
   // table, the router's range check and the reporter, and they are static once
   // the image has been resolved.
   reg [15:0] claim_blocks, served_blocks;
   reg        geo_decoupled;
   always @(posedge clk_sys) begin
      claim_blocks  <= claim_w;
      served_blocks <= served_w;
      geo_decoupled <= geo_dec_w;
   end

   block_router #(.FS_ENABLE(FS_ENABLE)) u_router (
                                                   .clk(clk_sys),
                                                   .claim_blocks(claim_blocks), .served_blocks(served_blocks),
                                                   .no_media(no_media),
                                                   .write_protect(write_protect), .write_refused(write_refused),
                                                   .sel_hit(sel_hit), .sel_data(sel_data), .window_live(window_live),
                                                   .fs_ok(fs_ok), .fs_ready(fs_ready), .fs_data_start(fs_data_start), .fs_spc_log(fs_spc_log),
                                                   .fs_map_idx(fs_map_idx), .fs_map_cluster(fs_map_cluster),
                                                   .blk_addr(blk_addr), .blk_rd(blk_rd), .blk_wr(blk_wr),
                                                   .blk_ready(blk_ready),
                                                   .buf_addr(buf_addr), .buf_rdata(buf_rdata), .buf_wdata(buf_wdata), .buf_we(buf_we),
                                                   .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_lba(sd_lba), .sd_done(r_done), .sd_err(r_err),
                                                   .ram_addr(ram_addr), .ram_rdata(ram_rdata), .ram_wdata(ram_wdata), .ram_we(ram_we),
                                                   .out_of_range(out_of_range), .oor_read(oor_read));

   // ---- SD self-test -------------------------------------------------------
   // With nothing plugged into the DB-25 there is no host, and nothing will
   // ever ask for a block. Ask for one ourselves: as soon as the card reports
   // ready, read sector 0 and check for the $55 $AA boot signature at offsets
   // 510 and 511, which any formatted card has. That turns a bare board with an
   // SD socket into a complete, self-contained test whose entire result is
   // readable off the LEDs.
   //
   // It runs exactly once, before any host traffic, so it can never collide
   // with a real request.
   wire [3:0]  sd_err_code;
   wire [5:0]  sd_dbg_cmd;
   wire [7:0]  sd_dbg_last;
   wire [7:0]  sd_dbg_r1;
   wire [7:0]  sd_dbg_r1a;
   wire [31:0] sd_dbg_r7;
   wire [127:0] sd_dbg_cap;
   wire         sd_dbg_v1, sd_dbg_ccs;
   wire [7:0]   sd_crc_errs, sd_cmds, sd_r55, sd_r41, sd_kh;
   wire [2:0]   sd_k, sd_ph, sd_phok;
   wire         sd_phf, sd_fast;
   wire         sd_clk_i, sd_cs_n_i, sd_mosi_i;

   // ---- PIN WALK ------------------------------------------------------------
   // When the card never answers, the question is which wire is not arriving.
   // The init burst is a poor thing to chase with a DMM: bursts a few
   // milliseconds long, a second apart. So when initialization has given up,
   // take the three output pins over and drive them one at a time, each held
   // LOW for two whole seconds while the UART says which one it is. A meter on
   // the socket terminal then reads a steady 0 V exactly when the report names
   // that pin, or it does not. Either result answers the question.
   //
   //   phase 0  /CS  low
   //   phase 1  MOSI low
   //   phase 2  CLK  low
   //   phase 3  all high      the reference reading
   //
   // MISO is an input and cannot be walked, so the report carries its live
   // level instead: short MISO to GND at the socket and msl should follow to 0.
   localparam [4:0] SD_S_ERR = 5'd16;

   // ONLY after several consecutive failures. The walk drives CS, MOSI and CLK
   // for seconds at a time, and sd_spi retries initialization about once a
   // second. An unconditional walk would sit on the bus between attempts, and
   // the card would see CS or MOSI held low for a second and a half before
   // each retry. The first attempt after power-up then gets all the way
   // through ACMD41, and every retry afterward gets no answer to CMD0 at all.
   //
   // Five failures is well past the point where a retry was going to help, and
   // leaves the diagnostic available for the next broken wire.
   reg [2:0]        fail_n;
   reg              sd_err_d;
   always @(posedge clk_sys) begin
      sd_err_d <= (sd_dbg == SD_S_ERR);
      if (rst || sd_ready) fail_n <= 0;
      else if ((sd_dbg == SD_S_ERR) && !sd_err_d && fail_n != 3'd5)
        fail_n <= fail_n + 3'd1;
   end

   wire       diag_en = (sd_dbg == SD_S_ERR) && !sd_ready && (fail_n == 3'd5);
   reg [27:0] pw_t;
   reg [2:0]  pw_ph;
   always @(posedge clk_sys) begin
      if (rst) begin pw_t <= 0; pw_ph <= 0; end
      else if (pw_t == 28'd75_000_000) begin       // 1.5 s at 50 MHz
         pw_t  <= 0;
         pw_ph <= (pw_ph == 3'd6) ? 3'd0 : pw_ph + 1'b1;
      end else pw_t <= pw_t + 1'b1;
   end

   // Walk EVERY FPGA output that reaches J2, not just the SD three. If the
   // socket's CS wire went to the wrong pin, driving the SD lines will never
   // move it, and the walk says nothing. Driving all of them says exactly
   // which FPGA output that wire is really attached to. Park the meter on the
   // terminal and read off the phase name when it drops.
   assign sd_cs_n  = diag_en ? (pw_ph != 3'd0) : sd_cs_n_i;
   assign sd_mosi  = diag_en ? (pw_ph != 3'd1) : sd_mosi_i;
   assign sd_clk   = diag_en ? (pw_ph != 3'd2) : sd_clk_i;
   assign pbsy_n   = diag_en ? (pw_ph != 3'd3)
     : EXERCISE_ENABLE ? (pbsy_n_i & exr_bsy_n)
       : pbsy_n_i;
   assign pparity  = diag_en ? (pw_ph != 3'd4) : pparity_i;
   assign pchk     = diag_en ? (pw_ph == 3'd5) : pchk_i;   // normally low: this one goes HIGH
   assign xcvr_dir = diag_en ? (pw_ph == 3'd6) : xcvr_dir_i;

   // A read request is HELD, not pulsed. sd_spi only samples rd in S_READY, and
   // a one-cycle pulse issued while it is still leaving S_DONE is simply lost.
   // Assert the want, convert it to a pulse when the target is actually
   // listening.

   // The block written by the test. Derived from the address so a read-back can
   // be checked byte by byte without storing a copy, and deliberately not a
   // constant so a stuck bus cannot pass.
   function [7:0] wpat(input [9:0] a);
      wpat = {a[7:0]} ^ 8'h5A;
   endfunction
   reg        rq_want;
   reg [31:0] rq_lba;
   reg        rq_wr;
   wire       rq_fire = rq_want && (sd_dbg == SD_S_READY);


   // Declared before use: yosys takes a forward reference with no warning.
   // iverilog rejects it.
   wire       rpt_tx, ver_tx, ver_active, ver_done, ver_req;
   wire       cap_tx, cap_dumping;
   wire [31:0] ver_lba;
   wire        ver_fire = ver_req && (sd_dbg == SD_S_READY);


   reg         ready_d, selftest_run, selftest_done, selftest_ok, sig_lo, sig_hi;
   reg         stage2;
   reg [31:0]  part_lba;

   // ---- FAT32 geometry, snooped out of the boot sector as it streams past
   // ---- No second read needed: the BPB fields are at fixed offsets in the
   // sector stage 2 already fetches.
   reg [7:0]   bpb_spc;        // $0D sectors per cluster
   reg [15:0]  bpb_rsvd;       // $0E reserved sectors
   reg [7:0]   bpb_nfat;       // $10 number of FATs
   reg [31:0]  bpb_fatsz;      // $24 sectors per FAT
   reg [31:0]  fat_start, data_start;

   // ---- the scan ----
   reg [31:0]  fat_sec;        // FAT sector being read, relative to fat_start
   reg [31:0]  ent;            // entry being assembled, little endian
   reg [31:0]  free_count;
   reg [31:0]  first_free;     // cluster number, 0 = none found yet
   reg [31:0]  target_lba;     // first sector of that cluster
   reg [2:0]   fstage;         // 0 MBR, 1 BPB, 2 scan FAT, 3 write, 4 verify, 5 restore, 6 done
   reg         wtest_ok, wtest_run;
   reg [31:0]  wbad;           // bytes that differed on read-back
   reg [31:0]  wcmp;           // bytes actually compared

   function [2:0] log2_spc(input [7:0] v);
      casez (v)
        8'b1???????: log2_spc = 3'd7; 8'b01??????: log2_spc = 3'd6;
        8'b001?????: log2_spc = 3'd5; 8'b0001????: log2_spc = 3'd4;
        8'b00001???: log2_spc = 3'd3; 8'b000001??: log2_spc = 3'd2;
        8'b0000001?: log2_spc = 3'd1; default:     log2_spc = 3'd0;
      endcase
   endfunction
   wire [2:0] spc_log = log2_spc(bpb_spc);

   always @(posedge clk_sys) begin
      ready_d <= sd_ready;
      if (rst) begin
         selftest_run <= 1'b0; selftest_done <= 1'b0; selftest_ok <= 1'b0;
         sig_lo <= 1'b0; sig_hi <= 1'b0; ready_d <= 1'b0;
         stage2 <= 1'b0; part_lba <= 0; rq_want <= 1'b0; rq_lba <= 0; rq_wr <= 1'b0;
         fstage <= 0; fat_sec <= 0; free_count <= 0; first_free <= 0; target_lba <= 0;
         wtest_ok <= 1'b0; wtest_run <= 1'b0; wbad <= 0; wcmp <= 0; ent <= 0;
         bpb_spc <= 0; bpb_rsvd <= 0; bpb_nfat <= 0; bpb_fatsz <= 0;
      end else begin
         if (sd_ready && !ready_d && !selftest_run) begin
            selftest_run <= 1'b1;
            rq_want      <= 1'b1;
            rq_lba       <= 32'd0;
         end
         if (rq_fire) begin rq_want <= 1'b0; rq_wr <= 1'b0; end
         // Stage 1 only: the verdict is about sector 0, and the sector the MBR
         // points at has no reason to carry a $55 $AA of its own.
         if (selftest_run && !selftest_done && !stage2 && sd_buf_we) begin
            if (sd_buf_addr == 10'd510) sig_lo <= (sd_buf_wdata == 8'h55);
            if (sd_buf_addr == 10'd511) sig_hi <= (sd_buf_wdata == 8'hAA);
         end
         // Follow the MBR. Sector 0 read and verified is good. Reading the
         // sector the partition table POINTS AT checks that the address
         // actually reaches the card, which reading sector 0 alone cannot. The
         // four bytes at $1C6 are the partition's start LBA, little endian.
         if (selftest_run && stage2 && (fstage == 3'd1) && sd_buf_we) begin
            case (sd_buf_addr)
              10'h00D: bpb_spc         <= sd_buf_wdata;
              10'h00E: bpb_rsvd[7:0]   <= sd_buf_wdata;
              10'h00F: bpb_rsvd[15:8]  <= sd_buf_wdata;
              10'h010: bpb_nfat        <= sd_buf_wdata;
              10'h024: bpb_fatsz[7:0]  <= sd_buf_wdata;
              10'h025: bpb_fatsz[15:8] <= sd_buf_wdata;
              10'h026: bpb_fatsz[23:16]<= sd_buf_wdata;
              10'h027: bpb_fatsz[31:24]<= sd_buf_wdata;
              default: ;
            endcase
         end
         if (selftest_run && !selftest_done && !stage2 && sd_buf_we) begin
            if (sd_buf_addr == 10'h1C6) part_lba[7:0]   <= sd_buf_wdata;
            if (sd_buf_addr == 10'h1C7) part_lba[15:8]  <= sd_buf_wdata;
            if (sd_buf_addr == 10'h1C8) part_lba[23:16] <= sd_buf_wdata;
            if (sd_buf_addr == 10'h1C9) part_lba[31:24] <= sd_buf_wdata;
         end
         // ---- staged probe
         // -----------------------------------------------------
         //   0  sector 0, check the $55 $AA signature, take the partition LBA
         //   1  the boot sector, snoop the BPB (above)
         //   2  walk the WHOLE FAT counting free clusters and noting the first
         //   3  write a pattern into that free cluster
         //   4  read it back and compare every byte
         //   5  zero it again, so the card is left as it was found
         //
         // The scan is exhaustive rather than stopping at the first zero entry:
         // a free-cluster count is the evidence that the FAT was actually
         // parsed and not merely indexed into, and it costs one pass at 1.5 ms
         // a sector.
         if (selftest_run && !selftest_done && sd_done) begin
            case (fstage)
              // The FAT scan, free-cluster search and write test below were
              // BRING-UP SCAFFOLDING: they existed to check the SD path when
              // nothing else did. src/fat32.v now parses the same structures
              // for real, and the verifier checks whole volumes, so a shipping
              // build does not need any of it. It is also not cheap: it
              // carries a dozen 32-bit accumulators through the whole design.
              3'd0: if (SELFTEST_FAT && !sd_err && sig_lo && sig_hi && part_lba != 0) begin
                 fstage  <= 3'd1;
                 stage2  <= 1'b1;
                 rq_want <= 1'b1;
                 rq_lba  <= part_lba;
              end else begin
                 selftest_done <= 1'b1;
                 // With the scan compiled out, reaching here having read a
                 // sector with a good signature IS the pass condition.
                 selftest_ok   <= !SELFTEST_FAT && !sd_err && sig_lo && sig_hi;
              end

              3'd1: if (!sd_err && bpb_fatsz != 0 && bpb_spc != 0) begin
                 fat_start  <= part_lba + {16'd0, bpb_rsvd};
                 // nfat is 1 or 2 on any real volume: shift, do not multiply.
                 data_start <= part_lba + {16'd0, bpb_rsvd}
                               + ((bpb_nfat == 8'd1) ? bpb_fatsz
                                  : (bpb_nfat == 8'd2) ? {bpb_fatsz[30:0], 1'b0}
                                  : {bpb_fatsz[29:0], 2'b00});
                 fat_sec    <= 0;
                 free_count <= 0;
                 first_free <= 0;
                 fstage     <= 3'd2;
                 rq_want    <= 1'b1;
                 rq_lba     <= part_lba + {16'd0, bpb_rsvd};
              end else begin
                 selftest_done <= 1'b1;
                 selftest_ok   <= 1'b0;
              end

              3'd2: if (sd_err) begin
                 selftest_done <= 1'b1; selftest_ok <= 1'b0;
              end else if (fat_sec + 1 < bpb_fatsz) begin
                 fat_sec <= fat_sec + 1;
                 rq_want <= 1'b1;
                 rq_lba  <= fat_start + fat_sec + 1;
              end else if (first_free >= 2) begin
                 // sectors per cluster is a power of two, always: shift by its
                 // log rather than instantiating a multiplier.
                 target_lba <= data_start + ((first_free - 2) << spc_log);
                 fstage     <= 3'd3;
                 wtest_run  <= 1'b1;
                 rq_want    <= 1'b1;
                 rq_wr      <= 1'b1;
                 rq_lba     <= data_start + ((first_free - 2) << spc_log);
              end else begin
                 selftest_done <= 1'b1;   // no free cluster: do not write
                 selftest_ok   <= 1'b0;
              end

              3'd3: begin                       // write done, read it back
                 wbad    <= 0;
                 wcmp    <= 0;
                 fstage  <= 3'd4;
                 rq_want <= 1'b1;
                 rq_lba  <= target_lba;
              end

              3'd4: begin                       // compared on the fly below
                 // 512 bytes actually compared, all equal, no error.
                 // Without the count, a read that returned nothing passes.
                 wtest_ok <= (wbad == 0) && (wcmp == 32'd512) && !sd_err;
                 fstage   <= 3'd5;
                 rq_want  <= 1'b1;
                 rq_wr    <= 1'b1;
                 rq_lba   <= target_lba;   // put it back to zeros
              end

              // One last read of a chosen sector, so the reporter's hex dump
              // shows that sector instead of sector 0. A CRC can say THAT two
              // sectors differ but never HOW. Dumping the card's copy and
              // diffing it against the image says whether the difference is
              // filesystem metadata (a free-cluster count, a directory
              // timestamp) or something the FPGA got wrong.
              3'd5: begin
                 wtest_run <= 1'b0;
                 if (DUMP_ENABLE) begin
                    fstage  <= 3'd6;
                    rq_want <= 1'b1;
                    rq_lba  <= DUMP_LBA;
                 end else begin
                    selftest_done <= 1'b1;
                    selftest_ok   <= sig_lo & sig_hi;
                 end
              end

              default: begin
                 wtest_run     <= 1'b0;
                 selftest_done <= 1'b1;
                 selftest_ok   <= sig_lo & sig_hi;
              end
            endcase
         end

         // FAT entries, assembled little endian as the sector streams past.
         // Entry index is (sector * 128) + (offset / 4); 0 and 1 are reserved.
         if (selftest_run && (fstage == 3'd2) && sd_buf_we) begin
            case (sd_buf_addr[1:0])
              2'd0: ent[7:0]   <= sd_buf_wdata;
              2'd1: ent[15:8]  <= sd_buf_wdata;
              2'd2: ent[23:16] <= sd_buf_wdata;
              2'd3: begin
                 if ({sd_buf_wdata, ent[23:0]} == 32'd0) begin
                    free_count <= free_count + 1;
                    if (first_free == 0)
                      first_free <= (fat_sec << 7) + {22'd0, sd_buf_addr[9:2]};
                 end
              end
            endcase
         end

         // Read-back comparison for the write test.
         if (selftest_run && (fstage == 3'd4) && sd_buf_we) begin
            wcmp <= wcmp + 1;
            if (sd_buf_wdata != wpat(sd_buf_addr)) wbad <= wbad + 1;
         end
      end
   end

   // ---- SPI clock ceiling sweep --------------------------------------------
   // The card is up at 100 kHz. Find the real limit instead of guessing it.
   //
   // Read the SAME sector at each rate and compare a checksum against the one
   // taken at the slowest rate, which is known good. That tests the data, not
   // just whether the command completed: a link that returns a sector with two
   // bits wrong passes a "did it error" test and fails this one.
   //
   // A phase lasts DIV+1 cycles, so the bit rate is 50e6 / (2*(DIV+1)); mind
   // the +1. The table reaches DIV=0, which is 25 MHz, the SD SPI ceiling.
   //
   //   index    0     1     2      3      4      5      6      7
   //   DIV    250    63    16      8      4      2      1      0
   //   rate  99.6k  391k  1.47M  2.78M  5.0M   8.3M  12.5M   25M
   reg [7:0] sd_speed;
   reg [2:0] sw_i;
   reg [7:0] sw_pass;               // bitmask: which rates returned good data
   reg [2:0] sw_best;
   reg       sw_run, sw_done, sw_busy;
   reg [15:0] sw_ck, sw_ref;
   reg        sw_want;
   wire       sw_fire = sw_want && (sd_dbg == SD_S_READY);

   function [7:0] speed_of(input [2:0] i);
      case (i)
        3'd0: speed_of = 8'd250; 3'd1: speed_of = 8'd63;
        3'd2: speed_of = 8'd16;  3'd3: speed_of = 8'd8;
        3'd4: speed_of = 8'd4;   3'd5: speed_of = 8'd2;
        3'd6: speed_of = 8'd1;   default: speed_of = 8'd0;
      endcase
   endfunction

   always @(posedge clk_sys) begin
      if (sw_fire) sw_want <= 1'b0;
      if (rst) begin
         sd_speed <= SD_DIV_FAST[7:0];   // start where the parameter says, not
         // where the sweep table happens to begin
         sw_i <= 0; sw_pass <= 0; sw_best <= 0;
         sw_run <= 1'b0; sw_done <= 1'b0; sw_busy <= 1'b0; sw_ck <= 0; sw_ref <= 0;
         sw_want <= 1'b0;
      end else if (!sw_run) begin
         // start once the self-test has finished and the card is known good
         if (SWEEP_ENABLE && selftest_done && selftest_ok) begin
            sw_run   <= 1'b1;
            sw_i     <= 0;
            sd_speed <= speed_of(3'd0);
            sw_ck    <= 0;
            sw_busy  <= 1'b1;
            sw_want  <= 1'b1;
         end
      end else if (!sw_done) begin
         if (sw_busy) begin
            if (sd_buf_we) sw_ck <= {sw_ck[14:0], sw_ck[15]} + sd_buf_wdata;
            if (sd_done) begin
               sw_busy <= 1'b0;
               if (sw_i == 3'd0) begin
                  sw_ref  <= sw_ck;               // the reference, at 100 kHz
                  sw_pass <= 8'd1;
                  sw_best <= 3'd0;
               end else if (!sd_err && sw_ck == sw_ref) begin
                  sw_pass <= sw_pass | (8'd1 << sw_i);
                  sw_best <= sw_i;
               end
            end
         end else if (sw_i == 3'd7) begin
            sw_done  <= 1'b1;
            sd_speed <= speed_of(sw_best);      // settle at the fastest good rate
         end else begin
            sw_i     <= sw_i + 3'd1;
            sd_speed <= speed_of(sw_i + 3'd1);
            sw_ck    <= 0;
            sw_busy  <= 1'b1;
            sw_want  <= 1'b1;
         end
      end
   end

   // ---- USB serial receive + raw SD provisioning --------------------------
   // One uart_rx feeds both the 'I' image-select parser (in g_fs below) and the
   // usb_prov raw sector read/write engine. Provisioning owns the SD directly
   // with the //e off; see src/usb_prov.v.
   wire [7:0] urx_data;
   wire       urx_stb;
   uart_rx #(.CLK_HZ(CLK_HZ)) u_urx (
                                     .clk(clk_sys), .rst(rst), .rx(usb_rx),
                                     .data(urx_data), .stb(urx_stb));

   wire        prov_rd, prov_wr, prov_active, prov_seen, prov_tx;
   wire [31:0] prov_lba;
   wire [7:0]  prov_rdata;
   // 'C' on the USB channel re-arms the bus capture (aim it at a write burst
   // after boot). One byte, no prov latch, no magic: it only resets the
   // capture ring, which is harmless.
   reg         cap_arm = 1'b0;
   always @(posedge clk_sys) cap_arm <= (urx_stb && urx_data == "C");

   usb_prov #(.CLK_HZ(CLK_HZ)) u_prov (
                                       .clk(clk_sys), .rst(rst),
                                       .rx_data(urx_data), .rx_stb(urx_stb),
                                       .engine_ready(sd_dbg == SD_S_READY), .sd_done(sd_done), .sd_err(sd_err),
                                       .prov_rd(prov_rd), .prov_wr(prov_wr), .prov_lba(prov_lba),
                                       .sd_buf_addr(sd_buf_addr), .prov_rdata(prov_rdata),
                                       .sd_buf_we(sd_buf_we), .sd_buf_wdata(sd_buf_wdata),
                                       .prov_active(prov_active), .seen_cmd(prov_seen), .tx(prov_tx));

   sd_arb u_arb (
                 .clk(clk_sys), .rst(rst),
                 .engine_ready(sd_dbg == SD_S_READY), .sd_done(sd_done), .sd_err(sd_err),
                 .r_rd(sd_rd), .r_wr(sd_wr),
                 .r_fire_rd(r_fire_rd), .r_fire_wr(r_fire_wr),
                 .r_done(r_done), .r_err(r_err),
                 .f_req(fs_req), .f_req_wr(fs_req_wr), .f_fire(fs_fire),
                 .f_done(f_done), .f_err(f_err), .own_fs(sd_own_fs));

   sd_spi #(.CLK_HZ(CLK_HZ), .DIV_INIT(SD_DIV_INIT), .DIV_FAST(SD_DIV_FAST),
            .DIV_WRITE(SD_DIV_WRITE)) u_sd (
                                            .clk(clk_sys), .rst(rst),
                                            .rd(prov_rd | r_fire_rd | (rq_fire && !rq_wr) | sw_fire | ver_fire
                                                | (fs_fire && !fs_req_wr)),
                                            .wr(prov_wr | r_fire_wr | (rq_fire && rq_wr) | (fs_fire && fs_req_wr)), .speed(sd_speed),
                                            .lba(prov_active ? prov_lba
                                                 : rq_fire  ? rq_lba
                                                 : (r_fire_rd || r_fire_wr) ? sd_lba
                                                 : fs_fire  ? fs_lba
                                                 : ver_fire ? ver_lba
                                                 : (sw_run && !sw_done) ? 32'd0 : sd_lba),
                                            .done(sd_done), .err(sd_err),
                                            .buf_addr(sd_buf_addr), .buf_wdata(sd_buf_wdata), .buf_we(sd_buf_we),
                                            // During the write test sd_spi
                                            // sources the pattern, not the
                                            // block RAM. GATED ON wtest_run,
                                            // and that matters: fstage stops at
                                            // 5 when the self-test finishes
                                            // rather than advancing past it, so
                                            // keying off fstage alone would
                                            // leave this stuck on the
                                            // constant-zero source forever.
                                            //
                                            // The filesystem sources its own
                                            // writes (persisting a selection)
                                            // from its sector buffer, not from
                                            // the block buffer. OWNER-keyed,
                                            // not request-keyed: keying on the
                                            // request would let a persist write
                                            // PENDING during a host write flip
                                            // the data source mid-sector.
                                            .buf_rdata(prov_active         ? prov_rdata
                                                       : sd_own_fs           ? fs_wr_data
                                                       : !wtest_run        ? ram_rdata
                                                       : (fstage == 3'd3)  ? wpat(sd_buf_addr)
                                                       : (fstage == 3'd5)  ? 8'h00
                                                       : ram_rdata),
                                            .sd_clk(sd_clk_i), .sd_cs_n(sd_cs_n_i), .sd_mosi(sd_mosi_i), .sd_miso(sd_miso),
                                            .ready(sd_ready), .err_code(sd_err_code), .dbg_state(sd_dbg),
                                            .dbg_cmd(sd_dbg_cmd), .dbg_retries(sd_retries), .dbg_fast(sd_fast));


   wire [9:0] rpt_addr;
   wire       rpt_active;

   // ---- USB_TX beacon (iCE40 reference arm only) ---------------------------
   // Drives a bare "L9" beacon on a second candidate UART ball, clocked off
   // the raw oscillator with a POR of its own, so it is alive whenever the
   // part is configured regardless of the PLL, /PRES, or the card. Generated
   // away, with the pin parked, on every other family.
   generate if (FAMILY != "ICE40") begin : g_nobeacon
      assign usb_tx_alt = 1'b1;          // UART idle level, so it is never a break
   end else begin : g_beacon
      reg [7:0]  bcn_ch;
      reg        bcn_stb;
      reg [2:0]  bcn_i;
      reg [24:0] bcn_t;
      wire       bcn_busy;
      reg [7:0]  bcn_por = 8'd0;
      wire       bcn_rst = ~bcn_por[7];
      always @(posedge clk) if (!bcn_por[7]) bcn_por <= bcn_por + 1'b1;

      uart_tx #(.CLK_HZ(100_000_000)) u_beacon (
                                                .clk(clk), .rst(bcn_rst), .data(bcn_ch), .stb(bcn_stb),
                                                .busy(bcn_busy), .tx(usb_tx_alt));

      always @(posedge clk) begin
         bcn_stb <= 1'b0;
         if (bcn_rst) begin bcn_i <= 0; bcn_t <= 0; end
         else begin
            bcn_t <= bcn_t + 1'b1;
            if (!bcn_busy && !bcn_stb) begin
               if (bcn_i != 0 || bcn_t == 0) begin
                  case (bcn_i)
                    3'd0: bcn_ch <= "L";
                    3'd1: bcn_ch <= "9";
                    3'd2: bcn_ch <= "\r";
                    default: bcn_ch <= "\n";
                  endcase
                  bcn_stb <= 1'b1;
                  bcn_i   <= (bcn_i == 3'd3) ? 3'd0 : bcn_i + 1'b1;
               end
            end
         end
      end
   end endgenerate

   // One block buffer, written either by the host (writes) or the SD reader.
   reg [7:0] blockram [0:1023];
   reg [7:0] ram_q;
   // Owner-gated: the persist's sector reads must not clobber the host's
   // block buffer, or the next host read could serve the residue.
   wire      sd_fill = sd_buf_we && !sd_own_fs;
   wire [9:0] wr_addr = sd_fill ? sd_buf_addr : ram_addr;
   wire [7:0] wr_data = sd_fill ? sd_buf_wdata : ram_wdata;
   wire       wr_en   = sd_fill | ram_we;
   // The reporter borrows the read port, but ONLY while the ProFile side is
   // idle. It is NOT enough that the reporter starts before any host appears:
   // it emits its status line forever. Without this gate it keeps the read
   // port once a host arrives, and the host's data phase reads the reporter's
   // dump address instead of the block. The result is a plausible-looking
   // CONSTANT rather than an obvious failure.
   //
   // The host always wins. A dump that is garbled because it was sampled mid
   // transaction is a diagnostic being wrong; a data phase that is garbled is
   // the drive being wrong.
   // While sd_spi is sourcing a CMD24 data block it owns the read port: it is
   // the only reader then, since the host cannot be mid-transaction and the
   // reporter only runs before a host appears.
   wire       sd_srcing = (sd_dbg >= 5'd28) && (sd_dbg <= 5'd29);   // S_WPREP/S_WDATA
   // ONE owner signal, not a list of special cases. The reporter may borrow the
   // read port only when nothing else needs it, and "nothing else" has to
   // include every consumer: `!tgt_busy` alone misses the block streamer,
   // whose build holds profile_target in RESET, so tgt_busy is permanently
   // low and the reporter would own the port forever.
   wire       buf_port_busy = tgt_busy | str_active;

   wire [9:0] rd_addr_mux = sd_srcing                   ? sd_buf_addr
              : (rpt_active && !buf_port_busy) ? rpt_addr
              : ram_addr;
   always @(posedge clk_sys) begin
      if (wr_en) blockram[wr_addr] <= wr_data;
      ram_q <= blockram[rd_addr_mux];
   end
   assign ram_rdata = ram_q;

   // ---- transaction telemetry ----------------------------------------------
   // These events have to be counted, not discarded. When the //e fails to
   // boot, the LEDs show live pin state and the status line is about the card.
   // Without these counters NOTHING says what the host actually asked for or
   // how far the transaction got. With them, a failed boot can be diagnosed
   // instead of guessed at.
   //
   // Saturating counters, so a runaway retry loop reads as $FF rather than
   // wrapping to a small number and looking normal.
   reg [7:0]  n_cmd, n_read_ok, n_write_ok, n_nak, n_abandon, n_badop, n_oor;
   reg [7:0]  last_op;
   reg [23:0] last_blk;
   // The block that was REFUSED, separate from last_blk: a retry storm keeps
   // last_blk moving, but the first refused block is the diagnosis.
   reg [23:0] last_oor_blk;

   always @(posedge clk_sys) begin
      if (rst) begin
         n_cmd <= 0; n_read_ok <= 0; n_write_ok <= 0; n_nak <= 0;
         n_abandon <= 0; n_badop <= 0; n_oor <= 0;
         last_op <= 8'h00; last_blk <= 24'h000000; last_oor_blk <= 24'h000000;
      end else begin
         if (ev_cmd) begin
            if (n_cmd != 8'hFF) n_cmd <= n_cmd + 8'd1;
            last_op  <= blk_op;
            last_blk <= blk_addr;
         end
         if (ev_read_ok  && n_read_ok  != 8'hFF) n_read_ok  <= n_read_ok  + 8'd1;
         if (ev_write_ok && n_write_ok != 8'hFF) n_write_ok <= n_write_ok + 8'd1;
         if (ev_nak      && n_nak      != 8'hFF) n_nak      <= n_nak      + 8'd1;
         if (ev_abandon  && n_abandon  != 8'hFF) n_abandon  <= n_abandon  + 8'd1;
         if (ev_badop    && n_badop    != 8'hFF) n_badop    <= n_badop    + 8'd1;
         if ((out_of_range || oor_read) && n_oor != 8'hFF) n_oor <= n_oor + 8'd1;
         if (out_of_range || oor_read) last_oor_blk <= blk_addr;
      end
   end

   // Eight 32-bit words, the same shape as the FAT line so the reporter can
   // reuse its shift register.
   wire [255:0] dbg_txn = {
                           {tr_nstb, dbg_cmd_idx, 1'b0, last_blk},          // 0 strobe COUNT + last block
                           {last_op, n_cmd, n_read_ok, n_write_ok},         // 1 what, and how many
                           {n_nak, n_abandon, n_badop, n_oor},              // 2 how it went wrong
                           // 3: the geometry as served.
                           {claim_blocks, served_blocks},
                           // 4: the last REFUSED block + the window state. The
                           // out-of-range counter saturates, so on its own it
                           // cannot say WHICH block was refused.
                           {6'd0, window_live, sel_armed, last_oor_blk},
                           // 5: flags. dbg_f0_lead is byte 0 of the command
                           // frame as seen at the strobe's LEADING edge; with
                           // tr0 (the trailing-edge value) it says where the
                           // byte is:
                           //   f0_lead=$00, tr0=$FF -> data is valid early and gone by the trailing
                           //                           edge; the delayed sample reads it.
                           //   f0_lead=$FF, tr0=$FF -> the bus is never driven for byte 0 at all,
                           //                           and no sampling edge can help.
                           {rise_vals[39:32], pres_lvl, geo_decoupled, status2,
                            n_pres, no_media, tr_prw, write_protect, tgt_busy},
                           // 6: the filesystem resolver, so a card can be
                           // diagnosed without a host
                           {fs_ready, fs_ok, fs_need_select, sel_want, fs_err, fs_img_count,
                            dbg_badframe[15:0]},       // 6: fs flags + IMG COUNT + BAD frame 4..5
                           // 7: the HOST-driven lines. fs_nclusters is on the
                           // FAT line already.
                           {n_pcmd, n_pstrb, n_prw, 5'h00, pcmd_lvl, pstrb_lvl, prw_lvl}};

   // ---- boot chronometer ----------------------------------------------------
   // Milliseconds from reset release (which on a flash boot is configuration
   // done) to SD bring-up complete and to the resolver's first ready. 12-bit
   // fields saturate at 4.095 s; $FFF also means "never happened". Declared
   // BEFORE the reporter that consumes it, for exactly the reason the
   // reporter's own comment below records.
   localparam integer MS_TICKS = CLK_HZ / 1000;
   reg [16:0]         ms_div = 17'd0;
   reg [11:0]         ms_now = 12'd0;
   reg [11:0]         t_sdrdy = 12'hFFF, t_fsrdy = 12'hFFF;
   always @(posedge clk_sys) begin
      if (rst) begin
         ms_div <= 17'd0; ms_now <= 12'd0;
         t_sdrdy <= 12'hFFF; t_fsrdy <= 12'hFFF;
      end else begin
         if (ms_div == MS_TICKS[16:0] - 17'd1) begin
            ms_div <= 17'd0;
            if (ms_now != 12'hFFF) ms_now <= ms_now + 12'd1;
         end else ms_div <= ms_div + 17'd1;
         if (sd_ready  && t_sdrdy == 12'hFFF) t_sdrdy <= ms_now;
         if (fs_ready  && t_fsrdy == 12'hFFF) t_fsrdy <= ms_now;
      end
   end

   // ---- serial report ------------------------------------------------------
   // The LEDs can say "the signature matched". Only this can say WHAT is on the
   // card, which is the difference between a checksum and a read. Instantiated
   // AFTER ram_q exists: yosys accepts a forward reference by inventing an
   // implicit wire; iverilog refuses it.
   reporter #(.CLK_HZ(CLK_HZ)) u_rpt (
                                      .clk(clk_sys), .rst(rst),
                                      .go(selftest_done), .sd_ready(sd_ready), .sd_ok(selftest_ok),
                                      .err_code(sd_err_code), .dbg_state(sd_dbg),
                                      .dbg_cmd(sd_dbg_cmd),
                                      .build_id(BUILD_ID),
                                      .fs_cfg(fs_dbg_cfg), .fs_plen(fs_dbg_plen),
                                      .fs_c1(fs_dbg_c1), .fs_c2(fs_dbg_c2),
                                      // Word 5 carries the boot chronometer:
                                      // [27:16] ms from reset release to
                                      // SD-init done, [11:0] ms to resolver
                                      // ready. $FFF = not yet. This is what
                                      // measures the instant-on budget against
                                      // the //e's power-to-first-command time
                                      // rather than estimating it.
                                      .fs_fatstart({4'd0, t_sdrdy, 4'd0, t_fsrdy}), .fs_nclus(fs_nclusters),
                                      .dbg_fat({part_lba, fat_start, data_start, bpb_fatsz,
                                                free_count, first_free, target_lba, wcmp}),
                                      .dbg_fst({wtest_ok, wtest_run, sd_fast, 2'd0, fstage}),
                                      .dbg_txn(dbg_txn),
                                      .rd_addr(rpt_addr), .rd_data(ram_q), .active(rpt_active),
                                      .tx(rpt_tx));

   // ---- filesystem ----------------------------------------------------------
   // Runs once the self-test has checked the card, and turns "block N" into
   // "the sector of the image file that holds block N". Generated away when
   // off, in which case the router maps a block straight to a sector and the
   // card has to be dedicated to one raw image.
   generate
      if (FS_ENABLE) begin : g_fs
         // ---- bench command channel (USB serial RX) ----
         // Two-byte command: 'I' then an image index. Selects that image
         // exactly as a host menu SELECT would (fat32 reselect + persist), so
         // a remote bench session can cycle images with no hands at the //e.
         // Any byte other than 'I' resets the parser; an out-of-range index is
         // ignored. urx_data/urx_stb and u_urx live at top level (shared with
         // usb_prov). This block only parses the 'I' select command.
         reg       urx_have_i = 1'b0;
         reg       usb_sel_stb = 1'b0;
         reg [7:0] usb_sel_idx = 8'd0;
         always @(posedge clk_sys) begin
            usb_sel_stb <= 1'b0;
            if (rst) urx_have_i <= 1'b0;
            else if (urx_stb && !prov_seen) begin
               if (!urx_have_i) begin
                  if (urx_data == "I") urx_have_i <= 1'b1;
               end else begin
                  urx_have_i <= 1'b0;
                  if (urx_data < fs_img_count) begin
                     usb_sel_idx <= urx_data;
                     usb_sel_stb <= 1'b1;
                  end
               end
            end
         end

         // resel_idx must stay STABLE for the whole rescan: fat32 compares it
         // at scan-judgment time (fat32.v S_DIR paths), many milliseconds after
         // the pulse. Muxing the index on the 1-cycle sel_stb pulse makes every
         // MENU selection resolve the USB channel's stale index instead, so
         // whatever the user picks boots the last USB pick. Register the index
         // at the pulse; delay the pulse one cycle so the S_DONE same-index
         // short-circuit samples the updated register. Menu wins a same-cycle
         // tie.
         reg       resel_p = 1'b0;
         reg [7:0] resel_idx_r = 8'd0;
         reg       resel_nopersist = 1'b0;
         always @(posedge clk_sys) begin
            resel_p <= sel_stb || usb_sel_stb;
            if (sel_stb) begin
               resel_idx_r <= {2'b00, sel_idx};
               // A SELECT from a knock session is ephemeral: serve it, boot it,
               // but leave the power-on default alone. Boot-menu and USB
               // selects persist. knock_menu is still high at the SELECT (it
               // clears on sel_stb one arbiter cycle later), so it is the
               // discriminator.
               resel_nopersist <= knock_menu;
            end else if (usb_sel_stb) begin
               resel_idx_r <= usb_sel_idx;
               resel_nopersist <= 1'b0;
            end
         end

         // A quarter second with no host block traffic = quiet. A booting //e
         // reads continuously with millisecond gaps, so this cleanly defers the
         // persist past every read of the boot. At the menu, the user's
         // reaction time provides the quiet.
         reg [23:0] quiet_cnt = 24'd0;
         wire       host_quiet = (quiet_cnt >= (CLK_HZ / 4));   // 250 ms, any CLK_HZ
         always @(posedge clk_sys) begin
            if (blk_rd || blk_wr)               quiet_cnt <= 24'd0;
            else if (!host_quiet)               quiet_cnt <= quiet_cnt + 24'd1;
         end

         fat32 #(.MAX_CLUSTERS(FS_MAX_CLUSTERS), .DIR_NAME(FS_DIR),
                 .AUTOPICK(FS_AUTOPICK), .RECURSE(FS_RECURSE)) u_fs (
                                                                     .clk(clk_sys), .rst(rst), .start(selftest_done & selftest_ok),
                                                                     .reselect(resel_p), .resel_idx(resel_idx_r),
                                                                     .host_quiet(host_quiet), .nopersist(resel_nopersist),
                                                                     .req(fs_req), .fire(fs_fire),
                                                                     .sd_done(f_done), .sd_err(f_err),
                                                                     .sd_buf_we(sd_buf_we && sd_own_fs), .sd_buf_wdata(sd_buf_wdata), .lba(fs_lba),
                                                                     .sd_buf_addr(sd_buf_addr), .wr_data(fs_wr_data), .req_wr(fs_req_wr),
                                                                     .ready(fs_ready), .ok(fs_ok), .err(fs_err),
                                                                     .image_blocks(fs_image_blocks), .data_start(fs_data_start),
                                                                     .spc_log(fs_spc_log), .nclusters(fs_nclusters),
                                                                     .need_select(fs_need_select),
                                                                     .img_count(fs_img_count), .img_index(fs_img_index),
                                                                     .dbg_c1(fs_dbg_c1), .dbg_c2(fs_dbg_c2), .dbg_fat_start(fs_dbg_fat_start),
                                                                     .dbg_persist(fs_dbg_persist),
                                                                     .dbg_cfg(fs_dbg_cfg), .dbg_plen(fs_dbg_plen),
                                                                     .cat_raddr(sel_cat_addr), .cat_rdata(fs_cat_data),
                                                                     .map_idx(fs_map_idx), .map_cluster(fs_map_cluster),
                                                                     .dbg_state(fs_state));
      end else begin : g_no_fs
         assign fs_req = 1'b0;         assign fs_lba = 32'd0;
         assign fs_ready = 1'b1;       assign fs_ok = 1'b0;
         assign fs_err = 4'd0;         assign fs_image_blocks = 32'd0;
         assign fs_data_start = 32'd0; assign fs_spc_log = 4'd0;
         assign fs_nclusters = 32'd0;  assign fs_map_cluster = 32'd0;
         assign fs_img_count = 8'd0;   assign fs_img_index = 8'd0;
         assign fs_cat_data = 8'h00; assign fs_wr_data = 8'h00;
         assign fs_req_wr = 1'b0;    assign fs_dbg_persist = 32'd0;
         assign fs_need_select = 1'b0;
         assign fs_state = 5'd0;
      end
   endgenerate

   // ---- the boot-time selector ---------------------------------------------
   // Block 0 is intercepted ONLY as part of a boot, which is why the previous
   // block matters: the card ROM reads the spare table first and block 0
   // second. A running system reading block 0 for its own reasons must get the
   // image's real boot block, not a menu.
   always @(posedge clk_sys) begin
      if (rst) begin
         prev_blk  <= 24'hFFFFFF;
         sel_armed <= 1'b1;
      end else begin
         if (blk_rd || blk_wr) prev_blk <= blk_addr;
         // Every boot re-arms. "Once per power-up" was inherited from the
         // Teensy, which is powered BY the //e and so re-arms on every boot.
         // A board powered independently of the //e does not. Arming once per
         // power-up would drop sel_armed at the first auto-select, and every
         // later //e boot would serve the image directly. The menu blob never
         // runs, and that blob is the only thing that ever reads open-apple.
         // The spare-table read ($FFFFFF) is the boot signature: it precedes
         // every boot attempt and only the $C500 boot entry issues it, never
         // the running driver. So re-arming on it cannot fire mid-session. A
         // selection still stands the arm down, so the post-select block-0 read
         // chains into the image instead of looping the menu.
         if (blk_rd && blk_addr == 24'hFFFFFF) sel_armed <= 1'b1;
         else if (sel_stb)                     sel_armed <= 1'b0;
      end
   end

   // With fewer than two images and a valid selection there is nothing to
   // choose, and the drive behaves exactly as if the selector did not exist.
   // ALWAYS, and that is a requirement rather than a simplification.
   //
   // The drive cannot see the keyboard. Open-apple is checked by the 6502 blob
   // this serves AS block 0, so the only way the key can ever bring the menu up
   // is if that blob always runs. From selector/SELECTOR.md: "Holding
   // open-apple at boot brings the menu up; without it the selector immediately
   // re-selects the remembered image and chains", which costs a few ms and is
   // invisible. Deciding here that there is "nothing to choose" takes the key
   // away from the user entirely.
   //
   // Do NOT gate this on the image count. Requiring one image makes open-apple
   // do nothing on a single-image card, and requiring two does nothing on an
   // empty one, which is exactly when the user most needs a menu to say so.
   //
   // The gate that DOES matter is `armed` below: the blob is served only as
   // part of a boot (the get-status block read that always precedes it) and
   // only until a choice is made, so this cannot loop.
   assign sel_want = 1'b1;

   generate
      if (SELECTOR_ENABLE) begin : g_sel
         selector u_sel (
                         .clk(clk_sys),
                         .blk_addr(blk_addr), .blk_rd(blk_rd), .buf_addr(buf_addr),
                         .img_count(fs_img_count), .img_current(fs_img_index),
                         .force_menu(fs_need_select || knock_menu),
                         .armed(sel_armed && sel_want && (prev_blk == 24'hFFFFFF)),
                         .window_live(window_live),
                         .cat_addr(sel_cat_addr), .cat_data(fs_cat_data),
                         .sel_idx(sel_idx), .sel_stb(sel_stb),
                         .hit(sel_hit), .boot_hit(sel_boot_hit), .rdata(sel_data));
      end else begin : g_no_sel
         assign sel_hit = 1'b0;  assign sel_data = 8'h00; assign sel_boot_hit = 1'b0;
         assign sel_cat_addr = 12'd0; assign sel_idx = 6'd0; assign sel_stb = 1'b0;
      end
   endgenerate

   // ---- GEOMETRY OVERLOADING: when is the magic window real? ---------------
   // The card ROM maps ProDOS $FFxx to wire $FFFFxx unconditionally, so on a
   // 100%-full 65535-block volume (Total Replay) the top 256 blocks arrive as
   // window addresses. The window is only MEANT during the boot menu, so:
   // open it when the selector serves its boot blob (only a boot produces
   // status-then-block-0, so this cannot happen mid-OS), close it on SELECT.
   // Closed, block_router serves $FFFFxx as image block $00FFxx. $FFFFFF
   // stays the spare table always (a 65535-block volume's top index is $FFFE,
   // so no OS ever asks for it as data). sel_armed is NOT the right signal:
   // a runtime STATUS call reads $FFFFFF and re-arms it by design.
   // ---- the KNOCK: runtime menu entry --------------------------------------
   // Four completed reads of specific window blocks in exact order, issued
   // through the STANDARD card ROM driver by a client program: ProDOS blocks
   // $FF41 $FF32 $FF45 $FF44 ("A2E" plus MouseText's check mark). Opens the
   // window exactly as a boot does and forces the menu, so the blob waits for
   // the user instead of auto-reselecting the current image. The card ROM's
   // designed retry cascade re-reads a block several times per call, so a
   // repeat of the just-accepted block holds position rather than resetting.
   // Any other read resets the sequence: the client issues its four calls
   // back to back, and anything interleaving shows it was not a knock. The
   // detector is passive: the four reads themselves serve normally.
   localparam [23:0] KN0 = 24'hFFFF41, KN1 = 24'hFFFF32,
                     KN2 = 24'hFFFF45, KN3 = 24'hFFFF44;
   reg [1:0]         kidx = 2'd0;
   reg [23:0]        kprev = 24'd0;
   wire [23:0]       kwant = (kidx == 2'd0) ? KN0 : (kidx == 2'd1) ? KN1
                     : (kidx == 2'd2) ? KN2 : KN3;
   always @(posedge clk_sys) begin
      knock_hit <= 1'b0;
      if (rst) begin
         kidx <= 2'd0; kprev <= 24'd0;
      end else if (blk_rd && !window_live) begin
         if (blk_addr == kprev) begin
            // the cascade retrying the accepted block: hold position
         end else if (blk_addr == kwant) begin
            kprev <= blk_addr;
            if (kidx == 2'd3) begin knock_hit <= 1'b1; kidx <= 2'd0; end
            else kidx <= kidx + 2'd1;
         end else begin
            kprev <= blk_addr;
            kidx  <= (blk_addr == KN0) ? 2'd1 : 2'd0;
         end
      end
   end

   always @(posedge clk_sys) begin
      if (rst)                          window_live <= 1'b0;
      else if (sel_boot_hit && blk_rd)  window_live <= 1'b1;
      else if (knock_hit)               window_live <= 1'b1;
      else if (sel_stb)                 window_live <= 1'b0;
      // A fresh boot cancels a stale knock: an abandoned knock must not force
      // the NEXT boot into the menu.
      if (rst)                          knock_menu <= 1'b0;
      else if (knock_hit)               knock_menu <= 1'b1;
      else if (sel_stb)                 knock_menu <= 1'b0;
      else if (sel_boot_hit && blk_rd)  knock_menu <= 1'b0;
   end

   // ---- whole-volume verification ------------------------------------------
   // Reads a range of the card and streams a CRC per checkpoint. It takes the
   // SD engine for the duration, so it only ever runs once the self-test has
   // finished and only when built with VERIFY_ENABLE. GENERATED AWAY when it is
   // off, not merely held idle: instantiated with enable tied low, its UART
   // and counters do not fold. A build that serves a host should not be
   // paying for a bench tool.
   generate
      if (VERIFY_ENABLE) begin : g_verify
         verifier #(.CLK_HZ(CLK_HZ), .START(VERIFY_START), .COUNT(VERIFY_COUNT),
                    .STRIDE(VERIFY_STRIDE)) u_verify (
                                                      .clk(clk_sys), .rst(rst),
                                                      .enable(1'b1), .go(selftest_done & selftest_ok),
                                                      .req(ver_req), .fire(ver_fire),
                                                      .sd_done(sd_done), .sd_err(sd_err), .sd_err_code(sd_err_code),
                                                      .sd_retries(sd_retries),
                                                      .sd_buf_we(sd_buf_we), .sd_buf_wdata(sd_buf_wdata),
                                                      .lba(ver_lba), .active(ver_active), .done(ver_done), .tx(ver_tx));
      end else begin : g_noverify
         assign ver_req    = 1'b0;
         assign ver_lba    = 32'd0;
         assign ver_active = 1'b0;
         assign ver_done   = 1'b1;
         assign ver_tx     = 1'b1;      // an idle UART line sits high
      end
   endgenerate

   // One UART, two talkers. The verifier wins while it is running: its output
   // is being parsed by a script, and a status line spliced into the middle of
   // it would be read as a corrupt checksum rather than as what it is. The
   // capture dump outranks the reporter: it happens once, after the boot has
   // already failed, and carries the waveform the counters could not.
   reg [21:0] probe_cnt = 22'd0;
   always @(posedge clk_sys) probe_cnt <= probe_cnt + 22'd1;
   assign usb_tx = CLK_PROBE    ? probe_cnt[21]
                   : prov_active  ? prov_tx
                   : cap_dumping  ? cap_tx
                   : ver_active   ? ver_tx
                   : rpt_tx;

   bus_capture #(.CLK_HZ(CLK_HZ)) u_cap (
                                         .clk(clk_sys), .rst(rst),
                                         .bus_in(bus_in), .pstrb_n(pstrb_n), .prw(prw), .pcmd_n(pcmd_n),
                                         .drive_en(drive_en), .host_stb(host_stb), .arm(cap_arm),
                                         .dumping(cap_dumping), .tx(cap_tx));


   // ---- bring-up display ---------------------------------------------------
   // LIVE LINE STATE, not protocol events. On a board that has never been
   // powered the first questions are electrical: does /PRES idle asserted on
   // this host, and which way round are the SD socket's CD and WP contacts.
   // Both are answered by looking at the board rather than by instrumenting
   // it.
   //
   // The synchronizers are not optional. pcmd_n and pres_n are asynchronous to
   // this clock. A metastable flop driving an LED is harmless, but there is no
   // reason to build one.
   reg [1:0] cmd_s, res_s, cd_s, wp_s, ph0_s, aux7_s;
   reg [1:0] stb_s, prw_s;
   always @(posedge clk_sys) begin
      cmd_s  <= {cmd_s[0],  ~pcmd_n};
      stb_s  <= {stb_s[0],  ~pstrb_n};
      prw_s  <= {prw_s[0],   prw};
      res_s  <= {res_s[0],  ~pres_n};
      cd_s   <= {cd_s[0],   ~sd_cd_n};
      wp_s   <= {wp_s[0],   ~sd_wp_n};
      ph0_s  <= {ph0_s[0],   ph0};
      aux7_s <= {aux7_s[0],  aux7};
   end

   // ---- /PRES ---------------------------------------------------------------
   // HONOR_PRES is 0 because the //e's reset line is documented as idling
   // asserted on some setups, which would pin the design in reset forever. But
   // the card ALSO re-triggers its drive-reset one-shot on the ROM's retry path
   // (259 Q4, $C50C/$C508; see docs/APPLE2_ProFile_card_map.md section 8 step
   // 4). A real ProFile resets when that fires. If it is firing here and we
   // ignore it, the ROM believes it is talking to a freshly reset drive and we
   // are not one.
   //
   // So: count the assertions and report the idle level. Both questions are
   // answered by looking, and neither can be answered by reasoning.
   always @(posedge clk_sys) begin
      if (rst) n_pres <= 7'd0;
      else if (res_s == 2'b01 && n_pres != 7'h7F) n_pres <= n_pres + 7'd1;
   end
   assign pres_lvl = res_s[1];      // 1 = /PRES ASSERTED right now
   assign pcmd_lvl  = cmd_s[1];     // 1 = /PCMD asserted
   assign pstrb_lvl = stb_s[1];     // 1 = /PSTRB asserted (low on the wire)
   assign prw_lvl   = prw_s[1];     // 1 = PR/-W high, i.e. the host is READING
   always @(posedge clk_sys) begin
      if (rst) begin
         n_pcmd <= 8'd0; n_pstrb <= 8'd0; n_prw <= 8'd0;
      end else begin
         if (cmd_s == 2'b01 && n_pcmd  != 8'hFF) n_pcmd  <= n_pcmd  + 8'd1;
         if (stb_s == 2'b01 && n_pstrb != 8'hFF) n_pstrb <= n_pstrb + 8'd1;
         if (prw_s == 2'b01 && n_prw   != 8'hFF) n_prw   <= n_prw   + 8'd1;
      end
   end

   // Two display pages, selected automatically so nothing has to be configured.
   // Until a host asserts /PCMD for the first time the LEDs show the SD card;
   // after that they show the bus, permanently.
   reg host_seen;
   always @(posedge clk_sys) if (rst) host_seen <= 1'b0; else if (cmd_s[1]) host_seen <= 1'b1;

   // ~1.5 Hz heartbeat: shows the part is configured and clocked, which is the
   // difference between "stuck in state 0" and "no bitstream".
   reg [25:0] hb = 0;
   always @(posedge clk_sys) if (rst) hb <= 0; else hb <= hb + 1'b1;

   wire [7:0] led_bus = {aux7_s[1],   // 7  DB-25 7,  SmartPort /ENABLE1
                         ph0_s[1],    // 6  DB-25 1,  SmartPort WRDATA
                         wp_s[1],     // 5  SD write protect, as wired
                         cd_s[1],     // 4  SD card detect, as wired
                         res_s[1],    // 3  /PRES asserted by the host
                         cmd_s[1],    // 2  /PCMD asserted by the host
                         drive_en,    // 1  we own the data bus
                         ~pbsy_n};    // 0  BSY asserted

   wire [7:0] led_sd  = {sd_ready,                                  // 7 card is up
                         selftest_done,                             // 6 test finished
                         selftest_done ? selftest_ok : hb[25],      // 5 pass, or alive
                         sd_ready ? {1'b0, sd_err_code}             // 4:0 why it stopped
                         : sd_dbg};

   // While the pin walk runs, the LEDs ARE the walk: one lit, stepping along in
   // time with the phase, so a meter can be read against the board itself
   // rather than against a scrolling terminal. led7 lights on the two inverted
   // phases (CHK and DIR idle low, so those drive their pin HIGH; look for 3.3
   // V, not 0).
   wire [7:0] led_walk = (8'd1 << pw_ph) | ((pw_ph >= 3'd5) ? 8'h80 : 8'h00);

   assign led = diag_en   ? led_walk
                : host_seen ? led_bus
                : led_sd;

   // Protocol event pulses that nothing else consumes are kept alive here so
   // the target's outputs are not optimized away.
   /* verilator lint_off UNUSED */
   wire _unused_events = 1'b0;
   /* verilator lint_on UNUSED */

endmodule

`default_nettype wire
