// profile_bus: the pin-level half of the ProFile target.
//
// Owns the data byte, its tri-state direction, the two 74LVC245
// controls, and the parity line. Everything above this module works
// with plain registered bytes and never touches a pad.
//
// Parity: the DRIVE owns /PPARITY, and it owns it for HOST-driven
// bytes as well as its own (PROTOCOL.md section 9.4). The card's
// parity latch samples the ack byte, so the line has to be correct
// during the ack window and any host-write drain, not only while we
// are sourcing data. Odd parity: the line is driven so that the nine
// bits together have odd parity.
//
// THE DATA BUS PADS ARE EXPLICIT. The eight PD bits go through
// instantiated vendor primitives (TRELLIS_IO on the ECP5) rather than
// a behavioral `pd = en ? d : 8'bz`. yosys can infer $_TBUF_ cells
// from the behavioral form and nextpnr can fold them into the pad
// output enables, but the failure mode if that inference ever goes
// wrong is two drivers fighting on a 5V bus with the //e's interface
// card on the other end. That is the only fault on this board that
// can destroy hardware, so it does not get to depend on inference.
// sim/trellis_io.v models the ECP5 primitive so the simulations
// exercise this same code and not a second, differently-written
// version of it. Only FAMILY="ECP5" is modeled for iverilog; the
// other arms are unsupported reference code and cannot be simulated.

`default_nettype none

module profile_bus #(
                     // Which vendor primitive the data pads use
                     parameter FAMILY = "ECP5",     // see the header comment
                     // How a host byte is captured. It is NOT taken
                     // at either edge of /PSTRB. The //e's card
                     // presents byte 0 of a command frame late: the
                     // 374's output enable (Q3, $C503) and the write
                     // that strobes it land together, so at the
                     // falling edge the bus is still floating and
                     // reads $FF through RPA. Sampling there would
                     // reject half of every boot's commands as bad
                     // opcodes. Sampling at the rising edge is no
                     // better, because write strobes on this card are
                     // LOW for 5-15 us with the byte valid and HIGH
                     // for about 0.5 us with the bus decaying to $FF,
                     // so a sample after the rise reads the NEXT
                     // window.
                     //
                     // So the code below qualifies the strobe at its
                     // FALL (ARM_DELAY, which is where PR/-W
                     // separates a real write from a phantom),
                     // then takes the byte LATCH_DELAY clocks INTO
                     // the low window, 4 us in, where it has been
                     // valid for microseconds and stays valid for
                     // microseconds more. A strobe shorter than the
                     // delay is caught by the rise guard instead.
                     // sim/tb_top_iie.v's LATE_NS parameter
                     // reproduces the late byte, so the simulation
                     // catches a regression here.
                     //
                     // How long after /PSTRB falls to decide whether
                     // this strobe is a real host write, in clocks
                     // at 50 MHz.
                     //
                     // 1 is the SMALLEST USABLE VALUE, and the right
                     // one. The decision only works at the edge. A
                     // real byte has PR/-W already low when the
                     // strobe falls. A phantom's PR/-W drops
                     // mid-strobe, so sampling later accepts the
                     // phantom, and its undriven bus reads as a $FF
                     // opcode. 0 does not arm at all (the countdown
                     // branch never runs).
                     parameter integer ARM_DELAY = 1,
                     // Clocks after the strobe's FALLING edge at
                     // which the byte is sampled: 4 us, which sits
                     // in the 5-15 us valid window with margin on
                     // both sides. 200 clocks at the 50 MHz this
                     // design runs at; top.v derives it from CLK_HZ.
                     // 0 samples at the fall itself, which reads $FF
                     // for byte 0 on this card.
                     parameter integer LATCH_DELAY = 200
                     ) (
                        input  wire        clk,
                        input  wire        rst,

                        // --- pads, through the 74LVC245 pair ---
                        inout  wire [7:0]  pd,          // PD7..PD0, bit 7 = DB-25 pin 13
                        output wire        xcvr_dir,    // '245 direction: 1 = FPGA drives the bus
                        output wire        xcvr_oe_n,   // '245 /OE, active low
                        output wire        pparity,     // odd parity for whatever is on the bus
                        input  wire        pstrb_n,     // host strobe, active low
                        input  wire        pcmd_n,      // host command, active low
                        input  wire        prw,         // 1 = host reads (drive drives), 0 = host writes

                        // --- upward interface ---
                        input  wire        drive_en,    // 1 = we own the bus this phase
                        input  wire [7:0]  drive_data,  // byte to present while drive_en
                        output reg  [7:0]  host_data,   // last byte the host strobed at us
                        // The same byte sampled at the strobe's
                        // LEADING edge. Diagnostic only: if these two
                        // differ, the host put the byte on the bus
                        // part way through its own strobe, which is
                        // the thing that can't be seen from a
                        // counter.
                        output reg  [7:0]  host_data_lead,
                        // Synchronized /PSTRB level, 1 = idle. The
                        // target needs this to refuse to start a
                        // command frame on a strobe that is already
                        // in progress.
                        output wire        stb_idle,
                        // The bus at the RISING edge of the first
                        // four strobes of a command phase, taken
                        // while trace_arm is high, and a count of
                        // every strobe seen in that phase. Held high
                        // for the WHOLE command phase, not one byte,
                        // so a strobe arriving before byte 0 is
                        // counted rather than mistaken for it. That
                        // is what tells "byte 0 arrived undriven"
                        // from "an extra strobe arrived first". (The
                        // Teensy build is sequential and is not
                        // listening between the ack and the start of
                        // its receive loop, so a strobe in that
                        // window costs it nothing. This FSM is always
                        // on and sees it.)
                        input  wire        trace_arm,
                        output reg  [7:0]  tr0, tr1, tr2, tr3,
                        // PR/-W as it stood at each of those four
                        // strobes, bit N for strobe N.
                        output reg  [3:0]  tr_prw,
                        // The bus re-read 8, 32 and 128 clocks AFTER
                        // strobe 0's rising edge, for sizing
                        // LATCH_DELAY on a card whose data arrives
                        // late: the first of these that is not $FF
                        // brackets the arrival time.
                        output reg  [7:0]  tr_d8, tr_d32, tr_d128,
                        // bus_in at the QUALIFYING RISE of each
                        // latched host byte, newest byte in the low 8
                        // bits. Comparing this against the delayed
                        // sample says whether a disagreement is in
                        // the bus or in the model of it.
                        output reg  [39:0] rise_vals,
                        output reg  [3:0]  tr_nstb,   // strobes seen this command phase, saturating
                        output reg         host_stb,    // one clk pulse per accepted host byte
                        output reg         read_stb,    // one clk pulse per byte the host took
                        output reg  [7:0]  bus_in,      // synchronized view of the pad, strobe or no strobe
                        output wire        host_read    // synchronized PR/-W: 1 = host reading, we may drive
                        );

   // ---- strobe conditioning ----
   // The //e's strobe low window is 355-660 ns. At 50 MHz that is
   // 17-33 clocks, so two flops of synchronizer plus edge detect is
   // ample and no filtering is needed. What DOES need care is the
   // phantom strobe: a strobe that falls while PR/-W is still high,
   // which is the write-side skew the Teensy build also ignores.
   reg       stb_armed;   // this strobe began as a real host write

   reg [3:0] arm_cnt;
   reg [2:0] stb_sync;
   reg [1:0] rw_sync;
   always @(posedge clk) begin
      // Same trap as the /PCMD debouncer: /PSTRB is active low, so an
      // unreset shift register comes up all-zero and the first real
      // clock manufactures a rising edge that never happened on the
      // wire. Reset to the idle level.
      if (rst) begin
         stb_sync <= 3'b111;
         rw_sync  <= 2'b11;
      end else begin
         stb_sync <= {stb_sync[1:0], pstrb_n};
         rw_sync  <= {rw_sync[0],   prw};
      end
   end
   assign stb_idle = stb_sync[1];
   wire stb_fall = (stb_sync[2:1] == 2'b10);
   wire stb_rise = (stb_sync[2:1] == 2'b01);
   wire host_reading = rw_sync[1];
   assign host_read = rw_sync[1];

   // One pad primitive per data bit, with no pull-up: the '245
   // drives this net from the other side and a pad pull-up would
   // fight it.
   //
   // Read the pad through pd_in, NEVER by reading pd
   // directly. Reading the inout works in simulation and is the kind
   // of thing that survives to the bitstream as a second, accidental
   // input path.
   wire [7:0] pd_in;
   genvar     gi;
   generate
      for (gi = 0; gi < 8; gi = gi + 1) begin : g_pd
         if (FAMILY == "XC7") begin : g_xc7
            // Unsupported reference arm. synth_xilinx infers an
            // IOBUF from the behavioral tristate and gets the enable
            // polarity right, so here the inference IS the vendor
            // path.
            assign pd[gi]  = drive_en ? drive_data[gi] : 1'bz;
            assign pd_in[gi] = pd[gi];
         end else if (FAMILY == "ECP5") begin : g_ecp5
            // TRELLIS_IO is the ECP5 flavor of the same thing. T is
            // the TRISTATE, so it is the INVERSE of drive_en: T high
            // is high-Z. Getting that polarity backward is the only
            // mistake here that turns the pads on whenever the design
            // meant to release them, which is the exact
            // hardware-destroying case the explicit primitive exists
            // to prevent.
            TRELLIS_IO #(.DIR("BIDIR")) u_pad (
                                               .B(pd[gi]),
                                               .I(drive_data[gi]),
                                               .T(~drive_en),
                                               .O(pd_in[gi]));
         end else begin : g_ice40
            SB_IO #(.PIN_TYPE(6'b1010_01), .PULLUP(1'b0)) u_pad (
                                                                 .PACKAGE_PIN(pd[gi]),
                                                                 .OUTPUT_ENABLE(drive_en),
                                                                 .D_OUT_0(drive_data[gi]),
                                                                 .D_OUT_1(1'b0),
                                                                 .D_IN_0(pd_in[gi]),
                                                                 .D_IN_1(),
                                                                 .CLOCK_ENABLE(1'b0),
                                                                 .INPUT_CLK(1'b0),
                                                                 .OUTPUT_CLK(1'b0),
                                                                 .LATCH_INPUT_VALUE(1'b0));
         end
      end
   endgenerate


   // A free-running synchronized copy of the pad. The command frame
   // and the write data phase arrive with strobes, but THE ACK DOES
   // NOT: the host parks $55 on the bus and raises /PCMD, and a real
   // drive just reads its port (the Z8 does `ld R0,P1` in
   // SetResponse). Nothing samples that with an edge, so the target
   // needs a continuous view as well as the strobed one.
   reg [7:0] bus_meta;
   always @(posedge clk) begin
      bus_meta <= pd_in;
      bus_in   <= bus_meta;
   end

   // ---- direction ----
   // The '245 direction follows drive_en, NOT PR/-W directly: PR/-W
   // carries sub-microsecond skew on every read strobe, and following
   // it naively drops byte 0. The FSM above decides when we own the
   // bus. The pad layer just obeys.
   assign xcvr_dir  = drive_en;
   assign xcvr_oe_n = 1'b0;                   // buffers always enabled once configured

   // Odd parity across the byte currently on the wire, whichever way
   // it is flowing. While the host drives, that is the byte we are
   // sampling live.
   wire [7:0] bus_now = drive_en ? drive_data : pd_in;
   assign pparity = ~(^bus_now);

   // ---- command-phase strobe trace ----
   // Sampled at each strobe's RISING edge, where the byte is valid
   // for certain (the Teensy build reads the bus repeatedly through
   // the low window and keeps the last value, which is the same
   // instant). tr_nstb is the count.
   reg tr_armed_d;
   reg [7:0] d_cnt;
   reg       d_run;
   reg [7:0] lat_cnt;
   reg       lat_pend;
   always @(posedge clk) begin
      tr_armed_d <= trace_arm;
      if (rst) begin
         tr0 <= 8'h00; tr1 <= 8'h00; tr2 <= 8'h00; tr3 <= 8'h00; tr_nstb <= 4'd0;
         tr_prw <= 4'd0;
      end else if (trace_arm && !tr_armed_d) begin
         tr_nstb <= 4'd0;                 // a new command phase started
         tr_prw  <= 4'd0;
      end else if (trace_arm && stb_rise) begin
         if (tr_nstb != 4'hF) tr_nstb <= tr_nstb + 4'd1;
         // PR/-W at each traced strobe. This is what separates the
         // two explanations for a $FF byte 0:
         //   prw HIGH at strobe 0 -> the host is not writing, so that
         //                           strobe is a PHANTOM (the card
         //                           strobing before its direction
         //                           flip lands); ARM_DELAY is the
         //                           filter for it.
         //   prw LOW at strobe 0 -> a real host write whose data
         //                           has not arrived yet; LATCH_DELAY
         //                           is the answer to it.
         case (tr_nstb)
           4'd0: begin tr0 <= bus_in; tr_prw[0] <= host_reading; end
           4'd1: begin tr1 <= bus_in; tr_prw[1] <= host_reading; end
           4'd2: begin tr2 <= bus_in; tr_prw[2] <= host_reading; end
           4'd3: begin tr3 <= bus_in; tr_prw[3] <= host_reading; end
           default: ;
         endcase
      end
      // Re-read the bus at three points after strobe 0, to time the
      // late byte.
      if (rst || (trace_arm && !tr_armed_d)) begin
         d_cnt <= 8'd0; d_run <= 1'b0;
         tr_d8 <= 8'hFF; tr_d32 <= 8'hFF; tr_d128 <= 8'hFF;
      end else if (trace_arm && stb_rise && tr_nstb == 4'd0) begin
         d_cnt <= 8'd0; d_run <= 1'b1;          // strobe 0 just ended: start timing
      end else if (d_run) begin
         d_cnt <= d_cnt + 8'd1;
         case (d_cnt)
           8'd7  : tr_d8   <= bus_in;
           8'd31 : tr_d32  <= bus_in;
           8'd127: begin tr_d128 <= bus_in; d_run <= 1'b0; end
           default: ;
         endcase
      end
   end

   // ---- host writes --------------------------------------------------------
   always @(posedge clk) begin
      host_stb <= 1'b0;
      read_stb <= 1'b0;
      if (rst) begin
         host_data <= 8'h00;
         host_data_lead <= 8'h00;
         rise_vals <= 40'h0;
         stb_armed <= 1'b0;
         arm_cnt   <= 4'd0;
         lat_cnt   <= 8'd0;
         lat_pend  <= 1'b0;
      end else begin
         // ARM at the falling edge, SAMPLE later in the low window,
         // and those are two different questions.
         //
         // Arming is the phantom filter. The //e's card can fire a
         // strobe BEFORE its direction flip lands: /PSTRB falls with
         // PR/-W still HIGH, and PR/-W drops part way
         // through. Testing PR/-W at the RISING edge is too late to
         // see that, so the phantom would be accepted as a byte.
         // Nothing drives the bus during it, so it reads $FF: a bogus
         // $FF opcode.
         //
         // Sampling late is the other half: the card's '374 can start
         // driving the real byte mid-strobe, so the falling-edge
         // value can be floating bus too.
         if (stb_fall) begin
            stb_armed      <= 1'b0;
            arm_cnt        <= ARM_DELAY[3:0];
            host_data_lead <= pd_in;             // diagnostic: what the fall saw
         end else if (arm_cnt != 4'd0) begin
            arm_cnt <= arm_cnt - 4'd1;
            // Decide once, ARM_DELAY clocks into the strobe. A
            // phantom (the card strobing before its direction flip
            // lands) still has PR/-W high here and is rejected. A
            // real byte has it low by now.
            if (arm_cnt == 4'd1) stb_armed <= !drive_en && !host_reading;
         end
         // Latch a host byte 4 us INTO the strobe's LOW window,
         // qualified at the FALL.
         //
         // WRITE strobes on this card have the OPPOSITE duty cycle
         // from the 355-660 ns READ strobe: LOW for 5-15 us with the
         // byte valid, HIGH for about 0.5 us with the bus decaying to
         // $FF. So the byte lives in the low window and is gone
         // roughly 400 ns after the rise. Sampling any time after the
         // rise reads the NEXT window instead: a frame shifted one
         // byte late, whose opcode is $FF.
         //
         // PR/-W goes low about 14 us before the opcode's strobe
         // falls, so qualifying at the fall is safe. The rise guard
         // covers any strobe shorter than the delay: at the rise the
         // byte still has ~400 ns left and bus_in is only 40 ns
         // stale.
         if (!drive_en && !host_reading && stb_fall) begin
            if (LATCH_DELAY == 0) begin
               host_data <= pd_in;
               host_stb  <= 1'b1;
            end else begin
               lat_cnt  <= LATCH_DELAY[7:0];
               lat_pend <= 1'b1;
            end
            rise_vals <= {rise_vals[31:0], bus_in};
            stb_armed <= 1'b0;
         end else if (lat_pend && stb_rise) begin
            host_data <= bus_in;             // short strobe: last valid instant
            host_stb  <= 1'b1;
            lat_pend  <= 1'b0;
         end else if (lat_pend) begin

            if (lat_cnt <= 8'd1) begin
               host_data <= bus_in;     // the byte, now that the card is driving it
               host_stb  <= 1'b1;
               lat_pend  <= 1'b0;
            end else begin
               lat_cnt <= lat_cnt - 8'd1;
            end
         end else if (stb_rise && drive_en) begin
            read_stb  <= 1'b1;                   // host took the byte we presented
         end
      end
   end

endmodule

`default_nettype wire
