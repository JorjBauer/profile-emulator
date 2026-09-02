// bus_exercise: drive a known pattern onto the ProFile bus, with no host.
//
// The drive talks first. This walks a deliberately unambiguous
// pattern across the data byte and the drive-driven control lines,
// slowly, and the sniffer's live monitor prints every change. That
// checks the half of the bus the drive owns:
//
//   - every PD bit lands on the DB-25 pin it should (the bit order is
//     not free: PD7 is pin 13 and PD0 is pin 5, see HARDWARE.md). A
//     one-hot walk makes a swapped or stuck bit obvious by inspection
//     rather than by arithmetic.
//   - /PPARITY is generated correctly, and for the RIGHT byte. It is
//   odd parity across the nine bits, so it must change on every
//   one-hot step.
//   - /PBSY can actually be asserted.
//   - the '245 turns around: the last steps release the bus, and the
//     sniffer should see the lines stop following the pattern.
//
// It cannot test /PCMD, /PSTRB, PR/-W or /PRES, because those are
// host-driven and nothing here can drive them. That half of the bus
// needs a real host.
//
// The sniffer has two modes. Its live monitor coalesces changes
// closer than 20 ms, which argues for a slow pattern. But the live
// monitor only runs while /PCMD is HIGH, and with no host attached
// /PCMD floats LOW: nothing drives the host-owned lines, and floating
// CMOS reads low. So the sniffer triggers instantly and goes into its
// capture loop instead, and THAT ends after 100 us with no
// transitions.
//
// So the dwell must be MICROseconds, not milliseconds. At 20 us the
// whole sixteen-step pattern takes 320 us. The capture records every
// edge with nanosecond timestamps. It keeps recording repeats until
// its 8192-sample buffer fills.

`default_nettype none

module bus_exercise #(
                      parameter integer CLK_HZ   = 50_000_000,
                      parameter integer DWELL_US = 20
                      ) (
                         input  wire       clk,
                         input  wire       rst,
                         input  wire       enable,      // build-time switch
                         input  wire       inhibit,     // a host appeared: get off the bus immediately

                         output reg        drive_en,    // 1 = we own the data bus
                         output reg  [7:0] drive_data,
                         output reg        bsy_n,       // active low
                         output wire [3:0] step_out     // which step, for the LEDs
                         );

   localparam integer DWELL = (CLK_HZ / 1_000_000) * DWELL_US;

   reg [31:0]         t;
   reg [3:0]          step;

   assign step_out = step;

   always @(posedge clk) begin
      if (rst || !enable) begin
         t          <= 32'd0;
         step       <= 4'd0;
         drive_en   <= 1'b0;
         drive_data <= 8'h00;
         bsy_n      <= 1'b1;
      end else if (inhibit) begin
         // A host is talking. Release everything at once and stay
         // released: the only fault on this board that can destroy
         // hardware is two drivers on the bus, and a diagnostic is
         // never worth that.
         drive_en <= 1'b0;
         bsy_n    <= 1'b1;
         t        <= 32'd0;
         step     <= 4'd0;
      end else begin
         if (t == DWELL[31:0]) begin
            t    <= 32'd0;
            step <= step + 4'd1;
         end else begin
            t <= t + 32'd1;
         end

         // A one-hot walk first, so a swapped pair shows up as two
         // steps out of order rather than as a checksum that happens
         // to be wrong. Then three patterns that catch a stuck bit
         // the walk could miss, then the control lines, then the
         // release.
         case (step)
           4'd0:  begin drive_en <= 1'b1; drive_data <= 8'h01; bsy_n <= 1'b1; end
           4'd1:  begin drive_en <= 1'b1; drive_data <= 8'h02; bsy_n <= 1'b1; end
           4'd2:  begin drive_en <= 1'b1; drive_data <= 8'h04; bsy_n <= 1'b1; end
           4'd3:  begin drive_en <= 1'b1; drive_data <= 8'h08; bsy_n <= 1'b1; end
           4'd4:  begin drive_en <= 1'b1; drive_data <= 8'h10; bsy_n <= 1'b1; end
           4'd5:  begin drive_en <= 1'b1; drive_data <= 8'h20; bsy_n <= 1'b1; end
           4'd6:  begin drive_en <= 1'b1; drive_data <= 8'h40; bsy_n <= 1'b1; end
           4'd7:  begin drive_en <= 1'b1; drive_data <= 8'h80; bsy_n <= 1'b1; end
           4'd8:  begin drive_en <= 1'b1; drive_data <= 8'h55; bsy_n <= 1'b1; end
           4'd9:  begin drive_en <= 1'b1; drive_data <= 8'hAA; bsy_n <= 1'b1; end
           4'd10: begin drive_en <= 1'b1; drive_data <= 8'hFF; bsy_n <= 1'b1; end
           4'd11: begin drive_en <= 1'b1; drive_data <= 8'h00; bsy_n <= 1'b1; end
           // Same byte twice, so the ONLY thing that changes between
           // these two steps is BSY. If the sniffer reports a data
           // change here as well, the fault is crosstalk or a shared
           // pin, not the BSY line.
           4'd12: begin drive_en <= 1'b1; drive_data <= 8'h5A; bsy_n <= 1'b0; end
           4'd13: begin drive_en <= 1'b1; drive_data <= 8'h5A; bsy_n <= 1'b1; end
           // Release. The sniffer should stop seeing $5A here:
           // whatever it reports is the idle level of the DB-25 with
           // nothing driving it.
           4'd14: begin drive_en <= 1'b0; drive_data <= 8'h5A; bsy_n <= 1'b1; end
           default: begin drive_en <= 1'b0; drive_data <= 8'h5A; bsy_n <= 1'b0; end
         endcase
      end
   end

endmodule

`default_nettype wire
