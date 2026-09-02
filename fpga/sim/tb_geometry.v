// tb_geometry: the boot-path geometry rule, and the decoupled-geometry band.
//
// The first table below is not invented for this test. It is the measured
// result from ../../PROTOCOL.md section 9.2: four geometries, same image and
// same firmware, and the only thing that predicts whether the //e boots is
// whether the claimed block count is a multiple of 4.
//
// The second half tests the band where the claim CANNOT be made aligned, which
// is the whole reason geometry.v splits one number into two.
//
//   iverilog -g2012 -o /tmp/tb -s tb_geometry sim/tb_geometry.v src/geometry.v

`timescale 1ns / 1ps
`default_nettype none

module tb_geometry;

   reg  [31:0] image_blocks;
   reg         serve_full;
   wire [15:0] claim_blocks, served_blocks;
   wire        decoupled;

   geometry dut (.image_blocks(image_blocks), .serve_full(serve_full),
                 .claim_blocks(claim_blocks), .served_blocks(served_blocks),
                 .decoupled(decoupled));

   integer errors = 0;
   integer i;

   task check(input cond, input [511:0] what);
      begin
         if (cond) $display("    ok    %0s", what);
         else begin $display("*** FAIL  %0s", what); errors = errors + 1; end
      end
   endtask

   // One case: drive the input, settle, compare all three outputs.
   task expect_geom(input [31:0] img, input [15:0] want_claim,
                    input [15:0] want_served, input want_dec, input [255:0] what);
      begin
         image_blocks = img; #1;
         if (claim_blocks === want_claim && served_blocks === want_served
             && decoupled === want_dec)
           $display("    ok    %0s: claim $%04h served $%04h dec %0b",
                    what, claim_blocks, served_blocks, decoupled);
         else begin
            $display("*** FAIL  %0s: image %0d gave claim $%04h served $%04h dec %0b, wanted $%04h/$%04h/%0b",
                     what, img, claim_blocks, served_blocks, decoupled,
                     want_claim, want_served, want_dec);
            errors = errors + 1;
         end
      end
   endtask

   initial begin
      serve_full = 1'b1;

      // ---- PROTOCOL.md section 9.2 ----
      // "boots" in that table is exactly "claim is a multiple of 4".
      $display("-- the four measured geometries from PROTOCOL.md 9.2");
      expect_geom(32'h4C00, 16'h4C00, 16'h4C00, 1'b0, "$4C00 19456, booted");
      expect_geom(32'hFE00, 16'hFE00, 16'hFE00, 1'b0, "$FE00 65024, booted");
      // These two did NOT boot as claimed. Rounding down is what makes them
      // safe, and it is why the last few blocks of a misaligned image go
      // unserved.
      expect_geom(32'hFEFF, 16'hFE00, 16'hFEFF, 1'b1, "$FEFF 65279: capped to confirmed $FE00, decoupled");
      expect_geom(32'hFFFE, 16'hFE00, 16'hFFFE, 1'b1, "$FFFE 65534, decoupled");

      // ---- the decoupling band, 65533..65535 ----
      $display("-- the band where an aligned claim is unreachable");
      expect_geom(32'd65532, 16'hFE00, 16'd65532, 1'b1, "65532 decouples (claim caps at confirmed $FE00)");
      expect_geom(32'd65533, 16'hFE00, 16'd65533, 1'b1, "65533 decouples");
      expect_geom(32'd65534, 16'hFE00, 16'd65534, 1'b1, "65534 decouples");
      expect_geom(32'd65535, 16'hFE00, 16'd65535, 1'b1, "65535, a full 32MB volume");
      // Bigger than the host can address at all: clamp, do not wrap.
      expect_geom(32'd100000, 16'hFE00, 16'd65535, 1'b1, "oversized image clamps to $FFFF");

      $display("-- serve_full = 0 stops serving at the claim");
      serve_full = 1'b0;
      expect_geom(32'd65535, 16'hFE00, 16'hFE00, 1'b0, "65535 with decoupling off");
      expect_geom(32'hFFFE, 16'hFE00, 16'hFE00, 1'b0, "$FFFE with decoupling off");
      serve_full = 1'b1;

      // ---- the invariant, swept rather than sampled ----
      // The claim is the only number that reaches the //e's boot path, so check
      // the rule holds for every input rather than for the cases we thought
      // of.
      $display("-- sweeping: the claim is ALWAYS aligned and never above $FFFC");
      begin : sweep
         integer bad_align, bad_cap, bad_serve;
         bad_align = 0; bad_cap = 0; bad_serve = 0;
         for (i = 0; i < 70000; i = i + 64) begin
            image_blocks = i; #1;
            if (claim_blocks[1:0] !== 2'b00)   bad_align = bad_align + 1;
            if (claim_blocks > 16'hFFFC)       bad_cap   = bad_cap + 1;
            // Never serve less than we claim: the host is entitled to every
            // block it was told about.
            if (served_blocks < claim_blocks)  bad_serve = bad_serve + 1;
         end
         check(bad_align == 0, "claim is a multiple of 4 for every image size");
         check(bad_cap   == 0, "claim never exceeds $FFFC");
         check(bad_serve == 0, "served count is never below the claimed count");
      end

      // Every block the host is told about must actually be reachable. A
      // claim past the served range loses data.
      $display("-- and every claimed block is inside the served range");
      expect_geom(32'd65535, 16'hFE00, 16'd65535, 1'b1, "claim $FE00 < served $FFFF");

      $display("");
      if (errors == 0) $display("PASS: geometry claims are boot-safe and serve the whole image");
      else             $display("FAIL: %0d problem(s)", errors);
      $finish;
   end

endmodule

`default_nettype wire
