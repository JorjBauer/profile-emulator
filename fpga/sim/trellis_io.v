// TRELLIS_IO: a simulation model of the ECP5 I/O primitive.
//
// src/profile_bus.v instantiates TRELLIS_IO explicitly rather than writing a
// behavioral tri-state and hoping the toolchain infers the right thing. That
// is the safer choice on a 5V bus, but it creates a new hazard. TRELLIS_IO is
// a Lattice primitive that yosys knows and iverilog does not. Without this
// model the simulated design and the synthesized design would stop being the
// same design. Everything the testbenches check about bus direction and
// contention would then be checked on code that is no longer built.
//
// This is the primitive the SHIPPING bitstream uses, and the only pad model in
// the tree: FAMILY="ECP5" is the only configuration the benches can build.
//
// THE POLARITY IS THE WHOLE POINT. T is the TRISTATE, so it is the inverse of
// an output enable: T high is high-Z, T low drives. profile_bus.v passes
// ~drive_en for exactly that reason. Getting it backward turns the pads on
// whenever the design meant to release them, which on this board means the
// FPGA and the //e's interface card driving the same 5V net at once. A model
// that got this wrong would hide the very fault the explicit primitive exists
// to prevent, so it is written to match Lattice's sense and nothing else.
//
// Only DIR="BIDIR" is modeled, because it is the only mode the design uses.

`default_nettype none

module TRELLIS_IO #(
                    parameter DIR = "BIDIR"
                    ) (
                       inout  wire B,      // the pad
                       input  wire I,      // data out, driven onto the pad when T is low
                       input  wire T,      // TRISTATE: high releases the pad
                       output wire O       // data in, always follows the pad
                       );

   initial begin
      if (DIR != "BIDIR") begin
         $display("*** TRELLIS_IO sim model: DIR=\"%0s\" is not modeled here.", DIR);
         $display("    Only \"BIDIR\" is. Extend this model rather than trusting it");
         $display("    for a direction it does not implement.");
         $finish;
      end
   end

   assign B = T ? 1'bz : I;
   assign O = B;

endmodule

`default_nettype wire
