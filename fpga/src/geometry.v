// geometry: turns an image size into the TWO block counts the drive needs.
//
// This is the HDL twin of applyGeometry() / geometryDecoupled() in
// the Teensy emulator.ino: the number we CLAIM and the number we
// SERVE are not the same number, and every place that recomputed them
// separately was a place they could drift apart.
//
//   claim_blocks   what the spare table reports at offset $12..$14
//                  (PROTOCOL.md (profile-dumper) section 5). Must be
//                  a multiple of 4 or the //e's ROM never hands off
//                  to the boot block. It must also fit the 16 bits the
//                  interface card actually keeps, so it tops out at
//                  $FFFC. See PROTOCOL.md section 9.2.
//   served_blocks  how many blocks we will actually read and write.
//
// For any image of 65532 blocks or fewer these are equal and none of
// this matters. They diverge for a full 32MB ProDOS volume: 65535
// blocks, which is three too many AND misaligned, so it can never be
// claimed.
//
// The multiple-of-4 rule is ONE instruction in the ROM's boot path
// ($CBFD: lda $46 / and #$03 / bne, guarding the jsr $CC75 handoff),
// where $46 holds the low byte of the count we reported. The data
// path does not bounds-check the block number against that count: a
// drive claiming $4C00 = 19456 blocks will serve a READ of block
// 40000.  So the claim only has to satisfy the boot. ProDOS takes
// the volume's real size from total_blocks in block 2 of the image,
// not from us. For the ProFile 10 card this was developed against
// (ROM 341-0299-B) this is fine.
//
// Rounding down leaves ANY misaligned image 1 to 3 blocks above its
// claim, not just the big ones. Only 65533..65535 gets this
// treatment, because that is the only range where the aligned claim
// is unreachable: anywhere below it `wozit resize` is available to
// keep images in the "known works under normal conditions" path.
// Again, though: this is based on that one ROM image and that one
// //e. Other ProFile 5 and ProFile 10 ROM versions, and the Apple ///
// and Lisa controllers, might bounds-check differently. 32MB images
// work here because of a lack of proper bounds checking.
//
// This module is combinational. The inputs are wires rather than
// parameters because the image size comes off the SD card when the
// image file is resolved at runtime.

`default_nettype none

module geometry (
                 input  wire [31:0] image_blocks,   // blocks the image file actually holds
                 input  wire        serve_full,     // 0 stops serving at the claim

                 output wire [15:0] claim_blocks,   // reported to the host
                 output wire [15:0] served_blocks,  // actually readable and writable
                 output wire        decoupled       // telemetry: serving above the claim
                 );

   // The host's block number is 24-bit but the interface card keeps
   // 16, so a bigger image is simply unreachable past $FFFF however
   // it is claimed.
   wire [15:0] img16 =
               (image_blocks > 32'h0000_FFFF) ? 16'hFFFF : image_blocks[15:0];

   // $FE00, NOT $FFFC, even though the mod-4 rule alone would allow
   // $FFFC.  The card maps ProDOS blocks $FFxx into the drive's
   // $FFFFxx magic window, so any claim above $FEFF exposes device
   // blocks that collide with the spare table and the selector
   // window. Claiming that high does not boot at all.  $FE00 is the
   // highest claim shown to work on hardware.
   wire [15:0] capped = (img16 > 16'hFE00) ? 16'hFE00 : img16;

   // Multiple of 4, always. This is the boot-path constraint of my
   // ROM (341-0299-B).
   assign claim_blocks = {capped[15:2], 2'b00};

   // Decoupling only ever means "serve the blocks the claim could not
   // reach".
   assign decoupled = serve_full && (img16 > 16'hFE00) && (img16 > claim_blocks);

   assign served_blocks = decoupled ? img16 : claim_blocks;

endmodule

`default_nettype wire
