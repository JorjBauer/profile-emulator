// spare_table: synthesizes the block $FFFFFF status/spare table.
//
// Layout from PROTOCOL.md (profile-dumper) section 5. Purely combinational from
// the byte index, so it costs no BRAM. Every host boot starts by reading this
// block (PROTOCOL.md (profile-dumper) section 9.1), so it has to be right
// before anything else can work.
//
// The block count arrives as a wire from geometry.v, which owns the rule that
// it be a multiple of 4 and no greater than $FFFC (section 9.2). It is clamped
// AGAIN here, because this is the only place the number reaches the host. A
// bad value costs a boot that fails with no read ever being attempted. The
// duplication is two gates and it is worth it.

`default_nettype none

module spare_table #(
                     parameter [15:0] FW_REV = 16'h0404          // reported firmware revision
                     ) (
                        input  wire [15:0] blocks_claim,            // from geometry.v
                        input  wire [9:0]  addr,
                        output reg  [7:0]  data
                        );

   // Multiple of 4, never above $FFFC, and 16-bit because that is all the card
   // keeps (section 9.2).
   wire [15:0] BLK_CAP = (blocks_claim > 16'hFFFC) ? 16'hFFFC : blocks_claim;
   wire [15:0] BLOCKS  = {BLK_CAP[15:2], 2'b00};

   always @* begin
      case (addr)
        // 0x00..0x0C: device name, 13 bytes, space padded
        10'h000: data = "P";  10'h001: data = "R";  10'h002: data = "O";
        10'h003: data = "F";  10'h004: data = "I";  10'h005: data = "L";
        10'h006: data = "E";  10'h007: data = " ";  10'h008: data = " ";
        10'h009: data = " ";  10'h00A: data = " ";  10'h00B: data = " ";
        10'h00C: data = " ";
        // 0x0D..0x0F: device number, 000010 = 10MB class
        10'h00D: data = 8'h00; 10'h00E: data = 8'h00; 10'h00F: data = 8'h10;
        // 0x10..0x11: firmware revision
        10'h010: data = FW_REV[15:8];  10'h011: data = FW_REV[7:0];
        // 0x12..0x14: total blocks, MS byte first
        10'h012: data = 8'h00;
        10'h013: data = BLOCKS[15:8];
        10'h014: data = BLOCKS[7:0];
        // 0x15..0x16: bytes per block = 532
        10'h015: data = 8'h02;  10'h016: data = 8'h14;
        // 0x17..0x19: spares total / used / bad
        10'h017: data = 8'h20;  10'h018: data = 8'h00;  10'h019: data = 8'h00;
        // 0x1A..: empty spared list, then empty bad-block list
        10'h01A: data = 8'hFF;  10'h01B: data = 8'hFF;  10'h01C: data = 8'hFF;
        10'h01D: data = 8'hFF;  10'h01E: data = 8'hFF;  10'h01F: data = 8'hFF;
        default: data = 8'h00;
      endcase
   end

endmodule

`default_nettype wire
