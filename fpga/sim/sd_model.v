// sd_model: an SDHC card in SPI mode, enough of one to bring sd_spi.v up.
//
// Models what the initialization sequence actually depends on, and nothing
// else:
//
//   CMD0   GO_IDLE_STATE     -> R1 = $01
//   CMD8   SEND_IF_COND      -> R7 = $01 + 00 00 01 AA (echoes the pattern)
//   CMD55  APP_CMD           -> R1 = $01 while idle, $00 after
//   ACMD41 SD_SEND_OP_COND   -> R1 = $01 for the first BUSY_REPS tries, then $00
//   CMD58  READ_OCR          -> R1 = $00 + OCR with CCS set (high capacity)
//   CMD17  READ_SINGLE_BLOCK -> R1 = $00, latency, $FE, 512 bytes, 2 CRC bytes
//
// The ACMD41 busy loop is the point of the model: it checks that the host polls
// until the card answers $00. NBUSY_TOKEN exists for the same reason: it puts
// latency before the $FE token, so the host must wait for the token.
//
// Sector contents are synthesized from the LBA (see fill_sector) so any sector
// can be checked without loading an image. Sector 0 also carries a $55 $AA
// signature at offset 510, so the test can assert on something a real card
// would also have.

`timescale 1ns / 1ps
`default_nettype none

module sd_model #(
                  parameter integer BUSY_REPS   = 3,   // ACMD41 answers busy this many times
                  parameter integer NBUSY_TOKEN = 4,   // $FF bytes before the $FE data token
                  parameter integer SKEW        = 0,   // extra idle BITS before the reply stream
                  parameter         V1          = 1'b0, // reject CMD8 and address by BYTE
                  parameter         STRICT_CRC  = 1'b1, // check CRC7 on EVERY command
                  parameter         FS_IMAGE    = 1'b0, // serve a small synthetic FAT32 volume
                  // Reject the first FAIL_READS reads with a bad R1, then
                  // behave: a transient real cards produce, so the retry path
                  // is exercised rather than merely written.
                  parameter integer FAIL_READS  = 0,
                  // Omit profile.cfg, so the resolver has nothing to obey.
                  parameter         FS_NOCFG    = 1'b0,
                  // Add a SECOND image ("two.img", clusters 9->11 fragmented)
                  // to /HD, so a resolver's LRU-2 chain cache can be exercised
                  // across two images.
                  parameter         TWO_IMG     = 1'b0,
                  // Give /HD an image with NO long-name chain in front of it,
                  // named in profile.cfg by its plain 8.3 name. A2OSX.IMG on
                  // the card this models is such a file: DOS-era tools write no
                  // LFN when the name already fits 8.3, so this is the common
                  // case rather than a corner one, and the resolver has to
                  // reach it through the short name alone.
                  parameter         NOLFN       = 1'b0,
                  // Point profile.cfg at the SECOND image instead of the first,
                  // so a bench can select the first one and have that be a
                  // real change of selection. Without it the only image is
                  // already the configured one, and the drive short-circuits a
                  // same-index reselect as a no-op.
                  parameter         CFG_TWO     = 1'b0,
                  // Three more images after two.img, the way a real card
                  // looks: long names spanning two LFN entries, a 32 MB
                  // and a 2 MB file among them, sizes all different. Only
                  // the catalog sees them; nothing resolves them.
                  parameter         MANY_IMG    = 1'b0,
                  parameter         IS_SDHC     = 1'b1
                  ) (
                     input  wire sd_clk,
                     input  wire sd_cs_n,
                     input  wire sd_mosi,
                     output wire sd_miso
                     );

   reg [7:0] out_byte = 8'hFF;      // byte being shifted out
   reg [2:0] bit_i    = 3'd0;
   // A card whose replies are not byte-aligned to the host's framing: a real
   // card can answer CMD8 two bit-times into a byte.
   //
   // Model it by delaying only the OUTPUT. The card's own framing for the
   // command it receives is unaffected; it is the reply that arrives late.
   // Stalling the shared bit counter instead would also delay command assembly
   // and make the model reject every command, a failure the design had nothing
   // to do with.
   reg [7:0] skew_sr = 8'hFF;
   reg [7:0] in_byte  = 8'h00;

   assign sd_miso = sd_cs_n ? 1'b1
                    : (SKEW == 0) ? out_byte[7] : skew_sr[SKEW-1];

   // Response queue: what the card will hand back, byte by byte.
   localparam integer QMAX = 600;
   reg [7:0] queue [0:QMAX-1];
   integer   qhead = 0, qtail = 0;

   task qpush(input [7:0] b);
      begin queue[qtail] = b; qtail = (qtail + 1) % QMAX; end
   endtask

   function [7:0] qpop;
      begin
         if (qhead == qtail) qpop = 8'hFF;
         else begin qpop = queue[qhead]; qhead = (qhead + 1) % QMAX; end
      end
   endfunction

   // Command assembly, AT THE BIT LEVEL. A command frame starts with '0' then
   // '1', and a real card locks onto that pattern rather than assuming the
   // host's byte boundaries. That matters here: sd_spi deliberately clocks a
   // few extra bits to slide its framing onto the card's, and a model that only
   // framed on multiples of 8 would reject every command afterward. It is also
   // the reason the resync is safe on real hardware.
   reg [47:0] cmd_sr = 0;
   reg [5:0]  cmd_bits = 0;
   // One writable sector, so a write can be checked by reading it back rather
   // than by trusting that the command completed.
   reg [7:0]  wsec [0:511];
   reg [31:0] wsec_lba = 32'hFFFF_FFFF;
   // The previous written sector as well. Persisting a selection is two writes,
   // the file and then its directory entry, and a model that remembers only the
   // last one cannot show that the first happened.
   reg [7:0]  wsec_prev [0:511];
   reg [31:0] wsec_prev_lba = 32'hFFFF_FFFF;
   integer    wcp;
   reg        in_wdata = 0;
   integer    busy_left = 0;   // bytes of write-busy remaining
   integer    fail_left = FAIL_READS;
   integer    wi = 0;
   reg [7:0]  cmd [0:5];
   integer    ci = 0;
   reg        in_cmd = 0;
   reg        app_cmd = 0;
   reg        idle = 1;
   integer    acmd_count = 0;
   integer    k;
   reg [31:0] arg;

   // A minimal but self-consistent FAT32 layout, so the FAT scanner can be
   // tested without a real card:
   //
   //   sector 0    MBR, one partition, type $0B, starting at LBA 64
   //   sector 64   BPB: 1 sector/cluster, 2 reserved, 1 FAT, FAT size 1
   //   sector 66   the FAT: entries 0 and 1 reserved, cluster 2 in use,
   //               everything from cluster 3 free
   //   data start  64 + 2 + 1 = 67, so cluster 3 is sector 68
   //
   // The scanner must therefore report first_free = 3 and target LBA 68.
   localparam [31:0] FS_PART = 32'd64;
   // One byte of an 8.3 name, space padded, no dot.
   function [7:0] fs_name(input [87:0] nm, input integer k);
      fs_name = nm[8*(10 - k) +: 8];
   endfunction

   // One UTF-16 byte of the long name "longname.img": character c, low or high.
   function [7:0] lfn_ch(input integer c, input hi);
      reg [7:0] ascii;
      begin
         case (c)
           0: ascii = "l"; 1: ascii = "o"; 2: ascii = "n"; 3: ascii = "g";
           4: ascii = "n"; 5: ascii = "a"; 6: ascii = "m"; 7: ascii = "e";
           8: ascii = "."; 9: ascii = "i"; 10: ascii = "m"; 11: ascii = "g";
           12: ascii = 8'h00;                 // the name's terminator
           default: ascii = 8'hFF;            // padding past the terminator
         endcase
         lfn_ch = hi ? ((c == 12 || c > 12) ? ((c == 12) ? 8'h00 : 8'hFF) : 8'h00)
           : ascii;
      end
   endfunction

   // One UTF-16 byte of the second image's long name "two.img" (7 chars,
   // terminator at index 7). Used only when TWO_IMG.
   function [7:0] lfn2_ch(input integer c, input hi);
      reg [7:0] ascii;
      begin
         case (c)
           0: ascii = "t"; 1: ascii = "w"; 2: ascii = "o"; 3: ascii = ".";
           4: ascii = "i"; 5: ascii = "m"; 6: ascii = "g";
           7: ascii = 8'h00;                  // terminator
           default: ascii = 8'hFF;            // padding past the terminator
         endcase
         lfn2_ch = hi ? ((c >= 7) ? ((c == 7) ? 8'h00 : 8'hFF) : 8'h00) : ascii;
      end
   endfunction

   // One UTF-16 byte of an arbitrary long name held right-justified in a
   // 256-bit vector: character c of a len-character name, low or high byte.
   // Past the end, the terminator (0000) once, then FFFF padding, which is
   // what every FAT formatter writes.
   function [7:0] lfn_g(input [255:0] nm, input integer len, input integer c, input hi);
      begin
         if (c < len)       lfn_g = hi ? 8'h00 : nm[8*(len-1-c) +: 8];
         else if (c == len) lfn_g = 8'h00;
         else               lfn_g = 8'hFF;
      end
   endfunction

   // Byte o (0..31) of long-name entry number k (1-based chunk) for that name.
   function [7:0] lfn_ent(input [255:0] nm, input integer len, input integer k, input last, input integer o);
      begin
         if (o == 0)                  lfn_ent = k[7:0] | (last ? 8'h40 : 8'h00);
         else if (o >= 1 && o <= 10)  lfn_ent = lfn_g(nm, len, 13*(k-1) + ((o-1) >> 1), (o-1) & 1);
         else if (o == 11)            lfn_ent = 8'h0F;
         else if (o >= 14 && o <= 25) lfn_ent = lfn_g(nm, len, 13*(k-1) + 5 + ((o-14) >> 1), (o-14) & 1);
         else if (o >= 28)            lfn_ent = lfn_g(nm, len, 13*(k-1) + 11 + ((o-28) >> 1), (o-28) & 1);
         else                         lfn_ent = 8'h00;   // type, checksum, cluster
      end
   endfunction

   // Byte o of an 8.3 entry: name, a normal file, first cluster, size.
   function [7:0] ent83(input [87:0] n11, input integer clus, input [31:0] size, input integer o);
      begin
         if (o < 11)       ent83 = fs_name(n11, o);
         else if (o == 11) ent83 = 8'h20;
         else if (o == 26) ent83 = clus[7:0];
         else if (o == 27) ent83 = clus[15:8];
         else if (o >= 28) ent83 = size[8*(o-28) +: 8];
         else              ent83 = 8'h00;
      end
   endfunction

   // The three MANY_IMG files, entries 7..14 of /HD (bytes 224..479), with
   // the end-of-directory zeros after them.
   localparam [255:0] NM_A = "TOTAL REPLAY V5.2.HDV";   localparam integer LN_A = 21;
   localparam [255:0] NM_B = "PRODOS322MG.IMG";         localparam integer LN_B = 15;
   localparam [255:0] NM_C = "NOX.IMG";                 localparam integer LN_C = 7;
   function [7:0] many_byte(input integer j);   // j = 0..255 past byte 224
      integer e, o;
      begin
         e = j / 32; o = j % 32;
         case (e)
           0: many_byte = lfn_ent(NM_A, LN_A, 2, 1'b1, o);
           1: many_byte = lfn_ent(NM_A, LN_A, 1, 1'b0, o);
           2: many_byte = ent83("TOTALR~1HDV", 20, 32'h0200_0000, o);   // 32 MB, 65536 blocks
           3: many_byte = lfn_ent(NM_B, LN_B, 2, 1'b1, o);
           4: many_byte = lfn_ent(NM_B, LN_B, 1, 1'b0, o);
           5: many_byte = ent83("PRODOS~1IMG", 21, 32'h0020_0000, o);   // 2 MB, 4096 blocks
           6: many_byte = lfn_ent(NM_C, LN_C, 1, 1'b1, o);
           7: many_byte = ent83("NOX     IMG", 22, 32'h0000_0C00, o);   // 3 KB, 6 blocks
           default: many_byte = 8'h00;
         endcase
      end
   endfunction

   // "/HD/longname.img" + newline, 17 bytes. With NOLFN it is the SHORT name
   // instead, "/HD/a2osx.img" + newline, 14 bytes, deliberately in lower case:
   // the on-disk 8.3 entry is upper case and space padded, so a match can only
   // succeed if the comparison folds case on both sides AND the resolver has
   // compacted "A2OSX   IMG" into "A2OSX.IMG" first.
   localparam integer CFG_LEN = NOLFN ? 14 : (CFG_TWO ? 12 : 17);
   function [7:0] fs_cfg(input integer k);
      reg [135:0] path;
      begin
         if (NOLFN) begin
            path = "/HD/a2osx.img";
            fs_cfg = (k == 13) ? 8'h0A : path[8*(12 - k) +: 8];
         end else if (CFG_TWO) begin
            path = "/HD/two.img";
            fs_cfg = (k == 11) ? 8'h0A : path[8*(10 - k) +: 8];
         end else begin
            path = "/HD/longname.img";
            fs_cfg = (k == 16) ? 8'h0A : path[8*(15 - k) +: 8];
         end
      end
   endfunction

   task fs_byte(input [31:0] lba, input integer j, output [7:0] b);
      begin
         b = 8'h00;
         if (lba == 0) begin
            if (j == 446+4) b = 8'h0B;                    // partition type
            else if (j == 446+8)  b = FS_PART[7:0];       // start LBA, little endian
            else if (j == 446+9)  b = FS_PART[15:8];
            else if (j == 446+10) b = FS_PART[23:16];
            else if (j == 446+11) b = FS_PART[31:24];
            else if (j == 510) b = 8'h55;
            else if (j == 511) b = 8'hAA;
         end else if (lba == FS_PART) begin
            if      (j == 8'h0B) b = 8'h00;               // bytes per sector, 512
            else if (j == 8'h0C) b = 8'h02;
            else if (j == 8'h0D) b = 8'd1;                // sectors per cluster
            else if (j == 8'h0E) b = 8'd2;                // reserved sectors
            else if (j == 8'h10) b = 8'd1;                // number of FATs
            else if (j == 8'h24) b = 8'd1;                // FAT size, 32-bit
            else if (j == 8'h2C) b = 8'd2;                // root directory cluster
            else if (j == 510) b = 8'h55;
            else if (j == 511) b = 8'hAA;

         end else if (lba == FS_PART + 2) begin          // the FAT
            // data_start = 64 + 2 + 1 = 67, so cluster N is LBA 67 + (N-2).
            //   2 root dir      3 the HD directory
            //   4 6 5 7         the image file, DELIBERATELY OUT OF ORDER
            //
            // The image's chain is deliberately fragmented. Files on real cards
            // are too, often across many breaks. A resolver that assumes
            // contiguity reads the wrong blocks here. On hardware it would
            // corrupt every read.
            case (j)
              0,1,2:    b = 8'hFF;    3:  b = 8'h0F;      // entry 0
              4,5,6:    b = 8'hFF;    7:  b = 8'h0F;      // entry 1
              8,9,10:   b = 8'hFF;    11: b = 8'h0F;      // 2 root dir, EOC
              12,13,14: b = 8'hFF;    15: b = 8'h0F;      // 3 HD dir, EOC
              16: b = 8'd6;                               // 4 -> 6
              20: b = 8'd7;                               // 5 -> 7
              24: b = 8'd5;                               // 6 -> 5
              28,29,30: b = 8'hFF;    31: b = 8'h0F;      // 7 EOC
              32,33,34: b = 8'hFF;    35: b = 8'h0F;      // 8 profile.cfg, EOC
              // image2 "two.img": 9 -> 11 (fragmented, skips 10), 11 EOC
              36: b = TWO_IMG ? 8'd11 : 8'h00;            // 9 -> 11
              44,45,46: b = TWO_IMG ? 8'hFF : 8'h00;
              47: b = TWO_IMG ? 8'h0F : 8'h00;            // 11 EOC
              default:  b = 8'h00;
            endcase

         end else if (lba == FS_PART + 3) begin          // root dir, cluster 2
            // A volume label to skip, the HD directory, and the config file.
            if      (j < 11)  b = fs_name("VOLNAME    ", j);
            else if (j == 11) b = 8'h08;                  // volume label: skip
            else if (j < 32)  b = 8'h00;
            else if (j < 43)  b = fs_name("HD         ", j - 32);
            else if (j == 43) b = 8'h10;                  // a directory
            else if (j == 58) b = 8'd3;                   // first cluster low = 3
            else if (j < 64)  b = 8'h00;
            else if (FS_NOCFG && j >= 64) b = 8'h00;      // no config at all
            else if (j < 75)  b = fs_name("PROFILE CFG", j - 64);
            else if (j == 75) b = 8'h00;                  // attr 0, as on the card
            else if (j == 90) b = 8'd8;                   // first cluster low = 8
            else if (j == 92) b = CFG_LEN[7:0];           // size of the path written
            else               b = 8'h00;

         end else if (lba == FS_PART + 4) begin          // /HD, cluster 3
            // "." and ".." to skip, a .TXT that must not match, then the image
            // behind a LONG FILE NAME. The card this models stores most of its
            // images that way and its profile.cfg refers to them by long path,
            // so the default layout here exercises the LFN route. Set NOLFN for
            // the other one: the same image with no long name at all, named in
            // the config by its bare 8.3 name.
            //
            // "longname.img" is 12 characters, so one LFN entry holds it:
            // sequence 1, with $40 set to mark the last chunk of the name.
            if      (j < 11)  b = fs_name(".          ", j);
            else if (j == 11) b = 8'h10;
            else if (j < 32)  b = 8'h00;
            else if (j < 43)  b = fs_name("..         ", j - 32);
            else if (j == 43) b = 8'h10;
            else if (j < 64)  b = 8'h00;
            else if (j < 75)  b = fs_name("README  TXT", j - 64);
            else if (j == 75) b = 8'h20;
            else if (j < 96)  b = 8'h00;
            // --- NOLFN: an image with NO long name at all --------------------
            // Its 8.3 entry stands alone in the slot the LFN chain would have
            // occupied, and the slot after it stays all zeros, which is the
            // end-of-directory marker. Same cluster and size as the long-named
            // image, so every downstream check is unchanged: only the route to
            // the name differs. This branch runs to a catch-all so none of the
            // long-name arms below can fire.
            else if (NOLFN && j < 107) b = fs_name("A2OSX   IMG", j - 96);
            else if (NOLFN && j == 107) b = 8'h20;        // a normal file
            else if (NOLFN && j == 122) b = 8'd4;         // first cluster low = 4
            else if (NOLFN && j == 124) b = 8'h00;        // size = 2048 = 4 blocks
            else if (NOLFN && j == 125) b = 8'h08;
            else if (NOLFN)   b = 8'h00;
            // --- the LFN entry, immediately before its 8.3 entry ---
            else if (j == 96)  b = 8'h41;                 // seq 1, last chunk
            else if (j == 107) b = 8'h0F;                 // LFN attribute
            else if (j == 108) b = 8'h00;                 // type
            else if (j == 109) b = 8'h00;                 // checksum, not checked
            // Offsets 1..10 of the entry hold characters 0..4, low byte first.
            // The entry starts at an even j, so j[0] is NOT the low/high
            // selector for this run: it has to be computed from the offset.
            else if (j >= 97 && j <= 106) b = lfn_ch((j - 97) >> 1, (j - 97) & 1);
            else if (j >= 110 && j <= 121) b = lfn_ch(5 + ((j - 110) >> 1), j[0]);
            else if (j >= 124 && j <= 127) b = lfn_ch(11 + ((j - 124) >> 1), j[0]);
            else if (j == 122 || j == 123) b = 8'h00;     // first cluster, always 0
            // --- the 8.3 entry it belongs to ---
            else if (j < 139) b = fs_name("LONGNA~1IMG", j - 128);
            else if (j == 139) b = 8'h20;                 // a normal file
            else if (j == 154) b = 8'd4;                  // first cluster low = 4
            else if (j == 156) b = 8'h00;                 // size = 2048 = 4 blocks
            else if (j == 157) b = 8'h08;
            // --- image2 "two.img": LFN at 160, 8.3 at 192 (only when TWO_IMG)
            // ---
            else if (TWO_IMG && j == 160) b = 8'h41;      // seq 1, last chunk
            else if (TWO_IMG && j == 171) b = 8'h0F;      // LFN attribute
            else if (TWO_IMG && j == 172) b = 8'h00;      // type
            else if (TWO_IMG && j == 173) b = 8'h00;      // checksum, not checked
            else if (TWO_IMG && j >= 161 && j <= 170)
              b = lfn2_ch((j - 161) >> 1, (j - 161) & 1);
            else if (TWO_IMG && j >= 174 && j <= 185)
              b = lfn2_ch(5 + ((j - 174) >> 1), j[0]);
            else if (TWO_IMG && j >= 188 && j <= 191)
              b = lfn2_ch(11 + ((j - 188) >> 1), j[0]);
            else if (TWO_IMG && (j == 186 || j == 187)) b = 8'h00;  // first cluster
            else if (TWO_IMG && j >= 192 && j < 203) b = fs_name("TWO     IMG", j - 192);
            else if (TWO_IMG && j == 203) b = 8'h20;      // a normal file
            else if (TWO_IMG && j == 218) b = 8'd9;       // first cluster low = 9
            else if (TWO_IMG && j == 221) b = 8'h04;      // size = 1024 = 2 blocks
            else if (MANY_IMG && j >= 224 && j < 480) b = many_byte(j - 224);
            else               b = 8'h00;

         end else if (lba == FS_PART + 9) begin          // profile.cfg, cluster 8
            // A path, exactly as the Teensy build persists it.
            if (j < CFG_LEN) b = fs_cfg(j);
            else             b = 8'h00;

         end else if (lba >= FS_PART + 5 && lba <= FS_PART + 8) begin
            // The image's four blocks, at cluster order 4,6,5,7 = LBA
            // 69,71,70,72.
            b = (lba[7:0] * 8'd11) + j[7:0];
         end
      end
   endtask

   task fill_sector(input [31:0] lba);
      integer j;
      reg [7:0] fb;
      begin
         qpush(8'hFE);
         if (FS_IMAGE) begin
            for (j = 0; j < 512; j = j + 1) begin
               fs_byte(lba, j, fb);
               qpush(fb);
            end
            qpush(8'h01); qpush(8'h02);
         end else begin
            for (j = 0; j < 512; j = j + 1) begin
               if (lba == 0 && j == 510)      qpush(8'h55);
               else if (lba == 0 && j == 511) qpush(8'hAA);
               else if (j < 4)                qpush(lba[8*(3-j) +: 8]);
               else                           qpush((lba[7:0] + j) & 8'hFF);
            end
            qpush(8'h01); qpush(8'h02);          // CRC16, not checked by sd_spi
         end
      end
   endtask

   // CRC7 over the 40-bit frame, x^7 + x^3 + 1. The real card checks this on
   // every command, not only CMD0 and CMD8: ACMD41 with a dummy CRC gets $09,
   // idle plus COM_CRC_ERROR. Modeling it strictly means a host that
   // regresses to hard-coded constants fails here rather than on the bench.
   function [7:0] crc7_of(input [39:0] f);
      integer b;
      reg [6:0] c;
      begin
         c = 0;
         for (b = 39; b >= 0; b = b - 1)
           c = ((c << 1) & 7'h7F) ^ (((f[b] ^ c[6]) == 1'b1) ? 7'h09 : 7'h00);
         crc7_of = {c, 1'b1};
      end
   endfunction

   task handle_command;
      begin
         arg = {cmd[1], cmd[2], cmd[3], cmd[4]};
         if (STRICT_CRC &&
             cmd[5] !== crc7_of({cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]})) begin
            $display("    sd_model: BAD CRC on CMD%0d, got %02h want %02h",
                     cmd[0][5:0], cmd[5],
                     crc7_of({cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]}));
            qpush(8'h09);                    // idle + COM_CRC_ERROR
         end else
           case (cmd[0][5:0])
             6'd0: begin idle = 1; app_cmd = 0; qpush(8'h01); end
             6'd8: begin
                // A v1 card does not know CMD8 and says so: R1 with the
                // illegal-command bit, and NO R7 tail. This is what a real v1
                // card does, and it must not be treated as fatal.
                if (V1) qpush(8'h05);
                else begin
                   qpush(8'h01);
                   qpush(8'h00); qpush(8'h00); qpush(8'h01); qpush(8'hAA);
                end
             end
             6'd55: begin app_cmd = 1; qpush(idle ? 8'h01 : 8'h00); end
             6'd41: begin
                // A v1 card must not see the HCS bit; if the host still sets
                // it, refuse, so the test catches a host that forgets to drop
                // it.
                if (V1 && arg[30]) qpush(8'h05);
                else if (!app_cmd) qpush(8'h05);         // ACMD without CMD55
                else begin
                   acmd_count = acmd_count + 1;
                   if (acmd_count > BUSY_REPS) begin idle = 0; qpush(8'h00); end
                   else qpush(8'h01);
                end
                app_cmd = 0;
             end
             6'd58: begin
                qpush(8'h00);
                qpush((IS_SDHC && !V1) ? 8'hC0 : 8'h80); // bit30 = CCS
                qpush(8'hFF); qpush(8'h80); qpush(8'h00);
             end
             6'd24: begin
                $display("SDMODEL CMD24 arg=%0d (lba=%0d) t=%0t", arg, (V1?(arg>>9):arg), $time);
                qpush(8'h00);                     // accepted; data block follows
                in_wdata = 1; wi = -1;            // -1: the $FE token has yet to arrive
                for (wcp = 0; wcp < 512; wcp = wcp + 1) wsec_prev[wcp] = wsec[wcp];
                wsec_prev_lba = wsec_lba;
                wsec_lba = V1 ? (arg >> 9) : arg;
             end
             6'd17: if (fail_left > 0) begin
                // R1 with the illegal-command bit set: the read never starts,
                // which is what a card does when it is not ready to talk about
                // that sector.
                fail_left = fail_left - 1;
                qpush(8'h04);
             end else begin
                qpush(8'h00);
                for (k = 0; k < NBUSY_TOKEN; k = k + 1) qpush(8'hFF);
                // A byte-addressed card expects arg = sector * 512, so a host
                // that forgets to scale would read the wrong sector and the LBA
                // echo in the sector body would catch it.
                if ((V1 ? (arg >> 9) : arg) == wsec_lba) begin
                   qpush(8'hFE);
                   for (k = 0; k < 512; k = k + 1) qpush(wsec[k]);
                   qpush(8'h01); qpush(8'h02);
                end else
                  fill_sector(V1 ? (arg >> 9) : arg);
             end
             default: qpush(8'h04);                     // illegal command
           endcase
      end
   endtask

   // SPI mode 0: the card samples MOSI on the rising edge and presents MISO on
   // the falling edge. Deselecting resets the bit phase, which is what makes
   // the 80 warm-up clocks with CS high harmless.
   integer bi;
   integer wbit = 0;
   reg [7:0] wsr = 8'hFF;
   always @(posedge sd_clk) begin
      if (!sd_cs_n) begin
         in_byte <= {in_byte[6:0], sd_mosi};
         if (in_wdata) begin
            // Find the $FE start token AT THE BIT LEVEL, then count bytes from
            // there. Assuming the host's byte boundaries and counting modulo 8
            // from an arbitrary starting phase frames the block wrong and the
            // card never answers, which looks exactly like a host that failed
            // to send it.
            wsr = {wsr[6:0], sd_mosi};
            if (wi < 0) begin
               if (wsr == 8'hFE) begin wi = 0; wbit = 0; end
            end else begin
               wbit = wbit + 1;
               if (wbit == 8) begin
                  wbit = 0;
                  if (wi < 512) begin
                     wsec[wi] = wsr; wi = wi + 1;
                  end else if (wi < 513) begin
                     wi = wi + 1;                  // two CRC bytes, ignored
                  end else begin
                     in_wdata = 0;
                     $display("SDMODEL write COMMIT lba=%0d data[0]=%02h data[1]=%02h t=%0t",
                              wsec_lba, wsec[0], wsec[1], $time);
                     // Data response, then a REALISTIC busy period. A card does
                     // not finish a write instantly, and it does not
                     // necessarily assert busy on the very next byte either:
                     // one $FF of slack first, then eight bytes held low. A
                     // host that samples once and calls it done issues its next
                     // command into a card that is still writing.
                     qpush(8'h05);
                     qpush(8'hFF);
                     for (k = 0; k < 8; k = k + 1) qpush(8'h00);
                     busy_left = 400;              // realistic long commit, MISO held low
                  end
               end
            end
         end else if (busy_left > 0) begin
            // A card that is writing ignores everything. Modeling that is the
            // whole point: it catches a host that skips the busy wait, which
            // otherwise fails only on real hardware.
            if (bit_i == 3'd7) busy_left = busy_left - 1;
         end else if (!in_cmd) begin
            // look for the start pair 0,1
            if (in_byte[0] == 1'b0 && sd_mosi == 1'b1) begin
               in_cmd   <= 1'b1;
               cmd_sr   <= 48'b01;
               cmd_bits <= 6'd2;
            end
         end else begin
            cmd_sr   <= {cmd_sr[46:0], sd_mosi};
            cmd_bits <= cmd_bits + 6'd1;
            if (cmd_bits == 6'd47) begin
               in_cmd <= 1'b0;
               for (bi = 0; bi < 6; bi = bi + 1)
                 cmd[bi] = {cmd_sr[46:0], sd_mosi} >> (8 * (5 - bi));
               handle_command;
            end
         end
      end else begin
         in_cmd   <= 1'b0;
         cmd_bits <= 0;
      end
   end

   always @(negedge sd_clk) begin
      if (!sd_cs_n) begin
         skew_sr <= {skew_sr[6:0], out_byte[7]};
         if (bit_i == 3'd7) begin
            // A committing card holds MISO LOW for the whole busy period. The
            // queue carries the data response; once it drains, keep driving 0
            // until busy_left reaches 0. Without this MISO would read $FF
            // (ready) after 8 bytes while the card still ignored commands: an
            // inconsistent model that would hide the "next write into a busy
            // card" behavior.
            out_byte <= (busy_left > 0 && qhead == qtail) ? 8'h00 : qpop();
            bit_i    <= 3'd0;
         end else begin
            out_byte <= {out_byte[6:0], 1'b1};
            bit_i    <= bit_i + 3'd1;
         end
      end else begin
         bit_i    <= 3'd0;
         out_byte <= 8'hFF;
         skew_sr  <= 8'hFF;
      end
   end

endmodule

`default_nettype wire
