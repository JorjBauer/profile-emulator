// fat32: find an image file on the card and work out where its blocks
// live.
//
// THE CHAIN IS STORED, NOT ASSUMED CONTIGUOUS. Real world cards have
// fragmented chains, even on fresh formats and copies.
//
// The table is MAX_CLUSTERS entries. At 32 KB clusters a full 32 MB
// volume is 1024 of them, which is the default.
//
// WHICH IMAGE. The configuration file names the image by path, and
// the boot menu can select another by index. With AUTOPICK and no
// configuration, the largest .IMG or .HDV wins.
//
// Scanning is done on the byte stream as sd_spi lands it, never
// through the block buffer, so this cannot collide with the reporter
// or a host data phase.

`default_nettype none

module fat32 #(
               parameter integer MAX_CLUSTERS = 1024,
               // 8.3 name of the directory to search, space padded, no dot. All
               // spaces means "search the root itself".
               parameter [87:0]  DIR_NAME = "HD         ",
               // Pick the largest image automatically when nothing is
               // configured.
               parameter         AUTOPICK = 1'b0,
               // Walk subdirectories of DIR_NAME, not just DIR_NAME itself,
               // so a configured path may name an image below it.
               parameter         RECURSE = 1'b1,
               // 8.3 name of the configuration file in the root, space padded.
               // It holds a PATH, e.g. "/HD/name.img".
               parameter [87:0]  CFG_NAME = "PROFILE CFG"
               ) (
                  input  wire        clk,
                  input  wire        rst,
                  input  wire        start,
                  // The menu chose an image. Rescan the directory and
                  // serve the one at this index, which is how a
                  // SELECT takes effect without a reboot.
                  input  wire        reselect,
                  // High when the host bus has been quiet long enough
                  // that SD writes will not contend with a boot. The
                  // persist waits for it, because its flash-latency
                  // writes would otherwise take the SD engine from
                  // the block-0 read the //e issues one millisecond
                  // after its menu SELECT.
                  input  wire        host_quiet,
                  // A reselect with this high serves the new image
                  // but does NOT persist it: knock-menu selections
                  // are ephemeral, the boot default stays.
                  input  wire        nopersist,
                  input  wire [7:0]  resel_idx,

                  // SD request face. req is a LEVEL held until fire,
                  // the same handshake the verifier and the self-test
                  // use, because the engine only listens when idle.
                  output reg         req,
                  input  wire        fire,
                  input  wire        sd_done,
                  input  wire        sd_err,
                  input  wire        sd_buf_we,
                  input  wire [7:0]  sd_buf_wdata,
                  input  wire [9:0]  sd_buf_addr,   // which byte sd_spi wants while writing
                  output wire [7:0]  wr_data,       // ...and the byte to give it
                  output reg         req_wr,        // this request is a WRITE
                  output reg  [31:0] lba,

                  // Results.
                  output reg         ready,
                  output reg         ok,
                  output reg  [3:0]  err,
                  output reg  [31:0] image_blocks,
                  output reg  [31:0] data_start,
                  output reg  [3:0]  spc_log,
                  output reg  [31:0] nclusters,
                  // An image exists but nothing selected it. The
                  // selector turns this into a forced menu; nothing
                  // is served until a choice is made.
                  output reg         need_select,
                  // How many images the directory holds, and which
                  // one was selected. The menu needs both: with fewer
                  // than two images and a valid choice there is
                  // nothing to choose and the selector stays out of
                  // the way entirely.
                  output reg  [7:0]  img_count,
                  output reg  [7:0]  img_index,
                  // Walk diagnostics, exported to the reporter: the
                  // first two FAT-derived chain entries and the real
                  // fat_start, so a resolve can be checked against the
                  // card with no host attached, and what the resolver
                  // READ from profile.cfg (its first eight bytes and
                  // the parsed length), so an E_CFGBAD can be traced
                  // to the file's actual content.
                  output reg  [63:0] dbg_cfg,
                  output wire [7:0]  dbg_plen,
                  output reg  [31:0] dbg_c1,
                  output reg  [31:0] dbg_c2,
                  // Persist progress, so a persist that did not land
                  // can be diagnosed from the report rather than
                  // inferred from ok. [6:0] persisted length, [8] guard
                  // reached with persist set, [9] have_cfg, [10]
                  // found, [11] S_PBUILD entered, [12] empty-record
                  // abort, [13] data write fired, [14] data write
                  // done, [15] dirent write done, [23:16] index.
                  output reg  [31:0] dbg_persist,
                  output wire [31:0] dbg_fat_start,

                  // Catalog read port, for the menu. Records are 64
                  // bytes; see the layout in SELECTOR.md.
                  input  wire [11:0] cat_raddr,
                  output reg  [7:0]  cat_rdata,

                  // Chain lookup, one cycle of latency. Give it a
                  // cluster index, get the cluster number back.
                  input  wire [15:0] map_idx,
                  output reg  [31:0] map_cluster,

                  output wire [4:0]  dbg_state
                  );

   localparam [3:0]
                   E_NONE = 4'd0, E_MBR = 4'd1, E_BPB = 4'd2, E_NODIR = 4'd3,
                   E_NOIMG = 4'd4, E_CHAIN = 4'd5, E_SD = 4'd6, E_TOOBIG = 4'd7,
                   E_NOSEL = 4'd8,   // images found, but none is configured
                   E_NOCFG = 4'd9,   // no configuration file at all
                   E_CFGBAD = 4'd10; // the configured name matched nothing

   localparam [4:0] 
                    S_IDLE   = 5'd0,
                    S_MBR    = 5'd1,  S_MBR_W  = 5'd2,
                    S_BPB    = 5'd3,  S_BPB_W  = 5'd4,
                    S_CALC   = 5'd5,
                    S_DIR    = 5'd6,  S_DIR_W  = 5'd7,
                    S_DIRNXT = 5'd8,
                    S_SCAN   = 5'd9,  S_SCAN_W = 5'd10,
                    S_SCANNX = 5'd11,
                    S_CHAIN  = 5'd12, S_CHAIN_W= 5'd13, S_FCACHE = 5'd29, S_PWAIT = 5'd31,
                    S_FATRD  = 5'd14, S_FATRD_W= 5'd15,
                    S_DONE   = 5'd16, S_FAIL   = 5'd17, S_FATCALC = 5'd18,
                    S_CFG    = 5'd19, S_CFG_W  = 5'd20, S_PARSE   = 5'd21,
                    // Persisting the choice: build the line, write it, then fix
                    // the size in the directory entry.
                    S_PBUILD = 5'd22, S_PWR   = 5'd23, S_PWR_W  = 5'd24,
                    S_PDRD   = 5'd25, S_PDRD_W = 5'd26, S_PDWR  = 5'd27, S_PDWR_W = 5'd28,
                    S_PFIX   = 5'd30;

   reg [4:0]        st, ret;
   assign dbg_state = st;

   reg [9:0]  bi;            // byte index within the sector being streamed
   // FAT sector cache; see S_FATRD.
   reg [31:0] fat_cache_lba;
   reg        fat_cache_ok;
   reg [7:0]  fatcache [0:511];
   reg [8:0]  fcb_a, fcb_a2;
   reg [7:0]  fcb_d, fcb_q;
   reg        fcb_we;
   reg [1:0]  fcb_i, fcb_ph;
   always @(posedge clk) begin
      if (fcb_we) fatcache[fcb_a2] <= fcb_d;
      fcb_q <= fatcache[fcb_a];
   end
   reg [31:0] part_lba, fat_start, rsvd, fatsz, root_clus;
   assign dbg_fat_start = fat_start;
   reg [7:0]  nfat, spc;
   reg [15:0] bps;
   reg [7:0]  fat_left;      // FATs still to add when computing data_start

   // Per-entry accumulators while scanning a directory sector.
   reg        e_name_ok;     // matches DIR_NAME
   reg        e_del;         // entry starts with $E5: deleted, must not match
   // The TRUE length of the name currently in nbuf, and the copy of
   // it latched when a record burst starts. Every reader is bounded
   // by the latched length, so stale buffer content behind a shorter
   // name is unreadable rather than relying on it having been
   // blanked in time: the blanking writes share nb_a/nb_d/nb_we with
   // the NEXT entry's streaming LFN writes, and the stream wins any
   // same-cycle conflict.
   reg [5:0]  nb_len;
   reg [5:0]  cmt_len;
   // The commit walk's PRIVATE copy of the assembled name. Reading
   // nbuf directly would share its one address register and write
   // port with the LFN capture path, interlocked by nothing but a
   // timing margin. When the NEXT entry's long-name slices arrive
   // during the walk (fast SD, or an unlucky layout), the walk's
   // reads and the entry's nm_end both read garbage. That misjudges a
   // name's length and can bind the config to the wrong file.  The
   // snapshot costs 48 cycles inside the 21-byte grace window, and
   // the walk then never shares nbuf with the capture path.
   reg [7:0]  cbuf [0:47];
   reg [7:0]  cb_q;
   reg [5:0]  cb_wa, cb_ra;
   reg        cb_we;
   reg        snap_run;
   reg [5:0]  snap_i;
   // 8.3 compaction state: the on-disk form is NAME padded to 8 with
   // spaces then a 3-char extension. It is compacted at write time
   // (name, dot, extension) so nbuf holds the real filename and the
   // catalog, the configured-name match and persist all see it. One
   // nbuf write per streamed byte, each landing one byte-slot late:
   //   bytes 0-7   name chars at their compacted positions
   //   byte 8      the DOT at position n83n; extension char held
   //   bytes 9-10  previous held char written; current held
   //   byte 11     (attr, non-LFN path): the last held char
   reg [3:0]  n83n;         // compacted name length
   reg [7:0]  e83_hold;
   reg        e83_have;
   reg [5:0]  e83_wp;       // where the next held char goes
   reg [7:0]  e_attr;
   reg [31:0] e_clus, e_size;
   reg [7:0]  e_ext0, e_ext1, e_ext2;
   reg [6:0]  parse_i;
   // parse_i + 1, carried in its own register. The walk needs that
   // sum in three places, one of which feeds a subtract, and an
   // adder in front of the subtract chain would be the critical path
   // of the design.
   reg [6:0]  parse_i1;
   reg [1:0]  parse_ph;
   reg [5:0]  seq_cand;     // sequence number of the LFN entry being read
   // (seq_cand - 1) * 13 is the name-buffer slot where this LFN entry's
   // first character lands. We used to recompute that product in every
   // path that needs it. On an earlier TQFP144 build it was the slowest
   // path in the whole design (19.5 ns of a 20 ns period). Now it's
   // computed once, when the entry's sequence byte arrives, and the
   // character paths just add their slot to it. Same 7-bit wrap as the
   // expression it replaces, so nothing downstream changes.
   reg [6:0]  lfn_base;
   reg        is_lfn;       // ...confirmed once its attribute byte arrives

   // Which character of the name a byte offset within an LFN entry
   // carries.  13 UTF-16 characters, split across three runs by the
   // entry's layout.
   function [3:0] lfn_slot(input [4:0] o);
      case (o)
        5'd1:  lfn_slot = 4'd0;  5'd3:  lfn_slot = 4'd1;  5'd5:  lfn_slot = 4'd2;
        5'd7:  lfn_slot = 4'd3;  5'd9:  lfn_slot = 4'd4;  5'd14: lfn_slot = 4'd5;
        5'd16: lfn_slot = 4'd6;  5'd18: lfn_slot = 4'd7;  5'd20: lfn_slot = 4'd8;
        5'd22: lfn_slot = 4'd9;  5'd24: lfn_slot = 4'd10; 5'd28: lfn_slot = 4'd11;
        5'd30: lfn_slot = 4'd12; default: lfn_slot = 4'd15;   // not a character
      endcase
   endfunction
   // One pulse per directory entry, raised the cycle AFTER its last
   // byte so every field has settled. Judging on "bi[4:0]==0" instead
   // fires on every idle cycle between bytes, repeatedly, with
   // whatever is currently latched.
   reg        entry_end;

   reg [31:0] dir_clus;      // directory being scanned
   reg [31:0] cur_clus;      // cluster being read within that directory
   reg [7:0]  cur_sec;       // sector within the cluster
   reg [31:0] best_clus, best_size;
   reg        found;
   reg        scanning_root; // looking for DIR_NAME rather than for an image
   reg        dir_done;      // a directory sector ended with a $00 entry

   // FAT lookup.
   reg [31:0] fat_clus;      // cluster whose entry we want
   reg [31:0] fat_val;       // the answer
   reg [6:0]  fat_off;       // which entry within the sector
   // Precomputed from fat_val, one cycle after it arrives. Testing
   // for the end of the chain AND turning the cluster into an LBA in
   // the same cycle puts a 32-bit compare, a subtract, a variable
   // shift and an add on one path. The extra cycle happens once per
   // FAT sector read and costs nothing measurable.
   reg        fat_eoc;
   reg [31:0] fat_lba;

   // ---- the configured path ---------------------------------------
   // Read out of profile.cfg. The WHOLE path is matched: the commit
   // walk builds the prefix it is standing in as it descends, and a
   // candidate matches only when prefix + "/" + name equals this
   // buffer end to end (the length equality at the M_IMG test is what
   // enforces "end to end"). That is what lets a config name an image
   // in a subdirectory. A path naming a directory the scan never
   // enters therefore selects nothing, which is the safe answer.
   //
   // A whole sector, for persisting a selection. Writing profile.cfg
   // is easy. Updating its SIZE means read-modify-write of the
   // directory sector, because everything else in that sector belongs
   // to other files.
   reg [7:0]  secbuf [0:511];
   reg [8:0]  sec_a;
   reg [7:0]  sec_d;
   reg        sec_we;
   reg [7:0]  sec_q, sec_wq;
   // Two readers: sd_spi while a sector is being written, and the
   // record burst while it copies the path prefix out.
   always @(posedge clk) begin
      if (sec_we) secbuf[sec_a] <= sec_d;
      sec_wq <= secbuf[sd_buf_addr[8:0]];
      sec_q  <= secbuf[sec_a];
   end
   assign wr_data = sec_wq;

   reg [31:0] cfg_ent_lba;   // sector holding profile.cfg's directory entry
   reg [8:0]  cfg_ent_off;   // and where in it
   reg [8:0]  bld_i;
   reg [8:0]  pb_ns;      // persist copy: one past the last non-space written
   reg [8:0]  ent_off;
   reg [3:0]  bld_ph;

   // ---- descending into subdirectories -------------------------
   // Images on a card are commonly spread across several directories
   // and levels. A single-directory scan would show only some of
   // them, and a configured path naming any other directory would
   // resolve to nothing.
   //
   // The current path prefix lives in the sector buffer, which is
   // otherwise idle until a selection is persisted, and doubles as
   // the path to write when one is. The stack holds where to resume
   // in the parent: its cluster, its sector, and the entry to carry
   // on from, because a directory is rescanned from the start of a
   // sector and would otherwise find the same subdirectory forever.
   reg [31:0] stk_clus [0:2];
   reg [7:0]  stk_sec  [0:2];
   reg [8:0]  stk_off  [0:2];
   reg [6:0]  stk_pfx  [0:2];
   reg [1:0]  depth;
   reg [8:0]  skip_until;   // entries before this one were already scanned
   reg [6:0]  pfx_len;
   reg        cand_dir;     // this entry is a subdirectory worth descending into
   reg        desc_pend;    // ...and the sector it was found in has finished
   reg [31:0] desc_clus;
   reg [6:0]  desc_len;
   reg [8:0]  desc_off;
   reg        pfx_ini;      // writing "/DIR" into the prefix
   reg [3:0]  pfx_i;
   reg [5:0]  nm_end;       // one past the last non-space of the captured name
   reg        path_ok;      // the whole path so far equals the configured one
   // The first five long-name characters sit at offsets 1 to 9,
   // BEFORE the attribute byte that says whether this is a long-name
   // entry at all. Writing them into the name buffer speculatively would
   // corrupt it: an ordinary 8.3 entry whose first byte happens to
   // have bit 6 set reads as a sequence number and scatters its name
   // across the buffer. They are held here and only committed once
   // the entry is confirmed.
   reg [7:0]  lfn_c [0:4];
   reg [2:0]  lfn_wi;
   reg        lfn_wr;
   reg        ent_dot;      // "." or "..", which must never be descended into
   reg [6:0]  new_len;
   reg [5:0]  nm_i;
   reg        persist;       // a selection is waiting to be written down

   reg [7:0]  pbuf [0:63];
   reg [5:0]  pbuf_a;
   reg [7:0]  pbuf_q;
   reg        pbuf_we;
   reg [7:0]  pbuf_d;
   always @(posedge clk) begin
      if (pbuf_we) pbuf[pbuf_a] <= pbuf_d;
      pbuf_q <= pbuf[pbuf_a];
   end

   reg [6:0]  path_len;      // characters before the newline
   reg [6:0]  name_start;    // one past the last slash
   reg [6:0]  name_len;
   reg [31:0] cfg_clus;
   reg        have_cfg;
   assign dbg_plen = {have_cfg, path_len};

   // ---- long file name matching -----------------------------------
   // Most of the card's images are stored with long names and its
   // profile.cfg refers to them that way, so 8.3 matching alone would
   // not resolve a real config: the short name of "a2desktop.img" is
   // "A2DESK~7IMG". Files whose names already fit 8.3 have no LFN
   // chain at all (A2OSX.IMG). Those resolve through the compacted
   // short name instead. Both routes end in the same buffer.
   //
   // LFN entries sit immediately BEFORE their 8.3 entry, in reverse
   // sequence order, 13 UTF-16 characters each. Each character's
   // position in the name is (seq-1)*13 + slot, so it can be compared
   // against the target as it streams, with no need to assemble the
   // name anywhere.
   reg        lfn_seen;      // this entry had an LFN chain in front of it
   reg [1:0]  mode;          // which of the three scans is running

   localparam [1:0] M_CFG = 2'd0, M_BYIDX = 2'd1, M_IMG = 2'd2, M_ANY = 2'd3;

   // ASCII case fold, for comparing names.
   function [7:0] fold(input [7:0] c);
      fold = (c >= "A" && c <= "Z") ? (c + 8'd32) : c;
   endfunction

   // ---- the catalog -----------------------------------------------
   // One 64-byte record per image: flags, then 48 characters of name,
   // then the block count. The menu reads it a page at a time.
   //
   // Names are captured SPECULATIVELY into the slot the next image
   // would take, because an LFN chain arrives BEFORE the 8.3 entry
   // that says whether the thing is an image at all. Nothing is
   // committed until that entry is judged, and a candidate that turns
   // out not to be an image simply has its slot cleared and reused.
   reg [7:0]  catram [0:4095];
   wire [11:0] cat_ra_eff = (st == S_PBUILD)
               ? ({img_index[5:0], 6'd0} + 12'd2 + {6'd0, nm_i})
               : cat_raddr;
   reg [11:0]  cat_wa;
   reg [7:0]   cat_wd;
   reg         cat_we;
   always @(posedge clk) begin
      if (cat_we) catram[cat_wa] <= cat_wd;
      // The read port is MUXED, not doubled: a second read port on a
      // 32-kbit RAM does not infer as BRAM and lowers to a large
      // array of FFs. S_PBUILD borrows the port to copy the chosen record
      // out; the few microseconds an external reader would see
      // persist addresses land while the //e is confirm-polling
      // block 0, not paging the catalog.
      cat_rdata <= catram[cat_ra_eff];
   end

   wire [11:0] cat_base = {img_count[5:0], 6'd0};

   // The candidate's name, in a small RAM.
   //
   // A RAM rather than a register file, which would need a 48-to-1
   // read mux and indexed writes. A RAM cannot blank in one cycle,
   // so the blanking is done at BYTE 11 of the entry rather than at
   // entry_end. The attribute byte is there, and the extension came
   // at bytes 8 to 10, so by then it is known whether this entry is
   // an image. Byte 11 to the next entry's byte 1 is 21 byte times,
   // over 300 cycles. From entry_end to the next entry's first name
   // character is only 32 cycles at the fastest SD clock. So the
   // name is burst out and the buffer blanked at byte 11, and a RAM
   // is fast enough there.
   reg [7:0]   nbuf [0:63];
   reg [5:0]   nb_a;
   reg [7:0]   nb_d;
   reg         nb_we;
   reg [7:0]   nb_q;
   always @(posedge clk) begin
      if (nb_we) nbuf[nb_a] <= nb_d;
      nb_q <= nbuf[nb_a];
      if (cb_we) cbuf[cb_wa] <= nb_q;
      cb_q <= cbuf[cb_ra];
   end
   reg        cand_img;     // this entry is an image, decided at byte 11
   reg [1:0]  nph;          // burst phase: address, then copy
   reg [5:0]  clr_i;
   reg        clr_run;
   // Committing a record is five writes (the flags and a 32-bit block
   // count), so it gets its own little sequencer. The base is
   // captured before img_count moves on, or the record lands in the
   // next slot.
   reg [5:0]  cmt_i;
   reg        cmt_run;
   reg [11:0] cmt_base;
   reg [31:0] cmt_blocks;
   reg        cmt_sel;

   // The //e's text screen has no inverse lowercase: screen codes $00
   // to $3F are uppercase and symbols only, so a lowercase character
   // in a highlighted row renders as garbage. Names are folded up in
   // one place, the catalog write, but everything that reads the
   // catalog inherits the folded form: the menu, and the persist,
   // which copies the chosen record into profile.cfg. So the config
   // on the card is uppercase whatever the file's real name was. That
   // is fine here because the configured-name match folds both sides
   // (see fold() below); the true-case name survives only in nbuf and
   // nothing that persists reads it.
   function [7:0] upper(input [7:0] c);
      upper = (c >= "a" && c <= "z") ? (c - 8'd32) : c;
   endfunction

   // The chain. One entry per cluster of the image file.
   //
   // LRU-2 chain cache: the table holds TWO images' chains, addressed
   // by {slot, cluster_index}. An ephemeral reselect between two
   // recently-used images is then a slot switch with NO SD re-scan;
   // only a new image pays the directory walk + FAT fill,
   // and it evicts the least-recently-used slot. See the reselect
   // handling in S_DONE / S_FAIL. Scalar per-image state (index,
   // length, block count) is snapshotted into slot_* alongside the
   // chain.
   localparam integer CLW = $clog2(MAX_CLUSTERS);   // cluster-index width
   reg [31:0]         chain [0:2*MAX_CLUSTERS-1];
   reg [15:0]         chain_n;

   reg [7:0]          slot_img [0:1];   // which img_index each slot currently holds
   reg [15:0]         slot_n   [0:1];   // its chain length (clusters)
   reg [31:0]         slot_ib  [0:1];   // its image_blocks
   reg [1:0]          slot_valid;       // per-slot valid
   reg                active_slot;      // slot that answers map_cluster / is served now
   reg                fill_slot;        // slot the current resolve writes into
   reg                lru;              // least-recently-used slot: the evict target

   always @(posedge clk) map_cluster <= chain[{active_slot, map_idx[CLW-1:0]}];

   // spc is a power of two; keep its log so the multiplies become shifts.
   function [3:0] log2_8(input [7:0] v);
      casez (v)
        8'b1???????: log2_8 = 4'd7;  8'b01??????: log2_8 = 4'd6;
        8'b001?????: log2_8 = 4'd5;  8'b0001????: log2_8 = 4'd4;
        8'b00001???: log2_8 = 4'd3;  8'b000001??: log2_8 = 4'd2;
        8'b0000001?: log2_8 = 4'd1;  default:     log2_8 = 4'd0;
      endcase
   endfunction

   // Which 11-byte name this scan is looking for. The image scan
   // matches against the configured path instead, byte by byte out of
   // pbuf, so it is excluded.
   function [7:0] dir_byte(input [3:0] k);
      dir_byte = DIR_NAME[8*(10 - k) +: 8];
   endfunction

   function [7:0] target_byte(input [1:0] m, input [3:0] k);
      target_byte = (m == M_CFG) ? CFG_NAME[8*(10 - k) +: 8]
                    : DIR_NAME[8*(10 - k) +: 8];
   endfunction

   wire dir_is_root = (DIR_NAME == "           ");

   // First sector of a cluster.
   function [31:0] clus_lba(input [31:0] c);
      clus_lba = data_start + ((c - 32'd2) << spc_log);
   endfunction

   always @(posedge clk) begin
      if (rst) begin
         // Everything a condition below tests has to be reset, or it
         // is x after configuration and the first comparison goes
         // whichever way x goes.
         st <= S_IDLE; ret <= S_IDLE; req <= 1'b0;
         ready <= 1'b0; ok <= 1'b0; err <= E_NONE;
         image_blocks <= 0; chain_n <= 0; found <= 1'b0;
         slot_valid <= 2'b00; active_slot <= 1'b0; fill_slot <= 1'b0; lru <= 1'b1;
         best_clus <= 0; best_size <= 0; nclusters <= 0;
         data_start <= 0; spc_log <= 0; lba <= 0; bi <= 0;
         fat_left <= 0; dir_done <= 1'b0; scanning_root <= 1'b0;
         entry_end <= 1'b0; cur_sec <= 0; cur_clus <= 0; dir_clus <= 0;
         fat_clus <= 0; fat_val <= 0; fat_off <= 0; part_lba <= 0;
         fat_eoc <= 1'b0; fat_lba <= 0; need_select <= 1'b0;
         mode <= M_CFG; have_cfg <= 1'b0; path_len <= 0; name_start <= 0;
         name_len <= 0; cfg_clus <= 0; pbuf_we <= 1'b0; pbuf_a <= 0; pbuf_d <= 0;
         lfn_seen <= 1'b0;
         seq_cand <= 0; lfn_base <= 7'd115; is_lfn <= 1'b0; parse_i <= 0; parse_i1 <= 7'd1;
         parse_ph <= 2'd0;
         img_count <= 8'd0; img_index <= 8'd0;
         e_del <= 1'b0; nb_len <= 6'd0; cmt_len <= 6'd0;
         n83n <= 4'd0; e83_hold <= 8'd0; e83_have <= 1'b0; e83_wp <= 6'd0;
         dbg_c1 <= 32'hDEADDEAD; dbg_c2 <= 32'hDEADDEAD; dbg_cfg <= 64'h0;
         dbg_persist <= 32'd0;
         cat_we <= 1'b0; cat_wa <= 0; cat_wd <= 0; clr_i <= 0; clr_run <= 1'b0;
         cmt_i <= 0; cmt_run <= 1'b0; cmt_base <= 0; cmt_blocks <= 0;
         snap_run <= 1'b0; snap_i <= 6'd0; cb_wa <= 0; cb_ra <= 0; cb_we <= 1'b0;
         cand_img <= 1'b0; nph <= 2'd0; nb_a <= 0; nb_d <= 0; nb_we <= 1'b0;
         cmt_sel <= 1'b0; req_wr <= 1'b0; sec_we <= 1'b0; sec_a <= 0;
         sec_d <= 0; cfg_ent_lba <= 0; cfg_ent_off <= 0; bld_i <= 0;
         new_len <= 0; nm_i <= 0; persist <= 1'b0; ent_off <= 0; bld_ph <= 0;
         depth <= 2'd0; skip_until <= 9'd0; pfx_len <= 7'd0; cand_dir <= 1'b0;
         nm_end <= 6'd0; ent_dot <= 1'b0; desc_pend <= 1'b0; desc_clus <= 0;
         path_ok <= 1'b1; lfn_wi <= 3'd0; lfn_wr <= 1'b0;
         desc_len <= 0; desc_off <= 0; pfx_ini <= 1'b0; pfx_i <= 0;
         fat_start <= 0; rsvd <= 0; fatsz <= 0; root_clus <= 0;
         fat_cache_ok <= 1'b0; fcb_we <= 1'b0; fcb_i <= 2'd0; fcb_ph <= 2'd0;
         nfat <= 0; spc <= 0; bps <= 0;
      end else begin
         fcb_we <= 1'b0;   // cache-write strobe: one cycle, set only in S_FATRD_W
         // Any SD error at all is fatal here: every read this does is
         // structural, and guessing past a failed one produces a
         // plausible wrong answer.
         if (sd_err && st != S_IDLE && st != S_DONE && st != S_FAIL) begin
            err <= E_SD; st <= S_FAIL;
         end else case (st)

                    S_IDLE: if (start) begin
                       lba <= 32'd0; req <= 1'b1; bi <= 0; st <= S_MBR;
                    end

                    S_MBR: if (fire) begin req <= 1'b0; bi <= 0; st <= S_MBR_W; end

                    S_MBR_W: begin
                       if (sd_buf_we) begin
                          // Partition 1's start LBA lives at $1C6, little
                          // endian.
                          case (bi)
                            10'h1C6: part_lba[7:0]   <= sd_buf_wdata;
                            10'h1C7: part_lba[15:8]  <= sd_buf_wdata;
                            10'h1C8: part_lba[23:16] <= sd_buf_wdata;
                            10'h1C9: part_lba[31:24] <= sd_buf_wdata;
                            10'h1FE: if (sd_buf_wdata != 8'h55) err <= E_MBR;
                            10'h1FF: if (sd_buf_wdata != 8'hAA) err <= E_MBR;
                            default: ;
                          endcase
                          bi <= bi + 10'd1;
                       end
                       if (sd_done) begin
                          if (err != E_NONE) st <= S_FAIL;
                          else begin lba <= part_lba; req <= 1'b1; st <= S_BPB; end
                       end
                    end

                    S_BPB: if (fire) begin req <= 1'b0; bi <= 0; st <= S_BPB_W; end

                    S_BPB_W: begin
                       if (sd_buf_we) begin
                          case (bi)
                            10'h00B: bps[7:0]     <= sd_buf_wdata;
                            10'h00C: bps[15:8]    <= sd_buf_wdata;
                            10'h00D: spc          <= sd_buf_wdata;
                            10'h00E: rsvd[7:0]    <= sd_buf_wdata;
                            10'h00F: rsvd[15:8]   <= sd_buf_wdata;
                            10'h010: nfat         <= sd_buf_wdata;
                            10'h024: fatsz[7:0]   <= sd_buf_wdata;
                            10'h025: fatsz[15:8]  <= sd_buf_wdata;
                            10'h026: fatsz[23:16] <= sd_buf_wdata;
                            10'h027: fatsz[31:24] <= sd_buf_wdata;
                            10'h02C: root_clus[7:0]   <= sd_buf_wdata;
                            10'h02D: root_clus[15:8]  <= sd_buf_wdata;
                            10'h02E: root_clus[23:16] <= sd_buf_wdata;
                            10'h02F: root_clus[31:24] <= sd_buf_wdata;
                            default: ;
                          endcase
                          bi <= bi + 10'd1;
                       end
                       if (sd_done) begin
                          rsvd[31:16] <= 16'd0;
                          st <= S_CALC;
                       end
                    end

                    // data_start = part + rsvd + nfat * fatsz, with
                    // the multiply done as repeated addition: nfat is
                    // 1 or 2 in practice and a 32-bit multiplier
                    // costs more timing margin than this design has
                    // to spare.
                    S_CALC: begin
                       if (bps != 16'd512 || spc == 8'd0) begin
                          err <= E_BPB; st <= S_FAIL;
                       end else if (fat_left == 8'd0 && data_start == 32'd0) begin
                          fat_start  <= part_lba + rsvd;
                          data_start <= part_lba + rsvd;
                          spc_log    <= log2_8(spc);
                          fat_left   <= nfat;
                       end else if (fat_left != 8'd0) begin
                          data_start <= data_start + fatsz;
                          fat_left   <= fat_left - 8'd1;
                       end else begin
                          // Root first. If no subdirectory was
                          // configured, the root IS the directory to
                          // search for images. The config file is
                          // read before anything else: which image to
                          // serve is a decision the user made, not one
                          // this module gets to make.
                          dir_clus      <= root_clus;
                          cur_clus      <= root_clus;
                          cur_sec       <= 8'd0;
                          mode          <= M_CFG;
                          scanning_root <= 1'b0;
                          have_cfg      <= 1'b0;
                          found         <= 1'b0;
                          best_size     <= 0;
                          dir_done      <= 1'b0;
                          lba           <= clus_lba(root_clus);
                          req           <= 1'b1;
                          st            <= S_DIR;
                       end
                    end

                    // ---- reading one directory sector ----
                    S_DIR: if (pfx_ini) begin
                       // "/" then DIR_NAME without its padding, one character
                       // per cycle.
                       sec_we <= 1'b1;
                       if (pfx_i == 4'd0) begin
                          sec_a <= 9'd0; sec_d <= "/"; pfx_len <= 7'd1; pfx_i <= 4'd1;
                       end else if (pfx_i == 4'd12 || dir_byte(pfx_i - 4'd1) == " ") begin
                          sec_we  <= 1'b0;
                          pfx_ini <= 1'b0;
                       end else begin
                          sec_a   <= {2'd0, pfx_len};
                          sec_d   <= dir_byte(pfx_i - 4'd1);
                          pfx_len <= pfx_len + 7'd1;
                          pfx_i   <= pfx_i + 4'd1;
                       end
                    end else if (fire) begin req <= 1'b0; bi <= 0; st <= S_DIR_W; end

                    S_DIR_W: begin
                       entry_end <= (sd_buf_we && bi[4:0] == 5'd31);
                       pbuf_we   <= 1'b0;
                       cat_we    <= 1'b0;

                       // Blank the candidate slot's name field, one
                       // byte per cycle. Started when an entry is
                       // judged, which leaves two whole directory
                       // entries of streaming time before the next
                       // chain's characters arrive.
                       //
                       // The record burst: for an image the record
                       // shows the whole path, so it emits the prefix,
                       // then a separator, then the name; for a
                       // directory only the name matters, appended to
                       // the prefix itself. The name buffer is blanked
                       // behind whichever is read. The held long-name
                       // characters are committed here too, now that
                       // the entry is confirmed.
                       cb_we <= 1'b0;
                       if (snap_run) begin
                          // Pipelined copy through the SHARED nbuf
                          // read port, one byte per cycle (address,
                          // registered read, write trail two behind),
                          // so the whole snapshot takes ~50 cycles
                          // and always completes before the next
                          // entry's commit can retrigger it, even at
                          // simulation's fastest byte pace.  Cycle
                          // accounting (all nonblocking): the address
                          // assigned in cycle s reads in s+1 and
                          // registers into nb_q for s+2; a write
                          // strobe assigned in cycle s fires in s+1
                          // and captures nb_q as seen THEN. Net:
                          // assign cb_wa = snap_i-1 and the byte that
                          // lands is nbuf[snap_i-1] exactly. Being
                          // one register stage off copies nbuf[k+1]
                          // into cb[k] and runs the whole compare one
                          // character ahead.
                          nb_we <= 1'b0;
                          nb_a  <= snap_i;
                          snap_i <= snap_i + 6'd1;
                          if (snap_i >= 6'd1) begin
                             cb_we <= 1'b1;
                             cb_wa <= snap_i - 6'd1;
                          end
                          if (snap_i == 6'd48) begin
                             snap_run <= 1'b0;
                             cmt_run  <= 1'b1;
                          end
                       end else if (lfn_wr) begin
                          if (lfn_c[lfn_wi] != 8'h00) begin
                             nb_a  <= (lfn_base + {4'd0, lfn_wi});
                             nb_d  <= lfn_c[lfn_wi];
                             nb_we <= 1'b1;
                             if ((lfn_base + {4'd0, lfn_wi}) + 7'd1
                                 > {1'b0, nb_len})
                               nb_len <= (lfn_base
                                          + {4'd0, lfn_wi}) + 6'd1;
                          end
                          if (lfn_wi == 3'd4) lfn_wr <= 1'b0;
                          else lfn_wi <= lfn_wi + 3'd1;
                       end else if (cmt_run && cmt_i < 6'd48) begin
                          if (nph == 2'd0) begin
                             if (cand_img && cmt_i < pfx_len[5:0])
                               sec_a <= {3'd0, cmt_i};                 // from the prefix
                             else
                               cb_ra <= cand_img ? (cmt_i - pfx_len[5:0] - 6'd1) : cmt_i;
                             // The configured path, one character per
                             // output position. The burst emits the
                             // WHOLE path in order, so this is the
                             // natural place to compare it: matching
                             // on the name alone would make two
                             // images with the same basename in
                             // different directories
                             // indistinguishable.
                             pbuf_a <= {1'b0, cmt_i};
                             nb_we  <= 1'b0;
                             sec_we <= 1'b0;
                             nph    <= 2'd1;
                          end else if (nph == 2'd1) begin
                             nph <= 2'd2;                              // let the read settle
                          end else begin
                             if (cand_img) begin
                                cat_we <= 1'b1;
                                cat_wa <= cmt_base + 12'd2 + {6'd0, cmt_i};
                                // Folded HERE, the only fold. The
                                // persist copies this record out, so
                                // profile.cfg gets the folded name
                                // too; the match tolerates that
                                // because it folds both sides.
                                cat_wd <= (cmt_i < pfx_len[5:0]) ? upper(sec_q)
                                  : (cmt_i == pfx_len[5:0]) ? "/"
                                : (cand_img && (cmt_i - pfx_len[5:0] - 6'd1) >= cmt_len)
                                  ? " " : upper(cb_q);
                             end
                             if (cand_dir) begin
                                sec_a  <= {2'd0, pfx_len} + 9'd1 + {3'd0, cmt_i};
                                sec_d  <= cb_q;
                                sec_we <= 1'b1;
                             end
                             if (cb_q != " " && cb_q != 8'h00
                                 && (cand_dir || cmt_i > pfx_len[5:0])
                                 && (!cand_img || (cmt_i - pfx_len[5:0] - 6'd1) < cmt_len))
                               nm_end <= (cand_img ? (cmt_i - pfx_len[5:0] - 6'd1) : cmt_i) + 6'd1;
                             // Blank behind the copy. Every entry is
                             // blanked the same way, the menu's
                             // chosen one included: the persist
                             // (S_PBUILD) reads the folded catalog
                             // record, not nbuf, so there is no
                             // reason to keep any entry's true-case
                             // name here.
                             if (!(cand_img && cmt_i <= pfx_len[5:0])) begin
                                nb_a  <= cand_img ? (cmt_i - pfx_len[5:0] - 6'd1) : cmt_i;
                                nb_d  <= " ";
                                nb_we <= 1'b1;
                             end
                             if (cand_img && {1'b0, cmt_i} < path_len) begin
                                if (fold((cmt_i < pfx_len[5:0]) ? sec_q
                                         : (cmt_i == pfx_len[5:0]) ? "/"
                                         : ((cmt_i - pfx_len[5:0] - 6'd1) >= cmt_len) ? " "
                                         : cb_q) != fold(pbuf_q))
                                  path_ok <= 1'b0;
                             end
                             nph   <= 2'd0;
                             cmt_i <= cmt_i + 6'd1;
                          end
                       end else if (cmt_run) begin
                          cat_we <= 1'b1;
                          case (cmt_i - 6'd48)
                            6'd0: begin cat_wa <= cmt_base;            // flags
                               cat_wd <= {6'b0, cmt_sel, 1'b1}; end
                            6'd1: begin cat_wa <= cmt_base + 12'd50;   // block count, LE
                               cat_wd <= cmt_blocks[7:0]; end
                            6'd2: begin cat_wa <= cmt_base + 12'd51;
                               cat_wd <= cmt_blocks[15:8]; end
                            6'd3: begin cat_wa <= cmt_base + 12'd52;
                               cat_wd <= cmt_blocks[23:16]; end
                            default: begin cat_wa <= cmt_base + 12'd53;
                               cat_wd <= cmt_blocks[31:24]; end
                          endcase
                          if (cmt_i == 6'd52) cmt_run <= 1'b0;
                          else if (cmt_i >= 6'd48) cmt_i <= cmt_i + 6'd1;
                       end

                       // The configured-name comparison is NOT done
                       // here as characters stream past. The burst
                       // emits the whole path in order and compares
                       // it there, which is correct for two images
                       // sharing a basename and considerably smaller
                       // than a per-character comparison pipeline.
                       if (sd_buf_we) begin

                          // ---- LFN characters, compared as they stream ----
                          // NOT just "mode == M_IMG": that mode is
                          // selected before the directory itself has
                          // been found, so while scanning_root is
                          // still set this is walking the ROOT
                          // looking for DIR_NAME, and capturing names
                          // there would fill the catalog with the
                          // root's own entries.  Every scan of the image
                          // directory, not just the one matching a
                          // configured name: a menu selection
                          // re-scans in M_BYIDX and needs the names
                          // too, both for the catalog and to persist
                          // the choice.
                          if (mode != M_CFG && !scanning_root) begin
                             if (bi[4:0] == 5'd0) begin
                                seq_cand <= sd_buf_wdata[5:0];
                                lfn_base <= ({1'b0, sd_buf_wdata[5:0]} - 7'd1) * 7'd13;
                                is_lfn   <= 1'b0;
                             end
                             if (bi[4:0] == 5'd11) begin
                                is_lfn <= (sd_buf_wdata == 8'h0F);
                                if (sd_buf_wdata == 8'h0F) begin
                                   lfn_seen <= 1'b1;
                                   lfn_wi   <= 3'd0;      // now commit the held characters
                                   lfn_wr   <= 1'b1;
                                end else if (!dir_done) begin
                                   if (!lfn_seen && e83_have) begin
                                      nb_a <= e83_wp; nb_d <= e83_hold; nb_we <= 1'b1;
                                      nb_len <= e83_wp + 6'd1;
                                      e83_have <= 1'b0;
                                   end
                                   // ...but not for the
                                   // end-of-directory marker. That
                                   // entry is all zeros, so it is no
                                   // candidate, and running the burst
                                   // for it would blank the name of
                                   // the image that came just before
                                   // it: the name persisting a
                                   // selection needs.
                                   //
                                   // Everything needed to judge a
                                   // real entry is in hand here: the
                                   // attribute, and the extension
                                   // from bytes 8 to 10. Burst the
                                   // name and blank the buffer NOW,
                                   // with 21 byte times before the
                                   // next entry's characters arrive,
                                   // instead of at entry_end where
                                   // there are only two.
                                   cand_img <= ent_off >= skip_until && !e_del
                                               && (sd_buf_wdata & 8'h1A) == 8'h00
                                               && ((e_ext0 == "I" && e_ext1 == "M" && e_ext2 == "G") ||
                                                   (e_ext0 == "H" && e_ext1 == "D" && e_ext2 == "V"));
                                   // A directory worth descending
                                   // into. "." and ".." are not.
                                   cand_dir <= RECURSE && !e_del
                                               && (sd_buf_wdata & 8'h10) != 8'h00
                                               && (sd_buf_wdata & 8'h08) == 8'h00
                                               && !ent_dot && depth != 2'd3
                                               && mode != M_CFG      // profile.cfg is at the root
                                               && !scanning_root     // and so is DIR_NAME
                                               && ent_off >= skip_until;
                                   cmt_base <= cat_base;
                                   cmt_i    <= 6'd0;
                                   nph      <= 2'd0;
                                   nm_end   <= 6'd0;
                                   path_ok  <= 1'b1;
                                   snap_run <= 1'b1;     // snapshot nbuf, THEN walk
                                   snap_i   <= 6'd0;
                                   // The 8.3 path writes its LAST
                                   // extension character in this very
                                   // cycle (held from byte 10), so
                                   // the plain nb_len is one short
                                   // for it; account for the
                                   // in-flight write.
                                   cmt_len  <= (!lfn_seen && e83_have) ? (e83_wp + 6'd1)
                                     : nb_len;
                                   nb_len   <= 6'd0;     // the next entry accumulates afresh
                                   // The separator goes in ahead of
                                   // the name the burst is about to
                                   // append.
                                   if ((sd_buf_wdata & 8'h10) != 8'h00 && !ent_dot) begin
                                      sec_a  <= {2'd0, pfx_len};
                                      sec_d  <= "/";
                                      sec_we <= 1'b1;
                                   end
                                end
                             end
                             // The 8.3 name, for an entry with no
                             // long name in front of it. It goes into
                             // nbuf compacted, exactly as a long name
                             // would, so everything downstream (the
                             // menu, the configured-name match, the
                             // persisted path) sees one kind of name
                             // and no caller has to ask which form it
                             // arrived in.
                             if (!lfn_seen && bi[4:0] < 5'd8) begin
                                if (bi[4:0] == 5'd0) begin
                                   n83n <= 4'd0; e83_have <= 1'b0;
                                end
                                if (sd_buf_wdata != " ") begin
                                   nb_a   <= {2'b0, (bi[4:0] == 5'd0) ? 4'd0 : n83n};
                                   nb_d   <= sd_buf_wdata;
                                   nb_we  <= 1'b1;
                                   n83n   <= ((bi[4:0] == 5'd0) ? 4'd0 : n83n) + 4'd1;
                                   nb_len <= {2'b0, ((bi[4:0] == 5'd0) ? 4'd0 : n83n)} + 6'd1;
                                end
                             end else if (!lfn_seen && bi[4:0] < 5'd11) begin
                                if (bi[4:0] == 5'd8 && sd_buf_wdata != " ") begin
                                   nb_a <= {2'b0, n83n}; nb_d <= "."; nb_we <= 1'b1;
                                   nb_len   <= {2'b0, n83n} + 6'd1;
                                   e83_hold <= sd_buf_wdata; e83_have <= 1'b1;
                                   e83_wp   <= {2'b0, n83n} + 6'd1;
                                end else if (bi[4:0] > 5'd8) begin
                                   if (e83_have) begin
                                      nb_a <= e83_wp; nb_d <= e83_hold; nb_we <= 1'b1;
                                      nb_len <= e83_wp + 6'd1;
                                      e83_wp <= e83_wp + 6'd1;
                                   end
                                   e83_have <= (sd_buf_wdata != " ");
                                   e83_hold <= sd_buf_wdata;
                                end
                             end

                             // Offsets below 11 are compared
                             // PROVISIONALLY: whether this is an LFN
                             // entry at all is not known until its
                             // attribute byte, which comes after
                             // them.
                             if (lfn_slot(bi[4:0]) != 4'd15
                                 && (bi[4:0] < 5'd11 || is_lfn)) begin

                                // ...and record it, so the menu can
                                // show the long name.  A terminator
                                // or padding byte CLEARS the held
                                // slot rather than leaving it. lfn_c
                                // persists across entries, and the
                                // commit writes every non-zero slot,
                                // so a name whose final chunk is
                                // shorter than five characters would
                                // otherwise inherit the previous
                                // entry's tail characters.
                                if (bi[4:0] < 5'd11
                                    && (sd_buf_wdata == 8'h00 || sd_buf_wdata == 8'hFF))
                                  lfn_c[lfn_slot(bi[4:0])] <= 8'h00;
                                if (sd_buf_wdata != 8'h00 && sd_buf_wdata != 8'hFF
                                    && (lfn_base
                                        + {3'd0, lfn_slot(bi[4:0])}) < 7'd48) begin
                                   if (bi[4:0] < 5'd11) begin
                                      // slots 0 to 4 only, which is what
                                      // offsets 1 to 9 give
                                      lfn_c[lfn_slot(bi[4:0])] <= sd_buf_wdata;
                                   end else begin
                                      nb_a  <= (lfn_base
                                                + {3'd0, lfn_slot(bi[4:0])});
                                      nb_d  <= sd_buf_wdata;
                                      nb_we <= 1'b1;
                                      if ((lfn_base
                                           + {3'd0, lfn_slot(bi[4:0])}) + 7'd1 > {1'b0, nb_len})
                                        nb_len <= (lfn_base
                                                   + {3'd0, lfn_slot(bi[4:0])}) + 6'd1;
                                   end
                                end
                             end
                          end

                          // Start of an entry: reset the accumulators.
                          if (bi[4:0] == 5'd0) begin
                             ent_off   <= bi[8:0];      // where this entry starts
                             ent_dot   <= (sd_buf_wdata == ".");
                             e_name_ok <= 1'b1;
                             e_attr    <= 8'h00;
                             e_clus    <= 32'd0;
                             e_size    <= 32'd0;
                             // $00 here means no further entries exist in this
                             // directory.
                             if (sd_buf_wdata == 8'h00) dir_done <= 1'b1;
                          end

                          if (bi[4:0] < 5'd11) begin
                             if (sd_buf_wdata != target_byte(mode, bi[3:0])) e_name_ok <= 1'b0;
                             if (bi[4:0] == 5'd8)  e_ext0 <= sd_buf_wdata;
                             if (bi[4:0] == 5'd9)  e_ext1 <= sd_buf_wdata;
                             if (bi[4:0] == 5'd10) e_ext2 <= sd_buf_wdata;
                          end
                          case (bi[4:0])
                            // $E5 marks a DELETED entry. Its LFN
                            // chain and its 8.3 fields are otherwise
                            // intact, so without this flag a deleted
                            // image matches a configured name exactly
                            // like a live one, and since every match
                            // overwrites best_clus, the LAST match
                            // wins, deleted or not.
                            5'd0:  e_del         <= (sd_buf_wdata == 8'hE5);
                            5'd11: e_attr        <= sd_buf_wdata;
                            5'd20: e_clus[23:16] <= sd_buf_wdata;
                            5'd21: e_clus[31:24] <= sd_buf_wdata;
                            5'd26: e_clus[7:0]   <= sd_buf_wdata;
                            5'd27: e_clus[15:8]  <= sd_buf_wdata;
                            5'd28: e_size[7:0]   <= sd_buf_wdata;
                            5'd29: e_size[15:8]  <= sd_buf_wdata;
                            5'd30: e_size[23:16] <= sd_buf_wdata;
                            5'd31: begin
                               e_size[31:24] <= sd_buf_wdata;
                               // Decided on the LAST byte, when
                               // everything is in hand. Skip deleted
                               // entries ($E5), long-file-name
                               // fragments (attr $0F) and volume
                               // labels (attr bit 3).
                            end
                            default: ;
                          endcase
                          bi <= bi + 10'd1;
                       end

                       // One cycle after the entry's last byte, judge
                       // it.
                       //
                       // NOT the end-of-directory entry,
                       // though. cand_img is a register set at byte
                       // 11, and an all-zero entry never reaches byte
                       // 11 (dir_done is already set), so it would
                       // keep the PREVIOUS entry's verdict and match
                       // a second time, overwriting the resolved
                       // image with zeros.
                       if (entry_end && !dir_done) begin
                          if (mode == M_CFG) begin
                             if (e_name_ok && !e_del && (e_attr & 8'h1A) == 8'h00 && e_size != 0) begin
                                cfg_clus    <= e_clus;
                                have_cfg    <= 1'b1;
                                // Both halves of persisting a choice:
                                // the file's own sector, and the
                                // directory entry whose size field
                                // has to follow it.
                                cfg_ent_lba <= lba;
                                cfg_ent_off <= ent_off;
                             end
                          end else if (scanning_root) begin
                             if (e_name_ok && !e_del && (e_attr & 8'h10) != 8'h00
                                 && (e_attr & 8'h08) == 8'h00) begin
                                dir_clus <= e_clus;
                                found    <= 1'b1;
                             end
                          end else if (cand_dir) begin
                             // NOTE the sector is still streaming:
                             // sd_spi owns the bus until
                             // sd_done. Descending here would issue a
                             // request into a busy engine, so it is
                             // only recorded, and S_DIRNXT acts on
                             // it.
                             desc_pend <= 1'b1;
                             desc_clus <= e_clus;
                             desc_len  <= pfx_len + 7'd1 + {1'b0, nm_end};
                             desc_off  <= ent_off + 9'd32;
                          end else if (mode == M_BYIDX) begin
                             // Chosen from the menu by position, not by name.
                             if ((e_attr & 8'h1A) == 8'h00 && !e_del
                                 && ((e_ext0 == "I" && e_ext1 == "M" && e_ext2 == "G") ||
                                     (e_ext0 == "H" && e_ext1 == "D" && e_ext2 == "V"))) begin
                                if (img_count == resel_idx) begin
                                   best_clus <= e_clus;
                                   best_size <= e_size;
                                   found     <= 1'b1;
                                   img_index <= img_count;
                                end
                                if (img_count != 8'hFF) img_count <= img_count + 8'd1;
                             end
                          end else if (mode == M_IMG) begin
                             // The configured name, matched either
                             // through the long name or, for a file
                             // whose name really is 8.3, through the
                             // short one. BOTH WORK, and neither is
                             // tested for here: the comparison runs
                             // off nbuf, which the 8.3 compaction
                             // above fills with "A2OSX.IMG" for an
                             // entry that arrived as "A2OSX IMG" with
                             // no LFN chain, so by this point there
                             // is only one kind of name left. `make
                             // sim-fat32-83` is that case, with
                             // AUTOPICK off so that resolving at all
                             // IS the name match.
                             //
                             // The match needs three things. The full
                             // path must match AND end exactly where
                             // the config ends, or a config naming a
                             // longer path would match a prefix of it.
                             // And it must BE an image: path_ok is only
                             // meaningful for candidates (the walk
                             // compares only when cand_img), so a
                             // non-image whose name length happens to
                             // match keeps an untouched path_ok, and a
                             // .TAG sibling the same length as the .IMG
                             // would otherwise bind the config and
                             // serve the wrong file. M_BYIDX has this
                             // extension guard; the name match needs it
                             // too.
                             if ((e_attr & 8'h1A) == 8'h00 && !e_del && path_ok
                                 && ((e_ext0 == "I" && e_ext1 == "M" && e_ext2 == "G") ||
                                     (e_ext0 == "H" && e_ext1 == "D" && e_ext2 == "V"))
                                 && ({1'b0, pfx_len} + 8'd1 + {2'd0, nm_end}) == {1'b0, path_len}) begin
                                best_clus <= e_clus;
                                best_size <= e_size;
                                found     <= 1'b1;
                                img_index <= img_count;
                             end
                             // Count every image in the directory
                             // whether or not it is the configured
                             // one, and do NOT stop at the match: the
                             // menu has to know how many there are to
                             // decide whether there is a choice to
                             // offer at all.
                             if ((e_attr & 8'h1A) == 8'h00 && !e_del
                                 && ((e_ext0 == "I" && e_ext1 == "M" && e_ext2 == "G") ||
                                     (e_ext0 == "H" && e_ext1 == "D" && e_ext2 == "V"))
                                 && img_count != 8'hFF) begin
                                // The name is already in the record:
                                // the burst ran from byte 11, where
                                // the attribute and extension were
                                // both known. Only the flags and the
                                // size remain, and the size is not
                                // known until byte 31.
                                img_count  <= img_count + 8'd1;
                                cmt_blocks <= e_size[31:9];
                                cmt_sel    <= path_ok
                                              && ({1'b0, pfx_len} + 8'd1 + {2'd0, nm_end})
                                                == {1'b0, path_len};
                                cmt_i      <= 6'd48;
                                cmt_run    <= 1'b1;
                                // NOT blanked here: the commit below
                                // reads nbuf over the next 48 cycles,
                                // so blanking now copies spaces into
                                // the record.
                             end else if (!is_lfn) begin
                                // Judged and rejected. Blank in one
                                // cycle so a shorter name later
                                // cannot show the tail of a longer
                                // one.
                                cand_img <= 1'b0; nph <= 2'd0; nb_a <= 0; nb_d <= 0; nb_we <= 1'b0;
                             end
                          end else begin
                             // "IMG" or "HDV", a real file, and
                             // bigger than the best so far.
                             if (((e_ext0 == "I" && e_ext1 == "M" && e_ext2 == "G") ||
                                  (e_ext0 == "H" && e_ext1 == "D" && e_ext2 == "V"))
                                 && (e_attr & 8'h1A) == 8'h00 && !e_del
                                 && e_size > best_size) begin
                                best_clus <= e_clus;
                                best_size <= e_size;
                                found     <= 1'b1;
                                img_index <= img_count;
                             end
                             if (((e_ext0 == "I" && e_ext1 == "M" && e_ext2 == "G") ||
                                  (e_ext0 == "H" && e_ext1 == "D" && e_ext2 == "V"))
                                 && (e_attr & 8'h1A) == 8'h00 && img_count != 8'hFF)
                               img_count <= img_count + 8'd1;
                          end
                          // An LFN chain describes the 8.3 entry that
                          // FOLLOWS it, so the accumulators are
                          // cleared only once that entry has been
                          // judged, and in EVERY mode: clearing it
                          // only in the M_IMG branch would leave
                          // lfn_seen set through a reselect rescan,
                          // and every 8.3-only file after the first
                          // long-named one would lose its name in the
                          // rebuilt catalog.
                          if (!is_lfn) lfn_seen <= 1'b0;
                       end

                       if (sd_done) st <= S_DIRNXT;
                    end

                    // Next sector of this cluster, or follow the
                    // chain to the next cluster.
                    S_DIRNXT: begin
                       if (RECURSE && desc_pend) begin
                          // Down a level, now the sector has been
                          // read out. Where to resume in this
                          // directory is pushed first.
                          desc_pend       <= 1'b0;
                          stk_clus[depth] <= cur_clus;
                          stk_sec[depth]  <= cur_sec;
                          stk_off[depth]  <= desc_off;
                          stk_pfx[depth]  <= pfx_len;
                          depth      <= depth + 2'd1;
                          pfx_len    <= desc_len;
                          cur_clus   <= desc_clus;
                          cur_sec    <= 8'd0;
                          skip_until <= 9'd0;
                          dir_done   <= 1'b0;
                          lba        <= clus_lba(desc_clus);
                          req        <= 1'b1;
                          st         <= S_DIR;
                       end else if (mode == M_CFG && have_cfg) begin
                          // Found profile.cfg. Read it before deciding
                          // anything.
                          lba      <= clus_lba(cfg_clus);
                          req      <= 1'b1;
                          bi       <= 0;
                          pbuf_a   <= 6'd0;
                          path_len <= 7'd0;
                          st       <= S_CFG;
                       end else if (scanning_root && found) begin
                          // The directory was located: start scanning
                          // IT for images.
                          scanning_root <= 1'b0;
                          // Seed the path prefix with "/DIR" so it is
                          // a full path from the root all the way
                          // down. Persisting a choice is then an
                          // append, and the catalog shows where an
                          // image actually lives.
                          pfx_ini       <= 1'b1;
                          pfx_i         <= 4'd0;
                          // M_CFG got us here only via the AUTOPICK
                          // fallback; otherwise the configured name
                          // is what we are looking for.
                          mode          <= (mode == M_ANY) ? M_ANY : M_IMG;
                          found         <= 1'b0;
                          cur_clus      <= dir_clus;
                          cur_sec       <= 8'd0;
                          dir_done      <= 1'b0;
                          lba           <= clus_lba(dir_clus);
                          req           <= 1'b1;
                          st            <= S_DIR;
                       end else if (dir_done) begin
                          // Back up a level before deciding anything:
                          // a subdirectory running out of entries is
                          // not the scan running out of entries.
                          if (RECURSE && depth != 2'd0) begin
                             depth      <= depth - 2'd1;
                             cur_clus   <= stk_clus[depth - 2'd1];
                             cur_sec    <= stk_sec[depth - 2'd1];
                             skip_until <= stk_off[depth - 2'd1];
                             pfx_len    <= stk_pfx[depth - 2'd1];
                             dir_done   <= 1'b0;
                             lba        <= clus_lba(stk_clus[depth - 2'd1])
                               + {24'd0, stk_sec[depth - 2'd1]};
                             req        <= 1'b1;
                             st         <= S_DIR;
                          end else if (mode == M_CFG) begin
                             // No configuration. The menu is the
                             // answer, not a guess.
                             if (AUTOPICK) begin
                                mode          <= M_ANY;
                                scanning_root <= !dir_is_root;
                                found         <= 1'b0;
                                dir_done      <= 1'b0;
                                cur_clus      <= root_clus;
                                cur_sec       <= 8'd0;
                                lba           <= clus_lba(root_clus);
                                req           <= 1'b1;
                                st            <= S_DIR;
                             end else begin
                                need_select <= 1'b1;
                                err         <= E_NOCFG;
                                st          <= S_FAIL;
                             end
                          end else if (scanning_root) begin err <= E_NODIR; st <= S_FAIL; end
                          else if (!found && (mode == M_IMG || mode == M_BYIDX)) begin
                             // Configured (or chosen) but not
                             // present. A stale choice must not fall
                             // back to serving something else.
                             need_select <= 1'b1;
                             err         <= E_CFGBAD;
                             st          <= S_FAIL;
                          end
                          else if (!found)   begin err <= E_NOIMG; st <= S_FAIL; end
                          else begin
                             // Found the image. Walk its chain.
                             image_blocks <= best_size[31:9];
                             chain_n      <= 16'd0;
                             fat_clus     <= best_clus;
                             st           <= S_CHAIN;
                          end
                       end else if (cur_sec + 8'd1 < spc) begin
                          cur_sec <= cur_sec + 8'd1;
                          lba     <= lba + 32'd1;
                          req     <= 1'b1;
                          st      <= S_DIR;
                       end else begin
                          fat_clus <= cur_clus;
                          ret      <= S_SCANNX;
                          st       <= S_FATRD;
                       end
                    end

                    // The directory's next cluster came back in fat_val.
                    S_SCANNX: begin
                       if (fat_eoc) begin
                          // Back up a level before deciding anything:
                          // a subdirectory running out of entries is
                          // not the scan running out of entries.
                          if (RECURSE && depth != 2'd0) begin
                             depth      <= depth - 2'd1;
                             cur_clus   <= stk_clus[depth - 2'd1];
                             cur_sec    <= stk_sec[depth - 2'd1];
                             skip_until <= stk_off[depth - 2'd1];
                             pfx_len    <= stk_pfx[depth - 2'd1];
                             dir_done   <= 1'b0;
                             lba        <= clus_lba(stk_clus[depth - 2'd1])
                               + {24'd0, stk_sec[depth - 2'd1]};
                             req        <= 1'b1;
                             st         <= S_DIR;
                          end else if (mode == M_CFG) begin
                             if (AUTOPICK) begin
                                mode          <= M_ANY;
                                scanning_root <= !dir_is_root;
                                found         <= 1'b0;
                                dir_done      <= 1'b0;
                                cur_clus      <= root_clus;
                                cur_sec       <= 8'd0;
                                lba           <= clus_lba(root_clus);
                                req           <= 1'b1;
                                st            <= S_DIR;
                             end else begin
                                need_select <= 1'b1; err <= E_NOCFG; st <= S_FAIL;
                             end
                          end
                          else if (scanning_root) begin err <= E_NODIR; st <= S_FAIL; end
                          else if (!found && (mode == M_IMG || mode == M_BYIDX)) begin
                             need_select <= 1'b1; err <= E_CFGBAD; st <= S_FAIL;
                          end
                          else if (!found)   begin err <= E_NOIMG; st <= S_FAIL; end
                          else begin
                             image_blocks <= best_size[31:9];
                             chain_n      <= 16'd0;
                             fat_clus     <= best_clus;
                             st           <= S_CHAIN;
                          end
                       end else begin
                          cur_clus <= fat_val;
                          cur_sec  <= 8'd0;
                          lba      <= fat_lba;
                          req      <= 1'b1;
                          st       <= S_DIR;
                       end
                    end

                    // ---- walk and STORE the image's cluster chain ----
                    S_CHAIN: begin
                       if (fat_clus >= 32'h0FFFFFF8 || fat_clus < 32'd2) begin
                          nclusters <= {16'd0, chain_n};
                          ok        <= 1'b1;
                          // LRU-2: this image's chain is now fully in
                          // fill_slot. Tag the slot, make it the
                          // active/most-recently-used one, and mark
                          // the other slot the evict target. A later
                          // reselect back to this image is then free.
                          slot_img[fill_slot]   <= img_index;
                          slot_n[fill_slot]     <= chain_n;
                          slot_ib[fill_slot]    <= image_blocks;
                          slot_valid[fill_slot] <= 1'b1;
                          active_slot           <= fill_slot;
                          lru                   <= ~fill_slot;
                          // The gate is FOUND, not nm_end. nm_end is
                          // per-entry scratch: it is zeroed at every
                          // judgment and rewritten by every
                          // candidate, so after a scan it describes
                          // the LAST entry in the directory, not the
                          // chosen one. The name comes from the
                          // catalog record in S_PBUILD, which fails
                          // closed on an empty record rather than
                          // writing "/HD/" + nothing.
                          //
                          // READY FIRST, persist second. The persist
                          // is an SD write with flash-erase latency.
                          // A live //e re-reads block 0 right after
                          // its menu selection, and its confirm poll
                          // would run out against a resolver held
                          // busy by the write. Declared ready, the
                          // host proceeds immediately. The persist
                          // then runs as ordinary atomic SD traffic
                          // between its block requests, no worse than
                          // a long seek, and completes long before
                          // the next boot.
                          ready <= 1'b1;
                          if (persist) begin
                             dbg_persist[8]     <= 1'b1;
                             dbg_persist[9]     <= have_cfg;
                             dbg_persist[10]    <= found;
                             dbg_persist[23:16] <= img_index;
                          end
                          if (persist && have_cfg && found) begin
                             dbg_persist[11] <= 1'b1;
                             st     <= S_PWAIT;
                          end else begin
                             st    <= S_DONE;
                          end
                       end else if (chain_n == MAX_CLUSTERS[15:0]) begin
                          err <= E_TOOBIG; st <= S_FAIL;
                       end else begin
                          chain[{fill_slot, chain_n[CLW-1:0]}] <= fat_clus;
                          if (chain_n == 16'd1) dbg_c1 <= fat_clus;
                          if (chain_n == 16'd2) dbg_c2 <= fat_clus;
                          chain_n        <= chain_n + 16'd1;
                          ret            <= S_CHAIN_W;
                          st             <= S_FATRD;
                       end
                    end

                    // Idle-time means IDLE time: hold the persist
                    // until the host has gone quiet, so a boot's
                    // burst of reads never races it for the SD
                    // engine. ready is
                    // already high, so blocks serve normally while
                    // this waits. A reselect arriving here supersedes
                    // the pending persist (the new resolve will
                    // persist its own result).
                    S_PWAIT: begin
                       if (reselect) begin
                          persist <= 1'b0;
                          st      <= S_DONE;    // S_DONE fields the reselect next cycle
                       end else if (host_quiet) begin
                          bld_i  <= 9'd0;
                          bld_ph <= 4'd0;
                          nm_i   <= 6'd0;
                          st     <= S_PBUILD;
                       end
                    end

                    S_CHAIN_W: begin
                       fat_clus <= fat_val;
                       st       <= S_CHAIN;
                    end

                    // ---- read one FAT entry ----
                    //
                    // 128 entries per 512-byte sector, four bytes each.
                    //
                    // THE SECTOR IS CACHED. Without the cache this
                    // state would cost one full SD read PER CLUSTER:
                    // a 32 MB image is 1016 clusters, so a chain walk
                    // would take seconds, while a live //e that had
                    // just selected an image from the menu is
                    // re-reading block 0 with a ~2 s retry budget.
                    //
                    // Sequential clusters live 128 to a sector, so
                    // caching the last sector makes the walk ~8
                    // reads. That takes about as long as a seek,
                    // which the host is designed to wait for.
                    S_FATRD: begin
                       if (fat_cache_ok && fat_cache_lba == fat_start + (fat_clus >> 7)) begin
                          fat_off <= fat_clus[6:0];
                          fcb_i   <= 2'd0;
                          fcb_ph  <= 2'd0;
                          st      <= S_FCACHE;
                       end else begin
                          lba     <= fat_start + (fat_clus >> 7);
                          fat_off <= fat_clus[6:0];
                          req     <= 1'b1;
                          bi      <= 0;
                          st      <= S_FATRD_W;
                       end
                    end

                    // Four sync reads out of the cached sector
                    // assemble the entry. Plain three-phase per byte:
                    // address, settle, capture.
                    S_FCACHE: begin
                       if (fcb_ph == 2'd0) begin
                          fcb_a  <= {fat_off, fcb_i};
                          fcb_ph <= 2'd1;
                       end else if (fcb_ph == 2'd1) begin
                          fcb_ph <= 2'd2;
                       end else begin
                          case (fcb_i)
                            2'd0: fat_val[7:0]   <= fcb_q;
                            2'd1: fat_val[15:8]  <= fcb_q;
                            2'd2: fat_val[23:16] <= fcb_q;
                            2'd3: fat_val[31:24] <= {4'h0, fcb_q[3:0]};
                          endcase
                          fcb_ph <= 2'd0;
                          if (fcb_i == 2'd3) st <= S_FATCALC;
                          else fcb_i <= fcb_i + 2'd1;
                       end
                    end

                    S_FATRD_W: begin
                       if (fire) req <= 1'b0;
                       if (sd_buf_we) begin
                          fcb_a2 <= bi[8:0]; fcb_d <= sd_buf_wdata; fcb_we <= 1'b1;
                          if (bi[8:2] == fat_off) begin
                             case (bi[1:0])
                               2'd0: fat_val[7:0]   <= sd_buf_wdata;
                               2'd1: fat_val[15:8]  <= sd_buf_wdata;
                               2'd2: fat_val[23:16] <= sd_buf_wdata;
                               2'd3: fat_val[31:24] <= {4'h0, sd_buf_wdata[3:0]};
                             endcase
                          end
                          bi <= bi + 10'd1;
                       end
                       if (sd_done) begin
                          fat_cache_lba <= fat_start + (fat_clus >> 7);
                          fat_cache_ok  <= 1'b1;
                          st <= S_FATCALC;
                       end
                    end

                    S_FATCALC: begin
                       fat_eoc <= (fat_val >= 32'h0FFFFFF8) || (fat_val < 32'd2);
                       fat_lba <= data_start + ((fat_val - 32'd2) << spc_log);
                       st      <= ret;
                    end

                    // ---- read profile.cfg ----
                    S_CFG: if (fire) begin req <= 1'b0; bi <= 0; st <= S_CFG_W; end

                    S_CFG_W: begin
                       pbuf_we <= 1'b0;
                       if (sd_buf_we) begin
                          // Stop at the first terminator
                          if (bi < 10'd8) dbg_cfg <= {dbg_cfg[55:0], sd_buf_wdata};
                          if (bi < 10'd64 && path_len == 7'd0
                              && sd_buf_wdata != 8'h0A && sd_buf_wdata != 8'h0D
                              && sd_buf_wdata != 8'h00) begin
                             pbuf_a  <= bi[5:0];
                             pbuf_d  <= sd_buf_wdata;
                             pbuf_we <= 1'b1;
                          end else if (path_len == 7'd0) begin
                             path_len <= bi[6:0];        // length, terminator excluded
                          end
                          bi <= bi + 10'd1;
                       end
                       if (sd_done) begin
                          if (path_len == 7'd0) path_len <= 7'd64;
                          parse_i  <= 7'd0;
                          parse_i1 <= 7'd1;
                          parse_ph <= 2'd0;
                          name_start <= 7'd0;
                          st <= S_PARSE;
                       end
                    end

                    // Walk the path for the last slash, which is what splits
                    // the directory part from the file name.
                    //
                    // NOTE: name_start and name_len are set by this
                    // walk and NOTHING reads them: the match compares
                    // the full path (see pbuf above). tb_fat32
                    // prints them, and they are UNUSED otherwise.
                    S_PARSE: begin
                       // THREE phases, not two. pbuf is a registered
                       // read: the address presented in phase 0 is
                       // only latched at the next edge, so pbuf_q
                       // does not hold that byte until phase
                       // 2. Reading it in phase 1 gets the PREVIOUS
                       // index, one character early.
                       if (parse_ph == 2'd0) begin
                          pbuf_a   <= parse_i[5:0];
                          parse_ph <= 2'd1;
                       end else if (parse_ph == 2'd1) begin
                          parse_ph <= 2'd2;
                       end else begin
                          if (pbuf_q == "/") name_start <= parse_i1;
                          parse_ph <= 2'd0;
                          if (parse_i1 >= path_len) begin
                             // Done: set up the search for the directory, then
                             // the file.
                             name_len      <= path_len - ((pbuf_q == "/") ? parse_i1
                                                          : name_start);
                             mode          <= M_IMG;
                             scanning_root <= !dir_is_root;
                             found         <= 1'b0;
                             dir_done      <= 1'b0;
                             img_count     <= 8'd0;
                             lfn_seen      <= 1'b0;
                             cur_clus      <= root_clus;
                             cur_sec       <= 8'd0;
                             lba           <= clus_lba(root_clus);
                             req           <= 1'b1;
                             st            <= S_DIR;
                          end else begin
                             parse_i  <= parse_i1;
                             parse_i1 <= parse_i1 + 7'd1;
                          end
                       end
                    end

                    // A selection can arrive at any time after the
                    // first resolve, and also after a failure:
                    // "nothing was configured" is exactly the state
                    // the menu exists to get out of.
                    
                    // ---- write the choice into profile.cfg -------
                    // The line is built into the sector buffer a
                    // character at a time: "/" + the directory + "/"
                    // + the name + newline, then zeros. It is built
                    // from the CATALOG RECORD of the chosen image,
                    // never from nbuf/nm_end. The name buffer is
                    // per-entry scratch that every entry scanned
                    // AFTER the chosen one rewrites. The catalog
                    // record is stable (one slot per image, rebuilt
                    // whole on every scan) and already holds the
                    // complete "/HD/NAME.IMG" path. Its case is
                    // folded to upper for the //e's display. The
                    // resolve compare folds both sides, so the cfg
                    // still matches on next boot.
                    S_PBUILD: begin
                       sec_we <= 1'b0;
                       case (bld_ph)
                         4'd0: begin                    // aim the borrowed catalog port
                            nm_i <= 6'd0; bld_i <= 9'd0; pb_ns <= 9'd0; bld_ph <= 4'd1;
                         end
                         4'd1: bld_ph <= 4'd2;          // one cycle for the read to land
                         4'd2: begin
                            // Copy THROUGH spaces ("Total Replay
                            // v5.2.hdv" is a legal name) and
                            // remember where the last non-space
                            // landed; the newline goes
                            // there. Stopping at the first space
                            // would persist a truncated path for any
                            // name containing one.
                            if (cat_rdata == 8'h00 || nm_i == 6'd48) begin
                               bld_i <= pb_ns;            // trim the trailing padding
                               if (pb_ns == 9'd0) begin
                                  // An empty record is a corrupt
                                  // catalog. Persisting it would write
                                  // "/HD/" + nothing: fail closed,
                                  // keep the old file.
                                  dbg_persist[12] <= 1'b1;
                                  persist <= 1'b0;
                                  st      <= S_DONE;
                               end else bld_ph <= 4'd6;
                            end else begin
                               sec_a <= bld_i; sec_d <= cat_rdata; sec_we <= 1'b1;
                               if (cat_rdata != " ") pb_ns <= bld_i + 9'd1;
                               bld_i <= bld_i + 9'd1; nm_i <= nm_i + 6'd1; bld_ph <= 4'd1;
                            end
                         end
                         4'd6: begin                                  // newline, then zeros
                            sec_a <= bld_i; sec_d <= 8'h0A; sec_we <= 1'b1;
                            new_len <= bld_i[6:0] + 7'd1;
                            dbg_persist[6:0] <= bld_i[6:0] + 7'd1;
                            bld_i <= bld_i + 9'd1; bld_ph <= 4'd7;
                         end
                         default: begin
                            if (bld_i == 9'd511) begin
                               sec_a <= bld_i; sec_d <= 8'h00; sec_we <= 1'b1;
                               lba <= clus_lba(cfg_clus); req <= 1'b1; req_wr <= 1'b1;
                               st  <= S_PWR;
                            end else begin
                               sec_a <= bld_i; sec_d <= 8'h00; sec_we <= 1'b1;
                               bld_i <= bld_i + 9'd1;
                            end
                         end
                       endcase
                    end

                    S_PWR: if (fire) begin dbg_persist[13] <= 1'b1; req <= 1'b0; st <= S_PWR_W; end
                    S_PWR_W: if (sd_done) begin
                       dbg_persist[14] <= 1'b1;
                       // Now the directory entry's size, which needs
                       // the sector read back, patched and written
                       // again: everything else in it is other files.
                       req_wr <= 1'b0;
                       lba    <= cfg_ent_lba; req <= 1'b1; bi <= 0; st <= S_PDRD;
                    end

                    S_PDRD: if (fire) begin req <= 1'b0; bi <= 0; st <= S_PDRD_W; end
                    S_PDRD_W: begin
                       sec_we <= 1'b0;
                       if (sd_buf_we) begin
                          sec_a <= bi[8:0]; sec_d <= sd_buf_wdata; sec_we <= 1'b1;
                          bi <= bi + 10'd1;
                       end
                       if (sd_done) begin bld_ph <= 4'd0; st <= S_PDWR; end
                    end

                    S_PDWR: begin
                       // Four bytes of size, little endian, at offset 28 of the
                       // entry.
                       sec_we <= 1'b1;
                       case (bld_ph)
                         4'd0: begin sec_a <= cfg_ent_off + 9'd28; sec_d <= {1'b0, new_len}; end
                         4'd1: begin sec_a <= cfg_ent_off + 9'd29; sec_d <= 8'h00; end
                         4'd2: begin sec_a <= cfg_ent_off + 9'd30; sec_d <= 8'h00; end
                         default: begin sec_a <= cfg_ent_off + 9'd31; sec_d <= 8'h00; end
                       endcase
                       if (bld_ph == 4'd3) begin
                          lba <= cfg_ent_lba; req <= 1'b1; req_wr <= 1'b1; st <= S_PDWR_W;
                       end else bld_ph <= bld_ph + 4'd1;
                    end

                    S_PDWR_W: begin
                       sec_we <= 1'b0;
                       if (fire) req <= 1'b0;
                       if (sd_done) begin
                          dbg_persist[15] <= 1'b1;
                          req_wr  <= 1'b0;
                          persist <= 1'b0;
                          ready   <= 1'b1;
                          nm_i    <= 6'd0;
                          st      <= S_PFIX;
                       end
                    end

                    // S_PDRD filled secbuf with the cfg's DIRECTORY
                    // SECTOR, destroying the "/HD" path prefix that
                    // every later catalog rebuild and every later
                    // persist copies from secbuf[0..pfx_len-1]. Left
                    // unrebuilt, the prefix is whatever sector bytes
                    // happen to be sitting there, so every menu entry
                    // shows a corrupted path and the next persist
                    // writes it into profile.cfg. Rebuild the prefix
                    // from DIR_NAME before returning to service.
                    S_PFIX: begin
                       if (dir_is_root || nm_i > 6'd11) begin
                          sec_we <= 1'b0;
                          st <= S_DONE;
                       end else if (nm_i == 6'd0) begin
                          sec_a <= 9'd0; sec_d <= "/"; sec_we <= 1'b1; nm_i <= 6'd1;
                       end else if (dir_byte(nm_i[3:0] - 4'd1) != " ") begin
                          sec_a <= {3'd0, nm_i}; sec_d <= dir_byte(nm_i[3:0] - 4'd1);
                          sec_we <= 1'b1; nm_i <= nm_i + 6'd1;
                       end else begin
                          sec_we <= 1'b0;
                          st <= S_DONE;
                       end
                    end

                    // Re-selecting the image that is ALREADY resolved
                    // is free, and it is also the case that matters:
                    // every unattended boot auto-selects the
                    // remembered image, so resel_idx == img_index
                    // every time nobody is at the keyboard. The full
                    // path below re-scans the directory and then
                    // PERSISTS profile.cfg (an SD write, with
                    // flash-erase latency) all while the //e is
                    // waiting in its confirm poll for the post-select
                    // block-0 read. The host's confirm poll lasts on
                    // the order of 100 us and then gives up, so a
                    // rescan there loses the boot outright. The fast
                    // path keeps ready HIGH, so the router never
                    // parks the read and the confirm answers
                    // immediately. A NEW selection still
                    // pays the rescan and persist, but that only
                    // happens with a human at the menu, who takes
                    // seconds to press the key anyway.
                    S_DONE: if (reselect && ok && resel_idx == img_index) begin
                       st <= S_DONE;                       // nothing to do: already serving it
                    end else if (reselect && nopersist && slot_valid[0]
                                 && slot_img[0] == resel_idx) begin
                       // LRU-2 cache hit, slot 0: switch and restore
                       // the scalar state, no SD access at all. Only
                       // ephemeral (knock) selects take this fast
                       // path; a persisting select still re-resolves
                       // so profile.cfg is rewritten.
                       active_slot <= 1'b0; lru <= 1'b1;
                       img_index <= resel_idx; chain_n <= slot_n[0];
                       nclusters <= {16'd0, slot_n[0]}; image_blocks <= slot_ib[0];
                       ok <= 1'b1; ready <= 1'b1; err <= E_NONE; need_select <= 1'b0;
                       st <= S_DONE;
                    end else if (reselect && nopersist && slot_valid[1]
                                 && slot_img[1] == resel_idx) begin
                       // LRU-2 cache hit, slot 1.
                       active_slot <= 1'b1; lru <= 1'b0;
                       img_index <= resel_idx; chain_n <= slot_n[1];
                       nclusters <= {16'd0, slot_n[1]}; image_blocks <= slot_ib[1];
                       ok <= 1'b1; ready <= 1'b1; err <= E_NONE; need_select <= 1'b0;
                       st <= S_DONE;
                    end else if (reselect) begin
                       // Miss (or a persisting select): full rescan,
                       // into the LRU slot so the currently-active
                       // slot survives for a fast switch back.
                       fill_slot <= lru;
                       mode <= M_BYIDX; found <= 1'b0; dir_done <= 1'b0; img_count <= 8'd0;
                       scanning_root <= 1'b0; cur_clus <= dir_clus; cur_sec <= 8'd0;
                       lfn_seen <= 1'b0; nb_len <= 6'd0;   // no residue from the last scan
                       ok <= 1'b0; ready <= 1'b0; err <= E_NONE; need_select <= 1'b0;
                       persist <= !nopersist;   // knock selections stay ephemeral
                       lba <= clus_lba(dir_clus); req <= 1'b1; st <= S_DIR;
                    end

                    S_FAIL: if (reselect && dir_clus != 32'd0) begin
                       fill_slot <= lru;                   // rescan into the evict slot
                       mode <= M_BYIDX; found <= 1'b0; dir_done <= 1'b0; img_count <= 8'd0;
                       scanning_root <= 1'b0; cur_clus <= dir_clus; cur_sec <= 8'd0;
                       lfn_seen <= 1'b0; nb_len <= 6'd0;   // no residue from the last scan
                       ok <= 1'b0; ready <= 1'b0; err <= E_NONE; need_select <= 1'b0;
                       persist <= !nopersist;   // knock selections stay ephemeral
                       lba <= clus_lba(dir_clus); req <= 1'b1; st <= S_DIR;
                    end else begin
                       ready <= 1'b1;
                       ok    <= 1'b0;
                    end

                    default: st <= S_IDLE;
                  endcase
      end
   end

endmodule

`default_nettype wire
