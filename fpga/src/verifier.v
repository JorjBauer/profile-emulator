// verifier: read a range of the card and say, over the UART, what is in it.
//
// The SD stack can bring a card up and read single sectors, but "a sector was
// read and it had $55 $AA in it" is not the same claim as "every block this
// drive will ever serve matches the image". The only way to make the second
// claim is to read the whole range and compare it against the same bytes read
// on a machine that is known to be right. This module feeds that comparison.
//
// It emits one line per checkpoint:
//
//     000000:1D0F
//     000001:8C2B
//     #03             <- retries used during the whole run (hex)
//     000002 7FFFF    <- a hex digit instead of ":" is the sd_spi error
//                        code for a read that FAILED ($7 = no data token)
//
// six hex digits of starting LBA, a colon, four hex digits of CRC16-CCITT over
// every byte in that checkpoint, and a newline. tools/verify_image.js computes
// the same thing from an image file and reports the first LBA that differs.
//
// STRIDE exists because the whole card is not the same job as the served
// volume. At one line per sector, 65535 blocks is about 790 KB of serial and a
// minute or so, which is the right way to check the 32MB a host can actually
// reach. A 4 GB card is 8.4 million sectors, and at one line each the UART, not
// the SD card, becomes the limit. Raising STRIDE folds many sectors into one
// CRC so the range can be swept coarsely first and narrowed afterward.
//
// The CRC is computed on the fly from the bytes as sd_spi lands them in the
// block buffer, so this never touches the buffer's read port and cannot collide
// with the reporter or with a host data phase.

`default_nettype none

module verifier #(
                  parameter integer CLK_HZ = 50_000_000,
                  parameter integer BAUD   = 115_200,
                  parameter [31:0]  START  = 32'd0,
                  parameter [31:0]  COUNT  = 32'd65536,
                  parameter [31:0]  STRIDE = 32'd1
                  ) (
                     input  wire        clk,
                     input  wire        rst,
                     input  wire        enable,        // build-time switch, so it costs nothing when off
                     input  wire        go,            // the self-test finished and the card is good

                     // SD side. req is a LEVEL, held until fire acknowledges
                     // it, because the engine is only listening while it is
                     // idle and a one-cycle pulse aimed at a busy engine is
                     // simply lost.
                     output reg         req,
                     input  wire        fire,
                     input  wire        sd_done,
                     input  wire        sd_err,
                     input  wire [3:0]  sd_err_code,
                     input  wire [7:0]  sd_retries,   // cumulative, for the summary line
                     input  wire        sd_buf_we,
                     input  wire [7:0]  sd_buf_wdata,
                     output reg  [31:0] lba,

                     output wire        active,        // suppress the reporter while this runs
                     output wire        done,
                     output wire        tx
                     );

   reg  [7:0] ch;
   reg        ch_stb;
   wire       busy;
   uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_tx (
                                                 .clk(clk), .rst(rst), .data(ch), .stb(ch_stb), .busy(busy), .tx(tx));

   function [7:0] hex(input [3:0] n);
      hex = (n < 4'd10) ? (8'h30 + n) : (8'h41 + n - 4'd10);
   endfunction

   localparam [2:0]
                   V_IDLE = 3'd0,
                   V_REQ  = 3'd1,   // asking for a sector
                   V_RUN  = 3'd2,   // sector in flight, CRC accumulating
                   V_DRAIN= 3'd3,   // last byte still going through the CRC
                   V_EMIT = 3'd4,   // twelve characters out of the UART
                   V_SUM  = 3'd6,   // "#NN": how many transfers were re-issued
                   V_DONE = 3'd5;

   reg [2:0]       st;
   reg [31:0]      left;        // sectors still to read in this run
   reg [31:0]      instride;    // sectors folded into the CRC so far
   reg [31:0]      ck_lba;      // LBA this checkpoint started at
   reg [3:0]       ei;          // which of the twelve characters
   reg             ck_err;      // a read in this checkpoint failed
   reg [3:0]       ck_code;     // and which error it was

   // ---- CRC16-CCITT, one bit per clock -------------------------------------
   // Deliberately bit serial instead of a combinational byte step. A byte
   // arrives every 48 clocks at the fastest SD rate this board runs, which is
   // far more time than eight bit steps need. An eight-deep XOR chain would
   // sit on the timing path for no gain.
   reg [15:0]      crc;
   reg [7:0]       bsr;
   reg [3:0]       bcnt;
   reg             crc_clr;            // one-cycle pulse from the FSM below
   wire            crc_busy = (bcnt != 4'd0);

   // crc is driven from HERE ONLY. The state machine below asks for a reset by
   // pulsing crc_clr rather than assigning crc itself: two always blocks
   // driving one reg is not legal Verilog, and iverilog is the only tool in
   // this flow that reliably says so.
   always @(posedge clk) begin
      if (rst || crc_clr) begin
         crc <= 16'hFFFF; bsr <= 8'h00; bcnt <= 4'd0;
      end else if (sd_buf_we) begin
         bsr  <= sd_buf_wdata;
         bcnt <= 4'd8;
      end else if (crc_busy) begin
         crc  <= (crc[15] ^ bsr[7]) ? ((crc << 1) ^ 16'h1021) : (crc << 1);
         bsr  <= {bsr[6:0], 1'b0};
         bcnt <= bcnt - 4'd1;
      end
   end

   assign active = enable && (st != V_IDLE) && (st != V_DONE);
   assign done   = (st == V_DONE);

   // The twelve characters of a line, spliced from the two values.
   function [7:0] line_ch(input [3:0] i, input [31:0] a, input [15:0] c, input e, input [3:0] c2);
      case (i)
        4'd0: line_ch = hex(a[23:20]);  4'd1: line_ch = hex(a[19:16]);
        4'd2: line_ch = hex(a[15:12]);  4'd3: line_ch = hex(a[11:8]);
        4'd4: line_ch = hex(a[7:4]);    4'd5: line_ch = hex(a[3:0]);
        // ":" is a clean checkpoint. Anything else is the sd_spi error code as
        // a single hex digit, so the diagnosis comes off the wire instead of
        // being guessed at from a checksum.
        4'd6: line_ch = e ? ((c2 < 4'd10) ? (8'h30 + c2) : (8'h41 + c2 - 4'd10))
          : ":";
        4'd7: line_ch = hex(c[15:12]);  4'd8: line_ch = hex(c[11:8]);
        4'd9: line_ch = hex(c[7:4]);    4'd10: line_ch = hex(c[3:0]);
        default: line_ch = "\n";
      endcase
   endfunction

   always @(posedge clk) begin
      ch_stb  <= 1'b0;
      crc_clr <= 1'b0;

      if (rst) begin
         st <= V_IDLE; req <= 1'b0; lba <= START; left <= COUNT;
         instride <= 0; ck_lba <= START; ei <= 0; ck_err <= 1'b0; ck_code <= 4'd0;
      end else case (st)

                 V_IDLE: if (enable && go) begin
                    crc_clr  <= 1'b1;
                    lba      <= START;
                    ck_lba   <= START;
                    left     <= COUNT;
                    instride <= 0;
                    ck_err   <= 1'b0;
                    ck_code  <= 4'd0;
                    req      <= 1'b1;
                    st       <= V_REQ;
                 end

                 V_REQ: if (fire) begin
                    req <= 1'b0;
                    st  <= V_RUN;
                 end

                 V_RUN: if (sd_done) begin
                    // A read that failed still has to produce a line, or the
                    // host script compares the wrong LBAs against each other
                    // from there on. Say so EXPLICITLY rather than leaving it
                    // to be inferred from the checksum. A failed read leaves
                    // the CRC at its $FFFF seed. $FFFF is also a perfectly
                    // ordinary checksum for real data (over 65536 sectors,
                    // about one hashes to it by chance). So "CRC is $FFFF"
                    // and "the read failed" are different claims, and the
                    // line has to distinguish them.
                    if (sd_err) begin ck_err <= 1'b1; ck_code <= sd_err_code; end
                    st <= V_DRAIN;
                 end

                 // Let the last byte finish going through the bit-serial CRC
                 // before the value is read out. Without this the final byte of
                 // every sector is missing from the checksum, which still
                 // produces stable, repeatable, completely wrong numbers.
                 V_DRAIN: if (!crc_busy) begin
                    if (instride + 32'd1 >= STRIDE || left <= 32'd1) begin
                       ei <= 0;
                       st <= V_EMIT;
                    end else begin
                       instride <= instride + 32'd1;
                       lba      <= lba + 32'd1;
                       left     <= left - 32'd1;
                       req      <= 1'b1;
                       st       <= V_REQ;
                    end
                 end

                 V_EMIT: if (!busy && !ch_stb) begin
                    ch     <= line_ch(ei, ck_lba, crc, ck_err, ck_code);
                    ch_stb <= 1'b1;
                    if (ei == 4'd11) begin
                       // line finished: reset the CRC and move to the next
                       // checkpoint
                       crc_clr  <= 1'b1;
                       instride <= 0;
                       ck_err   <= 1'b0;
                       ck_code  <= 4'd0;
                       if (left <= 32'd1) begin
                          ei <= 4'd0;
                          st <= V_SUM;
                       end else begin
                          lba    <= lba + 32'd1;
                          ck_lba <= lba + 32'd1;
                          left   <= left - 32'd1;
                          req    <= 1'b1;
                          st     <= V_REQ;
                       end
                    end else ei <= ei + 4'd1;
                 end

                 // A retry that succeeded leaves no trace in the per-sector
                 // lines. That is the point of retrying. But "it passed" and
                 // "it passed first time" are different claims about the
                 // hardware, and the run should say which.
                 V_SUM: if (!busy && !ch_stb) begin
                    case (ei)
                      4'd0: ch <= "#";
                      4'd1: ch <= (sd_retries[7:4] < 4'd10) ? (8'h30 + sd_retries[7:4])
                        : (8'h41 + sd_retries[7:4] - 4'd10);
                      4'd2: ch <= (sd_retries[3:0] < 4'd10) ? (8'h30 + sd_retries[3:0])
                        : (8'h41 + sd_retries[3:0] - 4'd10);
                      default: ch <= "\n";
                    endcase
                    ch_stb <= 1'b1;
                    if (ei == 4'd3) st <= V_DONE;
                    else            ei <= ei + 4'd1;
                 end

                 V_DONE: ;

                 default: st <= V_IDLE;
               endcase
   end

endmodule

`default_nettype wire
