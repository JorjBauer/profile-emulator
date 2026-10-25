// reporter: report what happened, over the UART.
//
// The board has no other output channel: eight LEDs, and until the
// DB-25 is connected, no host.
//
// WHY THIS IS ALL HEX AND NOT LABELED TEXT. A readable status line
// ("rdy=1 ok=1 err=0 ...") is a wide character mux over registers
// scattered across the die. It is large and sits on the critical
// path. Every field it names keeps that register alive upstream. So
// the values are packed into a 256-bit bundle, loaded into a shift
// register once, and clocked out as hex digits: one register load
// instead of a mux per column. The field layouts and their labels
// are documented in ../README.md.
//
// It repeats on a timer rather than firing once, so a terminal
// attached after power-up still sees it.

`default_nettype none

module reporter #(
                  parameter integer CLK_HZ = 50_000_000,
                  // The 512-byte hex dump of the block buffer. Useful
                  // on a bare bench, wasted logic once the card is
                  // known good.
                  parameter         DUMP = 1'b1
                  ) (
                     input  wire        clk,
                     input  wire        rst,
                     input  wire        go,             // self-test finished
                     input  wire        sd_ready,
                     input  wire        sd_ok,
                     input  wire [3:0]  err_code,
                     input  wire [4:0]  dbg_state,
                     input  wire [5:0]  dbg_cmd,
                     input  wire [255:0] dbg_fat,
                     input  wire [255:0] dbg_txn,
                     input  wire [7:0]  dbg_fst,
                     input  wire [7:0]  dbg_retries,
                     // Identifies the RUNNING bitstream: minutes
                     // since 2026-01-01 UTC, injected at synthesis. It is
                     // reported on every SD line so a test round can
                     // confirm which build is actually running before
                     // trusting anything else it says.
                     input  wire [31:0] build_id,
                     input  wire [63:0] fs_cfg,       // first 8 bytes of profile.cfg as read
                     input  wire [7:0]  fs_plen,      // {have_cfg, parsed path length}
                     input  wire [31:0] fs_c1,        // chain[1] as walked
                     input  wire [31:0] fs_c2,        // chain[2] as walked
                     input  wire [31:0] fs_fatstart,  // fat32's OWN fat_start, not the self-test's
                     input  wire [31:0] fs_nclus,

                     output reg  [9:0]  rd_addr,        // read address into the block buffer
                     input  wire [7:0]  rd_data,
                     output wire        active,

                     output wire        tx
                     );

   localparam integer PERIOD = CLK_HZ * 3 / 2;  // repeat every 1.5 s

   reg [7:0]          ch;
   reg                ch_stb;
   wire               busy;
   uart_tx #(.CLK_HZ(CLK_HZ)) u_tx (
                                    .clk(clk), .rst(rst), .data(ch), .stb(ch_stb), .busy(busy), .tx(tx));

   function [7:0] hex(input [3:0] n);
      hex = (n < 4'd10) ? (8'h30 + n) : (8'h41 + n - 4'd10);
   endfunction

   // ---- the SD bundle ----
   // Deliberately SHORT. Every field named here keeps its source
   // register alive inside sd_spi, so a field costs logic all the way
   // upstream, not just the bits it occupies. Add one only if it
   // earns that. The field layout is documented in ../README.md.
   wire [255:0] w_sd = {
                        {7'h0, sd_ready, 7'h0, sd_ok, 4'h0, err_code, 3'h0, dbg_state},   // 0
                        {2'h0, dbg_cmd, dbg_retries, 4'h0, dbg_fst[7:4], dbg_fst[3:0]},   // 1
                        build_id,                                                         // 2
                        fs_cfg[63:32],                                                    // 3
                        fs_cfg[31:0],                                                     // 4
                        fs_fatstart,                                                      // 5
                        fs_nclus,                                                         // 6
                        {fs_plen, fs_c1[23:0]}};                                          // 7

   localparam [3:0] 
                    R_IDLE = 4'd0, R_LINE = 4'd1, R_ADDR = 4'd2, R_BYTE = 4'd3,
                    R_EOL  = 4'd4, R_TAIL = 4'd5, R_WAIT = 4'd6;

   reg [3:0]        st;
   reg [1:0]        line;        // 0 = SD, 1 = FAT, 2 = TXN
   reg [255:0]      sr;
   reg [3:0]        nib;         // nibbles emitted of the current word
   reg [6:0]        ci;
   reg [9:0]        bi;
   reg [1:0]        bs;
   reg [31:0]       timer;
   reg              seen_go;

   assign active = (st != R_IDLE) && (st != R_WAIT);

   // "SD  ", "FAT ", "TXN "
   function [7:0] tag(input [1:0] l, input [1:0] k);
      case ({l, k})
        4'b0000: tag = "S"; 4'b0001: tag = "D"; 4'b0010: tag = " "; 4'b0011: tag = " ";
        4'b0100: tag = "F"; 4'b0101: tag = "A"; 4'b0110: tag = "T"; 4'b0111: tag = " ";
        default: tag = (k == 2'd0) ? "T" : (k == 2'd1) ? "X"
        : (k == 2'd2) ? "N" : " ";
      endcase
   endfunction

   always @(posedge clk) begin
      ch_stb <= 1'b0;

      if (rst) begin
         st <= R_IDLE; ci <= 0; bi <= 0; bs <= 0; timer <= 0; seen_go <= 1'b0;
         line <= 0; nib <= 0; rd_addr <= 0;
      end else begin
         if (go) seen_go <= 1'b1;

         case (st)
           R_IDLE: if (seen_go) begin
              st <= R_LINE; ci <= 0; line <= 2'd0; sr <= w_sd; nib <= 0;
              ch <= "\n"; ch_stb <= 1'b1;
           end

           // Four label characters, then eight words of eight hex
           // digits each separated by spaces, then a newline. 4 + 72
           // + 2 = 78 characters.
           R_LINE: if (!busy && !ch_stb) begin
              if (ci < 7'd4) begin
                 ch <= tag(line, ci[1:0]);
              end else if (ci < 7'd76) begin
                 // A shift register, not a select on a 256-bit
                 // bus: see the header comment.
                 if (nib == 4'd8) begin
                    ch  <= " ";
                    nib <= 0;
                 end else begin
                    ch  <= hex(sr[255:252]);
                    sr  <= {sr[251:0], 4'h0};
                    nib <= nib + 4'd1;
                 end
              end else begin
                 ch <= (ci == 7'd76) ? "\r" : "\n";
              end
              ch_stb <= 1'b1;

              if (ci == 7'd77) begin
                 ci <= 0; nib <= 0;
                 case (line)
                   2'd0: begin line <= 2'd1; sr <= dbg_fat; end
                   2'd1: begin line <= 2'd2; sr <= dbg_txn; end
                   default: begin
                      if (DUMP && sd_ok) begin st <= R_ADDR; bi <= 0; end
                      else                     st <= R_TAIL;
                   end
                 endcase
              end else ci <= ci + 1'b1;
           end

           // "0000: "
           R_ADDR: if (!busy && !ch_stb) begin
              case (ci)
                7'd0: ch <= hex({2'b00, bi[9:8]});
                7'd1: ch <= hex(bi[7:4]);
                7'd2: ch <= hex(bi[3:0]);
                7'd3: ch <= ":";
                default: ch <= " ";
              endcase
              ch_stb <= 1'b1;
              if (ci == 7'd4) begin ci <= 0; st <= R_BYTE; bs <= 0; rd_addr <= bi; end
              else ci <= ci + 1'b1;
           end

           R_BYTE: if (!busy && !ch_stb) begin
              case (bs)
                2'd0: begin ch <= hex(rd_data[7:4]); bs <= 2'd1; end
                2'd1: begin ch <= hex(rd_data[3:0]); bs <= 2'd2; end
                default: begin
                   ch <= " ";
                   bs <= 2'd0;
                   if (bi[3:0] == 4'hF) st <= R_EOL;
                   else begin bi <= bi + 1'b1; rd_addr <= bi + 1'b1; end
                end
              endcase
              ch_stb <= 1'b1;
           end

           R_EOL: if (!busy && !ch_stb) begin
              ch <= (ci == 7'd0) ? "\r" : "\n";
              ch_stb <= 1'b1;
              if (ci == 7'd1) begin
                 ci <= 0;
                 if (bi == 10'd511) st <= R_TAIL;
                 else begin bi <= bi + 1'b1; rd_addr <= bi + 1'b1; st <= R_ADDR; end
              end else ci <= ci + 1'b1;
           end

           R_TAIL: if (!busy && !ch_stb) begin
              ch <= "\n"; ch_stb <= 1'b1; st <= R_WAIT; timer <= 0;
           end

           R_WAIT: if (timer == PERIOD[31:0]) st <= R_IDLE;
           else timer <= timer + 1'b1;

           default: st <= R_IDLE;
         endcase
      end
   end

endmodule

`default_nettype wire
