// spi_flash: the few W25Q128JV commands the firmware updater needs, over
// the configuration flash's own pins once the FPGA is running.
//
// After configuration, /CS, MOSI and MISO are ordinary I/O, and the clock
// pin is reached through the ECP5's USRMCLK primitive (in top). SPI mode 0
// at a quarter of the system clock: 12.5 MHz at 50 MHz, far under the
// part's 50 MHz limit for the plain 03h read.
//
//   OP_ID      9Fh, the three JEDEC ID bytes into `jedec`
//   OP_ERASE   06h then D8h, a 64 KB block erase, then poll until done
//   OP_PROG    06h then 02h, program `len` bytes (at most 256, inside one
//              page) fetched through idx/wdata, then poll until done
//   OP_READ    03h, `len` bytes (at most 512) out through rd_stb/rdata
//
// Polling gives up after POLL_TIMEOUT_MS and reports !ok. The data sheet's
// worst case is 2 s for a 64 KB erase and 3 ms for a page.
// W25Q128JV data sheet, sections 8.2.6, 8.2.13, 8.2.27 and 8.2.29.
//
// wdata is expected one clock after idx changes (a registered block RAM
// read). This waits two clocks, so either works.

`default_nettype none

module spi_flash #(
                   parameter integer CLK_HZ = 50_000_000,
                   parameter integer POLL_TIMEOUT_MS = 3000
                   ) (
                      input  wire        clk,
                      input  wire        rst,

                      input  wire [2:0]  op,
                      input  wire        go,
                      input  wire [23:0] addr,
                      input  wire [9:0]  len,
                      output reg         busy,
                      output reg         done,      // one-cycle pulse
                      output reg         ok,        // valid with done

                      output reg  [8:0]  idx,       // program data: which byte
                      input  wire [7:0]  wdata,     // ...and the byte itself
                      output reg         rd_stb,    // read data: one pulse per byte
                      output reg  [7:0]  rdata,
                      output reg  [23:0] jedec,

                      output reg         csn,
                      output reg         sck,
                      output reg         mosi,
                      input  wire        miso
                      );

   localparam [2:0] OP_ID = 3'd1, OP_ERASE = 3'd2, OP_PROG = 3'd3, OP_READ = 3'd4;

   localparam integer TIMEOUT_CYC = (CLK_HZ / 1000) * POLL_TIMEOUT_MS;

   // ---- one byte, MSB first ----------------------------------------------
   // Four clocks per bit: MOSI changes with SCK low, SCK rises, MISO is
   // sampled, SCK falls on the next bit. MISO goes through one register
   // first; the flash drives it from the falling edge, a full clock
   // before we look.
   reg [7:0] tx, rx;
   reg [2:0] bitn;
   reg [1:0] ph;
   reg       xfer, xdone;
   reg       miso_s;

   always @(posedge clk) miso_s <= miso;

   // ---- the command sequencer --------------------------------------------
   localparam [3:0]
                   S_IDLE  = 4'd0,
                   S_WREN  = 4'd1,  S_WREN_E = 4'd2,
                   S_OPC   = 4'd3,  S_ADDR   = 4'd4,
                   S_DATA  = 4'd5,  S_FETCH  = 4'd6,
                   S_END   = 4'd7,
                   S_POLL  = 4'd8,  S_POLL_R = 4'd9,
                   S_FIN   = 4'd10, S_GAP    = 4'd11;

   reg [3:0]  st;
   reg [2:0]  cur;
   reg [23:0] a;
   reg [9:0]  n, cnt;
   reg [1:0]  ai;
   reg [1:0]  fetch;
   reg [2:0]  gap;
   reg [31:0] tmo;
   reg        first;

   task send(input [7:0] b);
      begin
         tx <= b; bitn <= 3'd0; ph <= 2'd0; xfer <= 1'b1;
      end
   endtask

   always @(posedge clk) begin
      done   <= 1'b0;
      rd_stb <= 1'b0;
      xdone  <= 1'b0;
      if (rst) begin
         st <= S_IDLE; busy <= 1'b0; ok <= 1'b0;
         csn <= 1'b1; sck <= 1'b0; mosi <= 1'b0;
         xfer <= 1'b0; idx <= 9'd0; jedec <= 24'd0;
      end else begin
         if (xfer) begin
            ph <= ph + 1'b1;
            case (ph)
              2'd0: begin sck <= 1'b0; mosi <= tx[7]; end
              2'd2: begin sck <= 1'b1; rx <= {rx[6:0], miso_s}; end
              2'd3: begin
                 tx <= {tx[6:0], 1'b0};
                 bitn <= bitn + 1'b1;
                 if (bitn == 3'd7) begin
                    xfer <= 1'b0; xdone <= 1'b1; sck <= 1'b0;
                 end
              end
              default: ;
            endcase
         end

         case (st)
           S_IDLE: if (go) begin
              cur <= op; a <= addr; n <= len; cnt <= 10'd0;
              busy <= 1'b1; ok <= 1'b0;
              if (op == OP_ERASE || op == OP_PROG) begin
                 csn <= 1'b0; send(8'h06); st <= S_WREN;
              end else begin
                 csn <= 1'b0;
                 send(op == OP_ID ? 8'h9F : 8'h03);
                 st <= S_OPC;
              end
           end

           // Write Enable is its own command: /CS has to rise after it.
           S_WREN: if (xdone) begin csn <= 1'b1; gap <= 3'd0; st <= S_WREN_E; end
           S_WREN_E: begin
              gap <= gap + 1'b1;
              if (gap == 3'd4) begin
                 csn <= 1'b0;
                 send(cur == OP_ERASE ? 8'hD8 : 8'h02);
                 st <= S_OPC;
              end
           end

           S_OPC: if (xdone) begin
              if (cur == OP_ID) begin
                 cnt <= 10'd0; send(8'h00); st <= S_DATA;
              end else begin
                 ai <= 2'd0; send(a[23:16]); st <= S_ADDR;
              end
           end

           S_ADDR: if (xdone) begin
              ai <= ai + 1'b1;
              case (ai)
                2'd0: send(a[15:8]);
                2'd1: send(a[7:0]);
                default: begin
                   if (cur == OP_ERASE) begin
                      csn <= 1'b1; st <= S_END;
                   end else if (cur == OP_PROG) begin
                      idx <= 9'd0; fetch <= 2'd0; st <= S_FETCH;
                   end else begin
                      send(8'h00); st <= S_DATA;
                   end
                end
              endcase
           end

           // Program data: idx is set, wait for the RAM, then send.
           S_FETCH: begin
              fetch <= fetch + 1'b1;
              if (fetch == 2'd2) begin send(wdata); st <= S_DATA; end
           end

           S_DATA: if (xdone) begin
              cnt <= cnt + 1'b1;
              if (cur == OP_PROG) begin
                 if (cnt + 1'b1 == n) begin
                    csn <= 1'b1; st <= S_END;
                 end else begin
                    idx <= cnt[8:0] + 1'b1; fetch <= 2'd0; st <= S_FETCH;
                 end
              end else if (cur == OP_ID) begin
                 jedec <= {jedec[15:0], rx};
                 if (cnt == 10'd2) begin csn <= 1'b1; ok <= 1'b1; st <= S_FIN; end
                 else send(8'h00);
              end else begin
                 rd_stb <= 1'b1; rdata <= rx;
                 if (cnt + 1'b1 == n) begin csn <= 1'b1; ok <= 1'b1; st <= S_FIN; end
                 else send(8'h00);
              end
           end

           // Erase and program run inside the flash after /CS rises. Poll
           // the busy bit (status register 1, bit 0) until it clears.
           S_END: begin gap <= 3'd0; tmo <= 32'd0; st <= S_POLL; end
           S_POLL: begin
              gap <= gap + 1'b1;
              if (gap == 3'd4) begin csn <= 1'b0; send(8'h05); st <= S_POLL_R; first <= 1'b1; end
           end
           S_POLL_R: begin
              tmo <= tmo + 1'b1;
              if (xdone) begin
                 if (!first && !rx[0]) begin
                    csn <= 1'b1; ok <= 1'b1; st <= S_FIN;
                 end else if (tmo >= TIMEOUT_CYC) begin
                    csn <= 1'b1; ok <= 1'b0; st <= S_FIN;
                 end else begin
                    first <= 1'b0; send(8'h00);   // status repeats while /CS stays low
                 end
              end
           end

           S_FIN: begin
              gap <= 3'd0; st <= S_GAP;
           end
           S_GAP: begin
              gap <= gap + 1'b1;
              if (gap == 3'd4) begin busy <= 1'b0; done <= 1'b1; st <= S_IDLE; end
           end

           default: st <= S_IDLE;
         endcase
      end
   end

endmodule

`default_nettype wire
