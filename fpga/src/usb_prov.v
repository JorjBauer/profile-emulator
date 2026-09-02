// usb_prov: raw SD sector read/write over the debug USB serial link.
//
// Provisioning and repair without pulling the card. With the //e powered off
// the ProFile side is idle, so this owns the SD engine directly (the same way
// the self-test and verifier do). It moves whole 512-byte sectors between the
// Mac and the card, so a corrupted image can be repaired in place rather than
// by pulling the card.
//
// Framed commands on usb_rx (115200 8N1), each led by a byte + the magic 'B'
// so stray traffic cannot trigger a write:
//
//   'P' 'B'                                  ping  -> replies 'A'
//   'R' 'B' lba[31:24..7:0]                  read  -> 'D', 512 bytes, 'K'
//   'W' 'B' lba[31:24..7:0] <512 bytes>      write -> 'K' ok / 'E' error
//
// The ACK ('K'/'E') gives the Mac flow control: it waits for the reply before
// the next command, so the link never overruns. Responses go out on usb_tx,
// muxed ahead of the reporter while prov_active is high. The whole channel is
// content-insensitive on the 512 data bytes (a fixed counter, never a
// value-triggered parser), so any image bytes pass through untouched.

`default_nettype none

module usb_prov #(
                  parameter integer CLK_HZ = 50_000_000
                  ) (
                     input  wire        clk,
                     input  wire        rst,

                     // command bytes from uart_rx
                     input  wire [7:0]  rx_data,
                     input  wire        rx_stb,

                     // SD engine face
                     input  wire        engine_ready,     // sd_dbg == SD_S_READY
                     input  wire        sd_done,
                     input  wire        sd_err,
                     output reg         prov_rd,          // 1-cycle fire
                     output reg         prov_wr,          // 1-cycle fire
                     output reg  [31:0] prov_lba,

                     // WRITE data: sd_spi drives sd_buf_addr and reads this
                     // back
                     input  wire [9:0]  sd_buf_addr,
                     output wire [7:0]  prov_rdata,

                     // READ data: snoop the sector sd_spi streams into the
                     // block buffer
                     input  wire        sd_buf_we,
                     input  wire [7:0]  sd_buf_wdata,
                     // sd_buf_addr (above) is the write address during a read,
                     // too

                     output reg         prov_active,      // owns SD + usb_tx while high
                     output wire        seen_cmd,         // a prov command arrived: gate 'I' off
                     output wire        tx                // to the usb_tx mux
                     );

   // 512-byte sector buffer, dual role: filled from USB on a write, filled from
   // the SD on a read.
   reg [7:0] pbuf [0:511];
   reg [7:0] pbuf_q;
   reg [8:0] pb_a;
   reg [7:0] pb_d;
   reg       pb_we;
   reg [8:0] out_i;
   reg       prov_wr_phase;   // high while an SD write is draining pbuf
   // sd_spi reads write-data through sd_buf_addr; the streamer reads through
   // out_i. One read port, muxed by who is reading.
   wire [8:0] pb_rd_a = prov_wr_phase ? sd_buf_addr[8:0] : out_i;
   always @(posedge clk) begin
      if (pb_we) pbuf[pb_a] <= pb_d;
      pbuf_q <= pbuf[pb_rd_a];
   end
   assign prov_rdata = pbuf_q;

   localparam [3:0]
                   S_IDLE   = 4'd0,
                   S_MAGIC  = 4'd1,
                   S_LBA    = 4'd2,
                   S_WDATA  = 4'd3,
                   S_WGO    = 4'd4,
                   S_RGO    = 4'd5,
                   S_ROUT   = 4'd6,
                   S_REPLY  = 4'd7;

   reg [3:0]       st;
   reg [1:0]       cmd;          // 0 ping, 1 read, 2 write
   reg [1:0]       lba_i;
   reg [8:0]       data_i;
   reg             seen_r;
   assign seen_cmd = seen_r;
   reg        fired;        // one SD op per GO, not one per ready-cycle

   // --- response transmitter ---
   reg [7:0]  tx_ch;
   reg        tx_stb;
   wire       tx_busy;
   uart_tx #(.CLK_HZ(CLK_HZ)) u_tx (
                                    .clk(clk), .rst(rst), .data(tx_ch), .stb(tx_stb),
                                    .busy(tx_busy), .tx(tx));

   reg [7:0]  reply_b;      // final ACK byte ('K'/'E'/'A')
   reg        have_reply;

   always @(posedge clk) begin
      prov_rd <= 1'b0;
      prov_wr <= 1'b0;
      pb_we   <= 1'b0;
      tx_stb  <= 1'b0;

      if (rst) begin
         st <= S_IDLE; prov_active <= 1'b0; prov_wr_phase <= 1'b0;
         seen_r <= 1'b0; lba_i <= 0; data_i <= 0; out_i <= 0;
         have_reply <= 1'b0; cmd <= 0; prov_lba <= 0; fired <= 1'b0;
      end else begin
         case (st)
           // -------- await a command lead byte --------
           S_IDLE: if (rx_stb) begin
              case (rx_data)
                "P": begin cmd <= 2'd0; st <= S_MAGIC; end
                "R": begin cmd <= 2'd1; st <= S_MAGIC; end
                "W": begin cmd <= 2'd2; st <= S_MAGIC; end
                default: st <= S_IDLE;   // ignore ('I' select etc.)
              endcase
           end

           S_MAGIC: if (rx_stb) begin
              if (rx_data == "B") begin
                 seen_r      <= 1'b1;      // latch: 'I' select is now gated off
                 prov_active <= 1'b1;
                 if (cmd == 2'd0) begin           // ping
                    reply_b <= "A"; have_reply <= 1'b1; st <= S_REPLY;
                 end else begin
                    lba_i <= 2'd0; fired <= 1'b0; st <= S_LBA;
                 end
              end else st <= S_IDLE;             // bad magic: resync
           end

           // -------- 4 LBA bytes, big-endian --------
           S_LBA: if (rx_stb) begin
              prov_lba <= {prov_lba[23:0], rx_data};
              if (lba_i == 2'd3) begin
                 if (cmd == 2'd2) begin data_i <= 9'd0; st <= S_WDATA; end
                 else                  st <= S_RGO;      // read
              end else lba_i <= lba_i + 2'd1;
           end

           // -------- 512 data bytes into pbuf (write) --------
           S_WDATA: if (rx_stb) begin
              pb_a <= data_i; pb_d <= rx_data; pb_we <= 1'b1;
              if (data_i == 9'd511) st <= S_WGO;
              else data_i <= data_i + 9'd1;
           end

           // -------- issue the SD write, wait, ACK --------
           S_WGO: begin
              prov_wr_phase <= 1'b1;
              if (engine_ready && !fired) begin
                 prov_wr <= 1'b1; fired <= 1'b1;
              end
              if (sd_done) begin
                 prov_wr_phase <= 1'b0;
                 reply_b <= sd_err ? "E" : "K";
                 have_reply <= 1'b1;
                 st <= S_REPLY;
              end
           end

           // -------- issue the SD read, wait, then stream --------
           S_RGO: begin
              if (engine_ready && !fired) begin prov_rd <= 1'b1; fired <= 1'b1; end
              if (sd_done) begin
                 if (sd_err) begin
                    reply_b <= "E"; have_reply <= 1'b1; st <= S_REPLY;
                 end else begin
                    // emit the 'D' header, then 512 bytes
                    tx_ch <= "D"; tx_stb <= 1'b1;
                    out_i <= 9'd0; st <= S_ROUT;
                 end
              end
           end

           S_ROUT: if (!tx_busy && !tx_stb) begin
              tx_ch  <= pbuf_q;
              tx_stb <= 1'b1;
              if (out_i == 9'd511) begin
                 reply_b <= "K"; have_reply <= 1'b1; st <= S_REPLY;
              end else out_i <= out_i + 9'd1;
           end

           // -------- send the final ACK byte --------
           S_REPLY: if (have_reply && !tx_busy && !tx_stb) begin
              tx_ch  <= reply_b;
              tx_stb <= 1'b1;
              have_reply <= 1'b0;
              st <= S_IDLE;
           end

           default: st <= S_IDLE;
         endcase

         // snoop the SD-read stream into pbuf
         if (prov_rd || (st == S_RGO)) begin
            if (sd_buf_we) begin pb_a <= sd_buf_addr[8:0]; pb_d <= sd_buf_wdata; pb_we <= 1'b1; end
         end
      end
   end

endmodule

`default_nettype wire
