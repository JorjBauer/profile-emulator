`timescale 1ns/1ps
// usb_prov protocol test with a mock SD engine backed by a memory array.
// Drives the command bytes straight into rx_data/rx_stb, mocks the SD engine's
// buf handshake, and checks a write-then-read round-trip byte for byte.
module tb_usb_prov;
   reg clk=0; always #10 clk=~clk;
   reg rst=1;
   reg [7:0] rx_data=0; reg rx_stb=0;
   reg sd_done=0, sd_err=0;
   wire prov_rd, prov_wr, prov_active, seen_cmd, tx;
   wire [31:0] prov_lba;
   wire [7:0]  prov_rdata;
   reg  [9:0]  sd_buf_addr=0;
   reg         sd_buf_we=0; reg [7:0] sd_buf_wdata=0;

   usb_prov #(.CLK_HZ(115_200)) dut (
                                     .clk(clk), .rst(rst), .rx_data(rx_data), .rx_stb(rx_stb),
                                     .engine_ready(1'b1), .sd_done(sd_done), .sd_err(sd_err),
                                     .prov_rd(prov_rd), .prov_wr(prov_wr), .prov_lba(prov_lba),
                                     .sd_buf_addr(sd_buf_addr), .prov_rdata(prov_rdata),
                                     .sd_buf_we(sd_buf_we), .sd_buf_wdata(sd_buf_wdata),
                                     .prov_active(prov_active), .seen_cmd(seen_cmd), .tx(tx));

   // the "card": one sector of storage
   reg [7:0] card [0:511];

   integer fails=0;
   task chk(input c, input [255:0] w); if(!c) begin fails=fails+1; $display("*** FAIL %0s",w); end endtask

   // feed one byte into the receiver
   task rxb(input [7:0] b);
      begin @(posedge clk); rx_data<=b; rx_stb<=1; @(posedge clk); rx_stb<=0; end
   endtask

   // mock the SD engine's write: pull 512 bytes through sd_buf_addr/prov_rdata
   integer i;
   task sd_write_capture;
      begin
         for (i=0;i<512;i=i+1) begin
            sd_buf_addr<=i[9:0];
            repeat(3) @(posedge clk);          // let prov_rdata settle
            card[i]=prov_rdata;
         end
         @(posedge clk); sd_done<=1; @(posedge clk); sd_done<=0;
      end
   endtask

   // mock the SD engine's read: stream the sector in via sd_buf_we
   task sd_read_serve;
      begin
         for (i=0;i<512;i=i+1) begin
            sd_buf_addr<=i[9:0]; sd_buf_wdata<=card[i]; sd_buf_we<=1;
            @(posedge clk); sd_buf_we<=0; @(posedge clk);
         end
         @(posedge clk); sd_done<=1; @(posedge clk); sd_done<=0;
      end
   endtask

   // capture the tx stream (bit-bang decode at the known baud is overkill;
   // instead snoop the module's own tx_ch/tx_stb through hierarchy)
   reg [7:0] txcap [0:600];
   integer txn=0;
   always @(posedge clk) if (dut.tx_stb) begin txcap[txn]=dut.tx_ch; txn=txn+1; end

   reg [3:0] pst=15;
   always @(posedge clk) if (dut.st != pst) begin
      $display("t=%0t st %0d -> %0d (sd_done=%b prov_wr=%b prov_wr_phase=%b have_reply=%b tx_busy=%b)",
               $time, pst, dut.st, sd_done, prov_wr, dut.prov_wr_phase, dut.have_reply, dut.tx_busy);
      pst <= dut.st;
   end
   initial begin
      repeat(3) @(posedge clk); rst=0; repeat(2) @(posedge clk);

      // ---- PING ---- (wait for the ack, as the Mac does)
      txn=0;
      rxb("P"); rxb("B");
      for (i=0;i<5000 && txn<1;i=i+1) @(posedge clk);
      chk(txn==1 && txcap[0]=="A", "ping replies A");

      // ---- WRITE lba=$00010002 with pattern byte i = (i*7+3) ----
      txn=0;
      rxb("W"); rxb("B");
      rxb(8'h00); rxb(8'h01); rxb(8'h00); rxb(8'h02);      // big-endian LBA
      for (i=0;i<512;i=i+1) rxb((i*7+3) & 8'hFF);
      // prov should now be in S_WGO asserting prov_wr; service it
      wait(prov_wr);
      chk(prov_lba==32'h00010002, "write LBA parsed");
      sd_write_capture;
      for (i=0;i<5000 && txn<1;i=i+1) @(posedge clk);
      chk(txn>=1 && txcap[txn-1]=="K", "write acks K");
      // verify the card got the exact pattern
      begin : vw
         integer bad; bad=0;
         for (i=0;i<512;i=i+1) if (card[i] !== ((i*7+3)&8'hFF)) bad=bad+1;
         chk(bad==0, "card holds the written pattern");
      end

      // ---- READ lba back, verify the streamed bytes ----
      txn=0;
      rxb("R"); rxb("B");
      rxb(8'h00); rxb(8'h01); rxb(8'h00); rxb(8'h02);
      wait(prov_rd);
      chk(prov_lba==32'h00010002, "read LBA parsed");
      sd_read_serve;
      // wait for the full stream: 'D' + 512 + 'K' = 514 tx bytes
      for (i=0;i<200000 && txn<514;i=i+1) @(posedge clk);
      chk(txn==514, "read streamed D + 512 + K");
      chk(txcap[0]=="D", "read header is D");
      chk(txcap[513]=="K", "read trailer is K");
      begin : vr
         integer bad; bad=0;
         for (i=0;i<512;i=i+1) if (txcap[1+i] !== ((i*7+3)&8'hFF)) bad=bad+1;
         chk(bad==0, "streamed bytes match the sector");
      end

      if (fails==0) $display("PASS: usb_prov ping/write/read round-trip");
      else $display("FAIL: %0d problem(s)", fails);
      $finish;
   end
   initial begin #500_000_000; $display("TIMEOUT txn=%0d", txn); $finish; end
endmodule
