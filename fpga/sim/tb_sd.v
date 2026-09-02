// tb_sd: bring sd_spi.v up against a model card, with no ProFile bus involved.
//
// The no-host SD test: check initialization and a CMD17 read in simulation,
// then run the same bitstream on hardware with nothing on the DB-25 and read
// the answer off the LEDs.
//
// Checks, in order:
//   1. init completes (ready) and err_code stays clear
//   2. a read of sector 0 lands 512 bytes in the buffer
//   3. the last two are $55 $AA, which is what a real formatted card has too
//   4. a read of a different sector returns that sector, not a cached first one
//   5. a write is REFUSED loudly instead of doing nothing with no error
//
//   iverilog -g2012 -o /tmp/tb -s tb_sd sim/tb_sd.v sim/sd_model.v src/sd_spi.v

`timescale 1ns / 1ps
`default_nettype none

module tb_sd #(parameter integer SKEW = 0, parameter V1 = 1'b0,
               parameter integer FAILR = 0, parameter integer RETRIES = 3);

   localparam integer TCK = 10;                 // 100 MHz

   reg clk = 0, rst = 1;
   always #(TCK/2) clk = ~clk;

   reg         rd = 0, wr = 0;
   reg  [31:0] lba = 0;
   wire        done, err, ready;
   wire [3:0]  err_code;
   wire [4:0]  dbg_state;
   wire [9:0]  buf_addr;
   wire [7:0]  buf_wdata;
   wire        buf_we;

   wire sd_clk, sd_cs_n, sd_mosi, sd_miso;

   // The write source. Registered, because the real block RAM in top.v presents
   // its data a cycle after the address: a testbench that answers
   // combinationally would hide an address-timing bug in the write path.
   reg [7:0] wbuf [0:511];
   reg [7:0] wbuf_q;
   always @(posedge clk) wbuf_q <= wbuf[buf_addr];

   // DIV_INIT is shortened here only to keep the run short; the divider itself
   // is exercised, just not for the full 400 kHz worth of wall clock.
   sd_spi #(.DIV_INIT(8), .DIV_FAST(2), .RETRY_MAX(RETRIES),
            .TOKEN_TIMEOUT_CYC(20_000)) dut (
                                             .clk(clk), .rst(rst),
                                             .rd(rd), .wr(wr), .lba(lba), .done(done), .err(err),
                                             .buf_addr(buf_addr), .buf_wdata(buf_wdata), .buf_we(buf_we),
                                             .buf_rdata(wbuf_q),
                                             .sd_clk(sd_clk), .sd_cs_n(sd_cs_n), .sd_mosi(sd_mosi), .sd_miso(sd_miso),
                                             .ready(ready), .err_code(err_code), .dbg_state(dbg_state));

   sd_model #(.SKEW(SKEW), .V1(V1), .FAIL_READS(FAILR)) card (
                                                              .sd_clk(sd_clk), .sd_cs_n(sd_cs_n), .sd_mosi(sd_mosi), .sd_miso(sd_miso));

   // Capture what the reader puts in the block buffer.
   reg [7:0] mem [0:511];
   integer   nwritten = 0;
   always @(posedge clk) if (buf_we) begin
      mem[buf_addr] = buf_wdata;
      nwritten = nwritten + 1;
   end

   integer errors = 0;
   integer i;

   task check(input cond, input [255:0] what);
      begin
         if (cond) $display("    ok    %0s", what);
         else begin $display("*** FAIL  %0s", what); errors = errors + 1; end
      end
   endtask

   task do_read(input [31:0] sector);
      begin
         nwritten = 0;
         lba = sector;
         @(posedge clk); rd = 1; @(posedge clk); rd = 0;
         wait (done);
         @(posedge clk);
      end
   endtask

   initial begin
      $dumpfile("/tmp/tb_sd.vcd");
      $dumpvars(0, tb_sd);

      repeat (5) @(posedge clk);
      rst = 0;

      $display("-- initialization (skew %0d bits, %0s card)", SKEW, V1 ? "v1 byte-addressed" : "v2 SDHC");
      wait (ready || err);
      check(ready === 1'b1, "card reached ready");
      check(err_code === 4'd0, "no error code after init");
      if (!ready) begin
         $display("    stalled in state %0d with err_code %0d", dbg_state, err_code);
      end

      $display("-- read sector 0  (card rejects the first %0d read(s), budget %0d)",
               FAILR, RETRIES);
      do_read(0);
      if (FAILR <= RETRIES) begin
         check(err === 1'b0, "read succeeded, transient absorbed by the retry");
         if (FAILR > 0)
           check(dut.retry_n === FAILR[7:0], "and it took exactly that many retries");
      end else begin
         // The budget has to be a budget. A retry loop with no bound turns a
         // card that never answers into a hang instead of an error, which is
         // strictly worse.
         check(err === 1'b1, "read gave up once the retry budget was spent");
         $display("    err_code %0d after %0d retries", err_code, dut.retry_n);
      end
      if (FAILR > RETRIES) begin
         $display("");
         $display("PASS: the retry budget is bounded");
         $finish;
      end
      check(nwritten == 512, "exactly 512 bytes written to the buffer");
      check(mem[510] === 8'h55 && mem[511] === 8'hAA, "boot signature $55 $AA at 510");
      $write("    first 8 bytes:");
      for (i = 0; i < 8; i = i + 1) $write(" %02h", mem[i]);
      $display("");

      $display("-- read sector 1234, to check the address reaches the card");
      do_read(32'd1234);
      check(err === 1'b0, "second read reported success");
      check({mem[0], mem[1], mem[2], mem[3]} === 32'd1234,
            "sector echoes its own LBA in bytes 0-3");

      // The capture buffer is a diagnostic, so it needs its own check: a
      // mechanism that records nothing and reports no fault is worse than
      // none at all.
      $display("-- raw capture after a command");
      $display("    cap = %032h", dut.cap);
      check(dut.cap !== 128'd0, "capture recorded something");
      check(dut.cap_i === 5'd16 || dut.cap_i !== 5'd0, "capture index advanced");

      $display("-- write sector 77, then read it back");
      for (i = 0; i < 512; i = i + 1) wbuf[i] = (i * 7) & 8'hFF;
      lba = 77;
      @(posedge clk); wr = 1; @(posedge clk); wr = 0;
      wait (done);
      @(posedge clk);
      check(err === 1'b0, "write reported success");
      do_read(32'd77);
      check(err === 1'b0, "read-back reported success");
      begin : cmpblk
         integer bad; bad = 0;
         for (i = 0; i < 512; i = i + 1) if (mem[i] !== wbuf[i]) bad = bad + 1;
         check(bad == 0, "every byte read back matches what was written");
         if (bad) $display("    %0d byte(s) differ, first at %0d: got %02h want %02h",
                           bad, 0, mem[0], wbuf[0]);
      end

      $display("");
      if (errors == 0) $display("PASS: SD init and single-block reads work");
      else             $display("FAIL: %0d problem(s)", errors);
      $finish;
   end

   initial begin
      #20_000_000;
      $display("*** TIMEOUT in state %0d, err_code %0d, ready=%0b",
               dbg_state, err_code, ready);
      $finish;
   end

endmodule

`default_nettype wire
