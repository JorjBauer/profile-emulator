`timescale 1ns/1ps
// tb_sd_sweep: the SD clock sweep against a card stub that goes bad above a
// chosen rate.
//
// The stub works at the request level, the way sd_spi looks from above: a
// read streams 512 bytes into the buffer, a write fetches 512 bytes by
// address. Above its read limit it flips a bit in every sector it reads, and
// above its write limit it flips one in every sector it stores. It can also
// fail a request with an error, count retries without failing, or hang for
// good after its first error. Like sd_spi, a request after a failure runs at
// the slow rate, so it can't fail on speed.
//
// The card has 256 sectors and its partition starts at 128, so the sweep's
// region is 94 to 126. Every case checks that nothing outside the region is
// ever written.
//
// Cases, each from reset:
//   1. writes go bad from DIV 16, reads from DIV 1: found, and zeros put back
//   2. a data sector in the region isn't blank: nothing is written
//   3. a GPT card: nothing is written
//   4. a partition at sector 32, too low for a region: nothing is written
//   5. the signature from an earlier run, with junk in the data sectors:
//      accepted, run and cleaned
//   6. a read error at DIV 8: that rate fails with the code, and the sweep
//      stops there
//   7. retries at write DIV 32 with good data: that rate fails, and the
//      sweep goes on
//   8. a card that hangs after its first error: the zeros don't go back, and
//      the signature stays
//   9. writes at DIV 4 report success and never land: caught, because the
//      sector still holds the previous rate's pattern

`default_nettype none

module tb_sd_sweep;

   localparam integer NSECT = 256;
   localparam integer PART  = 128;
   localparam integer SIG   = PART - 34;     // 94

   reg clk = 0, rst = 1;
   always #10 clk = ~clk;

   // ---- the card stub ----
   reg [7:0]  mem [0:NSECT*512-1];
   reg [7:0]  orig [0:NSECT*512-1];
   integer    rd_limit, wr_limit;   // a DIV below this corrupts
   integer    rd_err_div;           // a read at this DIV or below fails; -1 never
   integer    wr_retry_div;         // a write at exactly this DIV retries
   integer    wr_drop_div;          // a write at exactly this DIV never lands
   reg        hang_after_err;
   reg        hung, slow_next;
   reg [7:0]  retries = 0;
   integer    writes_outside;

   reg        engine_ready = 1;
   reg        sd_done = 0, sd_err = 0;
   reg [3:0]  sd_err_code = 0;
   reg [9:0]  buf_addr = 0;
   reg [7:0]  buf_wdata = 0;
   reg        buf_we = 0;

   wire       rd, wr, active, done;
   wire [31:0] lba, status, part_lba;
   wire [7:0]  speed, wspeed, wdata;
   wire [255:0] wres, rres;

   sd_sweep dut (
                 .clk(clk), .rst(rst), .start(!rst),
                 .engine_ready(engine_ready), .sd_done(sd_done), .sd_err(sd_err),
                 .sd_err_code(sd_err_code), .sd_retries(retries),
                 .rd(rd), .wr(wr), .lba(lba), .speed(speed), .wspeed(wspeed),
                 .buf_we(buf_we), .buf_addr(buf_addr), .buf_wdata(buf_wdata),
                 .wdata(wdata), .active(active), .done(done), .status(status),
                 .part_lba(part_lba), .wres(wres), .rres(rres));

   integer    i, a;
   reg [31:0] l;
   reg [7:0]  d, b;
   reg        w, fail;

   always @(posedge clk) begin
      if (rd || wr) begin
         l = lba; w = wr; d = wr ? wspeed : speed;
         engine_ready <= 1'b0;
         fork serve; join_none
      end
   end

   task finish(input e, input [3:0] code);
      begin
         @(posedge clk);
         sd_err <= e; sd_err_code <= e ? code : 4'd0; sd_done <= 1'b1;
         @(posedge clk);
         sd_done <= 1'b0; sd_err <= 1'b0; engine_ready <= 1'b1;
         if (e && hang_after_err) hung = 1'b1;
         slow_next = e;
      end
   endtask

   task serve;
      reg [31:0] sl;
      reg        sw, slow;
      reg [7:0]  sdiv;
      begin
         sl = l; sw = w; sdiv = d; slow = slow_next;
         repeat (20) @(posedge clk);
         if (hung) begin
            finish(1'b1, sw ? 4'd11 : 4'd7);
         end else if (!sw) begin
            if (!slow && rd_err_div >= 0 && sdiv <= rd_err_div) begin
               finish(1'b1, 4'd7);
            end else begin
               for (a = 0; a < 512; a = a + 1) begin
                  @(posedge clk);
                  b = mem[sl*512 + a];
                  if (!slow && sdiv < rd_limit && a == 100) b = b ^ 8'h10;
                  buf_addr <= a; buf_wdata <= b; buf_we <= 1'b1;
               end
               @(posedge clk); buf_we <= 1'b0;
               finish(1'b0, 4'd0);
            end
         end else begin
            if (sl < SIG || sl > SIG + 32) writes_outside = writes_outside + 1;
            for (a = 0; a < 512; a = a + 1) begin
               buf_addr <= a;
               @(posedge clk); #1;
               b = wdata;
               if (!slow && sdiv < wr_limit && a == 200) b = b ^ 8'h01;
               if (slow || sdiv != wr_drop_div) mem[sl*512 + a] = b;
            end
            if (!slow && sdiv == wr_retry_div) retries = retries + 1;
            finish(1'b0, 4'd0);
         end
      end
   endtask

   // ---- the card's contents ----
   task blank_card(input [7:0] ptype, input [31:0] pstart);
      begin
         for (i = 0; i < NSECT*512; i = i + 1) mem[i] = 8'h00;
         // something in every sector outside the gap
         for (i = 0; i < 64*512; i = i + 1) mem[i] = i[7:0] ^ 8'h3C;
         for (i = PART*512; i < NSECT*512; i = i + 1) mem[i] = i[9:2];
         mem[93*512 + 5] = 8'h77;     // and in the sectors on either side
         mem[127*512 + 5] = 8'h77;
         for (i = 446; i < 510; i = i + 1) mem[i] = 8'h00;
         mem[450] = ptype;
         mem[454] = pstart[7:0];  mem[455] = pstart[15:8];
         mem[456] = pstart[23:16]; mem[457] = pstart[31:24];
         mem[510] = 8'h55; mem[511] = 8'hAA;
      end
   endtask

   task snap;
      for (i = 0; i < NSECT*512; i = i + 1) orig[i] = mem[i];
   endtask

   // ---- checks ----
   integer    fails = 0;
   task check(input ok, input [8*64-1:0] what);
      begin
         if (!ok) begin
            $display("FAIL: %0s", what);
            fails = fails + 1;
         end else $display("  ok: %0s", what);
      end
   endtask

   // Tested and passed bits, rates 0 to 7, as two 8-bit masks.
   function [7:0] tested(input [255:0] r);
      integer j;
      for (j = 0; j < 8; j = j + 1) tested[j] = r[32*j + 31];
   endfunction
   function [7:0] passed(input [255:0] r);
      integer j;
      for (j = 0; j < 8; j = j + 1) passed[j] = r[32*j + 30];
   endfunction

   task same_outside(input [8*64-1:0] what);
      begin
         fail = 0;
         for (i = 0; i < NSECT*512; i = i + 1)
           if ((i < SIG*512 || i >= (SIG + 33)*512) && mem[i] !== orig[i])
             fail = 1;
         check(!fail && writes_outside == 0, what);
      end
   endtask

   task region_zero(input [8*64-1:0] what);
      begin
         fail = 0;
         for (i = SIG*512; i < (SIG + 33)*512; i = i + 1)
           if (mem[i] !== 8'h00) fail = 1;
         check(!fail, what);
      end
   endtask

   task region_same(input [8*64-1:0] what);
      begin
         fail = 0;
         for (i = SIG*512; i < (SIG + 33)*512; i = i + 1)
           if (mem[i] !== orig[i]) fail = 1;
         check(!fail, what);
      end
   endtask

   task run(input [8*64-1:0] name);
      begin
         $display("-- %0s", name);
         rst = 1; hung = 0; slow_next = 0; retries = 0; writes_outside = 0;
         snap;
         repeat (5) @(posedge clk);
         rst = 0;
         fork : wait_done
            begin wait (done); @(posedge clk); disable wait_done; end
            begin #400_000_000; $display("FAIL: timeout"); fails = fails + 1;
               disable wait_done; end
         join
         $display("  status %08h  writes %02h/%02h  reads %02h/%02h",
                  status, tested(wres), passed(wres), tested(rres),
                  passed(rres));
      end
   endtask

   task defaults;
      begin
         rd_limit = 0; wr_limit = 0; rd_err_div = -1; wr_retry_div = -1;
         wr_drop_div = -1;
         hang_after_err = 0;
      end
   endtask

   initial begin
      // 1
      defaults; wr_limit = 32; rd_limit = 2;
      blank_card(8'h0C, PART);
      run("1. writes bad from DIV 16, reads from DIV 1");
      check(status[27:24] == 0, "no reason to stop");
      check(tested(wres) == 8'b0000_0111 && passed(wres) == 8'b0000_0011,
            "writes: 391k and 758k pass, 1.47M fails, nothing above tried");
      check(wres[64 +: 16] == 16'd32, "1.47M: one bad byte in each of 32");
      check(tested(rres) == 8'b0111_1111 && passed(rres) == 8'b0011_1111,
            "reads: up to 8.33M pass, 12.5M fails, 25M not tried");
      check(rres[192 +: 16] == 16'd128, "12.5M: one bad byte in each of 128");
      check(status[19:16] == 2 && status[11:8] == 6, "pass counts");
      check(status[20], "zeros restored");
      check(part_lba == PART, "partition start");
      region_zero("the region is zeros again");
      same_outside("nothing outside the region changed");

      // 2
      defaults;
      blank_card(8'h0C, PART);
      mem[(SIG + 7)*512 + 300] = 8'h01;
      run("2. a data sector isn't blank");
      check(status[27:24] == 3, "stopped: in use");
      check(tested(wres) == 0 && tested(rres) == 0, "no rate tried");
      region_same("the region is untouched");
      same_outside("nothing outside the region changed");

      // 3
      defaults;
      blank_card(8'hEE, 1);
      run("3. a GPT card");
      check(status[27:24] == 2, "stopped: no gap");
      region_same("the region is untouched");
      same_outside("nothing outside the region changed");

      // 4
      defaults;
      blank_card(8'h0C, 32);
      run("4. a partition at sector 32");
      check(status[27:24] == 2, "stopped: no gap");
      region_same("the region is untouched");
      same_outside("nothing outside the region changed");

      // 5
      defaults;
      blank_card(8'h0C, PART);
      mem[SIG*512 + 0] = "P"; mem[SIG*512 + 1] = "E"; mem[SIG*512 + 2] = "S";
      mem[SIG*512 + 3] = "W"; mem[SIG*512 + 4] = "E"; mem[SIG*512 + 5] = "E";
      mem[SIG*512 + 6] = "P"; mem[SIG*512 + 7] = "1";
      for (i = (SIG + 1)*512; i < (SIG + 33)*512; i = i + 1) mem[i] = 8'hE7;
      run("5. the signature, and junk in the data sectors");
      check(status[27:24] == 0, "no reason to stop");
      check(passed(wres) == 8'hFF && passed(rres) == 8'hFF,
            "a perfect card passes every rate");
      check(status[19:16] == 8 && status[11:8] == 8, "pass counts");
      region_zero("the region is zeros again");
      same_outside("nothing outside the region changed");

      // 6
      defaults; rd_err_div = 8;
      blank_card(8'h0C, PART);
      run("6. a read error at DIV 8");
      check(passed(wres) == 8'hFF, "writes all pass");
      check(tested(rres) == 8'b0000_1111 && passed(rres) == 8'b0000_0111,
            "reads: 2.78M fails, nothing above tried");
      check(rres[120 +: 4] == 4'd7, "the SD error code is kept");
      check(status[20], "zeros restored");
      region_zero("the region is zeros again");
      same_outside("nothing outside the region changed");

      // 7
      defaults; wr_retry_div = 32;
      blank_card(8'h0C, PART);
      run("7. retries at write DIV 32");
      check(tested(wres) == 8'hFF, "every write rate tried");
      check(passed(wres) == 8'b1111_1101, "758k fails on its retries alone");
      check(wres[48 +: 8] == 8'd32 && wres[32 +: 16] == 0,
            "32 retries, no bad bytes");
      region_zero("the region is zeros again");
      same_outside("nothing outside the region changed");

      // 8
      defaults; rd_err_div = 4; hang_after_err = 1;
      blank_card(8'h0C, PART);
      run("8. a card that hangs after its first error");
      check(tested(rres) == 8'b0001_1111 && passed(rres) == 8'b0000_1111,
            "reads: 5.0M fails");
      check(!status[20], "zeros not restored");
      check(mem[SIG*512] == "P" && mem[SIG*512 + 7] == "1",
            "the signature stays, so a rerun is accepted");
      same_outside("nothing outside the region changed");

      // 9
      defaults; wr_drop_div = 4;
      blank_card(8'h0C, PART);
      run("9. writes at DIV 4 never land");
      check(tested(wres) == 8'b0001_1111 && passed(wres) == 8'b0000_1111,
            "writes: 5.0M fails");
      check(wres[128 +: 16] == 16'd16384, "every byte of all 32 is wrong");
      check(passed(rres) == 8'hFF, "reads all pass");
      region_zero("the region is zeros again");
      same_outside("nothing outside the region changed");

      if (fails == 0) $display("PASS: tb_sd_sweep");
      else            $display("FAIL: tb_sd_sweep, %0d checks", fails);
      $finish;
   end

endmodule

`default_nettype wire
