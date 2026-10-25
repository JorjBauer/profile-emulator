`timescale 1ns/1ps
// The persist-vs-host SD collision, at the arbiter seam.
//
// Without arbitration a host read pulsed into a busy engine is dropped, and
// the persist's completion is mistaken for the host's. PASS criteria: no
// request is ever lost, completions route
// only to the owner of the in-flight op, and the router always beats the
// persist to an idle engine.
module tb_sd_arb;
   reg clk = 0; always #10 clk = ~clk;
   reg rst = 1;
   reg engine_ready = 1, sd_done = 0, sd_err = 0;
   reg r_rd = 0, r_wr = 0, f_req = 0, f_req_wr = 0;
   wire r_fire_rd, r_fire_wr, r_done, r_err, f_fire, f_done, f_err, own_fs;

   sd_arb dut (.clk(clk), .rst(rst),
               .engine_ready(engine_ready), .sd_done(sd_done), .sd_err(sd_err),
               .r_rd(r_rd), .r_wr(r_wr),
               .r_fire_rd(r_fire_rd), .r_fire_wr(r_fire_wr),
               .r_done(r_done), .r_err(r_err),
               .f_req(f_req), .f_req_wr(f_req_wr), .f_fire(f_fire),
               .f_done(f_done), .f_err(f_err),
               .u_req(1'b0), .u_fire(), .u_done(), .u_err(),
               .own_fs(own_fs), .own_up());

   integer fails = 0;
   task check(input cond, input [255:0] what);
      if (!cond) begin
         fails = fails + 1;
         $display("*** FAIL %0s at t=%0t", what, $time);
      end
   endtask

   integer i;
   reg saw;
   initial begin
      repeat (3) @(posedge clk); rst = 0; repeat (2) @(posedge clk);

      // 1: the collision. fat32 fires a persist op; mid-flight the router
      // pulses a read. The pulse must be latched, and the persist's done must
      // NOT reach the router.
      f_req <= 1; @(posedge clk);
      check(f_fire === 1'b1, "persist fires on the idle engine");
      f_req <= 0; engine_ready <= 0; @(posedge clk);
      check(own_fs === 1'b1, "the in-flight op is owned by fat32");
      r_rd <= 1; @(posedge clk); r_rd <= 0;   // host read into a busy engine
      repeat (5) @(posedge clk);
      check(r_fire_rd === 1'b0, "host read is held while the engine is busy");
      // the persist op completes
      sd_done <= 1; @(posedge clk); sd_done <= 0;
      check(f_done === 1'b1 || dut.own_fs === 1'b1, "persist sees its own done");
      @(posedge clk);
      check(r_done === 1'b0, "the persist's done NEVER reaches the router");
      engine_ready <= 1; @(posedge clk);
      check(r_fire_rd === 1'b1, "the latched host read fires the moment the engine frees");
      engine_ready <= 0; @(posedge clk);
      check(own_fs === 1'b0, "and the new op is owned by the router");
      sd_done <= 1; @(posedge clk); sd_done <= 0;
      check(r_done === 1'b1 || !own_fs, "the host read's done routes to the router");
      @(posedge clk);
      check(f_done === 1'b0, "and not to fat32");
      engine_ready <= 1; @(posedge clk);

      // 2: priority. Both want the idle engine: the router wins, fat32 waits.
      r_wr <= 1; f_req <= 1; @(posedge clk); r_wr <= 0;
      check(r_fire_wr === 1'b1, "router write beats the persist to the engine");
      check(f_fire === 1'b0, "the persist waits");
      engine_ready <= 0; @(posedge clk);
      check(own_fs === 1'b0, "router owns the write");
      sd_done <= 1; @(posedge clk); sd_done <= 0;
      engine_ready <= 1; @(posedge clk);
      check(f_fire === 1'b1, "the persist gets the engine after the host is done");
      f_req <= 0; engine_ready <= 0; @(posedge clk);
      check(own_fs === 1'b1, "and owns its op");
      sd_err <= 1; @(posedge clk);
      check(f_err === 1'b1 && r_err === 1'b0, "errors route by owner too");
      sd_err <= 0; sd_done <= 1; @(posedge clk); sd_done <= 0;
      engine_ready <= 1; @(posedge clk);

      if (fails == 0)
        $display("PASS: SD completions route by ownership and no request is dropped");
      else
        $display("FAIL: %0d problem(s)", fails);
      $finish;
   end
   initial begin #100000; $display("TIMEOUT"); $finish; end
endmodule
