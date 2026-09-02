// tb_exercise: the no-host bus pattern, and the rule that it gets out of the
// way.
//
// Two things are being checked, in this order of importance:
//
//   1. IT RELEASES THE BUS THE MOMENT A HOST APPEARS. This thing exists only
//      because there is nothing else on the wire. The instant that stops being
//      true it is a second driver. That is the only fault on this board that
//      can damage the //e's interface card. A diagnostic is never worth that.
//   2. The pattern is the pattern: a one-hot walk, then the fixed bytes, then
//      BSY moving with the data held still, then the release.
//
//   iverilog -g2012 -o /tmp/tb -s tb_exercise sim/tb_exercise.v src/bus_exercise.v

`timescale 1ns / 1ps
`default_nettype none

module tb_exercise;

   localparam integer CLK_HZ   = 1_000_000;   // short, so the run is short
   localparam integer DWELL_US = 1000;
   localparam integer TCK      = 1_000_000_000 / CLK_HZ;
   localparam integer DWELL    = (CLK_HZ / 1_000_000) * DWELL_US;

   reg clk = 0, rst = 1, inhibit = 0;
   always #(TCK/2) clk = ~clk;

   wire       drive_en, bsy_n;
   wire [7:0] drive_data;
   wire [3:0] step;

   bus_exercise #(.CLK_HZ(CLK_HZ), .DWELL_US(DWELL_US)) dut (
                                                             .clk(clk), .rst(rst), .enable(1'b1), .inhibit(inhibit),
                                                             .drive_en(drive_en), .drive_data(drive_data), .bsy_n(bsy_n),
                                                             .step_out(step));

   integer errors = 0;
   task check(input cond, input [511:0] what);
      begin
         if (cond) $display("    ok    %0s", what);
         else begin $display("*** FAIL  %0s", what); errors = errors + 1; end
      end
   endtask

   // Walk to the middle of step n and sample there, well away from the edges.
   task at_step(input [3:0] n);
      begin
         while (step !== n) @(posedge clk);
         repeat (DWELL / 2) @(posedge clk);
      end
   endtask

   reg [7:0] seen [0:15];
   integer i, changes;

   initial begin
      repeat (5) @(posedge clk);
      rst = 0;

      $display("-- the one-hot walk");
      for (i = 0; i < 8; i = i + 1) begin
         at_step(i[3:0]);
         seen[i] = drive_data;
         if (drive_data === (8'h01 << i) && drive_en === 1'b1)
           $display("    ok    step %0d drives $%02h", i, drive_data);
         else begin
            $display("*** FAIL  step %0d drives $%02h with en=%0b, wanted $%02h en=1",
                     i, drive_data, drive_en, (8'h01 << i));
            errors = errors + 1;
         end
      end

      // Every step of a one-hot walk must differ from the one before it, or a
      // stuck bit reads as a plausible pattern.
      changes = 0;
      for (i = 1; i < 8; i = i + 1) if (seen[i] !== seen[i-1]) changes = changes + 1;
      check(changes == 7, "every one-hot step differs from the one before it");

      $display("-- the fixed patterns");
      at_step(4'd8);  check(drive_data === 8'h55, "step 8 drives $55");
      at_step(4'd9);  check(drive_data === 8'hAA, "step 9 drives $AA");
      at_step(4'd10); check(drive_data === 8'hFF, "step 10 drives $FF");
      at_step(4'd11); check(drive_data === 8'h00, "step 11 drives $00");

      $display("-- BSY, with the data byte deliberately held still");
      at_step(4'd12);
      check(bsy_n === 1'b0 && drive_data === 8'h5A, "step 12 asserts BSY, data $5A");
      at_step(4'd13);
      check(bsy_n === 1'b1 && drive_data === 8'h5A, "step 13 releases BSY, data unchanged");

      $display("-- the release");
      at_step(4'd14);
      check(drive_en === 1'b0, "step 14 lets go of the data bus");

      // ---- the safety property ----
      $display("-- a host appears mid-pattern");
      at_step(4'd2);
      check(drive_en === 1'b1, "driving before the host appears");
      @(posedge clk);
      inhibit = 1'b1;
      @(posedge clk);
      @(posedge clk);
      check(drive_en === 1'b0, "bus released within a cycle of the host appearing");
      check(bsy_n === 1'b1,    "and BSY released with it");

      // And it must STAY off, not resume on the next step boundary.
      repeat (DWELL * 4) @(posedge clk);
      check(drive_en === 1'b0, "and it stays off while the host is there");
      check(step === 4'd0,     "with the pattern held at the start");

      $display("");
      if (errors == 0) $display("PASS: the pattern is right and it yields to a host");
      else             $display("FAIL: %0d problem(s)", errors);
      $finish;
   end

   initial begin
      #200_000_000;
      $display("*** TIMEOUT at step %0d", step);
      $finish;
   end

endmodule

`default_nettype wire
