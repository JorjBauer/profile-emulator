// tb_verifier: decode the verifier's own UART output and check every number.
//
// The verifier's whole job is to produce checksums that a script on a Mac can
// compare against an image file. That makes the CRC definition a CONTRACT
// between two implementations in two languages. Testing it only end to end
// would leave the definition itself unchecked. This bench:
//
//   1. feeds the verifier sectors whose contents it knows,
//   2. decodes the serial line a bit at a time, exactly as a terminal would,
//   3. computes the expected CRC independently, and compares,
//   4. prints fixed vectors that tools/verify_image.js asserts against, so the
//      script and the Verilog cannot drift apart without a test failing.
//
//   iverilog -g2012 -o /tmp/tb -s tb_verifier sim/tb_verifier.v src/verifier.v src/uart_tx.v

`timescale 1ns / 1ps
`default_nettype none

module tb_verifier #(
                     parameter integer STRIDE = 1,
                     parameter integer COUNT  = 4
                     );

   // Slow clock and fast baud purely to keep the run short: what matters is the
   // ratio, and uart_tx divides one by the other.
   localparam integer CLK_HZ = 1_000_000;
   localparam integer BAUD   = 100_000;
   localparam integer TCK    = 1_000_000_000 / CLK_HZ;    // 1000 ns
   localparam integer DIVB   = CLK_HZ / BAUD;             // 10 clocks per bit
   localparam integer BITT   = DIVB * TCK;

   reg clk = 0, rst = 1;
   always #(TCK/2) clk = ~clk;

   reg         fire = 0, sd_done = 0, sd_buf_we = 0;
   reg  [7:0]  sd_buf_wdata = 0;
   wire        req, active, done, tx;
   wire [31:0] lba;

   verifier #(.CLK_HZ(CLK_HZ), .BAUD(BAUD), .START(0),
              .COUNT(COUNT), .STRIDE(STRIDE)) dut (
                                                   .clk(clk), .rst(rst), .enable(1'b1), .go(1'b1),
                                                   .req(req), .fire(fire), .sd_done(sd_done), .sd_err(1'b0),
                                                   .sd_err_code(4'd0), .sd_retries(8'd0),
                                                   .sd_buf_we(sd_buf_we), .sd_buf_wdata(sd_buf_wdata),
                                                   .lba(lba), .active(active), .done(done), .tx(tx));

   integer errors = 0;
   task check(input cond, input [511:0] what);
      begin
         if (cond) $display("    ok    %0s", what);
         else begin $display("*** FAIL  %0s", what); errors = errors + 1; end
      end
   endtask

   // The sector the model card would return for a given LBA. Any deterministic
   // function does; this one changes with both LBA and offset, so a verifier
   // that ignored the address or reused a stale buffer would be caught.
   function [7:0] sect_byte(input [31:0] a, input integer j);
      sect_byte = (a[7:0] * 8'd7) + j[7:0] + a[15:8];
   endfunction

   // CRC16-CCITT, MSB first, init $FFFF, no final xor. THE contract.
   function [15:0] crc_step(input [15:0] c, input [7:0] d);
      integer k;
      reg [15:0] x;
      reg [7:0]  b;
      begin
         x = c; b = d;
         for (k = 0; k < 8; k = k + 1) begin
            x = (x[15] ^ b[7]) ? ((x << 1) ^ 16'h1021) : (x << 1);
            b = {b[6:0], 1'b0};
         end
         crc_step = x;
      end
   endfunction

   // ---- the model card ------------------------------------------------------
   // mj is the model's OWN loop variable. It shares nothing with the checker
   // below: both run concurrently, and a shared integer would mean each one
   // walks the other's counter, which reads exactly like a design bug.
   integer mj;
   reg [31:0] cur;
   initial begin
      forever begin
         wait (req === 1'b1);
         @(posedge clk);
         cur  = lba;
         fire <= 1'b1;
         @(posedge clk);
         fire <= 1'b0;
         repeat (5) @(posedge clk);
         for (mj = 0; mj < 512; mj = mj + 1) begin
            sd_buf_wdata <= sect_byte(cur, mj);
            sd_buf_we    <= 1'b1;
            @(posedge clk);
            sd_buf_we    <= 1'b0;
            // More than eight clocks between bytes, which is what the
            // bit-serial CRC needs and what a real SD transfer gives it many
            // times over.
            repeat (10) @(posedge clk);
         end
         sd_done <= 1'b1;
         @(posedge clk);
         sd_done <= 1'b0;
      end
   end

   // ---- a terminal ----------------------------------------------------------
   task uart_get(output [7:0] b);
      integer k;
      begin
         @(negedge tx);              // start bit
         #(BITT + BITT/2);           // into the middle of bit 0
         for (k = 0; k < 8; k = k + 1) begin
            b[k] = tx;
            #BITT;
         end
      end
   endtask

   // One "AAAAAA:CCCC\n" line, decoded back into two numbers.
   task get_line(output [31:0] a, output [15:0] c);
      integer k;
      reg [7:0] ch;
      reg [3:0] n;
      begin
         a = 0; c = 0;
         for (k = 0; k < 6; k = k + 1) begin
            uart_get(ch);
            n = (ch >= "A") ? (ch - "A" + 8'd10) : (ch - "0");
            a = (a << 4) | n;
         end
         uart_get(ch);
         // ":" is a clean checkpoint, "!" is one whose read failed. This bench
         // never injects an error, so anything but a colon is wrong here.
         if (ch !== ":") begin
            $display("*** separator was $%02h, expected a colon", ch);
            errors = errors + 1;
         end
         for (k = 0; k < 4; k = k + 1) begin
            uart_get(ch);
            n = (ch >= "A") ? (ch - "A" + 8'd10) : (ch - "0");
            c = (c << 4) | n;
         end
         uart_get(ch);
         if (ch !== "\n") begin
            $display("*** terminator was $%02h, expected a newline", ch);
            errors = errors + 1;
         end
      end
   endtask

   reg [31:0] got_lba, want_lba;
   reg [15:0] got_crc, want_crc;
   integer i, s, j, vj;

   initial begin
      repeat (5) @(posedge clk);
      rst = 0;

      $display("-- stride %0d, %0d sectors", STRIDE, COUNT);

      for (i = 0; i < COUNT / STRIDE; i = i + 1) begin
         want_lba = i * STRIDE;
         want_crc = 16'hFFFF;
         for (s = 0; s < STRIDE; s = s + 1)
           for (j = 0; j < 512; j = j + 1)
             want_crc = crc_step(want_crc, sect_byte(want_lba + s, j));

         get_line(got_lba, got_crc);
         if (got_lba === want_lba && got_crc === want_crc)
           $display("    ok    %06h:%04h", got_lba, got_crc);
         else begin
            $display("*** FAIL  got %06h:%04h, wanted %06h:%04h",
                     got_lba, got_crc, want_lba, want_crc);
            errors = errors + 1;
         end
      end

      // ---- fixed vectors, for tools/verify_image.js to assert against ----
      // These lock the JavaScript implementation to this one. If
      // either side ever changes its CRC, one of the two test suites fails
      // immediately instead of the whole volume comparison reporting a mismatch
      // at sector 0.
      begin : vectors
         reg [15:0] v0, v1;
         v0 = 16'hFFFF;
         for (vj = 0; vj < 512; vj = vj + 1) v0 = crc_step(v0, 8'h00);
         v1 = 16'hFFFF;
         for (vj = 0; vj < 512; vj = vj + 1) v1 = crc_step(v1, vj[7:0]);
         $display("-- fixed vectors for the host script");
         $display("    512 bytes of $00      : %04h", v0);
         $display("    512 bytes of j & $FF  : %04h", v1);
      end

      // The run signs off with "#NN", the number of transfers that had to be
      // re-issued. Consume it before asking whether the run is finished, or the
      // check lands while those four characters are still going out.
      begin : summary
         reg [7:0] c0, c1, c2, c3;
         integer   n;
         uart_get(c0); uart_get(c1); uart_get(c2); uart_get(c3);
         check(c0 === "#",  "the run emits a retry summary line");
         check(c3 === "\n", "and terminates it");
         n = ((c1 >= "A") ? (c1 - "A" + 10) : (c1 - "0")) * 16
             + ((c2 >= "A") ? (c2 - "A" + 10) : (c2 - "0"));
         $display("    retries reported: %0d", n);
         check(n == 0, "no retries needed against a well-behaved model card");
      end

      repeat (20) @(posedge clk);
      check(done === 1'b1, "the run reported itself finished");
      check(active === 1'b0, "and released the UART afterward");

      $display("");
      if (errors == 0) $display("PASS: every checksum decoded off the wire matches");
      else             $display("FAIL: %0d problem(s)", errors);
      $finish;
   end

   initial begin
      #500_000_000;
      $display("*** TIMEOUT in verifier state, req %0b active %0b", req, active);
      $finish;
   end

endmodule

`default_nettype wire
