// uart_rx: 8N1 receiver for the debug UART's other direction.

`default_nettype none

module uart_rx #(
                 parameter integer CLK_HZ = 50_000_000,
                 parameter integer BAUD   = 115_200
                 ) (
                    input  wire       clk,
                    input  wire       rst,
                    input  wire       rx,
                    output reg  [7:0] data,
                    output reg        stb
                    );

   localparam integer DIV = CLK_HZ / BAUD;

   reg [1:0]          sync;
   always @(posedge clk) sync <= {sync[0], rx};
   wire line = sync[1];

   localparam S_IDLE = 2'd0, S_START = 2'd1, S_BITS = 2'd2, S_STOP = 2'd3;
   reg [1:0]  st;
   reg [15:0] cnt;
   reg [2:0]  bit_i;
   reg [7:0]  shift;

   always @(posedge clk) begin
      stb <= 1'b0;
      if (rst) begin
         st <= S_IDLE; cnt <= 0; bit_i <= 0; shift <= 0; data <= 0;
      end else case (st)
                 S_IDLE: if (!line) begin           // start edge
                    cnt <= DIV[15:0] / 16'd2;        // aim for the middle of the start bit
                    st  <= S_START;
                 end
                 S_START: begin
                    if (cnt == 0) begin
                       if (!line) begin               // still low: a real start bit
                          cnt <= DIV[15:0]; bit_i <= 0; st <= S_BITS;
                       end else st <= S_IDLE;         // glitch
                    end else cnt <= cnt - 16'd1;
                 end
                 S_BITS: begin
                    if (cnt == 0) begin
                       shift <= {line, shift[7:1]};   // LSB first
                       cnt   <= DIV[15:0];
                       if (bit_i == 3'd7) st <= S_STOP;
                       else bit_i <= bit_i + 3'd1;
                    end else cnt <= cnt - 16'd1;
                 end
                 S_STOP: begin
                    if (cnt == 0) begin
                       if (line) begin                // valid stop bit
                          data <= shift;
                          stb  <= 1'b1;
                       end
                       st <= S_IDLE;
                    end else cnt <= cnt - 16'd1;
                 end
               endcase
   end

endmodule

`default_nettype wire
