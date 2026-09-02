// uart_tx: 8N1 transmitter, no flow control.

`default_nettype none

module uart_tx #(
                 parameter integer CLK_HZ = 50_000_000,
                 parameter integer BAUD   = 115200
                 ) (
                    input  wire       clk,
                    input  wire       rst,
                    input  wire [7:0] data,
                    input  wire       stb,        // one-cycle pulse: send `data`
                    output reg        busy,
                    output reg        tx
                    );
   localparam integer DIV = CLK_HZ / BAUD;

   reg [15:0]         cnt;
   reg [3:0]          bit_i;
   reg [9:0]          sr;                  // {stop, data, start}

   always @(posedge clk) begin
      if (rst) begin
         tx <= 1'b1; busy <= 1'b0; cnt <= 0; bit_i <= 0; sr <= 10'h3FF;
      end else if (!busy) begin
         tx <= 1'b1;
         if (stb) begin
            sr    <= {1'b1, data, 1'b0};
            bit_i <= 4'd10;
            cnt   <= DIV[15:0] - 1;
            busy  <= 1'b1;
            tx    <= 1'b0;            // start bit immediately
         end
      end else if (cnt != 0) begin
         cnt <= cnt - 1'b1;
      end else begin
         cnt   <= DIV[15:0] - 1;
         sr    <= {1'b1, sr[9:1]};
         tx    <= sr[1];
         bit_i <= bit_i - 1'b1;
         if (bit_i == 4'd1) busy <= 1'b0;
      end
   end
endmodule

`default_nettype wire
