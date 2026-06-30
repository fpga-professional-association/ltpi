// =============================================================================
// ltpi_ser.sv  -  10-bit symbol -> serial bit (MSB/'a' first)
//
// The upstream (frame_tx) holds `symbol` stable for the whole 10-cycle window
// and advances to the next symbol when `sym_advance` pulses (on the last bit).
// Bit order matches ltpi_deser: word[9] ('a') goes out first.
// =============================================================================
`ifndef LTPI_SER_SV
`define LTPI_SER_SV

module ltpi_ser (
  input  logic       clk,
  input  logic       rst_n,
  input  logic       en,            // 1 = advance one bit this cycle
  input  logic [9:0] symbol,        // current symbol (stable across the window)
  output logic       tx_bit,
  output logic       sym_advance    // pulses on the final bit of the symbol
);
  logic [3:0] cnt;   // bit index within symbol, 0..9

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)      cnt <= 4'd0;
    else if (en)     cnt <= (cnt == 4'd9) ? 4'd0 : cnt + 4'd1;
  end

  assign tx_bit      = symbol[9 - cnt];
  assign sym_advance = en & (cnt == 4'd9);
endmodule
`endif
