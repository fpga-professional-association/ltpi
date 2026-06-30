// =============================================================================
// ltpi_deser.sv  -  serial bit -> 10-bit symbol, with comma bit-alignment
//
// Incoming bits arrive 'a' (MSB) first.  A free-running mod-10 phase counter
// marks symbol boundaries.  The unit locks bit-alignment ONCE, the first time
// the 7-bit comma sequence (0011111 / 1100000, present in K28.5/.1/.7 — the
// Link Detect comma) appears, and then holds.  It does NOT re-align on every
// comma: data symbols can coincidentally contain the comma pattern, which would
// otherwise re-frame mid-stream and corrupt CRC.  A `realign` pulse force-unlocks
// to re-acquire (real multi-rate PHYs use this after a PLL relock with a freshly
// recovered bit clock; the single-clock model here never needs it).
//
// The comma occupies bits a..f = the first 7 bits of a symbol, so when the
// just-shifted 7 LSBs equal the comma, we are 7 bits (index 6) into the symbol;
// 3 more bits complete it.  Setting phase := 6 makes sym_valid fire exactly when
// the full symbol (a in bit 9) is assembled.
//
// Verified by formal (formal/ltpi_deser.sby): after a comma, sym_valid frames
// symbols on the correct boundary and `aligned` latches.
// =============================================================================
`ifndef LTPI_DESER_SV
`define LTPI_DESER_SV

module ltpi_deser (
  input  logic       clk,
  input  logic       rst_n,
  input  logic       en,            // 1 = a valid serial bit this cycle
  input  logic       rx_bit,
  input  logic       realign,       // 1-cycle pulse: force-unlock to re-acquire
  output logic [9:0] symbol,
  output logic       sym_valid,     // 1 for one cycle when `symbol` is complete
  output logic       aligned        // symbol boundary has been found
);
  // Comma = first 7 symbol bits (a..f) in time order, oldest at MSB.  The 7 most
  // recent bits after this cycle's shift are cur7 = {sr[5:0], rx_bit}.
  localparam logic [6:0] COMMA_P = 7'b0011111;
  localparam logic [6:0] COMMA_N = 7'b1100000;
  localparam logic [3:0] ALIGN_PHASE = 4'd7;  // f just shifted -> g is phase 7 next

  logic [9:0] sr;
  logic [3:0] phase;
  logic [6:0] cur7;
  logic       comma_seen;

  assign cur7       = {sr[5:0], rx_bit};
  assign comma_seen = (cur7 == COMMA_P) || (cur7 == COMMA_N);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sr        <= 10'd0;
      phase     <= 4'd0;
      aligned   <= 1'b0;
      symbol    <= 10'd0;
      sym_valid <= 1'b0;
    end else if (en) begin
      sr        <= {sr[8:0], rx_bit};
      sym_valid <= 1'b0;
      if (realign) begin
        aligned <= 1'b0;             // force-unlock; re-acquire on next comma
      end else if (!aligned && comma_seen) begin
        phase   <= ALIGN_PHASE;      // lock once on the Detect comma
        aligned <= 1'b1;
      end else if (phase == 4'd9) begin
        phase     <= 4'd0;
        symbol    <= {sr[8:0], rx_bit};   // a..j with a in bit 9
        sym_valid <= aligned;
      end else begin
        phase <= phase + 4'd1;
      end
    end
  end
endmodule
`endif
