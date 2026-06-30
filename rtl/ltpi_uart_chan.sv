// =============================================================================
// ltpi_uart_chan.sv  -  UART channel tunneling (spec Sec 2.2.1.2, Tables 7/8)
//
// Two UART channels.  TXD/RXD are oversampled 3x per I/O frame; the three
// samples plus a flow-control bit are packed per channel into a nibble, two
// channels per byte (frame byte 7):
//   nibble = {FLOW, S2, S1, S0}   (TX dir = RTS, RX dir = CTS)
//   tx_byte = {ch1_nibble, ch0_nibble}
//
// The core supplies `sub_tick` (a strobe 3x per frame) and `sub_idx` (0,1,2) so
// the sampling/regeneration cadence tracks the frame rate (spec Fig 14/15).  On
// a CRC-bad frame the previous line state is held (core gates rx_valid).
// =============================================================================
`ifndef LTPI_UART_CHAN_SV
`define LTPI_UART_CHAN_SV

module ltpi_uart_chan (
  input  logic       clk,
  input  logic       rst_n,
  input  logic       sub_tick,     // 3 pulses per I/O frame
  input  logic [1:0] sub_idx,      // 0,1,2 within the frame

  // ---- local UART lines to tunnel (outgoing) ----
  input  logic       txd0, txd1,   // local transmit-data lines (sampled)
  input  logic       rts0, rts1,   // local flow control out
  // ---- recovered remote UART lines (incoming) ----
  output logic       rxd0, rxd1,   // regenerated receive-data lines
  output logic       cts0, cts1,   // recovered remote flow control

  // ---- frame byte 7 ----
  output logic [7:0] tx_byte,
  input  logic       rx_valid,
  input  logic [7:0] rx_byte
);
  // ---- TX: capture 3 samples per channel across the frame ----
  logic [2:0] samp0, samp1;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin samp0 <= 3'b111; samp1 <= 3'b111; end
    else if (sub_tick) begin
      samp0[sub_idx] <= txd0;
      samp1[sub_idx] <= txd1;
    end
  end
  assign tx_byte = {rts1, samp1, rts0, samp0};

  // ---- RX: latch received samples per frame, replay at sub-frame cadence ----
  logic [2:0] rsamp0, rsamp1;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rsamp0 <= 3'b111; rsamp1 <= 3'b111;
      cts0 <= 1'b0; cts1 <= 1'b0;
      rxd0 <= 1'b1; rxd1 <= 1'b1;     // UART idle = high
    end else begin
      if (rx_valid) begin
        rsamp0 <= rx_byte[2:0]; cts0 <= rx_byte[3];
        rsamp1 <= rx_byte[6:4]; cts1 <= rx_byte[7];
      end
      if (sub_tick) begin
        rxd0 <= rsamp0[sub_idx];
        rxd1 <= rsamp1[sub_idx];
      end
    end
  end
endmodule
`endif
