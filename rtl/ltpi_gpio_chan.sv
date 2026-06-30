// =============================================================================
// ltpi_gpio_chan.sv  -  GPIO channel tunneling (spec Sec 2.2.1.1, Table 6)
//
// Low-Latency GPIOs: 16 bits, refreshed every I/O frame (frame bytes 3,4).
// Normal-Latency GPIOs: NUM_NL bits, time-division multiplexed 16/frame across
// N = ceil(NUM_NL/16) frames; the window for a frame is selected by the frame
// counter X = (FrameCounter % N) * 16 (frame bytes 5,6).
//
// On a CRC-bad frame the receiver holds the previous GPIO states (spec Table 14):
// the core only pulses rx_valid for good frames, so this module just registers
// on rx_valid and naturally holds otherwise.
// =============================================================================
`ifndef LTPI_GPIO_CHAN_SV
`define LTPI_GPIO_CHAN_SV

module ltpi_gpio_chan #(
  parameter int NUM_NL = 32,
  parameter int N      = (NUM_NL + 15) / 16   // frames per full NL refresh
) (
  input  logic                clk,
  input  logic                rst_n,

  // frame counter (byte2 of the I/O frame), maintained by the core
  input  logic [7:0]          frame_cnt,

  // ---- local GPIO I/O ----
  input  logic [15:0]         ll_in,          // local LL GPIO inputs (to tunnel)
  output logic [15:0]         ll_out,          // recovered remote LL GPIOs
  input  logic [NUM_NL-1:0]   nl_in,
  output logic [NUM_NL-1:0]   nl_out,

  // ---- TX bytes into the I/O frame (bytes 3,4,5,6) ----
  output logic [15:0]         tx_ll,           // {byte4, byte3}
  output logic [15:0]         tx_nl,           // {byte6, byte5} = current NL window

  // ---- RX bytes from a good I/O frame ----
  input  logic                rx_valid,        // good I/O frame this cycle
  input  logic [7:0]          rx_frame_cnt,
  input  logic [15:0]         rx_ll,
  input  logic [15:0]         rx_nl
);
  // window index for a given frame counter
  function automatic int unsigned win_of(input logic [7:0] fc);
    win_of = (N <= 1) ? 0 : (fc % N);
  endfunction

  // ---- TX: LL every frame; NL window selected by the (local) frame counter ----
  assign tx_ll = ll_in;
  always @(*) begin
    int unsigned w;
    tx_nl = 16'd0;
    w = win_of(frame_cnt);
    for (int b = 0; b < 16; b++)
      if (w*16 + b < NUM_NL) tx_nl[b] = nl_in[w*16 + b];
  end

  // ---- RX: register LL each good frame; place NL window per received counter ----
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ll_out <= '0;
      nl_out <= '0;
    end else if (rx_valid) begin
      int unsigned w;
      ll_out <= rx_ll;
      w = win_of(rx_frame_cnt);
      for (int b = 0; b < 16; b++)
        if (w*16 + b < NUM_NL) nl_out[w*16 + b] <= rx_nl[b];
    end
  end

`ifdef FORMAL
  logic fpv = 1'b0;
  always_ff @(posedge clk) fpv <= 1'b1;
  always @(posedge clk) begin
    a_tx_ll: assert (tx_ll == ll_in);                       // LL is sent verbatim
    if (fpv && $past(rst_n) && rst_n) begin
      if ($past(rx_valid))
        a_ll_roundtrip: assert (ll_out == $past(rx_ll));    // LL recovered next cycle
      else
        a_ll_hold: assert (ll_out == $past(ll_out));        // hold on CRC-bad frame
    end
  end
`endif
endmodule
`endif
