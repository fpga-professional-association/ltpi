// =============================================================================
// ltpi_cdc_fifo.sv  -  dual-clock asynchronous FIFO (Gray-pointer, Cummings)
//
// Standard two-clock FIFO: binary pointers converted to Gray and double-FF
// synchronized into the opposite domain; full/empty are REGISTERED (computed
// from the next-pointer values) so there is no combinational pointer loop.
// Bridges the LTPI System-Clock core and the SERDES/LTPI-CLK PHY domains (spec
// Fig 19): one 10-bit 8b/10b symbol per entry, on TX (sys->serdes) and RX
// (serdes->sys).
//
// Verified by formal (formal/ltpi_cdc_fifo.sby): never writes when full, never
// reads when empty, occupancy stays within [0, DEPTH].
// =============================================================================
`ifndef LTPI_CDC_FIFO_SV
`define LTPI_CDC_FIFO_SV

module ltpi_cdc_fifo #(
  parameter int W  = 10,   // data width
  parameter int AW = 4     // address width -> depth = 2^AW
) (
  input  logic          wclk,
  input  logic          wrst_n,
  input  logic          wr_en,
  input  logic [W-1:0]  wdata,
  output logic          wfull,
  input  logic          rclk,
  input  logic          rrst_n,
  input  logic          rd_en,
  output logic [W-1:0]  rdata,
  output logic          rempty
);
  localparam int DEPTH = (1 << AW);
  logic [W-1:0] mem [0:DEPTH-1];

  // ---- write domain ----
  logic [AW:0] wbin, wgray;
  logic [AW:0] wbin_n, wgray_n;
  logic [AW:0] rgray_w1, rgray_w2;     // read gray synced into write domain

  assign wbin_n  = wbin + (wr_en & ~wfull);
  assign wgray_n = (wbin_n >> 1) ^ wbin_n;

  always_ff @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n) begin wbin <= '0; wgray <= '0; wfull <= 1'b0; end
    else begin
      wbin  <= wbin_n;
      wgray <= wgray_n;
      // full when next write gray == read gray with the top two bits inverted
      wfull <= (wgray_n == {~rgray_w2[AW:AW-1], rgray_w2[AW-2:0]});
    end
  end
  always_ff @(posedge wclk) if (wr_en & ~wfull) mem[wbin[AW-1:0]] <= wdata;

  always_ff @(posedge wclk or negedge wrst_n)
    if (!wrst_n) begin rgray_w1 <= '0; rgray_w2 <= '0; end
    else         begin rgray_w1 <= rgray; rgray_w2 <= rgray_w1; end

  // ---- read domain ----
  logic [AW:0] rbin, rgray;
  logic [AW:0] rbin_n, rgray_n;
  logic [AW:0] wgray_r1, wgray_r2;     // write gray synced into read domain

  assign rbin_n  = rbin + (rd_en & ~rempty);
  assign rgray_n = (rbin_n >> 1) ^ rbin_n;

  always_ff @(posedge rclk or negedge rrst_n) begin
    if (!rrst_n) begin rbin <= '0; rgray <= '0; rempty <= 1'b1; end
    else begin
      rbin   <= rbin_n;
      rgray  <= rgray_n;
      rempty <= (rgray_n == wgray_r2);   // empty when next read gray catches write
    end
  end
  assign rdata = mem[rbin[AW-1:0]];

  always_ff @(posedge rclk or negedge rrst_n)
    if (!rrst_n) begin wgray_r1 <= '0; wgray_r2 <= '0; end
    else         begin wgray_r1 <= wgray; wgray_r2 <= wgray_r1; end
endmodule
`endif
