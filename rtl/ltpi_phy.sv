// =============================================================================
// ltpi_phy.sv  -  LTPI dual-rate serial PHY (spec Sec 2.3 / Fig 19)
//
// Bridges the System-Clock symbol-parallel core to the LVDS serial pads through
// two asynchronous CDC FIFOs and a J=10 SERDES:
//
//   core(sys_clk) --tx_sym--> [TX CDC FIFO] --(tx_bit_clk)--> serializer --> tx_dat
//                                                              tx_clk = forwarded bit clock
//   rx_dat --> deserializer+comma bit-align --(rx_bit_clk)--> [RX CDC FIFO] --(sys_clk)--> core
//
// The PHY logic is RATE-AGNOSTIC: the line rate lives entirely in the bit-clock
// frequencies (base 25 Mbps SDR -> operational 800 Mbps = 400 MHz DDR), set by
// the IOPLL (Quartus) or the testbench (sim).  A `realign` pulse re-acquires
// comma/word alignment after a PLL relock at the speed switch.
//
// The bit-level serializer/deserializer here are a synthesizable BEHAVIORAL model
// of the vendor LVDS SERDES, used for vendor-neutral simulation; for real silicon
// at 800 Mbps (an 800 MHz bit clock exceeds the ~644 MHz fabric limit) the SERDES
// hard block (LVDS SERDES Intel FPGA IP, J=10, External-PLL) replaces them under
// `ifdef LTPI_ALTERA_SERDES` — the symbol/CDC boundary is unchanged.
// =============================================================================
`ifndef LTPI_PHY_SV
`define LTPI_PHY_SV
`include "ltpi_ser.sv"
`include "ltpi_deser.sv"
`include "ltpi_cdc_fifo.sv"

module ltpi_phy #(
  parameter int FIFO_AW = 5    // CDC FIFO depth = 2^AW symbols
) (
  // ---- core / system-clock domain ----
  input  logic        sys_clk,
  input  logic        sys_rst_n,
  input  logic [9:0]  tx_sym,
  input  logic        tx_sym_valid,
  output logic        tx_sym_ready,
  output logic [9:0]  rx_sym,
  output logic        rx_sym_valid,
  input  logic        realign,       // re-acquire comma alignment (after relock)
  output logic        rx_aligned,

  // ---- PHY serial clock domains (from IOPLL / testbench) ----
  input  logic        tx_bit_clk,
  input  logic        tx_bit_rst_n,
  input  logic        rx_bit_clk,    // = recovered/forwarded RX clock (rx_clk pad)
  input  logic        rx_bit_rst_n,

  // ---- LVDS serial pads ----
  output logic        tx_dat,
  output logic        tx_clk,        // forwarded bit clock
  input  logic        rx_dat
);
  localparam logic [9:0] K28_5 = 10'b0011111010;  // RD- comma (idle filler)

  // ======================= TX path =======================
  logic        txf_empty, txf_rd;
  logic [9:0]  txf_q;
  logic        ser_adv;

  ltpi_cdc_fifo #(.W(10), .AW(FIFO_AW)) u_txfifo (
    .wclk(sys_clk),    .wrst_n(sys_rst_n), .wr_en(tx_sym_valid & tx_sym_ready),
    .wdata(tx_sym),    .wfull(txf_full),
    .rclk(tx_bit_clk), .rrst_n(tx_bit_rst_n), .rd_en(txf_rd),
    .rdata(txf_q),     .rempty(txf_empty)
  );
  logic txf_full;
  assign tx_sym_ready = ~txf_full;

  // serialize the FIFO head (or an idle comma when empty), one symbol/window
  logic [9:0] ser_sym;
  assign ser_sym = txf_empty ? K28_5 : txf_q;
  assign txf_rd  = ser_adv & ~txf_empty;   // pull next symbol at the window end

  ltpi_ser u_ser (
    .clk(tx_bit_clk), .rst_n(tx_bit_rst_n), .en(1'b1),
    .symbol(ser_sym), .tx_bit(tx_dat), .sym_advance(ser_adv)
  );
  // Forward an edge-shifted clock so the far end samples mid-bit: tx_dat is
  // launched on tx_bit_clk's rising edge, so forwarding the inverted clock makes
  // the RX sample on the falling edge (center of the bit).  A real PHY uses the
  // 90-degree LTPI CLK (spec Fig 19); 180 deg is the behavioral stand-in.
  assign tx_clk = ~tx_bit_clk;             // source-synchronous forwarded clock

  // ======================= RX path =======================
  // sync the realign request into the rx_bit_clk domain (2-FF)
  logic ra_m1, ra_m2;
  always_ff @(posedge rx_bit_clk or negedge rx_bit_rst_n)
    if (!rx_bit_rst_n) {ra_m2, ra_m1} <= 2'b0;
    else               {ra_m2, ra_m1} <= {ra_m1, realign};

  logic [9:0] des_sym;
  logic       des_v, des_aligned;
  ltpi_deser u_deser (
    .clk(rx_bit_clk), .rst_n(rx_bit_rst_n), .en(1'b1),
    .rx_bit(rx_dat), .realign(ra_m2),
    .symbol(des_sym), .sym_valid(des_v), .aligned(des_aligned)
  );

  logic rxf_empty;
  ltpi_cdc_fifo #(.W(10), .AW(FIFO_AW)) u_rxfifo (
    .wclk(rx_bit_clk), .wrst_n(rx_bit_rst_n), .wr_en(des_v & des_aligned),
    .wdata(des_sym),   .wfull(/*unused: depth >= burst*/),
    .rclk(sys_clk),    .rrst_n(sys_rst_n), .rd_en(~rxf_empty),
    .rdata(rx_sym),    .rempty(rxf_empty)
  );
  assign rx_sym_valid = ~rxf_empty;

  // sync rx alignment status into the system-clock domain (2-FF)
  logic al_m1, al_m2;
  always_ff @(posedge sys_clk or negedge sys_rst_n)
    if (!sys_rst_n) {al_m2, al_m1} <= 2'b0;
    else            {al_m2, al_m1} <= {al_m1, des_aligned};
  assign rx_aligned = al_m2;
endmodule
`endif
