// Vendor LVDS I/O + DDR register wrapper for the LTPI link.
//
// Select the implementation with a compile-time define:
//   +define+LTPI_VENDOR_ALTERA   Intel/Altera (Cyclone 10 GX/LP, Arria,
//                                Agilex) - ALTDDIO megafunction cells
//   +define+LTPI_VENDOR_LATTICE  Lattice (ECP5, MachXO3/XO5, Avant) -
//                                ODDRX1F / IDDRX1F primitives
//   (neither)                    Generic behavioral model, used by
//                                simulation and formal runs
//
// Speed grades supported by this project's capability word (CAPS_DEFAULT):
//   200 MHz SDR -> 200 Mbps   any LVDS-capable bank on both vendors
//   400 MHz SDR -> 400 Mbps   Altera: any GX device / fast LP banks;
//                             Lattice: ECP5/Avant (MachXO3 tops out lower)
//   400 MHz DDR -> 800 Mbps   Altera: Cyclone 10 GX+ dedicated LVDS SERDES;
//                             Lattice: ECP5-5G / Avant with DELAYG-tuned
//                             IDDRX1F capture. Run timing closure - this is
//                             at the edge of generic DDR I/O on slow grades.
//
// The LVDS buffers themselves are instantiated by pin assignment (I/O
// standard "LVDS" / "LVDS25") - only the DDR registers appear as cells.

module ltpi_lvds_io (
    input  logic       clk,        // link clock from PLL (25..400 MHz)
    input  logic       rst,
    input  logic       ddr_mode,
    // TX from ltpi_phy_tx
    input  logic       tx_sdr,
    input  logic [1:0] tx_ddr,
    output logic       lvds_tx,    // to LVDS output buffer / pin pair
    // RX to ltpi_phy_rx
    input  logic       lvds_rx,    // from LVDS input buffer / pin pair
    output logic       rx_sdr,
    output logic [1:0] rx_ddr
);

`ifdef LTPI_VENDOR_ALTERA
    // ------------------------------------------------------------------
    logic tx_ddr_out;
    altddio_out #(
        .width(1), .oe_reg("UNREGISTERED")
    ) u_oddr (
        .outclock(clk),
        .datain_h(ddr_mode ? tx_ddr[0] : tx_sdr),  // rising-edge bit
        .datain_l(ddr_mode ? tx_ddr[1] : tx_sdr),  // falling-edge bit
        .dataout(tx_ddr_out),
        .aclr(rst), .aset(1'b0), .oe(1'b1),
        .outclocken(1'b1), .sclr(1'b0), .sset(1'b0)
    );
    assign lvds_tx = tx_ddr_out;

    logic rx_h, rx_l;
    altddio_in #(
        .width(1)
    ) u_iddr (
        .inclock(clk),
        .datain(lvds_rx),
        .dataout_h(rx_h),     // captured on rising edge
        .dataout_l(rx_l),     // captured on falling edge
        .aclr(rst), .aset(1'b0), .inclocken(1'b1),
        .sclr(1'b0), .sset(1'b0)
    );
    assign rx_ddr = {rx_l, rx_h};
    assign rx_sdr = rx_h;

`elsif LTPI_VENDOR_LATTICE
    // ------------------------------------------------------------------
    logic tx_q;
    ODDRX1F u_oddr (
        .SCLK(clk), .RST(rst),
        .D0(ddr_mode ? tx_ddr[0] : tx_sdr),        // rising-edge bit
        .D1(ddr_mode ? tx_ddr[1] : tx_sdr),        // falling-edge bit
        .Q(tx_q)
    );
    assign lvds_tx = tx_q;

    logic rx_q0, rx_q1, rx_dly;
    DELAYG #(.DEL_MODE("SCLK_CENTERED")) u_dly (.A(lvds_rx), .Z(rx_dly));
    IDDRX1F u_iddr (
        .SCLK(clk), .RST(rst),
        .D(rx_dly),
        .Q0(rx_q0),           // captured on rising edge
        .Q1(rx_q1)            // captured on falling edge
    );
    assign rx_ddr = {rx_q1, rx_q0};
    assign rx_sdr = rx_q0;

`else
    // ------------------------------------------------------------------
    // Generic behavioral model (simulation / formal): the "wire" carries
    // one SDR bit or a 2-bit DDR pair per clk; a same-cycle passthrough
    // keeps the byte-level timing identical to the vendor cells for the
    // purposes of the loopback simulation.
    assign lvds_tx = ddr_mode ? tx_ddr[0] : tx_sdr;
    assign rx_sdr  = lvds_rx;
    assign rx_ddr  = {1'b0, lvds_rx};
    // NOTE: the behavioral single-wire model cannot carry a true DDR pair;
    // simulations exercising DDR connect ser_ddr directly between PHYs
    // (see sim/tb_ltpi_system.sv).
`endif

endmodule
