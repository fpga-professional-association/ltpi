# LTPI timing constraints - Intel/Altera (Quartus).
# Pick ONE operating-point block. Part guidance per speed:
#
#   OPERATING POINT      LINK CLK  BITRATE   PARTS THAT CLOSE IT
#   -----------------    --------  -------   ---------------------------------
#   25  MHz SDR (base)    25 MHz   25 Mbps   any LVDS-capable device
#   200 MHz SDR          200 MHz   200 Mbps  MAX10, Cyclone 10 LP/GX, Arria,
#                                            Agilex - generic DDIO I/O
#   400 MHz SDR          400 MHz   400 Mbps  Cyclone 10 GX, Arria 10, Agilex
#                                            (fast I/O banks; C10LP marginal)
#   400 MHz DDR          400 MHz   800 Mbps  Cyclone 10 GX+ (true-LVDS DDIO),
#                                            Arria 10, Agilex
#   600 MHz DDR          600 MHz   1.2 Gbps  Cyclone 10 GX / Arria 10 with the
#                                            dedicated LVDS SERDES (SERDES
#                                            factor >= 4, not plain DDIO)
#   1 GHz  DDR (spec max) 1 GHz    2 Gbps    Arria 10 / Agilex LVDS SERDES
#                                            only (C10GX tops out ~1.434 Gbps
#                                            = ~700 MHz DDR)
#
# Above 400 MHz DDR the soft ltpi_phy_* serializers must be replaced by the
# device's dedicated LVDS SERDES (see repo issue #2) - plain fabric DDIO
# does not close there. Rates through 400 MHz DDR use ltpi_lvds_io.sv as-is.

# --- 200 MHz SDR (200 Mbps) ---
# create_clock -name ltpi_clk -period 5.000 [get_ports LTPI_RX_CLK]

# --- 400 MHz SDR (400 Mbps) ---
# create_clock -name ltpi_clk -period 2.500 [get_ports LTPI_RX_CLK]

# --- 400 MHz DDR (800 Mbps) --- (default)
create_clock -name ltpi_clk -period 2.500 [get_ports LTPI_RX_CLK]

# --- 600 MHz DDR (1.2 Gbps, LVDS SERDES parts only) ---
# create_clock -name ltpi_clk -period 1.667 [get_ports LTPI_RX_CLK]

# --- 1 GHz DDR (2 Gbps, spec maximum; Arria 10 / Agilex SERDES) ---
# create_clock -name ltpi_clk -period 1.000 [get_ports LTPI_RX_CLK]

derive_pll_clocks
derive_clock_uncertainty

# Source-synchronous RX: center-aligned capture, both edges in DDR mode.
# Scale the +/- window to a quarter bit-time of the chosen rate:
#   400M DDR: +/-0.625ns   600M DDR: +/-0.417ns   1G DDR: +/-0.250ns
set_input_delay -clock ltpi_clk -max 0.625 [get_ports LTPI_RX_DAT]
set_input_delay -clock ltpi_clk -min -0.625 [get_ports LTPI_RX_DAT]
set_input_delay -clock ltpi_clk -max 0.625 [get_ports LTPI_RX_DAT] -clock_fall -add_delay
set_input_delay -clock ltpi_clk -min -0.625 [get_ports LTPI_RX_DAT] -clock_fall -add_delay

# TX: clock forwarded with data (edge-aligned at pins, RX PHY re-centers).
set_output_delay -clock ltpi_clk -max 0.5 [get_ports LTPI_TX_DAT]
set_output_delay -clock ltpi_clk -min -0.5 [get_ports LTPI_TX_DAT]
set_output_delay -clock ltpi_clk -max 0.5 [get_ports LTPI_TX_DAT] -clock_fall -add_delay
set_output_delay -clock ltpi_clk -min -0.5 [get_ports LTPI_TX_DAT] -clock_fall -add_delay

# I/O standard: assign LVDS on the pin pairs in the .qsf:
#   set_instance_assignment -name IO_STANDARD LVDS -to LTPI_TX_DAT
#   set_instance_assignment -name IO_STANDARD LVDS -to LTPI_RX_DAT
#   set_instance_assignment -name IO_STANDARD LVDS -to LTPI_RX_CLK
