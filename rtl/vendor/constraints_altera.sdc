# LTPI timing constraints - Intel/Altera (Quartus), Cyclone 10 GX example.
# Link clock from IOPLL; pick ONE of the three supported operating points.

# --- 200 MHz SDR (200 Mbps) ---
# create_clock -name ltpi_clk -period 5.000 [get_ports LTPI_RX_CLK]

# --- 400 MHz SDR (400 Mbps) ---
# create_clock -name ltpi_clk -period 2.500 [get_ports LTPI_RX_CLK]

# --- 400 MHz DDR (800 Mbps) --- (default)
create_clock -name ltpi_clk -period 2.500 [get_ports LTPI_RX_CLK]

derive_pll_clocks
derive_clock_uncertainty

# Source-synchronous RX: center-aligned capture, both edges in DDR mode.
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
