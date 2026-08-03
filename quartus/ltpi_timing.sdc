# LTPI fabric timing target: 400 MHz link clock - the top operating point
# of the soft PHY (400 Mbps SDR / 800 Mbps DDR at the pins).
create_clock -name ltpi_clk -period 2.500 [get_ports clk]
derive_clock_uncertainty
set_false_path -from [get_ports rst]
set_false_path -from [get_ports ddr_mode]
# All data ports are virtual pins - this SDC targets pure fabric closure.
# Pin-level (source-synchronous DDR) budgets live in rtl/vendor/*.sdc.
