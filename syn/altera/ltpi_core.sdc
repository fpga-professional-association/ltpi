# =============================================================================
# ltpi_core.sdc  -  Timing for the symbol-parallel LTPI core (single clock).
#
# The core is one clock domain: `clk` is the System Clock = the parallel SYMBOL
# clock = line_rate / 10 (J=10 SERDES).  For the committed 400 MHz DDR (800 Mbps)
# operational rate the parallel clock is 80 MHz; constrain at 100 MHz (10 ns) to
# show headroom.  The async CDC and the high-rate bit clocks live in the PHY
# (ltpi_phy), not here.  Application I/O is virtual-pinned with placeholder
# budgets.
# =============================================================================
create_clock -name clk -period 10.000 [get_ports clk]
derive_clock_uncertainty

set_false_path -from [get_ports rst_n] -to [all_registers]

# all application/symbol I/O is registered into/out of the clk domain
set ALL_IN  [get_ports -nowarn {tx_sym_ready rx_sym[*] rx_sym_valid rx_aligned \
            ll_in[*] nl_in[*] txd0 txd1 rts0 rts1 \
            i2c_evt_det[*] i2c_evt_code[*] i2c_regen_done[*] \
            ini_req ini_write ini_addr[*] ini_wdata[*] ini_be[*] ini_tag[*] \
            avm_readdata[*] avm_waitrequest avm_readdatavalid \
            csr_addr[*] csr_read csr_write csr_wdata[*]}]
set ALL_OUT [get_ports -nowarn {tx_sym[*] tx_sym_valid realign speed_change op_speed[*] op_ddr \
            ll_out[*] nl_out[*] rxd0 rxd1 cts0 cts1 \
            i2c_regen[*] i2c_regen_code[*] i2c_scl_stretch[*] \
            ini_ack ini_cpl ini_rdata[*] ini_status[*] \
            avm_address[*] avm_read avm_write avm_writedata[*] avm_byteenable[*] \
            csr_rdata[*] csr_ready link_state[*] operational link_aligned}]

set_input_delay  -clock clk -max 2.0 $ALL_IN
set_input_delay  -clock clk -min 0.3 $ALL_IN
set_output_delay -clock clk -max 2.0 $ALL_OUT
set_output_delay -clock clk -min 0.3 $ALL_OUT
