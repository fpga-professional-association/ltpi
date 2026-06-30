# =============================================================================
# ltpi.sdc  -  Timing constraints for an LTPI endpoint (SCM or HPM)
#
# Single-clock source-synchronous build: the whole core (and the IO shim) runs
# on `clk` (the LTPI bit clock).  The incoming serial bit (rx_dat) and reset are
# asynchronous and cross into `clk` through the 2-FF synchronizer in the IO shim,
# so they are cut.  tx_dat/tx_clk are launched by `clk`.  Application I/O
# (GPIO/UART/I2C/Avalon/CSR) is given placeholder budgets (~25% of the period).
#
# Constrained at the LTPI base bit rate 25 MHz (40 ns) -- the spec minimum and
# the committed sign-off point; the design closes here with wide margin.
# Internal-logic Fmax is ~68 MHz (Cyclone 10 GX, slow 900mV 0C), limited by the
# combinational CRC-8 fold and 8b/10b decode clouds; pipeline those for the
# higher operational bit clocks (spec supports up to 1 GHz with DDR).
# =============================================================================
create_clock -name clk -period 40.000 [get_ports clk]
derive_clock_uncertainty

# Asynchronous serial RX + forwarded RX clock (metastability handled by the 2-FF
# synchronizer in ltpi_io_altera, not by STA) and the async reset: cut them.
set_false_path -from [get_ports {rx_dat rx_clk rst_n}] -to [all_registers]

# tx_dat / tx_clk are source-synchronous outputs launched by clk; give a budget.
set_output_delay -clock clk -max 2.5 [get_ports {tx_dat tx_clk}]
set_output_delay -clock clk -min 0.5 [get_ports {tx_dat tx_clk}]

# -----------------------------------------------------------------------------
# Application interface (clk domain): GPIO, UART, I2C, Avalon-MM, CSR.
# Placeholder budgets (~25% of the period each way); set to real board numbers
# for sign-off.  -nowarn keeps the script clean if a port was optimized away.
# -----------------------------------------------------------------------------
set APP_IN  [get_ports -nowarn {ll_in[*] nl_in[*] txd0 txd1 rts0 rts1 \
            i2c_evt_det[*] i2c_evt_code[*] i2c_regen_done[*] \
            ini_req ini_write ini_addr[*] ini_wdata[*] ini_be[*] ini_tag[*] \
            avm_readdata[*] avm_waitrequest avm_readdatavalid \
            csr_addr[*] csr_read csr_write csr_wdata[*]}]
set APP_OUT [get_ports -nowarn {ll_out[*] nl_out[*] rxd0 rxd1 cts0 cts1 \
            i2c_regen[*] i2c_regen_code[*] i2c_scl_stretch[*] \
            ini_ack ini_cpl ini_rdata[*] ini_status[*] \
            avm_address[*] avm_read avm_write avm_writedata[*] avm_byteenable[*] \
            csr_rdata[*] csr_ready link_state[*] operational link_aligned}]

set_input_delay  -clock clk -max 2.5 $APP_IN
set_input_delay  -clock clk -min 0.5 $APP_IN
set_output_delay -clock clk -max 2.5 $APP_OUT
set_output_delay -clock clk -min 0.5 $APP_OUT
