# Quartus Pro build: quartus_sh -t build.tcl <family> <part> <revname>
# e.g. quartus_sh -t build.tcl "Agilex 3" A3CW135BM16AE6S ltpi_a3
package require ::quartus::project

set family  [lindex $argv 0]
set part    [lindex $argv 1]
set rev     [lindex $argv 2]

project_new -overwrite -revision $rev ltpi_timing

set_global_assignment -name FAMILY $family
set_global_assignment -name DEVICE $part
set_global_assignment -name TOP_LEVEL_ENTITY ltpi_syn_top

foreach f {
    ../rtl/ltpi_pkg.sv
    ../rtl/ltpi_frame_rx.sv
    ../rtl/ltpi_frame_tx.sv
    ../rtl/ltpi_link_fsm.sv
    ../rtl/ltpi_gpio_channel.sv
    ../rtl/ltpi_uart_channel.sv
    ../rtl/ltpi_i2c_relay.sv
    ../rtl/ltpi_data_channel.sv
    ../rtl/ltpi_csr.sv
    ../rtl/ltpi_peer_decode.sv
    ../rtl/ltpi_phy.sv
    ../rtl/ltpi_top.sv
    ltpi_syn_top.sv
} {
    set_global_assignment -name SYSTEMVERILOG_FILE $f
}
set_global_assignment -name SEARCH_PATH ../rtl
set_global_assignment -name SDC_FILE ltpi_timing.sdc
set_global_assignment -name OPTIMIZATION_MODE "SUPERIOR PERFORMANCE WITH MAXIMUM PLACEMENT EFFORT"

# Only clk is a real pin. The serial pins are virtual too: pin-level DDR
# timing is owned by the vendor DDR I/O cells (ltpi_lvds_io) and the
# per-operating-point budgets in rtl/vendor/ - this project measures pure
# fabric (register-to-register) closure at the link rate.
foreach p {ll_gpio_in ll_gpio_out nl_gpio_in nl_gpio_out uart_txd_in
           uart_txd_out i2c_start_det i2c_stop_det i2c_scl_rise
           i2c_scl_fall i2c_sda_val i2c_scl_stretch i2c_sda_pull
           i2c_bus_start_gen i2c_bus_stop_gen dc_req_valid dc_req_write
           dc_req_addr dc_req_wdata dc_req_ready dc_rsp_valid
           dc_rsp_rdata dc_cmp_req dc_cmp_rdata dc_cmp_done csr_addr
           csr_we csr_re csr_wdata csr_rdata link_up rst ddr_mode
           ser_tx_sdr ser_tx_ddr ser_rx_sdr ser_rx_ddr} {
    set_instance_assignment -name VIRTUAL_PIN ON -to $p
    set_instance_assignment -name VIRTUAL_PIN ON -to ${p}\[*\]
}

project_close
