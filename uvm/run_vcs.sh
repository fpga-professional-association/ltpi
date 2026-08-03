#!/bin/sh
# Synopsys VCS
vcs -full64 -sverilog -ntb_opts uvm-1.2 +incdir+../rtl \
    ../rtl/ltpi_pkg.sv ../rtl/ltpi_frame_rx.sv ../rtl/ltpi_frame_tx.sv \
    ../rtl/ltpi_link_fsm.sv ../rtl/ltpi_gpio_channel.sv \
    ../rtl/ltpi_uart_channel.sv ../rtl/ltpi_i2c_relay.sv \
    ../rtl/ltpi_phy.sv ../rtl/ltpi_top.sv \
    ltpi_if.sv ltpi_uvm_pkg.sv tb_uvm_top.sv -o simv
./simv +UVM_TESTNAME=${1:-ltpi_bringup_test}
