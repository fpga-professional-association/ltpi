# Questa/ModelSim: do run_questa.do   (or: vsim -c -do run_questa.do)
vlib work
vlog -sv +incdir+../rtl ../rtl/ltpi_pkg.sv ../rtl/ltpi_frame_rx.sv \
     ../rtl/ltpi_frame_tx.sv ../rtl/ltpi_link_fsm.sv \
     ../rtl/ltpi_gpio_channel.sv ../rtl/ltpi_uart_channel.sv \
     ../rtl/ltpi_i2c_relay.sv ../rtl/ltpi_phy.sv ../rtl/ltpi_top.sv \
     ltpi_if.sv ltpi_uvm_pkg.sv tb_uvm_top.sv
vsim -c tb_uvm_top +UVM_TESTNAME=ltpi_bringup_test -do "run -all; quit -f"
# CRC-noise robustness test:
# vsim -c tb_uvm_top +UVM_TESTNAME=ltpi_crc_error_test -do "run -all; quit -f"
