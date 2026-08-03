// UVM testbench top: SCM-role ltpi_top DUT + peer-emulating UVM env.
`timescale 1ns/1ps
module tb_uvm_top;
    import uvm_pkg::*;
    import ltpi_uvm_pkg::*;

    logic clk = 0;
    always #2.5 clk = ~clk;    // 200 MHz

    ltpi_if vif (clk);

    initial begin
        vif.rst = 1;
        vif.ll_gpio_in = '0;
        vif.nl_gpio_in = '0;
        vif.uart_txd_in = 1'b1;
        repeat (10) @(posedge clk);
        vif.rst = 0;
    end

    ltpi_top #(
        .ROLE_SCM(1'b1), .NL_TOTAL(32), .CLK_HZ(200_000_000),
        // Shrunk thresholds keep UVM sims fast; override per-test with
        // defparam or a config class if the spec values are wanted.
        .DETECT_MIN_TX(8), .DETECT_MIN_RX(4), .ALIGN_MIN_RX(3),
        .SPEED_SCM_MIN_TX(3), .SPEED_HPM_MIN_RX(2), .SPEED_TIMEOUT_TX(64),
        .ADV_MIN_CYCLES(1500), .ADV_ALIGN_TIMEOUT(3000), .ADV_MIN_RX(3), .CFG_MAX_TX(16), .ACC_MAX_TX(8)
    ) dut (
        .clk(clk), .rst(vif.rst), .ddr_mode(1'b0),
        .local_speed_caps(16'h8121), .cfg_ready(1'b1),
        .retrain_req(1'b0), .soft_reset(1'b0),
        .ser_tx_sdr(vif.dut_tx_bit), .ser_tx_ddr(),
        .ser_rx_sdr(vif.agt_tx_bit), .ser_rx_ddr(2'b00),
        .ll_gpio_in(vif.ll_gpio_in), .ll_gpio_out(vif.ll_gpio_out),
        .nl_gpio_in(vif.nl_gpio_in), .nl_gpio_out(vif.nl_gpio_out),
        .uart_txd_in(vif.uart_txd_in), .uart_flow_in(1'b1),
        .uart_txd_out(vif.uart_txd_out), .uart_flow_out(),
        .i2c_scl_in(2'b11), .i2c_sda_in(2'b11),
        .i2c_scl_stretch(), .i2c_sda_pull(),
        .i2c_bus_start_gen(), .i2c_bus_stop_gen(),
        .dc_req_valid(1'b0), .dc_req_write(1'b0), .dc_req_addr(32'h0),
        .dc_req_wdata(32'h0), .dc_req_byteen(4'h0), .dc_req_ready(),
        .dc_rsp_valid(), .dc_rsp_rdata(), .dc_rsp_error(),
        .dc_cmp_req(), .dc_cmp_write(), .dc_cmp_addr(), .dc_cmp_wdata(),
        .dc_cmp_byteen(), .dc_cmp_rdata(32'h0), .dc_cmp_done(1'b0),
        .apb_s_psel(1'b0), .apb_s_penable(1'b0), .apb_s_pwrite(1'b0),
        .apb_s_paddr(32'h0), .apb_s_pwdata(32'h0), .apb_s_pstrb(4'h0),
        .apb_s_pready(), .apb_s_prdata(), .apb_s_pslverr(),
        .apb_m_psel(), .apb_m_penable(), .apb_m_pwrite(), .apb_m_paddr(),
        .apb_m_pwdata(), .apb_m_pstrb(),
        .apb_m_pready(1'b1), .apb_m_prdata(32'h0), .apb_m_pslverr(1'b0),
        .csr_addr(8'h0), .csr_we(1'b0), .csr_re(1'b0),
        .csr_wdata(32'h0), .csr_rdata(),
        .link_state(vif.link_state), .link_up(vif.link_up),
        .speed_select(vif.speed_select), .speed_valid(vif.speed_valid),
        .phy_aligned(vif.phy_aligned)
    );

    initial begin
        uvm_config_db#(virtual ltpi_if)::set(null, "*", "vif", vif);
        run_test("ltpi_bringup_test");   // override with +UVM_TESTNAME=
    end

    initial begin
        $dumpfile("tb_uvm_top.vcd");
        $dumpvars(0, tb_uvm_top);
    end
endmodule
