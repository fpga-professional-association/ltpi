// Synthesis/timing wrapper: one complete LTPI endpoint (SCM role,
// NUM_I2C=2, 32 NL GPIOs) with registered serial pins. All wide side-band
// buses are marked VIRTUAL_PIN in the .qsf so fabric timing - not package
// pin count - is what gets measured.
import ltpi_pkg::*;

module ltpi_syn_top (
    input  logic        clk,          // link clock (constrained in .sdc)
    input  logic        rst,
    input  logic        ddr_mode,
    output logic        ser_tx_sdr,
    output logic [1:0]  ser_tx_ddr,
    input  logic        ser_rx_sdr,
    input  logic [1:0]  ser_rx_ddr,

    // Virtual-pinned side-band (kept as ports so nothing optimizes away)
    input  logic [15:0] ll_gpio_in,
    output logic [15:0] ll_gpio_out,
    input  logic [31:0] nl_gpio_in,
    output logic [31:0] nl_gpio_out,
    input  logic        uart_txd_in,
    output logic        uart_txd_out,
    input  logic [1:0]  i2c_start_det, i2c_stop_det, i2c_scl_rise,
    input  logic [1:0]  i2c_scl_fall, i2c_sda_val,
    output logic [1:0]  i2c_scl_stretch, i2c_sda_pull,
    output logic [1:0]  i2c_bus_start_gen, i2c_bus_stop_gen,
    input  logic        dc_req_valid, dc_req_write,
    input  logic [31:0] dc_req_addr, dc_req_wdata,
    output logic        dc_req_ready, dc_rsp_valid,
    output logic [31:0] dc_rsp_rdata,
    output logic        dc_cmp_req,
    input  logic [31:0] dc_cmp_rdata,
    input  logic        dc_cmp_done,
    input  logic [7:0]  csr_addr,
    input  logic        csr_we, csr_re,
    input  logic [31:0] csr_wdata,
    output logic [31:0] csr_rdata,
    output logic        link_up
);

    // Register the serial boundary so I/O timing is clean 1-FF-deep.
    logic       rx_sdr_q;
    logic [1:0] rx_ddr_q;
    logic       tx_sdr_d;
    logic [1:0] tx_ddr_d;
    always_ff @(posedge clk) begin
        rx_sdr_q   <= ser_rx_sdr;
        rx_ddr_q   <= ser_rx_ddr;
        ser_tx_sdr <= tx_sdr_d;
        ser_tx_ddr <= tx_ddr_d;
    end

    ltpi_top #(
        .ROLE_SCM(1'b1), .NL_TOTAL(32), .NUM_I2C(2)
    ) u_ltpi (
        .clk(clk), .rst(rst), .ddr_mode(ddr_mode),
        .local_speed_caps(CAPS_DEFAULT), .cfg_ready(1'b1),
        .retrain_req(1'b0), .soft_reset(1'b0),
        .ser_tx_sdr(tx_sdr_d), .ser_tx_ddr(tx_ddr_d),
        .ser_rx_sdr(rx_sdr_q), .ser_rx_ddr(rx_ddr_q),
        .ll_gpio_in(ll_gpio_in), .ll_gpio_out(ll_gpio_out),
        .nl_gpio_in(nl_gpio_in), .nl_gpio_out(nl_gpio_out),
        .uart_txd_in(uart_txd_in), .uart_flow_in(1'b1),
        .uart_txd_out(uart_txd_out), .uart_flow_out(),
        .i2c_start_det(i2c_start_det), .i2c_stop_det(i2c_stop_det),
        .i2c_scl_rise(i2c_scl_rise), .i2c_scl_fall(i2c_scl_fall),
        .i2c_sda_val(i2c_sda_val),
        .i2c_scl_stretch(i2c_scl_stretch), .i2c_sda_pull(i2c_sda_pull),
        .i2c_bus_start_gen(i2c_bus_start_gen),
        .i2c_bus_stop_gen(i2c_bus_stop_gen),
        .dc_req_valid(dc_req_valid), .dc_req_write(dc_req_write),
        .dc_req_addr(dc_req_addr), .dc_req_wdata(dc_req_wdata),
        .dc_req_byteen(4'hF), .dc_req_ready(dc_req_ready),
        .dc_rsp_valid(dc_rsp_valid), .dc_rsp_rdata(dc_rsp_rdata),
        .dc_rsp_error(),
        .dc_cmp_req(dc_cmp_req), .dc_cmp_write(), .dc_cmp_addr(),
        .dc_cmp_wdata(), .dc_cmp_byteen(),
        .dc_cmp_rdata(dc_cmp_rdata), .dc_cmp_done(dc_cmp_done),
        .csr_addr(csr_addr), .csr_we(csr_we), .csr_re(csr_re),
        .csr_wdata(csr_wdata), .csr_rdata(csr_rdata),
        .link_state(), .link_up(link_up),
        .speed_select(), .speed_valid(), .phy_aligned()
    );

endmodule
