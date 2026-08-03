// UVM interface to the LTPI DUT: serial link + channel-side observation.
interface ltpi_if (input logic clk);
    logic rst;

    // Serial link (SDR bit stream; the UVM agent is the HPM peer)
    logic dut_tx_bit;    // DUT -> agent
    logic agt_tx_bit;    // agent -> DUT

    // DUT channel-side signals (driven/observed by the test)
    logic [15:0] ll_gpio_in;
    logic [15:0] ll_gpio_out;
    logic [31:0] nl_gpio_in;
    logic [31:0] nl_gpio_out;
    logic        uart_txd_in,  uart_txd_out;

    // Status
    logic [2:0]  link_state;
    logic        link_up;
    logic [15:0] speed_select;
    logic        speed_valid;
    logic        phy_aligned;

    clocking drv_cb @(posedge clk);
        output agt_tx_bit;
        input  dut_tx_bit;
    endclocking

    clocking mon_cb @(posedge clk);
        input dut_tx_bit, agt_tx_bit, link_state, link_up,
              speed_select, speed_valid, ll_gpio_out, uart_txd_out;
    endclocking
endinterface
