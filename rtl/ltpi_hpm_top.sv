// =============================================================================
// ltpi_hpm_top.sv  -  LTPI endpoint top for the HPM (Host Processor Module)
//
// Fig-19 structure: ltpi_core (system-clock symbol-parallel logic, ROLE_HPM) +
// ltpi_phy (dual-rate serial PHY).  The HPM responds to training (sends Accept).
// Mirrors ltpi_scm_top.
// =============================================================================
`ifndef LTPI_HPM_TOP_SV
`define LTPI_HPM_TOP_SV
`include "ltpi_pkg.sv"
`include "ltpi_core.sv"
`include "ltpi_phy.sv"

module ltpi_hpm_top #(
  parameter int          NUM_NL           = 32,
  parameter int          NUM_I2C          = 2,
  parameter int          ADVERTISE_CYCLES = 25000,
  parameter logic [15:0] SPEED_CAP        = 16'h8101, // x1(base)+x16(400MHz)+DDR
  parameter logic [15:0] PLATFORM_ID      = 16'h4850,
  parameter logic [63:0] CAPS             = 64'h0000_0000_0000_000F
) (
  input  logic clk,
  input  logic rst_n,
  input  logic tx_bit_clk,
  input  logic rx_clk,
  output logic tx_dat,
  output logic tx_clk,
  input  logic rx_dat,
  output logic        speed_change,
  output logic [3:0]  op_speed,
  output logic        op_ddr,
  input  logic [15:0]        ll_in,
  output logic [15:0]        ll_out,
  input  logic [NUM_NL-1:0]  nl_in,
  output logic [NUM_NL-1:0]  nl_out,
  input  logic txd0, txd1, rts0, rts1,
  output logic rxd0, rxd1, cts0, cts1,
  input  logic [NUM_I2C-1:0]   i2c_evt_det,
  input  logic [NUM_I2C*4-1:0] i2c_evt_code,
  output logic [NUM_I2C-1:0]   i2c_regen,
  output logic [NUM_I2C*4-1:0] i2c_regen_code,
  input  logic [NUM_I2C-1:0]   i2c_regen_done,
  output logic [NUM_I2C-1:0]   i2c_scl_stretch,
  input  logic        ini_req, ini_write,
  input  logic [31:0] ini_addr, ini_wdata,
  input  logic [3:0]  ini_be,
  input  logic [7:0]  ini_tag,
  output logic        ini_ack, ini_cpl,
  output logic [31:0] ini_rdata,
  output logic [3:0]  ini_status,
  output logic [31:0] avm_address, avm_writedata,
  output logic        avm_read, avm_write,
  output logic [3:0]  avm_byteenable,
  input  logic [31:0] avm_readdata,
  input  logic        avm_waitrequest, avm_readdatavalid,
  input  logic [7:0]  csr_addr,
  input  logic        csr_read, csr_write,
  input  logic [31:0] csr_wdata,
  output logic [31:0] csr_rdata,
  output logic        csr_ready,
  output logic [3:0]  link_state,
  output logic        operational,
  output logic        link_aligned
);
  logic [9:0] tx_sym, rx_sym;
  logic       tx_sym_valid, tx_sym_ready, rx_sym_valid, realign, rx_aligned;

  ltpi_phy u_phy (
    .sys_clk(clk), .sys_rst_n(rst_n),
    .tx_sym, .tx_sym_valid, .tx_sym_ready, .rx_sym, .rx_sym_valid,
    .realign, .rx_aligned,
    .tx_bit_clk, .tx_bit_rst_n(rst_n), .rx_bit_clk(rx_clk), .rx_bit_rst_n(rst_n),
    .tx_dat, .tx_clk, .rx_dat
  );

  ltpi_core #(
    .ROLE(ltpi_pkg::ROLE_HPM), .NUM_NL(NUM_NL), .NUM_I2C(NUM_I2C),
    .ADVERTISE_CYCLES(ADVERTISE_CYCLES), .SPEED_CAP(SPEED_CAP),
    .PLATFORM_ID(PLATFORM_ID), .CAPS(CAPS)
  ) u_core (
    .clk, .rst_n,
    .tx_sym, .tx_sym_valid, .tx_sym_ready, .rx_sym, .rx_sym_valid,
    .realign, .rx_aligned, .speed_change, .op_speed, .op_ddr,
    .ll_in, .ll_out, .nl_in, .nl_out,
    .txd0, .txd1, .rts0, .rts1, .rxd0, .rxd1, .cts0, .cts1,
    .i2c_evt_det, .i2c_evt_code, .i2c_regen, .i2c_regen_code,
    .i2c_regen_done, .i2c_scl_stretch,
    .ini_req, .ini_write, .ini_addr, .ini_wdata, .ini_be, .ini_tag,
    .ini_ack, .ini_cpl, .ini_rdata, .ini_status,
    .avm_address, .avm_read, .avm_write, .avm_writedata, .avm_byteenable,
    .avm_readdata, .avm_waitrequest, .avm_readdatavalid,
    .csr_addr, .csr_read, .csr_write, .csr_wdata, .csr_rdata, .csr_ready,
    .link_state, .operational, .link_aligned
  );
endmodule
`endif
