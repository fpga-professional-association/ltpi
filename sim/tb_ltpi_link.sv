// =============================================================================
// tb_ltpi_link.sv  -  End-to-end LTPI link testbench (SCM <-> HPM), dual-rate
//
// Instantiates ltpi_scm_top and ltpi_hpm_top with the Fig-19 dual-clock PHY:
//   * a shared System Clock (core domain),
//   * per-endpoint LVDS bit clocks modelling the IOPLL: they start at the
//     25 MHz base rate and, on each endpoint's speed_change, reconfigure (after
//     a modelled PLL relock) to the negotiated 400 MHz DDR = 800 Mbps rate.
// RX is source-synchronous: each endpoint's RX bit clock is the FAR end's
// forwarded tx_clk, so the receiver always tracks the transmitter's rate.
//
// Verifies: link trains at base, SWITCHES to the operational rate, re-aligns,
// reaches Operational, and tunnels all channels (GPIO/UART/I2C/Data) at speed.
// Prints "PASS n/n".
// =============================================================================
`timescale 1ns/1ps
`include "ltpi_scm_top.sv"
`include "ltpi_hpm_top.sv"

module tb_ltpi_link;
  import ltpi_pkg::*;

  localparam int NL = 32;
  localparam int NI = 2;
  // bit-clock half-periods: base 25 MHz (40 ns) and operational 800 Mbps (1.25 ns)
  localparam real HALF_BASE = 20.0;
  localparam real HALF_OPER = 0.625;

  logic sys_clk = 0, rst_n = 0;
  always #5 sys_clk = ~sys_clk;          // 100 MHz system clock (>= 80 MHz parallel)

  // ---- modelled IOPLL bit clocks (one per endpoint) ----
  real  scm_half = HALF_BASE, hpm_half = HALF_BASE;
  logic scm_bit_clk = 0, hpm_bit_clk = 0;
  always begin #(scm_half) scm_bit_clk = ~scm_bit_clk; end
  always begin #(hpm_half) hpm_bit_clk = ~hpm_bit_clk; end

  // serial nets
  logic s2h_dat, s2h_clk, h2s_dat, h2s_clk;
  // speed-switch control
  logic s_spdchg, h_spdchg; logic [3:0] s_opspd, h_opspd; logic s_opddr, h_opddr;

  // model PLL reconfiguration: on speed_change, relock then switch to op rate
  initial begin
    @(posedge s_spdchg);
    $display("[%0t] SCM speed_change: op_speed=%0d ddr=%b -> reconfiguring PLL to 800 Mbps", $time, s_opspd, s_opddr);
    #150;                                 // modelled PLL relock (<< 1 ms dwell)
    scm_half = HALF_OPER;
  end
  initial begin
    @(posedge h_spdchg);
    $display("[%0t] HPM speed_change: op_speed=%0d ddr=%b -> reconfiguring PLL to 800 Mbps", $time, h_opspd, h_opddr);
    #150;
    hpm_half = HALF_OPER;
  end

  // ---- SCM application signals ----
  logic [15:0] s_ll_in, s_ll_out;  logic [NL-1:0] s_nl_in, s_nl_out;
  logic s_txd0,s_txd1,s_rts0,s_rts1,s_rxd0,s_rxd1,s_cts0,s_cts1;
  logic [NI-1:0] s_i2c_det, s_i2c_regen, s_i2c_done, s_i2c_stretch;
  logic [NI*4-1:0] s_i2c_code, s_i2c_rcode;
  logic s_ini_req,s_ini_wr,s_ini_ack,s_ini_cpl; logic [31:0] s_ini_addr,s_ini_wd,s_ini_rd;
  logic [3:0] s_ini_be,s_ini_st; logic [7:0] s_ini_tag;
  logic [31:0] s_avm_addr,s_avm_wd,s_avm_rd; logic s_avm_rd_e,s_avm_wr_e,s_avm_wait,s_avm_rdv; logic [3:0] s_avm_be;
  logic [7:0] s_csr_a; logic s_csr_rd,s_csr_wr; logic [31:0] s_csr_wd,s_csr_rdv; logic s_csr_rdy;
  logic [3:0] s_state; logic s_oper, s_align;

  // ---- HPM application signals ----
  logic [15:0] h_ll_in, h_ll_out;  logic [NL-1:0] h_nl_in, h_nl_out;
  logic h_txd0,h_txd1,h_rts0,h_rts1,h_rxd0,h_rxd1,h_cts0,h_cts1;
  logic [NI-1:0] h_i2c_det, h_i2c_regen, h_i2c_done, h_i2c_stretch;
  logic [NI*4-1:0] h_i2c_code, h_i2c_rcode;
  logic h_ini_req,h_ini_wr,h_ini_ack,h_ini_cpl; logic [31:0] h_ini_addr,h_ini_wd,h_ini_rd;
  logic [3:0] h_ini_be,h_ini_st; logic [7:0] h_ini_tag;
  logic [31:0] h_avm_addr,h_avm_wd,h_avm_rd; logic h_avm_rd_e,h_avm_wr_e,h_avm_wait,h_avm_rdv; logic [3:0] h_avm_be;
  logic [7:0] h_csr_a; logic h_csr_rd,h_csr_wr; logic [31:0] h_csr_wd,h_csr_rdv; logic h_csr_rdy;
  logic [3:0] h_state; logic h_oper, h_align;

  // ================= DUTs =================
  ltpi_scm_top #(.NUM_NL(NL), .NUM_I2C(NI), .ADVERTISE_CYCLES(200)) scm (
    .clk(sys_clk), .rst_n,
    .tx_bit_clk(scm_bit_clk), .rx_clk(h2s_clk),
    .tx_dat(s2h_dat), .tx_clk(s2h_clk), .rx_dat(h2s_dat),
    .speed_change(s_spdchg), .op_speed(s_opspd), .op_ddr(s_opddr),
    .ll_in(s_ll_in), .ll_out(s_ll_out), .nl_in(s_nl_in), .nl_out(s_nl_out),
    .txd0(s_txd0),.txd1(s_txd1),.rts0(s_rts0),.rts1(s_rts1),.rxd0(s_rxd0),.rxd1(s_rxd1),.cts0(s_cts0),.cts1(s_cts1),
    .i2c_evt_det(s_i2c_det),.i2c_evt_code(s_i2c_code),.i2c_regen(s_i2c_regen),.i2c_regen_code(s_i2c_rcode),
    .i2c_regen_done(s_i2c_done),.i2c_scl_stretch(s_i2c_stretch),
    .ini_req(s_ini_req),.ini_write(s_ini_wr),.ini_addr(s_ini_addr),.ini_wdata(s_ini_wd),.ini_be(s_ini_be),.ini_tag(s_ini_tag),
    .ini_ack(s_ini_ack),.ini_cpl(s_ini_cpl),.ini_rdata(s_ini_rd),.ini_status(s_ini_st),
    .avm_address(s_avm_addr),.avm_read(s_avm_rd_e),.avm_write(s_avm_wr_e),.avm_writedata(s_avm_wd),.avm_byteenable(s_avm_be),
    .avm_readdata(s_avm_rd),.avm_waitrequest(s_avm_wait),.avm_readdatavalid(s_avm_rdv),
    .csr_addr(s_csr_a),.csr_read(s_csr_rd),.csr_write(s_csr_wr),.csr_wdata(s_csr_wd),.csr_rdata(s_csr_rdv),.csr_ready(s_csr_rdy),
    .link_state(s_state),.operational(s_oper),.link_aligned(s_align)
  );

  ltpi_hpm_top #(.NUM_NL(NL), .NUM_I2C(NI), .ADVERTISE_CYCLES(200)) hpm (
    .clk(sys_clk), .rst_n,
    .tx_bit_clk(hpm_bit_clk), .rx_clk(s2h_clk),
    .tx_dat(h2s_dat), .tx_clk(h2s_clk), .rx_dat(s2h_dat),
    .speed_change(h_spdchg), .op_speed(h_opspd), .op_ddr(h_opddr),
    .ll_in(h_ll_in), .ll_out(h_ll_out), .nl_in(h_nl_in), .nl_out(h_nl_out),
    .txd0(h_txd0),.txd1(h_txd1),.rts0(h_rts0),.rts1(h_rts1),.rxd0(h_rxd0),.rxd1(h_rxd1),.cts0(h_cts0),.cts1(h_cts1),
    .i2c_evt_det(h_i2c_det),.i2c_evt_code(h_i2c_code),.i2c_regen(h_i2c_regen),.i2c_regen_code(h_i2c_rcode),
    .i2c_regen_done(h_i2c_done),.i2c_scl_stretch(h_i2c_stretch),
    .ini_req(h_ini_req),.ini_write(h_ini_wr),.ini_addr(h_ini_addr),.ini_wdata(h_ini_wd),.ini_be(h_ini_be),.ini_tag(h_ini_tag),
    .ini_ack(h_ini_ack),.ini_cpl(h_ini_cpl),.ini_rdata(h_ini_rd),.ini_status(h_ini_st),
    .avm_address(h_avm_addr),.avm_read(h_avm_rd_e),.avm_write(h_avm_wr_e),.avm_writedata(h_avm_wd),.avm_byteenable(h_avm_be),
    .avm_readdata(h_avm_rd),.avm_waitrequest(h_avm_wait),.avm_readdatavalid(h_avm_rdv),
    .csr_addr(h_csr_a),.csr_read(h_csr_rd),.csr_write(h_csr_wr),.csr_wdata(h_csr_wd),.csr_rdata(h_csr_rdv),.csr_ready(h_csr_rdy),
    .link_state(h_state),.operational(h_oper),.link_aligned(h_align)
  );

  // HPM Avalon-MM slave (simple memory) for the Data channel
  logic [31:0] mem [0:15];
  assign h_avm_wait = 1'b0;
  always_ff @(posedge sys_clk or negedge rst_n) begin
    if (!rst_n) h_avm_rdv <= 1'b0;
    else begin
      h_avm_rdv <= 1'b0;
      if (h_avm_wr_e) mem[h_avm_addr[5:2]] <= h_avm_wd;
      if (h_avm_rd_e) begin h_avm_rd <= mem[h_avm_addr[5:2]]; h_avm_rdv <= 1'b1; end
    end
  end
  assign s_avm_wait = 1'b0; assign s_avm_rdv = 1'b0; assign s_avm_rd = 32'h0;
  always_ff @(posedge sys_clk) begin h_i2c_done <= h_i2c_regen; s_i2c_done <= s_i2c_regen; end

  // ================= test sequence =================
  integer pass=0, fail=0, i; logic ok; logic switched=0;
  task check(input string name, input logic cond);
    begin if (cond) begin pass++; $display("  PASS: %s", name); end
          else      begin fail++; $display("  FAIL: %s", name); end end
  endtask

  initial begin
    s_ll_in=0; s_nl_in=0; h_ll_in=0; h_nl_in=0;
    s_txd0=1;s_txd1=1;s_rts0=0;s_rts1=0; h_txd0=1;h_txd1=1;h_rts0=0;h_rts1=0;
    s_i2c_det=0; s_i2c_code=0; h_i2c_det=0; h_i2c_code=0;
    s_ini_req=0;s_ini_wr=0;s_ini_addr=0;s_ini_wd=0;s_ini_be=4'hF;s_ini_tag=0;
    h_ini_req=0;h_ini_wr=0;h_ini_addr=0;h_ini_wd=0;h_ini_be=4'hF;h_ini_tag=0;
    s_csr_a=0;s_csr_rd=0;s_csr_wr=0;s_csr_wd=0; h_csr_a=0;h_csr_rd=0;h_csr_wr=0;h_csr_wd=0;
    for (i=0;i<16;i++) mem[i] = 32'hDEAD0000 + i;
    rst_n=0; repeat(20) @(posedge sys_clk); rst_n=1;

    // 1. bring-up through the base->operational speed switch
    ok=0;
    for (i=0;i<2000000 && !ok;i++) begin @(posedge sys_clk); if (s_oper && h_oper) ok=1; end
    check("link reaches Operational (both sides) after base->operational speed switch", ok);
    check("speed switch occurred (SCM op_speed = x16/400MHz)", s_opspd==4'h8 && s_opddr==1'b1);
    check("operating at 800 Mbps bit rate (PLL reconfigured)", scm_half==HALF_OPER && hpm_half==HALF_OPER);
    if (!ok) begin $display("FAIL %0d/%0d (SCM st=%0d HPM st=%0d)", pass, pass+fail, s_state, h_state); $finish; end

    // 2. GPIO LL both directions (now at operational rate)
    s_ll_in=16'hA53C; h_ll_in=16'h1234; repeat(2000) @(posedge sys_clk);
    check("LL GPIO SCM->HPM at operational rate", h_ll_out==16'hA53C);
    check("LL GPIO HPM->SCM at operational rate", s_ll_out==16'h1234);

    // 3. NL GPIO
    s_nl_in=32'hCAFEF00D; repeat(6000) @(posedge sys_clk);
    check("NL GPIO SCM->HPM", h_nl_out==32'hCAFEF00D);

    // 4. UART line tunnels
    s_txd0=0; repeat(3000) @(posedge sys_clk);
    check("UART0 TXD level SCM->HPM (low)", h_rxd0==1'b0);
    s_txd0=1; repeat(3000) @(posedge sys_clk);
    check("UART0 TXD level SCM->HPM (high)", h_rxd0==1'b1);

    // 5. I2C START relays SCM->HPM
    @(posedge sys_clk); s_i2c_det[0]=1; s_i2c_code[3:0]=I2C_START;
    @(posedge sys_clk); s_i2c_det[0]=0;
    ok=0; for (i=0;i<4000;i++) begin @(posedge sys_clk); if (h_i2c_regen[0] && h_i2c_rcode[3:0]==I2C_START) ok=1; end
    check("I2C START relayed SCM->HPM (regenerated)", ok);

    // 6. Data channel Avalon read
    @(posedge sys_clk);
    s_ini_addr=32'h0000_0010; s_ini_wr=0; s_ini_be=4'hF; s_ini_tag=8'h5A; s_ini_req=1;
    @(posedge sys_clk); s_ini_req=0;
    ok=0; for (i=0;i<8000 && !ok;i++) begin @(posedge sys_clk); if (s_ini_cpl) ok=1; end
    check("Data channel read completion returned", ok);
    check("Data channel read data == HPM mem[4]", s_ini_rd==(32'hDEAD0000+4));

    $display("");
    if (fail==0) $display("PASS %0d/%0d  (LTPI dual-rate SCM<->HPM: 25MHz train -> 400MHz DDR operational)", pass, pass+fail);
    else         $display("FAIL %0d/%0d", pass, pass+fail);
    $finish;
  end

  initial begin #80_000_000; $display("FAIL: global timeout (SCM st=%0d HPM st=%0d)", s_state, h_state); $finish; end
endmodule
