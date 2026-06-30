// =============================================================================
// ltpi_csr.sv  -  LTPI Control and Status Registers (spec Sec 3.2, Table 36)
//
// A 32-bit register block (block-local byte offsets 0x00..0x80) with a simple
// synchronous BMC access port.  Register types: RO (status), RW (control/caps),
// RWC (write-1-to-clear status bits; counters clear on a 0xFFFFFFFF write).
// Status is sampled from the link FSM; error/frame counters are incremented by
// FSM event pulses; control bits drive link/channel control.
// =============================================================================
`ifndef LTPI_CSR_SV
`define LTPI_CSR_SV
`include "ltpi_pkg.sv"
import ltpi_pkg::*;

module ltpi_csr #(
  parameter logic [15:0] SPEED_CAP_DEFAULT = 16'h008F,
  parameter logic [63:0] CAPS_DEFAULT      = 64'h0,
  parameter logic [15:0] PLATFORM_ID       = 16'h0000
) (
  input  logic        clk,
  input  logic        rst_n,

  // ---- BMC access port ----
  input  logic [7:0]  csr_addr,     // block-local byte offset (word aligned)
  input  logic        csr_read,
  input  logic        csr_write,
  input  logic [31:0] csr_wdata,
  output logic [31:0] csr_rdata,
  output logic        csr_ready,

  // ---- status from FSM ----
  input  logic [3:0]  local_state,
  input  logic [3:0]  remote_state,
  input  logic [3:0]  speed_code,
  input  logic        ddr,
  input  logic        link_aligned,
  input  logic [15:0] remote_speed_cap,
  input  logic [7:0]  local_version,
  input  logic [7:0]  remote_version,
  input  logic [15:0] remote_platform,
  input  logic [63:0] remote_caps,
  input  logic [63:0] applied_caps,

  // ---- counter event pulses ----
  input  logic        ev_crc_err, ev_lost_err, ev_comma_err, ev_speed_to,
  input  logic        ev_cfg_err, ev_align_err,
  input  logic        ev_rx_detect, ev_rx_speed, ev_rx_adv, ev_rx_cfg,
  input  logic        ev_tx_detect, ev_tx_speed, ev_tx_adv, ev_tx_cfg,
  input  logic        ev_oper_rx, ev_oper_tx,

  // ---- control / capability outputs ----
  output logic [15:0] local_speed_cap,
  output logic [63:0] local_caps,
  output logic [15:0] local_platform,
  output logic        sw_reset,
  output logic        retrain,
  output logic        auto_config,
  output logic        trig_config,
  output logic        data_reset,
  output logic [6:0]  i2c_reset
);
  // ---- RW registers ----
  logic [15:0] speed_cap_q;
  logic [63:0] caps_q;
  logic [31:0] ctrl_q;          // 0x80 link control
  // ---- RWC status bits (0x00 [5:1]) ----
  logic stat_cfg_to, stat_speed_to, stat_comma, stat_crc, stat_lost;
  // ---- RWC counters ----
  logic [31:0] cnt_align, cnt_lost, cnt_crc, cnt_comma, cnt_speed_to, cnt_cfg_to;
  logic [31:0] cnt_rx_detect, cnt_rx_speed, cnt_rx_adv, cnt_rx_cfg;
  logic [31:0] cnt_tx_detect, cnt_tx_speed, cnt_tx_adv, cnt_tx_cfg;
  logic [31:0] cnt_oper_rx, cnt_oper_tx;

  assign local_speed_cap = speed_cap_q;
  assign local_caps      = caps_q;
  assign local_platform  = PLATFORM_ID;
  assign sw_reset        = ctrl_q[LC_SW_RESET];
  assign retrain         = ctrl_q[LC_RETRAIN];
  assign data_reset      = ctrl_q[LC_DATA_RESET];
  assign auto_config     = ctrl_q[LC_AUTO_CONFIG];
  assign trig_config     = ctrl_q[LC_TRIG_CONFIG];
  assign i2c_reset       = ctrl_q[8:2];

  // packed status word (0x00)
  logic [31:0] link_status_w;
  always @(*) begin
    link_status_w = 32'h0;
    link_status_w[19:16] = local_state;
    link_status_w[15:12] = remote_state;
    link_status_w[11:8]  = speed_code;
    link_status_w[7]     = ddr;
    link_status_w[5]     = stat_cfg_to;
    link_status_w[4]     = stat_speed_to;
    link_status_w[3]     = stat_comma;
    link_status_w[2]     = stat_crc;
    link_status_w[1]     = stat_lost;
    link_status_w[0]     = link_aligned;
  end

  // ---- read mux ----
  logic [31:0] rdata_c;
  always @(*) begin
    case (csr_addr)
      CSR_LINK_STATUS:    rdata_c = link_status_w;
      CSR_DETECT_CAP_LOC: rdata_c = {8'h0, speed_cap_q, local_version};
      CSR_DETECT_CAP_REM: rdata_c = {8'h0, remote_speed_cap, remote_version};
      CSR_PLATFORM_LOC:   rdata_c = {16'h0, PLATFORM_ID};
      CSR_PLATFORM_REM:   rdata_c = {16'h0, remote_platform};
      CSR_ADV_CAP_LOC_LO: rdata_c = caps_q[31:0];
      CSR_ADV_CAP_LOC_HI: rdata_c = caps_q[63:32];
      CSR_ADV_CAP_REM_LO: rdata_c = remote_caps[31:0];
      CSR_ADV_CAP_REM_HI: rdata_c = remote_caps[63:32];
      CSR_DEF_CFG_LO:     rdata_c = applied_caps[31:0];
      CSR_DEF_CFG_HI:     rdata_c = applied_caps[63:32];
      CSR_ALIGN_ERR_CNT:  rdata_c = cnt_align;
      CSR_LOST_ERR_CNT:   rdata_c = cnt_lost;
      CSR_CRC_ERR_CNT:    rdata_c = cnt_crc;
      CSR_COMMA_ERR_CNT:  rdata_c = cnt_comma;
      CSR_SPEED_TO_CNT:   rdata_c = cnt_speed_to;
      CSR_CFG_TO_CNT:     rdata_c = cnt_cfg_to;
      CSR_TRAIN_RX_LO:    rdata_c = {cnt_rx_cfg[7:0], cnt_rx_speed[7:0], cnt_rx_detect[15:0]};
      CSR_TRAIN_RX_HI:    rdata_c = cnt_rx_adv;
      CSR_TRAIN_TX_LO:    rdata_c = {cnt_tx_cfg[7:0], cnt_tx_speed[7:0], cnt_tx_detect[15:0]};
      CSR_TRAIN_TX_HI:    rdata_c = cnt_tx_adv;
      CSR_OPER_RX_CNT:    rdata_c = cnt_oper_rx;
      CSR_OPER_TX_CNT:    rdata_c = cnt_oper_tx;
      CSR_LINK_CONTROL:   rdata_c = ctrl_q;
      default:            rdata_c = 32'h0;
    endcase
  end
  assign csr_rdata = rdata_c;
  assign csr_ready = 1'b1;

  // RWC clear helper: an "all ones" write clears a counter
  function automatic logic clr(input logic wr, input logic [7:0] a, input logic [7:0] sel,
                               input logic [31:0] wd);
    clr = wr & (a==sel) & (wd==32'hFFFF_FFFF);
  endfunction

  // counter step with RWC clear
  `define CNT(name, sel, ev) \
    if (clr(csr_write, csr_addr, sel, csr_wdata)) name <= 32'h0; \
    else if (ev) name <= name + 1'b1;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      speed_cap_q <= SPEED_CAP_DEFAULT;
      caps_q      <= CAPS_DEFAULT;
      // default: auto-advance to Configure after Advertise (self-training link;
      // BMC can clear this to take manual control of the Configure step).
      ctrl_q      <= (32'h1 << LC_AUTO_CONFIG);
      {stat_cfg_to,stat_speed_to,stat_comma,stat_crc,stat_lost} <= '0;
      cnt_align<=0; cnt_lost<=0; cnt_crc<=0; cnt_comma<=0; cnt_speed_to<=0; cnt_cfg_to<=0;
      cnt_rx_detect<=0; cnt_rx_speed<=0; cnt_rx_adv<=0; cnt_rx_cfg<=0;
      cnt_tx_detect<=0; cnt_tx_speed<=0; cnt_tx_adv<=0; cnt_tx_cfg<=0;
      cnt_oper_rx<=0; cnt_oper_tx<=0;
    end else begin
      // ---- RW writes ----
      if (csr_write) begin
        case (csr_addr)
          CSR_DETECT_CAP_LOC: speed_cap_q     <= csr_wdata[23:8];
          CSR_ADV_CAP_LOC_LO: caps_q[31:0]    <= csr_wdata;
          CSR_ADV_CAP_LOC_HI: caps_q[63:32]   <= csr_wdata;
          CSR_LINK_CONTROL:   ctrl_q          <= csr_wdata;
          default: ;
        endcase
      end

      // ---- RWC status bits (set by event, write-1-clears that bit) ----
      stat_crc      <= (ev_crc_err)   ? 1'b1 : (csr_write && csr_addr==CSR_LINK_STATUS && csr_wdata[2]) ? 1'b0 : stat_crc;
      stat_lost     <= (ev_lost_err)  ? 1'b1 : (csr_write && csr_addr==CSR_LINK_STATUS && csr_wdata[1]) ? 1'b0 : stat_lost;
      stat_comma    <= (ev_comma_err) ? 1'b1 : (csr_write && csr_addr==CSR_LINK_STATUS && csr_wdata[3]) ? 1'b0 : stat_comma;
      stat_speed_to <= (ev_speed_to)  ? 1'b1 : (csr_write && csr_addr==CSR_LINK_STATUS && csr_wdata[4]) ? 1'b0 : stat_speed_to;
      stat_cfg_to   <= (ev_cfg_err)   ? 1'b1 : (csr_write && csr_addr==CSR_LINK_STATUS && csr_wdata[5]) ? 1'b0 : stat_cfg_to;

      // ---- counters (RWC) ----
      `CNT(cnt_align,     CSR_ALIGN_ERR_CNT, ev_align_err)
      `CNT(cnt_lost,      CSR_LOST_ERR_CNT,  ev_lost_err)
      `CNT(cnt_crc,       CSR_CRC_ERR_CNT,   ev_crc_err)
      `CNT(cnt_comma,     CSR_COMMA_ERR_CNT, ev_comma_err)
      `CNT(cnt_speed_to,  CSR_SPEED_TO_CNT,  ev_speed_to)
      `CNT(cnt_cfg_to,    CSR_CFG_TO_CNT,    ev_cfg_err)
      `CNT(cnt_rx_detect, CSR_TRAIN_RX_LO,   ev_rx_detect)
      `CNT(cnt_rx_speed,  CSR_TRAIN_RX_LO,   ev_rx_speed)
      `CNT(cnt_rx_cfg,    CSR_TRAIN_RX_LO,   ev_rx_cfg)
      `CNT(cnt_rx_adv,    CSR_TRAIN_RX_HI,   ev_rx_adv)
      `CNT(cnt_tx_detect, CSR_TRAIN_TX_LO,   ev_tx_detect)
      `CNT(cnt_tx_speed,  CSR_TRAIN_TX_LO,   ev_tx_speed)
      `CNT(cnt_tx_cfg,    CSR_TRAIN_TX_LO,   ev_tx_cfg)
      `CNT(cnt_tx_adv,    CSR_TRAIN_TX_HI,   ev_tx_adv)
      `CNT(cnt_oper_rx,   CSR_OPER_RX_CNT,   ev_oper_rx)
      `CNT(cnt_oper_tx,   CSR_OPER_TX_CNT,   ev_oper_tx)
    end
  end
  `undef CNT

`ifdef FORMAL
  logic fpv = 1'b0;
  always_ff @(posedge clk) fpv <= 1'b1;
  always @(posedge clk) if (fpv && $past(rst_n) && rst_n) begin
    // RWC counter: an all-ones write clears it (when no concurrent event)
    if ($past(csr_write) && $past(csr_addr)==CSR_CRC_ERR_CNT
        && $past(csr_wdata)==32'hFFFF_FFFF && !$past(ev_crc_err))
      a_rwc_clear: assert (cnt_crc == 32'h0);
    // RW control register reflects the last write
    if ($past(csr_write) && $past(csr_addr)==CSR_LINK_CONTROL)
      a_rw_ctrl: assert (ctrl_q == $past(csr_wdata));
    // RWC status bit set by event, cleared by write-1 (no concurrent event)
    if ($past(ev_crc_err)) a_stat_set: assert (stat_crc);
    if ($past(csr_write) && $past(csr_addr)==CSR_LINK_STATUS && $past(csr_wdata[2])
        && !$past(ev_crc_err)) a_stat_clr: assert (!stat_crc);
  end
`endif
endmodule
`endif
