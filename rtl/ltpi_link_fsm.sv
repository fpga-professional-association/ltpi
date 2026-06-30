// =============================================================================
// ltpi_link_fsm.sv  -  LTPI Link Training / Configuration / Operational FSM
//
// Implements the state machine of spec Sec 4 (Fig 27) for one endpoint.  ROLE
// selects SCM (drives training, sends Configure) or HPM (responds, sends Accept).
//
//   Link Detect -> Link Speed -> Advertise -> Configure(SCM)/Accept(HPM) -> Operational
//
// Frame-level: advanced by rx_frame_valid (one decoded RX frame) and
// tx_frame_done (one TX frame emitted).  Counts and timeouts use the exact spec
// thresholds (ltpi_pkg DETECT_*/SPEED_*/CONFIG_*/ACCEPT_*/LOST_*); the 1 ms
// Advertise dwell is the ADVERTISE_CYCLES parameter (cycle counted, shrinkable
// for sim/formal).  The FSM owns the TX frame kind + training payload and the
// negotiated speed/capabilities; channels run only while `operational`.
// =============================================================================
`ifndef LTPI_LINK_FSM_SV
`define LTPI_LINK_FSM_SV
`include "ltpi_pkg.sv"
import ltpi_pkg::*;

module ltpi_link_fsm #(
  parameter bit ROLE            = ltpi_pkg::ROLE_SCM,
  parameter int ADVERTISE_CYCLES = 32   // 1ms dwell; real value set per clock freq
) (
  input  logic        clk,
  input  logic        rst_n,

  // ---- received-frame interface (from ltpi_frame_rx) ----
  input  logic        rx_frame_valid,
  input  logic [1:0]  rx_comma,        // ltpi_pkg::comma_e
  input  logic [7:0]  rx_subtype,
  input  logic        rx_crc_ok,
  input  logic        rx_misalign,
  input  logic        rx_aligned,
  input  logic [103:0] rx_payload,     // bytes 2..14

  // ---- transmit-frame done strobe (from ltpi_frame_tx) ----
  input  logic        tx_frame_done,

  // ---- local capabilities (from CSR) ----
  input  logic [15:0] local_speed_cap, // Table 21
  input  logic [63:0] local_caps,      // Table 28 (8 bytes)
  input  logic [15:0] local_platform,  // Table 26
  input  logic [7:0]  local_version,   // BCD

  // ---- control (from CSR / app) ----
  input  logic        cfg_auto,        // auto-advance to Configure after Advertise
  input  logic        cfg_trigger,     // BMC-triggered Configure
  input  logic        soft_reset,      // -> Advertise
  input  logic        retrain,         // -> Link Detect

  // ---- transmit control ----
  output logic [2:0]  tx_kind,         // tx_kind_e
  output logic [1:0]  tx_comma_sel,    // for training frames
  output logic [111:0] tx_train_payload,// 14 bytes incl subtype (training frames)
  output logic        tx_operational,  // 1 = send Operational frame (core builds payload)

  // ---- status ----
  output logic [3:0]  link_state,      // ltpi_pkg::link_state_e
  output logic [3:0]  remote_state,
  output logic [3:0]  sel_speed_code,
  output logic        sel_ddr,
  output logic        link_aligned,
  output logic        operational,
  output logic        realign,         // pulse: re-acquire comma/word alignment
  output logic        speed_change,    // pulse: reconfigure PHY PLL to op rate
  output logic [3:0]  op_speed,        // operational speed code (CSR encoding)
  output logic        op_ddr,          // operational DDR select
  output logic [63:0] applied_caps,    // negotiated configuration (enables channels)
  output logic [15:0] remote_speed_cap,
  output logic [63:0] remote_caps,
  output logic [15:0] remote_platform,
  output logic [7:0]  remote_version,

  // ---- CSR event pulses ----
  output logic        ev_crc_err,
  output logic        ev_lost_err,
  output logic        ev_comma_err,
  output logic        ev_speed_to,
  output logic        ev_cfg_to,
  output logic        ev_align_err,
  output logic        ev_rx_detect, ev_rx_speed, ev_rx_adv, ev_rx_cfg,
  output logic        ev_tx_detect, ev_tx_speed, ev_tx_adv, ev_tx_cfg,
  output logic        ev_oper_rx,   ev_oper_tx
);
  // tx_kind encoding
  localparam logic [2:0] TXK_DETECT=3'd0, TXK_SPEED=3'd1, TXK_ADV=3'd2,
                         TXK_CFG=3'd3, TXK_ACC=3'd4, TXK_IO=3'd5, TXK_DATA=3'd6;

  logic [3:0] state, nstate;   // link_state_e values (ST_*); plain logic for tool portability

  // counters
  localparam int CW = 16;
  logic [CW-1:0] tx_cnt;     // TX frames in current state
  logic [3:0]    rx_good;    // consecutive correct expected RX frames (sat at 15)
  logic [3:0]    rx_lost;    // consecutive lost frames (sat at 15)
  logic [CW-1:0] adv_timer;  // advertise dwell
  logic          adv_done;   // 1 ms dwell satisfied

  // captured negotiated values
  logic [15:0] cap_rspeed;
  logic [63:0] cap_rcaps, cap_applied;
  logic [15:0] cap_rplat;
  logic [7:0]  cap_rver;
  logic [3:0]  spd_code;
  logic        spd_ddr;
  logic [15:0] spd_sel_word;

  // ---- classify a received frame ----
  logic rxk_detect, rxk_speed, rxk_adv, rxk_cfg, rxk_acc, rxk_io, rxk_data, rxk_known;
  always @(*) begin
    rxk_detect = (rx_comma==COMMA_TRAIN) && (rx_subtype==SUB_LINK_DETECT);
    rxk_speed  = (rx_comma==COMMA_TRAIN) && (rx_subtype==SUB_LINK_SPEED);
    rxk_adv    = (rx_comma==COMMA_CFG)   && (rx_subtype==SUB_ADVERTISE);
    rxk_cfg    = (rx_comma==COMMA_CFG)   && (rx_subtype==SUB_CONFIGURE);
    rxk_acc    = (rx_comma==COMMA_CFG)   && (rx_subtype==SUB_ACCEPT);
    rxk_io     = (rx_comma==COMMA_OPER)  && (rx_subtype==SUB_IO);
    rxk_data   = (rx_comma==COMMA_OPER)  && (rx_subtype==SUB_DATA);
    rxk_known  = rxk_detect|rxk_speed|rxk_adv|rxk_cfg|rxk_acc|rxk_io|rxk_data;
  end

  // stage numbers for skew handling (peer ahead = progress, not loss)
  logic [2:0] rx_stage, my_stage;
  always @(*) begin
    if      (rxk_detect)        rx_stage = 3'd0;
    else if (rxk_speed)         rx_stage = 3'd1;
    else if (rxk_adv)           rx_stage = 3'd2;
    else if (rxk_cfg|rxk_acc)   rx_stage = 3'd3;
    else                        rx_stage = 3'd4; // io/data
    case (state)
      ST_DETECT:    my_stage = 3'd0;
      ST_SPEED:     my_stage = 3'd1;
      ST_ADVERTISE: my_stage = 3'd2;
      ST_CONFIG:    my_stage = 3'd3;
      default:      my_stage = 3'd4;
    endcase
  end

  // A "good" RX frame: correct CRC, known type, current stage or a peer that is
  // ahead (forward progress).  A frame from a peer slightly behind is tolerated
  // (neither good nor lost) — that is the expected SCM/HPM training skew.
  // A frame is "lost" only on CRC error, misalignment, an unknown comma, or a
  // Link Detect frame seen in the config/operational phases (peer reset / link
  // lost — spec Fig 27 "Unexpected Frame e.g., Link Detect").
  logic rx_good_frame, rx_lost_frame, backward_reset;
  assign backward_reset = rxk_detect & (state==ST_ADVERTISE || state==ST_CONFIG || state==ST_OPER);
  assign rx_good_frame  = rx_frame_valid & rx_crc_ok & ~rx_misalign & rxk_known & (rx_stage >= my_stage);
  assign rx_lost_frame  = rx_frame_valid & (~rx_crc_ok | rx_misalign | ~rxk_known | backward_reset);

  // ---- highest common speed (fastest-first priority) ----
  function automatic logic [4:0] hi_common(input logic [15:0] a, input logic [15:0] b);
    // returns {found, bit_index[3:0]} (bit index into the 12 freq bits)
    logic [15:0] c; logic [3:0] idx; logic fnd;
    int order [0:11];
    begin
      order[0]=SPB_X40; order[1]=SPB_X32; order[2]=SPB_X24; order[3]=SPB_X16;
      order[4]=SPB_X12; order[5]=SPB_X10; order[6]=SPB_X8;  order[7]=SPB_X6;
      order[8]=SPB_X4;  order[9]=SPB_X3;  order[10]=SPB_X2; order[11]=SPB_X1;
      c = a & b; fnd = 1'b0; idx = 4'd0;
      for (int i=11; i>=0; i--)        // scan slowest..fastest, keep fastest found
        if (c[order[i]]) begin idx = order[i][3:0]; fnd = 1'b1; end
      hi_common = {fnd, idx};
    end
  endfunction

  logic [4:0] hc;
  always @(*) hc = hi_common(local_speed_cap, cap_rspeed);

  // selected speed word (one-hot freq bit + DDR if both support)
  always @(*) begin
    spd_ddr      = local_speed_cap[SPB_DDR] & cap_rspeed[SPB_DDR];
    spd_code     = ltpi_pkg::speed_csr_code(int'(hc[3:0]));
    spd_sel_word = 16'd0;
    if (hc[4]) spd_sel_word[hc[3:0]] = 1'b1;
    spd_sel_word[SPB_DDR] = spd_ddr;
  end

  // ---- captured config to apply: HPM accepts requested caps masked by its own;
  //      SCM applies what it requested (auto = local_caps) ----
  logic [63:0] req_caps;     // SCM's requested config
  assign req_caps = local_caps;

  // ============================ next-state ============================
  always @(*) begin
    nstate = state;
    case (state)
      ST_DETECT:
        if ((tx_cnt >= DETECT_TX_MIN && rx_good >= DETECT_RX_MIN) ||
            (rx_good_frame && rxk_speed))
          nstate = ST_SPEED;
      ST_SPEED: begin
        if (ROLE==ROLE_SCM) begin
          if (tx_cnt >= SPEED_TX_MIN) nstate = ST_ADVERTISE;
        end else begin
          if (rx_good >= SPEED_RX_MIN) nstate = ST_ADVERTISE;
        end
        // peer already advertising -> follow it forward (spec Note 2/3 skew)
        if (rx_good_frame && rx_stage >= 3'd2) nstate = ST_ADVERTISE;
        if (tx_cnt >= SPEED_TO_TX) nstate = ST_DETECT;  // timeout
      end
      ST_ADVERTISE: begin
        if (ROLE==ROLE_SCM) begin
          if (adv_done && (cfg_auto || cfg_trigger)) nstate = ST_CONFIG;
        end else begin
          if (rx_good_frame && rxk_cfg) nstate = ST_CONFIG; // HPM -> Accept
        end
        if (rx_lost >= LOST_CFG_MAX) nstate = ST_DETECT;
        if (adv_done && !rx_aligned)  nstate = ST_DETECT;   // realign failed
      end
      ST_CONFIG: begin
        if (ROLE==ROLE_SCM) begin
          if (rx_good_frame && rxk_acc) nstate = ST_OPER;
          else if (tx_cnt >= CONFIG_TX_MAX) nstate = ST_ADVERTISE;
        end else begin
          if (rx_good_frame && (rxk_io|rxk_data)) nstate = ST_OPER;
          else if (tx_cnt >= ACCEPT_TX_MAX) nstate = ST_ADVERTISE;
        end
        if (rx_lost >= LOST_CFG_MAX) nstate = ST_DETECT;
      end
      ST_OPER:
        if (rx_lost >= LOST_OPER_MAX) nstate = ST_DETECT;
      default: nstate = ST_DETECT;
    endcase
    // global overrides
    if (soft_reset) nstate = ST_ADVERTISE;
    if (retrain)    nstate = ST_DETECT;
  end

  // ============================ sequential ============================
  logic state_changed;
  assign state_changed = (nstate != state);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state      <= ST_DETECT;
      tx_cnt     <= '0;
      rx_good    <= '0;
      rx_lost    <= '0;
      adv_timer  <= '0;
      adv_done   <= 1'b0;
      cap_rspeed <= '0; cap_rcaps <= '0; cap_rplat <= '0; cap_rver <= '0;
      cap_applied<= '0;
    end else begin
      // capture remote training info from received frames
      if (rx_frame_valid && rx_crc_ok) begin
        if (rxk_detect) begin cap_rspeed <= rx_payload[23:8]; cap_rver <= rx_payload[7:0]; end
        if (rxk_adv)    begin cap_rcaps  <= rx_payload[87:24]; cap_rplat <= rx_payload[15:0]; end
        if (rxk_cfg && ROLE==ROLE_HPM) cap_applied <= rx_payload[71:8] & local_caps; // HPM accepts masked
        if (rxk_acc && ROLE==ROLE_SCM) cap_applied <= rx_payload[71:8];
      end

      // advertise dwell timer
      if (state==ST_ADVERTISE) begin
        if (!adv_done) adv_timer <= adv_timer + 1'b1;
        if (adv_timer >= ADVERTISE_CYCLES[CW-1:0]) adv_done <= 1'b1;
      end else begin
        adv_timer <= '0;
        adv_done  <= 1'b0;
      end

      // TX frame counter
      if (state_changed)        tx_cnt <= '0;
      else if (tx_frame_done)   tx_cnt <= (tx_cnt==16'hFFFF) ? tx_cnt : tx_cnt + 1'b1;

      // RX good / lost counters
      if (state_changed) begin
        rx_good <= '0; rx_lost <= '0;
      end else if (rx_frame_valid) begin
        if (rx_good_frame) begin
          rx_good <= (rx_good==4'hF) ? rx_good : rx_good + 1'b1;
          rx_lost <= '0;
        end else if (rx_lost_frame) begin
          rx_lost <= (rx_lost==4'hF) ? rx_lost : rx_lost + 1'b1;
          rx_good <= '0;
        end
      end

      // commit applied speed at Speed->Advertise; SCM computes, HPM mirrors via select word RX
      state <= nstate;
    end
  end

  // ============================ outputs ============================
  // TX frame kind from state + role
  always @(*) begin
    case (state)
      ST_DETECT:    tx_kind = TXK_DETECT;
      ST_SPEED:     tx_kind = TXK_SPEED;
      ST_ADVERTISE: tx_kind = TXK_ADV;
      ST_CONFIG:    tx_kind = (ROLE==ROLE_SCM) ? TXK_CFG : TXK_ACC;
      ST_OPER:      tx_kind = TXK_IO;     // core may override to TXK_DATA on demand
      default:      tx_kind = TXK_DETECT;
    endcase
  end

  assign tx_operational = (state==ST_OPER);
  assign operational    = (state==ST_OPER);
  assign link_aligned   = rx_aligned & (state==ST_OPER);

  // ---- speed switch + re-alignment (multi-rate PHY) ----
  // The base 25 MHz training runs in Link Detect/Speed; at Speed->Advertise the
  // PHY PLL is reconfigured to the operational rate (spec Sec 4.1.2.1), so the
  // RX SERDES must re-acquire comma/word alignment.  realign pulses on entry to
  // Link Detect (base re-acquire) and Advertise (post-switch re-acquire).
  logic [3:0] state_prev;
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) state_prev <= ST_DETECT; else state_prev <= state;

  assign speed_change = (state_prev == ST_SPEED) && (state == ST_ADVERTISE);
  assign realign      = ((state==ST_DETECT)    && (state_prev!=ST_DETECT)) ||
                        ((state==ST_ADVERTISE) && (state_prev==ST_SPEED));

  // latch the negotiated operational rate at the switch (held through Operational)
  logic [3:0] op_speed_q;
  logic       op_ddr_q;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin op_speed_q <= 4'h0; op_ddr_q <= 1'b0; end
    else if (speed_change) begin op_speed_q <= spd_code; op_ddr_q <= spd_ddr; end
  end
  assign op_speed = op_speed_q;
  assign op_ddr   = op_ddr_q;

  // build training payloads (byte1 = subtype at [7:0])
  always @(*) begin
    tx_comma_sel     = COMMA_TRAIN;
    tx_train_payload = '0;
    case (state)
      ST_DETECT: begin
        tx_comma_sel        = COMMA_TRAIN;
        tx_train_payload[7:0]   = SUB_LINK_DETECT;
        tx_train_payload[15:8]  = local_version;
        tx_train_payload[31:16] = local_speed_cap;
      end
      ST_SPEED: begin
        tx_comma_sel        = COMMA_TRAIN;
        tx_train_payload[7:0]   = SUB_LINK_SPEED;
        tx_train_payload[15:8]  = local_version;
        tx_train_payload[31:16] = spd_sel_word;
      end
      ST_ADVERTISE: begin
        tx_comma_sel        = COMMA_CFG;
        tx_train_payload[7:0]   = SUB_ADVERTISE;
        tx_train_payload[23:8]  = local_platform;
        tx_train_payload[31:24] = 8'h00;          // Capabilities Type = default
        tx_train_payload[95:32] = local_caps;     // bytes 5..12
      end
      ST_CONFIG: begin
        tx_comma_sel        = COMMA_CFG;
        if (ROLE==ROLE_SCM) begin
          tx_train_payload[7:0]  = SUB_CONFIGURE;
          tx_train_payload[15:8] = 8'h00;         // cap type
          tx_train_payload[79:16]= req_caps;      // bytes 3..10
        end else begin
          tx_train_payload[7:0]  = SUB_ACCEPT;
          tx_train_payload[15:8] = 8'h00;
          tx_train_payload[79:16]= cap_applied;   // echo accepted config
        end
      end
      default: ;
    endcase
  end

  // status outputs
  assign link_state       = state;
  assign sel_speed_code   = spd_code;
  assign sel_ddr          = spd_ddr;
  assign applied_caps     = cap_applied;
  assign remote_speed_cap = cap_rspeed;
  assign remote_caps      = cap_rcaps;
  assign remote_platform  = cap_rplat;
  assign remote_version   = cap_rver;

  // inferred remote state from most-recent received frame kind
  logic [3:0] rstate_q;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) rstate_q <= ST_DETECT;
    else if (rx_frame_valid && rx_crc_ok) begin
      if      (rxk_detect) rstate_q <= ST_DETECT;
      else if (rxk_speed)  rstate_q <= ST_SPEED;
      else if (rxk_adv)    rstate_q <= ST_ADVERTISE;
      else if (rxk_cfg||rxk_acc) rstate_q <= ST_CONFIG;
      else if (rxk_io||rxk_data) rstate_q <= ST_OPER;
    end
  end
  assign remote_state = rstate_q;

  // ---- CSR event pulses ----
  assign ev_crc_err   = rx_frame_valid & ~rx_crc_ok;
  assign ev_lost_err  = (state==ST_OPER) & (rx_lost==LOST_OPER_MAX-1) & rx_lost_frame;
  assign ev_comma_err = rx_frame_valid & (rx_comma==COMMA_NONE);
  assign ev_speed_to  = (state==ST_SPEED) & (tx_cnt>=SPEED_TO_TX) & tx_frame_done;
  assign ev_cfg_to    = (state==ST_CONFIG) & (ROLE==ROLE_SCM) & (tx_cnt>=CONFIG_TX_MAX) & tx_frame_done;
  assign ev_align_err = (state==ST_ADVERTISE) & adv_done & ~rx_aligned;
  assign ev_rx_detect = rx_good_frame & rxk_detect;
  assign ev_rx_speed  = rx_good_frame & rxk_speed;
  assign ev_rx_adv    = rx_good_frame & rxk_adv;
  assign ev_rx_cfg    = rx_good_frame & (rxk_cfg|rxk_acc);
  assign ev_tx_detect = tx_frame_done & (state==ST_DETECT);
  assign ev_tx_speed  = tx_frame_done & (state==ST_SPEED);
  assign ev_tx_adv    = tx_frame_done & (state==ST_ADVERTISE);
  assign ev_tx_cfg    = tx_frame_done & (state==ST_CONFIG);
  assign ev_oper_rx   = rx_good_frame & (state==ST_OPER);
  assign ev_oper_tx   = tx_frame_done & (state==ST_OPER);

`ifdef FORMAL
  // ---- safety invariants (k-induction) ----
  logic f_past_valid = 1'b0;
  always_ff @(posedge clk) f_past_valid <= 1'b1;
  // formal starts registers unconstrained; pin the initial state to the reset
  // value so the legal-state invariant is inductive.
  initial assume (state == ST_DETECT);

  always @(posedge clk) if (rst_n) begin
    a_state_legal:   assert (state <= ST_OPER);                 // legal state
    a_good_bound:    assert (rx_good <= 4'hF);                  // counters bounded
    a_lost_bound:    assert (rx_lost <= 4'hF);
    a_oper_iff:      assert (operational == (state==ST_OPER));  // operational <-> OPER
    a_good_xor_lost: assert (!(rx_good_frame && rx_lost_frame));
  end
  // link lost in Operational (7 consecutive lost) returns to Link Detect
  always @(posedge clk) if (f_past_valid && rst_n && $past(rst_n)
        && $past(rx_lost >= LOST_OPER_MAX) && $past(state==ST_OPER) && !$past(soft_reset))
    a_lost_to_detect: assert (state==ST_DETECT);

  // ---- reachability (BMC cover) ----
  always @(posedge clk) if (rst_n) begin
    c_reach_speed:  cover (state==ST_SPEED);
    c_reach_adv:    cover (state==ST_ADVERTISE);
    c_reach_config: cover (state==ST_CONFIG);
    c_reach_oper:   cover (state==ST_OPER);
  end
`endif
endmodule
`endif
