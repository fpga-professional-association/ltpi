// LTPI Link Training / Configuration / Operational state machine.
// Implements spec section 4 (Figure 27) for both roles:
//   ROLE_SCM = 1 : SCM CPLD side (sends Configure, waits for Accept)
//   ROLE_SCM = 0 : HPM FPGA side (waits for Configure, sends Accept)
//
// Threshold parameters default to the spec values (Tables 37, 38, 42, 43, 47)
// and are overridable so formal cover traces stay shallow.
//
// Deviations/clarifications from the spec text:
//  - Spec 4.1.2.2 body says "after sending 32 Configure Frames"; Table 43
//    says max TX = 31. We follow Table 43 (CFG_MAX_TX = 31).
//  - "Frame Alignment Lost" inside Advertise/Configure/Operational is treated
//    as an immediate link-lost -> Link Detect (conservative vs. counting it
//    into the 3/7 consecutive-lost-frame budget).
import ltpi_pkg::*;

module ltpi_link_fsm #(
    parameter bit ROLE_SCM = 1'b1,
    parameter int unsigned DETECT_MIN_TX    = 255,   // Table 37
    parameter int unsigned DETECT_MIN_RX    = 7,     // Table 37 (consecutive)
    parameter int unsigned ALIGN_MIN_RX     = 3,     // Fig 27 alignment parts
    parameter int unsigned SPEED_SCM_MIN_TX = 7,     // Table 38
    parameter int unsigned SPEED_HPM_MIN_RX = 3,     // Table 38
    parameter int unsigned SPEED_TIMEOUT_TX = 255,   // Table 38
    parameter int unsigned ADV_MIN_CYCLES   = 25000, // ~1ms min TX; Table 42
    // Spec 2.1v1.0 raised the Advertise Frame ALIGNMENT timeout to 100ms
    // (PLL relock allowance), distinct from the 1ms minimum TX time.
    parameter int unsigned ADV_ALIGN_TIMEOUT = 2_500_000, // ~100ms @ 25MHz
    parameter int unsigned ADV_MIN_RX       = 3,     // Table 42
    parameter int unsigned CFG_MAX_TX       = 31,    // Table 43 (SCM)
    parameter int unsigned ACC_MAX_TX       = 15,    // Table 43 (HPM)
    parameter int unsigned ADV_LOST_LIMIT   = 3,     // Table 42/43
    parameter int unsigned OP_LOST_LIMIT    = 7      // Table 47
)(
    input  logic        clk,
    input  logic        rst,          // physical reset
    input  logic        retrain_req,  // BMC / external link retraining request
    input  logic        soft_reset,   // soft reset: Operational -> Advertise
    input  logic [15:0] local_speed_caps, // Table 21 encoding, bit0 = 25MHz
    input  logic        cfg_ready,    // SCM: configuration selected (auto/BMC)
    // RX side (from ltpi_frame_rx)
    input  logic        rx_aligned,
    input  logic        rx_frame_valid,
    input  logic        rx_crc_ok,
    input  ltpi_pkg::frame_type_t rx_frame_type,
    input  logic [15:0] rx_speed_payload, // speed caps word of RX detect frame
    input  logic        rx_cfg_match, // SCM: Accept matches the requested cfg
                                      // HPM: Configure matches capabilities
    // TX side (to frame serializer)
    input  logic        tx_frame_done,   // one whole frame has been sent
    output ltpi_pkg::frame_type_t tx_frame_type,
    // Status
    output ltpi_pkg::link_state_t state,
    output logic [15:0] remote_speed_caps,
    output logic        remote_caps_valid,
    output logic [15:0] speed_select,    // one-hot, Table 23 encoding
    output logic        speed_valid,
    output logic        link_up
);

    // Speed bits are [11:0] of the caps word; [15] is the DDR flag (Table 21).
    localparam logic [15:0] SPEED_MASK = 16'h0FFF;

    localparam int unsigned TXW = 8;                    // fits 255
    localparam int unsigned ATW_MAX = (ADV_ALIGN_TIMEOUT > ADV_MIN_CYCLES)
                                      ? ADV_ALIGN_TIMEOUT : ADV_MIN_CYCLES;
    localparam int unsigned ATW = $clog2(ATW_MAX + 1);

    logic [TXW-1:0] tx_cnt;       // per-state TX frame counter (saturating)
    logic [3:0]     rx_cnt;       // per-state good-RX counter   (saturating)
    logic [2:0]     lost_cnt;     // consecutive lost frames
    logic [ATW-1:0] adv_timer;    // cycles spent in ADV_ALIGN / ADV

    // RX event decode
    logic rx_good;
    assign rx_good = rx_frame_valid && rx_crc_ok;

    logic rx_good_detect, rx_good_speed, rx_good_adv, rx_good_cfg,
          rx_good_acc, rx_good_op;
    assign rx_good_detect = rx_good && rx_frame_type == FRAME_LINK_DETECT;
    assign rx_good_speed  = rx_good && rx_frame_type == FRAME_LINK_SPEED;
    assign rx_good_adv    = rx_good && rx_frame_type == FRAME_ADVERTISE;
    assign rx_good_cfg    = rx_good && rx_frame_type == FRAME_CONFIGURE;
    assign rx_good_acc    = rx_good && rx_frame_type == FRAME_ACCEPT;
    assign rx_good_op     = rx_good && (rx_frame_type == FRAME_OPERATIONAL
                                        || rx_frame_type == FRAME_OP_DATA);

    // Expected frame types per state; anything else (or a CRC error) is a
    // "lost frame" event in the states that police the link (Tables 42/43/47).
    logic rx_expected;
    always_comb begin
        case (state)
            ST_ADV:     rx_expected = rx_good_adv || (!ROLE_SCM && rx_good_cfg);
            ST_CFG_ACC: rx_expected = ROLE_SCM
                                      ? (rx_good_adv || rx_good_acc)
                                      : (rx_good_cfg || rx_good_op);
            // HPM may lag in Accept: SCM still tolerates Accept frames here.
            ST_OPERATIONAL: rx_expected = rx_good_op
                                          || (ROLE_SCM && rx_good_acc);
            default:    rx_expected = 1'b1;  // training states don't police
        endcase
    end

    logic rx_lost_event;
    assign rx_lost_event = rx_frame_valid && !rx_expected;

    logic policing;
    assign policing = (state == ST_ADV) || (state == ST_CFG_ACC)
                                        || (state == ST_OPERATIONAL);

    // Refined Operational link-lost (spec 2.1v1.1, note under Figure 19):
    //   a) IO_LOST_LIMIT consecutive IO frames dropped by CRC errors,
    //      REGARDLESS of correctly received Data frames in between;
    //   b) OP_LOST_LIMIT consecutive operational frames (IO+Data) lost.
    // Training/config states keep the flat ADV_LOST_LIMIT budget.
    localparam int unsigned IO_LOST_LIMIT = 3;
    logic [1:0] io_bad_run;   // CRC-dropped IO frames, Data-frame-immune

    logic io_crc_drop, io_good;
    assign io_crc_drop = rx_frame_valid && !rx_crc_ok
                         && rx_frame_type == FRAME_OPERATIONAL;
    assign io_good     = rx_good && rx_frame_type == FRAME_OPERATIONAL;

    logic [2:0] lost_limit;
    assign lost_limit = (state == ST_OPERATIONAL) ? 3'(OP_LOST_LIMIT)
                                                  : 3'(ADV_LOST_LIMIT);

    logic link_lost;
    assign link_lost = policing
                       && ((rx_lost_event && lost_cnt == lost_limit - 1)
                           || (state == ST_OPERATIONAL && io_crc_drop
                               && io_bad_run == 2'(IO_LOST_LIMIT - 1))
                           || !rx_aligned);

    // ------------------------------------------------------------------
    // Next-state logic (Figure 27)
    // ------------------------------------------------------------------
    ltpi_pkg::link_state_t nstate;
    logic detect_done, speed_done, speed_timeout, adv_align_done,
          adv_align_timeout, adv_done, cfg_done, cfg_timeout;

    // Link Detect exit: (>=255 TX and >=7 consecutive good RX) OR any Link
    // Speed frame received (other side is already ahead).
    assign detect_done = (tx_cnt >= TXW'(DETECT_MIN_TX)
                          && rx_cnt >= 4'(DETECT_MIN_RX))
                         || rx_good_speed;

    // Link Speed exit: SCM by TX count, HPM by RX count (Note 3).
    assign speed_done    = ROLE_SCM ? (tx_cnt >= TXW'(SPEED_SCM_MIN_TX))
                                    : (rx_cnt >= 4'(SPEED_HPM_MIN_RX));
    assign speed_timeout = tx_cnt >= TXW'(SPEED_TIMEOUT_TX);

    assign adv_align_done    = rx_cnt >= 4'(ALIGN_MIN_RX);
    // 100ms alignment timeout (2.1v1.0) - PLL relock allowance, separate
    // from the 1ms minimum-TX time used in ST_ADV.
    assign adv_align_timeout = adv_timer >= ATW'(ADV_ALIGN_TIMEOUT);

    // Advertise exit: SCM proceeds to Configure once the 1ms minimum TX time
    // elapsed, >=3 good frames arrived and a configuration was selected.
    // HPM leaves as soon as a matching Configure shows up (even from the
    // alignment part - receiving it proves alignment).
    assign adv_done = ROLE_SCM
                      ? (adv_timer >= ATW'(ADV_MIN_CYCLES)
                         && rx_cnt >= 4'(ADV_MIN_RX) && cfg_ready)
                      : (rx_good_cfg && rx_cfg_match);

    // Configure/Accept exit into Operational.
    assign cfg_done    = ROLE_SCM ? (rx_good_acc && rx_cfg_match)
                                  : rx_good_op;
    assign cfg_timeout = tx_cnt >= (ROLE_SCM ? TXW'(CFG_MAX_TX)
                                             : TXW'(ACC_MAX_TX));

    always_comb begin
        nstate = state;
        if (retrain_req)
            nstate = ST_DETECT_ALIGN;
        else begin
            case (state)
                ST_DETECT_ALIGN:
                    if (rx_good_speed)        nstate = ST_SPEED;
                    else if (rx_cnt >= 4'(ALIGN_MIN_RX)) nstate = ST_DETECT;
                ST_DETECT:
                    if (detect_done)          nstate = ST_SPEED;
                ST_SPEED:
                    if (speed_done)           nstate = ST_ADV_ALIGN;
                    else if (speed_timeout)   nstate = ST_DETECT_ALIGN;
                ST_ADV_ALIGN:
                    if (!ROLE_SCM && rx_good_cfg && rx_cfg_match)
                                              nstate = ST_CFG_ACC;
                    else if (adv_align_done)  nstate = ST_ADV;
                    else if (adv_align_timeout) nstate = ST_DETECT_ALIGN;
                ST_ADV:
                    if (link_lost)            nstate = ST_DETECT_ALIGN;
                    else if (adv_done)        nstate = ST_CFG_ACC;
                ST_CFG_ACC:
                    if (link_lost)            nstate = ST_DETECT_ALIGN;
                    else if (cfg_done)        nstate = ST_OPERATIONAL;
                    else if (cfg_timeout)     nstate = ST_ADV;
                ST_OPERATIONAL:
                    if (link_lost)            nstate = ST_DETECT_ALIGN;
                    else if (soft_reset)      nstate = ST_ADV_ALIGN;
                default:                      nstate = ST_DETECT_ALIGN;
            endcase
        end
    end

    logic state_change;
    assign state_change = (nstate != state);

    // ------------------------------------------------------------------
    // State & counters
    // ------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            state <= ST_DETECT_ALIGN;
        end else begin
            state <= nstate;
        end
    end

    // Per-state TX counter, saturating at 255, cleared on any state change.
    always_ff @(posedge clk) begin
        if (rst || state_change)
            tx_cnt <= '0;
        else if (tx_frame_done && tx_cnt != '1)
            tx_cnt <= tx_cnt + 1'b1;
    end

    // Per-state good-RX counter (saturating at 15). In ST_DETECT the count
    // must be of CONSECUTIVE good frames: any CRC-failed frame resets it.
    logic rx_count_event;
    always_comb begin
        case (state)
            ST_DETECT_ALIGN: rx_count_event = rx_good_detect;
            ST_DETECT:       rx_count_event = rx_good_detect;
            ST_SPEED:        rx_count_event = rx_good_speed;
            ST_ADV_ALIGN:    rx_count_event = rx_good_adv || rx_good_cfg;
            ST_ADV:          rx_count_event = rx_good_adv;
            default:         rx_count_event = 1'b0;
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst || state_change)
            rx_cnt <= '0;
        else if (state == ST_DETECT && rx_frame_valid && !rx_crc_ok)
            rx_cnt <= '0;                          // consecutive requirement
        else if (rx_count_event && rx_cnt != '1)
            rx_cnt <= rx_cnt + 1'b1;
    end

    // Consecutive-lost-frame counter for the policing states.
    always_ff @(posedge clk) begin
        if (rst || state_change || !policing)
            lost_cnt <= '0;
        else if (rx_lost_event)
            lost_cnt <= lost_cnt + 1'b1;   // link_lost fires before overflow
        else if (rx_good && rx_expected)
            lost_cnt <= '0;
    end

    // CRC-dropped IO frame run (Operational only). Good DATA frames do NOT
    // reset it - only a correctly received IO frame does (2.1v1.1).
    always_ff @(posedge clk) begin
        if (rst || state_change || state != ST_OPERATIONAL)
            io_bad_run <= '0;
        else if (io_crc_drop)
            io_bad_run <= io_bad_run + 1'b1;  // link_lost fires at the limit
        else if (io_good)
            io_bad_run <= '0;
    end

    // Advertise-phase timer (PLL relock compensation / minimum TX time).
    always_ff @(posedge clk) begin
        if (rst || (state != ST_ADV_ALIGN && state != ST_ADV))
            adv_timer <= '0;
        else if (state_change)
            adv_timer <= '0;               // restart on ALIGN -> ADV
        else if (adv_timer != ATW'(ATW_MAX))
            adv_timer <= adv_timer + 1'b1; // saturate
    end

    // ------------------------------------------------------------------
    // Speed capability capture & selection (spec 4.1.1.2)
    // ------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            remote_speed_caps <= '0;
            remote_caps_valid <= 1'b0;
        end else if ((state == ST_DETECT_ALIGN || state == ST_DETECT)
                     && rx_good_detect) begin
            remote_speed_caps <= rx_speed_payload;
            remote_caps_valid <= 1'b1;
        end else if (nstate == ST_DETECT_ALIGN && state != ST_DETECT_ALIGN) begin
            remote_caps_valid <= 1'b0;   // retraining re-exchanges caps
        end
    end

    // Highest common frequency; 25MHz (bit 0) is mandatory for all
    // implementations, so it is the guaranteed fallback. DDR only when both
    // sides support it.
    logic [15:0] common_caps;
    assign common_caps = local_speed_caps & remote_speed_caps;

    function automatic logic [15:0] highest_bit(input logic [15:0] v);
        integer i;
        begin
            highest_bit = 16'h0000;
            for (i = 0; i < 16; i = i + 1)   // last set bit wins => highest
                if (v[i]) highest_bit = 16'h0001 << i;
        end
    endfunction

    // Sanitized adoption of a peer's Speed Select word (Table 24): accept it
    // only if it is a single supported speed; otherwise fall back to the
    // mandatory 25MHz. DDR only if we are DDR-capable too.
    function automatic logic [15:0] adopt_select(input logic [15:0] payload,
                                                 input logic [15:0] local_caps);
        logic [15:0] s;
        begin
            s = payload & SPEED_MASK;
            if (s != 0 && (s & (s - 1)) == 0 && (s & local_caps) == s)
                adopt_select = s;
            else
                adopt_select = 16'h0001;
            if (payload[15] && local_caps[15])
                adopt_select = adopt_select | 16'h8000;
        end
    endfunction

    always_ff @(posedge clk) begin
        if (rst) begin
            speed_select <= '0;
            speed_valid  <= 1'b0;
        end else if ((state == ST_DETECT || state == ST_DETECT_ALIGN)
                     && nstate == ST_SPEED) begin
            if (rx_good_speed)
                // Exit forced by a peer already in Link Speed: adopt the
                // Speed Select it sent (we may never have seen its caps).
                speed_select <= adopt_select(rx_speed_payload,
                                             local_speed_caps);
            else
                // Normal exit: highest common frequency from the exchanged
                // capabilities (spec 4.1.1.2).
                speed_select <= ((common_caps & SPEED_MASK) != 0
                                 ? highest_bit(common_caps & SPEED_MASK)
                                 : 16'h0001)
                                | (local_speed_caps[15] & remote_speed_caps[15]
                                   ? 16'h8000 : 16'h0000);
            speed_valid  <= 1'b1;
        end else if (nstate == ST_DETECT_ALIGN && state != ST_DETECT_ALIGN) begin
            speed_valid  <= 1'b0;          // retraining renegotiates speed
        end
    end

    // ------------------------------------------------------------------
    // TX frame selection & status
    // ------------------------------------------------------------------
    always_comb begin
        case (state)
            ST_DETECT_ALIGN, ST_DETECT: tx_frame_type = FRAME_LINK_DETECT;
            ST_SPEED:                   tx_frame_type = FRAME_LINK_SPEED;
            ST_ADV_ALIGN, ST_ADV:       tx_frame_type = FRAME_ADVERTISE;
            ST_CFG_ACC:                 tx_frame_type = ROLE_SCM ? FRAME_CONFIGURE
                                                                 : FRAME_ACCEPT;
            default:                    tx_frame_type = FRAME_OPERATIONAL;
        endcase
    end

    assign link_up = (state == ST_OPERATIONAL);

`ifdef FORMAL
    logic f_past_valid = 1'b0;
    always_ff @(posedge clk)
        f_past_valid <= 1'b1;

    initial assume (rst);

    // ---- Environment assumptions ------------------------------------
    // 25MHz base frequency support is mandatory for every implementation
    // (spec 3.1.1.1), and capability straps don't change on the fly.
    always_comb assume (local_speed_caps[0]);
    always_ff @(posedge clk)
        if (f_past_valid)
            assume (local_speed_caps == $past(local_speed_caps));
    // A detect frame from a compliant peer also advertises 25MHz.
    always_comb
        if (rx_frame_valid && rx_frame_type == FRAME_LINK_DETECT && rx_crc_ok)
            assume (rx_speed_payload[0]);
    // Frames only complete while the SERDES is aligned.
    always_comb
        if (rx_frame_valid) assume (rx_aligned);

    // ---- Safety properties (spec Figure 27 / Tables 37-47) ----------

    // P1: state register only ever holds a defined state.
    // (all combinational assertions are vacuous at time zero: the register
    // state there is unconstrained, reset applies from the first edge on)
    always_comb
        assert (!f_past_valid || state <= ST_OPERATIONAL);

    // P2: link_up exactly in Operational.
    always_comb
        assert (!f_past_valid || link_up == (state == ST_OPERATIONAL));

    // P3: Operational can only be entered from Configure/Accept, and only
    // by the spec handshake: SCM saw a matching Accept, HPM saw an
    // Operational frame. No path skips the configuration exchange.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst)
            && state == ST_OPERATIONAL && $past(state) != ST_OPERATIONAL) begin
            assert ($past(state) == ST_CFG_ACC);
            assert (!$past(retrain_req));
            if (ROLE_SCM)
                assert ($past(rx_good_acc && rx_cfg_match));
            else
                assert ($past(rx_good_op));
        end
    end

    // P4: Link Detect exit discipline (Table 37): leaving ST_DETECT for
    // ST_SPEED requires 255 TX + 7 consecutive good RX, or a Link Speed
    // frame from the peer.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst)
            && $past(state) == ST_DETECT && state == ST_SPEED)
            assert ($past(detect_done));
    end

    // P5: retrain request always forces Link Detect on the next cycle.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && $past(retrain_req))
            assert (state == ST_DETECT_ALIGN);
    end

    // P6: soft reset in Operational falls back to Advertise, never further
    // (Table 47: soft reset -> Advertise, hard reset -> Link Detect).
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && !$past(retrain_req)
            && $past(state) == ST_OPERATIONAL && $past(soft_reset)
            && !$past(link_lost))
            assert (state == ST_ADV_ALIGN);
    end

    // P7: losing the link in a policing state always lands in Link Detect.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && $past(link_lost))
            assert (state == ST_DETECT_ALIGN);
    end

    // P8: speed selection is sound whenever valid: exactly one speed bit,
    // and it is a speed the local side really supports; when the remote
    // capabilities are known-compatible it is common to both.
    logic [15:0] f_speed_bits;
    assign f_speed_bits = speed_select & SPEED_MASK;
    always_comb begin
        if (f_past_valid && speed_valid) begin
            assert (f_speed_bits != 0
                    && (f_speed_bits & (f_speed_bits - 1)) == 0); // one-hot
            assert ((f_speed_bits & local_speed_caps) == f_speed_bits
                    || f_speed_bits == 16'h0001);            // local or fallback
            assert (!speed_select[14] && !speed_select[13]
                    && !speed_select[12]);                   // reserved bits
        end
    end

    // P9: counter bounds (these double as induction strengthening).
    always_comb begin
        if (f_past_valid) begin
        assert (lost_cnt < (state == ST_OPERATIONAL ? 3'(OP_LOST_LIMIT)
                                                    : 3'(ADV_LOST_LIMIT))
                || !policing);
        assert (adv_timer <= ATW'(ATW_MAX));
        if (!policing) assert (lost_cnt == '0);
        // Refined IO-drop budget (2.1v1.1): the run never reaches the
        // limit inside Operational - link_lost fires first.
        assert (io_bad_run < 2'(IO_LOST_LIMIT) || state != ST_OPERATIONAL);
        if (state != ST_OPERATIONAL) assert (io_bad_run == '0);
        end
    end

    // P11: a valid speed selection exists in every state past Link Training
    // (needed by composition proofs: a side emitting Link Speed frames has
    // a select word to put in them).
    always_comb
        if (f_past_valid
            && (state == ST_SPEED || state == ST_ADV_ALIGN || state == ST_ADV
                || state == ST_CFG_ACC || state == ST_OPERATIONAL))
            assert (speed_valid);

    // P12: any progress in the Link Detect RX counter implies the remote
    // capabilities were captured (every counted frame also latches caps),
    // so the "computed" speed-select path never uses stale/empty caps.
    always_comb
        if (f_past_valid
            && (state == ST_DETECT || state == ST_DETECT_ALIGN)
            && rx_cnt != 0)
            assert (remote_caps_valid);

    // P10: Configure retry budget (Table 43): leaving CFG for Advertise
    // happens only on the TX-count timeout, without a completed handshake.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst)
            && $past(state) == ST_CFG_ACC && state == ST_ADV)
            assert ($past(cfg_timeout) && !$past(cfg_done)
                    && !$past(link_lost));
    end

    // ---- Cover properties (reachability sanity) ---------------------
    // Shallow-parameter tasks in the .sby make these tractable.
    logic f_was_operational = 1'b0;
    logic f_retrained       = 1'b0;
    always_ff @(posedge clk) begin
        if (f_past_valid && state == ST_OPERATIONAL)
            f_was_operational <= 1'b1;
        if (f_past_valid && f_was_operational && state == ST_DETECT_ALIGN)
            f_retrained <= 1'b1;
    end

    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            cover (state == ST_OPERATIONAL);                   // link comes up
            cover (f_retrained && state == ST_OPERATIONAL);    // full retrain
            cover (state == ST_ADV
                   && $past(state) == ST_CFG_ACC);             // cfg timeout
            cover (speed_valid && speed_select == 16'h0001);   // 25MHz fallback
            cover (speed_valid && speed_select[15]);           // DDR selected
        end
    end
`endif

endmodule

