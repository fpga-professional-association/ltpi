// Formal-only composition: an SCM link FSM and an HPM link FSM connected
// back-to-back through a frame-event transport channel.
//
// The transport abstracts serialization: whenever a side completes a frame
// (tx_frame_done, a free input - the solver controls frame pacing), the
// frame type currently selected by that side's FSM is delivered to the
// other side as a received frame after a 1-cycle delay. The env may corrupt
// deliveries (crc_ok low) unless the clean-channel assumption is enabled,
// so safety properties hold under loss as well.
//
// The speed-capability payload carried by Link Detect frames is wired from
// each side's real local capabilities, so speed negotiation is end-to-end.
import ltpi_pkg::*;

module ltpi_loopback #(
    // Shrunk thresholds keep the cover traces (full bring-up) shallow;
    // safety properties do not depend on the values.
    parameter int unsigned DETECT_MIN_TX    = 4,
    parameter int unsigned DETECT_MIN_RX    = 3,
    parameter int unsigned ALIGN_MIN_RX     = 2,
    parameter int unsigned SPEED_SCM_MIN_TX = 2,
    parameter int unsigned SPEED_HPM_MIN_RX = 2,
    parameter int unsigned SPEED_TIMEOUT_TX = 16,
    parameter int unsigned ADV_MIN_CYCLES   = 4,
    parameter int unsigned ADV_ALIGN_TIMEOUT = 8,
    parameter int unsigned ADV_MIN_RX       = 2,
    parameter int unsigned CFG_MAX_TX       = 4,
    parameter int unsigned ACC_MAX_TX       = 3
)(
    input logic clk,
    input logic rst
);

    // ---------------- environment (free) inputs ----------------
    (* anyseq *) logic        scm_tx_done, hpm_tx_done;
    (* anyseq *) logic        scm_cfg_ready;
    (* anyseq *) logic        corrupt_s2h, corrupt_h2s;  // channel corruption
    (* anyseq *) logic [15:0] scm_caps, hpm_caps;

    // Hold capabilities stable and 25MHz-compliant (spec 3.1.1.1).
    always_comb begin
        assume (scm_caps[0]);
        assume (hpm_caps[0]);
    end
    logic f_past_valid = 1'b0;
    always_ff @(posedge clk) f_past_valid <= 1'b1;
    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            assume (scm_caps == $past(scm_caps));
            assume (hpm_caps == $past(hpm_caps));
        end
    end
    initial assume (rst);

    // ---------------- SCM side ----------------
    ltpi_pkg::frame_type_t scm_tx_type,  hpm_tx_type;
    ltpi_pkg::link_state_t scm_state,    hpm_state;
    logic [15:0] scm_speed_sel, hpm_speed_sel;
    logic        scm_speed_valid, hpm_speed_valid;
    logic        scm_link_up, hpm_link_up;
    logic [15:0] scm_remote_caps, hpm_remote_caps;
    logic        scm_rcaps_valid, hpm_rcaps_valid;

    // Transport: 1-cycle delayed frame-event delivery.
    logic        s2h_valid, h2s_valid;
    ltpi_pkg::frame_type_t s2h_type, h2s_type;
    logic        s2h_ok, h2s_ok;
    logic [15:0] s2h_payload, h2s_payload;

    always_ff @(posedge clk) begin
        if (rst) begin
            s2h_valid <= 1'b0;
            h2s_valid <= 1'b0;
        end else begin
            s2h_valid   <= scm_tx_done;
            s2h_type    <= scm_tx_type;
            s2h_ok      <= !corrupt_s2h;
            // Link Detect frames carry Speed Capabilities (Table 22);
            // Link Speed frames carry the Speed Select word (Table 24).
            s2h_payload <= (scm_tx_type == FRAME_LINK_SPEED)
                           ? scm_speed_sel : scm_caps;
            h2s_valid   <= hpm_tx_done;
            h2s_type    <= hpm_tx_type;
            h2s_ok      <= !corrupt_h2s;
            h2s_payload <= (hpm_tx_type == FRAME_LINK_SPEED)
                           ? hpm_speed_sel : hpm_caps;
        end
    end

    ltpi_link_fsm #(
        .ROLE_SCM(1'b1),
        .DETECT_MIN_TX(DETECT_MIN_TX), .DETECT_MIN_RX(DETECT_MIN_RX),
        .ALIGN_MIN_RX(ALIGN_MIN_RX),
        .SPEED_SCM_MIN_TX(SPEED_SCM_MIN_TX),
        .SPEED_HPM_MIN_RX(SPEED_HPM_MIN_RX),
        .SPEED_TIMEOUT_TX(SPEED_TIMEOUT_TX),
        .ADV_MIN_CYCLES(ADV_MIN_CYCLES), .ADV_MIN_RX(ADV_MIN_RX),
        .ADV_ALIGN_TIMEOUT(ADV_ALIGN_TIMEOUT),
        .CFG_MAX_TX(CFG_MAX_TX), .ACC_MAX_TX(ACC_MAX_TX)
    ) u_scm (
        .clk(clk), .rst(rst),
        .retrain_req(1'b0), .soft_reset(1'b0),
        .local_speed_caps(scm_caps),
        .cfg_ready(scm_cfg_ready),
        .rx_aligned(1'b1),
        .rx_frame_valid(h2s_valid),
        .rx_crc_ok(h2s_ok),
        .rx_frame_type(h2s_type),
        .rx_speed_payload(h2s_payload),
        .rx_cfg_match(1'b1),          // symmetric default capabilities
        .tx_frame_done(scm_tx_done),
        .tx_frame_type(scm_tx_type),
        .state(scm_state),
        .remote_speed_caps(scm_remote_caps),
        .remote_caps_valid(scm_rcaps_valid),
        .speed_select(scm_speed_sel),
        .speed_valid(scm_speed_valid),
        .link_up(scm_link_up)
    );

    ltpi_link_fsm #(
        .ROLE_SCM(1'b0),
        .DETECT_MIN_TX(DETECT_MIN_TX), .DETECT_MIN_RX(DETECT_MIN_RX),
        .ALIGN_MIN_RX(ALIGN_MIN_RX),
        .SPEED_SCM_MIN_TX(SPEED_SCM_MIN_TX),
        .SPEED_HPM_MIN_RX(SPEED_HPM_MIN_RX),
        .SPEED_TIMEOUT_TX(SPEED_TIMEOUT_TX),
        .ADV_MIN_CYCLES(ADV_MIN_CYCLES), .ADV_MIN_RX(ADV_MIN_RX),
        .ADV_ALIGN_TIMEOUT(ADV_ALIGN_TIMEOUT),
        .CFG_MAX_TX(CFG_MAX_TX), .ACC_MAX_TX(ACC_MAX_TX)
    ) u_hpm (
        .clk(clk), .rst(rst),
        .retrain_req(1'b0), .soft_reset(1'b0),
        .local_speed_caps(hpm_caps),
        .cfg_ready(1'b0),             // HPM never initiates Configure
        .rx_aligned(1'b1),
        .rx_frame_valid(s2h_valid),
        .rx_crc_ok(s2h_ok),
        .rx_frame_type(s2h_type),
        .rx_speed_payload(s2h_payload),
        .rx_cfg_match(1'b1),
        .tx_frame_done(hpm_tx_done),
        .tx_frame_type(hpm_tx_type),
        .state(hpm_state),
        .remote_speed_caps(hpm_remote_caps),
        .remote_caps_valid(hpm_rcaps_valid),
        .speed_select(hpm_speed_sel),
        .speed_valid(hpm_speed_valid),
        .link_up(hpm_link_up)
    );

    // ---------------- helper history ----------------
    logic f_scm_ever_op = 1'b0;
    logic f_scm_ever_cfg = 1'b0;
    always_ff @(posedge clk) begin
        if (f_past_valid && scm_state == ST_OPERATIONAL) f_scm_ever_op  <= 1'b1;
        if (f_past_valid && scm_state == ST_CFG_ACC)     f_scm_ever_cfg <= 1'b1;
    end

    // The unique agreed speed-select value implied by the two (constant)
    // capability words - what both sides must converge on.
    function automatic logic [15:0] highest_bit16(input logic [15:0] v);
        integer i;
        begin
            highest_bit16 = 16'h0000;
            for (i = 0; i < 16; i = i + 1)
                if (v[i]) highest_bit16 = 16'h0001 << i;
        end
    endfunction

    logic [15:0] f_common, f_sel_star;
    assign f_common = scm_caps & hpm_caps;
    assign f_sel_star = ((f_common & 16'h0FFF) != 0
                         ? highest_bit16(f_common & 16'h0FFF) : 16'h0001)
                        | (f_common[15] ? 16'h8000 : 16'h0000);

    // ---------------- induction-strengthening invariants ----------------
    // T1: transport payload consistency - a Link Detect delivery carries the
    // sender's capability word, a Link Speed delivery carries the agreed
    // select (senders in ST_SPEED provably hold speed_valid, see FSM P11).
    always_comb begin
        if (f_past_valid && s2h_valid) begin
            if (s2h_type == FRAME_LINK_DETECT) assert (s2h_payload == scm_caps);
            if (s2h_type == FRAME_LINK_SPEED)  assert (s2h_payload == f_sel_star);
            if (s2h_type == FRAME_CONFIGURE)   assert (f_scm_ever_cfg);
            if (s2h_type == FRAME_OPERATIONAL) assert (f_scm_ever_op);
        end
        if (f_past_valid && h2s_valid) begin
            if (h2s_type == FRAME_LINK_DETECT) assert (h2s_payload == hpm_caps);
            if (h2s_type == FRAME_LINK_SPEED)  assert (h2s_payload == f_sel_star);
        end
    end

    // T2: whenever either side considers its remote capabilities captured,
    // they are exactly the peer's (constant) capability word.
    always_comb begin
        if (f_past_valid && scm_rcaps_valid)
            assert (scm_remote_caps == hpm_caps);
        if (f_past_valid && hpm_rcaps_valid)
            assert (hpm_remote_caps == scm_caps);
    end

    // T3: every valid speed selection equals the agreed value - stronger
    // than L3 and inductive on its own.
    always_comb begin
        if (f_past_valid && scm_speed_valid) assert (scm_speed_sel == f_sel_star);
        if (f_past_valid && hpm_speed_valid) assert (hpm_speed_sel == f_sel_star);
    end

    // ---------------- composition safety properties ----------------

    // L1: the HPM cannot reach Accept without the SCM ever having entered
    // Configure: the only frame that moves HPM out of Advertise is a
    // Configure frame, and only the SCM's Configure state emits it.
    always_comb
        if (f_past_valid && hpm_state == ST_CFG_ACC)
            assert (f_scm_ever_cfg);

    // L2: the HPM cannot be Operational unless the SCM has ever been
    // Operational: HPM needs an Operational frame, which only an
    // Operational SCM emits.
    always_comb
        if (f_past_valid && hpm_state == ST_OPERATIONAL)
            assert (f_scm_ever_op);

    // L3: speed agreement. Both sides compute the selection from the same
    // pair of capability words, so whenever both have a valid selection it
    // is identical - the core interoperability guarantee of Link Training.
    always_comb
        if (f_past_valid && scm_speed_valid && hpm_speed_valid)
            assert (scm_speed_sel == hpm_speed_sel);

    // L4: whenever both selections are valid they are one-hot in the speed
    // field and supported by BOTH sides' capabilities (or the mandatory
    // 25MHz fallback).
    always_comb
        if (f_past_valid && scm_speed_valid && hpm_speed_valid) begin
            assert ((scm_speed_sel & 16'h0FFF) != 0);
            assert (((scm_speed_sel & 16'h0FFF)
                     & ((scm_speed_sel & 16'h0FFF) - 1)) == 0);
            assert (((scm_speed_sel & 16'h0FFF) & scm_caps & hpm_caps) != 0
                    || (scm_speed_sel & 16'h0FFF) == 16'h0001);
            // DDR selected only if both sides are DDR-capable.
            assert (!scm_speed_sel[15] || (scm_caps[15] && hpm_caps[15]));
        end

    // ---------------- covers: the link really comes up ----------------
    logic f_both_up_seen = 1'b0;
    always_ff @(posedge clk)
        if (f_past_valid && scm_link_up && hpm_link_up)
            f_both_up_seen <= 1'b1;

    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            cover (scm_link_up && hpm_link_up);        // full bring-up
            // Bring-up negotiating 400MHz DDR: both sides advertise the
            // project default caps (25M + 200M SDR + 400M SDR + DDR).
            cover (scm_link_up && hpm_link_up
                   && scm_caps == CAPS_DEFAULT && hpm_caps == CAPS_DEFAULT
                   && scm_speed_sel == (CAP_400M_SDR | CAP_DDR));
            // Bring-up at 200MHz SDR against an SDR-only 200M peer.
            cover (scm_link_up && hpm_link_up
                   && scm_caps == CAPS_DEFAULT
                   && hpm_caps == (CAP_25M_SDR | CAP_200M_SDR)
                   && scm_speed_sel == CAP_200M_SDR);
        end
    end

`ifdef FORMAL
`endif

endmodule
