// Formal composition: two bidirectional I2C relays (SCM side with
// ARB_PRIORITY=1, HPM side with 0) exchanging events through the LTPI
// frame transport (modeled as a 1-cycle register with solver-controlled
// delivery strobes - frames arrive whenever the solver likes, including
// never, which subsumes CRC-dropped frames).
//
// Proves the property the single-relay proofs cannot: the two relays never
// both act as initiator of an in-progress transaction (mutual exclusion of
// the bus mastership claim), which is what makes bidirectional SPDM
// traffic safe. Covers demonstrate the HPM-initiated transaction (the SPDM
// response path) and the defer-then-claim scenario.
import ltpi_pkg::*;

module ltpi_i2c_loopback (
    input logic clk,
    input logic rst
);

    // ---------------- free environment ----------------
    (* anyseq *) logic a_start, a_stop, a_rise, a_fall, a_sda;
    (* anyseq *) logic b_start, b_stop, b_rise, b_fall, b_sda;
    (* anyseq *) logic dlv_a2b, dlv_b2a;    // frame delivery strobes

    logic f_past_valid = 1'b0;
    always_ff @(posedge clk) f_past_valid <= 1'b1;
    initial assume (rst);

    // ---------------- relays ----------------
    logic [3:0] a_tx, b_tx, a_state, b_state;
    logic a_init, b_init, a_def, b_def;
    logic a_stretch, b_stretch;

    // Transport: latch the sender's event on a delivery strobe; present it
    // with valid for one cycle.
    logic [3:0] a2b_ev, b2a_ev;
    logic       a2b_v,  b2a_v;
    always_ff @(posedge clk) begin
        if (rst) begin
            a2b_v <= 1'b0; b2a_v <= 1'b0;
        end else begin
            a2b_v  <= dlv_a2b;
            a2b_ev <= a_tx;
            b2a_v  <= dlv_b2a;
            b2a_ev <= b_tx;
        end
    end

    // Pacing feedback: a delivered frame IS a sent frame in this model -
    // the sender learns its (registered) event was carried when the
    // delivery strobe fires.
    ltpi_i2c_relay #(.ARB_PRIORITY(1'b1)) u_a (
        .clk(clk), .rst(rst),
        .start_det(a_start), .stop_det(a_stop),
        .scl_rise(a_rise), .scl_fall(a_fall), .sda_val(a_sda),
        .scl_stretch(a_stretch), .sda_pull(),
        .bus_start_gen(), .bus_stop_gen(),
        .tx_event(a_tx), .rx_event(b2a_ev), .rx_event_valid(b2a_v),
        .tx_sent_valid(a2b_v), .tx_sent_event(a2b_ev),
        .state(a_state), .initiator(a_init), .start_deferred(a_def)
    );

    ltpi_i2c_relay #(.ARB_PRIORITY(1'b0)) u_b (
        .clk(clk), .rst(rst),
        .start_det(b_start), .stop_det(b_stop),
        .scl_rise(b_rise), .scl_fall(b_fall), .sda_val(b_sda),
        .scl_stretch(b_stretch), .sda_pull(),
        .bus_start_gen(), .bus_stop_gen(),
        .tx_event(b_tx), .rx_event(a2b_ev), .rx_event_valid(a2b_v),
        .tx_sent_valid(b2a_v), .tx_sent_event(b2a_ev),
        .state(b_state), .initiator(b_init), .start_deferred(b_def)
    );

    localparam logic [3:0] S_IDLE = 4'd0, S_START = 4'd1, S_WAIT = 4'd2,
                           S_LDATA = 4'd3, S_RDATA = 4'd4,
                           S_RDATA_RCVD = 4'd5, S_STOP = 4'd6, S_STOP2 = 4'd7;

    // "Active initiator": claimed the bus AND progressed past the claim
    // (the peer confirmed with Start Received).
    logic a_active, b_active;
    assign a_active = a_init && a_state != S_START && a_state != S_IDLE;
    assign b_active = b_init && b_state != S_START && b_state != S_IDLE;

    // ---------------- lemmas (induction strengthening) ----------------
    // L1: responder-only events are never emitted by an initiator, and an
    // idle relay only emits Idle/Stop-Received-era events.
    function automatic logic resp_only(input logic [3:0] e);
        resp_only = (e == I2C_EV_START_RCVD) || (e == I2C_EV_START_ECHO)
                 || (e == I2C_EV_STOP_RCVD)  || (e == I2C_EV_STOP_ECHO);
    endfunction

    always_comb
        if (f_past_valid) begin
            if (a_init) assert (!resp_only(a_tx));
            if (b_init) assert (!resp_only(b_tx));
            if (a_state == S_IDLE) assert (a_tx == I2C_EV_IDLE);
            if (b_state == S_IDLE) assert (b_tx == I2C_EV_IDLE);
        end

    // L2: transport consistency - an in-flight Start Received implies the
    // sender was a responder when it was latched.
    always_ff @(posedge clk)
        if (f_past_valid && !$past(rst)) begin
            if (b2a_v && b2a_ev == I2C_EV_START_RCVD)
                assert (!$past(b_init));
            if (a2b_v && a2b_ev == I2C_EV_START_RCVD)
                assert (!$past(a_init));
        end

    // L3: a relay is a responder past S_START only while the peer still
    // claims (or has just released) the transaction - the responder's
    // non-idle states imply the peer is not idle-and-unclaimed... the
    // practical inductive form: an ACTIVE initiator implies its peer is
    // NOT an active initiator, checked directly below (S1); the lemmas
    // above cut the spurious induction states that would fake it.

    // ---------------- main safety property ----------------
    // S1: bus mastership mutual exclusion.
    always_comb
        if (f_past_valid)
            assert (!(a_active && b_active));

    // ---------------- covers ----------------
    logic f_b_bit_done = 1'b0;   // HPM-initiated bit fully handshaken
    logic f_b_deferred = 1'b0;
    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            if (b_init && $past(b_state) == S_LDATA && b_state == S_WAIT)
                f_b_bit_done <= 1'b1;
            if (b_def)
                f_b_deferred <= 1'b1;
        end
    end

    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            // C1: the SPDM response path - HPM claims the bus, ships a bit
            // through the full Event/Received handshake, SCM responds.
            cover (f_b_bit_done);
            // C2: HPM start deferred (arbitration or busy), later claimed.
            cover (f_b_deferred && b_init && b_state == S_WAIT);
            // C3: classic SCM-initiated direction still works.
            cover (a_init && a_state == S_WAIT);
            // C4: both sides tried to start; SCM won, HPM responds active.
            cover (a_active && b_def);
        end
    end

endmodule
