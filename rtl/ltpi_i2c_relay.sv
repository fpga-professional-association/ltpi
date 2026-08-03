// LTPI I2C/SMBus relay - event-based tunneling per spec 2.2.1.3
// (Figure 16/18, Tables 9 and 10) - BIDIRECTIONAL, frame-paced.
//
// Spec-level gaps this core closes:
//
// 1. BIDIRECTIONAL INITIATION (SPDM/MCTP requirement). LTPI 1.0 fixes the
//    bus controller on the SCM side, but SPDM responders master the bus to
//    send responses. This relay claims the initiator role PER TRANSACTION
//    on either side. Simultaneous-start arbitration: ARB_PRIORITY=1 (SCM)
//    wins; the loser - and any side whose master starts while a remote
//    transaction runs - DEFERS its local START via clock stretching and
//    forwards it once the bus is idle again.
//
// 2. EVENT-CHANGE SEMANTICS. Events repeat in every frame ("sent
//    continuously until new event is generated"), so all transitions fire
//    on delivered-value CHANGES (rx_new); the Echo events separate
//    identical consecutive events.
//
// 3. EVENT PACING. A new event may only REPLACE the current tx_event after
//    at least one frame has actually carried it (tx_sent_* feedback from
//    the frame layer) - otherwise a fast local bus can overwrite e.g. a
//    Data Received Echo between two frames and the peer never sees it.
//    Local bus pulses that arrive while the previous event is still
//    unsent are latched (pend_*) and the local bus is held by stretch.
//
// Clock stretching is supported in both directions: the initiator's bus
// is held until the peer confirms regeneration (R3); a stretching target
// on the responder side delays the real SCL edge that gates Data Received
// (R10), which propagates back as initiator-side stretch. No timeouts.
import ltpi_pkg::*;

module ltpi_i2c_relay #(
    parameter bit ARB_PRIORITY = 1'b1   // wins a simultaneous-start race
)(
    input  logic clk,
    input  logic rst,

    // Local bus signal detection (single-cycle pulses / sampled level)
    input  logic start_det,   // SDA falling edge while SCL high
    input  logic stop_det,    // SDA rising edge while SCL high
    input  logic scl_rise,
    input  logic scl_fall,
    input  logic sda_val,     // SDA sampled at scl_rise

    // Local bus open-drain controls
    output logic scl_stretch,   // hold SCL low (stretch / deferral / pacing)
    output logic sda_pull,      // pull SDA low (0-bit / START shaping)
    output logic bus_start_gen, // responder: shape a START on the bus
    output logic bus_stop_gen,  // responder: shape a STOP on the bus

    // LTPI event channel (4-bit field in every I/O frame, Table 11)
    output logic [3:0] tx_event,
    input  logic [3:0] rx_event,
    input  logic       rx_event_valid,   // event arrived in a CRC-good frame
    // Frame-layer feedback: a completed TX frame carried tx_sent_event.
    input  logic       tx_sent_valid,
    input  logic [3:0] tx_sent_event,

    // Status
    output logic [3:0] state,
    output logic       initiator,        // we source the current transaction
    output logic       start_deferred    // local START pending via stretch
);

    localparam logic [3:0] S_IDLE       = 4'd0;
    localparam logic [3:0] S_START      = 4'd1; // I: await Start Received
                                                // R: regenerating START
    localparam logic [3:0] S_WAIT       = 4'd2; // between bits
    localparam logic [3:0] S_LDATA      = 4'd3; // local bit sent, await Rcvd
    localparam logic [3:0] S_RDATA      = 4'd4; // remote bit on local bus
    localparam logic [3:0] S_RDATA_RCVD = 4'd5; // sent Data Rcvd, await Echo
    localparam logic [3:0] S_STOP       = 4'd6; // I: await Stop Received
                                                // R: regenerating STOP
    localparam logic [3:0] S_STOP2      = 4'd7; // R: Stop done, announcing

    logic rxe;
    assign rxe = rx_event_valid;

    // Delivered-value change detection.
    logic [3:0] prev_rx_ev;
    always_ff @(posedge clk) begin
        if (rst)
            prev_rx_ev <= I2C_EV_IDLE;
        else if (rxe)
            prev_rx_ev <= rx_event;
    end
    logic rx_new;
    assign rx_new = rxe && (rx_event != prev_rx_ev);

    logic rx_is_data;
    assign rx_is_data = (rx_event == I2C_EV_DATA0) || (rx_event == I2C_EV_DATA1);

    // ------------------------------------------------------------------
    // Event pacing: has the current tx_event been carried by a frame?
    // ------------------------------------------------------------------
    // Spec 2.1v1.1: Data Echo and Data Received Echo must be sent at least
    // 3 times before the sender may move on; other events need one framed
    // transmission (pacing) before being replaced.
    logic [3:0] tx_event_q;
    logic [1:0] sent_cnt;
    always_ff @(posedge clk) begin
        if (rst) begin
            tx_event_q <= I2C_EV_IDLE;
            sent_cnt   <= 2'd3;
        end else begin
            tx_event_q <= tx_event;
            if (tx_event != tx_event_q)
                sent_cnt <= 2'd0;
            else if (tx_sent_valid && tx_sent_event == tx_event
                     && sent_cnt != 2'd3)
                sent_cnt <= sent_cnt + 1'b1;
        end
    end

    logic echo3;   // events with the triple-send requirement
    assign echo3 = (tx_event == I2C_EV_DATA0_ECHO)
                || (tx_event == I2C_EV_DATA1_ECHO)
                || (tx_event == I2C_EV_DRCVD_ECHO);

    logic ok_change;   // safe to replace tx_event now
    assign ok_change = (tx_event == tx_event_q)
                       && (sent_cnt >= (echo3 ? 2'd3 : 2'd1));

    // ------------------------------------------------------------------
    // Transaction direction tracking ("controller" = initiator side)
    // ------------------------------------------------------------------
    logic [3:0] bit_pos;    // 0..8 within the 9-bit byte+ack unit
    logic       first_byte; // address byte in progress
    logic       rw_read;    // R/W bit of the address byte (1 = read)

    logic init_sources;
    always_comb begin
        if (bit_pos == 4'd8)                     // ACK/NACK slot
            init_sources = first_byte ? 1'b0 : (rw_read ? 1'b1 : 1'b0);
        else
            init_sources = first_byte ? 1'b1 : (rw_read ? 1'b0 : 1'b1);
    end

    logic dir_local;
    assign dir_local = initiator ? init_sources : !init_sources;

    // ------------------------------------------------------------------
    // Relay FSM
    // ------------------------------------------------------------------
    logic stretch_armed;  // initiator: SCL fell after the tracked event
    logic bit_cap;        // locally sampled bit value
    logic rbit_q;         // remote bit value being regenerated

    // Pending local-bus events latched while the previous tx_event is
    // still waiting to be framed (pacing) - see stretch handling.
    logic pend_ship;      // S_WAIT: bit ready to ship (scl_fall seen)
    logic pend_stop;      // S_WAIT: STOP seen
    logic pend_rstart;    // S_WAIT: repeated START seen
    logic pend_srcvd;     // S_START responder: start regenerated
    logic pend_drcvd;     // S_RDATA: remote bit clocked on local bus
    logic pend_stopr;     // S_STOP responder: stop regenerated

    logic bit_done_local, bit_done_remote;
    assign bit_done_local  = (state == S_LDATA) && rx_new
                             && rx_event == I2C_EV_DATA_RCVD;
    assign bit_done_remote = (state == S_RDATA_RCVD) && rx_new
                             && rx_event == I2C_EV_DRCVD_ECHO;

    logic lose_arb;
    assign lose_arb = !ARB_PRIORITY && initiator && state == S_START
                      && rx_new && rx_event == I2C_EV_START;

    always_ff @(posedge clk) begin
        if (rst) begin
            state          <= S_IDLE;
            tx_event       <= I2C_EV_IDLE;
            stretch_armed  <= 1'b0;
            bit_cap        <= 1'b0;
            rbit_q         <= 1'b1;
            bit_pos        <= '0;
            first_byte     <= 1'b1;
            rw_read        <= 1'b0;
            initiator      <= 1'b0;
            start_deferred <= 1'b0;
            pend_ship      <= 1'b0;
            pend_stop      <= 1'b0;
            pend_rstart    <= 1'b0;
            pend_srcvd     <= 1'b0;
            pend_drcvd     <= 1'b0;
            pend_stopr     <= 1'b0;
        end else begin
            // A local START while responding - or while our own Stop is
            // completing (2.1v1.1 back-to-back transactions) - is deferred:
            // the local master sits in stretch until it can be claimed.
            // (An initiator in S_WAIT handles repeated START directly.)
            if (start_det && state != S_IDLE
                && !(initiator && state == S_WAIT))
                start_deferred <= 1'b1;

            case (state)
                // ------------------------------------------------------
                S_IDLE: begin
                    // Claim only once our Idle event has been framed
                    // (the peer's S_STOP2 exit depends on seeing it).
                    if ((start_det || start_deferred) && ok_change) begin
                        state          <= S_START;
                        initiator      <= 1'b1;
                        tx_event       <= I2C_EV_START;
                        stretch_armed  <= start_deferred;
                        start_deferred <= 1'b0;
                        bit_pos        <= '0;
                        first_byte     <= 1'b1;
                        rw_read        <= 1'b0;
                    end else if (start_det) begin
                        start_deferred <= 1'b1;       // claim as soon as ok
                    end else if (rx_new && rx_event == I2C_EV_START) begin
                        state      <= S_START;
                        initiator  <= 1'b0;
                        tx_event   <= I2C_EV_START_ECHO;
                        bit_pos    <= '0;
                        first_byte <= 1'b1;
                        rw_read    <= 1'b0;
                    end
                end
                // ------------------------------------------------------
                S_START: begin
                    if (lose_arb) begin
                        initiator      <= 1'b0;
                        start_deferred <= 1'b1;
                        tx_event       <= I2C_EV_START_ECHO;
                        stretch_armed  <= 1'b0;
                        pend_srcvd     <= 1'b0;
                    end else if (initiator) begin
                        if (scl_fall)
                            stretch_armed <= 1'b1;
                        if (rx_new && rx_event == I2C_EV_START_RCVD) begin
                            state         <= S_WAIT;
                            stretch_armed <= 1'b0;
                        end
                    end else begin
                        // Regenerated START completes on its falling edge;
                        // announce Start Received once the Echo was framed.
                        if (scl_fall || pend_srcvd) begin
                            if (ok_change) begin
                                state      <= S_WAIT;
                                tx_event   <= I2C_EV_START_RCVD;
                                pend_srcvd <= 1'b0;
                            end else
                                pend_srcvd <= 1'b1;
                        end
                    end
                end
                // ------------------------------------------------------
                S_WAIT: begin
                    if (initiator && (start_det || pend_rstart)) begin
                        if (ok_change) begin
                            state         <= S_START;
                            tx_event      <= I2C_EV_START;
                            stretch_armed <= 1'b0;
                            pend_rstart   <= 1'b0;
                            pend_ship     <= 1'b0;
                            pend_stop     <= 1'b0;
                            bit_pos       <= '0;
                            first_byte    <= 1'b1;
                            rw_read       <= 1'b0;
                        end else
                            pend_rstart <= 1'b1;
                    end else if (initiator && (stop_det || pend_stop)) begin
                        if (ok_change) begin
                            state       <= S_STOP;
                            tx_event    <= I2C_EV_STOP;
                            pend_stop   <= 1'b0;
                            pend_ship   <= 1'b0;
                            pend_rstart <= 1'b0;
                        end else
                            pend_stop <= 1'b1;
                    end else if (!initiator && rx_new
                                 && rx_event == I2C_EV_START) begin
                        state       <= S_START;
                        tx_event    <= I2C_EV_START_ECHO;
                        pend_ship   <= 1'b0;
                        pend_stop   <= 1'b0;
                        pend_rstart <= 1'b0;
                        bit_pos     <= '0;
                        first_byte  <= 1'b1;
                    end else if (!initiator && rx_new
                                 && rx_event == I2C_EV_STOP) begin
                        state       <= S_STOP;
                        tx_event    <= I2C_EV_STOP_ECHO;
                        pend_ship   <= 1'b0;
                        pend_stop   <= 1'b0;
                        pend_rstart <= 1'b0;
                    end else if (dir_local) begin
                        // Local side sources this bit: sample at SCL rise;
                        // ship at SCL fall, once the previous event (e.g.
                        // Data Received Echo) has been framed.
                        if (scl_rise)
                            bit_cap <= sda_val;
                        if (scl_fall || pend_ship) begin
                            if (ok_change) begin
                                state       <= S_LDATA;
                                tx_event    <= bit_cap ? I2C_EV_DATA1
                                                       : I2C_EV_DATA0;
                                pend_ship   <= 1'b0;
                                pend_stop   <= 1'b0;
                                pend_rstart <= 1'b0;
                            end else
                                pend_ship <= 1'b1;
                        end
                    end else if (rx_new && rx_is_data) begin
                        state       <= S_RDATA;
                        rbit_q      <= (rx_event == I2C_EV_DATA1);
                        tx_event    <= (rx_event == I2C_EV_DATA1)
                                    ? I2C_EV_DATA1_ECHO : I2C_EV_DATA0_ECHO;
                        pend_ship   <= 1'b0;
                        pend_stop   <= 1'b0;
                        pend_rstart <= 1'b0;
                    end
                end
                // ------------------------------------------------------
                S_LDATA: begin
                    if (bit_done_local) begin
                        state    <= S_WAIT;
                        tx_event <= I2C_EV_DRCVD_ECHO;
                        advance_bit();
                    end
                end
                // ------------------------------------------------------
                S_RDATA: begin
                    // Announce Data Received after the REAL local falling
                    // edge (a stretching target delays it), and only once
                    // the Data Echo has been framed.
                    if (scl_fall || pend_drcvd) begin
                        if (ok_change) begin
                            state      <= S_RDATA_RCVD;
                            tx_event   <= I2C_EV_DATA_RCVD;
                            pend_drcvd <= 1'b0;
                        end else
                            pend_drcvd <= 1'b1;
                    end
                end
                // ------------------------------------------------------
                S_RDATA_RCVD: begin
                    if (bit_done_remote) begin
                        state <= S_WAIT;
                        advance_bit();
                    end
                end
                // ------------------------------------------------------
                S_STOP: begin
                    if (initiator) begin
                        if (rx_new && rx_event == I2C_EV_STOP_RCVD) begin
                            if (start_deferred && ok_change) begin
                                // 2.1v1.1 optimization: a deferred START
                                // may follow Stop Received directly,
                                // skipping the Idle event phase.
                                state          <= S_START;
                                tx_event       <= I2C_EV_START;
                                stretch_armed  <= 1'b1;  // master already fell
                                start_deferred <= 1'b0;
                                bit_pos        <= '0;
                                first_byte     <= 1'b1;
                                rw_read        <= 1'b0;
                            end else begin
                                state     <= S_IDLE;
                                tx_event  <= I2C_EV_IDLE;
                                initiator <= 1'b0;
                            end
                        end
                    end else begin
                        if (stop_det || pend_stopr) begin
                            if (ok_change) begin
                                state      <= S_STOP2;
                                tx_event   <= I2C_EV_STOP_RCVD;
                                pend_stopr <= 1'b0;
                            end else
                                pend_stopr <= 1'b1;
                        end
                    end
                end
                // ------------------------------------------------------
                S_STOP2: begin
                    if (rx_new && rx_event == I2C_EV_START) begin
                        // 2.1v1.1: the peer may start a new transaction
                        // straight after our Stop Received - respond.
                        state      <= S_START;
                        initiator  <= 1'b0;
                        tx_event   <= I2C_EV_START_ECHO;
                        bit_pos    <= '0;
                        first_byte <= 1'b1;
                        rw_read    <= 1'b0;
                    end else if (rx_new && rx_event == I2C_EV_IDLE) begin
                        state    <= S_IDLE;
                        tx_event <= I2C_EV_IDLE;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    // Advance the 9-slot bit position; capture R/W at the end of the
    // address byte's bit 7.
    task automatic advance_bit();
        begin
            if (bit_pos == 4'd8) begin
                bit_pos    <= '0;
                first_byte <= 1'b0;
            end else begin
                if (first_byte && bit_pos == 4'd7)
                    rw_read <= (state == S_LDATA)
                               ? (tx_event == I2C_EV_DATA1)
                               : rbit_q;
                bit_pos <= bit_pos + 1'b1;
            end
        end
    endtask

    // ------------------------------------------------------------------
    // Bus control outputs (open-drain enables only)
    // ------------------------------------------------------------------
    always_comb begin
        scl_stretch = (state == S_LDATA)
                   || (state == S_RDATA_RCVD)
                   || (initiator && state == S_START && stretch_armed)
                   || (initiator && state == S_STOP)
                   || (!initiator && state == S_WAIT && !dir_local)
                   || start_deferred
                   || pend_ship || pend_drcvd || pend_srcvd;  // pacing holds

        sda_pull = (state == S_RDATA && rbit_q == 1'b0)
                || (!initiator && state == S_START);

        bus_start_gen = (!initiator && state == S_START);
        bus_stop_gen  = (!initiator && state == S_STOP);
    end

`ifdef FORMAL
    logic f_past_valid = 1'b0;
    always_ff @(posedge clk)
        f_past_valid <= 1'b1;

    initial assume (rst);

    always_comb begin
        assume (!(scl_rise && scl_fall));
        assume (!(start_det && stop_det));
        assume (rx_event <= I2C_EV_DRCVD_ECHO);
        assume (tx_sent_event <= I2C_EV_DRCVD_ECHO);
    end

    // R1: TX event is always a defined encoding.
    always_comb
        if (f_past_valid)
            assert (tx_event <= I2C_EV_DRCVD_ECHO);

    // R2: states defined; initiator-only consistency.
    always_comb
        if (f_past_valid) begin
            assert (state <= S_STOP2);
            if (state == S_STOP2) assert (!initiator);
            if (state == S_IDLE)  assert (!initiator);
        end

    // R3: synchronization + pacing safety - the local bus is held while
    // waiting for the far side or for the frame layer.
    always_comb
        if (f_past_valid) begin
            if (state == S_LDATA)      assert (scl_stretch);
            if (state == S_RDATA_RCVD) assert (scl_stretch);
            if (initiator && state == S_STOP) assert (scl_stretch);
            if (start_deferred)        assert (scl_stretch);
            if (pend_ship || pend_drcvd || pend_srcvd)
                assert (scl_stretch);
        end

    // R4: open-drain integrity.
    always_comb
        if (f_past_valid && sda_pull)
            assert ((state == S_RDATA && !rbit_q)
                    || (!initiator && state == S_START));

    // R5: data phases start only from S_WAIT.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst)
            && state == S_LDATA && $past(state) != S_LDATA)
            assert ($past(state) == S_WAIT);
        if (f_past_valid && !$past(rst)
            && state == S_RDATA && $past(state) != S_RDATA)
            assert ($past(state) == S_WAIT);
    end

    // R6: the initiator leaves S_STOP only on Stop Received.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && $past(initiator)
            && $past(state) == S_STOP && state == S_IDLE)
            assert ($past(rx_new)
                    && $past(rx_event) == I2C_EV_STOP_RCVD);
    end

    // R7: bit position bounded.
    always_comb
        if (f_past_valid)
            assert (bit_pos <= 4'd8);

    // R9: arbitration back-off.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && $past(lose_arb)) begin
            assert (!initiator);
            assert (start_deferred);
        end
    end

    // R10: Data Received only after the real local SCL falling edge
    // (possibly delayed by pacing - pend_drcvd is itself set by the edge).
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst)
            && state == S_RDATA_RCVD && $past(state) == S_RDATA)
            assert ($past(scl_fall) || $past(pend_drcvd));
        if (f_past_valid && !$past(rst)
            && pend_drcvd && !$past(pend_drcvd))
            assert ($past(scl_fall) && $past(state) == S_RDATA);
    end

    // R11: pend flags live only in their owning states.
    always_comb
        if (f_past_valid) begin
            if (pend_ship || pend_stop || pend_rstart)
                assert (state == S_WAIT);
            if (pend_srcvd) assert (state == S_START && !initiator);
            if (pend_drcvd) assert (state == S_RDATA);
            if (pend_stopr) assert (state == S_STOP && !initiator);
        end

    // R12: pacing - tx_event changes only when the previous value was
    // framed, or through rx-handshake-driven transitions whose
    // predecessor event is implied delivered (the peer answered it).
    // The load-bearing check: an event REPLACED by a purely local cause
    // was always framed first.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst)
            && tx_event != $past(tx_event)
            && $past(state) == S_WAIT && state == S_LDATA)
            assert ($past(ok_change));
    end

    // Covers.
    logic f_did_lbit = 1'b0, f_did_rbit = 1'b0, f_did_start = 1'b0;
    logic f_was_deferred = 1'b0, f_was_pend = 1'b0;
    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            if ($past(state) == S_LDATA && state == S_WAIT)
                f_did_lbit <= 1'b1;
            if ($past(state) == S_RDATA_RCVD && state == S_WAIT)
                f_did_rbit <= 1'b1;
            if ($past(state) == S_START && state == S_WAIT)
                f_did_start <= 1'b1;
            if (start_deferred)
                f_was_deferred <= 1'b1;
            if (pend_ship)
                f_was_pend <= 1'b1;
        end
    end

    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            cover (f_did_start && f_did_lbit && state == S_IDLE
                   && $past(state) != S_IDLE);
            cover (f_did_rbit);
            cover (bit_pos == 4'd8);
            cover (!first_byte && rw_read);
            cover (initiator && state == S_WAIT);
            cover (f_was_deferred && initiator && state == S_START);
            cover (f_was_pend && state == S_LDATA);   // paced ship happened
            cover (echo3 && sent_cnt == 2'd3);        // echo sent 3x (2.1v1.1)
            cover ($past(state) == S_STOP && state == S_START
                   && initiator);                     // Stop->Start fast path
        end
    end
`endif

endmodule
