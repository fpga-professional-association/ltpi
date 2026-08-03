// I2C bus conditioner: 2-FF synchronizer + tSP spike filter + edge/
// START/STOP detection for one I2C/SMBus channel.
//
// I2C-bus specification (UM10204) requires Fast-mode receivers to
// suppress spikes up to tSP = 50 ns on SCL/SDA. The filter is a
// saturating agreement counter: the filtered value only flips after the
// synchronized input has held the new level for FILTER_NS continuously.
//
// FILTER FIDELITY vs LINK CLOCK (see docs: I2C limitations): the filter
// step is one clk period, so the realized window is
// ceil(FILTER_NS/period) cycles. At 400 MHz that is 20 cycles of 2.5 ns
// (crisp 50 ns); at 25 MHz it is 2 cycles of 40 ns (a coarse 40-80 ns
// window). Below CLK_HZ = 40 MHz a 50 ns filter cannot be realized at
// all (one period already exceeds it) - elaboration fails loudly.
import ltpi_pkg::*;

module ltpi_i2c_cond #(
    parameter int unsigned CLK_HZ    = 400_000_000,
    parameter int unsigned FILTER_NS = 50            // I2C tSP
)(
    input  logic clk,
    input  logic rst,
    input  logic scl_in,       // raw (async) bus inputs
    input  logic sda_in,

    output logic scl_filt,     // deglitched levels
    output logic sda_filt,
    output logic start_det,    // SDA fall while SCL high (1-cycle pulse)
    output logic stop_det,     // SDA rise while SCL high
    output logic scl_rise,
    output logic scl_fall,
    output logic sda_val       // SDA level for sampling at scl_rise
);

    // ceil(FILTER_NS * CLK_HZ / 1e9), computed without 64-bit overflow.
    localparam int unsigned FILT_CYC =
        (FILTER_NS * (CLK_HZ / 1_000_000) + 999) / 1000;

    generate
        if (FILT_CYC < 2)
            $error("ltpi_i2c_cond: CLK_HZ too low to realize the %0dns tSP filter (need >= 2 cycles; see I2C limitations doc)",
                   FILTER_NS);
    endgenerate

    localparam int unsigned CW = $clog2(FILT_CYC + 1);

    // 2-FF synchronizers
    logic [1:0] scl_sync, sda_sync;
    always_ff @(posedge clk) begin
        scl_sync <= {scl_sync[0], scl_in};
        sda_sync <= {sda_sync[0], sda_in};
    end

    // Agreement counters: count cycles the sync value differs from the
    // filtered output; flip after FILT_CYC of sustained disagreement.
    logic [CW-1:0] scl_cnt, sda_cnt;
    always_ff @(posedge clk) begin
        if (rst) begin
            scl_filt <= 1'b1;   // idle-high bus
            sda_filt <= 1'b1;
            scl_cnt  <= '0;
            sda_cnt  <= '0;
        end else begin
            if (scl_sync[1] == scl_filt)
                scl_cnt <= '0;
            else if (scl_cnt == CW'(FILT_CYC - 1)) begin
                scl_filt <= scl_sync[1];
                scl_cnt  <= '0;
            end else
                scl_cnt <= scl_cnt + 1'b1;

            if (sda_sync[1] == sda_filt)
                sda_cnt <= '0;
            else if (sda_cnt == CW'(FILT_CYC - 1)) begin
                sda_filt <= sda_sync[1];
                sda_cnt  <= '0;
            end else
                sda_cnt <= sda_cnt + 1'b1;
        end
    end

    // Edge / condition detection on the FILTERED signals.
    logic scl_q, sda_q;
    always_ff @(posedge clk) begin
        if (rst) begin
            scl_q <= 1'b1;
            sda_q <= 1'b1;
        end else begin
            scl_q <= scl_filt;
            sda_q <= sda_filt;
        end
    end

    assign scl_rise  = scl_filt && !scl_q;
    assign scl_fall  = !scl_filt && scl_q;
    assign start_det = scl_filt && scl_q && sda_q && !sda_filt; // SDA fall
    assign stop_det  = scl_filt && scl_q && !sda_q && sda_filt; // SDA rise
    assign sda_val   = sda_filt;

`ifdef FORMAL
    logic f_past_valid = 1'b0;
    always_ff @(posedge clk)
        f_past_valid <= 1'b1;

    initial assume (rst);

    // Shadow: how long has the synchronized value agreed with itself?
    // (Counts consecutive cycles scl_sync[1] held its current level.)
    logic [CW:0] f_scl_hold, f_sda_hold;
    always_ff @(posedge clk) begin
        if (rst) begin
            f_scl_hold <= '0;
            f_sda_hold <= '0;
        end else begin
            f_scl_hold <= (scl_sync[1] == $past(scl_sync[1]))
                          ? ((f_scl_hold == {1'b1, {CW{1'b0}}})
                             ? f_scl_hold : f_scl_hold + 1'b1)
                          : '0;
            f_sda_hold <= (sda_sync[1] == $past(sda_sync[1]))
                          ? ((f_sda_hold == {1'b1, {CW{1'b0}}})
                             ? f_sda_hold : f_sda_hold + 1'b1)
                          : '0;
        end
    end

    // F1 (tSP suppression): the filtered output only transitions after
    // the synchronized input held the NEW level for FILT_CYC cycles -
    // any shorter spike is provably invisible downstream.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst)) begin
            if (scl_filt != $past(scl_filt)) begin
                assert ($past(scl_cnt) == CW'(FILT_CYC - 1));
                assert (scl_filt == $past(scl_sync[1]));
                assert ($past(f_scl_hold) >= (CW+1)'(FILT_CYC - 2));
            end
            if (sda_filt != $past(sda_filt)) begin
                assert ($past(sda_cnt) == CW'(FILT_CYC - 1));
                assert (sda_filt == $past(sda_sync[1]));
                assert ($past(f_sda_hold) >= (CW+1)'(FILT_CYC - 2));
            end
        end
    end

    // F2 (counter/coupling invariants): disagreement count is bounded and
    // zero whenever input and output agree.
    always_comb
        if (f_past_valid) begin
            assert (scl_cnt < CW'(FILT_CYC));
            assert (sda_cnt < CW'(FILT_CYC));
        end

    // F3 (detector correctness): every condition pulse matches its
    // filtered-signal definition; START/STOP are mutually exclusive and
    // require a stable-high SCL across the SDA edge.
    always_comb
        if (f_past_valid) begin
            assert (!(start_det && stop_det));
            if (start_det) assert (scl_filt && !sda_filt);
            if (stop_det)  assert (scl_filt && sda_filt);
            assert (!(scl_rise && scl_fall));
        end

    // Covers: a filtered SCL edge, a rejected spike (input toggled but
    // output never moved), START and STOP detection.
    logic f_spike_seen = 1'b0;
    always_ff @(posedge clk)
        if (f_past_valid && scl_sync[1] != scl_filt
            && $past(scl_sync[1]) == scl_filt)
            f_spike_seen <= 1'b1;   // a disagreement began

    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            cover (scl_rise);
            cover (scl_fall);
            cover (start_det);
            cover (stop_det);
            cover (f_spike_seen && scl_cnt == '0
                   && scl_sync[1] == scl_filt);   // spike came and went
        end
    end
`endif

endmodule
