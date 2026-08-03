// LTPI GPIO channel: Low Latency + Normal Latency GPIO tunneling
// (spec 2.2.1.1, Tables 6 and 33).
//
// TX side: presents the GPIO fields of the Default I/O Frame - the frame
// counter, 16 LL GPIO bits refreshed every frame, and a 16-bit slice of the
// NL GPIO vector selected by (frame counter mod N), N = ceil(NL_TOTAL/16).
//
// RX side: consumes the GPIO fields of each received I/O frame. On a CRC-good
// frame the LL outputs and the addressed NL slice are updated; on a CRC error
// every output holds its previous state (spec Table 14: "state of GPIOs is
// maintained from the previous Frame").
import ltpi_pkg::*;

module ltpi_gpio_channel #(
    parameter int unsigned NL_TOTAL = 96   // total tunneled NL GPIOs
)(
    input  logic        clk,
    input  logic        rst,

    // ------------- TX (sample & encode) -------------
    input  logic [15:0] ll_in,
    input  logic [NL_TOTAL-1:0] nl_in,
    input  logic        tx_frame_start,   // pulse: an I/O frame begins
    output logic [7:0]  tx_frame_counter, // I/O frame byte 2
    output logic [15:0] tx_ll,            // I/O frame bytes 3..4
    output logic [15:0] tx_nl,            // I/O frame bytes 5..6

    // ------------- RX (decode & drive) -------------
    input  logic        rx_frame_valid,   // pulse: an I/O frame arrived
    input  logic        rx_crc_ok,
    input  logic [7:0]  rx_frame_counter,
    input  logic [15:0] rx_ll,
    input  logic [15:0] rx_nl,
    output logic [15:0] ll_out,
    output logic [NL_TOTAL-1:0] nl_out
);

    // Number of frames to cover all NL GPIOs (spec 2.2.1.1).
    localparam int unsigned NL_FRAMES = (NL_TOTAL + 15) / 16;

    // ------------------------------------------------------------------
    // TX path
    // ------------------------------------------------------------------
    logic [7:0] slice_q;    // tx_frame_counter mod NL_FRAMES, tracked directly

    always_ff @(posedge clk) begin
        if (rst) begin
            tx_frame_counter <= '0;
            slice_q          <= '0;
        end else if (tx_frame_start) begin
            tx_frame_counter <= tx_frame_counter + 1'b1;
            // Receiver computes (counter mod N); mirror the same sequence,
            // including the discontinuity at the 8-bit counter wrap.
            slice_q <= (tx_frame_counter + 1'b1 == 8'd0)
                       ? 8'd0
                       : 8'((9'(tx_frame_counter) + 9'd1) % NL_FRAMES);
        end
    end

    assign tx_ll = ll_in;

    // NL slice mux; bits beyond NL_TOTAL in the last slice read as 0.
    always_comb begin : tx_slice_mux
        integer i;
        tx_nl = '0;
        for (i = 0; i < 16; i = i + 1)
            if (32'(slice_q) * 16 + i < NL_TOTAL)
                tx_nl[i] = nl_in[32'(slice_q) * 16 + i];
    end

    // ------------------------------------------------------------------
    // RX path
    // ------------------------------------------------------------------
    logic [7:0] rx_slice;
    assign rx_slice = 8'(rx_frame_counter % NL_FRAMES);

    logic rx_good;
    assign rx_good = rx_frame_valid && rx_crc_ok;

    // Constant LHS indices (i/16, i%16 fold at elaboration) - equivalent to
    // writing the addressed 16-bit slice; simulator-portable, unlike a
    // dynamically-indexed NBA target.
    always_ff @(posedge clk) begin : rx_update
        integer i;
        if (rst) begin
            ll_out <= '0;
            nl_out <= '0;
        end else if (rx_good) begin
            ll_out <= rx_ll;
            for (i = 0; i < NL_TOTAL; i = i + 1)
                if ((i / 16) == 32'(rx_slice))
                    nl_out[i] <= rx_nl[i % 16];
        end
        // else: hold everything (CRC error / no frame), per Table 14
    end

`ifdef FORMAL
    logic f_past_valid = 1'b0;
    always_ff @(posedge clk)
        f_past_valid <= 1'b1;

    initial assume (rst);

    // G1: hold-on-error - without a CRC-good frame nothing moves.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && !$past(rx_good)) begin
            assert (ll_out == $past(ll_out));
            assert (nl_out == $past(nl_out));
        end
    end

    // G2: LL GPIOs always reflect the last good frame in full.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && $past(rx_good))
            assert (ll_out == $past(rx_ll));
    end

    // G3: a good frame updates exactly the addressed NL slice; every bit
    // outside it is untouched (no cross-slice corruption).
    always_ff @(posedge clk) begin : g3
        integer i;
        if (f_past_valid && !$past(rst) && $past(rx_good)) begin
            for (i = 0; i < NL_TOTAL; i = i + 1) begin
                if (32'($past(rx_slice)) == i / 16)
                    assert (nl_out[i] == $past(rx_nl[i % 16]));
                else
                    assert (nl_out[i] == $past(nl_out[i]));
            end
        end
    end

    // G4: the TX slice select always addresses a real slice.
    always_comb
        if (f_past_valid)
            assert (slice_q < 8'(NL_FRAMES));

    // G5: TX slice content matches the input vector at the addressed slice.
    always_comb begin : g5
        integer i;
        for (i = 0; i < 16; i = i + 1) begin
            if (f_past_valid) begin
                if (32'(slice_q) * 16 + i < NL_TOTAL)
                    assert (tx_nl[i] == nl_in[32'(slice_q) * 16 + i]);
                else
                    assert (tx_nl[i] == 1'b0);
            end
        end
    end

    // Covers: two different NL slices get written; counter wraps.
    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            cover ($past(rx_good) && $past(rx_slice) == 8'd0);
            cover ($past(rx_good) && $past(rx_slice) == 8'(NL_FRAMES - 1));
            cover (tx_frame_counter == 8'hFF && tx_frame_start);
        end
    end
`endif

endmodule
