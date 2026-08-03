// LTPI UART channel: one UART direction lane (spec 2.2.1.2, Table 8).
//
// The line level is oversampled 3x per LTPI frame; the three samples travel
// in a 4-bit field per Table 8: {flow_ctrl, D[2], D[1], D[0]} with D[0] the
// OLDEST sample ("first samples are stored in TXD[0] and last sample per
// frame is stored in TXD[2]"). Flow control (RTS/CTS) is sampled once per
// frame like a Low Latency GPIO.
//
// The receive side replays the three samples of the last CRC-good frame in
// order, at the same 3x pacing, regenerating the line with bounded jitter.
// On CRC error the regenerated line holds its level (frame dropped,
// spec Table 14).
//
// Two instances of this lane (UART0/UART1) form I/O-frame byte 7; the TX
// direction uses one lane per side, the flow-control bit carries RTS on the
// SCM->HPM path and CTS on the HPM->SCM path.
import ltpi_pkg::*;

module ltpi_uart_channel (
    input  logic clk,
    input  logic rst,

    // ------------- TX capture -------------
    input  logic tx_sample,        // 3 pulses per frame, evenly spaced
    input  logic txd_in,           // local UART line to tunnel
    input  logic flow_in,          // RTS/CTS, sampled at frame start
    input  logic tx_frame_start,   // latches the collected samples
    output logic [3:0] tx_field,   // {flow, D[2], D[1], D[0]}

    // ------------- RX replay -------------
    input  logic rx_frame_valid,
    input  logic rx_crc_ok,
    input  logic [3:0] rx_field,
    input  logic rx_replay,        // 3 pulses per frame, evenly spaced
    output logic txd_out,          // regenerated UART line
    output logic flow_out
);

    logic [2:0] samp;        // collection buffer, samp[i] = i-th sample
    logic [1:0] idx;         // next sample slot, saturates at 2

    logic [2:0] rsamp;       // replay buffer from last good frame
    logic [1:0] ridx;        // next replay slot, saturates at 2

    logic rx_good;
    assign rx_good = rx_frame_valid && rx_crc_ok;

    // ------------------------------------------------------------------
    // TX: collect 3 samples, hand them over at the frame boundary.
    // ------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            idx      <= '0;
            samp     <= '0;
            tx_field <= '0;
        end else if (tx_frame_start) begin
            tx_field <= {flow_in, samp[2], samp[1], samp[0]};
            idx      <= '0;
        end else if (tx_sample) begin
            samp[idx] <= txd_in;
            if (idx != 2'd2)
                idx <= idx + 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // RX: latch a good frame's samples, replay them in order.
    // ------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            rsamp    <= 3'b111;    // UART idle level
            ridx     <= 2'd2;
            txd_out  <= 1'b1;
            flow_out <= 1'b1;
        end else begin
            if (rx_good) begin
                rsamp    <= rx_field[2:0];
                flow_out <= rx_field[3];
                ridx     <= '0;
            end else if (rx_replay) begin
                txd_out <= rsamp[ridx];
                if (ridx != 2'd2)
                    ridx <= ridx + 1'b1;
            end
            // CRC error: nothing updates - the line holds its level.
        end
    end

`ifdef FORMAL
    logic f_past_valid = 1'b0;
    always_ff @(posedge clk)
        f_past_valid <= 1'b1;

    initial assume (rst);

    // Sample/latch strobes and frame events don't collide (frame timing
    // generator guarantees this: samples land on bytes 10/0/7, the frame
    // boundary on byte 15).
    always_comb begin
        assume (!(tx_frame_start && tx_sample));
        assume (!(rx_good && rx_replay));
    end

    // U1: the emitted field preserves capture order: D[0] oldest.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && $past(tx_frame_start))
            assert (tx_field == {$past(flow_in), $past(samp[2]),
                                 $past(samp[1]), $past(samp[0])});
    end

    // U2: replay reproduces the stored samples in order.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && $past(rx_replay) && !$past(rx_good))
            assert (txd_out == $past(rsamp[ridx]));
    end

    // U3: hold-on-error - the regenerated line only changes on a replay
    // strobe, and flow control only on a good frame.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst)) begin
            if (!$past(rx_replay))
                assert (txd_out == $past(txd_out));
            if (!$past(rx_good))
                assert (flow_out == $past(flow_out));
        end
    end

    // U4: slot indices stay in range.
    always_comb
        if (f_past_valid) begin
            assert (idx <= 2'd2);
            assert (ridx <= 2'd2);
        end

    // Covers: a 0-1-0 sample pattern is captured and then replayed in order
    // (proves the tunnel can carry a real UART edge sequence).
    logic [1:0] f_replayed;
    always_ff @(posedge clk) begin
        if (rst || rx_good)
            f_replayed <= '0;
        else if (rx_replay && f_replayed != 2'd3)
            f_replayed <= f_replayed + 1'b1;
    end

    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            cover ($past(tx_frame_start) && tx_field[2:0] == 3'b010);
            cover (f_replayed == 2'd3 && rsamp == 3'b010);
        end
    end
`endif

endmodule
