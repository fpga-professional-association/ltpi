// LTPI PHY layer: vendor-portable byte (de)serializers for the LVDS pair.
//
// SDR mode: 1 bit per clk cycle on `ser_sdr`   (8 cycles per byte)
//           - 200 Mbps @ 200MHz, 400 Mbps @ 400MHz link clock
// DDR mode: 2 bits per clk cycle on `ser_ddr`  (4 cycles per byte)
//           - 800 Mbps @ 400MHz link clock
//           ser_ddr[0] launches on the rising edge, ser_ddr[1] on the
//           falling edge of the vendor DDR output cell.
//
// These modules are pure generic logic; the I/O cells live in vendor
// wrappers (see vendor/):
//   Altera/Intel : ALTDDIO_OUT / ALTDDIO_IN (or GPIO Lite IP), true LVDS
//                  pins, IOPLL for the 200/400MHz link clock.
//   Lattice      : ODDRX1F / IDDRX1F (+ DELAYG), LVDS25 I/O, EHXPLLL.
//
// Bit order: LSB first. The receiver hunts for the K28.5 comma byte at any
// bit offset - including odd DDR offsets (built-in bitslip via dual-phase
// windows). LTPI training sends continuous comma-led frames during Link
// Detect, so alignment converges during training (spec 4.1.1.1).
import ltpi_pkg::*;

module ltpi_phy_tx (
    input  logic       clk,
    input  logic       rst,
    input  logic       ddr_mode,
    input  logic [7:0] byte_data,   // next byte to serialize
    output logic       byte_req,    // pulse: byte_data consumed, supply next
    output logic       ser_sdr,     // SDR serial bit
    output logic [1:0] ser_ddr      // DDR bit pair (0 = first on wire)
);

    logic [7:0] shift;
    logic [2:0] bit_idx;

    always_ff @(posedge clk) begin
        if (rst) begin
            bit_idx  <= '0;
            shift    <= '0;
            byte_req <= 1'b0;
        end else begin
            byte_req <= 1'b0;
            if (bit_idx == 3'd0) begin
                shift    <= byte_data;
                byte_req <= 1'b1;
            end else begin
                shift <= ddr_mode ? (shift >> 2) : (shift >> 1);
            end
            if (ddr_mode)
                bit_idx <= (bit_idx == 3'd6) ? 3'd0 : bit_idx + 3'd2;
            else
                bit_idx <= (bit_idx == 3'd7) ? 3'd0 : bit_idx + 3'd1;
        end
    end

    assign ser_sdr = shift[0];
    assign ser_ddr = shift[1:0];

`ifdef FORMAL
    logic f_past_valid = 1'b0;
    always_ff @(posedge clk) f_past_valid <= 1'b1;
    initial assume (rst);

    always_ff @(posedge clk)
        if (f_past_valid) assume (ddr_mode == $past(ddr_mode));

    // Shadow of the byte accepted at the last load.
    logic [7:0] f_byte;
    always_ff @(posedge clk)
        if (rst)
            f_byte <= '0;
        else if (bit_idx == 3'd0 && !rst)
            f_byte <= byte_data;

    // T1: counters stay in range / on parity.
    always_comb
        if (f_past_valid && !rst) begin
            assert (bit_idx <= 3'd7);
            if (ddr_mode) assert (bit_idx[0] == 1'b0);
        end

    // T2: the shift register holds exactly the not-yet-transmitted suffix
    // of the accepted byte - serialization is in-order and lossless.
    always_comb begin : t2
        integer i;
        for (i = 0; i < 8; i = i + 1) begin
            if (f_past_valid && !rst && byte_req) begin
                // Cycle after load: entire byte still pending.
                assert (shift[i] == f_byte[i]);
            end
        end
    end
    always_ff @(posedge clk) begin : t2b
        integer i;
        if (f_past_valid && !$past(rst) && !byte_req && !rst
            && bit_idx != 3'd0) begin
            // Mid-byte: current shift is the previous shift consumed by
            // the per-cycle step (1 or 2 bits).
            for (i = 0; i < 8; i = i + 1) begin
                if (!ddr_mode && i < 7)
                    assert (shift[i] == $past(shift[i + 1]));
                if (ddr_mode && i < 6)
                    assert (shift[i] == $past(shift[i + 2]));
            end
        end
    end

    always_ff @(posedge clk)
        if (f_past_valid) cover (byte_req && f_byte == 8'hBC);
`endif

endmodule

module ltpi_phy_rx (
    input  logic       clk,
    input  logic       rst,
    input  logic       ddr_mode,
    input  logic       hunt,        // allowed to (re)acquire alignment
    input  logic       realign,     // drop alignment and re-hunt (e.g. on
                                    // persistent CRC failure = false lock)
    input  logic       ser_sdr,
    input  logic [1:0] ser_ddr,
    output logic       aligned,
    output logic       phase_odd,   // DDR bitslip: boundary on odd offset
    output logic       byte_valid,
    output logic [7:0] byte_data
);

    // 9-bit history of the raw wire bit stream (LSB-first arrival), newest
    // bit at [8]. Holding one spare bit lets us evaluate both DDR phases.
    logic [8:0] s;
    logic [8:0] s_next;
    always_comb begin
        if (ddr_mode)
            s_next = {ser_ddr[1], ser_ddr[0], s[8:2]};
        else
            s_next = {ser_sdr, s[8:1]};
    end

    // Byte windows for the two possible boundaries.
    logic hit_even, hit_odd;
    assign hit_even = (s_next[8:1] == COMMA_LINK);
    assign hit_odd  = ddr_mode && (s_next[7:0] == COMMA_LINK);

    logic [3:0] cnt;   // wire bits consumed of the current byte
    logic [1:0] step;
    assign step = ddr_mode ? 2'd2 : 2'd1;

    always_ff @(posedge clk) begin
        if (rst) begin
            s          <= '0;
            cnt        <= '0;
            aligned    <= 1'b0;
            phase_odd  <= 1'b0;
            byte_valid <= 1'b0;
        end else if (realign) begin
            s          <= s_next;
            aligned    <= 1'b0;
            byte_valid <= 1'b0;
        end else begin
            s          <= s_next;
            byte_valid <= 1'b0;
            if (!aligned) begin
                if (hunt && hit_even) begin
                    aligned    <= 1'b1;
                    phase_odd  <= 1'b0;
                    cnt        <= '0;
                    byte_valid <= 1'b1;
                    byte_data  <= s_next[8:1];
                end else if (hunt && hit_odd) begin
                    aligned    <= 1'b1;
                    phase_odd  <= 1'b1;
                    cnt        <= 4'd1;      // 1 bit of next byte consumed
                    byte_valid <= 1'b1;
                    byte_data  <= s_next[7:0];
                end
            end else begin
                if (cnt + 4'(step) >= 4'd8) begin
                    byte_valid <= 1'b1;
                    byte_data  <= (cnt + 4'(step) == 4'd9)
                                  ? s_next[7:0] : s_next[8:1];
                    cnt        <= cnt + 4'(step) - 4'd8;
                end else begin
                    cnt <= cnt + 4'(step);
                end
            end
        end
    end

`ifdef FORMAL
    logic f_past_valid = 1'b0;
    always_ff @(posedge clk) f_past_valid <= 1'b1;
    initial assume (rst);

    always_ff @(posedge clk)
        if (f_past_valid) assume (ddr_mode == $past(ddr_mode));

    // R1: bytes only while aligned; data equals the exact wire window of
    // the boundary phase in force.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && byte_valid) begin
            assert (aligned);
            assert (byte_data == (phase_odd ? $past(s_next[7:0])
                                            : $past(s_next[8:1])));
        end
    end

    // R2: the first byte after acquiring alignment is the comma.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && !$past(realign)
            && aligned && !$past(aligned) && byte_valid)
            assert (byte_data == COMMA_LINK);
    end

    // R4: realign always drops alignment on the next cycle.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && $past(realign))
            assert (!aligned);
    end

    // R3: counter bounds; phase_odd only exists in DDR mode; SDR consumes
    // whole bytes with no leftover.
    always_comb
        if (f_past_valid && !rst) begin
            assert (cnt <= 4'd7);
            if (!ddr_mode) assert (!phase_odd);
            if (aligned && !ddr_mode) assert (cnt <= 4'd7);
            if (aligned && ddr_mode)  assert (cnt[0] == phase_odd);
        end

    always_ff @(posedge clk)
        if (f_past_valid) begin
            cover (aligned && byte_valid && !ddr_mode);
            cover (aligned && byte_valid && ddr_mode && !phase_odd);
            cover (aligned && byte_valid && ddr_mode && phase_odd);
        end
`endif

endmodule
