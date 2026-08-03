// Streaming CRC-8 generator/checker for LTPI frames.
// Poly x^8 + x^2 + x + 1, initial value 0 (spec section 2.4). The CRC covers
// the payload bytes after the comma symbol; a receiver that also feeds the
// received CRC byte through ends at 0 exactly when the frame is intact.
import ltpi_pkg::*;

module ltpi_crc8 (
    input  logic       clk,
    input  logic       rst,
    input  logic       clear,   // start of a new frame
    input  logic       en,      // consume one payload byte
    input  logic [7:0] data,
    output logic [7:0] crc
);

`include "ltpi_crc8_func.svh"

    always_ff @(posedge clk) begin
        if (rst || clear)
            crc <= 8'h00;
        else if (en)
            crc <= crc8_update(crc, data);
    end

`ifdef FORMAL
    logic f_past_valid = 1'b0;
    always_ff @(posedge clk)
        f_past_valid <= 1'b1;

    initial assume (rst);

    (* anyseq *) logic [7:0] f_any;

    // Zero-remainder theorem: for ANY running CRC value, absorbing a byte
    // equal to the CRC itself yields 0. This is what makes the receiver's
    // "residue == 0" check equivalent to "stored CRC == computed CRC".
    always_comb
        assert (crc8_update(f_any, f_any) == 8'h00);

    // The CRC of a single zero byte from init state is 0 (poly has no init
    // XOR), per spec: "The polynomial initial value is defined as '0'".
    always_comb
        assert (crc8_update(8'h00, 8'h00) == 8'h00);

    // Known-answer check baked in as a proof: CRC8/0x07 of "123456789" is
    // 0xF4 (standard CRC-8 check value).
    always_comb begin : f_known_answer
        logic [7:0] c;
        c = 8'h00;
        c = crc8_update(c, "1"); c = crc8_update(c, "2"); c = crc8_update(c, "3");
        c = crc8_update(c, "4"); c = crc8_update(c, "5"); c = crc8_update(c, "6");
        c = crc8_update(c, "7"); c = crc8_update(c, "8"); c = crc8_update(c, "9");
        assert (c == 8'hF4);
    end

    // Register behavior: clear dominates, and crc only changes on en.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst)) begin
            if ($past(clear))
                assert (crc == 8'h00);
            else if (!$past(en))
                assert (crc == $past(crc));
        end
    end
`endif

endmodule
