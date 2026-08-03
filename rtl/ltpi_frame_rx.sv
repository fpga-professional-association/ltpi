// LTPI frame receiver: assembles 16-byte frames from the (already aligned)
// SERDES byte stream, verifies CRC-8, and classifies the frame type from the
// comma symbol + subtype (spec Tables 19, 20, 25, 32).
//
// Frame layout (spec Table 22 et al.):
//   byte 0      comma symbol
//   byte 1      frame subtype
//   bytes 2-14  payload (CRC-covered together with byte 1)
//   byte 15     CRC-8 over bytes 1..14
import ltpi_pkg::*;

module ltpi_frame_rx (
    input  logic        clk,
    input  logic        rst,
    input  logic        aligned,     // SERDES comma alignment achieved
    input  logic        byte_valid,
    input  logic [7:0]  byte_data,
    output logic        frame_valid,   // 1-cycle pulse after 16th byte
    output logic        frame_crc_ok,
    output ltpi_pkg::frame_type_t frame_type,
    output logic [15:0] speed_payload, // frame bytes 3..4 (speed caps/select)
    output logic [103:0] payload       // frame bytes 2..14, byte2 = [7:0]
);

`include "ltpi_crc8_func.svh"

    logic [3:0]  byte_cnt;
    logic [7:0]  crc;
    logic [7:0]  comma_r;
    ltpi_pkg::frame_type_t cur_type;

    function automatic ltpi_pkg::frame_type_t classify(input logic [7:0] comma,
                                             input logic [7:0] subtype);
        begin
            classify = FRAME_INVALID;
            case (comma)
                COMMA_LINK:
                    case (subtype)
                        SUB_LINK_DETECT: classify = FRAME_LINK_DETECT;
                        SUB_LINK_SPEED:  classify = FRAME_LINK_SPEED;
                        default:         classify = FRAME_INVALID;
                    endcase
                COMMA_CFG:
                    case (subtype)
                        SUB_ADVERTISE: classify = FRAME_ADVERTISE;
                        SUB_CONFIGURE: classify = FRAME_CONFIGURE;
                        SUB_ACCEPT:    classify = FRAME_ACCEPT;
                        default:       classify = FRAME_INVALID;
                    endcase
                COMMA_OP:
                    case (subtype)
                        SUB_OP_IO:   classify = FRAME_OPERATIONAL;
                        SUB_OP_DATA: classify = FRAME_OP_DATA;
                        default:     classify = FRAME_INVALID;
                    endcase
                default:
                    classify = FRAME_INVALID;
            endcase
        end
    endfunction

    always_ff @(posedge clk) begin
        frame_valid <= 1'b0;
        if (rst || !aligned) begin
            byte_cnt    <= '0;
            crc         <= '0;
            frame_valid <= 1'b0;
        end else if (byte_valid) begin
            byte_cnt <= byte_cnt + 1'b1;   // 4-bit counter wraps 15 -> 0
            if (byte_cnt == 4'd0) begin
                comma_r <= byte_data;
                crc     <= 8'h00;
            end else if (byte_cnt < 4'd15) begin
                crc <= crc8_update(crc, byte_data);
            end
            if (byte_cnt == 4'd1)
                cur_type <= classify(comma_r, byte_data);
            if (byte_cnt == 4'd3) speed_payload[7:0]  <= byte_data;
            if (byte_cnt == 4'd4) speed_payload[15:8] <= byte_data;
            if (byte_cnt >= 4'd2 && byte_cnt <= 4'd14)
                payload[8*(byte_cnt - 4'd2) +: 8] <= byte_data;
            if (byte_cnt == 4'd15) begin
                frame_valid  <= 1'b1;
                frame_crc_ok <= (crc == byte_data);
                frame_type   <= cur_type;
            end
        end
    end

`ifdef FORMAL
    logic f_past_valid = 1'b0;
    always_ff @(posedge clk)
        f_past_valid <= 1'b1;

    initial assume (rst);

    // frame_valid pulses only as the direct consequence of the 16th byte.
    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            if (frame_valid)
                assert ($past(!rst && aligned && byte_valid && byte_cnt == 4'd15));
            // Never two pulses back to back: a new frame needs 16 bytes.
            if ($past(frame_valid))
                assert (!frame_valid);
        end
    end

    // Losing alignment resets the framer.
    always_ff @(posedge clk) begin
        if (f_past_valid && $past(!aligned || rst))
            assert (byte_cnt == 4'd0 && !frame_valid);
    end

    // A reported-good frame really has residue-matching CRC: frame_crc_ok
    // was compared against the CRC accumulated over bytes 1..14 only.
    // (The accumulator restarts at every byte 0, so cross-frame leakage is
    // impossible; this is enforced by the two assertions below.)
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && $past(aligned && byte_valid)
            && $past(byte_cnt) == 4'd0)
            assert (crc == 8'h00);
    end

    // Covers: receive a CRC-good Link Detect frame, and a CRC-bad frame.
    // Guarded by f_past_valid so the solver cannot satisfy them with the
    // unconstrained time-zero register state - the trace must contain a
    // genuine 16-byte frame after reset.
    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            cover (frame_valid && frame_crc_ok
                   && frame_type == FRAME_LINK_DETECT);
            cover (frame_valid && !frame_crc_ok);
            cover (frame_valid && frame_crc_ok
                   && frame_type == FRAME_OPERATIONAL);
        end
    end
`endif

endmodule

