// LTPI frame transmitter: streams back-to-back 16-byte frames.
// Byte 0 comma + byte 1 subtype are derived from the FSM's tx_frame_type;
// bytes 2..14 come from the payload input (channel mux / training words);
// byte 15 is the CRC-8 over bytes 1..14 (spec section 2.4, Table 22 et al.).
//
// The payload is sampled at the start of each frame so a frame is always
// internally consistent even if inputs change mid-frame.
import ltpi_pkg::*;

module ltpi_frame_tx (
    input  logic         clk,
    input  logic         rst,
    input  logic         en,            // stream enable (PHY ready)
    input  ltpi_pkg::frame_type_t frame_type,
    input  logic [103:0] payload,       // bytes 2..14, byte2 = [7:0]
    output logic         byte_valid,
    output logic [7:0]   byte_data,
    output logic         frame_done     // pulses with the CRC byte
);

`include "ltpi_crc8_func.svh"

    logic [3:0]   byte_cnt;
    logic [7:0]   crc;
    logic [103:0] payload_r;
    logic [7:0]   comma_r, subtype_r;

    function automatic logic [7:0] comma_of(input ltpi_pkg::frame_type_t t);
        begin
            case (t)
                FRAME_LINK_DETECT, FRAME_LINK_SPEED:            comma_of = COMMA_LINK;
                FRAME_ADVERTISE, FRAME_CONFIGURE, FRAME_ACCEPT: comma_of = COMMA_CFG;
                default:                                        comma_of = COMMA_OP;
            endcase
        end
    endfunction

    function automatic logic [7:0] subtype_of(input ltpi_pkg::frame_type_t t);
        begin
            case (t)
                FRAME_LINK_DETECT: subtype_of = SUB_LINK_DETECT;
                FRAME_LINK_SPEED:  subtype_of = SUB_LINK_SPEED;
                FRAME_ADVERTISE:   subtype_of = SUB_ADVERTISE;
                FRAME_CONFIGURE:   subtype_of = SUB_CONFIGURE;
                FRAME_ACCEPT:      subtype_of = SUB_ACCEPT;
                FRAME_OP_DATA:     subtype_of = SUB_OP_DATA;
                default:           subtype_of = SUB_OP_IO;
            endcase
        end
    endfunction

    // Current byte selection (combinational, from registered frame content).
    logic [7:0] cur_byte;
    always_comb begin
        case (byte_cnt)
            4'd0:    cur_byte = comma_r;
            4'd1:    cur_byte = subtype_r;
            4'd15:   cur_byte = crc;
            default: cur_byte = payload_r[8*(byte_cnt - 4'd2) +: 8];
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            byte_cnt   <= '0;
            crc        <= '0;
            byte_valid <= 1'b0;
            frame_done <= 1'b0;
            // Latch initial frame content so byte 0 is consistent.
            comma_r    <= comma_of(frame_type);
            subtype_r  <= subtype_of(frame_type);
            payload_r  <= payload;
        end else begin
            byte_valid <= 1'b0;
            frame_done <= 1'b0;
            if (en) begin
                byte_valid <= 1'b1;
                byte_data  <= cur_byte;
                byte_cnt   <= byte_cnt + 1'b1;   // wraps 15 -> 0
                case (byte_cnt)
                    // CRC restarts as the comma goes out, then absorbs each
                    // of bytes 1..14 in the cycle it is emitted, so it is
                    // complete exactly when byte 15 is selected.
                    4'd0:  crc <= 8'h00;
                    4'd15: begin
                        frame_done <= 1'b1;
                        // Re-latch content for the next frame.
                        comma_r   <= comma_of(frame_type);
                        subtype_r <= subtype_of(frame_type);
                        payload_r <= payload;
                    end
                    default:
                        crc <= crc8_update(crc, cur_byte);
                endcase
            end
        end
    end

`ifdef FORMAL
    logic f_past_valid = 1'b0;
    always_ff @(posedge clk)
        f_past_valid <= 1'b1;

    initial assume (rst);

    // Reference model: recompute the CRC over the actually emitted bytes
    // 1..14 and check the emitted byte 15 equals it. This is the exact
    // acceptance condition of ltpi_frame_rx, so it proves TX->RX
    // compatibility for every frame under all input sequences.
    logic [7:0] f_crc_ref;
    always_ff @(posedge clk) begin
        if (rst)
            f_crc_ref <= 8'h00;
        else if (byte_valid) begin
            if ($past(byte_cnt) == 4'd0)         // comma emitted
                f_crc_ref <= 8'h00;
            else if ($past(byte_cnt) != 4'd15)
                f_crc_ref <= crc8_update(f_crc_ref, byte_data);
        end
    end

    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && byte_valid
            && $past(byte_cnt) == 4'd15)
            assert (byte_data == f_crc_ref);     // emitted CRC is correct

        // Comma byte is always one of the three legal comma symbols.
        if (f_past_valid && !$past(rst) && byte_valid
            && $past(byte_cnt) == 4'd0)
            assert (byte_data == COMMA_LINK || byte_data == COMMA_CFG
                    || byte_data == COMMA_OP);

        // frame_done pulses exactly with the CRC byte.
        if (f_past_valid && !$past(rst))
            assert (frame_done == (byte_valid && $past(byte_cnt) == 4'd15));
    end

    // Induction strengthening: the shadow CRC trails the design CRC by
    // exactly the byte currently on the wire (or matches it when idle /
    // around the frame boundary), and the latched comma is always legal.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst)) begin
            if (byte_valid) begin
                if ($past(byte_cnt) == 4'd0)
                    assert (crc == 8'h00);
                else if ($past(byte_cnt) != 4'd15)
                    assert (crc == crc8_update(f_crc_ref, byte_data));
                else
                    assert (crc == f_crc_ref);
            end else
                assert (crc == f_crc_ref);
        end
    end

    always_comb
        if (f_past_valid)
            assert (comma_r == COMMA_LINK || comma_r == COMMA_CFG
                    || comma_r == COMMA_OP);

    // With en held, exactly one byte per cycle - no gaps inside a frame.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && $past(en))
            assert (byte_valid);
    end

    always_ff @(posedge clk) begin
        if (f_past_valid)
            cover (frame_done);   // a complete frame gets emitted
    end
`endif

endmodule
