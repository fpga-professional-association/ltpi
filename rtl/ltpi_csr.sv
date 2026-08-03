// LTPI Control and Status Registers - the hardware debug interface
// (spec 3.2, Tables 35/36). Byte-offset map, 32-bit registers, on a simple
// synchronous read/write port that drops directly onto APB, Avalon-MM,
// AXI-lite, a JTAG user register, or an I2C/UART register bridge:
//   we/re pulse with addr (byte offset, word aligned); rdata valid the
//   next cycle; writes take effect at the next clock edge.
//
//   0x00 Link Status     [19:16] local state  [15:12] remote state (est.)
//                        [11:8] speed code    [7] DDR   [5..1] RWC errors
//                        [0] PHY aligned
//   0x04 Caps Local      [23:8] RW speed-capability override (BMC debug)
//                        [7:0]  RO LTPI version 1.0
//   0x08 Caps Remote     [23:8] RO peer capability word
//   0x2C Alignment error counter (RWC)    0x30 Link lost counter (RWC)
//   0x34 CRC error counter (RWC)          0x38 Unknown comma counter (RWC)
//   0x3C Speed timeout counter (RWC)      0x40 Cfg/Accept timeout (RWC)
//   0x54 Operational RX frames (RWC)      0x58 Operational TX frames (RWC)
//   0x80 Link Control    [0] W1 soft reset pulse  [1] W1 retrain pulse
//                        [10] RW auto-configure (cfg_ready)
//
// State encoding per Table 36: 0 Link Detect, 1 Link Speed, 2 Advertise,
// 3 Configuration/Accept, 4 Operational.
import ltpi_pkg::*;

module ltpi_csr (
    input  logic        clk,
    input  logic        rst,

    // Register access port
    input  logic [7:0]  addr,     // byte offset, word aligned
    input  logic        we,
    input  logic        re,
    input  logic [31:0] wdata,
    output logic [31:0] rdata,

    // Status inputs from the core
    input  ltpi_pkg::link_state_t link_state,
    input  ltpi_pkg::frame_type_t last_rx_type,  // remote-state estimate
    input  logic [15:0] speed_select,
    input  logic        speed_valid,
    input  logic        phy_aligned,
    input  logic [15:0] remote_caps,

    // Event pulses from the core
    input  logic ev_align_err,
    input  logic ev_link_lost,
    input  logic ev_crc_err,
    input  logic ev_unk_comma,
    input  logic ev_speed_timeout,
    input  logic ev_cfg_timeout,
    input  logic ev_op_rx,
    input  logic ev_op_tx,

    // Control outputs to the core
    output logic        ctl_soft_reset,   // 1-cycle pulse
    output logic        ctl_retrain,      // 1-cycle pulse
    output logic        ctl_cfg_ready,    // level (auto-configure)
    output logic [15:0] ctl_caps_override // BMC speed-caps override
);

    // Table 36 state encoding from our internal state.
    function automatic logic [3:0] enc_state(input ltpi_pkg::link_state_t s);
        begin
            case (s)
                ST_DETECT_ALIGN, ST_DETECT: enc_state = 4'h0;
                ST_SPEED:                   enc_state = 4'h1;
                ST_ADV_ALIGN, ST_ADV:       enc_state = 4'h2;
                ST_CFG_ACC:                 enc_state = 4'h3;
                default:                    enc_state = 4'h4;  // Operational
            endcase
        end
    endfunction

    // Remote state estimated from the last received frame type.
    function automatic logic [3:0] enc_remote(input ltpi_pkg::frame_type_t t);
        begin
            case (t)
                FRAME_LINK_DETECT: enc_remote = 4'h0;
                FRAME_LINK_SPEED:  enc_remote = 4'h1;
                FRAME_ADVERTISE:   enc_remote = 4'h2;
                FRAME_CONFIGURE,
                FRAME_ACCEPT:      enc_remote = 4'h3;
                FRAME_OPERATIONAL: enc_remote = 4'h4;
                default:           enc_remote = 4'h0;
            endcase
        end
    endfunction

    // Speed code per Table 36 from the one-hot select.
    function automatic logic [3:0] enc_speed(input logic [15:0] sel);
        begin
            casez (sel[11:0])
                12'b????_????_???1: enc_speed = 4'h0;  // x1
                12'b????_????_??1?: enc_speed = 4'h1;  // x2
                12'b????_????_?1??: enc_speed = 4'h2;  // x3
                12'b????_????_1???: enc_speed = 4'h3;  // x4
                12'b????_???1_????: enc_speed = 4'h4;  // x6
                12'b????_??1?_????: enc_speed = 4'h5;  // x8  (200MHz)
                12'b????_?1??_????: enc_speed = 4'h6;  // x10
                12'b????_1???_????: enc_speed = 4'h7;  // x12
                12'b???1_????_????: enc_speed = 4'h8;  // x16 (400MHz)
                12'b??1?_????_????: enc_speed = 4'h9;  // x24
                12'b?1??_????_????: enc_speed = 4'hA;  // x32
                12'b1???_????_????: enc_speed = 4'hB;  // x40
                default:            enc_speed = 4'hF;
            endcase
        end
    endfunction

    // RWC error flags (status bits 5:1) and counters.
    logic [5:1]  err_flags;
    logic [31:0] cnt_align, cnt_lost, cnt_crc, cnt_comma, cnt_spdto,
                 cnt_cfgto, cnt_oprx, cnt_optx;

    function automatic logic [31:0] cnt_next(input logic [31:0] c,
                                             input logic ev);
        begin
            cnt_next = (ev && c != 32'hFFFF_FFFF) ? c + 32'd1 : c;
        end
    endfunction

    logic wr_status, wr_ctrl, wr_caps;
    assign wr_status = we && addr == 8'h00;
    assign wr_caps   = we && addr == 8'h04;
    assign wr_ctrl   = we && addr == 8'h80;

    always_ff @(posedge clk) begin
        if (rst) begin
            err_flags <= '0;
            cnt_align <= '0;  cnt_lost <= '0;  cnt_crc  <= '0;
            cnt_comma <= '0;  cnt_spdto <= '0; cnt_cfgto <= '0;
            cnt_oprx  <= '0;  cnt_optx <= '0;
            ctl_soft_reset   <= 1'b0;
            ctl_retrain      <= 1'b0;
            ctl_cfg_ready    <= 1'b1;
            ctl_caps_override<= CAPS_DEFAULT;
        end else begin
            // Sticky error flags: set on event, write-1-clear.
            err_flags[1] <= (err_flags[1] | ev_link_lost)
                            & ~(wr_status & wdata[1]);
            err_flags[2] <= (err_flags[2] | ev_crc_err)
                            & ~(wr_status & wdata[2]);
            err_flags[3] <= (err_flags[3] | ev_unk_comma)
                            & ~(wr_status & wdata[3]);
            err_flags[4] <= (err_flags[4] | ev_speed_timeout)
                            & ~(wr_status & wdata[4]);
            err_flags[5] <= (err_flags[5] | ev_cfg_timeout)
                            & ~(wr_status & wdata[5]);

            // Saturating counters, RWC (any write-1 to the reg clears).
            cnt_align <= (we && addr == 8'h2C && wdata != 0)
                         ? '0 : cnt_next(cnt_align, ev_align_err);
            cnt_lost  <= (we && addr == 8'h30 && wdata != 0)
                         ? '0 : cnt_next(cnt_lost, ev_link_lost);
            cnt_crc   <= (we && addr == 8'h34 && wdata != 0)
                         ? '0 : cnt_next(cnt_crc, ev_crc_err);
            cnt_comma <= (we && addr == 8'h38 && wdata != 0)
                         ? '0 : cnt_next(cnt_comma, ev_unk_comma);
            cnt_spdto <= (we && addr == 8'h3C && wdata != 0)
                         ? '0 : cnt_next(cnt_spdto, ev_speed_timeout);
            cnt_cfgto <= (we && addr == 8'h40 && wdata != 0)
                         ? '0 : cnt_next(cnt_cfgto, ev_cfg_timeout);
            cnt_oprx  <= (we && addr == 8'h54 && wdata != 0)
                         ? '0 : cnt_next(cnt_oprx, ev_op_rx);
            cnt_optx  <= (we && addr == 8'h58 && wdata != 0)
                         ? '0 : cnt_next(cnt_optx, ev_op_tx);

            // Link control: bits 0/1 are self-clearing pulses.
            ctl_soft_reset <= wr_ctrl && wdata[0];
            ctl_retrain    <= wr_ctrl && wdata[1];
            if (wr_ctrl)
                ctl_cfg_ready <= wdata[10];
            if (wr_caps)
                ctl_caps_override <= wdata[23:8];
        end
    end

    // Read mux (registered - rdata valid the cycle after re).
    always_ff @(posedge clk) begin
        if (rst)
            rdata <= '0;
        else if (re) begin
            case (addr)
                8'h00: rdata <= {12'd0, enc_state(link_state),
                                 enc_remote(last_rx_type),
                                 speed_valid ? enc_speed(speed_select) : 4'hF,
                                 speed_valid ? speed_select[15] : 1'b0,
                                 1'b0, err_flags[5:1], phy_aligned};
                8'h04: rdata <= {8'd0, ctl_caps_override, 8'h10};
                8'h08: rdata <= {8'd0, remote_caps, 8'h00};
                8'h2C: rdata <= cnt_align;
                8'h30: rdata <= cnt_lost;
                8'h34: rdata <= cnt_crc;
                8'h38: rdata <= cnt_comma;
                8'h3C: rdata <= cnt_spdto;
                8'h40: rdata <= cnt_cfgto;
                8'h54: rdata <= cnt_oprx;
                8'h58: rdata <= cnt_optx;
                8'h80: rdata <= {21'd0, ctl_cfg_ready, 10'd0};
                default: rdata <= 32'h0;
            endcase
        end
    end

`ifdef FORMAL
    logic f_past_valid = 1'b0;
    always_ff @(posedge clk)
        f_past_valid <= 1'b1;

    initial assume (rst);

    // C1: control pulses fire exactly on a write-1 to reg 0x80.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst)) begin
            assert (ctl_soft_reset == ($past(we) && $past(addr) == 8'h80
                                       && $past(wdata[0])));
            assert (ctl_retrain    == ($past(we) && $past(addr) == 8'h80
                                       && $past(wdata[1])));
        end
    end

    // C2: RWC semantics - a write-1 clears the flag even against a
    // simultaneous event... event wins only when no clear is issued.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst)) begin
            if ($past(we) && $past(addr) == 8'h00 && $past(wdata[2])
                )
                assert (!err_flags[2]);
            if (!$past(we) && $past(ev_crc_err))
                assert (err_flags[2]);
        end
    end

    // C3: counters saturate - never wrap to zero by counting.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst)
            && $past(cnt_crc) == 32'hFFFF_FFFF && !$past(we))
            assert (cnt_crc == 32'hFFFF_FFFF);
        if (f_past_valid && !$past(rst)
            && $past(we) && $past(addr) == 8'h34 && $past(wdata) != 0)
            assert (cnt_crc == 0);
    end

    // C4: read-only registers are unaffected by writes (remote caps mirror
    // is a pure function of the inputs; a write to 0x08 changes nothing).
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && $past(we) && $past(addr) == 8'h08)
            assert (cnt_crc == $past(cnt_crc)
                    || $past(ev_crc_err));   // no cross-register side effect
    end

    // C5: status read reflects the sampled state encoding.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && $past(re) && $past(addr) == 8'h00)
            assert (rdata[0] == $past(phy_aligned)
                    && rdata[19:16] == enc_state($past(link_state)));
    end

    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            cover ($past(re) && $past(addr) == 8'h00 && rdata[19:16] == 4'h4);
            cover (ctl_retrain);
            cover (err_flags[2] && cnt_crc != 0);
        end
    end
`endif

endmodule
