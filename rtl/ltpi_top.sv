// LTPI endpoint top: PHY serdes + framer + link FSM + channels.
// One instance per side (ROLE_SCM selects SCM CPLD vs HPM FPGA behavior).
//
// Datapath (byte domain, one byte per PHY byte_req):
//   ltpi_phy_rx -> ltpi_frame_rx -> ltpi_link_fsm / channel decoders
//   channel encoders -> payload mux -> ltpi_frame_tx -> ltpi_phy_tx
//
// Default I/O Frame layout handled here (spec Table 33):
//   byte 2  frame counter        bytes 3-4  LL GPIO
//   bytes 5-6 NL GPIO slice      byte 7     UART0/1 fields
//   byte 8  I2C/SMBus 0/1        bytes 9-14 reserved/OEM + CRC
import ltpi_pkg::*;

module ltpi_top #(
    parameter bit ROLE_SCM = 1'b1,
    parameter int unsigned NL_TOTAL = 32,
    // Training thresholds (spec defaults; shrink for simulation)
    parameter int unsigned DETECT_MIN_TX    = 255,
    parameter int unsigned DETECT_MIN_RX    = 7,
    parameter int unsigned ALIGN_MIN_RX     = 3,
    parameter int unsigned SPEED_SCM_MIN_TX = 7,
    parameter int unsigned SPEED_HPM_MIN_RX = 3,
    parameter int unsigned SPEED_TIMEOUT_TX = 255,
    parameter int unsigned ADV_MIN_CYCLES   = 25000,
    parameter int unsigned ADV_ALIGN_TIMEOUT = 2_500_000,
    parameter int unsigned ADV_MIN_RX       = 3,
    parameter int unsigned CFG_MAX_TX       = 31,
    parameter int unsigned ACC_MAX_TX       = 15
)(
    input  logic clk,           // link clock
    input  logic rst,
    input  logic ddr_mode,      // PHY rate select (from system controller)
    input  logic [15:0] local_speed_caps,
    input  logic cfg_ready,     // SCM: configuration selected (auto/BMC)
    input  logic retrain_req,
    input  logic soft_reset,

    // Serial link (connect to ltpi_lvds_io or directly in simulation)
    output logic       ser_tx_sdr,
    output logic [1:0] ser_tx_ddr,
    input  logic       ser_rx_sdr,
    input  logic [1:0] ser_rx_ddr,

    // GPIO channel
    input  logic [15:0] ll_gpio_in,
    output logic [15:0] ll_gpio_out,
    input  logic [NL_TOTAL-1:0] nl_gpio_in,
    output logic [NL_TOTAL-1:0] nl_gpio_out,

    // UART channel 0 (tunnel TX direction of this side)
    input  logic uart_txd_in,
    input  logic uart_flow_in,
    output logic uart_txd_out,
    output logic uart_flow_out,

    // I2C channel 0 relay bus-side interface
    input  logic i2c_start_det, i2c_stop_det, i2c_scl_rise, i2c_scl_fall,
    input  logic i2c_sda_val,
    output logic i2c_scl_stretch, i2c_sda_pull,
    output logic i2c_bus_start_gen, i2c_bus_stop_gen,

    // Data channel: local requester + completer master (AVMM/APB-style)
    input  logic        dc_req_valid,
    input  logic        dc_req_write,
    input  logic [31:0] dc_req_addr,
    input  logic [31:0] dc_req_wdata,
    input  logic [3:0]  dc_req_byteen,
    output logic        dc_req_ready,
    output logic        dc_rsp_valid,
    output logic [31:0] dc_rsp_rdata,
    output logic        dc_rsp_error,
    output logic        dc_cmp_req,
    output logic        dc_cmp_write,
    output logic [31:0] dc_cmp_addr,
    output logic [31:0] dc_cmp_wdata,
    output logic [3:0]  dc_cmp_byteen,
    input  logic [31:0] dc_cmp_rdata,
    input  logic        dc_cmp_done,

    // CSR / debug register interface (APB/AVMM/JTAG-bridge friendly)
    input  logic [7:0]  csr_addr,
    input  logic        csr_we,
    input  logic        csr_re,
    input  logic [31:0] csr_wdata,
    output logic [31:0] csr_rdata,

    // Status
    output ltpi_pkg::link_state_t link_state,
    output logic        link_up,
    output logic [15:0] speed_select,
    output logic        speed_valid,
    output logic        phy_aligned
);

    // ------------------------------------------------------------------
    // PHY RX -> frame RX
    // ------------------------------------------------------------------
    logic       rx_byte_valid;
    logic [7:0] rx_byte_data;
    logic       rx_phase_odd;

    logic        frame_valid, frame_crc_ok;
    ltpi_pkg::frame_type_t frame_type;
    logic [15:0] rx_speed_payload;
    logic [103:0] rx_payload;

    // False-lock recovery: 4 consecutive CRC-bad frames while aligned means
    // the comma hunt latched onto payload data - drop and re-acquire.
    logic [2:0] crc_fail_run;
    logic       phy_realign;
    always_ff @(posedge clk) begin
        if (rst || !phy_aligned)
            crc_fail_run <= '0;
        else if (frame_valid)
            crc_fail_run <= frame_crc_ok ? '0 : crc_fail_run + 1'b1;
    end
    assign phy_realign = (crc_fail_run == 3'd4);

    ltpi_phy_rx u_phy_rx (
        .clk(clk), .rst(rst),
        .ddr_mode(ddr_mode),
        .hunt(1'b1),
        .realign(phy_realign),
        .ser_sdr(ser_rx_sdr),
        .ser_ddr(ser_rx_ddr),
        .aligned(phy_aligned),
        .phase_odd(rx_phase_odd),
        .byte_valid(rx_byte_valid),
        .byte_data(rx_byte_data)
    );

    ltpi_frame_rx u_frame_rx (
        .clk(clk), .rst(rst),
        .aligned(phy_aligned),
        .byte_valid(rx_byte_valid),
        .byte_data(rx_byte_data),
        .frame_valid(frame_valid),
        .frame_crc_ok(frame_crc_ok),
        .frame_type(frame_type),
        .speed_payload(rx_speed_payload),
        .payload(rx_payload)
    );

    // ------------------------------------------------------------------
    // Link FSM
    // ------------------------------------------------------------------
    ltpi_pkg::frame_type_t tx_frame_type;
    logic tx_frame_done;
    logic [15:0] remote_speed_caps;
    logic remote_caps_valid;

    // CSR control merge: BMC (via CSR) and the pins both control the link;
    // the caps override can only RESTRICT the pin capabilities (spec: BMC
    // lowers the operational frequency for debug/recovery).
    logic        csr_soft_reset, csr_retrain, csr_cfg_ready;
    logic [15:0] csr_caps_override;

    ltpi_link_fsm #(
        .ROLE_SCM(ROLE_SCM),
        .DETECT_MIN_TX(DETECT_MIN_TX), .DETECT_MIN_RX(DETECT_MIN_RX),
        .ALIGN_MIN_RX(ALIGN_MIN_RX),
        .SPEED_SCM_MIN_TX(SPEED_SCM_MIN_TX),
        .SPEED_HPM_MIN_RX(SPEED_HPM_MIN_RX),
        .SPEED_TIMEOUT_TX(SPEED_TIMEOUT_TX),
        .ADV_MIN_CYCLES(ADV_MIN_CYCLES), .ADV_MIN_RX(ADV_MIN_RX),
        .ADV_ALIGN_TIMEOUT(ADV_ALIGN_TIMEOUT),
        .CFG_MAX_TX(CFG_MAX_TX), .ACC_MAX_TX(ACC_MAX_TX)
    ) u_fsm (
        .clk(clk), .rst(rst),
        .retrain_req(retrain_req | csr_retrain),
        .soft_reset(soft_reset | csr_soft_reset),
        .local_speed_caps(local_speed_caps & csr_caps_override),
        .cfg_ready(cfg_ready & csr_cfg_ready),
        .rx_aligned(phy_aligned),
        .rx_frame_valid(frame_valid),
        .rx_crc_ok(frame_crc_ok),
        .rx_frame_type(frame_type),
        .rx_speed_payload(rx_speed_payload),
        .rx_cfg_match(1'b1),     // default capabilities always match
        .tx_frame_done(tx_frame_done),
        .tx_frame_type(tx_frame_type),
        .state(link_state),
        .remote_speed_caps(remote_speed_caps),
        .remote_caps_valid(remote_caps_valid),
        .speed_select(speed_select),
        .speed_valid(speed_valid),
        .link_up(link_up)
    );

    // ------------------------------------------------------------------
    // Channels
    // ------------------------------------------------------------------
    // Declared wires, not inline port expressions: some simulators
    // mis-evaluate enum comparisons written directly in port connections.
    logic op_frame_any, op_frame_good;
    assign op_frame_any  = frame_valid && frame_type == FRAME_OPERATIONAL
                           && link_up;
    assign op_frame_good = op_frame_any && frame_crc_ok;

    // GPIO
    logic [7:0]  gpio_tx_counter;
    logic [15:0] gpio_tx_ll, gpio_tx_nl;

    ltpi_gpio_channel #(.NL_TOTAL(NL_TOTAL)) u_gpio (
        .clk(clk), .rst(rst),
        .ll_in(ll_gpio_in),
        .nl_in(nl_gpio_in),
        .tx_frame_start(tx_frame_done && link_up),
        .tx_frame_counter(gpio_tx_counter),
        .tx_ll(gpio_tx_ll),
        .tx_nl(gpio_tx_nl),
        .rx_frame_valid(op_frame_any),
        .rx_crc_ok(frame_crc_ok),
        .rx_frame_counter(rx_payload[7:0]),          // byte 2
        .rx_ll(rx_payload[23:8]),                    // bytes 3-4
        .rx_nl(rx_payload[39:24]),                   // bytes 5-6
        .ll_out(ll_gpio_out),
        .nl_out(nl_gpio_out)
    );

    // UART strobes: 3 evenly spaced per 16-byte frame (positions 0/5/10 of
    // the TX byte counter - spec Fig 15 shows an example distribution).
    logic [3:0] frame_pos;
    logic       tx_byte_req;
    always_ff @(posedge clk) begin
        if (rst)
            frame_pos <= '0;
        else if (tx_byte_req)
            frame_pos <= (frame_pos == 4'd15) ? 4'd0 : frame_pos + 1'b1;
    end

    logic uart_strobe;
    assign uart_strobe = tx_byte_req
                         && (frame_pos == 4'd0 || frame_pos == 4'd5
                             || frame_pos == 4'd10);

    logic [3:0] uart_tx_field;
    ltpi_uart_channel u_uart (
        .clk(clk), .rst(rst),
        .tx_sample(uart_strobe && !(tx_frame_done && link_up)),
        .txd_in(uart_txd_in),
        .flow_in(uart_flow_in),
        .tx_frame_start(tx_frame_done && link_up),
        .tx_field(uart_tx_field),
        .rx_frame_valid(op_frame_any),
        .rx_crc_ok(frame_crc_ok),
        .rx_field(rx_payload[43:40]),                // byte 7 low nibble
        .rx_replay(uart_strobe),
        .txd_out(uart_txd_out),
        .flow_out(uart_flow_out)
    );

    // I2C relay (channel 0). Bidirectional: either side can initiate a
    // transaction (SPDM/MCTP requirement); the SCM side wins a
    // simultaneous-start race, the loser defers via clock stretching.
    logic [3:0] i2c_tx_event;
    logic [3:0] i2c_state;
    logic       i2c_initiator, i2c_start_deferred;

    // Frame-layer pacing feedback: when an I/O frame completes, the value
    // latched at ITS start (i2c_framed_ev) has been carried on the wire.
    logic [3:0] i2c_framed_ev, i2c_sent_ev;
    logic       i2c_ev_sent;
    always_ff @(posedge clk) begin
        if (rst) begin
            i2c_framed_ev <= I2C_EV_IDLE;
            i2c_sent_ev   <= I2C_EV_IDLE;
            i2c_ev_sent   <= 1'b0;
        end else begin
            i2c_ev_sent <= 1'b0;
            if (tx_frame_done && link_up) begin
                i2c_sent_ev   <= i2c_framed_ev;
                i2c_framed_ev <= i2c_tx_event;
                i2c_ev_sent   <= 1'b1;
            end
        end
    end

    ltpi_i2c_relay #(.ARB_PRIORITY(ROLE_SCM)) u_i2c (
        .clk(clk), .rst(rst),
        .start_det(i2c_start_det), .stop_det(i2c_stop_det),
        .scl_rise(i2c_scl_rise), .scl_fall(i2c_scl_fall),
        .sda_val(i2c_sda_val),
        .scl_stretch(i2c_scl_stretch), .sda_pull(i2c_sda_pull),
        .bus_start_gen(i2c_bus_start_gen), .bus_stop_gen(i2c_bus_stop_gen),
        .tx_event(i2c_tx_event),
        .rx_event(rx_payload[51:48]),                // byte 8 low nibble
        .rx_event_valid(op_frame_good),
        .tx_sent_valid(i2c_ev_sent),
        .tx_sent_event(i2c_sent_ev),
        .state(i2c_state),
        .initiator(i2c_initiator),
        .start_deferred(i2c_start_deferred)
    );

    // ------------------------------------------------------------------
    // TX payload mux + frame TX + PHY TX
    // ------------------------------------------------------------------
    localparam logic [7:0] LTPI_VERSION = 8'h10;   // v1.0 BCD (Table 22)

    // ---------------- Data channel ----------------
    logic         dtx_req, dtx_grant, drx_valid;
    logic [103:0] dtx_payload;
    logic         send_data, framed_is_data;

    // Interleave: at each frame boundary decide whether the NEXT frame is
    // a Data Frame; never two in a row (spec 2.7: bound Random Latency).
    always_ff @(posedge clk) begin
        if (rst) begin
            send_data      <= 1'b0;
            framed_is_data <= 1'b0;
        end else if (tx_frame_done) begin
            framed_is_data <= send_data && link_up;
            send_data      <= dtx_req && link_up && !send_data;
        end
    end
    assign dtx_grant = tx_frame_done && framed_is_data;

    assign drx_valid = frame_valid && frame_crc_ok
                       && frame_type == FRAME_OP_DATA && link_up;

    ltpi_data_channel u_data (
        .clk(clk), .rst(rst), .link_up(link_up),
        .req_valid(dc_req_valid), .req_write(dc_req_write),
        .req_addr(dc_req_addr), .req_wdata(dc_req_wdata),
        .req_byteen(dc_req_byteen), .req_ready(dc_req_ready),
        .rsp_valid(dc_rsp_valid), .rsp_rdata(dc_rsp_rdata),
        .rsp_error(dc_rsp_error),
        .cmp_req(dc_cmp_req), .cmp_write(dc_cmp_write),
        .cmp_addr(dc_cmp_addr), .cmp_wdata(dc_cmp_wdata),
        .cmp_byteen(dc_cmp_byteen), .cmp_rdata(dc_cmp_rdata),
        .cmp_done(dc_cmp_done),
        .dtx_req(dtx_req), .dtx_payload(dtx_payload),
        .dtx_grant(dtx_grant),
        .drx_valid(drx_valid), .drx_payload(rx_payload)
    );

    // Effective TX frame type: substitute a Data Frame into the
    // operational stream when the data channel holds a message.
    ltpi_pkg::frame_type_t tx_ftype_eff;
    always_comb begin
        tx_ftype_eff = tx_frame_type;
        if (tx_frame_type == FRAME_OPERATIONAL && send_data)
            tx_ftype_eff = FRAME_OP_DATA;
    end

    logic [103:0] tx_payload;
    always_comb begin
        tx_payload = '0;
        case (tx_ftype_eff)
            FRAME_LINK_DETECT: begin
                tx_payload[7:0]   = LTPI_VERSION;             // byte 2
                tx_payload[23:8]  = local_speed_caps;         // bytes 3-4
            end
            FRAME_LINK_SPEED: begin
                tx_payload[7:0]   = LTPI_VERSION;
                tx_payload[23:8]  = speed_select;             // Table 24
            end
            FRAME_ADVERTISE, FRAME_CONFIGURE, FRAME_ACCEPT: begin
                tx_payload[23:8]  = 16'h0001;  // platform/caps type: default
                // Default capabilities: GPIO+I2C+UART+Data supported
                tx_payload[31:24] = 8'h0F;
            end
            FRAME_OP_DATA: begin // Default Data Frame, Table 34
                tx_payload = dtx_payload;
            end
            default: begin // FRAME_OPERATIONAL: Default I/O Frame, Table 33
                tx_payload[7:0]   = gpio_tx_counter;          // byte 2
                tx_payload[23:8]  = gpio_tx_ll;               // bytes 3-4
                tx_payload[39:24] = gpio_tx_nl;               // bytes 5-6
                tx_payload[43:40] = uart_tx_field;            // byte 7 lo
                tx_payload[47:44] = 4'h0;                     // UART1 unused
                tx_payload[51:48] = i2c_tx_event;             // byte 8 lo
                tx_payload[55:52] = I2C_EV_IDLE;              // I2C1 idle
            end
        endcase
    end

    logic       tx_byte_valid;
    logic [7:0] tx_byte_data;

    ltpi_frame_tx u_frame_tx (
        .clk(clk), .rst(rst),
        .en(tx_byte_req),
        .frame_type(tx_ftype_eff),
        .payload(tx_payload),
        .byte_valid(tx_byte_valid),
        .byte_data(tx_byte_data),
        .frame_done(tx_frame_done)
    );

    ltpi_phy_tx u_phy_tx (
        .clk(clk), .rst(rst),
        .ddr_mode(ddr_mode),
        .byte_data(tx_byte_data),
        .byte_req(tx_byte_req),
        .ser_sdr(ser_tx_sdr),
        .ser_ddr(ser_tx_ddr)
    );

    // ------------------------------------------------------------------
    // CSR debug block (spec Table 36): status, error counters, control
    // ------------------------------------------------------------------
    ltpi_pkg::frame_type_t last_rx_type;
    ltpi_pkg::link_state_t state_q;
    always_ff @(posedge clk) begin
        if (rst) begin
            last_rx_type <= FRAME_INVALID;
            state_q      <= ST_DETECT_ALIGN;
        end else begin
            if (frame_valid && frame_crc_ok)
                last_rx_type <= frame_type;
            state_q <= link_state;
        end
    end

    // Event pulses derived from state edges / frame events (declared
    // wires - not inline port expressions, see enum-port simulator note).
    logic ev_link_lost, ev_speed_to, ev_cfg_to, ev_crc_err, ev_unk_comma,
          ev_op_tx;
    assign ev_link_lost = (state_q == ST_ADV || state_q == ST_CFG_ACC
                           || state_q == ST_OPERATIONAL)
                          && link_state == ST_DETECT_ALIGN;
    assign ev_speed_to  = state_q == ST_SPEED
                          && link_state == ST_DETECT_ALIGN;
    assign ev_cfg_to    = state_q == ST_CFG_ACC && link_state == ST_ADV;
    assign ev_crc_err   = frame_valid && !frame_crc_ok;
    assign ev_unk_comma = frame_valid && frame_type == FRAME_INVALID;
    assign ev_op_tx     = tx_frame_done && link_up;

    ltpi_csr u_csr (
        .clk(clk), .rst(rst),
        .addr(csr_addr), .we(csr_we), .re(csr_re),
        .wdata(csr_wdata), .rdata(csr_rdata),
        .link_state(link_state),
        .last_rx_type(last_rx_type),
        .speed_select(speed_select),
        .speed_valid(speed_valid),
        .phy_aligned(phy_aligned),
        .remote_caps(remote_speed_caps),
        .ev_align_err(phy_realign),
        .ev_link_lost(ev_link_lost),
        .ev_crc_err(ev_crc_err),
        .ev_unk_comma(ev_unk_comma),
        .ev_speed_timeout(ev_speed_to),
        .ev_cfg_timeout(ev_cfg_to),
        .ev_op_rx(op_frame_good),
        .ev_op_tx(ev_op_tx),
        .ctl_soft_reset(csr_soft_reset),
        .ctl_retrain(csr_retrain),
        .ctl_cfg_ready(csr_cfg_ready),
        .ctl_caps_override(csr_caps_override)
    );

endmodule
