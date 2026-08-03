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
    parameter int unsigned NUM_I2C  = 2,     // 1..6 relay channels
    parameter int unsigned CLK_HZ   = 400_000_000, // link clock (tSP filter)
    parameter logic [15:0] PLATFORM_ID = PLATID_THIS_CORE, // Table 26 (OEM)
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

    // I2C/SMBus buses, NUM_I2C channels (1..6). RAW bus levels in - each
    // channel passes a 2-FF synchronizer + proven 50ns tSP spike filter
    // (ltpi_i2c_cond) before the relay. Channels 0-1 ride I/O-frame
    // byte 8, 2-3 byte 9, 4-5 byte 10 (Table 33 / Table 11 packing).
    input  logic [NUM_I2C-1:0] i2c_scl_in,
    input  logic [NUM_I2C-1:0] i2c_sda_in,
    output logic [NUM_I2C-1:0] i2c_scl_stretch,
    output logic [NUM_I2C-1:0] i2c_sda_pull,
    output logic [NUM_I2C-1:0] i2c_bus_start_gen,
    output logic [NUM_I2C-1:0] i2c_bus_stop_gen,

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

    // OEM APB channel (tunneled in I/O-frame OEM bytes 11-14)
    input  logic        apb_s_psel,
    input  logic        apb_s_penable,
    input  logic        apb_s_pwrite,
    input  logic [31:0] apb_s_paddr,
    input  logic [31:0] apb_s_pwdata,
    input  logic [3:0]  apb_s_pstrb,
    output logic        apb_s_pready,
    output logic [31:0] apb_s_prdata,
    output logic        apb_s_pslverr,
    output logic        apb_m_psel,
    output logic        apb_m_penable,
    output logic        apb_m_pwrite,
    output logic [31:0] apb_m_paddr,
    output logic [31:0] apb_m_pwdata,
    output logic [3:0]  apb_m_pstrb,
    input  logic        apb_m_pready,
    input  logic [31:0] apb_m_prdata,
    input  logic        apb_m_pslverr,

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
    ltpi_pkg::frame_type_t tx_ftype_eff;   // driven below (data interleave)
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

    // Retimed copy for the I2C relays: the frame-decode -> relay-FSM cone
    // was an Agilex 3 critical path. The payload nibbles stay stable for a
    // full frame time, so delaying only the valid strobe is safe.
    logic op_frame_good_q;
    always_ff @(posedge clk) begin
        if (rst) op_frame_good_q <= 1'b0;
        else     op_frame_good_q <= op_frame_good;
    end

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

    // I2C relays, NUM_I2C channels, one nibble each in I/O-frame bytes
    // 8-10 (Table 11 packing: channel n -> payload[48 + 4n +: 4]).
    // Bidirectional: either side can initiate a transaction (SPDM/MCTP
    // requirement); the SCM side wins a simultaneous-start race, the
    // loser defers via clock stretching. Adding a channel = bumping
    // NUM_I2C - the generate block does the rest.
    logic [3:0] i2c_tx_event    [NUM_I2C];
    logic [3:0] i2c_framed_ev   [NUM_I2C];
    logic [3:0] i2c_sent_ev     [NUM_I2C];
    logic       i2c_ev_sent;

    // Frame-layer pacing feedback: when an I/O frame completes, the value
    // latched at ITS start (i2c_framed_ev) has been carried on the wire.
    always_ff @(posedge clk) begin
        if (rst) begin
            i2c_ev_sent <= 1'b0;
            for (int k = 0; k < NUM_I2C; k++) begin
                i2c_framed_ev[k] <= I2C_EV_IDLE;
                i2c_sent_ev[k]   <= I2C_EV_IDLE;
            end
        end else begin
            i2c_ev_sent <= 1'b0;
            if (tx_frame_done && link_up) begin
                for (int k = 0; k < NUM_I2C; k++) begin
                    i2c_sent_ev[k]   <= i2c_framed_ev[k];
                    i2c_framed_ev[k] <= i2c_tx_event[k];
                end
                i2c_ev_sent <= 1'b1;
            end
        end
    end

    generate
        for (genvar gi = 0; gi < NUM_I2C; gi++) begin : g_i2c
            logic c_start, c_stop, c_rise, c_fall, c_sda;

            ltpi_i2c_cond #(.CLK_HZ(CLK_HZ)) u_cond (
                .clk(clk), .rst(rst),
                .scl_in(i2c_scl_in[gi]), .sda_in(i2c_sda_in[gi]),
                .scl_filt(), .sda_filt(),
                .start_det(c_start), .stop_det(c_stop),
                .scl_rise(c_rise), .scl_fall(c_fall), .sda_val(c_sda)
            );

            ltpi_i2c_relay #(.ARB_PRIORITY(ROLE_SCM)) u_i2c (
                .clk(clk), .rst(rst),
                .start_det(c_start), .stop_det(c_stop),
                .scl_rise(c_rise), .scl_fall(c_fall),
                .sda_val(c_sda),
                .scl_stretch(i2c_scl_stretch[gi]),
                .sda_pull(i2c_sda_pull[gi]),
                .bus_start_gen(i2c_bus_start_gen[gi]),
                .bus_stop_gen(i2c_bus_stop_gen[gi]),
                .tx_event(i2c_tx_event[gi]),
                .rx_event(rx_payload[48 + 4*gi +: 4]),
                .rx_event_valid(op_frame_good_q),
                .tx_sent_valid(i2c_ev_sent),
                .tx_sent_event(i2c_sent_ev[gi]),
                .state(), .initiator(), .start_deferred()
            );
        end
    endgenerate

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

    // ---------------- OEM APB channel ----------------
    logic [31:0] apb_oem_tx;
    logic        apb_oem_taken, apb_oem_rxv;
    assign apb_oem_taken = tx_frame_done && link_up
                           && tx_ftype_eff == FRAME_OPERATIONAL;
    assign apb_oem_rxv   = op_frame_good;

    ltpi_oem_apb u_oem_apb (
        .clk(clk), .rst(rst), .link_up(link_up),
        .s_psel(apb_s_psel), .s_penable(apb_s_penable),
        .s_pwrite(apb_s_pwrite), .s_paddr(apb_s_paddr),
        .s_pwdata(apb_s_pwdata), .s_pstrb(apb_s_pstrb),
        .s_pready(apb_s_pready), .s_prdata(apb_s_prdata),
        .s_pslverr(apb_s_pslverr),
        .m_psel(apb_m_psel), .m_penable(apb_m_penable),
        .m_pwrite(apb_m_pwrite), .m_paddr(apb_m_paddr),
        .m_pwdata(apb_m_pwdata), .m_pstrb(apb_m_pstrb),
        .m_pready(apb_m_pready), .m_prdata(apb_m_prdata),
        .m_pslverr(apb_m_pslverr),
        .oem_tx(apb_oem_tx), .oem_tx_taken(apb_oem_taken),
        .oem_rx_valid(apb_oem_rxv), .oem_rx(rx_payload[103:72])
    );


    // Effective TX frame type: substitute a Data Frame into the
    // operational stream when the data channel holds a message.
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
                // Table 26/29: OEM platform ID + default capabilities type,
                // then the real Table 28 capability bytes of THIS build.
                tx_payload[15:0]  = PLATFORM_ID;              // bytes 2-3
                tx_payload[23:16] = 8'h00;                    // caps type
                tx_payload[24]    = 1'b1;                     // GPIO chan
                tx_payload[25]    = 1'b1;                     // I2C chan
                tx_payload[26]    = 1'b1;                     // UART chan
                tx_payload[27]    = 1'b1;                     // Data chan
                tx_payload[39:32] = 8'(NL_TOTAL % 256);       // NL cnt lo
                tx_payload[41:40] = 2'(NL_TOTAL / 256);       // NL cnt hi
                tx_payload[53:48] = 6'((1 << NUM_I2C) - 1);   // I2C enables
                tx_payload[54]    = 1'b1;                     // Echo (>=2.1)
                tx_payload[67:64] = 4'h0A;                    // 115200 baud
                tx_payload[68]    = 1'b1;                     // flow control
                tx_payload[69]    = 1'b1;                     // UART0 enable
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
                for (int k = 0; k < 6; k++)                   // bytes 8-10
                    tx_payload[48 + 4*k +: 4] =
                        (k < NUM_I2C) ? i2c_tx_event[k] : I2C_EV_IDLE;
                tx_payload[103:72] = apb_oem_tx;              // bytes 11-14
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
    // Peer identification & feature-row decode (Advertise, Tables 26/28).
    // Tells firmware WHO is on the far end (ASPEED BMC, Lattice, OCP ref,
    // ...) and exactly which channels/features it advertised.
    // ------------------------------------------------------------------
    logic        peer_valid, peer_caps_default;
    logic [15:0] peer_platform_id;
    ltpi_pkg::peer_vendor_t peer_vendor;
    logic        pf_gpio, pf_i2c, pf_uart, pf_data, pf_oem;
    logic [9:0]  pf_nl_cnt;
    logic [5:0]  pf_i2c_en, pf_i2c_speed;
    logic        pf_i2c_echo, pf_uart_flow;
    logic [3:0]  pf_uart_baud;
    logic [1:0]  pf_uart_en;
    logic [15:0] peer_oem_caps;

    ltpi_peer_decode u_peer (
        .clk(clk), .rst(rst),
        .frame_valid(frame_valid),
        .frame_crc_ok(frame_crc_ok),
        .frame_type(frame_type),
        .payload(rx_payload),
        .peer_valid(peer_valid),
        .peer_platform_id(peer_platform_id),
        .peer_vendor(peer_vendor),
        .caps_default(peer_caps_default),
        .feat_gpio(pf_gpio), .feat_i2c(pf_i2c), .feat_uart(pf_uart),
        .feat_data(pf_data), .feat_oem(pf_oem),
        .feat_nl_gpio_cnt(pf_nl_cnt),
        .feat_i2c_en(pf_i2c_en), .feat_i2c_speed(pf_i2c_speed),
        .feat_i2c_echo(pf_i2c_echo),
        .feat_uart_baud(pf_uart_baud), .feat_uart_flow(pf_uart_flow),
        .feat_uart_en(pf_uart_en),
        .peer_oem_caps(peer_oem_caps)
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
        .local_platform_id(PLATFORM_ID),
        .peer_valid(peer_valid),
        .peer_platform_id(peer_platform_id),
        .peer_vendor(peer_vendor),
        .peer_caps_default(peer_caps_default),
        .peer_channels({pf_oem, pf_data, pf_uart, pf_i2c, pf_gpio}),
        .peer_nl_cnt(pf_nl_cnt),
        .peer_i2c_en(pf_i2c_en),
        .peer_i2c_speed(pf_i2c_speed),
        .peer_uart({pf_uart_en, pf_uart_flow, pf_uart_baud}),
        .peer_oem_caps(peer_oem_caps),
        .ctl_soft_reset(csr_soft_reset),
        .ctl_retrain(csr_retrain),
        .ctl_cfg_ready(csr_cfg_ready),
        .ctl_caps_override(csr_caps_override)
    );

endmodule
