// =============================================================================
// ltpi_core.sv  -  LTPI endpoint core (ROLE = SCM or HPM)
//
// Ties the frozen datapath:
//   pads(serial) <-> frame_tx / frame_rx <-> link_fsm + channels + CSR
//
// TX path: the link FSM owns training frames; in Operational the core emits I/O
// frames built from the channels (GPIO/UART/I2C), or a Data frame when the Data
// channel has a request/completion pending (interleaving, spec Sec 4.1.3).
// RX path: a good frame is routed by comma+subtype to the matching channel.
// Channels run only while the link is Operational and the channel is enabled in
// the negotiated capabilities.
// =============================================================================
`ifndef LTPI_CORE_SV
`define LTPI_CORE_SV
`include "ltpi_pkg.sv"
`include "ltpi_frame_tx.sv"
`include "ltpi_frame_rx.sv"
`include "ltpi_link_fsm.sv"
`include "ltpi_gpio_chan.sv"
`include "ltpi_uart_chan.sv"
`include "ltpi_i2c_relay.sv"
`include "ltpi_data_chan.sv"
`include "ltpi_csr.sv"
import ltpi_pkg::*;

module ltpi_core #(
  parameter bit          ROLE             = ltpi_pkg::ROLE_SCM,
  parameter int          NUM_NL           = 32,
  parameter int          NUM_I2C          = 2,    // 1..6
  parameter int          ADVERTISE_CYCLES = 32,
  parameter logic [15:0] SPEED_CAP        = 16'h008F,
  parameter logic [15:0] PLATFORM_ID      = 16'hABCD,
  // default advertised capabilities (Table 28): GPIO+I2C+UART+Data enabled
  parameter logic [63:0] CAPS             = 64'h0000_0000_0000_000F
) (
  input  logic        clk,        // system clock (core domain)
  input  logic        rst_n,

  // ---- PHY symbol interface (to ltpi_phy, system-clock side of the CDC FIFOs) ----
  output logic [9:0]  tx_sym,
  output logic        tx_sym_valid,
  input  logic        tx_sym_ready,
  input  logic [9:0]  rx_sym,
  input  logic        rx_sym_valid,
  output logic        realign,      // re-acquire comma/word alignment
  input  logic        rx_aligned,
  output logic        speed_change, // pulse: reconfigure PHY PLL to operational rate
  output logic [3:0]  op_speed,     // operational speed code
  output logic        op_ddr,       // operational DDR select

  // ---- GPIO channel ----
  input  logic [15:0]        ll_in,
  output logic [15:0]        ll_out,
  input  logic [NUM_NL-1:0]  nl_in,
  output logic [NUM_NL-1:0]  nl_out,

  // ---- UART channel (2 links) ----
  input  logic        txd0, txd1, rts0, rts1,
  output logic        rxd0, rxd1, cts0, cts1,

  // ---- I2C relay (abstract bus interface, NUM_I2C links) ----
  input  logic [NUM_I2C-1:0]     i2c_evt_det,
  input  logic [NUM_I2C*4-1:0]   i2c_evt_code,
  output logic [NUM_I2C-1:0]     i2c_regen,
  output logic [NUM_I2C*4-1:0]   i2c_regen_code,
  input  logic [NUM_I2C-1:0]     i2c_regen_done,
  output logic [NUM_I2C-1:0]     i2c_scl_stretch,

  // ---- Data channel: local initiator ----
  input  logic        ini_req,
  input  logic        ini_write,
  input  logic [31:0] ini_addr,
  input  logic [31:0] ini_wdata,
  input  logic [3:0]  ini_be,
  input  logic [7:0]  ini_tag,
  output logic        ini_ack,
  output logic        ini_cpl,
  output logic [31:0] ini_rdata,
  output logic [3:0]  ini_status,
  // ---- Data channel: Avalon-MM master (target) ----
  output logic [31:0] avm_address,
  output logic        avm_read,
  output logic        avm_write,
  output logic [31:0] avm_writedata,
  output logic [3:0]  avm_byteenable,
  input  logic [31:0] avm_readdata,
  input  logic        avm_waitrequest,
  input  logic        avm_readdatavalid,

  // ---- CSR (BMC) access ----
  input  logic [7:0]  csr_addr,
  input  logic        csr_read,
  input  logic        csr_write,
  input  logic [31:0] csr_wdata,
  output logic [31:0] csr_rdata,
  output logic        csr_ready,

  // ---- status ----
  output logic [3:0]  link_state,
  output logic        operational,
  output logic        link_aligned
);
  // module-local copy of the version byte: iverilog mis-infers a 1-bit width for
  // the typed package localparam ltpi_pkg::LTPI_VERSION when wildcard-imported.
  localparam logic [7:0] VERSION = 8'h10;  // BCD 1.0

  // forward declarations (signals referenced before their generating block)
  logic tx_frame_done, tx_frame_start, tx_sym_adv, data_pending;

  // ================= frame RX (symbol-parallel; symbols from the PHY) =========
  logic        rx_fv, rx_crc_ok, rx_code_err, rx_misalign;
  logic [1:0]  rx_comma;
  logic [7:0]  rx_subtype;
  logic [103:0] rx_payload;
  ltpi_frame_rx u_rx (
    .clk, .rst_n, .rx_sym, .rx_sym_valid,
    .frame_valid(rx_fv), .rx_comma(rx_comma),
    .rx_subtype(rx_subtype), .rx_payload(rx_payload),
    .rx_crc_ok(rx_crc_ok), .rx_code_err(rx_code_err), .rx_misalign(rx_misalign)
  );

  // ================= link FSM =================
  logic [2:0]   tx_kind;
  logic [1:0]   fsm_comma_sel;
  logic [111:0] fsm_train_payload;
  logic         tx_operational, csr_operational;
  logic [3:0]   sel_speed_code, remote_state;
  logic         sel_ddr;
  logic [63:0]  applied_caps;
  logic [15:0]  remote_speed_cap, remote_platform_w;
  logic [63:0]  remote_caps;
  logic [7:0]   remote_version_w;
  // CSR-driven control + capabilities
  logic [15:0]  csr_speed_cap;
  logic [63:0]  csr_caps;
  logic [15:0]  csr_platform;
  logic         csr_sw_reset, csr_retrain, csr_auto, csr_trig, csr_dreset;
  logic [6:0]   csr_i2c_reset;
  // FSM event pulses
  logic ev_crc, ev_lost, ev_comma, ev_speedto, ev_cfgto, ev_align;
  logic ev_rxd, ev_rxs, ev_rxa, ev_rxc, ev_txd, ev_txs, ev_txa, ev_txc, ev_orx, ev_otx;

  ltpi_link_fsm #(.ROLE(ROLE), .ADVERTISE_CYCLES(ADVERTISE_CYCLES)) u_fsm (
    .clk, .rst_n,
    .rx_frame_valid(rx_fv), .rx_comma(rx_comma), .rx_subtype(rx_subtype),
    .rx_crc_ok(rx_crc_ok), .rx_misalign(rx_misalign), .rx_aligned(rx_aligned),
    .rx_payload(rx_payload), .tx_frame_done(tx_frame_done),
    .local_speed_cap(csr_speed_cap), .local_caps(csr_caps),
    .local_platform(csr_platform), .local_version(VERSION),
    .cfg_auto(csr_auto), .cfg_trigger(csr_trig),
    .soft_reset(csr_sw_reset), .retrain(csr_retrain),
    .tx_kind(tx_kind), .tx_comma_sel(fsm_comma_sel),
    .tx_train_payload(fsm_train_payload), .tx_operational(tx_operational),
    .link_state(link_state), .remote_state(remote_state),
    .sel_speed_code(sel_speed_code), .sel_ddr(sel_ddr),
    .link_aligned(link_aligned), .operational(operational), .realign(realign),
    .speed_change(speed_change), .op_speed(op_speed), .op_ddr(op_ddr),
    .applied_caps(applied_caps), .remote_speed_cap(remote_speed_cap),
    .remote_caps(remote_caps), .remote_platform(remote_platform_w),
    .remote_version(remote_version_w),
    .ev_crc_err(ev_crc), .ev_lost_err(ev_lost), .ev_comma_err(ev_comma),
    .ev_speed_to(ev_speedto), .ev_cfg_to(ev_cfgto), .ev_align_err(ev_align),
    .ev_rx_detect(ev_rxd), .ev_rx_speed(ev_rxs), .ev_rx_adv(ev_rxa), .ev_rx_cfg(ev_rxc),
    .ev_tx_detect(ev_txd), .ev_tx_speed(ev_txs), .ev_tx_adv(ev_txa), .ev_tx_cfg(ev_txc),
    .ev_oper_rx(ev_orx), .ev_oper_tx(ev_otx)
  );

  // ================= channel enables (negotiated caps) =================
  logic gpio_en, i2c_en, uart_en, data_en;
  assign gpio_en = operational & applied_caps[CAP_CH_GPIO];
  assign i2c_en  = operational & applied_caps[CAP_CH_I2C];
  assign uart_en = operational & applied_caps[CAP_CH_UART];
  assign data_en = operational & applied_caps[CAP_CH_DATA];

  // ================= RX frame classification =================
  logic rx_good, rx_io, rx_data;
  assign rx_good = rx_fv & rx_crc_ok & ~rx_misalign;
  assign rx_io   = rx_good & (rx_comma==COMMA_OPER) & (rx_subtype==SUB_IO);
  assign rx_data = rx_good & (rx_comma==COMMA_OPER) & (rx_subtype==SUB_DATA);

  // ================= TX frame counter + sub-frame ticks =================
  // Sub-ticks track the symbol position within the TX frame (symbol-parallel,
  // flow-controlled by the PHY) to give UART 3 oversamples/frame.
  logic [7:0]  tx_frame_cnt;
  logic [3:0]  sym_idx;       // 0..15 symbol position within the TX frame
  logic        sub_tick;
  logic [1:0]  sub_idx;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_frame_cnt <= 8'h0;
      sym_idx <= 4'h0;
    end else begin
      // frame counter: increment when an Operational I/O frame is sent
      if (tx_frame_done && tx_operational && !data_pending)
        tx_frame_cnt <= tx_frame_cnt + 1'b1;
      // symbol position within the current frame
      if (tx_frame_start)   sym_idx <= 4'h0;
      else if (tx_sym_adv)  sym_idx <= sym_idx + 4'h1;
    end
  end
  // 3 sub-ticks per frame for UART oversampling (symbols 2, 7, 12)
  assign sub_tick = tx_sym_adv & ((sym_idx==4'd2)|(sym_idx==4'd7)|(sym_idx==4'd12));
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) sub_idx <= 2'd0;
    else if (tx_frame_start) sub_idx <= 2'd0;
    else if (sub_tick)       sub_idx <= (sub_idx==2'd2) ? 2'd0 : sub_idx + 1'b1;
  end

  // ================= GPIO channel =================
  logic [15:0] gp_tx_ll, gp_tx_nl;
  ltpi_gpio_chan #(.NUM_NL(NUM_NL)) u_gpio (
    .clk, .rst_n, .frame_cnt(tx_frame_cnt),
    .ll_in, .ll_out, .nl_in, .nl_out,
    .tx_ll(gp_tx_ll), .tx_nl(gp_tx_nl),
    .rx_valid(rx_io & gpio_en), .rx_frame_cnt(rx_payload[7:0]),
    .rx_ll(rx_payload[23:8]), .rx_nl(rx_payload[39:24])
  );

  // ================= UART channel =================
  logic [7:0] ua_tx;
  ltpi_uart_chan u_uart (
    .clk, .rst_n, .sub_tick(sub_tick & uart_en), .sub_idx,
    .txd0, .txd1, .rts0, .rts1, .rxd0, .rxd1, .cts0, .cts1,
    .tx_byte(ua_tx),
    .rx_valid(rx_io & uart_en), .rx_byte(rx_payload[47:40])
  );

  // ================= I2C relays =================
  logic [NUM_I2C*4-1:0] i2c_tx_evt;
  // received I2C bytes (bytes 8,9,10 carry links {1,0},{3,2},{5,4})
  logic [23:0] i2c_rx_word;           // up to 6 nibbles
  assign i2c_rx_word = {rx_payload[71:64], rx_payload[63:56], rx_payload[55:48]};
  genvar gi;
  generate
    for (gi=0; gi<NUM_I2C; gi=gi+1) begin : g_i2c
      ltpi_i2c_relay u_i2c (
        .clk, .rst_n, .enable(i2c_en),
        .frame_tick(tx_frame_done),
        .evt_det(i2c_evt_det[gi]), .evt_code(i2c_evt_code[gi*4 +: 4]),
        .regen(i2c_regen[gi]), .regen_code(i2c_regen_code[gi*4 +: 4]),
        .regen_done(i2c_regen_done[gi]), .scl_stretch(i2c_scl_stretch[gi]),
        .tx_evt(i2c_tx_evt[gi*4 +: 4]),
        .rx_evt(i2c_rx_word[gi*4 +: 4]), .rx_valid(rx_io & i2c_en)
      );
    end
  endgenerate

  // ================= Data channel =================
  logic        data_tx_sent;
  logic [7:0]  data_tx_tag;
  logic [79:0] data_tx_pl;
  ltpi_data_chan u_data (
    .clk, .rst_n, .enable(data_en),
    .ini_req, .ini_write, .ini_addr, .ini_wdata, .ini_be, .ini_tag,
    .ini_ack, .ini_cpl, .ini_rdata, .ini_status,
    .avm_address, .avm_read, .avm_write, .avm_writedata, .avm_byteenable,
    .avm_readdata, .avm_waitrequest, .avm_readdatavalid,
    .tx_pending(data_pending), .tx_tag(data_tx_tag), .tx_payload10(data_tx_pl),
    .tx_sent(data_tx_sent),
    .rx_valid(rx_data & data_en), .rx_tag(rx_payload[23:16]),
    .rx_payload10(rx_payload[103:24])
  );
  // a Data frame is sent (and consumed) when, in Operational, a data frame is
  // pending and the current TX frame completes
  assign data_tx_sent = tx_operational & data_en & data_pending & tx_frame_done;

  // ================= I2C TX nibble packing into bytes 8,9,10 =================
  logic [23:0] i2c_tx_word;
  always @(*) begin
    i2c_tx_word = 24'h0;
    for (int k=0;k<NUM_I2C;k++) i2c_tx_word[k*4 +: 4] = i2c_tx_evt[k*4 +: 4];
  end

  // ================= TX payload assembly =================
  logic [111:0] io_payload, data_payload, oper_payload, tx_payload;
  logic [1:0]   tx_comma;

  // I/O frame (bytes 1..14)
  assign io_payload = {
    16'h0000,             // bytes 13,14 OEM reserved
    16'h0000,             // bytes 11,12 OEM reserved
    i2c_tx_word[23:16],   // byte10 = I2C 4&5
    i2c_tx_word[15:8],    // byte9  = I2C 2&3
    i2c_tx_word[7:0],     // byte8  = I2C 0&1
    ua_tx,                // byte7  = UART
    gp_tx_nl[15:8],       // byte6  = NL GPIO1
    gp_tx_nl[7:0],        // byte5  = NL GPIO0
    gp_tx_ll[15:8],       // byte4  = LL GPIO1
    gp_tx_ll[7:0],        // byte3  = LL GPIO0
    tx_frame_cnt,         // byte2  = frame counter
    SUB_IO                // byte1  = subtype
  };
  // Data frame (bytes 1..14)
  assign data_payload = {
    data_tx_pl,           // bytes 5..14 = data payload (80b)
    data_tx_tag,          // byte4  = Tag
    gp_tx_ll[15:8],       // byte3  = LL GPIO1
    gp_tx_ll[7:0],        // byte2  = LL GPIO0
    SUB_DATA              // byte1  = subtype
  };

  logic send_data;
  assign send_data    = tx_operational & data_en & data_pending;
  assign oper_payload = send_data ? data_payload : io_payload;

  // training vs operational
  always @(*) begin
    if (tx_operational) begin
      tx_comma   = COMMA_OPER;
      tx_payload = oper_payload;
    end else begin
      tx_comma   = fsm_comma_sel;
      tx_payload = fsm_train_payload;
    end
  end

  // ================= frame TX (symbol-parallel; symbols to the PHY) ===========
  ltpi_frame_tx u_tx (
    .clk, .rst_n,
    .tx_comma_sel(tx_comma), .tx_payload(tx_payload),
    .tx_sym(tx_sym), .tx_sym_valid(tx_sym_valid), .tx_sym_ready(tx_sym_ready),
    .sym_advance(tx_sym_adv), .frame_start(tx_frame_start), .frame_done(tx_frame_done)
  );

  // ================= CSR =================
  ltpi_csr #(.SPEED_CAP_DEFAULT(SPEED_CAP), .CAPS_DEFAULT(CAPS), .PLATFORM_ID(PLATFORM_ID)) u_csr (
    .clk, .rst_n,
    .csr_addr, .csr_read, .csr_write, .csr_wdata, .csr_rdata, .csr_ready,
    .local_state(link_state), .remote_state(remote_state),
    .speed_code(sel_speed_code), .ddr(sel_ddr), .link_aligned(link_aligned),
    .remote_speed_cap(remote_speed_cap), .local_version(VERSION),
    .remote_version(remote_version_w), .remote_platform(remote_platform_w),
    .remote_caps(remote_caps), .applied_caps(applied_caps),
    .ev_crc_err(ev_crc), .ev_lost_err(ev_lost), .ev_comma_err(ev_comma),
    .ev_speed_to(ev_speedto), .ev_cfg_err(ev_cfgto), .ev_align_err(ev_align),
    .ev_rx_detect(ev_rxd), .ev_rx_speed(ev_rxs), .ev_rx_adv(ev_rxa), .ev_rx_cfg(ev_rxc),
    .ev_tx_detect(ev_txd), .ev_tx_speed(ev_txs), .ev_tx_adv(ev_txa), .ev_tx_cfg(ev_txc),
    .ev_oper_rx(ev_orx), .ev_oper_tx(ev_otx),
    .local_speed_cap(csr_speed_cap), .local_caps(csr_caps), .local_platform(csr_platform),
    .sw_reset(csr_sw_reset), .retrain(csr_retrain), .auto_config(csr_auto),
    .trig_config(csr_trig), .data_reset(csr_dreset), .i2c_reset(csr_i2c_reset)
  );
endmodule
`endif
