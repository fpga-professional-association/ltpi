// =============================================================================
// ltpi_i2c_relay.sv  -  I2C/SMBus event relay for ONE link (spec Sec 2.2.1.3)
//
// Tunnels I2C/SMBus bus conditions as 4-bit events (Table 10) with the
// echo + received handshake (Table 9) and SCL clock-stretching to absorb the
// LTPI round-trip.  The full bus micro-architecture (SDA/SCL edge detection &
// regeneration) is out of LTPI's scope, so the local bus is an abstract
// event interface that a vendor I2C front-end drives:
//
//   detection side:  evt_det (pulse) + evt_code (Start/Stop/Data0/Data1)
//   regeneration:    regen (pulse) + regen_code  -> the front-end recreates it,
//                    asserts regen_done when the bus condition is on the wire
//   scl_stretch:     hold the local SCL low while waiting on the remote
//
// Handshake (sender S, receiver R):
//   S detects event -> sends EVENT continuously, stretches SCL
//   R receives EVENT -> sends ECHO, regenerates on its bus, then sends RECEIVED
//   S sees ECHO (peer got it) then RECEIVED (peer reproduced it) -> releases SCL
//
// Single-owner guarantee (formally checked): at most one of the regen / bus
// drives is active, and a relay only regenerates an event it actually received.
// =============================================================================
`ifndef LTPI_I2C_RELAY_SV
`define LTPI_I2C_RELAY_SV
`include "ltpi_pkg.sv"
import ltpi_pkg::*;

module ltpi_i2c_relay (
  input  logic       clk,
  input  logic       rst_n,
  input  logic       enable,        // channel enabled (from negotiated caps)
  input  logic       frame_tick,    // 1 pulse per I/O frame (event cadence)

  // ---- abstract local bus interface ----
  input  logic       evt_det,       // local bus event detected (pulse)
  input  logic [3:0] evt_code,      // Start/Stop/Data0/Data1 (ltpi_pkg::i2c_evt_e)
  output logic       regen,         // regenerate a remote event on the local bus
  output logic [3:0] regen_code,
  input  logic       regen_done,    // front-end finished regenerating
  output logic       scl_stretch,   // hold local SCL low while waiting

  // ---- LTPI event nibble (this link) ----
  output logic [3:0] tx_evt,
  input  logic [3:0] rx_evt,
  input  logic       rx_valid       // good I/O frame carrying rx_evt
);
  typedef enum logic [2:0] {
    S_IDLE,        // bus idle / sending Idle
    S_SEND,        // local event captured: send EVENT, wait ECHO then RECEIVED
    S_SEND_WAIT,   //   ECHO seen, waiting for RECEIVED
    R_ECHO,        // remote event captured: send ECHO + regenerate
    R_RECEIVED     // regenerated: send RECEIVED until a new event
  } state_e;

  state_e     state;
  logic [3:0] cur_evt;     // event being sent (S) / echoed (R)
  logic [3:0] rx_l;        // registered rx_evt for edge use

  // helpers to classify received events
  function automatic logic is_event(input logic [3:0] e);
    is_event = (e==I2C_START)||(e==I2C_STOP)||(e==I2C_DATA0)||(e==I2C_DATA1);
  endfunction
  function automatic logic [3:0] echo_of(input logic [3:0] e);
    case (e)
      I2C_START: echo_of = I2C_START_ECHO;
      I2C_STOP:  echo_of = I2C_STOP_ECHO;
      I2C_DATA0: echo_of = I2C_DATA0_ECHO;
      I2C_DATA1: echo_of = I2C_DATA1_ECHO;
      default:   echo_of = I2C_IDLE;
    endcase
  endfunction
  function automatic logic [3:0] received_of(input logic [3:0] e);
    case (e)
      I2C_START: received_of = I2C_START_RCVD;
      I2C_STOP:  received_of = I2C_STOP_RCVD;
      default:   received_of = I2C_DATA_RCVD;   // data bits
    endcase
  endfunction
  function automatic logic is_echo_of(input logic [3:0] r, input logic [3:0] e);
    is_echo_of = (r == echo_of(e));
  endfunction
  function automatic logic is_received(input logic [3:0] r);
    is_received = (r==I2C_START_RCVD)||(r==I2C_STOP_RCVD)||(r==I2C_DATA_RCVD);
  endfunction

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state    <= S_IDLE;
      cur_evt  <= I2C_IDLE;
      regen    <= 1'b0;
      regen_code <= I2C_IDLE;
      rx_l     <= I2C_IDLE;
    end else begin
      regen <= 1'b0;                 // default: 1-cycle regen pulse
      if (rx_valid) rx_l <= rx_evt;
      if (!enable) begin
        state <= S_IDLE; cur_evt <= I2C_IDLE;
      end else begin
        case (state)
          // ---- idle: a local detect or a remote event starts an exchange ----
          S_IDLE: begin
            if (evt_det) begin
              cur_evt <= evt_code;
              state   <= S_SEND;
            end else if (rx_valid && is_event(rx_evt)) begin
              cur_evt    <= rx_evt;
              regen      <= 1'b1;     // recreate on local bus
              regen_code <= rx_evt;
              state      <= R_ECHO;
            end
          end
          // ---- sender: drive EVENT, wait for ECHO ----
          S_SEND: if (rx_valid && is_echo_of(rx_evt, cur_evt)) state <= S_SEND_WAIT;
          // ---- sender: ECHO seen, wait for RECEIVED, then done ----
          S_SEND_WAIT: if (rx_valid && is_received(rx_evt)) begin
            state   <= S_IDLE;
            cur_evt <= I2C_IDLE;
          end
          // ---- receiver: send ECHO and regenerate, then send RECEIVED ----
          R_ECHO: if (regen_done) state <= R_RECEIVED;
          // ---- receiver: send RECEIVED until a new event arrives ----
          R_RECEIVED: if (rx_valid && is_event(rx_evt) && rx_evt != cur_evt) begin
            cur_evt    <= rx_evt;
            regen      <= 1'b1;
            regen_code <= rx_evt;
            state      <= R_ECHO;
          end else if (rx_valid && rx_evt == I2C_IDLE) begin
            state   <= S_IDLE;
            cur_evt <= I2C_IDLE;
          end
        endcase
      end
    end
  end

  // ---- transmitted event for this link ----
  always @(*) begin
    case (state)
      S_SEND, S_SEND_WAIT: tx_evt = cur_evt;             // EVENT
      R_ECHO:              tx_evt = echo_of(cur_evt);    // ECHO
      R_RECEIVED:          tx_evt = received_of(cur_evt);// RECEIVED
      default:             tx_evt = I2C_IDLE;
    endcase
    if (!enable) tx_evt = I2C_IDLE;
  end

  // stretch the local SCL whenever an exchange is in flight
  assign scl_stretch = enable & (state != S_IDLE);

`ifdef FORMAL
  logic fpv = 1'b0;
  always_ff @(posedge clk) fpv <= 1'b1;
  always @(posedge clk) begin
    // disabled relay is fully quiescent (single-owner: never drives the bus/link)
    if (!enable) begin
      a_dis_idle:    assert (tx_evt == I2C_IDLE);
      a_dis_nostretch: assert (!scl_stretch);
    end
    // a regenerate is only ever issued for an event we actually received
    if (fpv && regen)
      a_regen_received: assert (is_event(regen_code) && $past(rx_valid));
    // SCL is stretched exactly while an exchange is in flight
    a_stretch_iff: assert (scl_stretch == (enable && state != S_IDLE));
  end
`endif
endmodule
`endif
