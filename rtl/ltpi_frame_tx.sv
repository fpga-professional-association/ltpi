// =============================================================================
// ltpi_frame_tx.sv  -  assemble a 16-symbol LTPI frame, one 8b/10b SYMBOL per
// clock (symbol-parallel; the high-rate bit serialization is done by the PHY
// SERDES, not here).
//
//   byte0      = comma K-code (selected by tx_comma_sel)
//   byte1..14  = tx_payload   (byte1 = Frame Subtype)
//   byte15     = CRC-8 over bytes 1..14 (spec Sec 2.4), computed INCREMENTALLY
//                (one crc8_step per emitted byte -> 8 XOR levels/clock, not a
//                112-deep combinational fold) so the path meets the 80 MHz
//                parallel-symbol clock (= 400 MHz DDR line / J=10).
//
// Handshake: emits one symbol when (tx_sym_valid & tx_sym_ready).  The link
// always has a frame to send, so tx_sym_valid is held high; the PHY's TX FIFO
// back-pressures via tx_sym_ready.  Frame inputs are latched at the boundary.
// =============================================================================
`ifndef LTPI_FRAME_TX_SV
`define LTPI_FRAME_TX_SV
`include "ltpi_pkg.sv"
`include "ltpi_8b10b.sv"
`include "ltpi_crc8.sv"

module ltpi_frame_tx (
  input  logic                clk,
  input  logic                rst_n,
  input  logic [1:0]          tx_comma_sel,   // ltpi_pkg::comma_e
  input  logic [111:0]        tx_payload,     // 14 bytes: [7:0]=subtype(byte1)..byte14
  output logic [9:0]          tx_sym,         // current 8b/10b symbol
  output logic                tx_sym_valid,
  input  logic                tx_sym_ready,   // PHY TX FIFO can accept a symbol
  output logic                sym_advance,    // pulses as each symbol is emitted
  output logic                frame_start,    // pulses as byte0 (comma) is emitted
  output logic                frame_done      // pulses as byte15 is emitted
);
  logic [111:0] pay_q;     // [7:0]=byte1 ... [111:104]=byte14
  logic [1:0]   csel;
  logic [3:0]   byte_idx;  // 0..15
  logic         rd;        // running disparity (1 = -1)
  logic [7:0]   crc_run;   // incremental CRC over bytes 1..14

  // current byte + control flag for the encoder (constant-select mux)
  logic [7:0] cur_byte;
  logic       cur_k;
  always @(*) begin
    cur_k    = (byte_idx == 4'd0);
    cur_byte = {6'b0, csel};                 // byte0 = comma index
    for (int b = 1; b < 15; b++)
      if (byte_idx == b[3:0]) cur_byte = pay_q[(b-1)*8 +: 8];
    if (byte_idx == 4'd15) cur_byte = crc_run;  // byte15 = incremental CRC
  end

  logic [10:0] enc;
  always @(*) enc = enc8b10b(rd, cur_byte, cur_k);
  assign tx_sym       = enc[9:0];
  assign tx_sym_valid = 1'b1;                 // continuous frame stream

  logic advance;
  assign advance = tx_sym_valid & tx_sym_ready;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      byte_idx <= 4'd0;
      rd       <= 1'b1;
      pay_q    <= '0;        // async reset loads constants; first frame is a
      csel     <= 2'd0;      // benign zero Detect frame, refreshed at the boundary
      crc_run  <= 8'h00;
    end else if (advance) begin
      rd <= enc[10];
      // incremental CRC: reset on the comma byte, fold bytes 1..14
      crc_run <= (byte_idx == 4'd0)  ? 8'h00 :
                 (byte_idx <= 4'd14) ? crc8_step(crc_run, cur_byte) : crc_run;
      if (byte_idx == 4'd15) begin
        byte_idx <= 4'd0;
        pay_q    <= tx_payload;
        csel     <= tx_comma_sel;
      end else begin
        byte_idx <= byte_idx + 4'd1;
      end
    end
  end

  assign sym_advance = advance;
  assign frame_start = advance & (byte_idx == 4'd0);
  assign frame_done  = advance & (byte_idx == 4'd15);
endmodule
`endif
