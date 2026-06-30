// =============================================================================
// ltpi_frame_rx.sv  -  consume one 8b/10b SYMBOL per clock, decode, frame on the
// comma K-code, and CRC-check (symbol-parallel; bit deserialization + comma
// bit-alignment are done by the PHY SERDES, which presents aligned 10-bit
// symbols through the RX CDC FIFO).
//
//   * 8b/10b decode is PIPELINED one stage (registered) to keep the 32-way
//     reverse-lookup off the frame-assembly critical path.
//   * CRC-8 is INCREMENTAL (one crc8_step per received byte); crc_run holds
//     CRC(bytes 1..14) by the time byte15 (the CRC) arrives, so there is no
//     112-deep combinational fold -> meets the 80 MHz parallel-symbol clock.
//
// Emits one frame_valid pulse per complete 16-symbol frame with the decoded
// comma class, subtype, 13-byte channel payload (bytes 2..14), and
// crc_ok / code_err / misalign status (spec Sec 3, 2.4).
// =============================================================================
`ifndef LTPI_FRAME_RX_SV
`define LTPI_FRAME_RX_SV
`include "ltpi_pkg.sv"
`include "ltpi_8b10b.sv"
`include "ltpi_crc8.sv"

module ltpi_frame_rx (
  input  logic         clk,
  input  logic         rst_n,
  input  logic [9:0]   rx_sym,        // aligned 8b/10b symbol from the PHY
  input  logic         rx_sym_valid,  // 1 when rx_sym is a fresh symbol
  output logic         frame_valid,   // 1-cycle pulse per complete frame
  output logic [1:0]   rx_comma,      // ltpi_pkg::comma_e of byte0
  output logic [7:0]   rx_subtype,    // byte1
  output logic [103:0] rx_payload,    // bytes 2..14 (13 bytes), [7:0]=byte2
  output logic         rx_crc_ok,
  output logic         rx_code_err,
  output logic         rx_misalign
);
  // ---- pipeline stage 1: registered 8b/10b decode ----
  logic        dval;
  logic        d_comma, d_k, d_cerr;
  logic [7:0]  d_byte;
  logic [10:0] dec_c;
  always @(*) dec_c = dec8b10b(rx_sym);   // {comma, k, code_err, data}
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dval <= 1'b0; d_comma <= 1'b0; d_k <= 1'b0; d_cerr <= 1'b0; d_byte <= 8'h0;
    end else begin
      dval    <= rx_sym_valid;
      d_comma <= dec_c[10];
      d_k     <= dec_c[9];
      d_cerr  <= dec_c[8];
      d_byte  <= dec_c[7:0];
    end
  end

  // ---- stage 2: frame assembly + incremental CRC ----
  logic [3:0]   bidx;         // 0 = idle/comma; 1..14 = data byte; 15 = CRC byte
  logic         collecting;
  logic [1:0]   comma_cls;
  logic [111:0] rbuf;         // bytes 1..14; [7:0]=byte1
  logic         cerr_acc;
  logic [7:0]   crc_run;      // CRC over bytes 1..14

  logic emit, mis;
  // emit when the CRC byte (bidx==15) is processed; misalign on a mid-frame comma
  assign emit = dval & collecting & ~d_comma & (bidx == 4'd15);
  assign mis  = dval & d_comma & collecting & (bidx != 4'd0);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bidx <= 4'd0; collecting <= 1'b0; comma_cls <= 2'd0;
      rbuf <= '0; cerr_acc <= 1'b0; crc_run <= 8'h00;
    end else if (dval) begin
      if (d_comma) begin
        comma_cls  <= ltpi_pkg::comma_decode(d_k, d_byte);
        bidx       <= 4'd1;
        collecting <= 1'b1;
        cerr_acc   <= 1'b0;
        crc_run    <= 8'h00;
      end else if (collecting) begin
        if (bidx <= 4'd14) begin
          rbuf[(bidx-4'd1)*8 +: 8] <= d_byte;
          crc_run  <= crc8_step(crc_run, d_byte);
          cerr_acc <= cerr_acc | d_cerr;
          bidx     <= bidx + 4'd1;
        end else begin            // bidx == 15: CRC byte -> frame complete
          collecting <= 1'b0;
          bidx       <= 4'd0;
        end
      end
    end
  end

  // ---- outputs (combinational at the emit cycle so crc_run/d_byte are coherent;
  //      the consumer latches them on frame_valid) ----
  assign frame_valid = emit;
  assign rx_misalign = mis;
  assign rx_comma    = comma_cls;
  assign rx_subtype  = rbuf[7:0];
  assign rx_payload  = rbuf[111:8];                 // bytes 2..14
  // crc_run already holds CRC(1..14) when the CRC byte (d_byte) is present
  assign rx_crc_ok   = (crc_run == d_byte) & ~cerr_acc & ~d_cerr;
  assign rx_code_err = cerr_acc;
endmodule
`endif
