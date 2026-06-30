// =============================================================================
// ltpi_crc8.sv  -  CRC-8 for LTPI frames (spec Sec 2.4)
//
//   Polynomial : x^8 + x^2 + x^1 + 1   (0x07)
//   Init value : 0x00
//   Coverage   : all payload bytes after the comma symbol (frame bytes 1..14),
//                MSB-first, result placed in frame byte 15.
//
// Pure combinational.  Exposes:
//   - crc8_step()   : fold one byte into a running CRC
//   - this module   : CRC over the packed 14-byte payload (byte1 = LSBytes[7:0])
// =============================================================================
`ifndef LTPI_CRC8_SV
`define LTPI_CRC8_SV
`include "ltpi_pkg.sv"

// One-byte CRC-8 step, MSB-first, poly 0x07.
function automatic logic [7:0] crc8_step(input logic [7:0] crc, input logic [7:0] data);
  logic [7:0] c;
  begin
    c = crc;
    for (int i = 7; i >= 0; i--) begin
      logic fb;
      fb = c[7] ^ data[i];
      c  = {c[6:0], 1'b0};
      if (fb) c = c ^ 8'h07;
    end
    crc8_step = c;
  end
endfunction

module ltpi_crc8 #(
  parameter int N = ltpi_pkg::PAYLOAD_BYTES   // number of bytes covered
) (
  input  logic [N*8-1:0] data,   // data[7:0] is the FIRST byte folded (frame byte 1)
  output logic [7:0]     crc
);
  // always @(*) (not always_comb): iverilog mis-schedules an always_comb that
  // calls a function when `data` is driven by an internal reg; equivalent for
  // synthesis/formal.
  logic [7:0] c;
  always @(*) begin
    c = 8'h00;
    for (int b = 0; b < N; b++)
      c = crc8_step(c, data[b*8 +: 8]);
    crc = c;
  end
endmodule
`endif
