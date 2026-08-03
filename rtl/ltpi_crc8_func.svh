// CRC-8 over one byte, poly x^8 + x^2 + x + 1 (0x07), init 0, MSB first
// (spec section 2.4). Include INSIDE a module body; yosys does not elaborate
// package-scoped function bodies called from modules.
function automatic logic [7:0] crc8_update(input logic [7:0] crc,
                                           input logic [7:0] data);
    logic [7:0] c;
    integer i;
    begin
        c = crc ^ data;
        for (i = 0; i < 8; i = i + 1)
            c = c[7] ? ((c << 1) ^ 8'h07) : (c << 1);
        crc8_update = c;
    end
endfunction
