// =============================================================================
// ltpi_8b10b.sv  -  IBM 8b/10b encoder + decoder for LTPI (spec Sec 2.6)
//
// Standard 5B/6B + 3B/4B code with running-disparity tracking.  LTPI only uses
// three control symbols, all commas: K28.5 / K28.6 / K28.7 (Table 19).  Data
// codes use the canonical RD- tables; non-balanced RD- sub-blocks all carry
// disparity +2, so a sub-block is complemented exactly when it is non-balanced
// and the current running disparity is positive.
//
// Transmission/bit order: word[9:0] = {a,b,c,d,e,i,f,g,h,j}, 'a' is word[9]
// (first on the wire).  This is the order the (de)serializer shifts.
//
// rd_neg == 1 means current running disparity is -1; rd_neg == 0 means +1.
//
// Verified by formal (formal/ltpi_8b10b.sby): decode(encode(x)) == x for all
// 256 data bytes and the 3 K-codes in both disparities; encoded word disparity
// is always in {-2,0,+2} and tracked correctly; comma symbols decode as K.
// =============================================================================
`ifndef LTPI_8B10B_SV
`define LTPI_8B10B_SV

// ---- shared tables (compilation-unit-scope automatic functions) -------------

// 5B/6B RD- code for the low 5 data bits; abcdei in [5:0] with a = bit5.
function automatic logic [5:0] enc6b(input logic [4:0] v);
  case (v)
    5'd0 : enc6b = 6'b100111; 5'd1 : enc6b = 6'b011101;
    5'd2 : enc6b = 6'b101101; 5'd3 : enc6b = 6'b110001;
    5'd4 : enc6b = 6'b110101; 5'd5 : enc6b = 6'b101001;
    5'd6 : enc6b = 6'b011001; 5'd7 : enc6b = 6'b111000;
    5'd8 : enc6b = 6'b111001; 5'd9 : enc6b = 6'b100101;
    5'd10: enc6b = 6'b010101; 5'd11: enc6b = 6'b110100;
    5'd12: enc6b = 6'b001101; 5'd13: enc6b = 6'b101100;
    5'd14: enc6b = 6'b011100; 5'd15: enc6b = 6'b010111;
    5'd16: enc6b = 6'b011011; 5'd17: enc6b = 6'b100011;
    5'd18: enc6b = 6'b010011; 5'd19: enc6b = 6'b110010;
    5'd20: enc6b = 6'b001011; 5'd21: enc6b = 6'b101010;
    5'd22: enc6b = 6'b011010; 5'd23: enc6b = 6'b111010;
    5'd24: enc6b = 6'b110011; 5'd25: enc6b = 6'b100110;
    5'd26: enc6b = 6'b010110; 5'd27: enc6b = 6'b110110;
    5'd28: enc6b = 6'b001110; 5'd29: enc6b = 6'b101110;
    5'd30: enc6b = 6'b011110; 5'd31: enc6b = 6'b101011;
    default: enc6b = 6'b100111;
  endcase
endfunction

// balanced (disparity 0) flag for the 5B/6B sub-block
function automatic logic bal6(input logic [4:0] v);
  case (v)
    5'd3,5'd5,5'd6,5'd7,5'd9,5'd10,5'd11,5'd12,5'd13,5'd14,
    5'd17,5'd18,5'd19,5'd20,5'd21,5'd22,5'd25,5'd26,5'd28: bal6 = 1'b1;
    default: bal6 = 1'b0;  // 0,1,2,4,8,15,16,23,24,27,29,30,31  (disp +2 in RD-)
  endcase
endfunction

// 3B/4B RD- code for the high 3 data bits; fghj in [3:0] with f = bit3.
function automatic logic [3:0] enc4b(input logic [2:0] v);
  case (v)
    3'd0: enc4b = 4'b1011; 3'd1: enc4b = 4'b1001;
    3'd2: enc4b = 4'b0101; 3'd3: enc4b = 4'b1100;
    3'd4: enc4b = 4'b1101; 3'd5: enc4b = 4'b1010;
    3'd6: enc4b = 4'b0110; 3'd7: enc4b = 4'b1110;
    default: enc4b = 4'b1011;
  endcase
endfunction

function automatic logic bal4(input logic [2:0] v);
  case (v)
    3'd1,3'd2,3'd3,3'd5,3'd6: bal4 = 1'b1;
    default: bal4 = 1'b0;   // 0,4,7  (disp +2 in RD-)
  endcase
endfunction

// Hardcoded comma words {a..j}.  RD- form (disp >= 0) and RD+ form.
// idx: 0 -> K28.5, 1 -> K28.6, 2 -> K28.7
function automatic logic [9:0] kword_minus(input logic [1:0] idx);
  case (idx)
    2'd0: kword_minus = 10'b0011111010; // K28.5
    2'd1: kword_minus = 10'b0011110110; // K28.6
    default: kword_minus = 10'b0011111000; // K28.7
  endcase
endfunction

// population count helper for 10-bit words
function automatic int unsigned ones10(input logic [9:0] w);
  ones10 = 0;
  for (int i=0;i<10;i++) ones10 += w[i];
endfunction

// Combinational 8b/10b encode given current running disparity (cur_rd: 1 = -1).
// Returns {next_rd, symbol[9:0]}.  ctrl=1 selects comma K28.5/6/7 via d[1:0].
function automatic logic [10:0] enc8b10b(input logic cur_rd, input logic [7:0] d,
                                         input logic ctrl);
  logic [5:0] six_m, six;
  logic [3:0] four_m, four;
  logic       b6, b4, rd1, nrd;
  logic [9:0] w;
  begin
    if (ctrl) begin
      w = cur_rd ? kword_minus(d[1:0]) : ~kword_minus(d[1:0]);
      if      (ones10(w) > 5) nrd = 1'b0;
      else if (ones10(w) < 5) nrd = 1'b1;
      else                    nrd = cur_rd;
      enc8b10b = {nrd, w};
    end else begin
      six_m  = enc6b(d[4:0]); b6 = bal6(d[4:0]);
      six    = (b6 | cur_rd) ? six_m : ~six_m;
      rd1    = cur_rd ^ ~b6;
      four_m = enc4b(d[7:5]); b4 = bal4(d[7:5]);
      four   = (b4 | rd1) ? four_m : ~four_m;
      nrd    = rd1 ^ ~b4;
      enc8b10b = {nrd, six, four};
    end
  end
endfunction

// Reverse 5B/6B (gated complement); returns {ok, value[4:0]}.
function automatic logic [5:0] dec6b(input logic [5:0] s);
  logic [4:0] r; logic found;
  begin
    r = 5'd0; found = 1'b0;
    for (int v=0; v<32; v++)
      if (s == enc6b(v[4:0]) || (!bal6(v[4:0]) && s == ~enc6b(v[4:0])))
        begin r = v[4:0]; found = 1'b1; end
    dec6b = {found, r};
  end
endfunction

// Reverse 3B/4B (gated complement); returns {ok, value[2:0]}.
function automatic logic [3:0] dec4b(input logic [3:0] s);
  logic [2:0] r; logic found;
  begin
    r = 3'd0; found = 1'b0;
    for (int v=0; v<8; v++)
      if (s == enc4b(v[2:0]) || (!bal4(v[2:0]) && s == ~enc4b(v[2:0])))
        begin r = v[2:0]; found = 1'b1; end
    dec4b = {found, r};
  end
endfunction

// Combinational 8b/10b decode.  Returns {comma, k, code_err, data[7:0]}.
function automatic logic [10:0] dec8b10b(input logic [9:0] symbol);
  logic [4:0] lo5; logic ok6;
  logic [2:0] hi3; logic ok4;
  logic [1:0] kidx; logic kmatch;
  logic kf, cm, ce; logic [7:0] db;
  begin
    {ok6, lo5} = dec6b(symbol[9:4]);
    {ok4, hi3} = dec4b(symbol[3:0]);
    kmatch = 1'b0; kidx = 2'd0;
    for (int i=0;i<3;i++)
      if (symbol == kword_minus(i[1:0]) || symbol == ~kword_minus(i[1:0]))
        begin kmatch = 1'b1; kidx = i[1:0]; end
    if (kmatch) begin
      kf = 1'b1; cm = 1'b1; ce = 1'b0;
      db = (kidx==2'd0) ? 8'hBC : (kidx==2'd1) ? 8'hDC : 8'hFC;
    end else begin
      kf = 1'b0; cm = 1'b0; ce = ~(ok6 & ok4); db = {hi3, lo5};
    end
    dec8b10b = {cm, kf, ce, db};
  end
endfunction

// ============================================================================
//  Encoder
// ============================================================================
module ltpi_8b10b_enc (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        en,        // advance one symbol
  input  logic [7:0]  data,      // data byte, or comma index when k=1
  input  logic        k,         // 1 = control comma (data[1:0] selects K28.5/6/7)
  output logic [9:0]  symbol,
  output logic        rd_neg     // running disparity AFTER this symbol (-1 if 1)
);
  logic rd;  // 1 = neg disparity (-1), 0 = pos (+1)

  logic [10:0] enc_c;
  logic [9:0]  sym_c;
  logic        nrd_c;
  always_comb begin
    enc_c = enc8b10b(rd, data, k);
    sym_c = enc_c[9:0];
    nrd_c = enc_c[10];
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rd     <= 1'b1;       // start RD = -1 (neg)
      symbol <= kword_minus(2'd0);
      rd_neg <= 1'b1;
    end else if (en) begin
      symbol <= sym_c;
      rd     <= nrd_c;
      rd_neg <= nrd_c;
    end
  end
endmodule

// ============================================================================
//  Decoder (combinational symbol -> byte; disparity-agnostic)
// ============================================================================
module ltpi_8b10b_dec (
  input  logic [9:0]  symbol,
  output logic [7:0]  data,
  output logic        k,           // control symbol
  output logic        comma,       // K28.5/6/7 comma detected
  output logic        code_err     // symbol not a legal data or used-K code
);
  logic [10:0] d;
  always_comb begin
    d        = dec8b10b(symbol);   // {comma, k, code_err, data}
    comma    = d[10];
    k        = d[9];
    code_err = d[8];
    data     = d[7:0];
  end
endmodule
`endif
