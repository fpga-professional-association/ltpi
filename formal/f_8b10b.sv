// Formal harness for the 8b/10b codec (ltpi_8b10b.sv).
// Proves, over ALL inputs (combinational, depth 1):
//   * data round-trip: dec(enc(x)) == x, not flagged K, no code error
//   * comma round-trip: enc(K28.5/6/7) decodes as K + comma
//   * DC balance: every encoded word has 4..6 ones (disparity in {-2,0,+2})
module f_8b10b (
  input logic       cur_rd,
  input logic [7:0] d,
  input logic       ctrl
);
  logic [10:0] e, dec;
  logic [9:0]  sym;
  always_comb begin
    e   = enc8b10b(cur_rd, d, ctrl);
    sym = e[9:0];
    dec = dec8b10b(sym);
  end

  function automatic int ones(input logic [9:0] w);
    ones = 0; for (int i=0;i<10;i++) ones += w[i];
  endfunction

  always_comb begin
    if (!ctrl) begin
      assert (dec[7:0] == d);     // value preserved
      assert (dec[9]  == 1'b0);   // not control
      assert (dec[8]  == 1'b0);   // no code error
    end else begin
      assert (dec[10] == 1'b1);   // comma
      assert (dec[9]  == 1'b1);   // control
    end
    assert (ones(sym) >= 4 && ones(sym) <= 6);   // DC balance
  end
endmodule
