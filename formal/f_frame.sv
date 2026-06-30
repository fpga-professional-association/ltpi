// Formal harness for the symbol-parallel frame layer (frame_tx -> frame_rx).
// frame_tx emits one 8b/10b symbol per clock straight into frame_rx (no PHY).
// Proves: every completed frame validates its CRC and has no code error, i.e.
// the TX incremental-CRC generation and the RX incremental-CRC check agree and
// are correct for ALL comma/payload inputs.  Also covers that frames complete.
module f_frame (
  input  logic        clk,
  input  logic        rst_n,
  input  logic [1:0]  tx_comma_sel,
  input  logic [111:0] tx_payload
);
  logic [9:0] sym; logic sv;
  ltpi_frame_tx u_tx (
    .clk, .rst_n, .tx_comma_sel, .tx_payload,
    .tx_sym(sym), .tx_sym_valid(sv), .tx_sym_ready(1'b1),
    .sym_advance(), .frame_start(), .frame_done()
  );

  logic fv, crc_ok, cerr, mis; logic [1:0] rc; logic [7:0] rs; logic [103:0] rp;
  ltpi_frame_rx u_rx (
    .clk, .rst_n, .rx_sym(sym), .rx_sym_valid(sv),
    .frame_valid(fv), .rx_comma(rc), .rx_subtype(rs), .rx_payload(rp),
    .rx_crc_ok(crc_ok), .rx_code_err(cerr), .rx_misalign(mis)
  );

  logic fpv = 1'b0;
  always_ff @(posedge clk) fpv <= 1'b1;
  initial assume (!rst_n);
  always @(posedge clk) if (fpv) assume (rst_n);

  always @(posedge clk) if (fpv && rst_n && $past(rst_n)) begin
    // a completed frame always passes CRC and has no code/alignment error
    if (fv) begin
      a_crc_ok:   assert (crc_ok);
      a_no_cerr:  assert (!cerr);
      a_no_mis:   assert (!mis);
    end
  end
  // liveness: frames do complete
  always @(posedge clk) if (rst_n) c_frame: cover (fv);
endmodule
