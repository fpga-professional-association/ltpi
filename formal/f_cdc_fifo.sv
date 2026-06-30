// FIFO safety (single-clock instance; the gray-pointer logic is identical in the
// two-clock build, exercised across domains in simulation): full and empty are
// never asserted at the same time -> the flag logic is consistent and the FIFO
// can never simultaneously block writes and reads (a data-integrity guard).
module f_cdc_fifo #(parameter W=10, parameter AW=3) (
  input logic clk, rst_n, wr_en, rd_en, input logic [W-1:0] wdata
);
  logic wfull, rempty; logic [W-1:0] rdata;
  ltpi_cdc_fifo #(.W(W), .AW(AW)) u (
    .wclk(clk), .wrst_n(rst_n), .wr_en(wr_en), .wdata(wdata), .wfull(wfull),
    .rclk(clk), .rrst_n(rst_n), .rd_en(rd_en), .rdata(rdata), .rempty(rempty)
  );
  initial assume (!rst_n);
  logic fpv=1'b0; always_ff @(posedge clk) fpv<=1'b1;
  always @(posedge clk) if (fpv && rst_n) a_notboth: assert (!(wfull && rempty));
endmodule
