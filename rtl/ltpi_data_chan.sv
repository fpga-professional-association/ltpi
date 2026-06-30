// =============================================================================
// ltpi_data_chan.sv  -  Data channel: Avalon-MM tunneling (spec Sec 2.2.1.4)
//
// Random-access Read/Write tunneling (Tables 12/13).  Each endpoint has:
//   * an INITIATOR interface (local logic / BMC issues a remote access), which
//     emits a Read/Write Request frame with a Tag and matches the returned
//     Completion by Tag (duplicate outstanding Tags are dropped — spec Sec 3.1.1.2);
//   * a TARGET interface: an Avalon-MM master that executes a received Request
//     against a local slave and returns a Read/Write Completion.
//
// 10-byte data payload (frame bytes 5..14):
//   [7:0]   = Command (ltpi_pkg::dc_cmd_e)
//   [39:8]  = Address[31:0]
//   [47:40] = {status[3:0], byte-enable[3:0]}
//   [79:48] = Data[31:0]
//
// One request and one completion may be outstanding at a time (sufficient and
// spec-compliant); completions take priority for the shared data-frame slot.
// =============================================================================
`ifndef LTPI_DATA_CHAN_SV
`define LTPI_DATA_CHAN_SV
`include "ltpi_pkg.sv"
import ltpi_pkg::*;

module ltpi_data_chan (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        enable,

  // ---- local initiator interface ----
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

  // ---- local Avalon-MM master (target side) ----
  output logic [31:0] avm_address,
  output logic        avm_read,
  output logic        avm_write,
  output logic [31:0] avm_writedata,
  output logic [3:0]  avm_byteenable,
  input  logic [31:0] avm_readdata,
  input  logic        avm_waitrequest,
  input  logic        avm_readdatavalid,

  // ---- LTPI data-frame interface (to/from core) ----
  output logic        tx_pending,    // a data frame is ready to send
  output logic [7:0]  tx_tag,        // frame byte4
  output logic [79:0] tx_payload10,  // frame bytes 5..14
  input  logic        tx_sent,       // core has sent the pending data frame
  input  logic        rx_valid,      // good data frame received
  input  logic [7:0]  rx_tag,
  input  logic [79:0] rx_payload10
);
  // received payload fields
  logic [7:0]  rx_cmd;
  logic [31:0] rx_addr, rx_data;
  logic [3:0]  rx_be;
  assign rx_cmd  = rx_payload10[7:0];
  assign rx_addr = rx_payload10[39:8];
  assign rx_be   = rx_payload10[43:40];
  assign rx_data = rx_payload10[79:48];

  // ---- initiator state ----
  logic        ini_busy;       // request outstanding
  logic [7:0]  ini_otag;
  logic        ini_req_tx;     // request frame waiting to be sent
  logic [79:0] ini_tx_pl;
  logic [7:0]  ini_tx_tag;

  // ---- target state ----
  typedef enum logic [1:0] {T_IDLE, T_RUN, T_CPL} tstate_e;
  tstate_e     tstate;
  logic [7:0]  t_tag;
  logic [31:0] t_addr, t_rdata, t_wdata;
  logic [3:0]  t_be;
  logic        t_is_write;
  logic        t_cpl_tx;       // completion frame waiting to be sent
  logic [79:0] t_tx_pl;

  // Avalon master drive (combinational from target state)
  always @(*) begin
    avm_address    = t_addr;
    avm_writedata  = t_wdata;
    avm_byteenable = t_be;
    avm_read       = (tstate==T_RUN) & ~t_is_write;
    avm_write      = (tstate==T_RUN) &  t_is_write;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ini_busy <= 1'b0; ini_otag <= '0; ini_req_tx <= 1'b0;
      ini_tx_pl <= '0; ini_tx_tag <= '0;
      ini_ack <= 1'b0; ini_cpl <= 1'b0; ini_rdata <= '0; ini_status <= '0;
      tstate <= T_IDLE; t_tag <= '0; t_addr <= '0; t_rdata <= '0;
      t_wdata <= '0; t_be <= '0; t_is_write <= 1'b0; t_cpl_tx <= 1'b0; t_tx_pl <= '0;
    end else begin
      ini_ack <= 1'b0;
      ini_cpl <= 1'b0;

      // -------- initiator: issue request --------
      if (enable && ini_req && !ini_busy && !ini_req_tx) begin
        ini_busy   <= 1'b1;
        ini_otag   <= ini_tag;
        ini_req_tx <= 1'b1;
        ini_tx_tag <= ini_tag;
        ini_tx_pl  <= {ini_wdata, 4'h0, ini_be,
                       ini_addr, (ini_write ? DC_WRITE_REQ : DC_READ_REQ)};
        ini_ack    <= 1'b1;
      end else if (enable && ini_req && ini_busy && ini_tag == ini_otag) begin
        // duplicate outstanding tag -> drop (no ack)
      end

      // -------- initiator: match completion --------
      if (rx_valid && (rx_cmd==DC_READ_CPL || rx_cmd==DC_WRITE_CPL)
                   && ini_busy && rx_tag==ini_otag) begin
        ini_busy   <= 1'b0;
        ini_cpl    <= 1'b1;
        ini_rdata  <= rx_data;
        ini_status <= rx_be;            // status nibble carried in BE field
      end

      // -------- target: accept request, run Avalon, build completion --------
      case (tstate)
        T_IDLE: begin
          if (enable && rx_valid && (rx_cmd==DC_READ_REQ || rx_cmd==DC_WRITE_REQ) && !t_cpl_tx) begin
            t_tag      <= rx_tag;
            t_addr     <= rx_addr;
            t_be       <= rx_be;
            t_wdata    <= rx_data;
            t_is_write <= (rx_cmd==DC_WRITE_REQ);
            tstate     <= T_RUN;
          end
        end
        T_RUN: begin
          if (t_is_write) begin
            if (!avm_waitrequest) begin
              t_tx_pl  <= {32'h0, DC_STATUS_OK, t_be, t_addr, DC_WRITE_CPL};
              t_cpl_tx <= 1'b1;
              tstate   <= T_CPL;
            end
          end else begin
            if (avm_readdatavalid) begin
              t_rdata  <= avm_readdata;
              t_tx_pl  <= {avm_readdata, DC_STATUS_OK, t_be, t_addr, DC_READ_CPL};
              t_cpl_tx <= 1'b1;
              tstate   <= T_CPL;
            end
          end
        end
        T_CPL: if (!t_cpl_tx) tstate <= T_IDLE;   // completion frame consumed
      endcase

      // -------- shared data-frame slot: completion has priority --------
      if (tx_sent) begin
        if (t_cpl_tx)        t_cpl_tx   <= 1'b0;
        else if (ini_req_tx) ini_req_tx <= 1'b0;
      end
    end
  end

  // present the pending data frame (completion first)
  always @(*) begin
    if (t_cpl_tx)        begin tx_pending = 1'b1; tx_tag = t_tag;     tx_payload10 = t_tx_pl;  end
    else if (ini_req_tx) begin tx_pending = 1'b1; tx_tag = ini_tx_tag; tx_payload10 = ini_tx_pl; end
    else                 begin tx_pending = 1'b0; tx_tag = 8'h0;      tx_payload10 = 80'h0;    end
  end

`ifdef FORMAL
  logic fpv = 1'b0;
  always_ff @(posedge clk) fpv <= 1'b1;
  initial assume (!rst_n);   // start from a clean reset (deterministic regs)
  always @(posedge clk) begin
    // Avalon master never reads and writes simultaneously (single transaction)
    a_rw_excl: assert (!(avm_read && avm_write));
    // target only drives Avalon while running a received request
    if (avm_read || avm_write) a_avm_run: assert (tstate == T_RUN);
    if (fpv && $past(rst_n) && rst_n) begin
      // a completion to the initiator only after a request was outstanding
      if (ini_cpl) a_cpl_needs_req: assert ($past(ini_busy));
      // a completion clears the outstanding request, unless a new request is
      // accepted the same cycle (which re-arms busy with a fresh Tag)
      if ($past(ini_cpl) &&
          !($past(enable) && $past(ini_req) && !$past(ini_busy) && !$past(ini_req_tx)))
        a_cpl_clears: assert (!ini_busy);
    end
  end
`endif
endmodule
`endif
