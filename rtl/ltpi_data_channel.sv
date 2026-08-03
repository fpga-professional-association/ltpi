// LTPI Data Channel: random-access read/write tunneling over Default Data
// Frames (spec 2.2.1.4, 3.1.1.2, Table 34). Both roles live in every
// endpoint (full duplex): the REQUESTER turns a local register-style
// request into a Data Frame and waits for the tagged response; the
// COMPLETER turns a received request into a local memory-mapped master
// access (AVMM/APB-style) and returns the tagged response.
//
// Tag rules implemented per spec 3.1.1.2:
//   - single outstanding request (a new local request is held off until
//     the response or error arrives - proven D1)
//   - the response must carry the request's tag (D2/D3)
//   - a CRC-dropped frame simply never arrives; the BMC-visible error
//     path is the DC_CMD_ERROR response (Table 12: "operation won't be
//     completed", status 0h04)
//
// Data Frame payload mapping (payload[103:0] = frame bytes 2..14):
//   byte 4  [23:16] tag        byte 5  [31:24] command
//   bytes 6-9  [63:32] address bytes 10-13 [95:64] data
//   byte 14 [103:96] {4'b0, byte-enables}
import ltpi_pkg::*;

module ltpi_data_channel (
    input  logic clk,
    input  logic rst,
    input  logic link_up,

    // ---------------- local requester port ----------------
    input  logic        req_valid,
    input  logic        req_write,
    input  logic [31:0] req_addr,
    input  logic [31:0] req_wdata,
    input  logic [3:0]  req_byteen,
    output logic        req_ready,     // low while a request is outstanding
    output logic        rsp_valid,     // 1-cycle pulse
    output logic [31:0] rsp_rdata,
    output logic        rsp_error,

    // ---------------- local completer master port ----------------
    output logic        cmp_req,       // level until cmp_done
    output logic        cmp_write,
    output logic [31:0] cmp_addr,
    output logic [31:0] cmp_wdata,
    output logic [3:0]  cmp_byteen,
    input  logic [31:0] cmp_rdata,
    input  logic        cmp_done,      // 1-cycle pulse completes the access

    // ---------------- frame side ----------------
    output logic         dtx_req,      // request one Data Frame slot
    output logic [103:0] dtx_payload,
    input  logic         dtx_grant,    // pulse: that frame was transmitted
    input  logic         drx_valid,    // pulse: CRC-good Data Frame received
    input  logic [103:0] drx_payload
);

    logic rsp_granted, req_granted;   // frame-slot grants (driven below)

    // ---------------- requester ----------------
    logic       out_pend;     // request outstanding (sent or queued)
    logic       out_sent;     // its frame has been transmitted
    logic [7:0] out_tag;
    logic [103:0] req_pl;

    logic [7:0]  rx_cmd, rx_tag;
    logic [31:0] rx_data;
    assign rx_tag  = drx_payload[23:16];
    assign rx_cmd  = drx_payload[31:24];
    assign rx_data = drx_payload[95:64];

    logic rx_is_rsp;
    assign rx_is_rsp = drx_valid
                       && (rx_cmd == DC_CMD_WRITE_RESP
                           || rx_cmd == DC_CMD_READ_RESP
                           || rx_cmd == DC_CMD_ERROR)
                       && rx_tag == out_tag && out_pend && out_sent;

    assign req_ready = !out_pend && link_up;

    always_ff @(posedge clk) begin
        if (rst || !link_up) begin
            out_pend  <= 1'b0;
            out_sent  <= 1'b0;
            out_tag   <= '0;
            rsp_valid <= 1'b0;
        end else begin
            rsp_valid <= 1'b0;
            if (req_valid && req_ready) begin
                out_pend <= 1'b1;
                out_sent <= 1'b0;
                req_pl   <= '0;
                req_pl[23:16] <= out_tag;
                req_pl[31:24] <= req_write ? DC_CMD_WRITE_REQ
                                           : DC_CMD_READ_REQ;
                req_pl[63:32] <= req_addr;
                req_pl[95:64] <= req_wdata;
                req_pl[99:96] <= req_byteen;
            end else if (out_pend && !out_sent && req_granted) begin
                out_sent <= 1'b1;
            end else if (rx_is_rsp) begin
                out_pend  <= 1'b0;
                rsp_valid <= 1'b1;
                rsp_rdata <= rx_data;
                rsp_error <= (rx_cmd == DC_CMD_ERROR);
                out_tag   <= out_tag + 8'd1;   // retire the tag
            end
        end
    end

    // ---------------- completer ----------------
    logic       c_pend;      // request being serviced locally
    logic       c_rsp_pend;  // response waiting for a frame slot
    logic [7:0] c_tag;
    logic [103:0] c_pl;

    logic rx_is_req;
    assign rx_is_req = drx_valid
                       && (rx_cmd == DC_CMD_WRITE_REQ
                           || rx_cmd == DC_CMD_READ_REQ)
                       && !c_pend && !c_rsp_pend;  // spec: repeats dropped
                                                   // while one is in flight

    always_ff @(posedge clk) begin
        if (rst || !link_up) begin
            c_pend     <= 1'b0;
            c_rsp_pend <= 1'b0;
            cmp_req    <= 1'b0;
        end else begin
            if (rx_is_req) begin
                c_pend     <= 1'b1;
                cmp_req    <= 1'b1;
                c_tag      <= rx_tag;
                cmp_write  <= (rx_cmd == DC_CMD_WRITE_REQ);
                cmp_addr   <= drx_payload[63:32];
                cmp_wdata  <= drx_payload[95:64];
                cmp_byteen <= drx_payload[99:96];
            end else if (c_pend && cmp_done) begin
                c_pend     <= 1'b0;
                cmp_req    <= 1'b0;
                c_rsp_pend <= 1'b1;
                c_pl       <= '0;
                c_pl[23:16] <= c_tag;
                c_pl[31:24] <= cmp_write ? DC_CMD_WRITE_RESP
                                         : DC_CMD_READ_RESP;
                c_pl[95:64] <= cmp_rdata;
            end else if (c_rsp_pend && rsp_granted) begin
                c_rsp_pend <= 1'b0;
            end
        end
    end

    // ---------------- frame-slot arbitration (response first) ----------
    assign dtx_req     = c_rsp_pend || (out_pend && !out_sent);
    assign dtx_payload = c_rsp_pend ? c_pl : req_pl;
    assign rsp_granted = dtx_grant && c_rsp_pend;
    assign req_granted = dtx_grant && !c_rsp_pend && out_pend && !out_sent;

`ifdef FORMAL
    logic f_past_valid = 1'b0;
    always_ff @(posedge clk)
        f_past_valid <= 1'b1;

    initial assume (rst);

    // Grants only in answer to a request, one per frame.
    always_comb
        assume (!dtx_grant || dtx_req);

    // D1: single outstanding - while a request is pending the port is not
    // ready, so a second one cannot be accepted.
    always_comb
        if (f_past_valid && out_pend)
            assert (!req_ready);

    // D2: a response pulse only ever results from a delivered response
    // frame carrying the outstanding tag.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && rsp_valid) begin
            assert ($past(rx_is_rsp));
            assert ($past(rx_tag) == $past(out_tag));
            assert ($past(out_sent));
        end
    end

    // D3: the completer never sends a response without a serviced request,
    // and the response carries the request's tag.
    always_comb
        if (f_past_valid && c_rsp_pend)
            assert (c_pl[23:16] == c_tag);
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst)
            && c_rsp_pend && !$past(c_rsp_pend))
            assert ($past(c_pend) && $past(cmp_done));
    end

    // D4: no frame-slot request without a message to send.
    always_comb
        if (f_past_valid && dtx_req)
            assert (c_rsp_pend || (out_pend && !out_sent));

    // D5: completer request level holds until done.
    always_comb
        if (f_past_valid)
            assert (cmp_req == c_pend);

    // Covers: full write and read round trips on the requester side, an
    // error response, and a completer service cycle.
    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            cover (rsp_valid && !rsp_error && $past(rx_cmd) == DC_CMD_READ_RESP);
            cover (rsp_valid && !rsp_error && $past(rx_cmd) == DC_CMD_WRITE_RESP);
            cover (rsp_valid && rsp_error);
            cover ($past(c_rsp_pend) && !c_rsp_pend);
        end
    end
`endif

endmodule
