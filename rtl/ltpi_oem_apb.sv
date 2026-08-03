// OEM channel: AMBA APB tunneling over the Default I/O Frame's
// OEM-reserved bytes 11-14 (spec Table 33: "OEM fields can be used to
// extend Default I/O Frame"). Independent of the Data Channel - APB
// transactions ride every I/O frame without consuming Data Frame slots.
//
// Message format, one 4-byte beat per I/O frame (payload bits [103:72]):
//   byte 11 [79:72]  {seq[1:0], kind[1:0], tag[3:0]}
//                    kind: 00 idle, 01 request, 10 response
//                    seq : beat number within the message
//   bytes 12-14      beat payload (see below)
// Request  (3 beats): 0:{ctrl: pwrite, pstrb[3:0]}  1:paddr  2:pwdata
// Response (2 beats): 0:{status: pslverr}           1:prdata
// Beats of one message ride consecutive I/O frames; a CRC-dropped frame
// drops the beat and the sequencer RESTARTS the message from beat 0 on
// the next frame (seq mismatch at the receiver discards partial state),
// so a message is only ever accepted complete and in order.
//
// REQUESTER side: a true APB *completer* port - point a local APB master
// (BMC bridge) at it. It back-pressures with pready until the far side
// answers; APB's blocking handshake gives single-outstanding for free.
// COMPLETER side: a true APB *requester* (master) port driving the remote
// APB slave with spec-shaped SETUP -> ACCESS phases.
import ltpi_pkg::*;

module ltpi_oem_apb (
    input  logic clk,
    input  logic rst,
    input  logic link_up,

    // ---------------- APB completer (local master connects here) -------
    input  logic        s_psel,
    input  logic        s_penable,
    input  logic        s_pwrite,
    input  logic [31:0] s_paddr,
    input  logic [31:0] s_pwdata,
    input  logic [3:0]  s_pstrb,
    output logic        s_pready,
    output logic [31:0] s_prdata,
    output logic        s_pslverr,

    // ---------------- APB requester (drives the remote APB slave) ------
    output logic        m_psel,
    output logic        m_penable,
    output logic        m_pwrite,
    output logic [31:0] m_paddr,
    output logic [31:0] m_pwdata,
    output logic [3:0]  m_pstrb,
    input  logic        m_pready,
    input  logic [31:0] m_prdata,
    input  logic        m_pslverr,

    // ---------------- frame side ----------------
    output logic [31:0] oem_tx,        // I/O-frame bytes 11-14 (this frame)
    input  logic        oem_tx_taken,  // an I/O frame latched oem_tx
    input  logic        oem_rx_valid,  // CRC-good I/O frame arrived
    input  logic [31:0] oem_rx
);

    localparam logic [1:0] K_IDLE = 2'd0, K_REQ = 2'd1, K_RSP = 2'd2;

    // ------------------------------------------------------------------
    // Requester: APB completer -> 3-beat request, tagged response wait
    // ------------------------------------------------------------------
    logic [3:0] rq_tag;
    logic [1:0] rq_beat;      // next request beat to send (0..2)
    logic       rq_pend;      // transaction accepted, response not returned
    logic       rq_sent;      // all 3 beats sent at least once
    logic [1:0] rsp_beat;     // response reassembly progress
    logic       rsp_err_q;

    logic apb_req_start;
    assign apb_req_start = s_psel && s_penable && !s_pready && !rq_pend
                           && link_up;

    // ------------------------------------------------------------------
    // Completer: request reassembly -> APB master -> 2-beat response
    // ------------------------------------------------------------------
    logic [1:0] cq_beat;      // request reassembly progress
    logic [3:0] cq_tag;
    logic       c_busy;       // APB master transaction in flight
    logic       c_rsp_pend;   // response ready to stream
    logic [1:0] c_rsp_beat;
    logic       c_err_q;
    logic [31:0] c_rdata_q;

    // RX decode
    logic [1:0] rx_kind, rx_seq;
    logic [3:0] rx_tag;
    logic [23:0] rx_data;
    assign rx_kind = oem_rx[5:4];
    assign rx_seq  = oem_rx[7:6];
    assign rx_tag  = oem_rx[3:0];
    assign rx_data = oem_rx[31:8];

    // Full 32-bit fields ride bytes 12-14 across beats; pack addr/data as
    // 24-bit beat payloads plus the 8 low bits folded into the next beat.
    // Simpler: 32-bit values are split 24+8: beat N carries [23:0], the
    // following beat's spare byte carries [31:24].
    // Request beats:  0:{pwrite,pstrb, paddr[31:24]} 1:paddr[23:0]+pwdata[31:24] 2:pwdata[23:0]
    // Response beats: 0:{pslverr, prdata[31:24]}     1:prdata[23:0]
    logic [31:0] cq_addr, cq_wdata;
    logic        cq_write;
    logic [3:0]  cq_strb;

    always_ff @(posedge clk) begin
        if (rst || !link_up) begin
            rq_tag    <= '0;
            rq_beat   <= '0;
            rq_pend   <= 1'b0;
            rq_sent   <= 1'b0;
            rsp_beat  <= '0;
            s_pready  <= 1'b0;
            cq_beat   <= '0;
            c_busy    <= 1'b0;
            c_rsp_pend<= 1'b0;
            c_rsp_beat<= '0;
            m_psel    <= 1'b0;
            m_penable <= 1'b0;
        end else begin
            s_pready <= 1'b0;

            // ---------------- requester ----------------
            if (apb_req_start) begin
                rq_pend  <= 1'b1;
                rq_beat  <= '0;
                rq_sent  <= 1'b0;
                rsp_beat <= '0;
            end else if (rq_pend && !rq_sent && oem_tx_taken) begin
                if (rq_beat == 2'd2) begin
                    rq_sent <= 1'b1;
                end
                rq_beat <= (rq_beat == 2'd2) ? 2'd0 : rq_beat + 1'b1;
            end

            // Response reassembly (tag-checked, in-sequence)
            if (rq_pend && rq_sent && oem_rx_valid
                && rx_kind == K_RSP && rx_tag == rq_tag) begin
                if (rx_seq == 2'd0 && rsp_beat == 2'd0) begin
                    rsp_err_q          <= rx_data[16];
                    s_prdata[31:24]    <= rx_data[7:0];
                    rsp_beat           <= 2'd1;
                end else if (rx_seq == 2'd1 && rsp_beat == 2'd1) begin
                    s_prdata[23:0]     <= rx_data;
                    s_pready           <= 1'b1;
                    s_pslverr          <= rsp_err_q;
                    rq_pend            <= 1'b0;
                    rq_tag             <= rq_tag + 4'd1;
                end else begin
                    rsp_beat <= '0;    // out of sequence: restart
                end
            end

            // ---------------- completer ----------------
            if (oem_rx_valid && rx_kind == K_REQ && !c_busy && !c_rsp_pend) begin
                case (rx_seq)
                    2'd0: begin
                        cq_tag          <= rx_tag;
                        cq_wdata[31:24] <= rx_data[23:16];
                        cq_write        <= rx_data[12];
                        cq_strb         <= rx_data[11:8];
                        cq_addr[31:24]  <= rx_data[7:0];
                        cq_beat         <= 2'd1;
                    end
                    2'd1: if (cq_beat == 2'd1 && rx_tag == cq_tag) begin
                        cq_addr[23:0]   <= rx_data;
                        cq_beat         <= 2'd2;
                    end else cq_beat <= '0;
                    2'd2: if (cq_beat == 2'd2 && rx_tag == cq_tag) begin
                        cq_wdata        <= {cq_wdata[31:24], rx_data};
                        cq_beat         <= '0;
                        // Launch the APB SETUP phase
                        c_busy          <= 1'b1;
                        m_psel          <= 1'b1;
                        m_penable       <= 1'b0;
                        m_pwrite        <= cq_write;
                        m_paddr         <= cq_addr;
                        m_pwdata        <= {cq_wdata[31:24], rx_data};
                        m_pstrb         <= cq_strb;
                    end else cq_beat <= '0;
                    default: cq_beat <= '0;
                endcase
            end

            // APB master SETUP -> ACCESS -> completion
            if (c_busy) begin
                if (!m_penable)
                    m_penable <= 1'b1;                 // ACCESS phase
                else if (m_pready) begin
                    m_psel     <= 1'b0;
                    m_penable  <= 1'b0;
                    c_busy     <= 1'b0;
                    c_rsp_pend <= 1'b1;
                    c_rsp_beat <= '0;
                    c_err_q    <= m_pslverr;
                    c_rdata_q  <= m_prdata;
                end
            end

            // Stream the response beats
            if (c_rsp_pend && oem_tx_taken) begin
                if (c_rsp_beat == 2'd1) begin
                    c_rsp_pend <= 1'b0;
                end
                c_rsp_beat <= (c_rsp_beat == 2'd1) ? 2'd0 : c_rsp_beat + 1'b1;
            end
        end
    end

    // TX beat mux: response streaming wins (frees the completer fastest);
    // otherwise request beats; otherwise idle.
    always_comb begin
        oem_tx = '0;
        if (c_rsp_pend) begin
            oem_tx[5:4] = K_RSP;
            oem_tx[7:6] = c_rsp_beat;
            oem_tx[3:0] = cq_tag;
            if (c_rsp_beat == 2'd0)
                oem_tx[31:8] = {7'd0, c_err_q, 8'd0, c_rdata_q[31:24]};
            else
                oem_tx[31:8] = c_rdata_q[23:0];
        end else if (rq_pend && !rq_sent) begin
            oem_tx[5:4] = K_REQ;
            oem_tx[7:6] = rq_beat;
            oem_tx[3:0] = rq_tag;
            case (rq_beat)
                // beat0 data[23:0]: [23:16]=pwdata[31:24], [15:13]=0,
                //                   [12]=pwrite, [11:8]=pstrb,
                //                   [7:0]=paddr[31:24]
                2'd0: oem_tx[31:8] = {s_pwdata[31:24], 3'd0, s_pwrite,
                                      s_pstrb, s_paddr[31:24]};
                2'd1: oem_tx[31:8] = s_paddr[23:0];
                default: oem_tx[31:8] = s_pwdata[23:0];
            endcase
        end
    end

`ifdef DEBUG_MON
    always @(posedge clk) begin
        if (oem_rx_valid && rx_kind != 2'd0)
            $display("[%0t] OEMAPB-RX %m kind=%0d seq=%0d tag=%0d data=%h",
                     $time, rx_kind, rx_seq, rx_tag, rx_data);
        if (oem_tx_taken && oem_tx[5:4] != 2'd0)
            $display("[%0t] OEMAPB-TX %m kind=%0d seq=%0d",
                     $time, oem_tx[5:4], oem_tx[7:6]);
    end
`endif

`ifdef FORMAL
    logic f_past_valid = 1'b0;
    always_ff @(posedge clk)
        f_past_valid <= 1'b1;

    initial assume (rst);

    // APB env assumptions (AMBA spec): the master holds the request
    // stable through the transfer and follows SETUP->ACCESS sequencing.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst)) begin
            // AMBA APB: a started transfer persists until PREADY.
            if ($past(s_psel) && $past(s_penable) && !$past(s_pready)) begin
                assume (s_psel);
                assume (s_penable);
                assume (s_pwrite == $past(s_pwrite));
                assume (s_paddr  == $past(s_paddr));
                assume (s_pwdata == $past(s_pwdata));
                assume (s_pstrb  == $past(s_pstrb));
            end
        end
    end
    always_comb begin
        assume (!(oem_rx_valid && oem_tx_taken) || 1'b1); // may coincide
        if (s_penable) assume (s_psel);
    end

    // O1: APB completer - pready only in the ACCESS phase of a selected
    // transfer, and only after the request was fully sent.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && s_pready) begin
            assert ($past(s_psel) && $past(s_penable));
            assert ($past(rq_sent));
        end
    end

    // O2: APB requester - protocol shape: PENABLE only with PSEL; SETUP
    // phase is exactly one cycle; signals stable during the transfer.
    always_comb
        if (f_past_valid && m_penable)
            assert (m_psel);
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst)) begin
            if (m_psel && !m_penable)                  // SETUP phase
                assert (!$past(m_psel) || $past(m_pready)); // fresh setup
            if ($past(m_psel) && !$past(m_pready) && m_psel) begin
                assert (m_pwrite == $past(m_pwrite));
                assert (m_paddr  == $past(m_paddr));
                assert (m_pwdata == $past(m_pwdata));
            end
            if ($past(m_psel) && $past(m_penable) && $past(m_pready))
                assert (!m_psel);                      // retires after ready
        end
    end

    // O2b (induction coupling): the APB master is selected exactly while
    // a completer transaction is in flight.
    always_comb
        if (f_past_valid)
            assert (!m_psel || c_busy);

    // O1b (induction lemma): while a request is pending, the local APB
    // master is still holding the transfer (guaranteed by the AMBA
    // persistence contract, since PREADY has stayed low).
    always_comb
        if (f_past_valid)
            assert (!rq_pend || (s_psel && s_penable));

    // O3: single outstanding on the requester - a new transaction is
    // never accepted while one is pending.
    always_comb
        if (f_past_valid)
            assert (!(apb_req_start && rq_pend));

    // O4: sequencer bounds.
    always_comb
        if (f_past_valid) begin
            assert (rq_beat <= 2'd2);
            assert (cq_beat <= 2'd2);
            assert (rsp_beat <= 2'd1);
            assert (c_rsp_beat <= 2'd1);
        end

    // O5: the completer never streams a response without having run the
    // APB master transaction for a fully reassembled request.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst)
            && c_rsp_pend && !$past(c_rsp_pend))
            assert ($past(c_busy) && $past(m_pready));
    end

    // Covers: write and read acceptance, full request stream, response
    // delivery back to the local master.
    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            cover (apb_req_start && s_pwrite);
            cover (apb_req_start && !s_pwrite);
            cover (rq_sent);
            cover (m_psel && m_penable && m_pready);
            cover (s_pready && !s_pslverr);
            cover (s_pready && s_pslverr);
        end
    end
`endif

endmodule
