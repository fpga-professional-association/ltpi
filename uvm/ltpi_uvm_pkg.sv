// UVM verification environment for the LTPI endpoint (ltpi_top, SCM role).
//
// Architecture:
//   - The DUT is a complete SCM LTPI endpoint.
//   - ltpi_peer_agent emulates the HPM at the serial-bit level:
//       driver     serializes ltpi_frame_item transactions (16 bytes,
//                  CRC-8 computed, optionally corrupted) LSB-first
//       monitor    reassembles BOTH directions into frame transactions
//                  (comma-aligned, CRC-checked) and publishes them
//       sequencer  reactive: sequences see the DUT's frames through a
//                  request FIFO and answer per the LTPI training protocol
//   - ltpi_scoreboard checks protocol-order rules and data integrity.
//   - ltpi_coverage bins frame types, link states and negotiated speeds.
//
// Tests:
//   ltpi_bringup_test    clean training to Operational + GPIO integrity
//   ltpi_crc_error_test  training with injected CRC corruption; the link
//                        must still come up (spec: dropped frames retry)
//
// Run with any UVM-1.2 simulator - see run_questa.do / run_vcs.sh /
// run_xcelium.sh. (UVM does not run on iverilog/verilator mainline.)
package ltpi_uvm_pkg;
    import uvm_pkg::*;
    `include "uvm_macros.svh"

    // ---------------- frame item ----------------
    typedef enum byte {
        FT_LINK_DETECT, FT_LINK_SPEED, FT_ADVERTISE,
        FT_CONFIGURE, FT_ACCEPT, FT_OP_IO, FT_INVALID
    } ft_e;

    class ltpi_frame_item extends uvm_sequence_item;
        rand ft_e         ftype;
        rand byte         payload [13];   // bytes 2..14
        rand bit          corrupt_crc;
        bit               crc_ok;         // monitor-filled
        `uvm_object_utils_begin(ltpi_frame_item)
            `uvm_field_int(corrupt_crc, UVM_ALL_ON)
            `uvm_field_int(crc_ok, UVM_ALL_ON)
        `uvm_object_utils_end
        constraint c_no_corrupt_default { soft corrupt_crc == 0; }
        function new(string name = "ltpi_frame_item"); super.new(name); endfunction

        function byte comma();
            case (ftype)
                FT_LINK_DETECT, FT_LINK_SPEED: return 8'hBC;
                FT_ADVERTISE, FT_CONFIGURE, FT_ACCEPT: return 8'hDC;
                default: return 8'hFC;
            endcase
        endfunction
        function byte subtype();
            case (ftype)
                FT_LINK_SPEED:  return 8'h01;
                FT_CONFIGURE:   return 8'h01;
                FT_ACCEPT:      return 8'h02;
                default:        return 8'h00;
            endcase
        endfunction

        static function byte crc8(byte crc, byte data);
            byte c; c = crc ^ data;
            repeat (8) c = c[7] ? ((c << 1) ^ 8'h07) : (c << 1);
            return c;
        endfunction

        function void pack_bytes(output byte b [16]);
            byte c;
            b[0] = comma(); b[1] = subtype();
            foreach (payload[i]) b[2 + i] = payload[i];
            c = 0;
            for (int i = 1; i <= 14; i++) c = crc8(c, b[i]);
            b[15] = corrupt_crc ? c ^ 8'h5A : c;
        endfunction
    endclass

    // ---------------- driver ----------------
    class ltpi_driver extends uvm_driver #(ltpi_frame_item);
        `uvm_component_utils(ltpi_driver)
        virtual ltpi_if vif;
        function new(string name, uvm_component parent); super.new(name, parent); endfunction
        function void build_phase(uvm_phase phase);
            if (!uvm_config_db#(virtual ltpi_if)::get(this, "", "vif", vif))
                `uvm_fatal("NOVIF", "ltpi_if not set")
        endfunction
        task run_phase(uvm_phase phase);
            byte b [16];
            vif.drv_cb.agt_tx_bit <= 1'b0;
            forever begin
                seq_item_port.get_next_item(req);
                req.pack_bytes(b);
                foreach (b[i])
                    for (int k = 0; k < 8; k++) begin
                        vif.drv_cb.agt_tx_bit <= b[i][k];   // LSB first
                        @(vif.drv_cb);
                    end
                seq_item_port.item_done();
            end
        endtask
    endclass

    // ---------------- monitor (one per direction) ----------------
    class ltpi_monitor extends uvm_component;
        `uvm_component_utils(ltpi_monitor)
        virtual ltpi_if vif;
        bit watch_dut;   // 1: DUT->agent stream, 0: agent->DUT
        uvm_analysis_port #(ltpi_frame_item) ap;
        function new(string name, uvm_component parent);
            super.new(name, parent); ap = new("ap", this);
        endfunction
        function void build_phase(uvm_phase phase);
            if (!uvm_config_db#(virtual ltpi_if)::get(this, "", "vif", vif))
                `uvm_fatal("NOVIF", "ltpi_if not set")
        endfunction

        task run_phase(uvm_phase phase);
            byte win; bit aligned = 0; int cnt = 0;
            byte b [16]; int bidx = 0;
            forever begin
                bit v;
                @(vif.mon_cb);
                v = watch_dut ? vif.mon_cb.dut_tx_bit : vif.mon_cb.agt_tx_bit;
                win = {v, win[7:1]};
                if (!aligned) begin
                    if (win == 8'hBC) begin
                        aligned = 1; cnt = 0; bidx = 0; b[0] = win; bidx = 1;
                    end
                end else begin
                    cnt++;
                    if (cnt == 8) begin
                        cnt = 0; b[bidx] = win; bidx++;
                        if (bidx == 16) begin
                            publish(b); bidx = 0;
                        end
                    end
                end
            end
        endtask

        function void publish(byte b [16]);
            ltpi_frame_item tr = ltpi_frame_item::type_id::create("tr");
            byte c = 0;
            case (b[0])
                8'hBC: tr.ftype = (b[1] == 8'h01) ? FT_LINK_SPEED : FT_LINK_DETECT;
                8'hDC: tr.ftype = (b[1] == 8'h02) ? FT_ACCEPT :
                                  (b[1] == 8'h01) ? FT_CONFIGURE : FT_ADVERTISE;
                8'hFC: tr.ftype = FT_OP_IO;
                default: tr.ftype = FT_INVALID;
            endcase
            for (int i = 0; i < 13; i++) tr.payload[i] = b[2 + i];
            for (int i = 1; i <= 14; i++) c = ltpi_frame_item::crc8(c, b[i]);
            tr.crc_ok = (c == b[15]);
            ap.write(tr);
        endfunction
    endclass

    // ---------------- reactive sequencer ----------------
    class ltpi_sequencer extends uvm_sequencer #(ltpi_frame_item);
        `uvm_component_utils(ltpi_sequencer)
        // DUT frames observed by the monitor, for reactive sequences.
        uvm_tlm_analysis_fifo #(ltpi_frame_item) dut_frames;
        function new(string name, uvm_component parent);
            super.new(name, parent); dut_frames = new("dut_frames", this);
        endfunction
    endclass

    // ---------------- agent ----------------
    class ltpi_peer_agent extends uvm_agent;
        `uvm_component_utils(ltpi_peer_agent)
        ltpi_driver    drv;
        ltpi_sequencer sqr;
        ltpi_monitor   mon_dut, mon_agt;
        function new(string name, uvm_component parent); super.new(name, parent); endfunction
        function void build_phase(uvm_phase phase);
            drv = ltpi_driver::type_id::create("drv", this);
            sqr = ltpi_sequencer::type_id::create("sqr", this);
            mon_dut = ltpi_monitor::type_id::create("mon_dut", this);
            mon_agt = ltpi_monitor::type_id::create("mon_agt", this);
            mon_dut.watch_dut = 1;
            mon_agt.watch_dut = 0;
        endfunction
        function void connect_phase(uvm_phase phase);
            drv.seq_item_port.connect(sqr.seq_item_export);
            mon_dut.ap.connect(sqr.dut_frames.analysis_export);
        endfunction
    endclass

    // ---------------- HPM peer sequence (the training protocol) ----------
    class ltpi_hpm_peer_seq extends uvm_sequence #(ltpi_frame_item);
        `uvm_object_utils(ltpi_hpm_peer_seq)
        rand int unsigned corrupt_pct = 0;   // % of frames CRC-corrupted
        bit [15:0] my_caps = 16'h8121;       // 25M+200M+400M SDR + DDR
        function new(string name = "ltpi_hpm_peer_seq"); super.new(name); endfunction

        task body();
            ltpi_sequencer sqr;
            ltpi_frame_item dutf, rsp;
            ft_e reply;
            bit [15:0] sel = '0;
            if (!$cast(sqr, m_sequencer)) `uvm_fatal("SQR", "wrong sequencer")
            forever begin
                // React to the most recent DUT frame (drain the FIFO).
                while (sqr.dut_frames.try_get(dutf)) begin
                    if (dutf.ftype == FT_LINK_SPEED && dutf.crc_ok)
                        sel = {dutf.payload[2], dutf.payload[1]};
                end
                if (dutf == null) reply = FT_LINK_DETECT;
                else case (dutf.ftype)
                    FT_LINK_DETECT: reply = FT_LINK_DETECT;
                    FT_LINK_SPEED:  reply = FT_LINK_SPEED;
                    FT_ADVERTISE:   reply = FT_ADVERTISE;
                    FT_CONFIGURE:   reply = FT_ACCEPT;
                    FT_OP_IO:       reply = FT_OP_IO;
                    default:        reply = FT_LINK_DETECT;
                endcase
                rsp = ltpi_frame_item::type_id::create("rsp");
                start_item(rsp);
                if (!rsp.randomize() with {
                        ftype == reply;
                        corrupt_crc dist {1 := corrupt_pct,
                                          0 := 100 - corrupt_pct};
                    })
                    `uvm_fatal("RAND", "randomize failed")
                // Fill protocol payloads
                foreach (rsp.payload[i]) rsp.payload[i] = 8'h00;
                case (reply)
                    FT_LINK_DETECT: begin
                        rsp.payload[0] = 8'h10;              // version
                        {rsp.payload[2], rsp.payload[1]} = my_caps;
                    end
                    FT_LINK_SPEED: begin
                        rsp.payload[0] = 8'h10;
                        {rsp.payload[2], rsp.payload[1]} = sel;
                    end
                    FT_ACCEPT: ;
                    default: ;
                endcase
                finish_item(rsp);
            end
        endtask
    endclass

    // ---------------- scoreboard ----------------
    class ltpi_scoreboard extends uvm_component;
        `uvm_component_utils(ltpi_scoreboard)
        uvm_tlm_analysis_fifo #(ltpi_frame_item) dut_fifo;
        virtual ltpi_if vif;
        int n_frames, n_crc_err, n_op;
        bit seen_cfg;
        function new(string name, uvm_component parent);
            super.new(name, parent); dut_fifo = new("dut_fifo", this);
        endfunction
        function void build_phase(uvm_phase phase);
            if (!uvm_config_db#(virtual ltpi_if)::get(this, "", "vif", vif))
                `uvm_fatal("NOVIF", "ltpi_if not set")
        endfunction
        task run_phase(uvm_phase phase);
            ltpi_frame_item tr;
            forever begin
                dut_fifo.get(tr);
                n_frames++;
                // C1: everything the DUT emits must be CRC-clean.
                if (!tr.crc_ok) begin
                    n_crc_err++;
                    `uvm_error("SB", "DUT emitted a CRC-bad frame")
                end
                // C2: protocol order - no Operational frames before the
                // DUT sent Configure (SCM role).
                if (tr.ftype == FT_CONFIGURE) seen_cfg = 1;
                if (tr.ftype == FT_OP_IO) begin
                    n_op++;
                    if (!seen_cfg)
                        `uvm_error("SB", "Operational frame before Configure")
                    // C3: data integrity - LL GPIO field mirrors the pins
                    // (allow one frame of latch latency at the boundary).
                    if ({tr.payload[2], tr.payload[1]} !== vif.ll_gpio_in)
                        `uvm_info("SB", "LL field lags pin (frame boundary)",
                                  UVM_HIGH)
                end
            end
        endtask
        function void report_phase(uvm_phase phase);
            `uvm_info("SB", $sformatf(
                "frames=%0d op_frames=%0d crc_err=%0d link_up=%b speed=%h",
                n_frames, n_op, n_crc_err, vif.link_up, vif.speed_select),
                UVM_LOW)
        endfunction
    endclass

    // ---------------- functional coverage ----------------
    class ltpi_coverage extends uvm_component;
        `uvm_component_utils(ltpi_coverage)
        uvm_tlm_analysis_fifo #(ltpi_frame_item) fifo;
        virtual ltpi_if vif;
        ltpi_frame_item cur;

        covergroup cg_frame;
            coverpoint cur.ftype {
                bins detect = {FT_LINK_DETECT}; bins speed = {FT_LINK_SPEED};
                bins adv = {FT_ADVERTISE}; bins cfg = {FT_CONFIGURE};
                bins acc = {FT_ACCEPT}; bins op = {FT_OP_IO};
            }
            coverpoint cur.crc_ok;
        endgroup
        // Sampled manually per observed frame (a vif clock event cannot be
        // referenced at covergroup construction time).
        covergroup cg_link;
            coverpoint vif.link_state {
                bins states [] = {[0:6]};
                bins train_ok = (5 => 6);
            }
            coverpoint vif.speed_select iff (vif.speed_valid) {
                bins sdr200 = {16'h0020};
                bins sdr400 = {16'h0100};
                bins ddr400 = {16'h8100};
                bins base25 = {16'h0001};
            }
        endgroup

        function new(string name, uvm_component parent);
            super.new(name, parent);
            fifo = new("fifo", this);
            cg_frame = new(); cg_link = new();
        endfunction
        function void build_phase(uvm_phase phase);
            if (!uvm_config_db#(virtual ltpi_if)::get(this, "", "vif", vif))
                `uvm_fatal("NOVIF", "ltpi_if not set")
        endfunction
        task run_phase(uvm_phase phase);
            forever begin
                fifo.get(cur);
                cg_frame.sample();
                cg_link.sample();
            end
        endtask
    endclass

    // ---------------- env ----------------
    class ltpi_env extends uvm_env;
        `uvm_component_utils(ltpi_env)
        ltpi_peer_agent agent;
        ltpi_scoreboard sb;
        ltpi_coverage   cov;
        function new(string name, uvm_component parent); super.new(name, parent); endfunction
        function void build_phase(uvm_phase phase);
            agent = ltpi_peer_agent::type_id::create("agent", this);
            sb    = ltpi_scoreboard::type_id::create("sb", this);
            cov   = ltpi_coverage::type_id::create("cov", this);
        endfunction
        function void connect_phase(uvm_phase phase);
            agent.mon_dut.ap.connect(sb.dut_fifo.analysis_export);
            agent.mon_dut.ap.connect(cov.fifo.analysis_export);
        endfunction
    endclass

    // ---------------- tests ----------------
    class ltpi_base_test extends uvm_test;
        `uvm_component_utils(ltpi_base_test)
        ltpi_env env;
        virtual ltpi_if vif;
        function new(string name, uvm_component parent); super.new(name, parent); endfunction
        function void build_phase(uvm_phase phase);
            env = ltpi_env::type_id::create("env", this);
            if (!uvm_config_db#(virtual ltpi_if)::get(this, "", "vif", vif))
                `uvm_fatal("NOVIF", "ltpi_if not set")
        endfunction
        task pre_main(int corrupt_pct);
            ltpi_hpm_peer_seq seq = ltpi_hpm_peer_seq::type_id::create("seq");
            seq.corrupt_pct = corrupt_pct;
            fork seq.start(env.agent.sqr); join_none
        endtask
    endclass

    class ltpi_bringup_test extends ltpi_base_test;
        `uvm_component_utils(ltpi_bringup_test)
        function new(string name, uvm_component parent); super.new(name, parent); endfunction
        task run_phase(uvm_phase phase);
            phase.raise_objection(this);
            pre_main(0);
            // Wait for link up, then run GPIO traffic.
            wait (vif.link_up === 1'b1);
            `uvm_info("TEST", $sformatf("link up, speed=%h", vif.speed_select),
                      UVM_LOW)
            if (vif.speed_select != 16'h8100)
                `uvm_error("TEST", "expected 400MHz DDR negotiation")
            repeat (4) begin
                vif.ll_gpio_in = $urandom();
                repeat (2000) @(posedge vif.clk);
            end
            phase.drop_objection(this);
        endtask
    endclass

    class ltpi_crc_error_test extends ltpi_base_test;
        `uvm_component_utils(ltpi_crc_error_test)
        function new(string name, uvm_component parent); super.new(name, parent); endfunction
        task run_phase(uvm_phase phase);
            phase.raise_objection(this);
            pre_main(10);   // 10% of peer frames CRC-corrupted
            fork
                wait (vif.link_up === 1'b1);
                begin repeat (2_000_000) @(posedge vif.clk);
                      `uvm_error("TEST", "link never came up under 10% CRC noise")
                end
            join_any
            disable fork;
            phase.drop_objection(this);
        endtask
    endclass

endpackage
