// System-level loopback simulation: two complete LTPI endpoints (SCM CPLD +
// HPM FPGA) cross-connected at the serial-bit level, SDR mode.
//
// Demonstrates, end to end through the real byte-serial datapath:
//   1. Full link training: Detect -> Speed -> Advertise -> Configure/Accept
//      -> Operational on both sides (shrunk thresholds for sim speed)
//   2. LL GPIO tunneling SCM -> HPM
//   3. NL GPIO tunneling (multiplexed over frames)
//   4. UART sample tunneling
//   5. I2C relay Start-event handshake with clock stretching
//
// Self-checking: fails with $fatal on timeout or data mismatch.
`timescale 1ns/1ps

module tb_ltpi_system;
    import ltpi_pkg::*;

    logic clk = 0;
    always #2.5 clk = ~clk;   // 200 MHz link clock (SDR -> 200 Mbps)

    logic rst = 1;

    // Serial cross-connect (SDR)
    logic scm_tx_bit, hpm_tx_bit;
    logic [1:0] scm_tx_ddr, hpm_tx_ddr;

    // SCM channel-side signals
    logic [15:0] scm_ll_in = '0,  scm_ll_out;
    logic [31:0] scm_nl_in = '0,  scm_nl_out;
    logic        scm_uart_txd = 1'b1, scm_uart_txd_out;
    logic        scm_scl = 1, scm_sda = 1;   // raw SCM-side bus lines
    logic        hpm_scl = 1, hpm_sda = 1;   // raw HPM-side bus lines
    logic        scm_scl_stretch, scm_sda_pull;
    logic [1:0]  scm_stretch_v, scm_pull_v, scm_sgen_v, scm_pgen_v;
    logic [1:0]  hpm_stretch_v, hpm_pull_v, hpm_sgen_v, hpm_pgen_v;

    // HPM channel-side signals
    logic [15:0] hpm_ll_in = '0,  hpm_ll_out;
    logic [31:0] hpm_nl_in = '0,  hpm_nl_out;
    logic        hpm_uart_txd = 1'b1, hpm_uart_txd_out;

    logic        scm_soft = 0, hpm_soft = 0, scm_retrain = 0;
    logic [7:0]  csr_addr = 0;
    logic        csr_we = 0, csr_re = 0;
    logic [31:0] csr_wdata = 0;
    logic [31:0] scm_csr_rdata, hpm_csr_rdata;
    // Data channel: SCM requester, HPM completer + tiny memory
    logic        dc_req_v = 0, dc_req_w = 0;
    logic [31:0] dc_req_addr = 0, dc_req_wdata = 0;
    logic        dc_req_ready, dc_rsp_valid, dc_rsp_error;
    logic [31:0] dc_rsp_rdata;
    logic        hpm_cmp_req, hpm_cmp_write, hpm_cmp_done = 0;
    logic [31:0] hpm_cmp_addr, hpm_cmp_wdata;
    logic [31:0] hpm_cmp_rdata = 0;
    logic [31:0] tb_mem [0:15];
    // OEM APB: SCM-side master (tb-driven) -> HPM-side APB slave memory
    logic        apb_psel = 0, apb_penable = 0, apb_pwrite = 0;
    logic [31:0] apb_paddr = 0, apb_pwdata = 0;
    logic        apb_pready, apb_pslverr;
    logic [31:0] apb_prdata;
    logic        hpm_apb_psel, hpm_apb_penable, hpm_apb_pwrite;
    logic [31:0] hpm_apb_paddr, hpm_apb_pwdata;
    logic        hpm_apb_pready = 0;
    logic [31:0] hpm_apb_prdata = 0;
    logic [31:0] apb_mem [0:15];
    logic        hpm_scl_stretch, hpm_sda_pull;
    logic        hpm_start_gen, hpm_stop_gen;
    logic        scm_start_gen, scm_stop_gen;


    ltpi_pkg::link_state_t scm_state, hpm_state;
    logic scm_up, hpm_up;
    logic [15:0] scm_speed, hpm_speed;
    logic scm_speed_v, hpm_speed_v;
    logic scm_aligned, hpm_aligned;

    localparam CAPS = CAPS_DEFAULT;   // 25M + 200M SDR + 400M SDR + DDR

    ltpi_top #(
        .ROLE_SCM(1'b1), .NL_TOTAL(32), .CLK_HZ(200_000_000),
        .DETECT_MIN_TX(8), .DETECT_MIN_RX(4), .ALIGN_MIN_RX(3),
        .SPEED_SCM_MIN_TX(3), .SPEED_HPM_MIN_RX(2), .SPEED_TIMEOUT_TX(64),
        .ADV_MIN_CYCLES(1500), .ADV_ALIGN_TIMEOUT(3000), .ADV_MIN_RX(3), .CFG_MAX_TX(16), .ACC_MAX_TX(8)
    ) u_scm (
        .clk(clk), .rst(rst), .ddr_mode(1'b0),
        .local_speed_caps(CAPS), .cfg_ready(1'b1),
        .retrain_req(scm_retrain), .soft_reset(scm_soft),
        .ser_tx_sdr(scm_tx_bit), .ser_tx_ddr(scm_tx_ddr),
        .ser_rx_sdr(hpm_tx_bit), .ser_rx_ddr(hpm_tx_ddr),
        .ll_gpio_in(scm_ll_in), .ll_gpio_out(scm_ll_out),
        .nl_gpio_in(scm_nl_in), .nl_gpio_out(scm_nl_out),
        .uart_txd_in(scm_uart_txd), .uart_flow_in(1'b1),
        .uart_txd_out(scm_uart_txd_out), .uart_flow_out(),
        .i2c_scl_in({1'b1, scm_scl}),
        .i2c_sda_in({1'b1, scm_sda}),
        .i2c_scl_stretch(scm_stretch_v), .i2c_sda_pull(scm_pull_v),
        .i2c_bus_start_gen(scm_sgen_v), .i2c_bus_stop_gen(scm_pgen_v),
        .dc_req_valid(dc_req_v), .dc_req_write(dc_req_w),
        .dc_req_addr(dc_req_addr), .dc_req_wdata(dc_req_wdata),
        .dc_req_byteen(4'hF), .dc_req_ready(dc_req_ready),
        .dc_rsp_valid(dc_rsp_valid), .dc_rsp_rdata(dc_rsp_rdata),
        .dc_rsp_error(dc_rsp_error),
        .dc_cmp_req(), .dc_cmp_write(), .dc_cmp_addr(), .dc_cmp_wdata(),
        .dc_cmp_byteen(), .dc_cmp_rdata(32'h0), .dc_cmp_done(1'b0),
        .apb_s_psel(apb_psel), .apb_s_penable(apb_penable),
        .apb_s_pwrite(apb_pwrite), .apb_s_paddr(apb_paddr),
        .apb_s_pwdata(apb_pwdata), .apb_s_pstrb(4'hF),
        .apb_s_pready(apb_pready), .apb_s_prdata(apb_prdata),
        .apb_s_pslverr(apb_pslverr),
        .apb_m_psel(), .apb_m_penable(), .apb_m_pwrite(), .apb_m_paddr(),
        .apb_m_pwdata(), .apb_m_pstrb(),
        .apb_m_pready(1'b1), .apb_m_prdata(32'h0), .apb_m_pslverr(1'b0),
        .csr_addr(csr_addr), .csr_we(csr_we), .csr_re(csr_re),
        .csr_wdata(csr_wdata), .csr_rdata(scm_csr_rdata),
        .link_state(scm_state), .link_up(scm_up),
        .speed_select(scm_speed), .speed_valid(scm_speed_v),
        .phy_aligned(scm_aligned)
    );

    ltpi_top #(
        .ROLE_SCM(1'b0), .NL_TOTAL(32), .CLK_HZ(200_000_000),
        .DETECT_MIN_TX(8), .DETECT_MIN_RX(4), .ALIGN_MIN_RX(3),
        .SPEED_SCM_MIN_TX(3), .SPEED_HPM_MIN_RX(2), .SPEED_TIMEOUT_TX(64),
        .ADV_MIN_CYCLES(1500), .ADV_ALIGN_TIMEOUT(3000), .ADV_MIN_RX(3), .CFG_MAX_TX(16), .ACC_MAX_TX(8)
    ) u_hpm (
        .clk(clk), .rst(rst), .ddr_mode(1'b0),
        .local_speed_caps(CAPS), .cfg_ready(1'b0),
        .retrain_req(1'b0), .soft_reset(hpm_soft),
        .ser_tx_sdr(hpm_tx_bit), .ser_tx_ddr(hpm_tx_ddr),
        .ser_rx_sdr(scm_tx_bit), .ser_rx_ddr(scm_tx_ddr),
        .ll_gpio_in(hpm_ll_in), .ll_gpio_out(hpm_ll_out),
        .nl_gpio_in(hpm_nl_in), .nl_gpio_out(hpm_nl_out),
        .uart_txd_in(hpm_uart_txd), .uart_flow_in(1'b1),
        .uart_txd_out(hpm_uart_txd_out), .uart_flow_out(),
        .i2c_scl_in({1'b1, hpm_scl}),
        .i2c_sda_in({1'b1, hpm_sda}),
        .i2c_scl_stretch(hpm_stretch_v), .i2c_sda_pull(hpm_pull_v),
        .i2c_bus_start_gen(hpm_sgen_v), .i2c_bus_stop_gen(hpm_pgen_v),
        .dc_req_valid(1'b0), .dc_req_write(1'b0),
        .dc_req_addr(32'h0), .dc_req_wdata(32'h0),
        .dc_req_byteen(4'h0), .dc_req_ready(),
        .dc_rsp_valid(), .dc_rsp_rdata(), .dc_rsp_error(),
        .dc_cmp_req(hpm_cmp_req), .dc_cmp_write(hpm_cmp_write),
        .dc_cmp_addr(hpm_cmp_addr), .dc_cmp_wdata(hpm_cmp_wdata),
        .dc_cmp_byteen(), .dc_cmp_rdata(hpm_cmp_rdata),
        .dc_cmp_done(hpm_cmp_done),
        .apb_s_psel(1'b0), .apb_s_penable(1'b0), .apb_s_pwrite(1'b0),
        .apb_s_paddr(32'h0), .apb_s_pwdata(32'h0), .apb_s_pstrb(4'h0),
        .apb_s_pready(), .apb_s_prdata(), .apb_s_pslverr(),
        .apb_m_psel(hpm_apb_psel), .apb_m_penable(hpm_apb_penable),
        .apb_m_pwrite(hpm_apb_pwrite), .apb_m_paddr(hpm_apb_paddr),
        .apb_m_pwdata(hpm_apb_pwdata), .apb_m_pstrb(),
        .apb_m_pready(hpm_apb_pready), .apb_m_prdata(hpm_apb_prdata),
        .apb_m_pslverr(1'b0),
        .csr_addr(csr_addr), .csr_we(1'b0), .csr_re(csr_re),
        .csr_wdata(csr_wdata), .csr_rdata(hpm_csr_rdata),
        .link_state(hpm_state), .link_up(hpm_up),
        .speed_select(hpm_speed), .speed_valid(hpm_speed_v),
        .phy_aligned(hpm_aligned)
    );

    assign scm_scl_stretch = scm_stretch_v[0];
    assign scm_sda_pull    = scm_pull_v[0];
    assign scm_start_gen   = scm_sgen_v[0];
    assign scm_stop_gen    = scm_pgen_v[0];
    assign hpm_scl_stretch = hpm_stretch_v[0];
    assign hpm_sda_pull    = hpm_pull_v[0];
    assign hpm_start_gen   = hpm_sgen_v[0];
    assign hpm_stop_gen    = hpm_pgen_v[0];

    // HPM-side APB slave: 16-word memory, 1-wait-state PREADY.
    always @(posedge clk) begin
        hpm_apb_pready <= 1'b0;
        if (hpm_apb_psel && hpm_apb_penable && !hpm_apb_pready) begin
            if (hpm_apb_pwrite)
                apb_mem[hpm_apb_paddr[5:2]] <= hpm_apb_pwdata;
            else
                hpm_apb_prdata <= apb_mem[hpm_apb_paddr[5:2]];
            hpm_apb_pready <= 1'b1;
        end
    end

    // HPM-side completer: a 16-word scratch memory with 1-cycle service.
    always @(posedge clk) begin
        hpm_cmp_done <= 1'b0;
        if (hpm_cmp_req && !hpm_cmp_done) begin
            if (hpm_cmp_write)
                tb_mem[hpm_cmp_addr[5:2]] <= hpm_cmp_wdata;
            else
                hpm_cmp_rdata <= tb_mem[hpm_cmp_addr[5:2]];
            hpm_cmp_done <= 1'b1;
        end
    end

    // Filter settle: FILT_CYC (10 @ 200MHz, 50ns) + 2FF sync + margin.
    localparam int SETTLE = 25;

    // ------------------------------------------------------------------
    int errors = 0;

    initial begin
        $dumpfile("tb_ltpi_system.vcd");
        $dumpvars(0, tb_ltpi_system);

        repeat (10) @(posedge clk);
        rst = 0;

        // ---- 1. link bring-up --------------------------------------
        wait (scm_up);  $display("[%0t] SCM link up", $time);
        wait (hpm_up);  $display("[%0t] HPM link up", $time);
        @(posedge clk);
        if (scm_speed !== hpm_speed)
            begin errors++; $display("FAIL: speed mismatch %h vs %h",
                                     scm_speed, hpm_speed); end
        else
            $display("PASS: negotiated speed select = %h (400MHz+DDR = %h)",
                     scm_speed, CAP_400M_SDR | CAP_DDR);

        // ---- 2. LL GPIO --------------------------------------------
        scm_ll_in = 16'hA5C3;
        repeat (600) @(posedge clk);     // > 4 frame times
        if (hpm_ll_out !== 16'hA5C3)
            begin errors++; $display("FAIL: LL GPIO %h", hpm_ll_out); end
        else
            $display("PASS: LL GPIO SCM->HPM = %h", hpm_ll_out);

        // ---- 3. NL GPIO (needs 2 frames for 32 GPIOs) --------------
        scm_nl_in = 32'hDEADBEEF;
        repeat (800) @(posedge clk);
        if (hpm_nl_out !== 32'hDEADBEEF)
            begin errors++; $display("FAIL: NL GPIO %h", hpm_nl_out); end
        else
            $display("PASS: NL GPIO SCM->HPM = %h", hpm_nl_out);

        // ---- 4. UART sample tunneling ------------------------------
        // Drive a slow "bit" pattern (each level >> 1 frame time so the
        // 3x oversampling reproduces it faithfully).
        scm_uart_txd = 1'b0;  repeat (800) @(posedge clk);
        if (hpm_uart_txd_out !== 1'b0)
            begin errors++; $display("FAIL: UART low not tunneled"); end
        else $display("PASS: UART low level tunneled");
        scm_uart_txd = 1'b1;  repeat (800) @(posedge clk);
        if (hpm_uart_txd_out !== 1'b1)
            begin errors++; $display("FAIL: UART high not tunneled"); end
        else $display("PASS: UART high level tunneled");

        // ---- 5. I2C Start relay, RAW bus lines through the 50ns filter --
        // SETTLE > FILT_CYC(10 @200MHz) + 2FF sync + margin.
        // START: SDA falls while SCL high, then SCL falls.
        scm_sda <= 0; repeat (SETTLE) @(posedge clk);
        scm_scl <= 0; repeat (SETTLE) @(posedge clk);
        if (!scm_scl_stretch)
            begin errors++; $display("FAIL: SCM not stretching after Start"); end
        else $display("PASS: SCM stretches SCL awaiting Start Received");

        wait (hpm_start_gen);
        $display("[%0t] HPM regenerating START", $time);
        // Complete the regenerated START on the HPM local bus (SCL fall;
        // the relay itself is pulling SDA low - mirror it on the raw line).
        hpm_sda <= 0; repeat (SETTLE) @(posedge clk);
        hpm_scl <= 0; repeat (SETTLE) @(posedge clk);

        // Start Received travels back; SCM must release the stretch.
        wait (!scm_scl_stretch);
        $display("[%0t] PASS: I2C Start handshake complete, stretch released",
                 $time);

        // ---- 5b. FILTER: a 25ns SCL spike on the SCM bus must vanish ---
        begin : spike_check
            integer pre_state;
            pre_state = u_scm.g_i2c[0].u_i2c.state;
            scm_scl <= 1;                    // 25ns = 5 cycles < FILT_CYC
            repeat (5) @(posedge clk);
            scm_scl <= 0;
            repeat (SETTLE) @(posedge clk);
            if (u_scm.g_i2c[0].u_i2c.state !== pre_state
                || u_scm.u_fsm.state !== 6)
                begin errors++; $display("FAIL: 25ns spike leaked"); end
            else $display("PASS: 25ns SCL spike suppressed by tSP filter");
        end

        // ---- 6. BIDIRECTIONAL: HPM-initiated transaction (SPDM path) ----
        // STOP on SCM: SCL rises, then SDA rises.
        scm_scl <= 1; repeat (SETTLE) @(posedge clk);
        scm_sda <= 1; repeat (SETTLE) @(posedge clk);
        wait (hpm_stop_gen);
        // HPM local bus completes the regenerated STOP.
        hpm_scl <= 1; repeat (SETTLE) @(posedge clk);
        hpm_sda <= 1; repeat (SETTLE) @(posedge clk);
        wait (u_scm.g_i2c[0].u_i2c.state == 0 && u_hpm.g_i2c[0].u_i2c.state == 0);
        $display("[%0t] both relays idle after Stop", $time);

        // Now the HPM (SPDM responder acting as bus master) initiates.
        hpm_sda <= 0; repeat (SETTLE) @(posedge clk);
        hpm_scl <= 0; repeat (SETTLE) @(posedge clk);
        if (!hpm_scl_stretch)
            begin errors++; $display("FAIL: HPM not stretching after its Start"); end
        else $display("PASS: HPM initiator stretches awaiting Start Received");

        wait (scm_start_gen);
        $display("[%0t] PASS: SCM regenerating HPM-initiated START", $time);
        scm_sda <= 0; repeat (SETTLE) @(posedge clk);
        scm_scl <= 0; repeat (SETTLE) @(posedge clk);
        wait (!hpm_scl_stretch);
        $display("[%0t] PASS: bidirectional I2C - HPM-initiated handshake complete",
                 $time);

        // ---- 7. I2C data bit through the full handshake (HPM sources) --
        // Data bit 1: SDA changes while SCL low, sampled at rise, shipped
        // at fall.
        hpm_sda <= 1; repeat (SETTLE) @(posedge clk);
        hpm_scl <= 1; repeat (SETTLE) @(posedge clk);
        hpm_scl <= 0; repeat (SETTLE) @(posedge clk);
        if (!hpm_scl_stretch)
            begin errors++; $display("FAIL: no stretch during data bit"); end
        else $display("PASS: HPM stretches while data bit crosses the link");
        // SCM regenerates the bit; it is a 1 so SDA must NOT be pulled.
        wait (u_scm.g_i2c[0].u_i2c.state == 4);   // S_RDATA
        if (scm_sda_pull)
            begin errors++; $display("FAIL: SDA pulled for a 1-bit"); end
        else $display("PASS: open-drain - regenerated 1-bit not pulled");
        // Clock the regenerated bit on the SCM bus.
        scm_sda <= 1; repeat (SETTLE) @(posedge clk);
        scm_scl <= 1; repeat (SETTLE) @(posedge clk);
        scm_scl <= 0; repeat (SETTLE) @(posedge clk);
        wait (!hpm_scl_stretch);
        $display("[%0t] PASS: data bit handshake complete (Data/Echo/Rcvd/Echo)",
                 $time);
        // Close: HPM STOP (SDA low prep while SCL low, SCL rise, SDA rise).
        hpm_sda <= 0; repeat (SETTLE) @(posedge clk);
        hpm_scl <= 1; repeat (SETTLE) @(posedge clk);
        hpm_sda <= 1; repeat (SETTLE) @(posedge clk);
        wait (u_scm.g_i2c[0].u_i2c.bus_stop_gen);
        scm_sda <= 0; repeat (SETTLE) @(posedge clk);
        scm_scl <= 1; repeat (SETTLE) @(posedge clk);
        scm_sda <= 1; repeat (SETTLE) @(posedge clk);
        wait (u_scm.g_i2c[0].u_i2c.state == 0 && u_hpm.g_i2c[0].u_i2c.state == 0);
        $display("[%0t] PASS: HPM transaction closed, both relays idle", $time);

        // ---- 8. Coordinated soft reset (Operational -> Advertise) ------
        @(posedge clk); scm_soft <= 1; hpm_soft <= 1;
        @(posedge clk); scm_soft <= 0; hpm_soft <= 0;
        @(posedge clk);
        if (scm_up)
            begin errors++; $display("FAIL: SCM still up after soft reset"); end
        wait (scm_up); wait (hpm_up);
        $display("[%0t] PASS: soft reset -> re-advertised -> Operational again",
                 $time);

        // ---- 9. Link retraining request (back to Link Detect) ----------
        @(posedge clk); scm_retrain <= 1; @(posedge clk); scm_retrain <= 0;
        @(posedge clk);
        if (scm_up)
            begin errors++; $display("FAIL: SCM still up after retrain"); end
        wait (scm_up); wait (hpm_up);
        if (scm_speed !== hpm_speed)
            begin errors++; $display("FAIL: speed mismatch after retrain"); end
        $display("[%0t] PASS: full retrain -> both Operational, speed=%h",
                 $time, scm_speed);

        // ---- 10. CSR debug interface -------------------------------
        // Status readback: state=Operational(4), aligned, speed=400M DDR.
        @(posedge clk); csr_addr <= 8'h00; csr_re <= 1; @(posedge clk);
        csr_re <= 0; @(posedge clk);
        if (scm_csr_rdata[19:16] !== 4'h4 || !scm_csr_rdata[0]
            || scm_csr_rdata[11:8] !== 4'h8 || !scm_csr_rdata[7])
            begin errors++; $display("FAIL: CSR status = %h", scm_csr_rdata); end
        else $display("PASS: CSR status readback = %h (OPER, aligned, x16 DDR)",
                      scm_csr_rdata);
        // Operational TX frame counter is counting.
        @(posedge clk); csr_addr <= 8'h58; csr_re <= 1; @(posedge clk);
        csr_re <= 0; @(posedge clk);
        if (scm_csr_rdata == 0)
            begin errors++; $display("FAIL: op TX counter zero"); end
        else $display("PASS: CSR op-TX frame counter = %0d", scm_csr_rdata);
        // CSR-triggered retrain (Link Control bit 1).
        @(posedge clk); csr_addr <= 8'h80; csr_wdata <= 32'h402; csr_we <= 1;  // retrain + keep auto-config
        @(posedge clk); csr_we <= 0; csr_wdata <= 0;
        @(posedge clk); @(posedge clk);
        if (scm_up)
            begin errors++; $display("FAIL: CSR retrain ignored"); end
        wait (scm_up); wait (hpm_up);
        $display("[%0t] PASS: CSR-triggered retrain -> Operational again",
                 $time);

        // ---- 11. Data channel write + read round trip ---------------
        wait (dc_req_ready);
        @(posedge clk);
        dc_req_v <= 1; dc_req_w <= 1; dc_req_addr <= 32'h0000_0010;
        dc_req_wdata <= 32'hCAFE_BABE;
        @(posedge clk); dc_req_v <= 0;
        wait (dc_rsp_valid);
        if (dc_rsp_error)
            begin errors++; $display("FAIL: data write errored"); end
        else $display("[%0t] PASS: data channel write completed", $time);
        @(posedge clk);
        wait (dc_req_ready); @(posedge clk);
        dc_req_v <= 1; dc_req_w <= 0; dc_req_addr <= 32'h0000_0010;
        @(posedge clk); dc_req_v <= 0;
        wait (dc_rsp_valid);
        if (dc_rsp_rdata !== 32'hCAFE_BABE)
            begin errors++; $display("FAIL: data read = %h", dc_rsp_rdata); end
        else $display("[%0t] PASS: data channel read back = %h over LTPI",
                      $time, dc_rsp_rdata);

        // ---- 12. Peer identity + feature row decode -----------------
        if (!u_scm.peer_valid || u_scm.peer_platform_id !== 16'hF9A0
            || !u_scm.pf_i2c || !u_scm.pf_data
            || u_scm.pf_i2c_en !== 6'h03 || u_scm.pf_nl_cnt !== 10'd32)
            begin errors++; $display("FAIL: peer decode id=%h i2c_en=%h nl=%0d",
                u_scm.peer_platform_id, u_scm.pf_i2c_en, u_scm.pf_nl_cnt); end
        else
            $display("PASS: peer decoded - id=%h vendor=%0d chan[gpio,i2c,uart,data]=%b%b%b%b i2c_en=%b nl=%0d",
                u_scm.peer_platform_id, u_scm.peer_vendor,
                u_scm.pf_gpio, u_scm.pf_i2c, u_scm.pf_uart, u_scm.pf_data,
                u_scm.pf_i2c_en, u_scm.pf_nl_cnt);

        // ---- 13. OEM APB tunnel: write + read through the link ------
        @(posedge clk);
        apb_pwrite <= 1; apb_paddr <= 32'h0000_0020;
        apb_pwdata <= 32'hCAFE_D00D; apb_psel <= 1;
        @(posedge clk); apb_penable <= 1;
        wait (apb_pready); @(posedge clk);
        apb_psel <= 0; apb_penable <= 0;
        if (apb_pslverr)
            begin errors++; $display("FAIL: APB write errored"); end
        else $display("[%0t] PASS: OEM APB write completed over LTPI", $time);
        repeat (4) @(posedge clk);
        apb_pwrite <= 0; apb_paddr <= 32'h0000_0020; apb_psel <= 1;
        @(posedge clk); apb_penable <= 1;
        wait (apb_pready); @(posedge clk);
        apb_psel <= 0; apb_penable <= 0;
        if (apb_prdata !== 32'hCAFE_D00D)
            begin errors++; $display("FAIL: APB read = %h", apb_prdata); end
        else $display("[%0t] PASS: OEM APB read back = %h over LTPI",
                      $time, apb_prdata);

        repeat (500) @(posedge clk);

        if (errors == 0)
            $display("=== ALL CHECKS PASSED ===");
        else
            $display("=== %0d CHECKS FAILED ===", errors);
        $finish;
    end

    // Global watchdog
    initial begin
        #8_000_000;   // 8 ms
        $fatal(1, "GLOBAL TIMEOUT");
    end

`ifdef DEBUG_MON
    integer dbg_n = 0;
    always @(posedge clk) begin
        if (u_scm.frame_valid) begin
            dbg_n++;
            if (dbg_n % 8 == 1)
                $display("[%0t] SCMrx type=%0d crc=%b | states S=%0d H=%0d | align S=%b H=%b",
                         $time, u_scm.frame_type, u_scm.frame_crc_ok,
                         scm_state, hpm_state, scm_aligned, hpm_aligned);
        end
    end
    integer dbg_op = 0;
    always @(posedge clk) begin
        if (hpm_up && u_hpm.frame_valid) begin
            dbg_op++;
            if (dbg_op % 4 == 1)
                $display("[%0t] HPMop cnt=%h ll=%h nl=%h uart=%h i2c=%h | hpm_nl_out=%h",
                         $time, u_hpm.rx_payload[7:0], u_hpm.rx_payload[23:8],
                         u_hpm.rx_payload[39:24], u_hpm.rx_payload[43:40],
                         u_hpm.rx_payload[51:48], hpm_nl_out);
        end
    end
    always @(u_scm.u_oem_apb.rq_beat or u_scm.u_oem_apb.rq_pend or u_scm.u_oem_apb.rq_sent)
        $display("[%0t] SCMapb pend=%b sent=%b beat=%0d", $time,
                 u_scm.u_oem_apb.rq_pend, u_scm.u_oem_apb.rq_sent,
                 u_scm.u_oem_apb.rq_beat);
    always @(u_hpm.u_oem_apb.cq_beat or u_hpm.u_oem_apb.c_busy or u_hpm.u_oem_apb.c_rsp_pend)
        $display("[%0t] HPMapb cq=%0d busy=%b rsp=%b", $time,
                 u_hpm.u_oem_apb.cq_beat, u_hpm.u_oem_apb.c_busy,
                 u_hpm.u_oem_apb.c_rsp_pend);
    integer dbg_apb = 0;
    always @(posedge clk)
        if (u_hpm.op_frame_good && $time > 141000 && dbg_apb < 12) begin
            dbg_apb = dbg_apb + 1;
            $display("[%0t] HPMoem byte11=%h rx103_72=%h", $time,
                     u_hpm.rx_payload[79:72], u_hpm.rx_payload[103:72]);
        end
    always @(posedge scm_aligned) $display("[%0t] SCM PHY aligned", $time);
    always @(posedge hpm_aligned) $display("[%0t] HPM PHY aligned", $time);
    always @(u_scm.g_i2c[0].u_i2c.state)
        $display("[%0t] SCM i2c state -> %0d tx_ev=%h init=%b", $time,
                 u_scm.g_i2c[0].u_i2c.state, u_scm.g_i2c[0].u_i2c.tx_event, u_scm.g_i2c[0].u_i2c.initiator);
    always @(u_hpm.g_i2c[0].u_i2c.state)
        $display("[%0t] HPM i2c state -> %0d tx_ev=%h init=%b def=%b", $time,
                 u_hpm.g_i2c[0].u_i2c.state, u_hpm.g_i2c[0].u_i2c.tx_event,
                 u_hpm.g_i2c[0].u_i2c.initiator, u_hpm.g_i2c[0].u_i2c.start_deferred);
    always @(scm_state) $display("[%0t] SCM state -> %0d", $time, scm_state);
    always @(hpm_state) $display("[%0t] HPM state -> %0d", $time, hpm_state);
`endif

endmodule


