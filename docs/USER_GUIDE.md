# LTPI IP User Guide

**Verified to: OCP DC-SCM 2.2 LTPI, Version 1.0 (Revision 1.2, Aug 11
2025)** — including every normative change in the 2.1v1.0 / 2.1v1.1 /
2.2v1.0 revision history. Clause-by-clause mapping: `TRACEABILITY.md`.

## 1. What you instantiate

One `ltpi_top` per end of the link:

```systemverilog
ltpi_top #(
    .ROLE_SCM     (1'b1),          // 1 = SCM CPLD side, 0 = HPM FPGA side
    .NL_TOTAL     (32),            // normal-latency GPIOs (1..1024)
    .NUM_I2C      (2),             // I2C/SMBus relay channels (1..6)
    .PLATFORM_ID  (16'hF9A0),      // Table 26 OEM ID you advertise
    .ADV_MIN_CYCLES    (25000),    // 1 ms  @ your link clock
    .ADV_ALIGN_TIMEOUT (2_500_000) // 100 ms @ your link clock (2.1v1.0)
) u_ltpi ( ... );
```

Wire `ser_tx_*`/`ser_rx_*` through `rtl/vendor/ltpi_lvds_io.sv`
(`+define+LTPI_VENDOR_ALTERA` or `LTPI_VENDOR_LATTICE`), apply the matching
constraints file from `rtl/vendor/`, feed a PLL-generated link clock, and
release `rst`. Training runs by itself; `link_up` rises when Operational.

Capabilities: drive `local_speed_caps` with a `CAPS_*` constant from
`ltpi_pkg` — `CAPS_DEFAULT` (25M + 200M/400M SDR + DDR) or `CAPS_FULL`
(every Table 21 rate to 1 GHz DDR). Negotiation picks the highest rate both
ends advertise, so only advertise what your part's I/O closes (see the
part tables in the constraints files).

## 2. Channels

| Channel | Ports | Notes |
|---|---|---|
| LL GPIO | `ll_gpio_in/out[15:0]` | refreshed every frame |
| NL GPIO | `nl_gpio_in/out[NL_TOTAL-1:0]` | multiplexed over ⌈NL_TOTAL/16⌉ frames |
| UART | `uart_txd_in/out`, `uart_flow_*` | 3× oversampled; flow bit per frame |
| I2C/SMBus | `i2c_scl_in/i2c_sda_in[NUM_I2C-1:0]` | RAW bus levels in - each channel has a built-in proven 50ns tSP filter + edge detect (`ltpi_i2c_cond`); **either side may initiate** (SPDM/MCTP-ready); obey `scl_stretch`/`sda_pull` as open-drain pull-downs. Speed envelope: `I2C_LIMITATIONS.md` (>=400Mbps recommended) |
| Data | `dc_req_*` (requester), `dc_cmp_*` (completer) | 32-bit R/W with byte enables, single outstanding, tag-tracked; AVMM/APB-shaped |
| OEM APB | `apb_s_*` (true APB completer), `apb_m_*` (true APB requester) | AMBA APB tunneled in I/O-frame OEM bytes 11-14; point a local APB master at `apb_s_*`, the far side drives its APB slave; APB protocol formally proven |

**Adding an I2C channel** = bump `NUM_I2C` and connect the new index —
the generate block, frame nibble packing (bytes 8–10), pacing, and the
advertised enable mask all follow automatically.

## 3. CSR debug interface (hardware bring-up)

Simple synchronous port (`csr_addr/we/re/wdata/rdata`, byte offsets,
32-bit) — drop behind APB/AVMM/AXI-lite or a JTAG/I2C register bridge.

| Offset | Register |
|---|---|
| 0x00 | Link Status: [19:16] local state (0=Detect…4=Operational), [15:12] remote-state estimate, [11:8] speed code, [7] DDR, [5:1] sticky RWC errors (cfg-timeout, speed-timeout, unknown-comma, CRC, link-lost), [0] PHY aligned |
| 0x04 | Local caps (RW override — BMC may *restrict* speeds for debug) |
| 0x08 | Remote speed caps |
| 0x0C / 0x10 | Local platform ID / **peer platform ID + decoded vendor** ([18:16] vendor code, [19] peer-valid) |
| 0x1C / 0x20 | **Peer feature row** (advertised Table 28): channels, NL count, per-I2C enables+speeds / UART baud-flow-enables, OEM caps |
| 0x2C–0x40 | Error counters (RWC): alignment, link-lost, CRC, unknown comma, speed timeout, cfg timeout |
| 0x54 / 0x58 | Operational RX / TX frame counters (RWC) |
| 0x80 | Link Control: [0] W1 soft reset, [1] W1 retrain, [10] auto-configure |

### Peer identification (e.g. ASPEED BMC on the far end)

Read 0x10: `peer_valid` + vendor code (1=this core, 2=ASPEED AST1700-class,
3=Lattice, 4=OCP/Intel ref, 5=Microchip, 0=unknown) decoded from the
Advertise Platform ID. **Platform IDs are OEM-defined with no public
registry** — the ROM in `ltpi_pkg.sv` ships placeholder IDs; align them
with each partner's platform agreement. The feature row at 0x1C/0x20 is
spec-defined (Table 28) and trustworthy regardless of vendor: use it to
decide which channels to enable against any compliant peer.

## 4. Simulating

```powershell
# self-checking system loopback (two endpoints, 12 test groups)
cd sim
iverilog -g2012 -I ../rtl -o tb.vvp ../rtl/ltpi_pkg.sv ../rtl/ltpi_frame_rx.sv \
  ../rtl/ltpi_frame_tx.sv ../rtl/ltpi_link_fsm.sv ../rtl/ltpi_gpio_channel.sv \
  ../rtl/ltpi_uart_channel.sv ../rtl/ltpi_i2c_cond.sv ../rtl/ltpi_i2c_relay.sv \
  ../rtl/ltpi_phy.sv ../rtl/ltpi_csr.sv ../rtl/ltpi_data_channel.sv \
  ../rtl/ltpi_peer_decode.sv ../rtl/ltpi_oem_apb.sv ../rtl/ltpi_top.sv \
  tb_ltpi_system.sv
vvp tb.vvp                    # expect "=== ALL CHECKS PASSED ==="
python render_waveform.py     # timing-diagram PNG from the VCD
```

Compile with `-DDEBUG_MON` for live state/frame trace prints. Verilator
(coverage) and the UVM bench (`uvm/`, Questa/VCS/Xcelium) are described in
the README. Formal: `cd formal && sby -f <suite>.sby`.

## 5. Customizing

- **Thresholds**: every Table 37/38/42/43/47 constant is a parameter;
  shrink them in sims (the TB shows working values), keep spec defaults in
  hardware. `ADV_MIN_CYCLES`/`ADV_ALIGN_TIMEOUT` are in link-clock cycles.
- **Speeds**: compose `CAP_*` constants; add DDR only if both your I/O and
  the constraint budget close (part tables in `rtl/vendor/*.sdc/.lpf`).
- **Vendors/peers**: extend the `PLATID_*` table + `vendor_of()` in
  `ltpi_pkg.sv`/`ltpi_peer_decode.sv` — re-run
  `sby -f ltpi_peer_decode.sby` (the totality proof PD3 keeps you honest).
- **New frame fields / OEM channels**: extend the payload mux in
  `ltpi_top` (bytes 11–14 are OEM-reserved in I/O frames) and mirror the
  decode on RX; follow the pacing pattern if events must survive framing.
- **After ANY RTL change**: re-run the full formal regression (all
  `.sby` suites) and the system sim; both must be clean.

## 6. Troubleshooting (via CSR + waveforms)

| Symptom | Check | Likely cause |
|---|---|---|
| `link_up` never rises | 0x00[0] aligned? | no RX bit stream / PLL not locked / pairs swapped |
| Aligned but stuck in Detect (state 0) | 0x34 CRC counter climbing? | polarity inversion, wrong bit order, marginal timing — see constraints windows |
| Stuck in Advertise (state 2) | 0x00[15:12] remote state | SCM: `cfg_ready` (or CSR 0x80[10]) low; peer stuck earlier |
| Trains then drops repeatedly | 0x30 link-lost + 0x2C alignment counters | noise/timing at the negotiated rate — restrict caps via 0x04 and retrain (0x80[1]) to bisect the failing speed |
| I2C bus hangs (SCL held low) | relay stretch = protocol wait | peer not answering events: check both ends' 0x54/0x58 counters advance; a stretch that never releases means the far bus never produced its edge (`scl_fall` detector wiring) |
| Data channel `req_ready` stuck low | response lost | check link-lost counters; single-outstanding model — reset via retrain if the peer dropped the response |
| Wrong/odd peer behavior | 0x10 vendor + 0x1C/0x20 feature row | peer advertises fewer channels than you assumed — gate features on the decoded row |
| Nothing in sim, `$dumpvars` empty | Verilator needs `--trace`; Icarus writes `tb_ltpi_system.vcd` by default | |

Waveform-first debugging: `sim/render_waveform.py` gives an at-a-glance
training/state/channel picture; GTKWave on the VCD for bit-level dives.
