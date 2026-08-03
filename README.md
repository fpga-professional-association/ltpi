# Formally Verified LTPI Implementation (OCP DC-SCM 2.0)

SystemVerilog implementation, machine-checked formal proofs, simulation, and
UVM testbench for the **LVDS Tunneling Protocol & Interface (LTPI)**.

> **Verified to: OCP DC-SCM 2.2 LTPI, Version 1.0 (Revision 1.2, Aug 11
> 2025)** - the latest public release, including all normative changes of
> the 2.1v1.0 / 2.1v1.1 / 2.2v1.0 revision history. See
> `docs/TRACEABILITY.md` for the clause-by-clause map and
> `docs/USER_GUIDE.md` for integration, simulation, customization, and
> troubleshooting.

Targets **both Altera/Intel and Lattice FPGAs** (vendor-portable RTL + thin
I/O wrappers) at **200 MHz SDR (200 Mbps), 400 MHz SDR (400 Mbps), or
400 MHz DDR (800 Mbps)** — the capability word `CAPS_DEFAULT` advertises all
three and training negotiates the fastest common rate.

## Layout

```
rtl/   ltpi_pkg.sv           constants, enums, capability words
       ltpi_crc8_func.svh    CRC-8 x^8+x^2+x+1 (shared include)
       ltpi_crc8.sv          streaming CRC generator/checker
       ltpi_frame_rx.sv      16-byte frame assembly + CRC + classify
       ltpi_frame_tx.sv      frame serializer (always emits valid CRC)
       ltpi_link_fsm.sv      link training/config/operational FSM (SCM+HPM)
       ltpi_gpio_channel.sv  LL GPIO (every frame) + NL GPIO (N-frame mux)
       ltpi_uart_channel.sv  UART 3x-oversample tunneling + RTS/CTS
       ltpi_i2c_relay.sv     I2C/SMBus event relay w/ clock stretching
       ltpi_phy.sv           SDR/DDR serdes, comma hunt, DDR bitslip
       ltpi_top.sv           full endpoint: PHY+framer+FSM+channels
       vendor/               ALTDDIO / ODDRX1F wrappers, SDC + LPF constraints
formal/  *.sby               SymbiYosys proofs (see table below)
         ltpi_loopback.sv    SCM+HPM composition proof
sim/   tb_ltpi_system.sv     two endpoints, serial cross-connect, self-checking
       render_waveform.py    VCD -> timing-diagram PNG
uvm/   ltpi_if.sv, ltpi_uvm_pkg.sv, tb_uvm_top.sv, run_{questa,vcs,xcelium}
```

## Proof suites (all PASS)

| Suite | Tasks | What is proven |
|---|---|---|
| `ltpi_crc8` | bmc, prove | zero-remainder theorem, 0xF4 known answer, register semantics |
| `ltpi_frame_rx` | bmc, prove, cover | 16-byte cadence, CRC accumulator isolation, alignment reset; covers construct a real CRC-valid frame |
| `ltpi_frame_tx` | bmc, prove, cover | **every emitted frame carries a correct CRC** (shadow-CRC refinement), legal comma, cadence |
| `ltpi_link_fsm` | {scm,hpm} × {bmc, prove, cover} | P1–P12: state validity, handshake-gated Operational entry, Table 37/38/42/43/47 exit disciplines, one-hot supported speed select, counter bounds |
| `ltpi_gpio_channel` | bmc, prove, cover | hold-on-CRC-error, exact NL slice addressing (no cross-slice writes), TX mux correctness incl. partial last slice |
| `ltpi_uart_channel` | bmc, prove, cover | capture order D[0]=oldest, in-order replay, hold-on-error |
| `ltpi_i2c_relay` | {pri,sec} × {bmc, prove, cover} | R1–R10: legal events, **stretch whenever awaiting the far side or deferring**, open-drain integrity (a tunneled '1' is never pulled low), full-handshake-before-next-bit, Stop discipline, arbitration back-off, Data-Received-only-after-real-SCL-edge (target stretch safe); covers: initiator round trip on EITHER side, remote bit, ACK slot, read decode, deferred-then-claimed START |
| `ltpi_i2c_loopback` | bmc, prove, cover | **two-relay composition**: bus-mastership mutual exclusion proven unbounded (never two active initiators), responder-event lemmas; covers: **HPM-initiated transaction (the SPDM response path)**, defer-then-claim, simultaneous-start race resolved by priority |
| `ltpi_phy` | {tx,rx} × {bmc, prove, cover} | serializer order/losslessness, RX byte = exact wire window, comma-first after alignment, DDR odd-offset bitslip, realign |
| `ltpi_loopback` | bmc, prove, cover | **SCM↔HPM composition**: L1–L4 + T1–T3 — Accept implies Configure happened, HPM-Operational implies SCM-Operational, **speed agreement** (both sides always select the same one-hot mutually-supported speed, incl. the adopt-peer-select path), DDR only when both capable; covers: full bring-up, 400 MHz DDR negotiation, 200 MHz SDR fallback |

Run everything: activate `..\..\oss-cad-suite\environment.ps1`, then
`sby -f <suite>.sby` in `formal/`.

### Spec-level findings closed by this implementation

1. **Speed-select divergence** (link FSM composition proof): a side exiting
   Link Detect via a received Link Speed frame may never have seen the
   peer's capability word, so it must **adopt the Speed Select payload of
   that frame** (Table 24) instead of computing its own — otherwise the two
   sides can permanently disagree on speed. Implemented as sanitized
   adoption.
2. **I2C bidirectionality for SPDM** — LTPI 1.0 fixes the I2C controller
   on the SCM side, but SPDM over MCTP/SMBus requires *both* endpoints to
   master the bus (the SPDM responder initiates its own response
   transfers). `ltpi_i2c_relay` therefore claims the initiator role **per
   transaction** on either side, with deterministic simultaneous-start
   arbitration (`ARB_PRIORITY`, SCM wins) and **START deferral via clock
   stretching** for the losing/busy side — the same mechanism the spec
   prescribes for a START racing a STOP. Clock stretching works in both
   directions: the initiator's bus is held until the peer confirms
   regeneration (R3), and a stretching target on the responder side simply
   delays the real SCL edge that gates Data Received (R10), which
   propagates back as initiator-side stretch. Mutual exclusion of the
   mastership claim is proven unbounded in `ltpi_i2c_loopback`.
3. **Event-change semantics**: LTPI events repeat in every frame ("sent
   continuously until new event is generated"), so relay transitions must
   fire on event *changes*, never levels — otherwise a responder
   re-triggers forever on the repeated Start event. The Echo events exist
   precisely to separate identical consecutive events; the relay keys every
   protocol transition on `rx_new` (delivered value differs from the
   previous delivered value).

## Simulation (`sim/`)

`tb_ltpi_system.sv` connects two complete endpoints at the serial-bit level
and self-checks: training to Operational on both sides, 0x8100 (400 MHz+DDR)
negotiation, LL GPIO, NL GPIO (multi-frame mux), UART levels, the
SCM-initiated I2C Start/Stop handshake with clock stretching, **and an
HPM-initiated I2C transaction** (the SPDM response direction: HPM claims
the bus, stretches awaiting Start Received, SCM regenerates the START).
**ALL CHECKS PASSED** under Icarus:

```powershell
cd sim
iverilog -g2012 -I ../rtl -o tb.vvp ../rtl/*.sv tb_ltpi_system.sv
vvp tb.vvp        # writes tb_ltpi_system.vcd
python render_waveform.py   # renders ltpi_loopback_waveform.png
```

## UVM testbench (`uvm/`)

Reactive-peer architecture: the DUT is a full SCM endpoint; the UVM agent
emulates the HPM at bit level (driver serializes CRC-correct — or
deliberately corrupted — frames; monitors reassemble both directions).
Scoreboard checks DUT frames are always CRC-clean and protocol-ordered;
functional coverage bins frame types, link states, and negotiated speeds.
Tests: `ltpi_bringup_test` (clean bring-up + GPIO traffic, expects 400 MHz
DDR) and `ltpi_crc_error_test` (link must survive 10% frame corruption).
Requires a UVM-1.2 simulator (Questa/VCS/Xcelium — run scripts included);
UVM does not run under Icarus/mainline Verilator, so these sources are
compile-targeted at those tools rather than validated here.

## FPGA integration

* Instantiate `ltpi_top` + `vendor/ltpi_lvds_io.sv` with
  `+define+LTPI_VENDOR_ALTERA` or `+define+LTPI_VENDOR_LATTICE`.
* Constraints: `vendor/constraints_altera.sdc` (Quartus) /
  `vendor/constraints_lattice.lpf` (Diamond); pick the 200/400 MHz block.
* `ddr_mode` + PLL reconfiguration on speed switch are system-level: after
  training, read `speed_select` (bit15 = DDR, bit8 = 400 MHz, bit5 =
  200 MHz), reprogram the PLL, and let the Advertise-phase re-alignment
  (1 ms window, spec Note 4) absorb the lock time.
* 800 Mbps DDR needs Cyclone 10 GX-class LVDS or ECP5-5G/Avant with DELAYG
  tuning; 200/400 SDR close on mainstream devices of both vendors.

## Coverage

**Code coverage** (Verilator `--coverage`, system TB `sim/tb_ltpi_system.sv`,
11 self-checking test groups): **89.4% line coverage** overall —
`ltpi_frame_tx`, `ltpi_gpio_channel`, `ltpi_uart_channel`, `crc8` at 100%;
`ltpi_link_fsm` 90.8%; `ltpi_i2c_relay` 84.0%. Residual gaps are
DDR/bitslip PHY paths, invalid-frame arms and adopt-select (all covered by
formal proofs instead), plus unread CSR addresses. Rebuild:
`sim/` → `verilator --binary --timing --coverage -CFLAGS
-fno-declone-ctor-dtor ...`, report via `verilator_coverage` +
`sim/cov_summary.py`.

**Formal property inventory**: 127 assertions, 30 assumptions, 39 cover
points across 12 proof suites; every cover point is reached (a cover task
fails otherwise), so 100% cover-point reachability.

**Mutation coverage** (`formal/mutation_coverage.py`, mcy methodology:
inject single-operator bugs outside the FORMAL blocks, expect the
verification suite to notice): 62 mutants across 6 modules.

| Kill stage | killed | cumulative |
|---|---|---|
| Module formal properties (BMC) | 21 | 33.9% |
| + Composition proofs (loopback suites) | +9 | 48.4% |
| + Self-checking system simulation | +9 | **62.9%** |

Survivor analysis (23, listed in `formal/mutation_report.txt`): about half
are *equivalent or benign* mutants — loop-bound `<`→`<=` writing an
ignored out-of-range bit, the TX frame counter `+`→`-` (the receiver
decodes each frame's counter value, so ordering is immaterial), and
`>=`→`>` off-by-ones on "at least N" spec thresholds. The remainder are
genuine detection gaps concentrated where module assertions share decode
cones with the RTL (a mutation shifts both, so the property is
tautological) and in paths the sim doesn't drive (I2C read-direction
decode, arbitration race, fast UART edges). The composition proofs kill
where module properties can't precisely because they judge each side
against an independent reference — the right place to grow the property
set further.

## Vendor parity matrix

Compared against the three public LTPI implementations' user guides
(OCP/Intel `LTPI_User_Guide.pdf` rev 1.1; Lattice DC-SCM LTPI IP
FPGA-IPUG-02200 / Radiant IP 1.5.x docs; Microchip CoreLTPI UG):

| Feature | OCP/Intel ref | Lattice IP | Microchip CoreLTPI | **This core** |
|---|---|---|---|---|
| LL GPIO (16) | ✓ | ✓ | ✓ | ✓ proven |
| NL GPIO | ≤1024 | ✓ | ✓ | ✓ parameterized, proven |
| UART channels | 2 | ≤24 | – | 1 lane instantiated (Table 8 field for 2; add a 2nd instance on byte 7 hi-nibble) |
| I2C/SMBus channels | 6 | ✓ (+MCTP) | – | 1 instantiated (frame fields for 6); **bidirectional + clock stretch proven** — exceeds all three (needed for MCTP/SPDM) |
| Data channel | AVMM + mailbox | APB | – | ✓ AVMM/APB-style, tag-tracked, proven (D1–D5) + end-to-end sim |
| CSR block | ✓ (AVMM, peakRDL) | ✓ | ✓ | ✓ Table 36 subset proven (status/state/speed, RWC errors, counters, link control) |
| Link training | ✓ | ✓ | GPIO-profile | ✓ proven both roles + composition |
| Speeds | 25–1000MHz | ✓ | input-clock based | 25M/200M/400M SDR + 400M DDR wired; caps word extensible to all Table 21 rates |
| Debug | debug ports | debug ports | – | CSR debug regs + status outputs + VCD flows |
| Formal proofs | – | – | – | **12 suites, unbounded induction** (unique to this core) |

Known deltas: OEM channels/frames and the 6-channel I2C / 24-UART scale
are structural replication of the proven single-channel modules (frame
bytes 8-10 and 7-hi are reserved for them in the payload mux); the
spec's full CSR register file beyond the Table 36 subset (platform IDs,
advertise capability mirrors) follows the same proven RWC/RO patterns.

## Known deviations / notes

* Table 43 (31 Configure frames max) is followed where §4.1.2.2 text says 32.
* Frame-alignment loss inside policing states retrains immediately
  (conservative vs. counting into the 3/7 lost-frame budgets).
* UART sample positions use 0/5/10 of the byte counter (spec Fig. 15 is an
  example distribution, not normative).
* 8b/10b symbol coding is delegated to the SERDES/PHY layer; the byte-domain
  logic here treats commas as reserved byte values, matching the frame-level
  behavior of the spec's reference flow.
