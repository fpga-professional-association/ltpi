# LTPI IP Core — DC-SCM 2.0 LVDS Tunneling Protocol & Interface (SCM + HPM)

[![verify](https://github.com/fpga-professional-association/ltpi/actions/workflows/verify.yml/badge.svg)](https://github.com/fpga-professional-association/ltpi/actions/workflows/verify.yml)

A device-agnostic SystemVerilog implementation of the **OCP DC-SCM 2.0 LTPI**
(LVDS Tunneling Protocol & Interface, Revision 1.0) for **both link endpoints** —
the **SCM** (Secure Control Module, drives link training) and the **HPM** (Host
Processor Module, responds). LTPI tunnels low-speed platform-management signals
(GPIO, UART, I2C/SMBus, a memory-mapped Data channel) between the DC-SCM and the
host board over a single source-synchronous LVDS serial pair per direction, using
8b/10b-framed 16-symbol frames.

**Dual-rate**: the link trains at the **25 MHz base** (SDR), then switches to the
negotiated **operational rate — up to 400 MHz DDR (800 Mbps)** — exactly as the
spec intends (Fig 19). The high-rate bit (de)serialization is done by a **J=10
SERDES**, so the **symbol-parallel core runs in the fabric at the parallel symbol
clock** (line rate / 10 = 80 MHz at 400 MHz DDR), bridged by **asynchronous CDC
FIFOs** to the System Clock. Both roles are the *same* parameterized core
(`ROLE = SCM | HPM`); the only device-specific block is the LVDS SERDES + IOPLL
in `ltpi_phy`.

The design is checked **three independent ways**:

| Method | Tool | Result |
|---|---|---|
| **Formal** | yosys 0.66 + SymbiYosys + boolector | **ALL GREEN** — 8 configs / 10 proof tasks: 8b/10b round-trip & DC-balance, **frame-layer round-trip + incremental CRC**, **CDC-FIFO safety**, link-FSM safety (k-induction) **and reachability to Operational**, GPIO/I2C/Data/CSR safety |
| **Simulation** | Icarus Verilog (live dual-rate SCM↔HPM link) | **11 / 11 PASS** — trains at 25 MHz, **switches to 400 MHz DDR**, re-aligns, reaches Operational, then LL+NL GPIO, UART, I2C event relay, Data-channel Avalon read **at the operational rate** |
| **Synthesis / STA** | Altera Quartus Prime Pro 25.3, Cyclone 10 GX `10CX220YF780E5G` | **Symbol-parallel core meets timing at 100 MHz** (+1.0 ns slack, Fmax ≈ 111 MHz) — comfortably above the **80 MHz parallel clock** that 400 MHz DDR requires; 1,147 ALMs (1 %), 1,357 regs, 0 RAM/DSP. The high-rate bit SERDES is the Cyclone 10 GX LVDS SERDES hard block (≤1.434 Gbps) |

---

## What it implements (spec coverage)

- **Link state machine** (spec §4, Fig 27): **Link Detect → Link Speed → Advertise →
  Configure (SCM) / Accept (HPM) → Operational**, with the exact frame counts and
  timeouts (255 Detect TX, 7/3 consecutive correct RX, 3-frame alignment, 7/3 Speed,
  ≥1 ms Advertise dwell, 31/15 Configure/Accept, 3/7 consecutive-lost link-loss),
  highest-common speed selection, and SCM/HPM training-skew tolerance (Notes 2/3).
- **8b/10b symbol layer** (§2.6): full IBM 5B/6B + 3B/4B codec with running disparity,
  K28.5/K28.6/K28.7 commas, a 1-bit serializer, and a comma-aligning deserializer.
- **16-symbol frames** (§3): Link Detect/Speed, Advertise/Configure/Accept, and
  Operational I/O + Data frames, each with a **CRC-8** (poly `x⁸+x²+x+1`, init 0,
  over bytes 1..14 — §2.4).
- **Channels** tunneled in the I/O frame:
  - **GPIO** (§2.2.1.1): 16 Low-Latency GPIOs every frame + Normal-Latency GPIOs
    time-multiplexed 16/frame by the frame counter; hold-last on CRC error.
  - **UART** (§2.2.1.2): 2 links, TXD/RXD 3× oversampled with RTS/CTS flow control.
  - **I2C/SMBus** (§2.2.1.3): per-link event relay (Start/Stop/Data + echo/received
    handshake, Table 10) with SCL clock-stretching and a single-owner bus drive.
  - **Data channel** (§2.2.1.4): Avalon-MM Read/Write tunneling with Tag tracking,
    Read/Write Completion and CRC-Error commands (Tables 12/13); on-demand Data
    frames interleave the I/O-frame stream.
- **CSR block** (§3.2, Table 36): link status/state/speed, local/remote capabilities,
  platform IDs, per-stage RX/TX frame counters and error counters (RWC), and the
  Link Control register (soft reset, retrain, auto/trigger Configure, channel resets).

---

## Top-level datapath (spec Fig 19)

`ltpi_scm_top` / `ltpi_hpm_top` wrap `ltpi_phy` (LVDS SERDES + CDC FIFOs) +
`ltpi_core` (the System-Clock, one-symbol-per-clock datapath):

```
                       ltpi_phy (PHY / SERDES domains)        ltpi_core (System Clock)
 LVDS pads ── SERDES deser+comma-align ─[RX CDC FIFO]─► rx_sym ─► frame_rx ─ 8b10b dec ─ incr.CRC ─┐
   (tx_clk    (rx_bit_clk)                                                                          │
    rx_clk)                                                                       link_fsm ──► CSR  │
 LVDS pads ◄─ SERDES serializer ◄────────[TX CDC FIFO]◄─ tx_sym ◄─ frame_tx ◄ 8b10b enc ◄ incr.CRC ◄┘
              (tx_bit_clk)                                            ▲   GPIO / UART / I2C / Data
   IOPLL: 25 MHz base ── speed_change ──► 400 MHz DDR (op rate)       └─ TX payload mux / RX routing
```

| File (`rtl/`) | Role |
|---|---|
| `ltpi_pkg.sv` | comma codes, frame offsets, state/speed/event/cmd enums, capability map, CSR offsets, spec thresholds |
| `ltpi_crc8.sv` | CRC-8 (0x07, init 0); the single-byte step used incrementally per symbol |
| `ltpi_8b10b.sv` | IBM 8b/10b encoder + decoder (data + K28.5/6/7 commas, running disparity) |
| `ltpi_frame_tx.sv` / `ltpi_frame_rx.sv` | **symbol-parallel** (1 symbol/clock) 16-symbol frame assemble/encode and decode/CRC-check; **incremental CRC**, **pipelined decode** |
| `ltpi_link_fsm.sv` | training/configuration/operational FSM + **base→operational speed switch** (`ROLE`-parameterized) |
| `ltpi_gpio_chan.sv` `ltpi_uart_chan.sv` `ltpi_i2c_relay.sv` `ltpi_data_chan.sv` | the four channels |
| `ltpi_csr.sv` | Table 36 register file with a BMC access port |
| `ltpi_core.sv` | System-Clock endpoint core: frame tx/rx + FSM + channels + CSR; symbol interface to the PHY |
| `ltpi_ser.sv` / `ltpi_deser.sv` | bit serializer / deserializer + comma bit-align (behavioral SERDES model inside the PHY) |
| `ltpi_cdc_fifo.sv` | dual-clock async FIFO (Gray-pointer) bridging core ↔ SERDES domains |
| `ltpi_phy.sv` | dual-rate serial PHY: J=10 SERDES + TX/RX CDC FIFOs (vendor LVDS SERDES + IOPLL on silicon) |
| `ltpi_scm_top.sv` / `ltpi_hpm_top.sv` | ROLE wrappers: `ltpi_core` + `ltpi_phy` + serial clocks |

---

## Build & run

```bash
# Simulation (Icarus): live SCM<->HPM link, all channels
sim/run.sh                       # -> "PASS 9/9"

# Formal (OSS CAD Suite: yosys + SymbiYosys + boolector)
source tools/env.sh
formal/run_all.sh                # -> "FORMAL: ALL GREEN"

# Synthesis + fit + STA on Quartus Prime Pro (Cyclone 10 GX)
syn/altera/build.sh core         # symbol-parallel core, timing @ parallel clock (recommended)
syn/altera/build.sh both         # both endpoint tops (incl. behavioral PHY)
syn/altera/build.sh scm syn      # one top, synthesis-only (fast check)
```

See [`syn/altera/README.md`](syn/altera/README.md) for the Quartus flow and
[`reports/`](reports/) for captured sim/formal/Quartus logs.

---

## Design decisions / scope notes

- **Dual-rate, J=10 SERDES, symbol-parallel core (spec Fig 19).** Training runs at the
  25 MHz base; at the Link Speed → Advertise transition the FSM pulses `speed_change`
  and the IOPLL reconfigures to the negotiated operational rate (up to **400 MHz DDR =
  800 Mbps**). The bit (de)serialization is a J=10 SERDES, so the fabric core never runs
  above `line_rate / 10` (80 MHz at 400 MHz DDR) — proven to close timing at 100 MHz.
  The RX is source-synchronous: each end's RX bit clock is the far end's forwarded
  `tx_clk`, so the receiver always tracks the transmitter's rate; a `realign` pulse
  re-acquires comma/word alignment after the PLL relock (the ≥1 ms Advertise dwell,
  `ADVERTISE_CYCLES`, covers relock — shrunk for sim/formal).
- **Why not 800 MHz DDR on Cyclone 10 GX:** the device's true-LVDS SERDES tops out at
  ~1.434 Gbps (I/O-PLL VCO limit), so 800 MHz DDR (1.6 Gbps) would need the GX
  transceiver or Arria 10. The committed target is **400 MHz DDR (800 Mbps)**, well
  within the LVDS SERDES window; the RTL is rate-parameterized, so a faster device/PHY
  only changes the IOPLL settings and `SPEED_CAP`.
- **PHY model.** `ltpi_phy` uses a synthesizable behavioral SERDES (`ltpi_ser`/`ltpi_deser`)
  for vendor-neutral sim/formal; on silicon the LVDS SERDES Intel FPGA IP (J=10,
  External-PLL) + IOPLL replace it under `ifdef LTPI_ALTERA_SERDES` — the symbol/CDC
  boundary is unchanged. The `core` synthesis target proves the timing-critical fabric;
  the bit-serial SERDES is the vendor hard block (an 800 MHz bit clock exceeds the
  ~644 MHz fabric limit, so it cannot be soft logic).
- The I2C relay and Data channel use the abstract local-bus / Avalon-MM interfaces the
  spec leaves to the implementation (the full I2C bus micro-architecture is explicitly
  out of LTPI scope, §2.2.1.3).
