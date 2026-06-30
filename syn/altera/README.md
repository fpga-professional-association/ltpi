# Quartus synthesis (Altera Cyclone 10 GX)

Targets **Cyclone 10 GX `10CX220YF780E5G`** with Quartus Prime Pro 25.3.

`build.sh` stages the RTL onto the Windows side (Quartus runs as a Windows
process, so the project lives under `C:\ltpi_quartus`) and runs Analysis &
Synthesis [+ Fitter + STA]:

```bash
syn/altera/build.sh core        # symbol-parallel core, full syn+fit+sta (recommended)
syn/altera/build.sh both        # both endpoint tops (incl. behavioral PHY)
syn/altera/build.sh scm syn     # one top, synthesis-only (fast check)
```

## `core` — the timing-critical proof

`ltpi_core.qsf` synthesizes **`ltpi_core`**: the System-Clock, one-symbol-per-clock
datapath (frame tx/rx + **incremental CRC** + **pipelined 8b/10b** + link FSM +
channels + CSR). This is the fabric that must keep up with the line rate, because
the high-rate bit (de)serialization is offloaded to the **J=10 LVDS SERDES** hard
block — at **400 MHz DDR (800 Mbps)** the parallel symbol clock is **80 MHz**.

The whole symbol/application interface is `VIRTUAL_PIN ON` (synthesized as a core
block), and `ltpi_core.sdc` constrains the single `clk` domain at **100 MHz** (10 ns)
to show headroom over the 80 MHz requirement.

### Result

| | `ltpi_core` |
|---|---|
| Logic (ALMs) | 1,147 / 80,330 (1 %) |
| Registers | 1,357 |
| RAM / DSP | 0 / 0 |
| Setup slack @ 100 MHz | **+1.0 ns (met)** → Fmax ≈ **111 MHz** |
| 400 MHz DDR parallel clock | 80 MHz — **met with margin** |

Replacing the old combinational 14-byte CRC with an incremental (1 byte/symbol)
CRC and pipelining the 8b/10b decode lifted the fabric Fmax from ~68 MHz to
~111 MHz, so the symbol-parallel core comfortably tracks the 800 Mbps line.

## PHY / bit SERDES

The bit-serial layer (an 800 MHz bit clock for 400 MHz DDR) exceeds the ~644 MHz
fabric limit and **must** be the **LVDS SERDES Intel FPGA IP** (J=10, External-PLL
mode) + **IOPLL** (integer mode) — instantiated in `ltpi_phy` under
`ifdef LTPI_ALTERA_SERDES`. The IOPLL is reconfigured at `speed_change` to switch
25 Mbps base → 800 Mbps operational; relock (tDLOCK ≤ 1 ms) is covered by the
spec's 1 ms Advertise dwell. For simulation/formal `ltpi_phy` uses a synthesizable
behavioral SERDES so the link is vendor-neutral and self-contained.

The endpoint tops (`ltpi_scm.qsf` / `ltpi_hpm.qsf`) include the behavioral PHY for a
full synthesizability check; for real silicon at 800 Mbps swap in the SERDES IP.
Captured summaries are in [`../../reports/quartus/`](../../reports/quartus/).
