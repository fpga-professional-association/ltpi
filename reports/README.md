# Reports

Captured outputs of the three verification flows (dual-rate architecture).

| File | Flow | Summary |
|---|---|---|
| `sim_run.log` | Icarus Verilog (`sim/run.sh`) | dual-rate SCM↔HPM link — **PASS 11/11** (25 MHz train → 400 MHz DDR → all channels) |
| `formal_run.log` | SymbiYosys + boolector (`formal/run_all.sh`) | **FORMAL: ALL GREEN** (10 proof tasks) |
| `quartus/core_build.log` | Quartus Prime Pro 25.3 | symbol-parallel `ltpi_core` syn + fit + STA |
| `quartus/ltpi_core.fit.summary` | Quartus Fitter | 1,147 ALMs (1 %), 1,357 regs, 0 RAM/DSP |
| `quartus/ltpi_core.sta.summary` | Quartus STA | meets 100 MHz (+1.0 ns) → Fmax ≈ 111 MHz (> 80 MHz parallel clock for 400 MHz DDR) |
| `quartus/{scm,hpm}_build.log`, `*.fit.summary` | Quartus | endpoint-top synthesizability (incl. behavioral PHY) |

Regenerate with the commands in the top-level [`README.md`](../README.md).
