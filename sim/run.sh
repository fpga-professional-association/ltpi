#!/usr/bin/env bash
# Compile + run the LTPI SCM<->HPM end-to-end testbench with Icarus Verilog.
# All RTL is pulled in via `include from the two tops (include-guarded), so only
# the testbench file is given to iverilog (-I rtl resolves the includes).
set -u
cd "$(dirname "$0")/.."
OUT=sim/tb_ltpi_link.vvp
LOG=sim/iverilog.log
iverilog -g2012 -I rtl -s tb_ltpi_link -o "$OUT" sim/tb_ltpi_link.sv >"$LOG" 2>&1
rc=$?
# iverilog returns non-zero on real errors; "sorry:" notes (harmless) keep it 0.
if [ "$rc" -ne 0 ]; then echo "COMPILE FAILED ($rc)"; grep -iE 'error' "$LOG" | head; exit 1; fi
vvp "$OUT"
