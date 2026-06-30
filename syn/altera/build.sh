#!/usr/bin/env bash
# Stage the RTL on the Windows side and run Quartus Prime Pro for an LTPI endpoint.
# Quartus runs as a Windows process, so the project lives under C:\ (not \\wsl$).
#
#   syn/altera/build.sh core            # symbol-parallel core, timing @ parallel clk (recommended)
#   syn/altera/build.sh both            # both endpoint tops, full syn+fit+sta
#   syn/altera/build.sh scm             # SCM top only
#   syn/altera/build.sh hpm syn         # HPM top, synthesis only (fast check)
#
# Arg1: core | scm | hpm | both (default core).   Arg2: syn | all (default all).
# The 'core' target proves the System-Clock symbol-parallel datapath closes at
# the parallel clock; the bit-serial PHY is the vendor LVDS SERDES (not synth-
# esized here).  The 'scm'/'hpm' tops include the behavioral PHY for completeness.
set -u
STAGE=/mnt/c/ltpi_quartus
QBIN=/mnt/c/altera_pro/25.3/quartus/bin64
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
WHICH="${1:-core}"
MODE="${2:-all}"

mkdir -p "$STAGE"
cp "$REPO"/rtl/*.sv           "$STAGE"/
cp "$REPO"/syn/altera/*.qsf   "$STAGE"/
cp "$REPO"/syn/altera/*.sdc   "$STAGE"/

run_one () {
  local proj="$1"   # ltpi_scm | ltpi_hpm
  echo "=================================================================="
  echo "  Quartus build: $proj"
  echo "=================================================================="
  cd "$STAGE"
  echo "--- Analysis & Synthesis ---"
  "$QBIN/quartus_syn.exe" "$proj" 2>&1 | tr -d '\r' | tail -20
  local rc=${PIPESTATUS[0]}
  [ "$rc" -ne 0 ] && { echo "SYNTH FAILED ($proj rc=$rc)"; return "$rc"; }
  [ "$MODE" = "syn" ] && { echo "SYNTH OK ($proj, syn-only)"; return 0; }
  echo "--- Fitter (place & route) ---"
  "$QBIN/quartus_fit.exe" "$proj" 2>&1 | tr -d '\r' | tail -12
  echo "--- STA ---"
  "$QBIN/quartus_sta.exe" "$proj" 2>&1 | tr -d '\r' | tail -40
}

rc=0
case "$WHICH" in
  core) run_one ltpi_core; rc=$? ;;
  scm)  run_one ltpi_scm; rc=$? ;;
  hpm)  run_one ltpi_hpm; rc=$? ;;
  both) run_one ltpi_scm; rc=$?; [ "$rc" -eq 0 ] && { run_one ltpi_hpm; rc=$?; } ;;
  *)    echo "usage: build.sh [core|scm|hpm|both] [syn|all]"; exit 2 ;;
esac
exit "$rc"
