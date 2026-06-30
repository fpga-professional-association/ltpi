#!/usr/bin/env bash
# Run the full LTPI formal suite (SymbiYosys + boolector via OSS CAD Suite).
#   source ../tools/env.sh   # puts sby/yosys/boolector on PATH
set -u
cd "$(dirname "$0")"
source ../tools/env.sh >/dev/null 2>&1
SBYS="ltpi_8b10b ltpi_frame ltpi_cdc_fifo ltpi_link_fsm ltpi_gpio_chan ltpi_i2c_relay ltpi_data_chan ltpi_csr"
rc_all=0
for s in $SBYS; do
  for d in ${s} ${s}_prove ${s}_cover; do rm -rf "$d"; done
  out=$(sby -f "${s}.sby" 2>&1)
  echo "$out" | grep -E "DONE \(" | while read -r l; do printf "  %-22s %s\n" "$s" "${l#*DONE }"; done
  echo "$out" | grep -q "DONE (FAIL\|DONE (ERROR\|DONE (UNKNOWN" && rc_all=1
done
echo "==========================================================="
if [ "$rc_all" -eq 0 ]; then echo "FORMAL: ALL GREEN"; else echo "FORMAL: failures above"; fi
exit $rc_all
