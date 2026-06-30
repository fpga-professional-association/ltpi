#!/usr/bin/env bash
# Source this to put the open-source formal/sim toolchain on PATH.
#   source tools/env.sh
export OSS_CAD_SUITE="$HOME/eda-tools/oss-cad-suite"
if [ -d "$OSS_CAD_SUITE" ]; then
  export PATH="$OSS_CAD_SUITE/bin:$PATH"
  echo "LTPI toolchain ready: $(yosys -V 2>/dev/null | head -1) | $(sby --version 2>/dev/null | head -1)"
else
  echo "WARNING: OSS CAD Suite not found at $OSS_CAD_SUITE (sby/boolector needed for formal)"
fi
