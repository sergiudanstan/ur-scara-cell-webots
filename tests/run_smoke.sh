#!/usr/bin/env bash
# Headless end-to-end test: runs the cell in Webots (fast mode) for SMOKE_T simulated seconds with scripted
# safety events, then the PLC prints PASS/FAIL and quits with that status.
#   tests/run_smoke.sh              (SMOKE_T=330 by default)
set -eu
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WEBOTS=${WEBOTS:-$(command -v webots || echo /Applications/Webots.app/Contents/MacOS/webots)}
T=${SMOKE_T:-330}
tmp="$ROOT/worlds/.smoke.wbt"
sed "s/controller \"cell_plc\"/controller \"cell_plc\" controllerArgs [ \"--smoke=$T\" ]/" "${WORLD:-$ROOT/worlds/ur_scara_cell.wbt}" > "$tmp"
log="$ROOT/tests/.smoke.log"
status=0
# portable watchdog (macOS has no `timeout`)
perl -e 'alarm shift; exec @ARGV' "${SMOKE_TIMEOUT:-900}" \
  "$WEBOTS" --batch --mode=fast --minimize --no-rendering --stdout --stderr "$tmp" > "$log" 2>&1 || status=$?
rm -f "$tmp"
grep -E "\[SMOKE\]|error|Error|Traceback" "$log" | tail -40
grep -q "\[SMOKE\] PASS" "$log" && [ "$status" = 0 ] && { echo "smoke test: PASS"; exit 0; }
echo "smoke test: FAIL (webots exit $status, log $log)"; exit 1
