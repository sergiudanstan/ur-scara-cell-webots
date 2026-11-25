#!/usr/bin/env bash
# Builds site/: Webots HTML5 animation of a full cycle (8 trays, cart trip, scanner stop), for GitHub Pages.
#   tools/build_site.sh            (T0=0 T1=322 simulated seconds by default)
set -eu
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WEBOTS=${WEBOTS:-$(command -v webots || echo /Applications/Webots.app/Contents/MacOS/webots)}
T0=${T0:-0}; T1=${T1:-322}
tmp="$ROOT/worlds/.site.wbt"
# 40 ms step = 25 frames/s in the animation (keeps the JSON small); the cell logic is step-size independent
sed 's/basicTimeStep 16/basicTimeStep 40/' "$ROOT/worlds/ur_scara_cell.wbt" > "$tmp"
cat >> "$tmp" <<WBT
Robot { name "animation recorder" controller "animation_recorder" supervisor TRUE
        controllerArgs [ "$ROOT/site/cell.html" "$T0" "$T1" ] }
Robot { name "media recorder" controller "media_recorder" supervisor TRUE controllerArgs [ "--keys=120:P" "--quit=1e9" ] }
WBT
rm -f "$ROOT"/site/cell.{html,css,json,w3d}
mkdir -p "$ROOT/site"
perl -e 'alarm shift; exec @ARGV' "${SITE_TIMEOUT:-1800}" "$WEBOTS" --batch --mode=fast --minimize --stdout --stderr "$tmp" \
  > "$ROOT/site/.build.log" 2>&1 || true
rm -f "$tmp"
for f in cell.html cell.json cell.w3d; do
  [ -s "$ROOT/site/$f" ] || { echo "site/$f missing (see site/.build.log)"; exit 1; }
done
rm -f "$ROOT/site/.build.log"
python3 - "$ROOT/site/cell.html" <<'PY2'
import re, sys
p = sys.argv[1]
s = open(p, encoding="utf-8").read()
desc = ("<p><strong>UR5e + Epson SCARA T6 kitting cell</strong> (Webots R2025a), one full cycle: eight trays, "
        "the outfeed cart trip, and an operator walking into the scanner fields.<br><br>"
        "Infeed conveyor &rarr; Epson T6-602S places 6 parts per tray &rarr; UR5e swaps the full tray "
        "for an empty one from the magazine &rarr; full cart (2&times;2&times;2 trays) leaves through the "
        "scanner-guarded opening.<br>"
        "Left click: orbit &middot; right click: pan &middot; wheel: zoom &middot; bottom bar: play / scrub.<br><br>"
        "Source: <a href='https://github.com/sergiudanstan/ur-scara-cell-webots'>github.com/sergiudanstan/ur-scara-cell-webots</a></p>")
s = re.sub(r'<div class="description-container">.*?</div>', '<div class="description-container">' + desc + '</div>', s, flags=re.S)
s = re.sub(r"<h1>.*?</h1>", "<h1>3D animation: <strong>UR5e + Epson SCARA kitting cell</strong></h1>", s, flags=re.S)
s = re.sub(r"<title>.*?</title>", "<title>UR5e + Epson SCARA kitting cell (Webots)</title>", s, flags=re.S)
s = s.replace("<html xmlns=", "<html lang=\"en\" xmlns=")
open(p, "w", encoding="utf-8").write(s)
PY2
cat > "$ROOT/site/index.html" <<'HTML'
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta http-equiv="refresh" content="0; url=cell.html">
  <title>UR5e + Epson SCARA kitting cell (Webots)</title>
</head>
<body><p><a href="cell.html">Open the 3D animation of the cell</a></p></body>
</html>
HTML
touch "$ROOT/site/.nojekyll"
du -sh "$ROOT/site"
