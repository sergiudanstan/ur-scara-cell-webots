#!/usr/bin/env bash
# Renders stills (PNG) and a cycle video (MP4) of the cell with Webots (needs a display).
#   tools/build_media.sh shots            -> docs/*.png
#   tools/build_media.sh movie <t0> <t1>  -> $MEDIA_OUT/cycle.mp4  (default ~/Downloads/ur-scara-cell)
set -eu
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WEBOTS=${WEBOTS:-$(command -v webots || echo /Applications/Webots.app/Contents/MacOS/webots)}
MODE=${1:-shots}
tmp="$ROOT/worlds/.media.wbt"
cp "$ROOT/worlds/ur_scara_cell.wbt" "$tmp"
if [ "$MODE" = shots ]; then
  OUT="$ROOT/docs"
  ARGS='"--out='"$OUT"'"
    "--shot=3:overview.png:6.2,-6.0,3.4:0.3,0.0,0.7"
    "--shot=11.3:scara_place.png:0.95,-0.95,1.25:0.18,-0.02,0.8"
    "--shot=21:ur_swap.png:3.0,-2.7,2.1:0.5,-0.2,0.8"
    "--shot=24.3:ur_cart.png:3.0,-3.1,1.9:0.8,-0.5,0.6"
    "--shot=40:top.png:0.35,0.0,4.6:0.35,0.01,0.0"
    "--keys=26:P" "--shot=28.3:operator.png:7.0,-6.4,3.6:1.0,-0.3,0.7"'
else
  OUT=${MEDIA_OUT:-$HOME/Downloads/ur-scara-cell}
  T0=${2:-0}; T1=${3:-60}
  ARGS='"--out='"$OUT"'" "--movie=cycle.mp4:'"$T0"':'"$T1"':5.6,-5.4,3.2:0.3,0.0,0.7:2.6,-2.6,1.9:0.45,-0.15,0.8"'
fi
cat >> "$tmp" <<WBT
Robot { name "media recorder" controller "media_recorder" supervisor TRUE controllerArgs [ $ARGS ] }
WBT
mkdir -p "$OUT"
perl -e 'alarm shift; exec @ARGV' "${MEDIA_TIMEOUT:-1500}" "$WEBOTS" --batch --mode=fast --stdout --stderr "$tmp" 2>&1 \
  | grep -E "^(shot|movie)|Traceback|Error" || true
rm -f "$tmp"
ls -la "$OUT" | grep -E "png|mp4" || true
