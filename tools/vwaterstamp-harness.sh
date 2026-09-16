#!/bin/sh
# Survival harness for the vwaterstamp / vwaterdetect video filters.
#
# Generates a textured synthetic clip, stamps it, pushes it through a set of
# chains and reports lock time and timing error per chain. Expected offset
# with clock=pts and t0 = 1 us is 0.000 ms, except where the chain changes
# it (a delay reads as that delay, a speed change reads as growing offset
# plus the matching drift).
#
# Usage: tools/vwaterstamp-harness.sh [workdir] [level_dB] [path/to/ffmpeg]
set -e
W=${1:-/tmp/vwaterstamp-harness}; LEVEL=${2:--26}; FF=${3:-./ffmpeg}
FF=$(cd "$(dirname "$FF")" && pwd)/$(basename "$FF")
mkdir -p "$W"; cd "$W"

# a synthetic clip with camera-like noise: flat colour with sharp edges alone is
# the worst case for this design, real pictures carry texture
[ -f prog.mp4 ] || $FF -y -hide_banner -loglevel error -f lavfi -i "testsrc2=size=1280x720:rate=25:duration=50,noise=alls=6:allf=t" -pix_fmt yuv420p -c:v libx264 -preset veryfast -crf 12 prog.mp4
# intermediates are lossless H.264 (qp 0), which is exact and small
[ -f stamped.mkv ] || $FF -y -hide_banner -loglevel error -i prog.mp4 -vf "vwaterstamp=level=$LEVEL:id=3:t0=1" -c:v libx264 -preset ultrafast -qp 0 -pix_fmt yuv420p stamped.mkv

det() { # label file
  out=$($FF -hide_banner -nostats -i "$2" -map 0:v:0 -vf "vwaterdetect=clock=pts" -f null - 2>&1 | grep 'vwaterdetect lock' || true)
  n=$(printf '%s\n' "$out" | grep -c 'lock:1' || true)
  first=$(printf '%s\n' "$out" | grep 'lock:1' | head -1 | sed 's/.*vwaterdetect //')
  last=$(printf '%s\n' "$out" | tail -1 | sed 's/.*vwaterdetect //')
  printf '%-24s locked-frames:%-2s first[%s]  last[%s]\n' "$1" "$n" "$first" "$last"
}
chain() { # label filter [codec args]
  $FF -y -hide_banner -loglevel error -i stamped.mkv -vf "$2" -fps_mode passthrough ${3:--c:v libx264 -preset ultrafast -qp 0} -pix_fmt yuv420p c.mkv && det "$1" c.mkv
}
echo "== video, level $LEVEL dB, 1280x720p25 (expected offset 0.000 ms, id 3) =="
det   "clean lossless"        stamped.mkv
chain "x264 crf 18"           "null"        "-c:v libx264 -preset veryfast -crf 18"
chain "x264 crf 28"           "null"        "-c:v libx264 -preset veryfast -crf 28"
chain "x264 1 Mbit/s"         "null"        "-c:v libx264 -preset veryfast -b:v 1M"
chain "scale 1920x1080"       "scale=1920:1080"
chain "scale 640x360 + x264"  "scale=640:360" "-c:v libx264 -preset veryfast -crf 23"
chain "letterbox 2.39:1"      "scale=1280:536,pad=1280:720:0:92"
chain "pillarbox 4:3"         "scale=960:720,pad=1280:720:160:0"
chain "brightness/contrast"   "eq=brightness=0.05:contrast=1.2:gamma=1.1"
chain "delay 3 frames (120ms)" "tpad=start=3"
chain "fps 25 -> 30"          "fps=30"
chain "fps 25 -> 23.976 speed" "setpts=PTS*1.001"
chain "crop 5% (loss expected)" "crop=1216:684,scale=1280:720"
echo "== unstamped material: must not lock =="
det   "unstamped (no lock)"    prog.mp4
