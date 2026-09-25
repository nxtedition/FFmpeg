#!/bin/sh
# Survival harness for the vwaterstamp / vwaterdetect video filters.
#
# Generates a textured synthetic clip, stamps it, pushes it through a set of
# chains and checks every row against what it should read: lock, id and
# offset (clock=pts, t0 = 1 us, so 0.000 ms unless the chain adds a delay;
# a speed change reads a growing offset plus the matching drift). Rows
# marked LOSS are known not to survive at this level and are reported, not
# failed.
#
# Needs a build with: vwaterstamp vwaterdetect waterstamp waterdetect
# testsrc2 color noise eq scale pad crop fps tpad settb setpts format
# signalstats metadata aevalsrc aformat, libx264, and the matroska/mp4
# (de)muxers.
#
# Usage: tools/vwaterstamp-harness.sh [workdir] [level_dB] [path/to/ffmpeg]
# Exit status: the number of failed rows.
set -e
W=${1:-/tmp/vwaterstamp-harness}; LEVEL=${2:--26}; FF=${3:-./ffmpeg}
FF=$(cd "$(dirname "$FF")" && pwd)/$(basename "$FF")
mkdir -p "$W"; cd "$W"
PASS=0; FAIL=0; LOSS=0
q() { $FF -y -hide_banner -loglevel error "$@"; }
LL="-c:v libx264 -preset ultrafast -qp 0"      # lossless intermediates

# a synthetic clip with camera-like noise: flat colour with sharp edges alone is
# the worst case for this design, real pictures carry texture
[ -f prog.mp4 ] || q -f lavfi -i "testsrc2=size=1280x720:rate=25:duration=50,noise=alls=6:allf=t" -pix_fmt yuv420p -c:v libx264 -preset veryfast -crf 12 prog.mp4
[ -f stamped.mkv ] || q -i prog.mp4 -vf "vwaterstamp=level=$LEVEL:id=3:t0=1" $LL -pix_fmt yuv420p stamped.mkv

locks() { # file [detector options]
  $FF -hide_banner -nostats $3 -i "$1" -map 0:v:0 -vf "vwaterdetect=clock=pts${2:+:$2}" -f null - 2>&1 |
    grep 'vwaterdetect lock:1' | sed 's/.*vwaterdetect //' || true
}
field() { printf '%s\n' "$1" | tr ' ' '\n' | grep "^$2:" | head -1 | cut -d: -f2; }
near() { awk -v a="$1" -v b="$2" -v t="$3" 'BEGIN { d = a - b; exit !(d <= t && d >= -t) }'; }
row() {
  case $1 in PASS) PASS=$((PASS + 1));; FAIL) FAIL=$((FAIL + 1));; LOSS) LOSS=$((LOSS + 1));; esac
  printf '%-4s %-34s %s\n' "$1" "$2" "$3"
}
# expect label file offset_ms tol_ms [drift_ppm drift_tol [options [loss]]]
expect() {
  out=$(locks "$2" "$7"); last=$(printf '%s\n' "$out" | grep "id:3 " | tail -1)
  n=$(printf '%s\n' "$out" | grep -c "id:3 " || true)
  if [ -z "$last" ]; then row "${8:-FAIL}" "$1" "no lock"; return; fi
  off=$(field "$last" offset); dr=$(field "$last" drift); want=$3
  [ -n "$5" ] && want=$(awk -v t="$(field "$last" t)" -v p="$5" -v o="$3" 'BEGIN { printf "%.3f", o + t * 1000 * (1 / (1 + p / 1e6) - 1) }')
  if ! near "$off" "$want" "$4"; then row FAIL "$1" "offset $off, want $want +-$4 ($last)"; return; fi
  if [ -n "$5" ] && ! near "$dr" "$5" "$6"; then row FAIL "$1" "drift $dr, want $5 +-$6"; return; fi
  if printf '%s\n' "$out" | grep -qv "id:3 "; then row FAIL "$1" "other ids locked"; return; fi
  row PASS "$1" "frames:$n  first[$(printf '%s\n' "$out" | head -1)]  last[$last]"
}
chain() { # label filter offset tol [codec args [loss]]
  q -i stamped.mkv -vf "$2" -fps_mode passthrough ${5:-$LL} -pix_fmt yuv420p c.mp4
  expect "$1" c.mp4 "$3" "$4" "" "" "" "$6"
}

echo "== video, level $LEVEL dB, 1280x720p25 (id 3) =="
expect "clean lossless"           stamped.mkv 0 0.5
chain  "x264 crf 18"              "null"        0 0.5 "-c:v libx264 -preset veryfast -crf 18"
chain  "x264 crf 28"              "null"        0 1.5 "-c:v libx264 -preset veryfast -crf 28"
chain  "x264 1 Mbit/s"            "null"        0 1.5 "-c:v libx264 -preset veryfast -b:v 1M" LOSS
chain  "scale 1920x1080"          "scale=1920:1080" 0 0.5
chain  "scale 640x360 + x264"     "scale=640:360" 0 1.5 "-c:v libx264 -preset veryfast -crf 23" LOSS
chain  "letterbox 2.39:1 after"   "scale=1280:536,pad=1280:720:0:92" 0 0.5
chain  "pillarbox 4:3 after"      "scale=960:720,pad=1280:720:160:0" 0 0.5
chain  "brightness/contrast"      "eq=brightness=0.05:contrast=1.2:gamma=1.1" 0 0.5
chain  "delay 3 frames (120ms)"   "tpad=start=3" 120 0.5
chain  "fps 25 -> 30"             "fps=30" 0 1.5
# a speed change needs a timebase finer than the frame period all the way
# through (settb before setpts, passthrough, encoder time base), or the scaled
# timestamps round straight back to the nominal ones
q -i stamped.mkv -vf "settb=1/90000,setpts=PTS*1.001" -fps_mode:v passthrough -enc_time_base 1/90000 $LL -video_track_timescale 90000 -pix_fmt yuv420p c.mp4
expect "fps 25 -> 23.976 speed"   c.mp4 0 2 -1000 80
chain  "crop 5%"                  "crop=1216:684,scale=1280:720" 0 0.5 "" LOSS

echo "== content and formats =="
[ -f lb.mkv ] || q -f lavfi -i "testsrc2=size=1280x536:rate=25:duration=50,noise=alls=6:allf=t,pad=1280:720:0:92" $LL -pix_fmt yuv420p lb.mkv
q -i lb.mkv -vf "vwaterstamp=level=$LEVEL:id=3:t0=1" $LL -pix_fmt yuv420p c.mkv
expect "stamped with its bars"    c.mkv 0 0.5
[ -f dark.mkv ] || q -f lavfi -i "testsrc2=size=1280x720:rate=25:duration=50,noise=alls=6:allf=t,eq=eval=frame:contrast='if(between(t,20,26),0,1)':brightness='if(between(t,20,26),-0.45,0)'" $LL -pix_fmt yuv420p dark.mkv
q -i dark.mkv -vf "vwaterstamp=level=$LEVEL:id=3:t0=1" $LL -pix_fmt yuv420p c.mkv
n=$(locks c.mkv | grep -c 'id:3 ' || true)
[ "$n" -ge 6 ] && row PASS "6 s black mid-clip, lock held" "frames:$n" || row FAIL "6 s black mid-clip, lock held" "frames:$n, want >= 6"
q -i prog.mp4 -vf "format=yuv422p10le,vwaterstamp=level=$LEVEL:id=3:t0=1" $LL -pix_fmt yuv422p10le c.mkv
pf=$($FF -hide_banner -i c.mkv 2>&1 | grep -o 'yuv422p10le' | head -1)
[ "$pf" = yuv422p10le ] && row PASS "10-bit stays 10-bit" "$pf" || row FAIL "10-bit stays 10-bit" "${pf:-converted}"
expect "10-bit stamp, 10-bit detect" c.mkv 0 0.5
for m in normal test; do
  y=$($FF -hide_banner -nostats -f lavfi -i "color=black:size=640x360:rate=25:duration=1,format=yuv420p" -vf "vwaterstamp=mode=$m:t0=1,signalstats,metadata=print:key=lavfi.signalstats.YMIN" -frames:v 1 -f null - 2>&1 | grep -o 'YMIN=[0-9]*' | head -1 | cut -d= -f2)
  [ "$y" -ge 16 ] && row PASS "legal range on black, $m" "Y min $y" || row FAIL "legal range on black, $m" "Y min $y"
done

echo "== picture and sound of one source =="
# audio starts 200 ms after the picture; both stamped in one command, with
# and without an explicit t0: the two recovered times must agree
[ -f av.mkv ] || q -f lavfi -i "testsrc2=size=1280x720:rate=25:duration=60,noise=alls=6:allf=t" -itsoffset 0.2 -f lavfi -i "aevalsrc=exprs='0.15*sin(2*PI*220*t)+0.1*sin(2*PI*659*t)+0.08*sin(2*PI*1760*t)':s=48000:d=60,aformat=sample_fmts=flt" -map 0:v -map 1:a $LL -pix_fmt yuv420p -c:a pcm_s16le av.mkv
for T0 in "" ":t0=1000000"; do
  q -copyts -i av.mkv -vf "vwaterstamp=id=3$T0" -af "waterstamp=id=3$T0" $LL -pix_fmt yuv420p -c:a pcm_s24le c.mkv
  v=$(locks c.mkv "epoch=0" -copyts | tail -1)
  a=$($FF -hide_banner -nostats -copyts -i c.mkv -map 0:a:0 -af waterdetect=clock=pts:epoch=0 -f null - 2>&1 | grep 'waterdetect lock:1' | tail -1 | sed 's/.*waterdetect //')
  d=$(awk -v a="$(field "$v" offset)" -v b="$(field "$a" offset)" 'BEGIN { printf "%.3f", a - b }')
  near "$d" 0 1 && row PASS "A/V agree, ${T0:-wall clock}" "video - audio offset $d ms" || row FAIL "A/V agree, ${T0:-wall clock}" "video - audio offset ${d} ms (v[$v] a[$a])"
done

echo "== test mode (fixed 8 LSB, visible) with wide speed search =="
q -t 40 -i prog.mp4 -vf "vwaterstamp=mode=test:id=3:t0=1" $LL tm.mkv
q -i tm.mkv -vf scale=640:360 -c:v libx264 -preset veryfast -crf 33 c.mp4
expect "test mode, 360p crf 33"   c.mp4 0 1.5
q -i tm.mkv -vf "scale=640:360,settb=1/90000,setpts=PTS*1.03" -fps_mode:v passthrough -enc_time_base 1/90000 -c:v libx264 -preset veryfast -crf 23 -video_track_timescale 90000 c.mp4
expect "test mode, 360p, +3 %, wide" c.mp4 0 5 -29126 300 "wide=1"
rm -f tm.mkv

echo "== unstamped material: must not lock =="
n=$(locks prog.mp4 | grep -c . || true)
[ "$n" -eq 0 ] && row PASS "unstamped" "no lock" || row FAIL "unstamped" "$n locks"

echo "== $PASS passed, $FAIL failed, $LOSS known losses =="
exit $FAIL
