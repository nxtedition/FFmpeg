#!/bin/sh
# Survival harness for the waterstamp / waterdetect filters (WATERSTAMP v1, spec §8).
#
# Generates synthetic programme, stamps it, pushes it through processing
# chains and checks every row against what it should read: lock, source id,
# offset (anchor t0 = 1 us, media timestamps as clock, so 0.000 ms unless
# the chain adds a delay) and, for speed changes, drift. Rows marked LOSS
# are known not to survive and are reported, not failed.
#
# Needs a build with: waterstamp waterdetect anoisesrc aevalsrc aformat
# lowpass acompressor equalizer loudnorm highpass aresample asetrate atempo
# adelay amix volume pan volumedetect concat asetpts, the aac codec, and the
# wav/matroska (de)muxers.
#
# Usage: tools/waterstamp-harness.sh [workdir] [level_dB] [path/to/ffmpeg]
# Exit status: the number of failed rows.
set -e
W=${1:-/tmp/waterstamp-harness}; LEVEL=${2:--23}; FF=${3:-./ffmpeg}
FF=$(cd "$(dirname "$FF")" && pwd)/$(basename "$FF")
mkdir -p "$W"; cd "$W"
PASS=0; FAIL=0; LOSS=0
q() { $FF -y -hide_banner -loglevel error "$@"; }

gen() {
  [ -f prog_pink.wav ] || q -f lavfi -i "anoisesrc=color=pink:sample_rate=48000:amplitude=0.3:duration=60:seed=1,aformat=sample_fmts=flt:channel_layouts=stereo" -c:a pcm_s24le prog_pink.wav
  [ -f prog_pink2.wav ] || q -f lavfi -i "anoisesrc=color=pink:sample_rate=48000:amplitude=0.3:duration=60:seed=11,aformat=sample_fmts=flt:channel_layouts=stereo" -c:a pcm_s24le prog_pink2.wav
  [ -f prog_sparse.wav ] || q -f lavfi -i "aevalsrc=exprs='random(0)*0.6*lt(mod(t\,5)\,2)|random(1)*0.6*lt(mod(t\,5)\,2)':s=48000:d=60,lowpass=f=4000,aformat=sample_fmts=flt" -c:a pcm_s24le prog_sparse.wav
  [ -f prog_music.wav ] || q -f lavfi -i "aevalsrc=exprs='0.15*sin(2*PI*220*t)+0.1*sin(2*PI*659*t)+0.08*sin(2*PI*1760*t)+0.05*sin(2*PI*2637*t)*sin(2*PI*0.5*t)|0.15*sin(2*PI*330*t)+0.1*sin(2*PI*880*t)+0.06*sin(2*PI*2093*t)':s=48000:d=60,aformat=sample_fmts=flt" -c:a pcm_s24le prog_music.wav
  [ -f prog_51.wav ] || q -f lavfi -i "anoisesrc=color=pink:sample_rate=48000:amplitude=0.3:duration=60:seed=5,aformat=sample_fmts=flt:channel_layouts=5.1" -c:a pcm_s24le prog_51.wav
}

locks() { # file [detector options] -> every "lock:1 ..." line
  $FF -hide_banner -nostats -i "$1" -map 0:a:0 -af "waterdetect=clock=pts${2:+:$2}" -f null - 2>&1 |
    grep 'waterdetect lock:1' | sed 's/.*waterdetect //' || true
}
field() { printf '%s\n' "$1" | tr ' ' '\n' | grep "^$2:" | head -1 | cut -d: -f2; }
near() { awk -v a="$1" -v b="$2" -v t="$3" 'BEGIN { d = a - b; exit !(d <= t && d >= -t) }'; }
row() { # verdict label detail
  case $1 in PASS) PASS=$((PASS + 1));; FAIL) FAIL=$((FAIL + 1));; LOSS) LOSS=$((LOSS + 1));; esac
  printf '%-4s %-36s %s\n' "$1" "$2" "$3"
}
# expect label file id offset_ms tol_ms [drift_ppm drift_tol] [detector options]
expect() {
  out=$(locks "$2" "$8"); last=$(printf '%s\n' "$out" | grep "id:$3 " | tail -1)
  n=$(printf '%s\n' "$out" | grep -c "id:$3 " || true)
  if [ -z "$last" ]; then row FAIL "$1" "no lock on id $3"; return; fi
  off=$(field "$last" offset); dr=$(field "$last" drift); want=$4
  # a speed change: the offset grows with source time at the drift rate
  [ -n "$6" ] && want=$(awk -v t="$(field "$last" t)" -v p="$6" -v o="$4" 'BEGIN { printf "%.3f", o + t * 1000 * (1 / (1 + p / 1e6) - 1) }')
  if ! near "$off" "$want" "$5"; then row FAIL "$1" "offset $off, want $want +-$5 ($last)"; return; fi
  if [ -n "$6" ] && ! near "$dr" "$6" "$7"; then row FAIL "$1" "drift $dr, want $6 +-$7"; return; fi
  row PASS "$1" "frames:$n  first[$(printf '%s\n' "$out" | grep "id:$3 " | head -1)]  last[$last]"
}
ids() { locks "$1" "$2" | awk '{print $2}' | sort -u | tr '\n' ' ' | sed 's/ $//'; }
expect_ids() { # label file "id:a id:b" [detector options]
  got=$(ids "$2" "$4")
  if [ "$got" = "$3" ]; then row PASS "$1" "ids locked: ${got:-none}"; else row FAIL "$1" "ids locked: ${got:-none}, want ${3:-none}"; fi
}

gen
for M in pink sparse music; do
  IN=stamped_$M.wav
  q -i prog_$M.wav -af "waterstamp=level=$LEVEL:id=3:t0=1" -c:a pcm_s24le "$IN"
  echo "== material: $M  level $LEVEL dB  (id 3) =="
  expect "clean"              "$IN" 3 0 0.3
  q -i "$IN" -af "adelay=100|100" c.wav;                                      expect "adelay 100ms"    c.wav 3 100 0.3
  q -i "$IN" -af "acompressor=ratio=4:threshold=-18dB,equalizer=f=2000:w=1:g=-6" c.wav; expect "compressor+EQ" c.wav 3 0 0.3
  q -i "$IN" -af "loudnorm=I=-23:LRA=7" c.wav;                                expect "loudnorm"        c.wav 3 0 0.3
  q -i "$IN" -af "highpass=f=300:poles=2" c.wav;                              expect "highpass 300Hz"  c.wav 3 0 0.3
  for b in 64k 96k 128k; do
    q -i "$IN" -c:a aac -b:a $b c.m4a;                                        expect "aac $b"          c.m4a 3 0 0.3
  done
  q -i "$IN" -af "aresample=16000,aresample=48000" c.wav;                     expect "48->16->48 kHz"  c.wav 3 0 0.3
  q -i "$IN" -af "asetrate=48048,aresample=48000" c.wav;                      expect "speed +1000ppm"  c.wav 3 0 0.5 1000 20
  q -i "$IN" -af "asetrate=47952,aresample=48000" c.wav;                      expect "speed -1000ppm"  c.wav 3 0 0.5 -1000 20
  q -i "$IN" -af "asetrate=48012,aresample=48000" c.wav;                      expect "speed +250ppm (off grid)" c.wav 3 0 0.5 250 20
  q -i "$IN" -af "asetrate=48002,aresample=48000" c.wav;                      expect "speed +41ppm"    c.wav 3 0 0.5 41 15
  q -i "$IN" -af "atempo=1.001" c.wav
  [ -n "$(ids c.wav)" ] && row PASS "atempo 1.001" "locks despite the splices" || row LOSS "atempo 1.001" "time-stretch splices the sequence"
  expect_ids "unstamped (no lock)" prog_$M.wav ""
done

echo "== margin and mixing =="
q -i prog_pink.wav -af "waterstamp=level=$((LEVEL - 5)):id=3:t0=1" -f wav c.wav
expect "pink, 5 dB below level"       c.wav 3 0 0.3
q -i prog_pink.wav  -af "waterstamp=level=$LEVEL:id=3:t0=1" -f wav a.wav
q -i prog_pink2.wav -af "waterstamp=level=$LEVEL:id=5:t0=1" -f wav a2.wav
q -i a.wav -i a2.wav -filter_complex "[0][1]amix=inputs=2:normalize=0,volume=0.5" -f wav c.wav
expect_ids "equal mix of two sources"   c.wav "id:3 id:5"
q -i prog_music.wav -af "waterstamp=level=$LEVEL:id=5:t0=1" -f wav b.wav
q -i a.wav -i b.wav -filter_complex "[0][1]amix=inputs=2:normalize=0,volume=0.5" -f wav c.wav
expect_ids "mix, id 3 12 dB under id 5" c.wav "id:3 id:5"
q -i a.wav -af "waterstamp=level=$LEVEL:id=7:t0=2000001" -f wav c.wav
expect_ids "re-stamp +2.000 s"          c.wav "id:3 id:7"
q -i a.wav -af "adelay=137|137,waterstamp=level=$LEVEL:id=7:t0=1" -f wav c.wav
expect_ids "delay 137 ms + re-stamp"    c.wav "id:3 id:7"
expect "  ... id 3 reads the delay"     c.wav 3 137 0.3

echo "== keyed stamps =="
q -i prog_music.wav -af "waterstamp=level=$LEVEL:id=3:t0=1:key=nxt-secret" -f wav k.wav
expect_ids "keyed, same key"            k.wav "id:3" "key=nxt-secret"
expect_ids "keyed, no key"              k.wav ""
expect_ids "keyed, other key"           k.wav "" "key=other-secret"
expect_ids "unkeyed, with key"          b.wav "" "key=nxt-secret"

echo "== channels =="
q -i prog_pink.wav -af "pan=stereo|c0=c0|c1=0*c1" lonly.wav
q -i lonly.wav -af "waterstamp=level=$LEVEL:id=3:t0=1" -c:a pcm_s24le c.wav
mv=$($FF -hide_banner -nostats -i c.wav -af "pan=mono|c0=c1,volumedetect" -f null - 2>&1 | grep -o 'max_volume: [-0-9.]*' | awk '{print $2}')
awk -v v="$mv" 'BEGIN { exit !(v <= -90) }' && row PASS "silent channel stays silent" "max $mv dB" || row FAIL "silent channel stays silent" "max $mv dB"
expect "stereo with one silent channel" c.wav 3 0 0.3
q -i prog_51.wav -af "waterstamp=level=$LEVEL:id=3:t0=1" -c:a pcm_s24le c.wav
mv=$($FF -hide_banner -nostats -i c.wav -i prog_51.wav -filter_complex "[0]pan=mono|c0=c3[x];[1]pan=mono|c0=c3[y];[x][y]amerge,pan=mono|c0=c0-c1,volumedetect" -f null - 2>&1 | grep -o 'max_volume: [-0-9.]*' | awk '{print $2}')
awk -v v="$mv" 'BEGIN { exit !(v <= -90) }' && row PASS "5.1: LFE untouched" "difference max $mv dB" || row FAIL "5.1: LFE untouched" "difference max $mv dB"
expect "5.1 programme"                  c.wav 3 0 0.3
q -i stamped_music.wav -af waterdetect=clock=pts -f wav c.wav
fmt=$($FF -hide_banner -i c.wav 2>&1 | grep -o 'Audio: .*' | cut -d, -f2,3 | tr -d ' ')
[ "$fmt" = "48000Hz,stereo" ] && row PASS "detector passes audio through" "$fmt" || row FAIL "detector passes audio through" "$fmt"

echo "== time =="
q -i prog_music.wav -af "waterstamp=level=$LEVEL:id=3:t0=18000000001" -f wav e2.wav
q -i stamped_music.wav -i e2.wav -filter_complex "[0][1]concat=n=2:v=0:a=1,asetpts='if(gte(T,60),PTS+18000/TB,PTS)'" -c:a pcm_s16le c.mkv
last=$(locks c.mkv "epoch=0" | tail -1); tt=$(field "$last" t)
near "$tt" 18048 7 && row PASS "epoch across a 5 h pts jump" "$last" || row FAIL "epoch across a 5 h pts jump" "t $tt, want ~18048"
q -i prog_music.wav -af "waterstamp=level=$LEVEL:id=3:t0=100000001" -f wav e3.wav
q -i stamped_music.wav -i e3.wav -filter_complex "[0][1]concat=n=2:v=0:a=1" -f wav c.wav
last=$(locks c.wav | tail -1); off=$(field "$last" offset)
near "$off" -40000 1 && row PASS "source time jumps +40 s" "$last" || row FAIL "source time jumps +40 s" "$last"

echo "== test mode (fixed -20 dBFS, audible) =="
q -i prog_pink.wav -af "waterstamp=mode=test:id=3:t0=1" -c:a pcm_s16le tm.wav
q -i tm.wav -c:a aac -b:a 24k c.m4a;                   expect "test mode, aac 24k"          c.m4a 3 0 0.3
q -i tm.wav -af "asetrate=49440,aresample=48000" c.wav; expect "test mode, +3 %, wide"      c.wav 3 0 5 30000 100 "wide=1"

echo "== null test, pink: (stamped - programme) level =="
$FF -hide_banner -nostats -i stamped_pink.wav -i prog_pink.wav -filter_complex "[0][1]amerge,pan=stereo|c0=c0-c2|c1=c1-c3,volumedetect" -f null - 2>&1 | grep -oE '(mean|max)_volume: .*'

echo "== $PASS passed, $FAIL failed, $LOSS known losses =="
exit $FAIL
