#!/bin/sh
# Survival harness for the waterstamp / waterdetect filters (WATERSTAMP v1, spec §8).
#
# Generates three synthetic programme types, stamps each at the given level,
# pushes the stamped audio through a set of processing chains and reports
# whether the detector locks, how long it took and the timing error.
# Expected offset is 0.000 ms (anchor t0 = 1 us, media timestamps as clock)
# except where the chain changes it: the 100 ms delay reads 100.0 ms, and a
# speed change reads a growing offset plus the matching drift in ppm.
#
# Usage: tools/waterstamp-harness.sh [workdir] [level_dB] [path/to/ffmpeg]
set -e
W=${1:-/tmp/waterstamp-harness}; LEVEL=${2:--23}; FF=${3:-./ffmpeg}
FF=$(cd "$(dirname "$FF")" && pwd)/$(basename "$FF")
mkdir -p "$W"; cd "$W"

gen() {
  [ -f prog_pink.wav ] || $FF -y -hide_banner -loglevel error -f lavfi -i "anoisesrc=color=pink:sample_rate=48000:amplitude=0.3:duration=60:seed=1,aformat=sample_fmts=flt:channel_layouts=stereo" -c:a pcm_s24le prog_pink.wav
  [ -f prog_sparse.wav ] || $FF -y -hide_banner -loglevel error -f lavfi -i "aevalsrc=exprs='random(0)*0.6*lt(mod(t\,5)\,2)|random(1)*0.6*lt(mod(t\,5)\,2)':s=48000:d=60,lowpass=f=4000,aformat=sample_fmts=flt" -c:a pcm_s24le prog_sparse.wav
  [ -f prog_music.wav ] || $FF -y -hide_banner -loglevel error -f lavfi -i "aevalsrc=exprs='0.15*sin(2*PI*220*t)+0.1*sin(2*PI*659*t)+0.08*sin(2*PI*1760*t)+0.05*sin(2*PI*2637*t)*sin(2*PI*0.5*t)|0.15*sin(2*PI*330*t)+0.1*sin(2*PI*880*t)+0.06*sin(2*PI*2093*t)':s=48000:d=60,aformat=sample_fmts=flt" -c:a pcm_s24le prog_music.wav
}

det() { # label file
  out=$($FF -hide_banner -nostats -i "$2" -map 0:a:0 -af "waterdetect=clock=pts" -f null - 2>&1 | grep 'waterdetect lock' || true)
  n=$(printf '%s\n' "$out" | grep -c 'lock:1' || true)
  first=$(printf '%s\n' "$out" | grep 'lock:1' | head -1 | sed 's/.*waterdetect //')
  last=$(printf '%s\n' "$out" | tail -1 | sed 's/.*waterdetect //')
  printf '%-20s locked-frames:%-2s first[%s]  last[%s]\n' "$1" "$n" "$first" "$last"
}
chain() { # label filter
  $FF -y -hide_banner -loglevel error -i "$IN" -af "$2" -f wav c.wav && det "$1" c.wav
}

gen
for M in pink sparse music; do
  IN=stamped_$M.wav
  $FF -y -hide_banner -loglevel error -i prog_$M.wav -af "waterstamp=level=$LEVEL:id=3:t0=1" -c:a pcm_s24le "$IN"
  echo "== material: $M  level $LEVEL dB  (expected offset 0.000 ms, id 3) =="
  det   "clean"             "$IN"
  chain "adelay 100ms"      "adelay=100|100"
  chain "compressor+EQ"     "acompressor=ratio=4:threshold=-18dB,equalizer=f=2000:w=1:g=-6"
  chain "loudnorm"          "loudnorm=I=-23:LRA=7"
  chain "highpass 300Hz"    "highpass=f=300:poles=2"
  for b in 64k 96k 128k; do
    $FF -y -hide_banner -loglevel error -i "$IN" -c:a aac -b:a $b c.m4a && det "aac $b" c.m4a
  done
  chain "48->16->48 kHz"    "aresample=16000,aresample=48000"
  chain "speed +1000ppm"    "asetrate=48048,aresample=48000"
  chain "speed -1000ppm"    "asetrate=47952,aresample=48000"
  chain "speed +41ppm"      "asetrate=48002,aresample=48000"
  chain "atempo 1.001(loss)" "atempo=1.001"
done
echo "== several stamps in one signal (expect every id listed) =="
$FF -y -hide_banner -loglevel error -i prog_pink.wav  -af "waterstamp=level=$LEVEL:id=3:t0=1" -f wav a.wav
$FF -y -hide_banner -loglevel error -i prog_music.wav -af "waterstamp=level=$LEVEL:id=5:t0=1" -f wav b.wav
$FF -y -hide_banner -loglevel error -i a.wav -i b.wav -filter_complex "[0][1]amix=inputs=2:normalize=0,volume=0.5" -f wav c.wav
out=$($FF -hide_banner -nostats -i c.wav -map 0:a:0 -af "waterdetect=clock=pts" -f null - 2>&1 | grep 'waterdetect lock:1' | sed 's/.*waterdetect //' | awk '{print $2}' | sort -u | tr '\n' ' ')
printf '%-34s ids locked: %s\n' "mix same clock (id 3 buried by 5)" "$out"
$FF -y -hide_banner -loglevel error -i a.wav -af "waterstamp=level=$LEVEL:id=7:t0=2000001" -f wav c.wav
out=$($FF -hide_banner -nostats -i c.wav -map 0:a:0 -af "waterdetect=clock=pts" -f null - 2>&1 | grep 'waterdetect lock:1' | sed 's/.*waterdetect //' | awk '{print $2}' | sort -u | tr '\n' ' ')
printf '%-34s ids locked: %s\n' "re-stamp +2.000 s (ids 3, 7)" "$out"
$FF -y -hide_banner -loglevel error -i a.wav -af "adelay=137|137,waterstamp=level=$LEVEL:id=7:t0=1" -f wav c.wav
out=$($FF -hide_banner -nostats -i c.wav -map 0:a:0 -af "waterdetect=clock=pts" -f null - 2>&1 | grep 'waterdetect lock:1' | sed 's/.*waterdetect //' | awk '{print $2}' | sort -u | tr '\n' ' ')
printf '%-34s ids locked: %s\n' "delay 137 ms + re-stamp (ids 3, 7)" "$out"
echo "== keyed stamps (expect: same key locks; no key, other key, and unkeyed stamp with key do not) =="
$FF -y -hide_banner -loglevel error -i prog_music.wav -af "waterstamp=level=$LEVEL:id=3:t0=1:key=nxt-secret" -f wav k.wav
for D in "key=nxt-secret" "" "key=other-secret"; do
  out=$($FF -hide_banner -nostats -i k.wav -map 0:a:0 -af "waterdetect=clock=pts${D:+:$D}" -f null - 2>&1 | grep 'waterdetect lock:1' | sed 's/.*waterdetect //' | awk '{print $2}' | sort -u | tr '\n' ' ')
  printf '%-34s ids locked: %s\n' "keyed stamp, detect ${D:-unkeyed}" "${out:-none}"
done
out=$($FF -hide_banner -nostats -i b.wav -map 0:a:0 -af "waterdetect=clock=pts:key=nxt-secret" -f null - 2>&1 | grep 'waterdetect lock:1' | sed 's/.*waterdetect //' | awk '{print $2}' | sort -u | tr '\n' ' ')
printf '%-34s ids locked: %s\n' "unkeyed stamp, detect with key" "${out:-none}"
echo "== null test, pink: (stamped - programme) level =="
$FF -hide_banner -nostats -i stamped_pink.wav -i prog_pink.wav -filter_complex "[0][1]amerge,pan=stereo|c0=c0-c2|c1=c1-c3,volumedetect" -f null - 2>&1 | grep -E 'mean_volume|max_volume'
