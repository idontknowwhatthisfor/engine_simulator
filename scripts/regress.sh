#!/usr/bin/env bash
# Regression check for refactors that must not change how any engine behaves.
#
#   scripts/regress.sh baseline   capture everything into regress/baseline/ (run on the code before the change)
#   scripts/regress.sh check      capture into regress/check/ and compare against the baseline
#
# For every engine and variant it captures:
#   - --dump-params (every parameter, the mesh geometry and content hash, the exhaust layout)
#   - the full --selftest and --phystest output
#   - the --wav render, compared by sha256 (the sound has a fixed RNG seed; `baseline` renders it twice
#     to prove that, and falls back to duration + RMS within 0.1% if the two renders differ)
#   - screenshots: the engine alone (--engine-only), the whole scene (gearbox, exhaust), the scene with
#     the HUD, a running engine with the HUD (glow, flames, exhaust heat), the HUD and help on a stand
#     (--engine-only), with HUD-changing options, and in manual and manual DCT, compared over every
#     pixel (a few pixels a few levels apart pass as GPU jitter: JITTER_PIXELS, JITTER_LEVELS)
# and the sha256 of every generated .mesh and the --engine-type list output.
# `check` prints PASS / FAIL per item and exits 1 if anything failed; everything printed is also saved
# in regress/<mode>/report.txt. Output goes to regress/ (gitignored).
set -u
cd "$(dirname "$0")/.."

MODE=${1:-}
if [[ "$MODE" != baseline && "$MODE" != check ]]; then
    echo "usage: scripts/regress.sh baseline|check" >&2
    exit 2
fi
if [[ "$MODE" == check && ! -d regress/baseline ]]; then
    echo "no regress/baseline: run scripts/regress.sh baseline on the unchanged code first" >&2
    exit 2
fi
OUT=regress/$MODE
rm -rf "$OUT"
mkdir -p "$OUT"
BIN=./engine_sim
# everything printed also goes to $OUT/report.txt, so a long check can be read in full afterwards
exec > >(tee "$OUT/report.txt") 2>&1
# a screenshot passes as GPU jitter when at most this many pixels differ, by at most this many levels
JITTER_PIXELS=50
JITTER_LEVELS=8

# name|arguments: every engine and version of one, from `--engine-type list --ids` ("h2 --variant tuned"
# -> "h2-tuned|--engine-type h2 --variant tuned", "inline4 --turbo yes" -> "inline4-turbo|...")
[[ -x $BIN ]] || make engine_sim >/dev/null || exit 2
VARIANTS=()
while read -r id extra; do
    name=$id
    case "$extra" in
        "--turbo yes") name=$id-turbo ;;
        --variant\ *) name=$id-${extra#--variant } ;;
    esac
    VARIANTS+=("$name|--engine-type $id${extra:+ $extra}")
done < <($BIN --engine-type list --ids)
# name|arguments for the screenshots. A shot that is not paused steps the physics in real time for
# its few frames (frame timing varies), so the scene shots pause at once with --stopat
SHOTS=(
    "engine|--nohud --noblur --mute --engine-only --crank 30"
    "scene|--nohud --noblur --mute --crank 30 --stopat 30"
    "hud|--noblur --mute --crank 30 --stopat 30"
    "running|--noblur --mute --simtime 3 --start --throttle 0.5 --stopat 370"
    # HUD and help on a stand (the bikes' --engine-only) and with the options that change HUD lines
    "stand|--noblur --mute --engine-only --simtime 3 --start --throttle 0.5 --stopat 370"
    "options|--noblur --mute --no-wheelie --grade 5 --sport --bov no --simtime 3 --start --throttle 0.3 --stopat 370"
    # manual (a car: NORMAL, then DCT; a bike: its manual box, --dct does nothing): HUD gear line, help, message
    "manual|--noblur --mute --manual --simtime 3 --start --throttle 0.3 --stopat 370"
    "dct|--noblur --mute --manual --dct --simtime 3 --start --throttle 0.3 --stopat 370"
)

wav_stats() {  # duration and RMS of a 16-bit mono WAV
    python3 -I -c '
import sys, wave, array, math
w = wave.open(sys.argv[1]); n = w.getnframes(); a = array.array("h", w.readframes(n))
rms = math.sqrt(sum(x * x for x in a) / max(1, len(a)))
print("%.6f %.6f" % (n / w.getframerate(), rms))' "$1"
}

capture() {
    local name=$1 args=$2
    # shellcheck disable=SC2086
    $BIN $args --dump-params >"$OUT/$name.params.txt" 2>&1
    $BIN $args --selftest >"$OUT/$name.selftest.txt" 2>&1
    $BIN $args --phystest >"$OUT/$name.phystest.txt" 2>&1
    $BIN $args --wav "$OUT/$name.wav" 2>&1 | sed "s#$OUT/##" >"$OUT/$name.wav.log"
    shasum -a 256 "$OUT/$name.wav" | cut -d' ' -f1 >"$OUT/$name.wav.sha256"
    wav_stats "$OUT/$name.wav" >"$OUT/$name.wav.stats"
    for s in "${SHOTS[@]}"; do
        local shot=${s%%|*} sargs=${s#*|}
        $BIN $args $sargs --screenshot "$OUT/$name.$shot.png" >/dev/null 2>&1
    done
}

echo "capturing $MODE ($(git rev-parse --short HEAD 2>/dev/null)$(git diff --quiet 2>/dev/null || echo ', with uncommitted changes'))"
for v in "${VARIANTS[@]}"; do
    name=${v%%|*}
    printf "  %s\n" "$name"
    capture "$name" "${v#*|}"
done
shasum -a 256 engines/*/*.mesh >"$OUT/meshes.sha256"
$BIN --engine-type list >"$OUT/list.txt" 2>&1

if [[ "$MODE" == baseline ]]; then
    # the sound must be deterministic for a hash comparison: render once more and compare
    nondet=0
    for v in "${VARIANTS[@]}"; do
        name=${v%%|*}
        # shellcheck disable=SC2086
        $BIN ${v#*|} --wav "$OUT/$name.again.wav" >/dev/null 2>&1
        if ! cmp -s "$OUT/$name.wav" "$OUT/$name.again.wav"; then
            echo "  WARNING: two renders of $name.wav differ: comparing duration + RMS (0.1%) instead of sha256"
            nondet=1
        fi
        rm -f "$OUT/$name.again.wav"
    done
    echo "$nondet" >"$OUT/wav_nondeterministic"
    [[ $nondet == 0 ]] && echo "  WAV renders are deterministic (two runs byte-identical)"
    echo "baseline written to $OUT/ (report: $OUT/report.txt)"
    exit 0
fi

# ---- check ------------------------------------------------------------------------------------
B=regress/baseline
fails=0
result() {  # status item detail
    printf "  %-5s %-28s %s\n" "$1" "$2" "$3"
    [[ $1 == FAIL ]] && fails=$((fails + 1))
}
same_text() {  # item file
    if cmp -s "$B/$2" "$OUT/$2"; then
        result PASS "$1" "identical"
    else
        result FAIL "$1" "differs: diff $B/$2 $OUT/$2"
        diff "$B/$2" "$OUT/$2" | head -20 | sed 's/^/          /'
    fi
}
nondet=$(cat "$B/wav_nondeterministic" 2>/dev/null || echo 0)
for v in "${VARIANTS[@]}"; do
    name=${v%%|*}
    echo "$name"
    # parameters: lines removed or changed fail; lines only added (a new field) are listed
    if cmp -s "$B/$name.params.txt" "$OUT/$name.params.txt"; then
        result PASS "dump-params" "identical"
    else
        removed=$(diff "$B/$name.params.txt" "$OUT/$name.params.txt" | grep -c '^<')
        added=$(diff "$B/$name.params.txt" "$OUT/$name.params.txt" | grep -c '^>')
        if [[ $removed == 0 ]]; then
            result PASS "dump-params" "unchanged, plus $added new line(s):"
            diff "$B/$name.params.txt" "$OUT/$name.params.txt" | grep '^>' | head -40 | sed 's/^/          /'
        else
            result FAIL "dump-params" "$removed line(s) changed or removed:"
            diff "$B/$name.params.txt" "$OUT/$name.params.txt" | head -30 | sed 's/^/          /'
        fi
    fi
    same_text "selftest" "$name.selftest.txt"
    same_text "phystest" "$name.phystest.txt"
    same_text "wav log" "$name.wav.log"
    if [[ $nondet == 0 ]]; then
        if cmp -s "$B/$name.wav.sha256" "$OUT/$name.wav.sha256"; then
            result PASS "wav sha256" "$(cut -c1-16 "$OUT/$name.wav.sha256")"
        else
            result FAIL "wav sha256" "$(cut -c1-16 "$B/$name.wav.sha256") -> $(cut -c1-16 "$OUT/$name.wav.sha256") (duration rms: $(cat "$B/$name.wav.stats") -> $(cat "$OUT/$name.wav.stats"))"
        fi
    else
        read -r d0 r0 <"$B/$name.wav.stats"
        read -r d1 r1 <"$OUT/$name.wav.stats"
        if python3 -I -c "import sys; d0,r0,d1,r1=map(float,sys.argv[1:]); sys.exit(0 if d0==d1 and abs(r1-r0)<=1e-3*r0 else 1)" "$d0" "$r0" "$d1" "$r1"; then
            result PASS "wav duration+rms" "$d1 s, rms $r1"
        else
            result FAIL "wav duration+rms" "$d0 s rms $r0 -> $d1 s rms $r1"
        fi
    fi
    for s in "${SHOTS[@]}"; do
        shot=${s%%|*}
        f=$name.$shot.png
        if [[ ! -f $OUT/$f ]]; then
            result FAIL "screenshot $shot" "not written"
        elif [[ ! -f $B/$f ]]; then
            result NEW "screenshot $shot" "not in the baseline (a shot added since): run baseline again to compare it"
        elif cmp -s "$B/$f" "$OUT/$f"; then
            result PASS "screenshot $shot" "identical"
        else
            d=$(python3 -I scripts/pngdiff.py "$B/$f" "$OUT/$f")
            # GPU noise: a few pixels a few levels apart. The same code built and shot at different times
            # differed by up to 8 pixels, 3 levels; a real change moves hundreds of pixels by tens of levels
            if python3 -I -c "import sys; n,m=int(sys.argv[1]),int(sys.argv[2]); sys.exit(0 if n<=$JITTER_PIXELS and m<=$JITTER_LEVELS else 1)" \
                "$(echo "$d" | cut -d' ' -f1)" "$(echo "$d" | sed 's/.*max channel diff //')" 2>/dev/null; then
                result PASS "screenshot $shot" "tiny GPU difference: $d"
            else
                result FAIL "screenshot $shot" "$d (compare: $($BIN --compare "$B/$f" "$OUT/$f" | tr -d '\n'))"
            fi
        fi
    done
done
echo "meshes"
while read -r sum file; do
    now=$(shasum -a 256 "$file" 2>/dev/null | cut -d' ' -f1)
    if [[ "$now" == "$sum" ]]; then
        result PASS "$(basename "$file")" "sha256 ${sum:0:16}"
    else
        result FAIL "$(basename "$file")" "sha256 ${sum:0:16} -> ${now:0:16}"
    fi
done <"$B/meshes.sha256"
echo "engine list"
same_text "--engine-type list" "list.txt"
echo
if [[ $fails == 0 ]]; then
    echo "regress: PASS (everything matches the baseline)"
else
    echo "regress: FAIL ($fails item(s) differ from the baseline)"
fi
echo "report: $OUT/report.txt"
[[ $fails == 0 ]]
