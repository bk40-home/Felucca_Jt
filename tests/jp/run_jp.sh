#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# The JP engine's own host tests (firmware/src/eng_jp.c). Run from the repo root:
#   tests/jp/run_jp.sh [MVANCE]
# MVANCE (optional): an M-vance checkout. With it, the tables are checked entry for entry against M-vance's and the
# engine is compared with M-vance's own fixed point (mv_osc.c, mv_filter.c), component by component.
# UPDATE=1 rewrites tests/jp/jp_golden.txt (only for a reviewed, intended change of the sound).
# Exit status: 0 when every check passed, 1 when one failed (a check failing does not stop the rest; a build error
# does, set -e). Each check is an explicit if: under set -e, "a && echo ok" would not stop or fail on a mismatch.
set -e
cd "$(dirname "$0")/../.."
OUT=build/jp
MV="$1"
fail=0
bad() { echo "FAIL: $1"; fail=1; }
mkdir -p "$OUT"
# the tables: committed eng_jp_tables.h must be what the generator writes from tools/jp_laws.json
python3 tools/gen_jp_tables.py tools/jp_laws.json "$OUT/eng_jp_tables.h" ${MV:+--check-mvance "$MV/firmware/gen/mv_tables.h"}
if cmp "$OUT/eng_jp_tables.h" firmware/src/eng_jp_tables.h; then echo "ok: firmware/src/eng_jp_tables.h is the generator's"
else bad "firmware/src/eng_jp_tables.h is not what tools/gen_jp_tables.py writes"; fi
python3 tests/jp/gen_fe_tables.py "$OUT/fe_tables.h"
W="-std=gnu99 -Wall -Wextra -Werror -Wdouble-promotion -Wno-unused-function -Wno-unused-parameter"
I="-I$OUT -Ifirmware/engines/src -Ifirmware/src -Itests/jp"
${CC:-cc} $W -O2 $I -o "$OUT/jp_host" tests/jp/jp_host.c -lm
"$OUT/jp_host" golden > "$OUT/jp_golden.txt"
if [ "${UPDATE:-0}" = 1 ]; then
    cp "$OUT/jp_golden.txt" tests/jp/jp_golden.txt
    echo "tests/jp/jp_golden.txt rewritten ($(wc -l < tests/jp/jp_golden.txt | tr -d ' ') renders)"
elif diff -u tests/jp/jp_golden.txt "$OUT/jp_golden.txt"; then
    echo "ok: $(wc -l < tests/jp/jp_golden.txt | tr -d ' ') renders as in tests/jp/jp_golden.txt"
else
    bad "renders differ from tests/jp/jp_golden.txt (above; UPDATE=1 after review)"
fi
# health: silent tails, no DC or clipping beyond the warnings (exit status = failures)
if "$OUT/jp_host" health > "$OUT/health.txt"; then tail -n 1 "$OUT/health.txt"
else tail -n 15 "$OUT/health.txt"; bad "health"; fi
# retrigger: no preset steps more on a retrigger than while playing (exit status 1 when one does)
if "$OUT/jp_host" retrig > "$OUT/retrig.txt"; then
    echo "ok: presets whose retrigger is no bigger a step than their playing: $(grep -c '^ok' "$OUT/retrig.txt")"
else grep -v '^ok' "$OUT/retrig.txt"; bad "retrigger"; fi
"$OUT/jp_host" level || bad "level"
mkdir -p "$OUT/wav"
if "$OUT/jp_host" wav "$OUT/wav" > /dev/null; then echo "ok: preset phrases in $OUT/wav"; else bad "wav"; fi
SAN="-O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer"
if ${CC:-cc} $W $SAN $I -o "$OUT/jp_host_san" tests/jp/jp_host.c -lm 2>/dev/null; then
    "$OUT/jp_host_san" golden > "$OUT/jp_golden_san.txt" || bad "ASan/UBSan golden"
    if cmp "$OUT/jp_golden.txt" "$OUT/jp_golden_san.txt"; then echo "ok: ASan/UBSan: the golden renders, the same"
    else bad "ASan/UBSan build renders differently"; fi
    s=0
    "$OUT/jp_host_san" stress 1 3000 || s=1
    "$OUT/jp_host_san" stress 7 3000 || s=1
    if [ "$s" = 0 ]; then echo "ok: ASan/UBSan stress"; else bad "ASan/UBSan stress"; fi
else
    echo "skip: ASan/UBSan (unavailable)"
fi
if [ -n "$MV" ]; then
    M="$MV/firmware"
    ${CC:-cc} -std=gnu99 -O2 -w -I"$M/src" -I"$M/gen" -c "$M/src/mv_osc.c" -o "$OUT/mv_osc.o"
    ${CC:-cc} -std=gnu99 -O2 -w -I"$M/src" -I"$M/gen" -c "$M/src/mv_filter.c" -o "$OUT/mv_filter.o"
    ${CC:-cc} $W -Wno-double-promotion -O2 $I -I"$M/src" -o "$OUT/cmp_jp" tests/jp/cmp_jp.c "$OUT/mv_osc.o" \
        "$OUT/mv_filter.o" -lm
    "$OUT/cmp_jp" || bad "comparison with M-vance"
else
    echo "skip: the comparison with M-vance (give an M-vance checkout)"
fi
if [ "$fail" = 0 ]; then echo "JP TESTS PASSED"; else echo "JP TESTS FAILED"; exit 1; fi
