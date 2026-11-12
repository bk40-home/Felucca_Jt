#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""The Felucca engine tables (FS, CTL, SINE, PITCH_OCT / pitch_inc, SVF_G, TANH_Q15) for the eng_jp host tests,
written from the formulas Felucca-Engines documents (tools/gen_engine_tables.py, README). 44.1 kHz only.

    gen_fe_tables.py OUT_H
"""
import math
import sys
from pathlib import Path

FS, CTL = 44100, 32


def arr(name, ctype, vals, per=12):
    out = [f"static const {ctype} {name}[{len(vals)}] = {{"]
    out += ["    " + ", ".join(str(v) for v in vals[i:i + per]) + "," for i in range(0, len(vals), per)]
    return out + ["};"]


def main(path):
    L = ["/* eng_jp host tests: Felucca's engine tables at 44.1 kHz */", "#pragma once", "#include <stdint.h>",
         f"#define FS {FS}", f"#define CTL {CTL}", ""]
    L += arr("SINE", "int16_t", [int(round(32767 * math.sin(2 * math.pi * i / 1024))) for i in range(1025)])
    inc = [min(int(440.0 * 2 ** ((p / 16 - 69) / 12) / FS * 2 ** 32), 0x7FFFFFFF) for p in range(2048)]
    octv = inc[1856:2048]
    assert all(inc[q] == octv[(q + 64) % 192] >> (10 - (q + 64) // 192) for q in range(2048))
    L += arr("PITCH_OCT", "uint32_t", octv, 8)
    L += ["static inline uint32_t pitch_inc(uint32_t p)", "{",
          "    uint32_t i = p + 64u, o = (i * 21846u) >> 22;", "    return PITCH_OCT[i - o * 192u] >> (10u - o);", "}"]
    hz = [30.0 * ((16000 / 30) ** (v / 127)) for v in range(128)]
    L += arr("SVF_G", "uint16_t", [int(4096 * math.tan(math.pi * min(f, 0.45 * FS) / FS)) for f in hz])
    L += arr("TANH_Q15", "int16_t", [int(32767 * math.tanh(i / 256 * 2)) for i in range(257)])
    L += arr("CUTOFF_HZ", "uint32_t", [int(round(f)) for f in hz])
    Path(path).write_text("\n".join(L) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main(sys.argv[1])
