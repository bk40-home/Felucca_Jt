#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Kris Bishop
"""eng_jp's lookup tables (Felucca's JP engine, firmware/src/eng_jp.c), as a C header.

    gen_jp_tables.py JT_LAWS_JSON OUT_H [--fs 44100] [--check-mvance MV_TABLES_H]

JT_LAWS_JSON is M-vance's docs/ref/jt_laws.json: the JT-8000 laws (the JP-8000 filter fitted to the JE-8086,
Szabo's Super Saw detune curve, the JpFeedbackOsc grids), the same source M-vance's tools/gen_tables.py reads.
Every table is quantised exactly as M-vance quantises it (q(x, bits) = round(x * 2^bits)), so the engine's
laws are M-vance's laws; --check-mvance compares the shared tables with M-vance's generated mv_tables.h, entry
for entry.

The one deliberate difference: the filter cutoff KNOB follows Felucca's law (the engines' tables,
gen_engine_tables.py cutoff_hz: 30 Hz .. 16 kHz over 0..127), so Felucca's Hz display and its cutoff modulation
(vmod_t.cutoff, 0..127 << 8 units) mean the same thing on JP as on every other engine. The filter itself (g =
tan(pi fc / fs), its damping, trims and level compensation) is the JP's.

Felucca runs at 44.1 kHz; the header carries the rate it was made for and eng_jp.c refuses another.
"""
import argparse
import json
import math
import re
import sys
from pathlib import Path

CTL = 32                         # Felucca's control block (samples)


def q(x, bits):
    """M-vance's quantiser: round to nearest"""
    return int(round(x * (1 << bits)))


def fail(msg):
    sys.exit(f"gen_jp_tables.py: {msg}")


def fe_hz(v):
    """Felucca's cutoff law (Felucca-Engines gen_engine_tables.py cutoff_hz), v continuous 0..127"""
    return 30.0 * ((16000 / 30) ** (v / 127))


def arr(ctype, name, vals, per, note):
    out = [f"/* {note} */", f"static const {ctype} {name}[{len(vals)}] = {{"]
    for i in range(0, len(vals), per):
        sfx = "u" if ctype.startswith("uint32") else ""
        out.append("    " + ", ".join(f"{v}{sfx}" for v in vals[i:i + per]) + ",")
    out.append("};")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("laws", type=Path, help="M-vance docs/ref/jt_laws.json")
    ap.add_argument("out", type=Path, help="the header to write (eng_jp_tables.h)")
    ap.add_argument("--fs", type=int, default=44100, help="sample rate (Felucca: 44100)")
    ap.add_argument("--check-mvance", type=Path, help="M-vance firmware/gen/mv_tables.h: compare the shared tables")
    a = ap.parse_args()
    FS = a.fs
    L = json.loads(a.laws.read_text(encoding="utf-8"))

    # ---- the filter: g = tan(pi fc / fs) on Felucca's knob law, every quarter step, Q26 -----------------
    # 0 .. 127 in quarter steps (509 entries). The voice interpolates quadratically through entries j..j+2
    # (j = min(i, N - 3)), as M-vance does on its own law: a linear table detunes a high-Q ring (M-vance found
    # 0.5 cent against the float filter). Checked here as the pole frequency, the same limits as M-vance's.
    n = 127 * 4 + 1
    g_tab = [q(math.tan(math.pi * fe_hz(i / 4) / FS), 26) for i in range(n)]
    if max(g_tab) >= 1 << 31:
        fail("g overflows int32")
    worst = 0.0
    for i in range(n - 1):
        j = min(i, n - 3)
        for t in (0.125, 0.375, 0.625, 0.875):
            v = (i + t) / 4
            fc = fe_hz(v)
            u = i + t - j
            g0, g1, g2 = g_tab[j], g_tab[j + 1], g_tab[j + 2]
            gi = (g0 + u * (g1 - g0) + u * (u - 1) / 2 * (g2 - 2 * g1 + g0)) / (1 << 26)
            ce = abs(1200 * math.log2(math.atan(gi) * FS / math.pi / fc))
            if ce > (0.1 if fc < 16000 else 1.0):
                fail(f"g interpolation {ce:.3f} cents at {fc:.0f} Hz")
            worst = max(worst, ce)
    # the HP / BP ceiling (12.4 kHz, where the JE's high-pass saturates), as a knob position (Q8) and its exact g
    hb_hz = L["flt_ceil_hpbp_hz"][0]
    ceil_hb_q8 = q(127 * math.log(hb_hz / 30) / math.log(16000 / 30), 8)
    g_ceil_hb = q(math.tan(math.pi * hb_hz / FS), 26)

    # ---- damping, trims, R5a level compensation: per resonance byte, as M-vance ----------------------------
    flt_r = [q(r, 31) for r in L["flt_r"]]
    flt_t24 = [q(t, 30) for t in L["flt_trim24"]]
    flt_t12 = [q(t, 30) for t in L["flt_trim12"]]
    flt_lc = [q(x, 16) for x in L["flt_lc_log2_per_code"]]
    # R5a works in the JP's knob codes above 64 (M-vance: the CUTOFF byte - 64). Here the knob is Felucca's, so
    # each Felucca knob byte is given the JP code of the same frequency: excess = max(0, code - 64), Q8
    spo = L["flt_steps_per_oct"][0]
    f64 = L["flt_cut_hz"][64]
    for c in (0, 127):
        if abs(64 + spo * math.log2(L["flt_cut_hz"][c] / f64) - c) > 1e-3:
            fail("the JP cutoff law is not one exponential")
    r5a_ex = [max(0, q(64 + spo * math.log2(fe_hz(v) / f64) - 64, 8)) for v in range(128)]

    exp2n = [min(q(2 ** (-i / 256), 31), 0xFFFFFFFF) for i in range(257)]

    # ---- Super Saw and the feedback oscillator ---------------------------------------------------------------
    ssaw = [q(x, 24) for x in L["ssaw_detune_lut"]]
    jf = {k[5:]: v for k, v in L.items() if k.startswith("jfbk_")}
    grids = [("JP_FB_L2B_C3_Q16", "kLog2BlocksC3"), ("JP_FB_L2B_C5_Q16", "kLog2BlocksC5"),
             ("JP_FB_PULL_Q16", "kLockPull"), ("JP_FB_CHAOS_Q16", "kChaos"), ("JP_FB_WANDER_Q16", "kWander")]
    split_inc = q(jf["kAnchorSplitHz"] / FS, 32)
    # the fixed high-pass y = a (y + x - x1) written as y = u - b u (u = y + x - x1), b = 1 - a, Q16 (32-bit)
    hp_b = q(1 - jf["kHpTau"] / (jf["kHpTau"] + 1 / FS), 16)

    out = ["/* SPDX-License-Identifier: GPL-3.0-only */",
           "/* GENERATED by tools/gen_jp_tables.py from M-vance's docs/ref/jt_laws.json (the JT-8000 laws). Do not edit.",
           " * eng_jp.c's tables: the JP filter on Felucca's cutoff knob law, the Super Saw detune curve and the",
           " * feedback oscillator's JE-8086 grids. Integers only (the FM-1 has no FPU). */",
           "#pragma once", "#include <stdint.h>", "",
           f"#define JP_TABLES_FS {FS}                 /* the rate these were made for */",
           "",
           "/* ---- the filter */",
           f"#define JP_G_N {n}                       /* entries of JP_G_Q26: knob 0..127 in quarter steps */",
           f"#define JP_CEIL_HB_Q8 {ceil_hb_q8}           /* HP / BP ceiling ({hb_hz} Hz) as a knob position, Q8 */",
           f"#define JP_G_CEIL_HB_Q26 {g_ceil_hb}      /* g exactly at that ceiling */",
           f"/* g interpolation: worst {worst:.4f} cent (pole frequency, quadratic, 30 Hz .. 16 kHz) */"]
    out += arr("uint32_t", "JP_G_Q26", g_tab, 8,
               "integrator gain tan(pi fc / fs), Q26, fc on Felucca's knob law, every quarter step of 0..127")
    out += arr("uint32_t", "JP_R_Q31", flt_r, 8, "section damping R = 1/2Q per resonance byte, Q31 (JpFilter)")
    out += arr("uint32_t", "JP_T24_Q30", flt_t24, 8, "-24 trim R/R0 per resonance byte (between the sections), Q30")
    out += arr("uint32_t", "JP_T12_Q30", flt_t12, 8, "-12 output trim (R/R0)^0.1 per resonance byte, Q30")
    out += arr("int32_t", "JP_LC_Q16", flt_lc, 8, "R5a level compensation, -24 modes: log2(gain) per JP code above 64, Q16")
    out += arr("uint16_t", "JP_R5A_EX_Q8", r5a_ex, 12,
               "R5a: the JP knob code above 64 of each Felucca knob byte (the same frequency), Q8, 0 at or below")
    out += arr("uint32_t", "JP_EXP2N_Q31", exp2n, 8, "2^(-i / 256), Q31 (R5a gain, feedback osc block count)")
    out += ["", "/* ---- Super Saw */"]
    out += arr("uint32_t", "JP_SSAW_DETUNE_Q24", ssaw, 8,
               "detune depth per DETUNE position (Szabo, the JT's SupersawOsc kDetuneLut), Q24, 256 entries over 0..1")
    out += ["", "/* ---- feedback oscillator (JpFeedbackOsc): 8 feedback rows (bytes 0 32 48 64 80 96 112 127) x 9",
            " * harmonics columns (0 16 32 48 64 80 96 112 127), row-major, Q16 */"]
    for name, key in grids:
        out += arr("int32_t", name, [q(x, 16) for x in jf[key]], 9, f"JpFeedbackOsc {key}")
    out += [f"#define JP_FB_SPLIT_INC {split_inc}u   /* {jf['kAnchorSplitHz']} Hz: the C5 maps at or above */",
            f"#define JP_FB_MIX_START_Q16 {q(jf['kMixFbStart'], 16)}",
            f"#define JP_FB_MIX_SPAN_Q16 {q(jf['kMixFbSpan'], 16)}",
            f"#define JP_FB_HP_B_Q16 {hp_b}          /* 1 - a of the fixed high-pass, tau {jf['kHpTau']} s at fs */",
            f"#define JP_FB_MAX_INNER {q(jf['kMaxInnerInc'], 32)}u",
            f"#define JP_FB_WANDER_STEP_Q16 {q(jf['kWanderStep'], 16)}",
            f"#define JP_FB_WANDER_LEAK_Q16 {q(jf['kWanderLeak'], 16)}"]
    a.out.write_text("\n".join(out) + "\n", encoding="utf-8")
    print(f"gen_jp_tables: {a.out} ({FS} Hz), g interpolation worst {worst:.4f} cent")

    if a.check_mvance:
        mv = a.check_mvance.read_text(encoding="utf-8")

        def mv_table(name):
            m = re.search(r"\b" + name + r"\[\d+\]\s*=\s*\{(.*?)\};", mv, re.S)
            if not m:
                fail(f"--check-mvance: {name} not found")
            return [int(x) for x in re.findall(r"-?\d+", m.group(1))]
        pairs = [("MV_FLT_R_Q31", flt_r), ("MV_FLT_T24_Q30", flt_t24), ("MV_FLT_T12_Q30", flt_t12),
                 ("MV_FLT_LC_Q16", flt_lc), ("MV_EXP2N_Q31", exp2n), ("MV_SSAW_DETUNE_Q24", ssaw)]
        pairs += [("MV_JFBK_" + nm[6:], [q(x, 16) for x in jf[key]]) for nm, key in grids]
        bad = [nm for nm, mine in pairs if mv_table(nm) != mine]
        if bad:
            fail("differs from M-vance: " + ", ".join(bad))
        print(f"gen_jp_tables: {len(pairs)} shared tables equal M-vance's, entry for entry")


if __name__ == "__main__":
    main()
