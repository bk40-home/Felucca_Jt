/* SPDX-License-Identifier: GPL-3.0-only */
/* Copyright (C) 2026 Kris Bishop */
/* JP: a JP-8000 / JP-8080-style voice for Felucca, from M-vance (github.com/bk40-home/M-vance), whose DSP is the
 * JT-8000's (Kris Bishop) ported to fixed point and checked against the float JT sample by sample.
 *
 *   WAVE    OSC 1: SSAW (Super Saw: seven saws, Szabo's detune curve and the JP mix law), SAW (the JP's measured
 *           SHAPE law), SQR (JP width law, AC-coupled), TRI M (triangle mod: four band-limited wraps), FBK (the
 *           feedback oscillator, from the JE-8086's grids), SYNC (SAW, OSC 2 hard-synced to it), RING (SAW, and
 *           OSC 1 x OSC 2 in OSC 2's place in the mix)
 *   CTL 1   the wave's CTRL 1: DETUNE / SHAPE / WIDTH / DEPTH / HARMONICS. The host's shape routing (ENV / LFO /
 *           matrix -> SHP) moves it, as the JP's LFO1 -> CTRL does; on SYNC it sweeps OSC 2's pitch instead (3/16
 *           semitone a step, as ANALOG's SYNC), the JP's sync sweep
 *   CTL 2   SSAW: MIX. FBK: FEEDBACK. The others: X-MOD (OSC 2 -> OSC 1 through-zero FM, the JP's 69 Hz a step)
 *   OSC 2   a JP saw (a JP square at OSC 1's width when OSC 1 is SQR), OSC 2 semitones from OSC 1
 *   CUT RES TYPE  the JP filter (JpFilter, fitted to the JE-8086): LP / HP / BP, -24 (two TPT sections, the R/R0
 *           trim between them, the R5a level compensation) or -12; the damping reaches Q ~590. CUT follows
 *           Felucca's knob law (30 Hz .. 16 kHz: the Hz the screen shows, the units of the host's ENV / LFO ->
 *           FILTER); the filter's response is the JP's
 *   BAL     OSC 1 <-> OSC 2: 0 OSC 1 alone (OSC 2 is not computed), 64 both full, 127 OSC 2 alone (JP law)
 * The amplitude and its envelope, the LFOs, GLIDE, UNISON and velocity are the host's (voice.c, mod.c).
 *
 * Arithmetic: 32-bit everywhere except the filter, which keeps M-vance's int64 products (Kris, 11 Nov 2026: the
 * JP's top resonance needs ~30-bit coefficients at low cutoffs). No 64-bit division (jp_udiv64_32), no float.
 *
 * State: voice_t ph[0] OSC 1 (and FBK's master), ph[1] OSC 2, ph[2] the sync clock; s[0] s[1] the filter's first
 * section, s[6] s[7] its second (kept over a retrigger: keep 0xC3); s[2] the voice's random numbers; s[3] 1 while
 * the sync clock runs. The rest (the Super Saw's seven phases and high-pass, the feedback oscillator, the filter's
 * error feedback) lives in the part's engine memory (jp_part_t, engine_t.mem).
 *
 * Checked (tests/jp/run_jp.sh): against M-vance's own fixed point, component by component, the same inputs: SAW / SQR /
 * TRI M -74.5 dB, Super Saw -84.6 dB (its seven steps within 0.007 cent), FBK -86.5 dB, the filter bit for bit at the
 * same g (and g on Felucca's knob law within 0.011 cent of tan), R5a the same gains; ASan / UBSan clean.
 * Cost (estimates; the FM-1 is the measurement): the host bench (x86-64 -O2, 8 voices held) 150 .. 219 instructions a
 * sample a voice against ANALOG's 141 .. 145 (1.06 .. 1.55 x; SUPERSAW the heaviest). On the chip M-vance measured
 * this DSP at 8 voices of Super Saw into LP24 as OSC 24 % + FILTER 10 % of the buffer (its DSP at -O2; Felucca builds
 * at -Os), where Felucca measured 8 voices of ANALOG SAW LEAD at 17 % in all (eng_adpcm.c): a heavy JP voice about two
 * to three ANALOG voices, a single-oscillator LP12 one well under two. A RISC-V RV32IM build (no 64-bit multiply
 * instruction) needs no runtime helper: no 64-bit division, no memset.
 *
 * The host includes this after dsp.c (mulq16, clamp, noise32, voice_amp, pitch_inc) and engines.c's ENG_MEM /
 * ENG_MEM_FITS, as eng_grain.c / eng_adpcm.c. eng_jp_tables.h is made by tools/gen_jp_tables.py. */
#include "eng_jp_tables.h"
#if FS != JP_TABLES_FS
#error "eng_jp.c: eng_jp_tables.h was made for another rate (tools/gen_jp_tables.py --fs)"
#endif

enum { JP_WAVE, JP_CTL1, JP_CTL2, JP_O2, JP_CUT, JP_RES, JP_TYPE, JP_BAL };   /* P_E0 + these */
enum { JW_SSAW, JW_SAW, JW_SQR, JW_TRIM, JW_FBK, JW_SYNC, JW_RING, JW_N };
enum { JF_LP24, JF_LP12, JF_HP24, JF_HP12, JF_BP24, JF_BP12 };                   /* M-vance's (the JT's) order */
static const char *const N_JP_WAVE[] = {"SSAW", "SAW", "SQR", "TRI M", "FBK", "SYNC", "RING"};
static const char *const N_JP_TYPE[] = {"LP24", "LP12", "HP24", "HP12", "BP24", "BP12"};

/* CTL 1 / CTL 2 as each wave reads them (desc hook): the same range and default as edit[1] / edit[2] */
static const param_desc_t JP_C1[JW_N] = {
    {"DTUN", F_PCT, 0, 127, 0, 0, 0}, {"SHAP", F_PCT, 0, 127, 0, 0, 0}, {"PW", F_PCT, 0, 127, 0, 0, 0},
    {"DPTH", F_PCT, 0, 127, 0, 0, 0}, {"HARM", F_PCT, 0, 127, 0, 0, 0}, {"SHAP", F_PCT, 0, 127, 0, 0, 0},
    {"SHAP", F_PCT, 0, 127, 0, 0, 0},
};
static const param_desc_t JP_C2[JW_N] = {
    {"MIX", F_PCT, 0, 127, 0, 0, 0}, {"XMOD", F_PCT, 0, 127, 0, 0, 0}, {"XMOD", F_PCT, 0, 127, 0, 0, 0},
    {"XMOD", F_PCT, 0, 127, 0, 0, 0}, {"FDBK", F_PCT, 0, 127, 0, 0, 0}, {"XMOD", F_PCT, 0, 127, 0, 0, 0},
    {"XMOD", F_PCT, 0, 127, 0, 0, 0},
};
static uint32_t jp_wave(const track_t *t)
{
    uint32_t w = (uint32_t)t->p[P_E0 + JP_WAVE];
    return w < JW_N ? w : JW_SAW;
}
static const param_desc_t *jp_desc(const track_t *t, uint32_t k)
{
    if (k == 1u)
        return &JP_C1[jp_wave(t)];
    if (k == 2u)
        return &JP_C2[jp_wave(t)];
    return 0;
}

/* ------------------------------------------------------------------------------------- the part's memory --- */
typedef struct {
    uint32_t sph[7];                  /* Super Saw: the seven phases (Q32) */
    int32_t shx, shy;                 /* Super Saw high-pass: last input, output (Q24) */
    uint32_t fpsi;                    /* FBK: the inner (block) phase (Q32) */
    int32_t fwalk;                    /* FBK: the leaky random walk, Q24 cycles (M-vance's Q32 >> 8: 32 bits) */
    int32_t fhx, fhy;                 /* FBK high-pass: last input, output (Q24) */
    uint32_t fblocks;                 /* FBK law: inner cycles per master cycle (Q16) */
    int32_t fpull, fchaos, fwander, fsaw, fblk;   /* FBK law, Q16: made only when its three inputs move */
    uint32_t eb[4];                   /* filter: the integrators' remainders (error feedback): A bp, A lp, B bp, B lp */
    uint8_t fkey_h, fkey_f, fkey_c5;  /* FBK: the HARMONICS / FEEDBACK bytes and map the law was made for */
    uint8_t fkey_ok;                  /* .. 0: not yet (the memory is zeroed when the part switches to JP) */
    uint8_t fmode;                    /* filter: the mode + 1 its states belong to (0: none) */
} jp_vx_t;
typedef struct {
    jp_vx_t v[NVOICE];
} jp_part_t;
ENG_MEM_FITS("JP", sizeof(jp_part_t));
static jp_vx_t *jp_vx(const track_t *t, const voice_t *v)
{
    return &((jp_part_t *)ENG_MEM((uint32_t)(t - trk) % NPART))->v[(uint32_t)(v - t->v) % NVOICE];
}

/* ------------------------------------------------------------------------------------------ 32-bit helpers --- */
/* floor(a x k / 2^16) modulo 2^32, a unsigned, k signed (any): k = q 2^16 + r (q = k >> 16, r = k & 65535), so
 * a k / 2^16 = a q + a r / 2^16, the first wrapping as a phase step does (M-vance's int64 product cast to 32 bits:
 * the same bits) */
static inline uint32_t jp_smul_q16(uint32_t a, int32_t k)
{
    return a * (uint32_t)(k >> 16) + ((a >> 16) * ((uint32_t)k & 0xFFFFu)) +
           (((a & 0xFFFFu) * ((uint32_t)k & 0xFFFFu)) >> 16);
}
/* a (unsigned, < 2^31) x k (signed, |k| < 2^15) / 2^18 */
static inline int32_t jp_smul_q18(uint32_t a, int32_t k)
{
    return (((int32_t)(a >> 16) * k) >> 2) + (((int32_t)(a & 0xFFFFu) * k) >> 18);
}
/* a x k / 2^16, both unsigned, k <= 65535 (the product's top bits are a's) */
static inline uint32_t jp_umul_q16(uint32_t a, uint32_t k)
{
    return (a >> 16) * k + (((a & 0xFFFFu) * k) >> 16);
}
/* the knob as Q20 (0 .. 2^20 = the JT's float 0 .. 1) from Q8 (0 .. 127 << 8): x 2^20 / 32512 = x 32768 / 1016, one
 * division a block; on whole bytes exactly M-vance's byte x 2^20 / 127 */
static inline int32_t jp_k20(int32_t kq8)
{
    return (int32_t)((uint32_t)clamp(kq8, 0, 127 << 8) * 32768u / 1016u);   /* 32512 x 32768 < 2^32 */
}

/* n / d for a 64-bit n on the chip's 32 / 32 divide (M-vance mv_udiv64_32: Hacker's Delight divlu, two 16-bit
 * quotient digits): the FM-1 has no 64-bit divide and no libgcc. The quotient must fit 32 bits, d != 0 */
static uint32_t jp_udiv64_32(uint64_t nn, uint32_t d)
{
    uint32_t u1 = (uint32_t)(nn >> 32), u0 = (uint32_t)nn, s = 0, vn1, vn0, un32, un10, un1, un0, q1, q0, rhat, un21;
    while (!(d & 0x80000000u)) {                      /* normalise (no clz on the chip without libgcc) */
        d <<= 1;
        s++;
    }
    vn1 = d >> 16;
    vn0 = d & 0xFFFFu;
    un32 = s ? (u1 << s) | (u0 >> (32u - s)) : u1;
    un10 = u0 << s;
    un1 = un10 >> 16;
    un0 = un10 & 0xFFFFu;
    q1 = un32 / vn1;
    rhat = un32 - q1 * vn1;
    while (q1 >= 65536u || q1 * vn0 > (rhat << 16) + un1) {
        q1--;
        rhat += vn1;
        if (rhat >= 65536u)
            break;
    }
    un21 = (un32 << 16) + un1 - q1 * d;
    q0 = un21 / vn1;
    rhat = un21 - q0 * vn1;
    while (q0 >= 65536u || q0 * vn0 > (rhat << 16) + un0) {
        q0--;
        rhat += vn1;
        if (rhat >= 65536u)
            break;
    }
    return (q1 << 16) + q0;
}

/* ===================================================================================== the oscillators ====== */
/* Laws and constants from M-vance mv_osc.c (the JT's OscCore, float value in the comment), here in 32-bit forms.
 *   JvSAW   y = G (2ph - 1) - c(k) ph (ph - 1/2)(ph - 1); c = beta(k) x 4pi^2/3 x G, beta = 1.2118 - 4.70 k (k >= 0.0334)
 *   TRI M   y = wrap(g(k) x tri(ph)), g = 1.635 - 0.967 (k - 1/2) within 1.375 .. 2.02; four band-limited wraps
 *   SQR     width 50 % + 47.2 % x k (95 % cap), the width's DC removed (the JP's AC-coupled pulse) */
#define JP_SAW_G_Q16 39549            /* G = 0.6034671 */
#define JP_SAW_KMIN_Q20 35022         /* k >= 0.0334 */
#define JP_SAW_B0_Q16 79417           /* 1.2118 */
#define JP_SAW_BK_Q16 308019          /* 4.70 = 4 x 65536 + 45875 */
#define JP_SAW_SC_FR 61685            /* 4pi^2/3 x G = 7.94133 (M-vance 520437 Q16) = 7 + 61685 / 65536 */
#define JP_TRI_GMID_Q16 107151        /* 1.635 */
#define JP_TRI_GK_Q16 63373           /* 0.967 */
#define JP_TRI_GMIN_Q16 90112         /* 1.375 */
#define JP_TRI_GMAX_Q16 132383        /* 2.02 */
#define JP_PW_SPAN_HI 1933u           /* 0.944 x 2^31 (M-vance 2027224432) = 1933 x 2^20 + 20439 x 2^4 */
#define JP_PW_SPAN_LO 20439u
#define JP_PW_MAX 4080218931u         /* 95 % of the cycle */
#define JP_KQ8_MAX (127 << 8)

typedef struct {
    uint32_t pw;                      /* SQR: the width as a phase (Q32) */
    int32_t dc;                       /* SQR: the width's DC (Q15), subtracted */
    int32_t saw_ch;                   /* SAW: c (Q16) = saw_ch x 65536 + saw_cl */
    int32_t saw_cl;
    int32_t tri_gh, tri_gl;           /* TRI M: g (Q16) = tri_gh x 65536 + tri_gl */
    uint32_t tri_p[4];                /* TRI M: the four wrap phases (Q32) */
} jp_osc_t;

/* a wave's shape from its knob (Q20), once a block */
static void jp_shape(jp_osc_t *o, uint32_t wave, int32_t k)
{
    if (wave == JW_SQR) {
        /* 2^31 + 0.944 k 2^31: k x 2027224432 >> 20 exactly (M-vance's) as k x 1933 + k x 20439 >> 16 (< 2^32) */
        uint32_t pw = 0x80000000u + (uint32_t)k * JP_PW_SPAN_HI + jp_umul_q16((uint32_t)k, JP_PW_SPAN_LO);
        o->pw = pw > JP_PW_MAX ? JP_PW_MAX : pw;
        o->dc = (int32_t)(o->pw >> 16) - 32768;       /* 2 pw - 1, Q15 */
    } else if (wave == JW_TRIM) {
        int32_t g = JP_TRI_GMID_Q16 - (mulq16(k - 524288, JP_TRI_GK_Q16) >> 4);   /* Q16 */
        uint32_t q, r, inv;
        g = clamp(g, JP_TRI_GMIN_Q16, JP_TRI_GMAX_Q16);
        o->tri_gh = g >> 16;                          /* 1 or 2 */
        o->tri_gl = g & 0xFFFF;
        /* 1/g in Q28 by long division in 32-bit steps (M-vance): 2^31 / g16 = 1/g in Q15, then the remainder */
        q = 0x80000000u / (uint32_t)g;
        r = 0x80000000u - q * (uint32_t)g;            /* < g < 2^18 */
        inv = (q << 13) + ((r << 13) / (uint32_t)g);  /* Q28 */
        /* the wrap phases (OscCore: (1 -+ 1/g) / 4, (3 -+ 1/g) / 4): Q28 / 4 -> Q32 is << 2 */
        o->tri_p[0] = (268435456u - inv) << 2;
        o->tri_p[1] = (268435456u + inv) << 2;
        o->tri_p[2] = (805306368u - inv) << 2;
        o->tri_p[3] = (805306368u + inv) << 2;
    } else {                                          /* SAW */
        int32_t beta, c;
        if (k < JP_SAW_KMIN_Q20)
            k = JP_SAW_KMIN_Q20;
        /* beta = 1.2118 - 4.70 k, Q16: floor(308019 k / 2^20) = k / 4 + 45875 k / 2^20 in 32 bits, exactly */
        beta = JP_SAW_B0_Q16 - ((k >> 2) + ((int32_t)((uint32_t)(k & 3) * 4u + jp_umul_q16((uint32_t)k, 45875u)) >> 4));
        c = 7 * beta + mulq16(beta, JP_SAW_SC_FR);    /* beta x 7.94133, Q16 (M-vance's floor, exactly) */
        o->saw_ch = c >> 16;
        o->saw_cl = c & 0xFFFF;
    }
}

/* polyBLEP for a jump of height 2, Q15 (M-vance's, exactly): d = the phase since the jump (Q32, wrapping); within one
 * step after it -(1 - d/inc)^2, within one step before it +(1 - (2^32 - d)/inc)^2, else 0. One 32-bit division,
 * only next to a jump: incq = (inc >> 15) | 1 is the step in Q17 units, so d / incq is d / inc in Q15. (dsp.c's blep
 * is the same polynomial with inc >> 15 as the divisor: 0.5 % apart at low notes, so M-vance's is kept) */
static inline int32_t jp_blep(uint32_t d, uint32_t inc, uint32_t incq)
{
    uint32_t x;
    if (d < inc) {
        x = d / incq;
        if (x > 32767u)
            x = 32767u;
        x = 32768u - x;
        return -(int32_t)((x * x) >> 15);
    }
    d = 0u - d;                                       /* the distance to the jump ahead */
    if (d <= inc) {
        x = d / incq;
        if (x > 32767u)
            x = 32767u;
        x = 32768u - x;
        return (int32_t)((x * x) >> 15);
    }
    return 0;
}

/* The three waves' loops, with the phase step as a parameter (STEP advances ph for sample i): one body for the
 * plain, the X-MOD (a step a sample) and the sync-slave (a restart a sample) renders, so they cannot drift apart.
 * polyBLEP uses the base step in all three, as the JT. */
#define JP_WAVE_LOOPS(STEP)                                                                                      \
    if (wave == JW_SQR) {                                                                                        \
        const uint32_t pw = o->pw;                                                                               \
        const int32_t dc = o->dc;                                                                                \
        for (i = 0; i < n; i++) {                                                                                \
            int32_t y;                                                                                           \
            STEP;                                                                                                \
            y = ph < pw ? 32768 : -32768;                                                                        \
            y += jp_blep(ph, inc, incq);              /* the rise at 0 */                                        \
            y -= jp_blep(ph - pw, inc, incq);         /* the fall at pw */                                       \
            o_[i] = y - dc;                                                                                      \
        }                                                                                                        \
    } else if (wave == JW_TRIM) {                                                                                \
        const int32_t gh = o->tri_gh, gl = o->tri_gl;                                                            \
        const uint32_t pa = o->tri_p[0], pb = o->tri_p[1], pc = o->tri_p[2], pd = o->tri_p[3];                 \
        for (i = 0; i < n; i++) {                                                                                \
            int32_t tri, y;                                                                                      \
            STEP;                                                                                                \
            tri = ph < 0x80000000u ? (int32_t)(ph >> 15) - 32768 : 98304 - (int32_t)(ph >> 15);                \
            y = tri * gh + ((tri * gl) >> 16);        /* g tri, Q15 (|y| < 3 x 32768): floor(tri g / 2^16) */    \
            /* wrapped into -1..+1 by the phase against the four jump phases the band limiting uses, never by    \
             * the value (a value test can disagree by a sample and leave a full-height spike: M-vance) */        \
            if (ph < pa)                                                                                         \
                y += 65536;                                                                                      \
            else if (ph >= pb && ph < pc)                                                                        \
                y -= 65536;                                                                                      \
            else if (ph >= pd)                                                                                   \
                y += 65536;                                                                                      \
            y -= jp_blep(ph - pa, inc, incq) + jp_blep(ph - pb, inc, incq);   /* jumps of -2 */                  \
            y += jp_blep(ph - pc, inc, incq) + jp_blep(ph - pd, inc, incq);   /* jumps of +2 */                  \
            o_[i] = y;                                                                                           \
        }                                                                                                        \
    } else {                                                                                                     \
        const int32_t ch = o->saw_ch, cl = o->saw_cl;                                                            \
        for (i = 0; i < n; i++) {                                                                                \
            int32_t y, saw, b3;                                                                                  \
            STEP;                                                                                                \
            y = (int32_t)(ph >> 16) - 32768;          /* ph - 1/2, Q16 */                                        \
            saw = y - jp_blep(ph, inc, incq);         /* 2ph - 1, band-limited, Q15 */                           \
            /* ph (ph - 1/2)(ph - 1) = y' (y'^2 - 1/4), y' = ph - 1/2: y^2 - 2^30 is Q32 of y'^2 - 1/4, kept Q18;  \
             * |y x that| < 2^30 (its peak is at y' = 0.29), so b3 is Q34, then Q30 */                           \
            b3 = (y * ((y * y - 1073741824) >> 14)) >> 4;                                                        \
            /* G saw - c b3 (c Q16 x b3 Q30 >> 31 -> Q15), c split in two so each product fits 32 bits */        \
            o_[i] = ((saw * JP_SAW_G_Q16) >> 16) - (((ch * b3) >> 15) + ((cl * (b3 >> 15)) >> 16));              \
        }                                                                                                        \
    }

/* plain: n samples of wave from *pph at step inc */
static void jp_osc_run(const jp_osc_t *o, uint32_t wave, uint32_t *pph, uint32_t inc, int32_t *o_, uint32_t n)
{
    const uint32_t incq = (inc >> 15) | 1u;
    uint32_t ph = *pph, i;
    JP_WAVE_LOOPS(ph += inc)
    *pph = ph;
}
/* X-MOD carrier: sample i steps inc + d[i] (any sign: through-zero FM runs the phase backwards; Q32 wraps) */
static void jp_osc_fm(const jp_osc_t *o, uint32_t wave, uint32_t *pph, uint32_t inc, int32_t *o_, uint32_t n,
                      const int32_t *d)
{
    const uint32_t incq = (inc >> 15) | 1u;
    uint32_t ph = *pph, i;
    JP_WAVE_LOOPS(ph += inc + (uint32_t)d[i])
    *pph = ph;
}
/* sync slave: sample i restarts at rst[i] (the master wrapped inside it), JP_RUN = run on. The restart itself is not
 * band-limited, as on the JT (JT_SYNC_ANTIALIAS 0) */
#define JP_RUN 0xFFFFFFFFu
static void jp_osc_sync(const jp_osc_t *o, uint32_t wave, uint32_t *pph, uint32_t inc, int32_t *o_, uint32_t n,
                        const uint32_t *rst)
{
    const uint32_t incq = (inc >> 15) | 1u;
    uint32_t ph = *pph, i;
    JP_WAVE_LOOPS(ph = rst[i] == JP_RUN ? ph + inc : rst[i])
    *pph = ph;
}

/* ---- Super Saw (M-vance mv_ssaw: the JT's SupersawOsc, Szabo 2010 + the JP mix law measured on the JE-8086):
 * seven NAIVE saws (their aliasing is the JP's "air", Szabo 3.4) at f (1 + offset_i x depth(DETUNE)); gains: centre
 * -0.55366 m + 0.99785, each side -0.73764 m^2 + 1.2841 m + 0.044372 (m = MIX); a one-pole high-pass at the note's
 * own frequency (Szabo 3.3), then x 0.71 and a +-3 bound. Sum and high-pass in Q24. */
static const int32_t JP_SS_OFF_Q24[7] = {      /* Szabo table 1, offsets at full detune: M-vance's Q31 / 128 */
    -1845882, -1055025, -327551, 0, 334071, 1042962, 1802752};

typedef struct {
    uint32_t si[7];                   /* the seven steps (Q32) */
    int32_t gc, gs;                   /* centre / side gain (Q15) */
    uint32_t b, bs;                   /* the high-pass's 1 - a, as b / 2^(16 + bs): b < 65536 with 16 significant bits */
} jp_ss_t;

/* a block's Super Saw law: the steps from the centre step and DETUNE (Q8, with the host's shape routing), the gains
 * from MIX, the high-pass from the pitch */
static void jp_ssaw_set(jp_ss_t *ss, uint32_t inc, int32_t det_q8, uint32_t mix)
{
    uint32_t pos = (uint32_t)det_q8 * 255u / 127u;    /* the LUT position, Q8 (0 .. 255 << 8) */
    uint32_t li = pos >> 8, fr = pos & 255u, k, m16, w;
    int32_t depth, c, sd;
    /* depth (Q24) from the LUT, linear between entries, then Q16 */
    depth = li >= 255u ? (int32_t)JP_SSAW_DETUNE_Q24[255]
                       : (int32_t)JP_SSAW_DETUNE_Q24[li] +
                             ((((int32_t)JP_SSAW_DETUNE_Q24[li + 1] - (int32_t)JP_SSAW_DETUNE_Q24[li]) * (int32_t)fr) >> 8);
    for (k = 0; k < 7u; k++) {                        /* inc x (1 + offset x depth): offset x depth in Q18 */
        int32_t d18 = mulq16(JP_SS_OFF_Q24[k], (uint32_t)(depth >> 8)) >> 6;
        ss->si[k] = inc + (uint32_t)jp_smul_q18(inc, d18);   /* (within 0.01 cent of M-vance's 64-bit steps) */
    }
    m16 = mix * 65536u / 127u;                        /* MIX, Q16 */
    c = 65395 - mulq16(36285, m16);
    sd = 2908 + mulq16(84155, m16) - mulq16(mulq16(48342, m16), m16);
    ss->gc = clamp(c, 0, 65536) >> 1;                 /* Q15 */
    ss->gs = clamp(sd, 0, 65536) >> 1;
    /* high-pass b = 1 - a = w / (1 + w), w = 2 pi f / fs = 2 pi inc / 2^32 (from 1 Hz, held at 14 kHz): w in Q31 is
     * pi x inc (< 2^32 up there); b = w / (1 + w) in Q31 by the 64 / 32 division (no 64-bit multiply); then b is
     * normalised so the sample loop's 32-bit product keeps 16 significant bits at every pitch (a fixed Q16 b is
     * 0.08 % coarse at 65 Hz: M-vance keeps Q31 in 64 bits) */
    w = inc < 97392u ? 97392u : inc > 1363148800u ? 1363148800u : inc;
    w = w * 3u + jp_umul_q16(w, 9279u);               /* pi = 3 + 9279 / 65536: Q31 */
    {
        uint64_t den = (1ull << 31) + w, num = (uint64_t)w << 31;
        uint32_t b31, sh = 0;
        while (den >> 32) {                           /* normalised to 32 bits for the division */
            den >>= 1;
            num >>= 1;
        }
        b31 = jp_udiv64_32(num, (uint32_t)den);       /* (quotient < 2^31) */
        while ((b31 >> sh) > 65535u)
            sh++;
        ss->b = b31 >> sh;                            /* b x 2^(31 - sh), sh <= 15 */
        ss->bs = 15u - sh;                            /* the extra shift after the Q16 product */
    }
}

static void jp_ssaw_run(jp_vx_t *x, const jp_ss_t *ss, int32_t *o_, uint32_t n)
{
    const int32_t gc = ss->gc, gs = ss->gs;
    const uint32_t b = ss->b, bs = ss->bs;
    int32_t hx = x->shx, hy = x->shy;
    uint32_t sp[7], si[7], i, k;
    for (k = 0; k < 7u; k++) {
        sp[k] = x->sph[k];
        si[k] = ss->si[k];
    }
    for (i = 0; i < n; i++) {
        /* naive saws 2 ph - 1 (Q15), read before the step: the centre, and the six sides together */
        int32_t ce = (int32_t)(sp[3] >> 16) - 32768, u, y;
        int32_t sides = (int32_t)(sp[0] >> 16) + (int32_t)(sp[1] >> 16) + (int32_t)(sp[2] >> 16) +
                        (int32_t)(sp[4] >> 16) + (int32_t)(sp[5] >> 16) + (int32_t)(sp[6] >> 16) - 6 * 32768;
        int32_t sum;
        sp[0] += si[0]; sp[1] += si[1]; sp[2] += si[2]; sp[3] += si[3];
        sp[4] += si[4]; sp[5] += si[5]; sp[6] += si[6];
        sum = ((ce * gc) >> 6) + (((sides >> 2) * gs) >> 4);   /* Q30 -> Q24 (the sides' two low bits dropped) */
        u = hy + sum - hx;                            /* y = a (y + x - x1) = u - b u */
        hy = u - (mulq16(u, b) >> bs);
        hx = sum;
        y = mulq16(hy >> 4, 1454);                    /* x 0.71 (M-vance 23265 / 32768), Q24 -> Q15 */
        o_[i] = clamp(y, -3 * 32768, 3 * 32768);
    }
    for (k = 0; k < 7u; k++)
        x->sph[k] = sp[k];
    x->shx = hx;
    x->shy = hy;
}

/* ---- Feedback oscillator (M-vance mv_jfbk: the JT's JpFeedbackOsc): a naive master saw at the note and an inner
 * two-level "block" oscillator at N x the note, N from the JE-8086 grids by FEEDBACK x HARMONICS (the C3 maps below
 * F#4, the C5 maps at or above), pulled toward the master at each master wrap (lock), with a leaky random walk
 * (wander) and an output-driven rate (chaos) in the cells where the JE never settles; saw -> blocks over FEEDBACK
 * 10 .. 40 %; a fixed 1.92 ms one-pole high-pass. The law runs only when its inputs move. */
static const uint8_t JP_FB_ROW[8] = {0, 32, 48, 64, 80, 96, 112, 127};   /* FEEDBACK bytes of the grid rows */
static const uint8_t JP_FB_COL[9] = {0, 16, 32, 48, 64, 80, 96, 112, 127};   /* HARMONICS bytes of the columns */

static int32_t jp_fb_grid(const int32_t *t, uint32_t fb, uint32_t h)   /* bilinear, Q16 in and out */
{
    uint32_t r = 0, c = 0;
    int32_t fr, fc, top, bot;
    while (r < 6u && fb > JP_FB_ROW[r + 1])
        r++;
    while (c < 7u && h > JP_FB_COL[c + 1])
        c++;
    fr = clamp((((int32_t)fb - JP_FB_ROW[r]) * 65536) / (JP_FB_ROW[r + 1] - JP_FB_ROW[r]), 0, 65536);
    fc = clamp((((int32_t)h - JP_FB_COL[c]) * 65536) / (JP_FB_COL[c + 1] - JP_FB_COL[c]), 0, 65536);
    top = t[r * 9 + c] + mulq16(t[r * 9 + c + 1] - t[r * 9 + c], (uint32_t)fc);
    bot = t[(r + 1) * 9 + c] + mulq16(t[(r + 1) * 9 + c + 1] - t[(r + 1) * 9 + c], (uint32_t)fc);
    return top + mulq16(bot - top, (uint32_t)fr);
}

/* 2^x, x = 0 .. ~5.1 in Q16, result Q16: whole octaves by shift, the fraction from JP_EXP2N_Q31 (2^f = 2 x
 * 2^-(1 - f)), linear between its 1/256-octave entries */
static uint32_t jp_exp2_q16(int32_t x)
{
    uint32_t oc = (uint32_t)x >> 16, k = 65536u - ((uint32_t)x & 0xFFFFu), j = k >> 8, fr = k & 255u, v;
    v = j >= 256u ? JP_EXP2N_Q31[256] : JP_EXP2N_Q31[j] - (((JP_EXP2N_Q31[j] - JP_EXP2N_Q31[j + 1]) * fr) >> 8);
    return (v >> 14) << oc;                           /* Q31 x 2 -> Q16, then the octaves */
}

static void jp_fb_law(jp_vx_t *x, uint32_t h, uint32_t fb, uint32_t c5)
{
    if (x->fkey_ok && x->fkey_h == h && x->fkey_f == fb && x->fkey_c5 == c5)
        return;                                       /* nothing moved */
    x->fkey_ok = 1;
    x->fkey_h = (uint8_t)h;
    x->fkey_f = (uint8_t)fb;
    x->fkey_c5 = (uint8_t)c5;
    if (!fb) {                                        /* no feedback: the exact JP base, a saw through the high-pass */
        x->fblocks = 65536;
        x->fpull = 65536;
        x->fchaos = x->fwander = 0;
        x->fsaw = 46811;                              /* 1 / 1.4 */
        x->fblk = 0;
    } else {
        int32_t l2 = jp_fb_grid(c5 ? JP_FB_L2B_C5_Q16 : JP_FB_L2B_C3_Q16, fb, h), d, mx;
        x->fblocks = jp_exp2_q16(l2 < 0 ? 0 : l2);    /* never fewer than one block */
        x->fpull = clamp(jp_fb_grid(JP_FB_PULL_Q16, fb, h), 0, 65536);
        x->fchaos = jp_fb_grid(JP_FB_CHAOS_Q16, fb, h);
        x->fwander = mulq16(jp_fb_grid(JP_FB_WANDER_Q16, fb, h), JP_FB_WANDER_STEP_Q16);
        /* mix = clamp((fb / 127 - start) / span, 0, 1): saw -> blocks (unsigned: d x 65536 < 2^32) */
        d = (int32_t)(fb * 65536u / 127u) - JP_FB_MIX_START_Q16;
        mx = d <= 0 ? 0 : (int32_t)((uint32_t)d * 65536u / (uint32_t)JP_FB_MIX_SPAN_Q16);
        mx = clamp(mx, 0, 65536);
        x->fblk = mx;
        x->fsaw = mulq16(65536 - mx, 46811);          /* (1 - mix) / 1.4 */
    }
}

static void jp_fbk(jp_vx_t *x, uint32_t *pph, uint32_t inc, int32_t *o_, uint32_t n, int32_t *rng)
{
    uint32_t ph = *pph, psi = x->fpsi, inner, q, i;
    const int32_t pull = x->fpull, chaos = x->fchaos, wander = x->fwander, sg = x->fsaw, bg = x->fblk;
    int32_t hx = x->fhx, hy = x->fhy, walk = x->fwalk;
    if (inc > JP_FB_MAX_INNER)
        inc = JP_FB_MAX_INNER;                        /* the JT's 0.45 fs */
    /* inner step = blocks (Q16) x inc, Nyquist-guarded, in 32 bits: whole blocks, then the fraction */
    q = x->fblocks >> 16;
    if (q && inc > JP_FB_MAX_INNER / q) {
        inner = JP_FB_MAX_INNER;
    } else {
        uint32_t lo = jp_umul_q16(inc, x->fblocks & 0xFFFFu), s = inc * q + lo;
        inner = (s < lo || s > JP_FB_MAX_INNER) ? JP_FB_MAX_INNER : s;
    }
    for (i = 0; i < n; i++) {
        uint32_t old = ph;
        int32_t v, blk, u;
        ph += inc;
        if (ph < old) {                               /* master wrap: pull psi toward a whole inner cycle */
            psi -= (uint32_t)mulq16((int32_t)psi, (uint32_t)pull);   /* (int32_t)psi: the signed distance */
            if (wander > 0) {                         /* the leaky random walk (drift cells only) */
                int32_t d = 2 * mulq16((int32_t)noise32(rng), (uint32_t)wander) -
                            (int32_t)((uint32_t)mulq16(walk, JP_FB_WANDER_LEAK_Q16) << 8);
                walk += d >> 8;
                psi += (uint32_t)d;
            }
        }
        /* the inner rate, moved by the output in the chaos cells (chaos Q16 x out Q15 >> 15 = Q16; chaos halved
         * first: |hy >> 9| reaches ~2^17 after the high-pass's overshoot, and 2^17 x 19661 passes 2^31) */
        psi += chaos ? inner + jp_smul_q16(inner, ((chaos >> 1) * (hy >> 9)) >> 14) : inner;
        blk = psi < 0x80000000u ? -32768 : 32768;
        v = mulq16((int32_t)(ph >> 16) - 32768, (uint32_t)sg) + mulq16(blk, (uint32_t)bg);   /* Q15 */
        u = hy + v * 512 - hx;                        /* the fixed high-pass, Q24: y = u - b u (v signed: x 512) */
        hy = u - mulq16(u, JP_FB_HP_B_Q16);
        hx = v * 512;
        o_[i] = hy >> 9;
    }
    *pph = ph;
    x->fpsi = psi;
    x->fhx = hx;
    x->fhy = hy;
    x->fwalk = walk;
}

/* ======================================================================================== the filter ======= */
/* M-vance mv_filter.c, the JT's JpFilter (formats and the reasons for them there):
 *   section  Zavalishin TPT state-variable filter, R = 1/2Q: hp = d (x - s2) - a d s1, bp = g hp + s1, lp = g bp + s2
 *            (a = 2R + g, d = 1 / (1 + 2Rg + g^2), both a block); above g = 1 (~11 kHz) the direct form
 *   -24      two identical sections, the R/R0 trim between them (the first's resonant peak, up to ~590 x, back to
 *            ~1 x before the second rings), x the R5a level compensation at the output
 *   -12      one section, x (R/R0)^0.1
 *   output   linear to 4 x full scale, then a tanh shoulder into the 8 x knee (JT FilterSection H2b)
 * States Q18, g Q26, d and a d Q30. The integrators carry the bits their g products shift out into the next sample
 * (first-order error feedback): low-cutoff rounding noise stays below the signal. */
#define JP_FULL_Q30 1073741824
#define JP_LIN_Q15 (4 * 32768)        /* linear up to 4 x full scale */
#define JP_KNEE_Q15 (8 * 32768)       /* the 8 x knee */
static const uint16_t JP_SH_TANH[65] = {   /* tanh(t), t = 0..4 in 1/16 steps, Q15: the output shoulder */
    0, 2045, 4075, 6073, 8025, 9919, 11743, 13486, 15143, 16706, 18173, 19542, 20813, 21986, 23066, 24054,
    24956, 25776, 26519, 27191, 27797, 28341, 28830, 29268, 29660, 30010, 30322, 30600, 30847, 31067, 31262, 31435,
    31589, 31726, 31846, 31953, 32048, 32132, 32206, 32271, 32329, 32381, 32426, 32466, 32501, 32532, 32560, 32584,
    32606, 32625, 32642, 32657, 32670, 32681, 32691, 32700, 32708, 32715, 32721, 32727, 32732, 32736, 32740, 32743,
    32746};

/* 2^(-x), x >= 0 in Q16, result Q30: the R5a gain */
static int32_t jp_exp2n_q30(int32_t x)
{
    uint32_t i = (uint32_t)x >> 8, fr = (uint32_t)x & 255u, v;
    if (i >= 256u * 30u)
        return 0;
    v = JP_EXP2N_Q31[i & 255u];
    v -= ((v - JP_EXP2N_Q31[(i & 255u) + 1u]) * fr) >> 8;
    return (int32_t)((v >> 1) >> (i >> 8));
}

typedef struct {
    int32_t g, d, ad;                 /* g Q26, d and a d Q30 */
    int32_t gd, ggd, r2;              /* the direct form (g > 1): g d, g^2 d, 2R, Q30 */
} jp_fk_t;

/* g at a knob position (Q8, clamped by the caller): quadratic through entries j .. j+2 (quarter steps) */
static uint32_t jp_flt_g(int32_t kq8)
{
    uint32_t i = (uint32_t)kq8 >> 6, j = i < JP_G_N - 3u ? i : JP_G_N - 3u;
    int32_t u = kq8 - (int32_t)(j << 6);              /* Q6 from entry j */
    int64_t g0 = JP_G_Q26[j], g1 = JP_G_Q26[j + 1u], g2 = JP_G_Q26[j + 2u];
    return (uint32_t)(g0 + ((u * (g1 - g0)) >> 6) + (((int64_t)u * (u - 64) * (g2 - 2 * g1 + g0)) >> 13));
}

static void jp_flt_coef(jp_fk_t *f, uint32_t g, uint32_t r)
{
    uint32_t sh = 0;
    /* d = 1 / (1 + 2Rg + g^2): the denominator in Q26 needs up to 37 bits; normalised to 32 for the division */
    uint64_t den = (1ull << 26) + (((uint64_t)r * g) >> 30) + (((uint64_t)g * g) >> 26);
    while (den >> 32) {
        den >>= 1;
        sh++;
    }
    f->g = (int32_t)g;
    f->d = (int32_t)jp_udiv64_32(1ull << (56 - sh), (uint32_t)den);   /* Q30 */
    f->ad = (int32_t)((((uint64_t)(r >> 4) + g) * (uint32_t)f->d) >> 26);
    f->gd = f->ggd = f->r2 = 0;
    if (g > (1u << 26)) {                             /* the direct form's coefficients (all <= 1) */
        f->gd = (int32_t)(((uint64_t)g * (uint32_t)f->d) >> 26);
        f->ggd = (int32_t)(((uint64_t)g * (uint32_t)f->gd) >> 26);
        f->r2 = (int32_t)r;                           /* R Q31 is 2R Q30: the same bits */
    }
}

/* one section on x (Q18): the three taps, the states committed; eb / el carry the low 26 bits each g product
 * shifts out into the next sample. hp = d x - a d s1 - d s2 is taken as d (x - s2) - a d s1 (M-vance) */
#define JP_SEC(x, s1, s2, eb, el, hp, bp, lp, WHP)                                                               \
    do {                                                                                                         \
        int64_t t_;                                                                                              \
        hp = (int32_t)(((int64_t)d * ((x) - (s2)) - (int64_t)ad * (s1) + (1 << 29)) >> 30);                    \
        t_ = (int64_t)g * hp + eb;                                                                               \
        eb = (uint32_t)t_ & 67108863u;                                                                           \
        bp = (int32_t)(t_ >> 26) + (s1);                                                                         \
        t_ = (int64_t)g * bp + el;                                                                               \
        el = (uint32_t)t_ & 67108863u;                                                                           \
        lp = (int32_t)(t_ >> 26) + (s2);                                                                         \
        s1 = 2 * bp - (s1);                                                                                      \
        s2 = 2 * lp - (s2);                                                                                      \
    } while (0)
/* the direct form (g > 1): bp and lp from x and the states with coefficients <= 1; hp (HP only) from x = hp + 2R bp
 * + lp. 30-bit remainders */
#define JP_SEC_HI(x, s1, s2, eb, el, hp, bp, lp, WHP)                                                            \
    do {                                                                                                         \
        int64_t t_;                                                                                              \
        int32_t u_ = (x) - (s2);                                                                                 \
        t_ = (int64_t)gd * u_ + (int64_t)d * (s1) + eb;                                                          \
        eb = (uint32_t)t_ & 1073741823u;                                                                         \
        bp = (int32_t)(t_ >> 30);                                                                                \
        t_ = (int64_t)ggd * u_ + (int64_t)gd * (s1) + el;                                                        \
        el = (uint32_t)t_ & 1073741823u;                                                                         \
        lp = (int32_t)(t_ >> 30) + (s2);                                                                         \
        if (WHP)                                                                                                 \
            hp = (x) - lp - (int32_t)(((int64_t)r2 * bp + (1 << 29)) >> 30);                                   \
        s1 = 2 * bp - (s1);                                                                                      \
        s2 = 2 * lp - (s2);                                                                                      \
    } while (0)
/* a section over a block, in place: one loop per form and tap, each holding only its own states (M-vance: both
 * -24 sections in one loop spilled 64-bit values to the stack every sample on the chip) */
#define JP_PASS(NAME, SEC, TAP, WHP)                                                                             \
    static void NAME(const jp_fk_t *f, int32_t *buf, uint32_t n, int32_t *ps1, int32_t *ps2, uint32_t *peb,     \
                     uint32_t *pel)                                                                              \
    {                                                                                                            \
        const int32_t g = f->g, d = f->d, ad = f->ad, gd = f->gd, ggd = f->ggd, r2 = f->r2;                      \
        int32_t s1 = *ps1, s2 = *ps2;                                                                            \
        uint32_t eb = *peb, el = *pel, i;                                                                        \
        (void)g;                                                                                                 \
        (void)ad;                                                                                                \
        (void)gd;                                                                                                \
        (void)ggd;                                                                                               \
        (void)r2;                                                                                                \
        for (i = 0; i < n; i++) {                                                                                \
            int32_t x = buf[i], hp = 0, bp, lp;                                                                  \
            (void)hp;                                                                                            \
            (void)bp;                                                                                            \
            (void)lp;                                                                                            \
            SEC(x, s1, s2, eb, el, hp, bp, lp, WHP);                                                             \
            buf[i] = TAP;                                                                                        \
        }                                                                                                        \
        *ps1 = s1;                                                                                               \
        *ps2 = s2;                                                                                               \
        *peb = eb;                                                                                               \
        *pel = el;                                                                                               \
    }
JP_PASS(jp_lo_lp, JP_SEC, lp, 0)
JP_PASS(jp_lo_hp, JP_SEC, hp, 0)
JP_PASS(jp_lo_bp, JP_SEC, bp, 0)
JP_PASS(jp_hi_lp, JP_SEC_HI, lp, 0)
JP_PASS(jp_hi_hp, JP_SEC_HI, hp, 1)
JP_PASS(jp_hi_bp, JP_SEC_HI, bp, 0)
typedef void (*jp_pass_fn)(const jp_fk_t *, int32_t *, uint32_t, int32_t *, int32_t *, uint32_t *, uint32_t *);

static int32_t jp_shoulder(int32_t y)             /* linear to 4 x, a tanh shoulder into the 8 x knee */
{
    int32_t a = y < 0 ? -y : y, t, s;
    uint32_t i, fr;
    if (a <= JP_LIN_Q15)
        return y;
    t = (a - JP_LIN_Q15) >> 7;                        /* (|y| - 4) / 4 in 1/16 steps, 6 bits of fraction */
    i = (uint32_t)t >> 6;
    fr = (uint32_t)t & 63u;
    s = i >= 64u ? 32767 : JP_SH_TANH[i] + (((JP_SH_TANH[i + 1] - JP_SH_TANH[i]) * (int32_t)fr) >> 6);
    s = JP_LIN_Q15 + (s << 2);                        /* 4 + 4 tanh(u), Q15 */
    if (s > JP_KNEE_Q15)
        s = JP_KNEE_Q15;
    return y < 0 ? -s : s;
}

/* g for the modulated knob position kq8 (Q8): LP up to the top of the knob (16 kHz), HP / BP up to 12.4 kHz (where
 * the JE's high-pass saturates; exactly at it) */
static uint32_t jp_filter_g(uint32_t mode, int32_t kq8)
{
    if (mode <= JF_LP12)
        return jp_flt_g(clamp(kq8, 0, JP_KQ8_MAX));
    kq8 = clamp(kq8, 0, JP_CEIL_HB_Q8);
    return kq8 >= JP_CEIL_HB_Q8 ? JP_G_CEIL_HB_Q26 : jp_flt_g(kq8);
}

/* the filter over a block of buf (Q15 in, Q15 out up to the 8 x knee) at integrator gain g (Q26); cut is the
 * knob byte (the R5a compensation follows the knob, not its modulation) */
static void jp_filter_run(jp_vx_t *x, voice_t *v, int32_t *buf, uint32_t n, uint32_t mode, uint32_t reso,
                          uint32_t cut, uint32_t g)
{
    static const jp_pass_fn PASS[2][3] = {{jp_lo_lp, jp_lo_hp, jp_lo_bp}, {jp_hi_lp, jp_hi_hp, jp_hi_bp}};
    const uint32_t tap = mode <= JF_LP12 ? 0u : mode <= JF_HP12 ? 1u : 2u;
    const uint32_t cascade = mode == JF_LP24 || mode == JF_HP24 || mode == JF_BP24;
    jp_fk_t f;
    jp_pass_fn pass;
    int32_t mid, outg, s1, s2;
    uint32_t i;
    if (x->fmode != mode + 1u) {                      /* a new mode: its states start clean (M-vance mv_flt_reset) */
        x->fmode = (uint8_t)(mode + 1u);
        v->s[0] = v->s[1] = v->s[6] = v->s[7] = 0;
        x->eb[0] = x->eb[1] = x->eb[2] = x->eb[3] = 0;
    }
    jp_flt_coef(&f, g, JP_R_Q31[reso]);
    /* the trims: -24 R/R0 between the sections and R5a at the output (from the KNOB, not its modulation: M-vance);
     * -12 (R/R0)^0.1 at the output */
    if (cascade) {
        uint32_t ex = JP_R5A_EX_Q8[cut];
        mid = (int32_t)JP_T24_Q30[reso];
        outg = (ex && JP_LC_Q16[reso] < 0) ? jp_exp2n_q30((int32_t)(((uint32_t)-JP_LC_Q16[reso] * ex) >> 8)) : JP_FULL_Q30;
    } else {
        mid = JP_FULL_Q30;
        outg = (int32_t)JP_T12_Q30[reso];
    }
    pass = PASS[g > (1u << 26) ? 1 : 0][tap];
    for (i = 0; i < n; i++)
        buf[i] *= 8;                                  /* Q15 -> Q18 */
    s1 = v->s[0];
    s2 = v->s[1];
    pass(&f, buf, n, &s1, &s2, &x->eb[0], &x->eb[1]);
    v->s[0] = s1;
    v->s[1] = s2;
    if (cascade) {                                    /* -24: the R/R0 trim, then the second section */
        for (i = 0; i < n; i++)
            buf[i] = (int32_t)(((int64_t)buf[i] * mid + (1 << 29)) >> 30);
        s1 = v->s[6];
        s2 = v->s[7];
        pass(&f, buf, n, &s1, &s2, &x->eb[2], &x->eb[3]);
        v->s[6] = s1;
        v->s[7] = s2;
    }
    if (outg == JP_FULL_Q30) {                        /* no trim (most -24 settings): (x 2^30 + 2^32) >> 33 */
        for (i = 0; i < n; i++) {
            int32_t y = (buf[i] + 4) >> 3;
            buf[i] = (y > JP_LIN_Q15 || y < -JP_LIN_Q15) ? jp_shoulder(y) : y;
        }
    } else {
        for (i = 0; i < n; i++) {                     /* the output trim (Q30), Q18 -> Q15 rounded */
            int32_t y = (int32_t)(((int64_t)buf[i] * outg + (1ll << 32)) >> 33);
            buf[i] = (y > JP_LIN_Q15 || y < -JP_LIN_Q15) ? jp_shoulder(y) : y;
        }
    }
}

/* ========================================================================================= the voice ======== */
/* X-MOD: 69 Hz of OSC 1 deviation per knob step (JpXmodLaw, triangle calibration) as a Q32 step; the scale by
 * OSC 2's wave (M-vance mv_voice.c) */
#define JP_XMOD_STEP ((uint32_t)((69ull * 4294967296ull + FS / 2u) / FS))
#define JP_XMOD_SAW_Q16 32768         /* 0.50: saw */
#define JP_XMOD_P50_Q16 19399         /* 0.296: pulse at 50 % */
#define JP_XMOD_PSL_Q16 2147          /* (0.296 - 0.277) / 0.58: per unit of |2 pw - 1| */
#define JP_XMOD_SYNC_Q16 40829        /* 0.623: under sync, the saw */
/* the voice's output: the filter's (up to the 8 x knee) / 8 into Felucca's voice level (dsp.c voice_amp: its input
 * then stays within 32768), x 16: a JP saw at the open filter sits with ANALOG's SAW (jp_host level: +0.9 dB rms;
 * the Super Saw -0.7 dB). A multiply, not a shift: the samples are signed */
#define JP_OUT_MUL 16

static void jp_note_on(track_t *t, voice_t *v)
{
    jp_vx_t *x = jp_vx(t, v);
    uint32_t k;
    if (!v->s[2])
        v->s[2] = (int32_t)((0x2468ACEu + v->age * 2654435761u) | 1u);   /* the voice's random numbers */
    if (v->env || v->env_out)
        return;     /* still sounding (a retrigger, a steal): the oscillators, the seven, the feedback osc and the sync
                     * clock run on, and the host keeps ph[] and the filter (keep). New phases here would be a
                     * waveform jump: a click (M-vance, fm1-port/mvance-crackle-diagnosis-2026-11-02.md) */
    /* from silence: random start phases (the JT's OscSection::noteOn), a clean filter, the feedback osc fresh */
    v->ph[0] = (uint32_t)noise32(&v->s[2]);
    v->ph[1] = (uint32_t)noise32(&v->s[2]);
    v->ph[2] = v->ph[0];
    v->s[3] = 0;                                      /* the sync clock starts from OSC 1 */
    for (k = 0; k < 7u; k++)
        x->sph[k] = (uint32_t)noise32(&v->s[2]);      /* Super Saw: seven random phases (Szabo 3.4) */
    x->fpsi = 0;
    x->fwalk = 0;
    x->fhx = x->fhy = 0;
    v->s[0] = v->s[1] = v->s[6] = v->s[7] = 0;
    x->eb[0] = x->eb[1] = x->eb[2] = x->eb[3] = 0;
}

static void jp_render(track_t *t, voice_t *v, int32_t *out, uint32_t n, const vmod_t *m)
{
    const int16_t *p = t->p;
    jp_vx_t *x = jp_vx(t, v);
    const uint32_t wave = jp_wave(t);
    const int32_t c1 = clamp(p[P_E0 + JP_CTL1], 0, 127), c2 = clamp(p[P_E0 + JP_CTL2], 0, 127);
    const int32_t o2 = clamp(p[P_E0 + JP_O2], -24, 24), bal = clamp(p[P_E0 + JP_BAL], 0, 127);
    const uint32_t cut = (uint32_t)clamp(p[P_E0 + JP_CUT], 0, 127), reso = (uint32_t)clamp(p[P_E0 + JP_RES], 0, 127);
    const uint32_t ftype = (uint32_t)clamp(p[P_E0 + JP_TYPE], JF_LP24, JF_BP12);
    const uint32_t own = wave == JW_SSAW || wave == JW_FBK, ring = wave == JW_RING, sync = wave == JW_SYNC;
    const int32_t shp = m->shape - (64 << 8);         /* the host's shape routing, steps << 8 */
    const int32_t xk = own ? 0 : c2;                  /* X-MOD knob */
    /* BALANCE (JT mix.balance: linear, both full at the centre), Q15 */
    const int32_t g1 = bal > 64 ? 32767 * (127 - bal) / 63 : 32767, g2 = bal < 64 ? 32767 * bal / 64 : 32767;
    const uint32_t need2 = g2 > 0 || ring || xk > 0, need1 = g1 > 0 || ring;
    const uint32_t w1 = sync || ring ? JW_SAW : wave, w2 = wave == JW_SQR ? JW_SQR : JW_SAW;
    int32_t a[CTL], c[CTL], kq8;
    uint32_t inc1 = m->inc, inc2 = 0, ph0 = v->ph[0], ph1 = v->ph[1], ph2 = v->ph[2], i;
    jp_osc_t o1, oo2;
    if (n > CTL)
        n = CTL;
    /* CTL 1 + the host's shape routing (SYNC: the routing sweeps OSC 2's pitch instead) */
    kq8 = clamp((c1 << 8) + (sync ? 0 : shp), 0, JP_KQ8_MAX);
    if (need2) {
        int32_t p2 = m->pitch16 + o2 * 16 + (sync ? (shp * 3) >> 8 : 0);   /* 1/16 semitone */
        inc2 = pitch_inc((uint32_t)clamp(p2, 0, 2047));
        if (m->fine)                                  /* UNISON detune, TUNE, bend (as ANALOG's OSC 2) */
            inc2 += (uint32_t)((int32_t)(inc2 >> 12) * m->fine);
        /* OSC 2: a square at OSC 1's width, or a saw: at OSC 1's shape on SAW / SYNC / RING, the plain JP saw else */
        jp_shape(&oo2, w2, w2 == JW_SQR || wave == JW_SAW || sync || ring ? jp_k20(kq8) : 0);
        if (sync) {                                   /* the clock: OSC 1's unmodulated phase; its wraps restart OSC 2 */
            uint32_t rst[CTL], clk = v->s[3] ? ph2 : ph0, fq = (inc1 >> 12) | 1u;
            for (i = 0; i < n; i++) {
                uint32_t c0 = clk;
                clk += inc1;
                /* past the wrap by clk: clk / inc1 of a sample (Q12), so OSC 2 restarts that far into its cycle */
                rst[i] = clk < c0 ? (clk / fq) * (inc2 >> 12) : JP_RUN;
            }
            ph2 = clk;
            jp_osc_sync(&oo2, w2, &ph1, inc2, c, n, rst);
        } else {
            jp_osc_run(&oo2, w2, &ph1, inc2, c, n);
        }
    }
    v->s[3] = (int32_t)sync;
    if (need1) {
        if (wave == JW_SSAW) {
            jp_ss_t ss;
            jp_ssaw_set(&ss, inc1, kq8, (uint32_t)c2);
            jp_ssaw_run(x, &ss, a, n);
        } else if (wave == JW_FBK) {
            jp_fb_law(x, (uint32_t)(kq8 + 128) >> 8, (uint32_t)c2, inc1 >= JP_FB_SPLIT_INC);
            jp_fbk(x, &ph0, inc1, a, n, &v->s[2]);
        } else {
            jp_shape(&o1, w1, jp_k20(kq8));
            if (xk) {                                 /* through-zero linear FM of OSC 1 by OSC 2, Hz-fixed depth */
                int32_t d[CTL], sc, full;
                if (w2 == JW_SQR) {                   /* a pulse: by its width. |2 pw - 1| = |pw - 2^31| / 2^31 */
                    uint32_t dv = oo2.pw >= 0x80000000u ? oo2.pw - 0x80000000u : 0x80000000u - oo2.pw;
                    sc = JP_XMOD_P50_Q16 - (int32_t)(((uint32_t)JP_XMOD_PSL_Q16 * (dv >> 15)) >> 16);
                } else {
                    sc = sync ? (JP_XMOD_SAW_Q16 * JP_XMOD_SYNC_Q16) >> 16 : JP_XMOD_SAW_Q16;
                }
                /* the deviation at OSC 2 = +-1 (Q32): 69 Hz x knob steps x the scale (< 2^30), then x OSC 2 */
                full = mulq16((int32_t)(JP_XMOD_STEP * (uint32_t)xk), (uint32_t)sc) >> 15;
                for (i = 0; i < n; i++)
                    d[i] = full * c[i];
                jp_osc_fm(&o1, w1, &ph0, inc1, a, n, d);
            } else {
                jp_osc_run(&o1, w1, &ph0, inc1, a, n);
            }
        }
    }
    /* the mix: OSC 1 x g1 + (RING ? OSC 1 x OSC 2 : OSC 2) x g2 (on the JP the ring product takes OSC 2's place) */
    if (!need1)
        for (i = 0; i < n; i++)
            a[i] = 0;
    if (ring)
        for (i = 0; i < n; i++)
            c[i] = ((a[i] >> 2) * c[i]) >> 13;        /* (|a| <= 3 x 2^15: no overflow) */
    if (need2 && g2) {
        for (i = 0; i < n; i++)
            a[i] = (((a[i] >> 1) * g1) >> 14) + (((c[i] >> 1) * g2) >> 14);
    } else if (g1 != 32767) {
        for (i = 0; i < n; i++)
            a[i] = ((a[i] >> 1) * g1) >> 14;
    }
    /* the filter, on the knob plus the host's cutoff routing (ENV / LFO / matrix -> FILTER) */
    jp_filter_run(x, v, a, n, ftype, reso, cut, jp_filter_g(ftype, (int32_t)(cut << 8) + m->cutoff));
    for (i = 0; i < n; i++)
        out[i] += voice_amp(a[i] >> 3, m, i) * JP_OUT_MUL;
    v->ph[0] = ph0;
    v->ph[1] = ph1;
    v->ph[2] = ph2;
}

/* ---- presets: the engine's own sounds (no Roland patch data), one per wave and the Super Saw twice. Eight: engine
 * 14's favourites are one settings byte (favorites.c), so every one can be a favourite (Kris, 12 Nov 2026).
 * e[]: WAVE CTL1 CTL2 O2 CUT RES TYPE BAL */
static const preset_t JP_PRESETS[] = {
    {"SUPERSAW", {JW_SSAW, 90, 100, 0, 112, 10, JF_LP24, 0}, {2, 80, 110, 45}, 10, 0, FX(0, 20, 30, 45), PAT(5)},
    {"HYPER PAD", {JW_SSAW, 110, 127, 12, 82, 5, JF_LP24, 30}, {75, 90, 115, 95}, 8, 0, FX(0, 60, 25, 75), PAT(5)},
    {"SAW BRASS", {JW_SAW, 20, 0, 12, 52, 15, JF_LP24, 40}, {30, 70, 95, 45}, 40, 0, FX(0, 25, 20, 40), PAT(6)},
    {"PWM STRS", {JW_SQR, 40, 0, 12, 76, 10, JF_LP12, 24}, {65, 85, 110, 85}, 6, 0, FX(0, 55, 20, 65), PAT(5)},
    {"SYNC LEAD", {JW_SYNC, 30, 0, 19, 100, 20, JF_LP24, 127}, {0, 75, 100, 30}, 20, 1, FX(10, 0, 40, 25), PAT(4)},
    {"RING BELL", {JW_RING, 10, 0, 19, 105, 0, JF_LP12, 100}, {0, 90, 0, 80}, 0, 0, FX(0, 20, 35, 55), PAT(6)},
    {"TRI KEYS", {JW_TRIM, 40, 0, 0, 95, 10, JF_LP12, 0}, {1, 85, 40, 60}, 15, 0, FX(0, 30, 25, 40), PAT(6)},
    {"FBK BASS", {JW_FBK, 40, 50, 0, 62, 40, JF_LP24, 0}, {0, 60, 60, 25}, 35, 1, FX(10, 0, 10, 10), PAT(2)},
};
_Static_assert(NELEM(JP_PRESETS) <= 8u, "eng_jp: engine 14's favourites hold 8 presets (favorites.c)");

static const engine_t ENG_JP = {
    .name = "JP",
    .page_title = {"OSC", "FILTER"},
    .edit = {
        {"WAVE", F_ENUM, 0, JW_N - 1, JW_SAW, N_JP_WAVE, 0},
        {"SHAP", F_PCT, 0, 127, 0, 0, 0},             /* CTL 1 (jp_desc names it by WAVE) */
        {"XMOD", F_PCT, 0, 127, 0, 0, 0},             /* CTL 2 */
        {"OSC2", F_SEMI, -24, 24, 0, 0, 0},
        {"CUT", F_CUTOFF, 0, 127, 110, 0, 0},
        {"RES", F_PCT, 0, 127, 0, 0, 0},
        {"TYPE", F_ENUM, 0, JF_BP12, JF_LP24, N_JP_TYPE, 0},
        {"BAL", F_PCT, 0, 127, 0, 0, 0},
    },
    .presets = JP_PRESETS,
    .npresets = NELEM(JP_PRESETS),
    .knob = {P_E0 + JP_CUT, P_E0 + JP_RES, P_E0 + JP_CTL1, P_E0 + JP_CTL2},
    .note_on = jp_note_on,
    .render = jp_render,
    .desc = jp_desc,
    .keep = 0xC3,                                     /* the filter's sections: s[0] s[1], s[6] s[7] */
    .mem = sizeof(jp_part_t),
};
