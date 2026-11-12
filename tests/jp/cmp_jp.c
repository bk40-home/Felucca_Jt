/* SPDX-License-Identifier: GPL-3.0-only */
/* eng_jp against M-vance, component by component (M-vance's own fixed point: mv_osc.c, mv_filter.c, linked as
 * they are; M-vance is itself checked against the float JT by its tools/compare). The same inputs to both, sample
 * by sample: the error's RMS in dB under the signal's, and its largest sample.
 *
 *   SAW / SQR / TRI M   shape bytes 0 .. 127, notes 24 .. 96
 *   SUPER SAW           the same seven phases, DETUNE x MIX, notes 36 .. 84
 *   FEEDBACK            the grid points with no random walk (wander 0), notes 36 .. 84
 *   FILTER              six modes x resonance x cutoff on one saw: M-vance driven at the JP code of the Hz eng_jp's
 *                       Felucca knob gives; the R5a gain checked apart at whole JP codes
 * Also the Super Saw's detune depth at every DETUNE byte against M-vance's (see the note at ssaw_depth). */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define NVOICE 8
#define NPART 4
#include "jp_prelude.h"
#include "mv_osc.h"
#include "mv_filter.h"

#define N 8192
static double worst_db = -1e9;

static void report(const char *what, const int32_t *a, const int32_t *b, uint32_t n, double limit_db, int *bad)
{
    double es = 0, ss = 0, db;
    int32_t emax = 0;
    uint32_t i;
    for (i = 0; i < n; i++) {
        int32_t e = a[i] - b[i];
        es += (double)e * e;
        ss += (double)b[i] * b[i];
        if (abs(e) > emax)
            emax = abs(e);
    }
    db = 10 * log10((es + 1e-9) / (ss + 1e-9));
    if (db > worst_db)
        worst_db = db;
    if (db > limit_db) {
        printf("FAIL %-44s error %7.1f dB (limit %.0f), max %d\n", what, db, limit_db, emax);
        (*bad)++;
    }
}

static int cmp_osc(void)
{
    static const uint32_t W_JP[3] = {JW_SAW, JW_SQR, JW_TRIM}, W_MV[3] = {MV_W_SAW, MV_W_SQR, MV_W_TRIMOD};
    static const char *const NM[3] = {"SAW", "SQR", "TRI M"};
    static int32_t a[N], b[N];
    int bad = 0;
    uint32_t w, k, note;
    worst_db = -1e9;
    for (w = 0; w < 3u; w++)
        for (k = 0; k <= 127u; k += 9u)
            for (note = 24; note <= 96; note += 12) {
                jp_osc_t o;
                mv_osc_t m;
                uint32_t ph = 0x12345678u, inc = pitch_inc(note * 16u), i;
                char id[64];
                memset(&m, 0, sizeof m);
                jp_shape(&o, W_JP[w], jp_k20((int32_t)k << 8));
                m.ph = ph;
                m.inc = inc;
                mv_osc_shape(&m, W_MV[w], k, 0);
                for (i = 0; i < N; i += 32) {
                    jp_osc_run(&o, W_JP[w], &ph, inc, a + i, 32);
                    mv_osc_render(&m, W_MV[w], b + i, 32);
                }
                snprintf(id, sizeof id, "%s shape %u note %u", NM[w], k, note);
                report(id, a, b, N, -70, &bad);
            }
    printf("%s oscillators SAW / SQR / TRI M: %d cases, worst error %.1f dB\n", bad ? "FAIL" : "ok", 3 * 15 * 7,
           worst_db);
    return bad;
}

/* the depth M-vance's mv_ssaw_set reaches for a DETUNE byte: its seven steps, back to depth = (inc_i / inc - 1) /
 * offset_i, read from the outer voice (offset -0.110) */
static double mv_depth(uint32_t det, uint32_t inc)
{
    mv_ssaw_t s;
    memset(&s, 0, sizeof s);
    mv_ssaw_set(&s, inc, det, 64);
    return ((double)(int32_t)(s.inc[0] - inc) / inc) / -0.11002313;
}

static int cmp_ssaw(void)
{
    static int32_t a[N], b[N];
    static const uint32_t DET[] = {0, 3, 6, 16, 32, 64, 90, 110, 127}, MIX[] = {0, 40, 64, 100, 127};
    int bad = 0, flagged = 0;
    uint32_t di, mi, note, d;
    double worst_ct = 0;
    worst_db = -1e9;
    for (di = 0; di < sizeof DET / sizeof DET[0]; di++)
        for (mi = 0; mi < sizeof MIX / sizeof MIX[0]; mi++)
            for (note = 36; note <= 84; note += 24) {
                jp_vx_t x;
                mv_ssaw_t s;
                uint32_t inc = pitch_inc(note * 16u), i, k, r = 0x9E3779B9u;
                char id[64];
                memset(&x, 0, sizeof x);
                memset(&s, 0, sizeof s);
                for (k = 0; k < 7u; k++) {
                    r ^= r << 13;
                    r ^= r >> 17;
                    r ^= r << 5;
                    x.sph[k] = s.ph[k] = r;
                }
                mv_ssaw_set(&s, inc, DET[di], MIX[mi]);
                {   /* the steps: within 0.01 cent of M-vance's; then the same steps into both loops */
                    jp_ss_t ss;
                    jp_ssaw_set(&ss, inc, (int32_t)DET[di] << 8, MIX[mi]);
                    for (k = 0; k < 7u; k++) {
                        double ce = fabs(1200 * log2((double)ss.si[k] / s.inc[k]));
                        if (ce > worst_ct)
                            worst_ct = ce;
                        ss.si[k] = s.inc[k];
                    }
                    for (i = 0; i < N; i += 32) {
                        jp_ssaw_run(&x, &ss, a + i, 32);
                        mv_ssaw_render(&s, b + i, 16);
                        mv_ssaw_render(&s, b + i + 16, 16);
                    }
                }
                snprintf(id, sizeof id, "SSAW detune %u mix %u note %u", DET[di], MIX[mi], note);
                report(id, a, b, N, -70, &bad);
            }
    printf("%s SUPER SAW: %d cases, steps within %.4f cent of M-vance's, then the waves: worst error %.1f dB\n",
           bad || worst_ct > 0.01 ? "FAIL" : "ok", 9 * 5 * 3, worst_ct, worst_db);
    bad += worst_ct > 0.01;
    /* the detune depth over every byte: eng_jp's (signed interpolation) against M-vance's mv_ssaw_set */
    for (d = 0; d <= 127u; d++) {
        uint32_t inc = pitch_inc(60u * 16u), pos = d * 255u * 256u / 127u, li = pos >> 8, fr = pos & 255u;
        double ref = li >= 255u ? JP_SSAW_DETUNE_Q24[255] / 16777216.0
                                : (JP_SSAW_DETUNE_Q24[li] + ((double)JP_SSAW_DETUNE_Q24[li + 1] -
                                                             JP_SSAW_DETUNE_Q24[li]) * fr / 256.0) / 16777216.0;
        double mv = mv_depth(d, inc);
        if (fabs(mv - ref) > 1e-3) {
            printf("note M-vance SSAW DETUNE %3u: depth %.4f, the LUT says %.4f\n", d, mv, ref);
            flagged++;
        }
    }
    if (flagged)
        printf("note M-vance mv_ssaw_set: %d DETUNE bytes off the LUT (eng_jp follows the LUT; see the delivery)\n", flagged);
    return bad;
}

static int cmp_fbk(void)
{
    /* grid points whose wander is 0 (no random walk): the two must agree sample for sample */
    static const uint8_t FB[] = {0, 32, 48, 64, 64, 80};
    static const uint8_t H[] = {64, 64, 32, 16, 48, 64};
    static int32_t a[N], b[N];
    int bad = 0;
    uint32_t c, note;
    worst_db = -1e9;
    for (c = 0; c < sizeof FB; c++)
        for (note = 36; note <= 84; note += 24) {
            jp_vx_t x;
            mv_jfbk_t f;
            uint32_t inc = pitch_inc(note * 16u), ph = 0x31415926u, i;
            int32_t rng = 1;
            char id[64];
            memset(&x, 0, sizeof x);
            mv_jfbk_init(&f, 1u);
            mv_jfbk_key(&f, ph);
            for (i = 0; i < N; i += 32) {
                jp_fb_law(&x, H[c], FB[c], inc >= JP_FB_SPLIT_INC);
                jp_fbk(&x, &ph, inc, a + i, 32, &rng);
                mv_jfbk_set(&f, inc, H[c], FB[c]);
                mv_jfbk_render(&f, b + i, 16);
                mv_jfbk_render(&f, b + i + 16, 16);
            }
            snprintf(id, sizeof id, "FBK fb %u harm %u note %u", FB[c], H[c], note);
            report(id, a, b, N, -50, &bad);
        }
    printf("%s FEEDBACK (no-wander cells): %d cases, worst error %.1f dB\n", bad ? "FAIL" : "ok", 6 * 3, worst_db);
    return bad;
}

static int cmp_filter(void)
{
    static const uint32_t RES[] = {0, 32, 64, 96, 110, 116}, CUT[] = {5, 20, 40, 60, 80, 100, 115, 121};
    static int32_t src[N], a[N], b[N];
    const double spo = 11.9590793, f64 = 549.658813;   /* M-vance's cutoff law (docs/ref/jt_laws.json) */
    int bad = 0;
    uint32_t mode, ri, ci, i;
    double worst_gc = 0;
    {   /* the source: a JP saw at note 36 */
        jp_osc_t o;
        uint32_t ph = 0;
        jp_shape(&o, JW_SAW, 0);
        for (i = 0; i < N; i += 32)
            jp_osc_run(&o, JW_SAW, &ph, pitch_inc(36u * 16u), src + i, 32);
    }
    worst_db = -1e9;
    for (mode = JF_LP24; mode <= JF_BP12; mode++)
        for (ri = 0; ri < sizeof RES / sizeof RES[0]; ri++)
            for (ci = 0; ci < sizeof CUT / sizeof CUT[0]; ci++) {
                jp_vx_t x;
                voice_t v;
                mv_flt_t f;
                uint32_t cut = CUT[ci];
                int32_t kq8 = (int32_t)cut << 8;
                double hz = 30.0 * pow(16000.0 / 30.0, cut / 127.0);
                int32_t code = (int32_t)lround((64 + spo * log2(hz / f64)) * 256);
                char id[64];
                memset(&x, 0, sizeof x);
                memset(&v, 0, sizeof v);
                memset(&f, 0, sizeof f);
                mv_flt_set(&f, mode, RES[ri], 64);    /* (byte 64: no R5a; these resonances have none either) */
                mv_flt_reset(&f);
                for (i = 0; i < N; i += 32) {
                    memcpy(a + i, src + i, 32 * sizeof(int32_t));
                    memcpy(b + i, src + i, 32 * sizeof(int32_t));
                    mv_flt_cutoff(&f, code);
                    {   /* g: eng_jp's (Felucca's knob law) against M-vance's at the same Hz, as the pole, in cents */
                        double ce = fabs(1200 * log2(atan(jp_filter_g(mode, kq8) / 67108864.0) /
                                                     atan(f.g / 67108864.0)));
                        if (ce > worst_gc)
                            worst_gc = ce;
                    }
                    jp_filter_run(&x, &v, a + i, 32, mode, RES[ri], cut, (uint32_t)f.g);   /* the same g into both */
                    mv_flt_render(&f, b + i, 32);
                }
                snprintf(id, sizeof id, "FILTER mode %u res %u cut %u (%.0f Hz)", mode, RES[ri], cut, hz);
                report(id, a + 1024, b + 1024, N - 1024, -45, &bad);
            }
    printf("%s FILTER: %d cases, the same g: worst error %.1f dB; g on Felucca's law against M-vance's at the same Hz: "
           "within %.3f cent (the pole)\n", bad ? "FAIL" : "ok", 6 * 6 * 8, worst_db, worst_gc);
    {   /* R5a: eng_jp's gain at whole JP codes above 64 against M-vance's (its exp2n on the knob byte) */
        uint32_t r, c, rb = 0;
        for (r = 117; r <= 127u; r++)
            for (c = 65; c <= 127u; c++) {
                mv_flt_t f;
                int32_t mine = jp_exp2n_q30((int32_t)(((uint32_t)-JP_LC_Q16[r] * ((c - 64u) << 8)) >> 8));
                memset(&f, 0, sizeof f);
                mv_flt_set(&f, MV_FLT_LP24, r, c);
                if (mine != f.out)
                    rb++;
            }
        printf("%s R5a level compensation: %u of %u gains differ from M-vance's\n", rb ? "FAIL" : "ok", rb, 11u * 63u);
        bad += rb != 0;
    }
    return bad;
}

int main(void)
{
    int bad = cmp_osc() + cmp_ssaw() + cmp_fbk() + cmp_filter();
    printf("%s: %d failures\n", bad ? "FAIL" : "ok", bad);
    return bad != 0;
}
