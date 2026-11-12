/* SPDX-License-Identifier: GPL-3.0-only */
/* eng_jp host tests: a minimal Felucca-like host (the engine interface of felucca_engine.h, voice starts and
 * retriggers as Felucca's voice.c does them, the part's engine memory as engines.c) around eng_jp.c, with ANALOG
 * (Felucca-Engines) beside it as the yardstick for level and cost.
 *
 *   jp_host golden          every preset, every WAVE x TYPE, sweeps of every knob: one line each (hash, peak, rms,
 *                           dc, clips); a change of the sound changes the lines (tests/jp_golden.txt)
 *   jp_host health          the same renders checked: no clip at the voice, |dc| small, silence after the release
 *   jp_host level           JP's SAW against ANALOG's SAW, open filter, same phrase: the RMS ratio in dB
 *   jp_host retrig          the largest sample step at a retrigger against the largest within the notes
 *   jp_host wav DIR         every preset's phrase as DIR/jp_NN_NAME.wav (44.1 kHz, 16-bit, mono)
 *   jp_host bench E P N     engine E (jp / analog) preset P, 8 voices held, N blocks (for valgrind --tool=callgrind)
 *   jp_host stress SEED N   N random knob / modulation settings, every block (run it under -fsanitize=undefined)
 *
 * Build: tests/jp/run_jp.sh */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "jp_prelude.h"          /* the host's part: track, engine memory, dsp.c, ANALOG, JP */

/* ------------------------------------------------------------------------------------- the voice host --- */
static uint32_t vage;

/* Felucca voice.c voice_start: a voice that is not sounding starts its envelope at 0; a sounding one keeps its
 * phases and the engine's keep slots over note_on */
static void voice_on(track_t *t, const engine_t *e, voice_t *v, uint32_t note, int32_t fine)
{
    int sounding = v->active && v->stage != 0;
    uint32_t ph0 = v->ph[0], ph1 = v->ph[1], ph2 = v->ph[2], k;
    int32_t s[8];
    memcpy(s, v->s, sizeof s);
    v->note = (uint8_t)note;
    v->vel = 100;
    v->gate = 1;
    v->active = 1;
    v->stage = 1;
    v->age = ++vage;
    v->pitch16 = v->pitch_cur = (int32_t)note * 16;
    v->fine = fine;
    if (!sounding)
        v->env = v->env_out = 0;
    e->note_on(t, v);
    if (sounding && !e->sampled) {
        v->ph[0] = ph0;
        v->ph[1] = ph1;
        v->ph[2] = ph2;
        for (k = 0; k < 8u; k++)
            if (k != 2u && k != 3u && (e->keep >> k & 1u))
                v->s[k] = s[k];
    }
}

/* an envelope a block: attack in 4 blocks, sustain 3/4, release 1/16 a block (fe_host's) */
static int32_t env_tick(voice_t *v)
{
    if (!v->active)
        return 0;
    switch (v->stage) {
    case 1:
        v->env += 1 << 22;
        if (v->env >= 1 << 24) {
            v->env = 1 << 24;
            v->stage = 2;
        }
        break;
    case 2:
        v->env -= (v->env - (3 << 22)) >> 3;
        break;
    case 3:
        v->env -= v->env >> 4;
        if (v->env < (1 << 24) / 1000) {
            v->env = 0;
            v->stage = 0;
            v->active = 0;
        }
        break;
    default:
        break;
    }
    v->env_out = v->env >> 9;
    return v->env >> 9;
}

typedef struct {
    uint64_t h;
    int32_t peak, clips;
    double sum, sum2;
    uint32_t n;
    int32_t tail;                     /* largest |sample| in the last 10 blocks (after the release) */
    int32_t jump_rt, jump_in;         /* largest step at a retrigger block's start / inside the notes */
} res_t;

static int32_t mod_cut, mod_shp;      /* the host's routing this block (set by the phrase) */
static int16_t *wav_buf;              /* when set: the render, out >> 1 clamped to 16 bits */
static uint32_t wav_n;

/* one block of part t: every active voice, the host's modulation, into out (n = CTL) */
static void render_block(track_t *t, const engine_t *e, int32_t *out, int32_t *last)
{
    uint32_t k;
    memset(out, 0, sizeof(int32_t) * CTL);
    if (e->block)
        e->block(t);
    for (k = 0; k < NVOICE; k++) {
        voice_t *v = &t->v[k];
        vmod_t m;
        int32_t a;
        uint32_t base;
        if (!v->active)
            continue;
        a = env_tick(v);
        if (!v->active)
            continue;
        base = pitch_inc((uint32_t)clamp(v->pitch16, 0, 2047));
        memset(&m, 0, sizeof m);
        m.pitch16 = v->pitch16;
        m.inc = base + (uint32_t)((int32_t)(base >> 12) * v->fine);
        m.fine = v->fine;
        m.amp0 = last[k];
        m.amp1 = a;
        m.envq15 = a;
        m.cutoff = mod_cut;
        m.shape = (64 << 8) + mod_shp;
        last[k] = a;
        e->render(t, v, out, CTL, &m);
    }
}

#define NBLK 240

/* the phrase (fe_host's): a low note, a detuned voice an octave and a fifth up, the first voice retriggered on
 * another note, a third note, all released, the tails. sweep: the host's cutoff and shape routing move */
static res_t render(const engine_t *e, const int8_t *ep, uint32_t part, int sweep)
{
    track_t *t = &trk[part];
    int32_t out[CTL], last[NVOICE] = {0}, prev = 0;
    uint32_t b, i, k;
    res_t r;
    memset(&r, 0, sizeof r);
    r.h = 14695981039346656037ull;
    memset(t, 0, sizeof *t);
    memset(&eng_mem[part], 0, sizeof eng_mem[part]);   /* engines.c eng_mem_claim: zeroed for a new owner */
    vage = 0;
    for (k = 0; k < 8u; k++)
        t->p[P_E0 + k] = ep[k];
    t->p[P_ATK] = 0;
    t->p[P_REL] = 60;
    wav_n = 0;
    for (b = 0; b < NBLK; b++) {
        int rt = 0;
        if (b == 0)
            voice_on(t, e, &t->v[0], 43, 0);
        if (b == 30)
            voice_on(t, e, &t->v[1], 62, 23);
        if (b == 70) {
            voice_on(t, e, &t->v[0], 50, 0);          /* a retrigger of a sounding voice */
            rt = 1;
        }
        if (b == 100)
            voice_on(t, e, &t->v[2], 79, -17);
        if (b == 150)
            for (k = 0; k < NVOICE; k++)
                if (t->v[k].active) {
                    t->v[k].gate = 0;
                    t->v[k].stage = 3;
                }
        mod_cut = sweep ? ((int32_t)(b % 96u) - 48) * 128 : 0;
        mod_shp = sweep ? ((int32_t)(b % 64u) - 32) * 256 : 0;
        render_block(t, e, out, last);
        for (i = 0; i < CTL; i++) {
            uint32_t x = (uint32_t)out[i];
            int32_t a = out[i] < 0 ? -out[i] : out[i], st = out[i] - prev;
            if (st < 0)
                st = -st;
            if (rt && i < 2u) {
                if (st > r.jump_rt)
                    r.jump_rt = st;
            } else if (b > 4 && b < 150 && st > r.jump_in) {
                r.jump_in = st;
            }
            prev = out[i];
            for (k = 0; k < 4u; k++) {
                r.h ^= (x >> (8u * k)) & 0xFFu;
                r.h *= 1099511628211ull;
            }
            if (a > r.peak)
                r.peak = a;
            if (a > 65535)
                r.clips++;
            if (b >= NBLK - 10 && a > r.tail)
                r.tail = a;
            r.sum += out[i];
            r.sum2 += (double)out[i] * out[i];
            r.n++;
            if (wav_buf) {
                int32_t y = out[i] >> 1;
                wav_buf[wav_n++] = (int16_t)clamp(y, -32767, 32767);
            }
        }
    }
    return r;
}

static double rms_of(const res_t *r) { return r->n ? __builtin_sqrt(r->sum2 / r->n) : 0.0; }
static double dc_of(const res_t *r) { return r->n ? r->sum / r->n : 0.0; }

static void line(const char *id, const res_t *r)
{
    printf("%s %016llx %d %.0f %.1f %d\n", id, (unsigned long long)r->h, r->peak, rms_of(r), dc_of(r), r->clips);
}

static void defaults(int8_t *ep)
{
    uint32_t k;
    for (k = 0; k < 8u; k++)
        ep[k] = (int8_t)ENG_JP.edit[k].def;
}

/* ANALOG's SAW at the open filter through the phrase: the level yardstick */
static res_t analog_ref(void)
{
    int8_t ea[8];
    uint32_t k;
    for (k = 0; k < 8u; k++)
        ea[k] = (int8_t)ENG_ANALOG.edit[k].def;
    ea[0] = 0;
    ea[1] = 0;
    ea[2] = 0;
    ea[4] = 127;
    ea[5] = 0;
    ea[6] = 0;
    return render(&ENG_ANALOG, ea, 1, 0);
}

/* health of one render: FAIL when the tail (the last 10 blocks, 80 after the release) is neither 40 dB under the
 * render's peak nor 60 dB under ANALOG's open SAW (a voice that never ends, a filter stuck or ringing on its own);
 * warnings: DC above 3 % of the peak, a peak above 4 x ANALOG's open SAW (a hot resonance, the JP's) */
static int warn_dc, warn_hot;
static int health(const char *id, const res_t *r, int32_t ref, int verbose)
{
    double dc = dc_of(r);
    int fail = r->tail * 100 > r->peak && r->tail * 1000 > ref, wdc = (dc < 0 ? -dc : dc) > 0.03 * r->peak, whot = r->peak > 4 * ref;
    warn_dc += wdc;
    warn_hot += whot;
    if (verbose || fail || wdc || whot)
        printf("%-4s %-22s peak %6d (%.1f x ANALOG)  rms %6.0f  dc %7.1f  tail %4d%s%s\n", fail ? "FAIL" : "ok", id,
               r->peak, (double)r->peak / ref, rms_of(r), dc, r->tail, wdc ? "  [dc]" : "", whot ? "  [hot]" : "");
    return fail;
}

/* --------------------------------------------------------------------------------------------- tests --- */
static int run_golden(int check)
{
    char id[64];
    uint32_t j, w, ty;
    int bad = 0;
    const int32_t ref = check ? analog_ref().peak : 1;
    for (j = 0; j < ENG_JP.npresets; j++) {
        res_t r = render(&ENG_JP, ENG_JP.presets[j].e, j % NPART, 1);
        snprintf(id, sizeof id, "jp/%u %s", j, ENG_JP.presets[j].name);
        if (!check)
            line(id, &r);
        else
            bad += health(id, &r, ref, 1);
    }
    for (w = 0; w < JW_N; w++)                        /* every WAVE through every TYPE, mid knobs, OSC 2 in */
        for (ty = 0; ty <= JF_BP12; ty++) {
            int8_t ep[8];
            res_t r;
            defaults(ep);
            ep[JP_WAVE] = (int8_t)w;
            ep[JP_CTL1] = 64;
            ep[JP_CTL2] = 64;
            ep[JP_O2] = 7;
            ep[JP_CUT] = 90;
            ep[JP_RES] = 64;
            ep[JP_TYPE] = (int8_t)ty;
            ep[JP_BAL] = 64;
            r = render(&ENG_JP, ep, (w + ty) % NPART, 1);
            snprintf(id, sizeof id, "jp/w%u/t%u", w, ty);
            if (!check)
                line(id, &r);
            else
                bad += health(id, &r, ref, 0);
        }
    {   /* every knob over its range on each wave (the others mid), the host routing sweeping */
        static const int8_t probe[] = {0, 1, 4, 5, 32, 63, 64, 96, 126, 127};
        uint32_t knob, pi;
        for (w = 0; w < JW_N; w++)
            for (knob = JP_CTL1; knob <= JP_BAL; knob++)
                for (pi = 0; pi < sizeof probe; pi++) {
                    int8_t ep[8];
                    res_t r;
                    defaults(ep);
                    ep[JP_WAVE] = (int8_t)w;
                    ep[JP_CTL1] = 40;
                    ep[JP_CTL2] = 40;
                    ep[JP_O2] = 12;
                    ep[JP_CUT] = 100;
                    ep[JP_RES] = 40;
                    ep[JP_BAL] = 64;
                    if (knob == JP_TYPE && probe[pi] > JF_BP12)
                        continue;
                    ep[knob] = knob == JP_O2 ? (int8_t)(probe[pi] * 48 / 127 - 24) : probe[pi];
                    r = render(&ENG_JP, ep, pi % NPART, 1);
                    snprintf(id, sizeof id, "jp/w%u/k%u=%d", w, knob, ep[knob]);
                    if (!check)
                        line(id, &r);
                    else
                        bad += health(id, &r, ref, 0);
                }
    }
    if (check)
        printf("%s: %d failures (tail not silent); warnings: %d dc, %d hot\n", bad ? "FAIL" : "ok", bad, warn_dc,
               warn_hot);
    return bad != 0;
}

static int run_level(void)
{
    int8_t ep[8], ea[8];
    res_t rj, ra;
    uint32_t k;
    defaults(ep);
    ep[JP_WAVE] = JW_SAW;
    ep[JP_CUT] = 127;
    ep[JP_RES] = 0;
    ep[JP_TYPE] = JF_LP24;
    for (k = 0; k < 8u; k++)
        ea[k] = (int8_t)ENG_ANALOG.edit[k].def;
    ea[0] = 0;                                        /* ANALOG SAW */
    ea[1] = 0;                                        /* no detune */
    ea[2] = 0;                                        /* MIX: OSC 1 alone, as JP's BAL 0 */
    ea[4] = 127;                                      /* CUT open */
    ea[5] = 0;                                        /* RES 0 */
    ea[6] = 0;                                        /* no drive */
    rj = render(&ENG_JP, ep, 0, 0);
    ra = render(&ENG_ANALOG, ea, 1, 0);
    printf("JP SAW rms %.0f peak %d | ANALOG SAW rms %.0f peak %d | JP / ANALOG %+.2f dB (rms), %+.2f dB (peak)\n",
           rms_of(&rj), rj.peak, rms_of(&ra), ra.peak, 20 * __builtin_log10(rms_of(&rj) / rms_of(&ra)),
           20 * __builtin_log10((double)rj.peak / ra.peak));
    defaults(ep);
    ep[JP_WAVE] = JW_SSAW;
    ep[JP_CTL1] = 90;
    ep[JP_CTL2] = 100;
    ep[JP_CUT] = 127;
    rj = render(&ENG_JP, ep, 0, 0);
    printf("JP SSAW (90/100) rms %.0f peak %d: %+.2f dB re ANALOG SAW\n", rms_of(&rj), rj.peak,
           20 * __builtin_log10(rms_of(&rj) / rms_of(&ra)));
    return 0;
}

static int run_retrig(void)
{
    uint32_t j;
    int bad = 0;
    for (j = 0; j < ENG_JP.npresets; j++) {
        res_t r = render(&ENG_JP, ENG_JP.presets[j].e, 0, 0);
        int ok = r.jump_rt <= r.jump_in;              /* the retrigger is no bigger a step than the playing */
        printf("%-4s %-12s step at retrigger %6d, largest within the notes %6d\n", ok ? "ok" : "FAIL",
               ENG_JP.presets[j].name, r.jump_rt, r.jump_in);
        bad += !ok;
    }
    return bad != 0;
}

static void put32(FILE *f, uint32_t v) { fputc((int)(v & 255u), f); fputc((int)(v >> 8 & 255u), f); fputc((int)(v >> 16 & 255u), f); fputc((int)(v >> 24), f); }
static void put16(FILE *f, uint32_t v) { fputc((int)(v & 255u), f); fputc((int)(v >> 8 & 255u), f); }

static int run_wav(const char *dir)
{
    static int16_t buf[NBLK * CTL];
    uint32_t j;
    wav_buf = buf;
    for (j = 0; j < ENG_JP.npresets; j++) {
        char path[512], nm[16];
        uint32_t c;
        FILE *f;
        render(&ENG_JP, ENG_JP.presets[j].e, 0, 0);
        for (c = 0; c < 15u && ENG_JP.presets[j].name[c]; c++)
            nm[c] = ENG_JP.presets[j].name[c] == ' ' ? '_' : ENG_JP.presets[j].name[c];
        nm[c] = 0;
        snprintf(path, sizeof path, "%s/jp_%02u_%s.wav", dir, j, nm);
        f = fopen(path, "wb");
        if (!f)
            return 1;
        fwrite("RIFF", 1, 4, f);
        put32(f, 36u + wav_n * 2u);
        fwrite("WAVEfmt ", 1, 8, f);
        put32(f, 16);
        put16(f, 1);
        put16(f, 1);
        put32(f, FS);
        put32(f, FS * 2u);
        put16(f, 2);
        put16(f, 16);
        fwrite("data", 1, 4, f);
        put32(f, wav_n * 2u);
        for (c = 0; c < wav_n; c++)
            put16(f, (uint16_t)buf[c]);
        fclose(f);
        printf("%s\n", path);
    }
    wav_buf = 0;
    return 0;
}

/* 8 voices held on one part for nblk blocks: the cost of the engine's render (callgrind counts it) */
static int run_bench(const char *eng, uint32_t pre, uint32_t nblk)
{
    const engine_t *e = !strcmp(eng, "analog") ? &ENG_ANALOG : &ENG_JP;
    track_t *t = &trk[0];
    int32_t out[CTL], last[NVOICE] = {0};
    uint32_t b, k;
    volatile int32_t sink = 0;
    if (pre >= e->npresets)
        return 1;
    memset(t, 0, sizeof *t);
    for (k = 0; k < 8u; k++)
        t->p[P_E0 + k] = e->presets[pre].e[k];
    for (k = 0; k < NVOICE; k++)
        voice_on(t, e, &t->v[k], 48 + 3 * k, (int32_t)k * 7 - 24);
    for (b = 0; b < nblk; b++) {
        mod_cut = ((int32_t)(b % 96u) - 48) * 64;
        mod_shp = ((int32_t)(b % 64u) - 32) * 128;
        render_block(t, e, out, last);
        if (t->v[0].stage == 3)
            break;
        sink += out[0];
    }
    printf("%s/%s: %u blocks x %u voices (%u samples)\n", eng, e->presets[pre].name, b, NVOICE, b * CTL);
    return 0;
}

/* random knobs and host routing every block, 4 parts, voices starting and stopping (for the sanitizers) */
static int run_stress(uint32_t seed, uint32_t n)
{
    int32_t out[CTL], last[NPART][NVOICE];
    uint32_t it, b, k;
    int32_t rs = (int32_t)(seed | 1u);
    memset(trk, 0, sizeof trk);
    memset(eng_mem, 0, sizeof eng_mem);
    memset(last, 0, sizeof last);
    for (it = 0; it < n; it++) {
        track_t *t = &trk[it % NPART];
        for (k = 0; k < 8u; k++) {
            const param_desc_t *d = &ENG_JP.edit[k];
            t->p[P_E0 + k] = (int16_t)(d->min + (int32_t)(noise32(&rs) % (uint32_t)(d->max - d->min + 1)));
        }
        for (b = 0; b < 8u; b++) {
            uint32_t r = noise32(&rs);
            voice_t *v = &t->v[r % NVOICE];
            if ((r >> 8) % 3u == 0u)
                voice_on(t, &ENG_JP, v, (r >> 12) % 128u, (int32_t)((r >> 20) % 200u) - 100);
            else if ((r >> 8) % 3u == 1u && v->active) {
                v->gate = 0;
                v->stage = 3;
            }
            mod_cut = (int32_t)((r >> 3) % (256u * 256u)) - 128 * 256;   /* far past both ends */
            mod_shp = (int32_t)((r >> 5) % (252u * 256u)) - 126 * 256;
            render_block(t, &ENG_JP, out, last[it % NPART]);
        }
    }
    printf("stress: %u settings x 8 blocks, done\n", n);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2)
        return 2;
    if (!strcmp(argv[1], "golden"))
        return run_golden(0);
    if (!strcmp(argv[1], "health"))
        return run_golden(1);
    if (!strcmp(argv[1], "level"))
        return run_level();
    if (!strcmp(argv[1], "retrig"))
        return run_retrig();
    if (!strcmp(argv[1], "wav") && argc > 2)
        return run_wav(argv[2]);
    if (!strcmp(argv[1], "bench") && argc > 4)
        return run_bench(argv[2], (uint32_t)atoi(argv[3]), (uint32_t)atoi(argv[4]));
    if (!strcmp(argv[1], "stress") && argc > 3)
        return run_stress((uint32_t)atoi(argv[2]), (uint32_t)atoi(argv[3]));
    return 2;
}
