/* SPDX-License-Identifier: GPL-3.0-only */
/* eng_jp host tests: the host's part of the unity build (Felucca's shapes: track_t, the parts' engine memory as
 * engines.c), then dsp.c, ANALOG (the yardstick) and JP. Included once, by jp_host.c or cmp_jp.c */
#include <stdint.h>
#include <string.h>
#define NVOICE 8
#define NPART 4
#include "fe_tables.h"
#include "felucca_engine.h"

enum { P_ATK, P_REL, P_FTYPE, P_E0, P_E1, P_E2, P_E3, P_E4, P_E5, P_E6, P_E7, P_COUNT };
typedef struct track {
    int16_t p[P_COUNT];
    voice_t v[NVOICE];
} track_t;
static track_t trk[NPART];

/* the parts' engine memory, as Felucca's engines.c */
#define ENG_MEM_PART 12888u
static union {
    uint64_t align;
    uint8_t b[ENG_MEM_PART];
} eng_mem[NPART];
#define ENG_MEM(p) ((void *)eng_mem[p].b)
#define ENG_MEM_FITS(name, size) _Static_assert((size) <= ENG_MEM_PART, name " state over ENG_MEM_PART")

#include "dsp.c"
#include "eng_analog.c"
#include "eng_jp.c"
