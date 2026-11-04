/* SPDX-License-Identifier: MIT */
/**
 * FM-aware NanoMPX compressed codec (v6 wire: magic 0xC6).
 *
 * The composite is split into the bands an FM MPX carries:
 *   - pilot   parametric: a PLL in the encoder tracks the 19 kHz pilot; every
 *             packet carries its phase, frequency and amplitude, and the
 *             decoder regenerates it (clean, phase-continuous)
 *   - mono    0–15 kHz, low-passed and decimated to 48 kHz
 *   - stereo  L−R, demodulated with sin(2θ) of the tracked pilot, 48 kHz
 *   - RDS     57 kHz ±2.4 kHz as complex (I/Q) baseband at 6 kHz, so any RDS
 *             carrier phase passes
 * Band filters are 577-tap Kaiser-windowed sincs (≈68 dB stop band), so the
 * bands neither alias nor leak into each other.
 *
 * Each band is transform coded: sine-window MDCT (hop 256 at 48 kHz, 32 at
 * 6 kHz: 5.3 ms), a quantiser step per group of bins chosen from the block's
 * own spectrum (noise kept under a flat floor and a set distance below the
 * local signal: nothing is spent on empty bins, no noise shows above the
 * programme), and an adaptive range coder whose models follow the stream
 * (reset on keyframes). Codec delay: 576 (filters) + 2048 (MDCT) samples,
 * 13.7 ms.
 */

#include "internal.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define COMP_MAGIC 0xC6
#define PILOT_HZ 19000.0
#define W19 (2.0 * M_PI * PILOT_HZ / (double)NANOMPX_SAMPLE_RATE)

#define NF 577                   /* taps of every band filter */
#define DEC_A 4                  /* mono / stereo: 192 kHz -> 48 kHz */
#define DEC_R 32                 /* RDS I/Q: 192 kHz -> 6 kHz */
#define FC_A 15750.0             /* audio cut-off: pass 15.0 kHz, stop 16.5 kHz */
#define FC_R 2950.0              /* RDS cut-off: pass 2.2 kHz, stop 3.7 kHz */
#define KAISER_BETA 6.5

#define NBANDS 4                 /* mono, stereo, RDS I, RDS Q */
#ifndef H_A
#define H_A 512                  /* MDCT hop, audio bands */
#endif
#define H_R (H_A * DEC_A / DEC_R) /* MDCT hop, RDS bands: the same delay */
#define H_MAX H_A
#define K_A (H_A * 11 / 16)      /* audio bins coded (to 16.5 kHz) */
#define K_R (H_R * 7 / 8)        /* RDS bins coded (to 2.6 kHz) */
#ifndef BW_A
#define BW_A (H_A / 32)          /* bins per quantiser step (750 Hz) */
#endif
#define BW_R (H_R / 8)
#define NSF_MAX (K_A / BW_A)
#define D_EXTRA (2 * H_A * DEC_A)        /* MDCT delay at 192 kHz (= 2·H_R·DEC_R) */
#define CODEC_DELAY (NF - 1 + D_EXTRA)

#define RING 32768               /* decimated samples per band by index & (RING-1) */
#define PRING 16                 /* decoder: pilot parameters of recent frames */

/* Pilot PLL, per packet: phase and frequency corrections. */
#define PLL_KP 0.1
#define PLL_KI 0.005
#define PILOT_MIN 0.003          /* below: no pilot, the loop holds */
/* θ in exact integers, so encoder and decoder predict it alike on any CPU:
 * phase in 2^-24 cycles, frequency in 2^-40 cycles per sample. */
#define PH_ONE 16777216.0
#define WQ_ONE 1099511627776.0
#define AMP_LSB (0.25 / 65536.0)         /* 16-bit amplitude */

#define HDR_BYTES 4              /* magic, profile, count */

#ifdef NANOMPX_STATS
size_t nanompx_stat_bytes[4];    /* bench: bytes per band */
#endif

/* --- band filters ----------------------------------------------------------- */

static float h_a[NF], h_r[NF];   /* DC gain 1 */

static double bessel_i0(double x)
{
    double s = 1.0, t = 1.0;
    int k;
    for (k = 1; k < 40; k++) {
        t *= (x / (2.0 * k)) * (x / (2.0 * k));
        s += t;
    }
    return s;
}

static void design_lp(float *h, double fc)
{
    const int m = (NF - 1) / 2;
    double sum = 0.0, wc = 2.0 * fc / (double)NANOMPX_SAMPLE_RATE;
    double i0b = bessel_i0(KAISER_BETA);
    double tmp[NF];
    int k;
    for (k = 0; k < NF; k++) {
        double t = (double)(k - m);
        double r = t / (double)m;
        double sinc = t == 0.0 ? wc : sin(M_PI * wc * t) / (M_PI * t);
        tmp[k] = sinc * bessel_i0(KAISER_BETA * sqrt(1.0 - r * r)) / i0b;
        sum += tmp[k];
    }
    for (k = 0; k < NF; k++)
        h[k] = (float)(tmp[k] / sum);
}

static int band_dec(int b) { return b < 2 ? DEC_A : DEC_R; }
static const float *band_h(int b) { return b < 2 ? h_a : h_r; }
static int band_hop(int b) { return b < 2 ? H_A : H_R; }
static int band_k(int b) { return b < 2 ? K_A : K_R; }
static int band_bw(int b) { return b < 2 ? BW_A : BW_R; }

/* floor(a / b) for b > 0 */
static int64_t fdiv(int64_t a, int64_t b)
{
    return a >= 0 ? a / b : -((-a + b - 1) / b);
}

/* Decimated samples of a band are those at 192 kHz indices m·dec. */
static int64_t mcount(int64_t x, int dec) { return fdiv(x + dec - 1, dec); }

/* MDCT blocks complete once the samples up to x are in: j < J(x). */
static int64_t blocks_upto(int64_t x, int b)
{
    return fdiv(mcount(x, band_dec(b)), band_hop(b)) - 1;
}

/* --- MDCT (sine window, orthonormal, via a half-size complex FFT) ---------- */

typedef struct mdct_plan {
    int h, m;                    /* hop, FFT size h/2 */
    float win[2 * H_MAX];
    float pre_c[H_MAX / 2], pre_s[H_MAX / 2], post_c[H_MAX / 2], post_s[H_MAX / 2];
    float tw_c[H_MAX / 4], tw_s[H_MAX / 4];
    int rev[H_MAX / 2];
    float scale;
} mdct_plan_t;

static mdct_plan_t plan_a, plan_r;
static int tables_ready;

static void plan_init(mdct_plan_t *p, int h)
{
    int i, bits = 0;
    p->h = h;
    p->m = h / 2;
    while ((1 << bits) < p->m)
        bits++;
    for (i = 0; i < 2 * h; i++)
        p->win[i] = (float)sin(M_PI * (i + 0.5) / (2.0 * h));
    for (i = 0; i < p->m; i++) {
        int r = 0, b;
        for (b = 0; b < bits; b++)
            if (i & (1 << b))
                r |= 1 << (bits - 1 - b);
        p->rev[i] = r;
        p->pre_c[i] = (float)cos(-M_PI * (4 * i + 1) / (4.0 * h));
        p->pre_s[i] = (float)sin(-M_PI * (4 * i + 1) / (4.0 * h));
        p->post_c[i] = (float)cos(-M_PI * i / (double)h);
        p->post_s[i] = (float)sin(-M_PI * i / (double)h);
    }
    for (i = 0; i < p->m / 2; i++) {
        p->tw_c[i] = (float)cos(-2.0 * M_PI * i / p->m);
        p->tw_s[i] = (float)sin(-2.0 * M_PI * i / p->m);
    }
    p->scale = (float)sqrt(2.0 / h);
}

static void tables_init(void)
{
    if (tables_ready)
        return;
    design_lp(h_a, FC_A);
    design_lp(h_r, FC_R);
    plan_init(&plan_a, H_A);
    plan_init(&plan_r, H_R);
    tables_ready = 1;
}

static void fft(const mdct_plan_t *p, float *re, float *im)
{
    int i, len, m = p->m;
    for (i = 0; i < m; i++) {
        int r = p->rev[i];
        if (i < r) {
            float t = re[i]; re[i] = re[r]; re[r] = t;
            t = im[i]; im[i] = im[r]; im[r] = t;
        }
    }
    for (len = 2; len <= m; len <<= 1) {
        int half = len >> 1, step = m / len, j;
        for (i = 0; i < m; i += len)
            for (j = 0; j < half; j++) {
                float wr = p->tw_c[j * step], wi = p->tw_s[j * step];
                float xr = re[i + j + half], xi = im[i + j + half];
                float tr = xr * wr - xi * wi, ti = xr * wi + xi * wr;
                re[i + j + half] = re[i + j] - tr;
                im[i + j + half] = im[i + j] - ti;
                re[i + j] += tr;
                im[i + j] += ti;
            }
    }
}

/* out[k] = Σ v[n]·cos(π/h·(n+½)(k+½)), k, n < h */
static void dct4(const mdct_plan_t *p, const float *v, float *out)
{
    float re[H_MAX / 2], im[H_MAX / 2];
    int i, h = p->h, m = p->m;
    for (i = 0; i < m; i++) {
        float a = v[2 * i], b = v[h - 1 - 2 * i];
        re[i] = a * p->pre_c[i] - b * p->pre_s[i];
        im[i] = a * p->pre_s[i] + b * p->pre_c[i];
    }
    fft(p, re, im);
    for (i = 0; i < m; i++) {
        float dr = re[i] * p->post_c[i] - im[i] * p->post_s[i];
        float di = re[i] * p->post_s[i] + im[i] * p->post_c[i];
        out[2 * i] = dr;
        out[h - 1 - 2 * i] = -di;
    }
}

/* X[k] from 2h windowed samples x */
static void mdct_fwd(const mdct_plan_t *p, const float *x, float *X)
{
    float v[H_MAX];
    int i, h = p->h, q = h / 2;
    for (i = 0; i < q; i++)
        v[i] = -x[3 * q - 1 - i] * p->win[3 * q - 1 - i] - x[3 * q + i] * p->win[3 * q + i];
    for (i = q; i < h; i++)
        v[i] = x[i - q] * p->win[i - q] - x[3 * q - 1 - i] * p->win[3 * q - 1 - i];
    dct4(p, v, X);
    for (i = 0; i < h; i++)
        X[i] *= p->scale;
}

/* 2h windowed output samples y for overlap-add */
static void mdct_inv(const mdct_plan_t *p, const float *X, float *y)
{
    float u[H_MAX];
    int i, h = p->h, q = h / 2;
    dct4(p, X, u);
    for (i = 0; i < q; i++) {
        y[3 * q - 1 - i] = -u[i];
        y[3 * q + i] = -u[i];
    }
    for (i = q; i < h; i++) {
        y[i - q] = u[i];
        y[3 * q - 1 - i] = -u[i];
    }
    for (i = 0; i < 2 * h; i++)
        y[i] *= p->scale * p->win[i];
}

static const mdct_plan_t *band_plan(int b) { return b < 2 ? &plan_a : &plan_r; }

/* --- range coder (LZMA style) ---------------------------------------------- */

typedef struct rc_enc {
    uint64_t low;
    uint32_t range;
    uint8_t cache;
    uint64_t cache_size;
    uint8_t *buf;
    size_t pos, cap;
    int err;
} rc_enc_t;

static void rce_init(rc_enc_t *r, uint8_t *buf, size_t cap)
{
    memset(r, 0, sizeof(*r));
    r->range = 0xffffffffu;
    r->cache_size = 1;
    r->buf = buf;
    r->cap = cap;
}

static void rce_byte(rc_enc_t *r, uint8_t b)
{
    if (r->pos < r->cap)
        r->buf[r->pos++] = b;
    else
        r->err = 1;
}

static void rce_shift(rc_enc_t *r)
{
    if ((uint32_t)r->low < 0xff000000u || (r->low >> 32) != 0) {
        uint8_t carry = (uint8_t)(r->low >> 32), t = r->cache;
        do {
            rce_byte(r, (uint8_t)(t + carry));
            t = 0xff;
        } while (--r->cache_size);
        r->cache = (uint8_t)(r->low >> 24);
    }
    r->cache_size++;
    r->low = (r->low & 0x00ffffffu) << 8;
}

static void rce_code(rc_enc_t *r, uint32_t cum, uint32_t freq, uint32_t tot)
{
    uint32_t q = r->range / tot;
    r->low += (uint64_t)q * cum;
    r->range = q * freq;
    while (r->range < (1u << 24)) {
        r->range <<= 8;
        rce_shift(r);
    }
}

static void rce_bits(rc_enc_t *r, uint32_t v, int n)
{
    while (n > 0) {
        int k = n > 16 ? 16 : n;
        n -= k;
        rce_code(r, (v >> n) & ((1u << k) - 1u), 1, 1u << k);
    }
}

static void rce_finish(rc_enc_t *r)
{
    /* Any value in [low, low+range) decodes the same: take the one with the
     * most trailing zero bits, flush, and drop the trailing zero bytes (the
     * decoder reads zeros past the end). */
    int k;
    for (k = 1; k <= 4; k++) {
        uint64_t mask = (1ull << (32 - 8 * k)) - 1ull;
        uint64_t v = (r->low + mask) & ~mask;
        if (v - r->low < (uint64_t)r->range) {
            r->low = v;
            break;
        }
    }
    for (k = 0; k < 5; k++)
        rce_shift(r);
    while (r->pos > 0 && r->buf[r->pos - 1] == 0)
        r->pos--;
}

typedef struct rc_dec {
    uint32_t range, code, q;
    const uint8_t *buf;
    size_t pos, len;
    int err;
} rc_dec_t;

static uint8_t rcd_byte(rc_dec_t *r)
{
    if (r->pos < r->len)
        return r->buf[r->pos++];
    /* past the end: zeros (the encoder trims them), within reason */
    r->pos++;
    if (r->pos > r->len + 4096)
        r->err = 1;
    return 0;
}

static void rcd_init(rc_dec_t *r, const uint8_t *buf, size_t len)
{
    int i;
    memset(r, 0, sizeof(*r));
    r->buf = buf;
    r->len = len;
    r->range = 0xffffffffu;
    for (i = 0; i < 5; i++)
        r->code = (r->code << 8) | rcd_byte(r);
}

static uint32_t rcd_peek(rc_dec_t *r, uint32_t tot)
{
    uint32_t v;
    r->q = r->range / tot;
    v = r->code / r->q;
    return v < tot ? v : tot - 1;
}

static void rcd_take(rc_dec_t *r, uint32_t cum, uint32_t freq)
{
    r->code -= r->q * cum;
    r->range = r->q * freq;
    while (r->range < (1u << 24)) {
        r->code = (r->code << 8) | rcd_byte(r);
        r->range <<= 8;
    }
}

static uint32_t rcd_bits(rc_dec_t *r, int n)
{
    uint32_t v = 0;
    while (n > 0) {
        int k = n > 16 ? 16 : n;
        uint32_t s = rcd_peek(r, 1u << k);
        rcd_take(r, s, 1);
        v = (v << k) | s;
        n -= k;
    }
    return v;
}

/* --- adaptive models -------------------------------------------------------- */

#define MSYM_MAX 26
#ifndef M_INC
#define M_INC 24
#endif
#ifndef M_LIMIT
#define M_LIMIT (1 << 13)
#endif

typedef struct model {
    uint16_t f[MSYM_MAX];
    uint16_t tot;
    uint8_t n;
} model_t;

static void model_init(model_t *m, int n)
{
    int i;
    m->n = (uint8_t)n;
    for (i = 0; i < n; i++)
        m->f[i] = 1;
    m->tot = (uint16_t)n;
}

static void model_update(model_t *m, int s)
{
    m->f[s] += M_INC;
    m->tot += M_INC;
    if (m->tot > M_LIMIT) {
        int i;
        m->tot = 0;
        for (i = 0; i < m->n; i++) {
            m->f[i] = (uint16_t)((m->f[i] + 1) >> 1);
            m->tot += m->f[i];
        }
    }
}

static void model_put(rc_enc_t *r, model_t *m, int s)
{
    uint32_t cum = 0;
    int i;
    for (i = 0; i < s; i++)
        cum += m->f[i];
    rce_code(r, cum, m->f[s], m->tot);
    model_update(m, s);
}

static int model_get(rc_dec_t *r, model_t *m)
{
    uint32_t v = rcd_peek(r, m->tot), cum = 0;
    int s = 0;
    while (s < m->n - 1 && cum + m->f[s] <= v)
        cum += m->f[s++];
    rcd_take(r, cum, m->f[s]);
    model_update(m, s);
    return s;
}

/* Coefficient magnitudes 0..MAG_ESC-1, MAG_ESC = escape; step deltas. */
#define MAG_ESC 17
#define NCTX 108                 /* two previous bins (3·3) × same bin last block (4) × 3 positions */
#define SD_RANGE 12              /* step delta −12..12, then escape */
#define SD_ESC (2 * SD_RANGE + 1)

typedef struct band_models {
    model_t mag[NCTX];
    float last[K_A];             /* |X| of each bin in the previous block */
    model_t sd[2];               /* first step of a block, the rest */
    int prev_first;              /* step index of the previous block's first group */
} band_models_t;

static void models_init(band_models_t *bm)
{
    int i;
    for (i = 0; i < NCTX; i++)
        model_init(&bm->mag[i], MAG_ESC + 1);
    model_init(&bm->sd[0], SD_ESC + 1);
    model_init(&bm->sd[1], SD_ESC + 1);
    memset(bm->last, 0, sizeof(bm->last));
    bm->prev_first = 40;
}

/* step = 2^(−idx/4); idx 0..127 */
static float step_of(int idx) { return exp2f(-(float)idx * 0.25f); }

/* Context: the two bins below, and this bin's level in the previous block
 * against the current step (tones last from block to block). */
static int mag_ctx(const band_models_t *bm, const int32_t *q, int k, int K, float inv_step)
{
    int a = k > 0 ? abs(q[k - 1]) : 0, b = k > 1 ? abs(q[k - 2]) : 0, t;
    float l = bm->last[k] * inv_step;
    if (a > 2)
        a = 2;
    if (b > 2)
        b = 2;
    t = l < 0.5f ? 0 : l < 1.5f ? 1 : l < 4.f ? 2 : 3;
    return ((a * 3 + b) * 4 + t) * 3 + (k * 3) / K;
}

static void remember(band_models_t *bm, const int32_t *q, const int *idx, int bw, int K)
{
    int k;
    for (k = 0; k < K; k++)
        bm->last[k] = fabsf((float)q[k]) * step_of(idx[k / bw]);
}


/* --- block coding ----------------------------------------------------------- */

/* Per band (mono, stereo, RDS): noise power per coefficient never above
 * floor, nor rel below the block's mean power, nor smr below the local
 * signal averaged over ±(sp + spk·bin) bins; a group of bins shares the step
 * that meets the targets of all but its weakest pct of bins (those may fall
 * to zero). Audio is coded for the least squared error: one noise level per
 * block, rel below its power (quiet passages as clean as loud ones), which
 * the rate control then moves; a setting of −100 dB is off. */
typedef struct quality {
    float floor_db[3], rel_db[3], smr_db[3], spk[3], pct[3];
} quality_t;

#ifdef NANOMPX_STATS
#include <stdio.h>
/* bench: tuning overrides from the environment */
static float tune(const char *name, int b, float def)
{
    char key[32];
    const char *v;
    snprintf(key, sizeof(key), "%s%d", name, b);
    v = getenv(key);
    return v ? (float)atof(v) : def;
}
#else
#define tune(name, b, def) (def)
#endif

/* Rate control: the encoder spends its bitrate, easing the noise targets
 * when content is dense and tightening them (up to RATE_TIGHT_MAX dB) when it
 * is light. A leaky bucket: 1 dB per RATE_DB_BITS of debt or credit. */
#ifndef RATE_DB_BITS
#define RATE_DB_BITS 3000.0
#endif
#ifndef RATE_EASE_MAX
#define RATE_EASE_MAX 60.0       /* enough to hold the rate on full-scale noise */
#endif
#define RATE_TIGHT_MAX 6.0

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* Starting targets for a payload bitrate (kbps), where the rate control
 * lands with the least easing: about 1 dB per 10 kbps of audio, and RDS a
 * little coarser at low rates (its floor keeps it decodable) and cleaner at
 * high ones, where it costs little (+10 dB for about 6 kbps). */
static quality_t rate_quality(double kbps)
{
    float rel = clampf((float)(66.0 + 0.1 * (kbps - 316.0)), 30.f, 90.f);
    float smr = clampf((float)(12.0 + (kbps - 316.0) / 24.0), 4.f, 14.f);
    float rds = clampf((float)(-50.0 - (kbps - 200.0) / 24.0), -66.f, -50.f);
    /*             floor           rel                   smr                 spread     pct */
    quality_t q = { { 0, 0, rds }, { rel, rel - 4, -100 }, { -100, -100, smr }, { 0, 0, 0 }, { 0, 0, 0 } };
    int b;
    for (b = 0; b < 3; b++) {
        q.floor_db[b] = tune("NMPX_FLOOR", b, q.floor_db[b]);
        q.rel_db[b] = tune("NMPX_REL", b, q.rel_db[b]);
        q.smr_db[b] = tune("NMPX_SMR", b, q.smr_db[b]);
        q.spk[b] = tune("NMPX_SPK", b, q.spk[b]);
        q.pct[b] = tune("NMPX_PCT", b, q.pct[b]);
    }
    return q;
}

static void put_mag(rc_enc_t *r, band_models_t *bm, int ctx, uint32_t a)
{
    if (a < MAG_ESC) {
        model_put(r, &bm->mag[ctx], (int)a);
        return;
    }
    model_put(r, &bm->mag[ctx], MAG_ESC);
    a -= MAG_ESC;               /* Elias-gamma of a+1 */
    {
        uint32_t v = a + 1;
        int nb = 0;
        while ((v >> nb) > 1)
            nb++;
        rce_bits(r, (uint32_t)nb, 5);
        rce_bits(r, v & ((1u << nb) - 1u), nb);
    }
}

static uint32_t get_mag(rc_dec_t *r, band_models_t *bm, int ctx)
{
    int s = model_get(r, &bm->mag[ctx]);
    uint32_t nb, v;
    if (s < MAG_ESC)
        return (uint32_t)s;
    nb = rcd_bits(r, 5);
    v = (1u << nb) | rcd_bits(r, (int)nb);
    return v - 1 + MAG_ESC;
}

static void put_sd(rc_enc_t *r, model_t *m, int d, int idx)
{
    if (d >= -SD_RANGE && d <= SD_RANGE) {
        model_put(r, m, d + SD_RANGE);
        return;
    }
    model_put(r, m, SD_ESC);
    rce_bits(r, (uint32_t)idx, 7);
}

static int get_sd(rc_dec_t *r, model_t *m, int prev)
{
    int s = model_get(r, m);
    if (s < SD_ESC)
        return prev + s - SD_RANGE;
    return (int)rcd_bits(r, 7);
}

/* Code one MDCT block of band b from X[0..K). */
static void enc_block(rc_enc_t *r, band_models_t *bm, int b, const float *X, quality_t ql)
{
    int K = band_k(b), bw = band_bw(b), nsf = K / bw, i, s, t = b < 2 ? b : 2;
    float smr = powf(10.f, -ql.smr_db[t] / 10.f);
    float fl = powf(10.f, ql.floor_db[t] / 10.f);
    float E[K_A], Es[K_A];
    int idx[NSF_MAX], prev;
    int32_t q[K_A];
    int sp0 = b < 2 ? 2 : 1;
    float spk = ql.spk[t];                  /* extra spread per bin */
    float dz = tune("NMPX_DZ", 0, 0.44f);   /* rounding offset: a slight dead zone */
    float pct = ql.pct[t];                  /* which of the group's targets sets the step */

    {
        float mean = 0.f;
        for (i = 0; i < K; i++) {
            E[i] = X[i] * X[i];
            mean += E[i];
        }
        mean /= (float)K;
        if (mean * powf(10.f, -ql.rel_db[t] / 10.f) < fl)
            fl = mean * powf(10.f, -ql.rel_db[t] / 10.f);
        if (fl < 1e-16f)
            fl = 1e-16f;
    }
    for (i = 0; i < K; i++) {
        int sp = sp0 + (int)(spk * (float)i);
        int a = i - sp < 0 ? 0 : i - sp, z = i + sp >= K ? K - 1 : i + sp, j;
        float acc = 0.f;
        for (j = a; j <= z; j++)
            acc += E[j];
        Es[i] = acc / (float)(z - a + 1);
    }
    for (s = 0; s < nsf; s++) {
        float dl[BW_A], d, st;
        int n = 0, j;
        for (i = s * bw; i < (s + 1) * bw; i++) {
            float di = Es[i] * smr;
            if (di > fl)
                di = fl;
            if (di < fl * 1e-3f)
                di = fl * 1e-3f;   /* empty bins: a sane step, they code as 0 */
            /* insertion sort: the group's targets, lowest first */
            for (j = n; j > 0 && dl[j - 1] > di; j--)
                dl[j] = dl[j - 1];
            dl[j] = di;
            n++;
        }
        /* The step meets the target of all but the weakest bins of the group. */
        d = dl[(int)(pct * (float)(n - 1))];
        st = sqrtf(12.f * d);
        idx[s] = (int)lrintf(-4.f * log2f(st));
        if (idx[s] < 0)
            idx[s] = 0;
        if (idx[s] > 127)
            idx[s] = 127;
    }
    prev = bm->prev_first;
    for (s = 0; s < nsf; s++) {
        put_sd(r, &bm->sd[s ? 1 : 0], idx[s] - prev, idx[s]);
        prev = idx[s];
    }
    bm->prev_first = idx[0];
    for (s = 0; s < nsf; s++) {
        float inv = 1.f / step_of(idx[s]);
        for (i = s * bw; i < (s + 1) * bw; i++) {
            float a = fabsf(X[i]) * inv;
            int32_t v = (int32_t)(a + dz);
            q[i] = X[i] < 0.f ? -v : v;
        }
    }
    for (i = 0; i < K; i++) {
        uint32_t a = (uint32_t)abs(q[i]);
        put_mag(r, bm, mag_ctx(bm, q, i, K, 1.f / step_of(idx[i / bw])), a);
        if (a)
            rce_bits(r, q[i] < 0, 1);
    }
    remember(bm, q, idx, bw, K);
}

static int dec_block(rc_dec_t *r, band_models_t *bm, int b, float *X)
{
    int K = band_k(b), H = band_hop(b), bw = band_bw(b), nsf = K / bw, i, s, prev;
    int idx[NSF_MAX];
    int32_t q[K_A];

    prev = bm->prev_first;
    for (s = 0; s < nsf; s++) {
        idx[s] = get_sd(r, &bm->sd[s ? 1 : 0], prev);
        if (idx[s] < 0 || idx[s] > 127)
            return -1;
        prev = idx[s];
    }
    bm->prev_first = idx[0];
    for (i = 0; i < K; i++) {
        uint32_t a = get_mag(r, bm, mag_ctx(bm, q, i, K, 1.f / step_of(idx[i / bw])));
        if (a > (1u << 24))
            return -1;
        q[i] = (int32_t)a;
        if (a && rcd_bits(r, 1))
            q[i] = -q[i];
    }
    remember(bm, q, idx, bw, K);
    for (s = 0; s < nsf; s++) {
        float st = step_of(idx[s]);
        for (i = s * bw; i < (s + 1) * bw; i++)
            X[i] = (float)q[i] * st;
    }
    for (i = K; i < H; i++)
        X[i] = 0.f;
    return r->err ? -1 : 0;
}

/* --- carriers ------------------------------------------------------------- */

/* sin θ, sin 2θ, cos 3θ, sin 3θ for θ = phi + w·j, j = 0..n-1 (s1 may be NULL). */
static void carriers(double phi, double w, size_t n, float *s1, float *s2, float *c3, float *s3)
{
    double s = sin(phi), c = cos(phi), ds = sin(w), dc = cos(w);
    size_t j;
    for (j = 0; j < n; j++) {
        double t;
        if (s1)
            s1[j] = (float)s;
        s2[j] = (float)(2.0 * s * c);
        c3[j] = (float)(c * (4.0 * c * c - 3.0));
        s3[j] = (float)(s * (3.0 - 4.0 * s * s));
        t = s * dc + c * ds;
        c = c * dc - s * ds;
        s = t;
        if ((j & 255) == 255) {      /* keep the rotator on the unit circle */
            double g = 1.5 - 0.5 * (s * s + c * c);
            s *= g;
            c *= g;
        }
    }
}

static double wrap_2pi(double p)
{
    p = fmod(p, 2.0 * M_PI);
    return p < 0.0 ? p + 2.0 * M_PI : p;
}

/* Pilot parameters as sent. */
static int64_t wq_nominal(void) { return llround(PILOT_HZ / (double)NANOMPX_SAMPLE_RATE * WQ_ONE); }
static double ph_rad(uint32_t ph) { return (double)ph / PH_ONE * 2.0 * M_PI; }
static double wq_rad(int64_t wq) { return (double)wq / WQ_ONE * 2.0 * M_PI; }
static uint32_t ph_of(double rad) { return (uint32_t)llround(wrap_2pi(rad) / (2.0 * M_PI) * PH_ONE) & 0xffffffu; }

/* θ at the start of the frame after one of len samples. */
static uint32_t ph_next(uint32_t ph, int64_t wq, size_t len)
{
    return (uint32_t)(ph + (uint32_t)((wq * (int64_t)len + 32768) >> 16)) & 0xffffffu;
}

static uint32_t amp_q(double a)
{
    long v = lrint(a / AMP_LSB);
    return (uint32_t)(v < 0 ? 0 : v > 65535 ? 65535 : v);
}

/* Signed exp-Golomb in bypass bits. */
static void put_eg(rc_enc_t *r, int64_t v)
{
    uint64_t u = v < 0 ? ((uint64_t)(-(v + 1)) << 1) | 1u : (uint64_t)v << 1;
    int nb = 0;
    u++;
    while ((u >> nb) > 1)
        nb++;
    rce_bits(r, (uint32_t)nb, 6);
    if (nb > 32)
        rce_bits(r, (uint32_t)(u >> 32), nb - 32);
    rce_bits(r, (uint32_t)u, nb > 32 ? 32 : nb);
}

static int64_t get_eg(rc_dec_t *r)
{
    int nb = (int)rcd_bits(r, 6);
    uint64_t u = 1;
    if (nb > 40)
        return 0;
    if (nb > 32)
        u = (u << (nb - 32)) | rcd_bits(r, nb - 32);
    u = (u << (nb > 32 ? 32 : nb)) | rcd_bits(r, nb > 32 ? 32 : nb);
    u--;
    return (u & 1u) ? -(int64_t)(u >> 1) - 1 : (int64_t)(u >> 1);
}

/* The pilot fields: full on keyframes (and after a gap), else as deltas. */
typedef struct pilot_tx {
    int have;
    uint64_t next_base, base_prev;
    uint32_t ph, amp;
    int64_t wq;
} pilot_tx_t;

static void pilot_put(rc_enc_t *r, pilot_tx_t *t, int full, uint64_t base, size_t count,
                      uint32_t ph, int64_t wq, uint32_t amp)
{
    full = full || !t->have || base != t->next_base;
    rce_bits(r, (uint32_t)full, 1);
    if (full) {
        rce_bits(r, ph, 24);
        put_eg(r, wq - wq_nominal());
        rce_bits(r, amp, 16);
    } else {
        int32_t dph = (int32_t)((ph - ph_next(t->ph, t->wq, (size_t)(base - t->base_prev))) << 8) >> 8;
        put_eg(r, dph);
        put_eg(r, wq - t->wq);
        put_eg(r, (int64_t)amp - (int64_t)t->amp);
    }
    t->have = 1;
    t->ph = ph;
    t->wq = wq;
    t->amp = amp;
    t->base_prev = base;
    t->next_base = base + count;
}

static int pilot_get(rc_dec_t *r, pilot_tx_t *t, uint64_t base, size_t count,
                     uint32_t *ph, int64_t *wq, uint32_t *amp)
{
    int full = (int)rcd_bits(r, 1);
    if (full) {
        *ph = rcd_bits(r, 24);
        *wq = wq_nominal() + get_eg(r);
        *amp = rcd_bits(r, 16);
    } else {
        if (!t->have || base != t->next_base)
            return -1;
        *ph = (ph_next(t->ph, t->wq, (size_t)(base - t->base_prev)) + (uint32_t)get_eg(r)) & 0xffffffu;
        *wq = t->wq + get_eg(r);
        *amp = (uint32_t)((int64_t)t->amp + get_eg(r));
    }
    t->have = 1;
    t->ph = *ph;
    t->wq = *wq;
    t->amp = *amp;
    t->base_prev = base;
    t->next_base = base + count;
    return r->err ? -1 : 0;
}

/* --- encoder -------------------------------------------------------------- */

struct nanompx_comp_enc {
    unsigned kbps;              /* whole packets, the caller's header included */
    unsigned overhead;          /* caller's header bytes per packet */
    quality_t q;                /* starting targets for the payload bitrate */
    size_t q_count;             /* the packet length q was set for */
    float *in[NBANDS];          /* band inputs: NF-1 samples of history, then the frame */
    size_t cap;
    float *s2, *c3, *s3, *x;
    float *ring[NBANDS];        /* decimated band samples */
    band_models_t bm[NBANDS];
    int models_ok;
    /* pilot PLL: θ of the next frame's first sample and the frequency */
    int pll_on;
    uint64_t next_base;
    uint32_t ph;
    int64_t wq;
    pilot_tx_t tx;
    /* rate cap: bits over budget, and how far the quality is eased (dB) */
    double debt;
    float ease;
};

static int ensure_enc(nanompx_comp_enc_t *e, size_t n)
{
    int b;
    if (e->cap >= n)
        return 0;
    for (b = 0; b < NBANDS; b++) {
        float *p = (float *)realloc(e->in[b], sizeof(float) * (NF - 1 + n));
        if (!p)
            return -1;
        if (!e->cap)
            memset(p, 0, sizeof(float) * (NF - 1));
        e->in[b] = p;
    }
    free(e->s2);
    e->s2 = (float *)malloc(sizeof(float) * n * 4);
    if (!e->s2) {
        e->cap = 0;
        return -1;
    }
    e->c3 = e->s2 + n;
    e->s3 = e->c3 + n;
    e->x = e->s3 + n;
    e->cap = n;
    return 0;
}

nanompx_comp_enc_t *nanompx_comp_enc_create(unsigned kbps, unsigned overhead)
{
    nanompx_comp_enc_t *e = (nanompx_comp_enc_t *)calloc(1, sizeof(*e));
    int b;
    if (!e)
        return NULL;
    tables_init();
    for (b = 0; b < NBANDS; b++) {
        e->ring[b] = (float *)calloc(RING, sizeof(float));
        if (!e->ring[b]) {
            nanompx_comp_enc_destroy(e);
            return NULL;
        }
    }
    nanompx_comp_enc_set_bitrate(e, kbps, overhead);
    return e;
}

void nanompx_comp_enc_destroy(nanompx_comp_enc_t *e)
{
    int b;
    if (!e)
        return;
    for (b = 0; b < NBANDS; b++) {
        free(e->in[b]);
        free(e->ring[b]);
    }
    free(e->s2);
    free(e);
}

int nanompx_comp_enc_set_bitrate(nanompx_comp_enc_t *e, unsigned kbps, unsigned overhead)
{
    if (!e || kbps < NANOMPX_KBPS_MIN || kbps > NANOMPX_KBPS_MAX)
        return NANOMPX_ERR_INVALID;
    e->kbps = (unsigned)tune("NMPX_KBPS", 0, (float)kbps);
    e->overhead = overhead;
    e->q_count = 0;
    e->debt = 0.0;
    e->ease = 0.f;
    return NANOMPX_OK;
}

unsigned nanompx_comp_codec_delay_samples(void)
{
    return (unsigned)CODEC_DELAY;
}

/* Pilot in x[0..n) against θ = phi + w·j: amplitude, and phase error ε for a
 * pilot A·sin(θ + ε). Hann-weighted, which keeps the mono and stereo
 * sidebands out of it. */
static void pilot_measure(const float *x, size_t n, double phi, double w, double *amp,
                          double *err)
{
    double s = sin(phi), c = cos(phi), ds = sin(w), dc = cos(w), zr = 0.0, zi = 0.0, t;
    double ws = 0.0;
    size_t j;
    for (j = 0; j < n; j++) {
        double hw = 0.5 - 0.5 * cos(2.0 * M_PI * ((double)j + 0.5) / (double)n);
        zr += hw * x[j] * c;    /* x · e^{-jθ} */
        zi -= hw * x[j] * s;
        ws += hw;
        t = s * dc + c * ds;
        c = c * dc - s * ds;
        s = t;
    }
    zr /= ws;
    zi /= ws;
    /* A·sin(θ+ε)·e^{-jθ} averages to (A/2)·e^{j(ε−π/2)} */
    *amp = 2.0 * sqrt(zr * zr + zi * zi);
    *err = atan2(zi, zr) + M_PI / 2.0;
    if (*err > M_PI)
        *err -= 2.0 * M_PI;
}

int nanompx_comp_encode(nanompx_comp_enc_t *e,
                        const int32_t *samples, size_t count,
                        uint8_t *dst, size_t dst_cap, size_t *out_len,
                        int keyframe, uint64_t sample_base)
{
    quality_t ql;
    float ease = e ? e->ease : 0.f;
    rc_enc_t rc;
    double phi, wv, amp, err;
    size_t i;
    int b;

    if (!e || !samples || !dst || !out_len || count == 0 || count > 65535)
        return NANOMPX_ERR_INVALID;
    if (e->q_count != count) {
        /* the payload's share: the caller's header is spent per packet */
        e->q = rate_quality(e->kbps - e->overhead * 8.0 * NANOMPX_SAMPLE_RATE / (double)count / 1000.0);
        e->q_count = count;
    }
    ql = e->q;
    if (ensure_enc(e, count) != 0)
        return NANOMPX_ERR_NOMEM;
    if (dst_cap < HDR_BYTES + 32)
        return NANOMPX_ERR_OVERFLOW;

    for (i = 0; i < count; i++)
        e->x[i] = (float)samples[i] / 8388608.f;

    /* Pilot: continue the loop, or acquire (start, or a gap in the input). */
    if (!e->pll_on || sample_base != e->next_base) {
        pilot_measure(e->x, count, 0.0, W19, &amp, &err);
        e->ph = ph_of(err);
        e->wq = wq_nominal();
        e->pll_on = 1;
    }
    phi = ph_rad(e->ph);
    wv = wq_rad(e->wq);
    pilot_measure(e->x, count, phi, wv, &amp, &err);

    dst[0] = COMP_MAGIC;
    dst[1] = (uint8_t)(e->kbps / 8);
    dst[2] = (uint8_t)(count & 0xff);
    dst[3] = (uint8_t)((count >> 8) & 0xff);
    rce_init(&rc, dst + HDR_BYTES, dst_cap - HDR_BYTES);
    pilot_put(&rc, &e->tx, keyframe, sample_base, count, e->ph, e->wq, amp_q(amp));
    {
        /* The next frame's θ continues this one's; corrections are small steps. */
        uint32_t next = ph_next(e->ph, e->wq, count);
        if (amp >= PILOT_MIN) {
            next = (next + (uint32_t)llround(PLL_KP * err / (2.0 * M_PI) * PH_ONE)) & 0xffffffu;
            e->wq += llround(PLL_KI * err / (double)count / (2.0 * M_PI) * WQ_ONE);
        }
        e->ph = next;
        e->next_base = sample_base + count;
    }

    /* Band inputs at 192 kHz, after their NF-1 samples of history. */
    carriers(phi, wv, count, NULL, e->s2, e->c3, e->s3);
    for (i = 0; i < count; i++) {
        float x = e->x[i];
        e->in[0][NF - 1 + i] = x;
        e->in[1][NF - 1 + i] = 2.f * x * e->s2[i];
        e->in[2][NF - 1 + i] = 2.f * x * e->c3[i];
        e->in[3][NF - 1 + i] = -2.f * x * e->s3[i];
    }

    if (keyframe || !e->models_ok) {
        for (b = 0; b < NBANDS; b++)
            models_init(&e->bm[b]);
        e->models_ok = 1;
    }
    /* Over the bitrate: noise allowed higher and closer to the signal;
     * under it: lower and further. */
    for (b = 0; b < 2; b++) {
        ql.floor_db[b] += ease;
        ql.rel_db[b] -= ease;
        ql.smr_db[b] -= 0.3f * ease;
    }

    for (b = 0; b < NBANDS; b++) {
        const float *h = band_h(b);
        const mdct_plan_t *p = band_plan(b);
        int dec = band_dec(b), H = band_hop(b);
        int64_t m0 = mcount((int64_t)sample_base, dec);
        int64_t m1 = mcount((int64_t)(sample_base + count), dec), m, j;
        float *ring = e->ring[b];

        /* decimated samples m·dec of this frame: y(n) = Σ h[k]·x(n−k) */
        for (m = m0; m < m1; m++) {
            const float *xp = e->in[b] + (size_t)(m * dec - (int64_t)sample_base);
            float acc = 0.f;
            int k;
            for (k = 0; k < NF; k++)
                acc += h[k] * xp[k];
            ring[(uint64_t)m & (RING - 1)] = acc;
        }
        /* MDCT blocks that are complete now */
        for (j = blocks_upto((int64_t)sample_base, b); j < blocks_upto((int64_t)(sample_base + count), b); j++) {
            float xb[2 * H_MAX], X[H_MAX];
            int k;
#ifdef NANOMPX_STATS
            size_t before = rc.pos;
#endif
            for (k = 0; k < 2 * H; k++)
                xb[k] = ring[(uint64_t)(j * H + k) & (RING - 1)];
            mdct_fwd(p, xb, X);
            enc_block(&rc, &e->bm[b], b, X, ql);
#ifdef NANOMPX_STATS
            nanompx_stat_bytes[b] += rc.pos - before;
#endif
        }
        memmove(e->in[b], e->in[b] + count, sizeof(float) * (NF - 1));
    }
    rce_finish(&rc);
    if (rc.err)
        return NANOMPX_ERR_OVERFLOW;
    *out_len = HDR_BYTES + rc.pos;
    {
        /* Rate: a leaky bucket of debt (over budget) or credit (under). */
        double budget = e->kbps * 1000.0 * (double)count / (double)NANOMPX_SAMPLE_RATE;
        e->debt += 8.0 * (double)(*out_len + e->overhead) - budget;
        if (e->debt < -RATE_TIGHT_MAX * RATE_DB_BITS)
            e->debt = -RATE_TIGHT_MAX * RATE_DB_BITS;
        if (e->debt > RATE_EASE_MAX * RATE_DB_BITS)
            e->debt = RATE_EASE_MAX * RATE_DB_BITS;
        e->ease = (float)(e->debt / RATE_DB_BITS);
    }
    (void)ease;
    return NANOMPX_OK;
}

/* --- decoder -------------------------------------------------------------- */

typedef struct pframe {
    uint64_t base;
    size_t len;
    double phi, w, a0, a1;
} pframe_t;

struct nanompx_comp_dec {
    float *ring[NBANDS];        /* decoded band samples by index & (RING-1) */
    band_models_t bm[NBANDS];
    int models_ok;
    pilot_tx_t tx;
    pframe_t pf[PRING];         /* pilot parameters of the last frames */
    unsigned npf, pf_head;
    float *s1, *s2, *c3, *s3, *pa;
    size_t ccap;
};

nanompx_comp_dec_t *nanompx_comp_dec_create(void)
{
    nanompx_comp_dec_t *d = (nanompx_comp_dec_t *)calloc(1, sizeof(*d));
    int b;
    if (!d)
        return NULL;
    tables_init();
    for (b = 0; b < NBANDS; b++) {
        d->ring[b] = (float *)calloc(RING, sizeof(float));
        if (!d->ring[b]) {
            nanompx_comp_dec_destroy(d);
            return NULL;
        }
    }
    return d;
}

void nanompx_comp_dec_destroy(nanompx_comp_dec_t *d)
{
    int b;
    if (!d)
        return;
    for (b = 0; b < NBANDS; b++)
        free(d->ring[b]);
    free(d->s1);
    free(d);
}

void nanompx_comp_dec_reset(nanompx_comp_dec_t *d)
{
    int b;
    if (!d)
        return;
    for (b = 0; b < NBANDS; b++)
        if (d->ring[b])
            memset(d->ring[b], 0, RING * sizeof(float));
    d->models_ok = 0;
    d->tx.have = 0;
    d->npf = 0;
    d->pf_head = 0;
}

static const pframe_t *pf_get(const nanompx_comp_dec_t *d, unsigned age)
{
    return &d->pf[(d->pf_head + PRING - 1 - age) % PRING];
}

/* Pilot parameters for sample t: the newest frame starting at or before t
 * (the oldest kept, extrapolated back, for earlier t); *until is where the
 * next kept frame starts. */
static const pframe_t *pf_at(const nanompx_comp_dec_t *d, int64_t t, int64_t *until)
{
    const pframe_t *p = NULL;
    unsigned i;
    *until = INT64_MAX;
    for (i = 0; i < d->npf; i++) {
        p = pf_get(d, i);
        if ((int64_t)p->base <= t)
            break;
        *until = (int64_t)p->base;
    }
    return p;
}

static int ensure_dec(nanompx_comp_dec_t *d, size_t count)
{
    if (d->ccap < count) {
        free(d->s1);
        d->s1 = (float *)malloc(sizeof(float) * count * 5);
        if (!d->s1) {
            d->ccap = 0;
            return -1;
        }
        d->s2 = d->s1 + count;
        d->c3 = d->s2 + count;
        d->s3 = d->c3 + count;
        d->pa = d->s3 + count;
        d->ccap = count;
    }
    return 0;
}

/* Band b at 192 kHz sample n: its decimated samples, zero-stuffed, through h. */
static float synth(const nanompx_comp_dec_t *d, int b, int64_t n)
{
    int dec = band_dec(b);
    const float *h = band_h(b), *v = d->ring[b];
    int64_t m = fdiv(n, dec);
    int k = (int)(n - m * dec);
    float acc = 0.f;
    for (; k < NF; k += dec, m--)
        acc += h[k] * v[(uint64_t)m & (RING - 1)];
    return acc * (float)dec;
}

int nanompx_comp_decode(nanompx_comp_dec_t *d,
                        nanompx_profile_t profile,
                        const uint8_t *src, size_t src_len,
                        int32_t *samples, size_t max_samples, size_t *out_count,
                        int keyframe, uint64_t sample_base)
{
    size_t count, i;
    rc_dec_t rc;
    pframe_t *pf;
    int b;

    (void)profile;
    if (!d || !src || !samples || !out_count || src_len < HDR_BYTES)
        return NANOMPX_ERR_INVALID;
    if (src[0] != COMP_MAGIC)
        return NANOMPX_ERR_INVALID;
    count = (size_t)src[2] | ((size_t)src[3] << 8);
    if (count == 0 || count > max_samples)
        return NANOMPX_ERR_INVALID;
    if (ensure_dec(d, count) != 0)
        return NANOMPX_ERR_NOMEM;
    if (keyframe || !d->models_ok) {
        for (b = 0; b < NBANDS; b++)
            models_init(&d->bm[b]);
        d->models_ok = 1;
    }

    rcd_init(&rc, src + HDR_BYTES, src_len - HDR_BYTES);
    pf = &d->pf[d->pf_head];
    pf->base = sample_base;
    pf->len = count;
    {
        uint32_t ph, amp;
        int64_t wq;
        if (pilot_get(&rc, &d->tx, sample_base, count, &ph, &wq, &amp))
            return NANOMPX_ERR_INVALID;
        pf->phi = ph_rad(ph);
        pf->w = wq_rad(wq);
        pf->a1 = (double)amp * AMP_LSB;
    }
    pf->a0 = pf->a1;
    if (d->npf) {
        const pframe_t *pp = pf_get(d, 0);
        if (pp->base + pp->len == sample_base)
            pf->a0 = pp->a1;     /* ramp from where the last frame ended */
    }
    if (rc.err)
        return NANOMPX_ERR_INVALID;

    for (b = 0; b < NBANDS; b++) {
        const mdct_plan_t *p = band_plan(b);
        int H = band_hop(b);
        float *ring = d->ring[b];
        int64_t j;

        for (j = blocks_upto((int64_t)sample_base, b); j < blocks_upto((int64_t)(sample_base + count), b); j++) {
            float X[H_MAX], y[2 * H_MAX];
            int k;
            if (dec_block(&rc, &d->bm[b], b, X))
                return NANOMPX_ERR_INVALID;
            mdct_inv(p, X, y);
            /* block j covers [jH, jH+2H): the second half is new */
            for (k = H; k < 2 * H; k++)
                ring[(uint64_t)(j * H + k) & (RING - 1)] = 0.f;
            for (k = 0; k < 2 * H; k++)
                ring[(uint64_t)(j * H + k) & (RING - 1)] += y[k];
        }
    }
    d->pf_head = (d->pf_head + 1) % PRING;
    if (d->npf < PRING)
        d->npf++;

    /* Carriers and pilot at t = n − delay, a segment per frame of parameters. */
    for (i = 0; i < count;) {
        int64_t t = (int64_t)(sample_base + i) - CODEC_DELAY;
        int64_t end = (int64_t)(sample_base + count) - CODEC_DELAY, until;
        const pframe_t *p = pf_at(d, t, &until);
        double off = (double)(t - (int64_t)p->base);
        size_t len, j;

        if (until < end)
            end = until;
        len = (size_t)(end - t);
        carriers(p->phi + p->w * off, p->w, len, d->s1 + i, d->s2 + i, d->c3 + i, d->s3 + i);
        for (j = 0; j < len; j++) {
            double f = (off + (double)j) / (double)p->len;
            if (f < 0.0)
                f = 0.0;
            if (f > 1.0)
                f = 1.0;
            d->pa[i + j] = (float)(p->a0 + (p->a1 - p->a0) * f);
        }
        i += len;
    }

    for (i = 0; i < count; i++) {
        int64_t n = (int64_t)(sample_base + i) - D_EXTRA;
        float y = synth(d, 0, n) + synth(d, 1, n) * d->s2[i] + synth(d, 2, n) * d->c3[i] -
                  synth(d, 3, n) * d->s3[i] + d->pa[i] * d->s1[i];
        if (y > 1.f)
            y = 1.f;
        if (y < -1.f)
            y = -1.f;
        samples[i] = (int32_t)lrintf(y * 8388607.f);
    }
    *out_count = count;
    return NANOMPX_OK;
}
