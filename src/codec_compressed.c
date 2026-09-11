/* SPDX-License-Identifier: MIT */
/**
 * FM-aware NanoMPX compressed codec (v4 wire: magic 0xC4).
 *
 * - Continuous parametric 19 kHz pilot (protected; 2×/3× carriers)
 * - Band coding: mono (0–15 kHz), stereo L−R (≈23–53 kHz), RDS (≈57 kHz)
 * - Decimated predictive block-float + adaptive Rice entropy on each band
 * - Soft peak limit (no invented overshoots)
 *
 * Quality intent is band-weighted (mono/stereo/pilot), not flat MPX SNR.
 */

#include "internal.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if !defined(log2f)
static float nanompx_log2f(float x) { return logf(x) / logf(2.f); }
#define log2f nanompx_log2f
#endif
#if !defined(exp2f)
static float nanompx_exp2f(float x) { return expf(x * logf(2.f)); }
#define exp2f nanompx_exp2f
#endif

#define COMP_MAGIC 0xC4
#define PILOT_HZ 19000.0
#define BLOCK 32
#define NS_COEF 0.65f
#define SCALE_HEADROOM 1.12f
#define FIR_N 65
#define CODEC_DELAY (FIR_N - 1) /* analysis + synthesis group delay */
#define DEC_AUDIO 4             /* 192 kHz → 48 kHz */
#define DEC_RDS 32              /* 192 kHz → 6 kHz */
#define RICE_MAX_UNARY 512

/* ~15 kHz LP @ 192 kHz (Hamming sinc), DC gain 1 */
static const float fir_lp15[FIR_N] = {
    4.86099489e-19f, 3.96990738e-04f, 7.81865353e-04f, 1.08760938e-03f, 1.20508496e-03f,
    1.00213809e-03f, 3.75330061e-04f, -6.79962711e-04f, -2.00896590e-03f, -3.27888746e-03f,
    -4.02663909e-03f, -3.77732726e-03f, -2.21143576e-03f, 6.65855831e-04f, 4.41298689e-03f,
    8.15292182e-03f, 1.07171196e-02f, 1.09235540e-02f, 7.93556001e-03f, 1.61257727e-03f,
    -7.25097802e-03f, -1.68889439e-02f, -2.47772121e-02f, -2.80851330e-02f, -2.42856940e-02f,
    -1.17933474e-02f, 9.52453673e-03f, 3.81012140e-02f, 7.07750248e-02f, 1.03251922e-01f,
    1.30847368e-01f, 1.49357774e-01f, 1.55874186e-01f, 1.49357774e-01f, 1.30847368e-01f,
    1.03251922e-01f, 7.07750248e-02f, 3.81012140e-02f, 9.52453673e-03f, -1.17933474e-02f,
    -2.42856940e-02f, -2.80851330e-02f, -2.47772121e-02f, -1.68889439e-02f, -7.25097802e-03f,
    1.61257727e-03f, 7.93556001e-03f, 1.09235540e-02f, 1.07171196e-02f, 8.15292182e-03f,
    4.41298689e-03f, 6.65855831e-04f, -2.21143576e-03f, -3.77732726e-03f, -4.02663909e-03f,
    -3.27888746e-03f, -2.00896590e-03f, -6.79962711e-04f, 3.75330061e-04f, 1.00213809e-03f,
    1.20508496e-03f, 1.08760938e-03f, 7.81865353e-04f, 3.96990738e-04f, 4.86099489e-19f,
};

/* ~2.5 kHz LP @ 192 kHz for RDS baseband */
static const float fir_lp25[FIR_N] = {
    5.26328633e-04f, 6.35514090e-04f, 7.91018877e-04f, 1.00779741e-03f, 1.30038011e-03f,
    1.68243947e-03f, 2.16636389e-03f, 2.76285104e-03f, 3.48053169e-03f, 4.32563504e-03f,
    5.30170489e-03f, 6.40937563e-03f, 7.64621485e-03f, 9.00663834e-03f, 1.04819010e-02f,
    1.20601657e-02f, 1.37266493e-02f, 1.54638447e-02f, 1.72518133e-02f, 1.90685430e-02f,
    2.08903633e-02f, 2.26924083e-02f, 2.44491164e-02f, 2.61347562e-02f, 2.77239642e-02f,
    2.91922833e-02f, 3.05166876e-02f, 3.16760809e-02f, 3.26517577e-02f, 3.34278140e-02f,
    3.39914984e-02f, 3.43334946e-02f, 3.44481284e-02f, 3.43334946e-02f, 3.39914984e-02f,
    3.34278140e-02f, 3.26517577e-02f, 3.16760809e-02f, 3.05166876e-02f, 2.91922833e-02f,
    2.77239642e-02f, 2.61347562e-02f, 2.44491164e-02f, 2.26924083e-02f, 2.08903633e-02f,
    1.90685430e-02f, 1.72518133e-02f, 1.54638447e-02f, 1.37266493e-02f, 1.20601657e-02f,
    1.04819010e-02f, 9.00663834e-03f, 7.64621485e-03f, 6.40937563e-03f, 5.30170489e-03f,
    4.32563504e-03f, 3.48053169e-03f, 2.76285104e-03f, 2.16636389e-03f, 1.68243947e-03f,
    1.30038011e-03f, 1.00779741e-03f, 7.91018877e-04f, 6.35514090e-04f, 5.26328633e-04f,
};

typedef struct profile_cfg {
    int bits_mono;
    int bits_stereo;
    int bits_rds;
    int rice_mono;
    int rice_stereo;
    int rice_rds;
} profile_cfg_t;

static profile_cfg_t profile_cfg(nanompx_profile_t p)
{
    profile_cfg_t c;
    switch (p) {
    case NANOMPX_PROFILE_L:
        /* Nominal rice_k; encoder may adapt per block */
        c.bits_mono = 10;
        c.bits_stereo = 8;
        c.bits_rds = 5;
        c.rice_mono = 6;
        c.rice_stereo = 5;
        c.rice_rds = 3;
        break;
    case NANOMPX_PROFILE_M:
        c.bits_mono = 7;
        c.bits_stereo = 5;
        c.bits_rds = 3;
        c.rice_mono = 4;
        c.rice_stereo = 3;
        c.rice_rds = 2;
        break;
    case NANOMPX_PROFILE_S:
    default:
        c.bits_mono = 5;
        c.bits_stereo = 4;
        c.bits_rds = 2;
        c.rice_mono = 3;
        c.rice_stereo = 3;
        c.rice_rds = 2;
        break;
    }
    return c;
}

static int32_t float_to_pcm24(float x)
{
    if (x > 1.f)
        x = 1.f;
    if (x < -1.f)
        x = -1.f;
    return (int32_t)lrintf(x * 8388607.f);
}

static float pcm24_to_float(int32_t s) { return (float)s / 8388608.f; }

typedef struct bitw {
    uint8_t *buf;
    size_t cap;
    size_t byte_pos;
    int bit_pos;
} bitw_t;

static void bitw_init(bitw_t *w, uint8_t *buf, size_t cap)
{
    w->buf = buf;
    w->cap = cap;
    w->byte_pos = 0;
    w->bit_pos = 0;
    if (cap)
        memset(buf, 0, cap);
}

static int bitw_put(bitw_t *w, uint32_t val, int bits)
{
    int i;
    for (i = 0; i < bits; i++) {
        if (w->byte_pos >= w->cap)
            return -1;
        if (val & (1u << i))
            w->buf[w->byte_pos] |= (uint8_t)(1u << w->bit_pos);
        w->bit_pos++;
        if (w->bit_pos == 8) {
            w->bit_pos = 0;
            w->byte_pos++;
        }
    }
    return 0;
}

static size_t bitw_bytes(const bitw_t *w)
{
    return w->byte_pos + (w->bit_pos ? 1 : 0);
}

typedef struct bitr {
    const uint8_t *buf;
    size_t len;
    size_t byte_pos;
    int bit_pos;
} bitr_t;

static void bitr_init(bitr_t *r, const uint8_t *buf, size_t len)
{
    r->buf = buf;
    r->len = len;
    r->byte_pos = 0;
    r->bit_pos = 0;
}

static int bitr_get_bits(bitr_t *r, int bits, uint32_t *out)
{
    uint32_t val = 0;
    int i;
    for (i = 0; i < bits; i++) {
        if (r->byte_pos >= r->len)
            return -1;
        if (r->buf[r->byte_pos] & (1u << r->bit_pos))
            val |= (1u << i);
        r->bit_pos++;
        if (r->bit_pos == 8) {
            r->bit_pos = 0;
            r->byte_pos++;
        }
    }
    *out = val;
    return 0;
}

static int bitw_put_f32(bitw_t *w, float f)
{
    union {
        float f;
        uint32_t u;
    } u;
    u.f = f;
    return bitw_put(w, u.u & 0xff, 8) || bitw_put(w, (u.u >> 8) & 0xff, 8) ||
           bitw_put(w, (u.u >> 16) & 0xff, 8) || bitw_put(w, (u.u >> 24) & 0xff, 8);
}

static int bitr_get_f32(bitr_t *r, float *out)
{
    uint32_t raw = 0, b;
    union {
        float f;
        uint32_t u;
    } u;
    if (bitr_get_bits(r, 8, &b))
        return -1;
    raw = b & 0xff;
    if (bitr_get_bits(r, 8, &b))
        return -1;
    raw |= (b & 0xff) << 8;
    if (bitr_get_bits(r, 8, &b))
        return -1;
    raw |= (b & 0xff) << 16;
    if (bitr_get_bits(r, 8, &b))
        return -1;
    raw |= (b & 0xff) << 24;
    u.u = raw;
    *out = u.f;
    return 0;
}

static uint8_t scale_to_u8(float scale)
{
    float c;
    if (scale < 1e-8f)
        scale = 1e-8f;
    c = 140.f + 16.f * log2f(scale);
    if (c < 0.f)
        c = 0.f;
    if (c > 255.f)
        c = 255.f;
    return (uint8_t)lrintf(c);
}

static float u8_to_scale(uint8_t code)
{
    return exp2f(((float)code - 140.f) / 16.f);
}

static uint32_t zig_encode(int32_t v)
{
    if (v < 0)
        return (uint32_t)((((uint32_t)(-v)) << 1) - 1u);
    return (uint32_t)v << 1;
}

static int32_t zig_decode(uint32_t u)
{
    if (u & 1u)
        return -(int32_t)(u >> 1) - 1;
    return (int32_t)(u >> 1);
}

static int rice_put(bitw_t *w, int32_t v, int k)
{
    uint32_t u = zig_encode(v);
    uint32_t q = (k > 0) ? (u >> k) : u;
    uint32_t r = (k > 0) ? (u & ((1u << k) - 1u)) : 0;
    uint32_t i;
    if (q > RICE_MAX_UNARY)
        return -1;
    for (i = 0; i < q; i++) {
        if (bitw_put(w, 1, 1))
            return -1;
    }
    if (bitw_put(w, 0, 1))
        return -1;
    if (k > 0 && bitw_put(w, r, k))
        return -1;
    return 0;
}

static int rice_get(bitr_t *r, int k, int32_t *out)
{
    uint32_t q = 0;
    uint32_t rem = 0;
    uint32_t bit;
    while (1) {
        if (bitr_get_bits(r, 1, &bit))
            return -1;
        if (bit == 0)
            break;
        q++;
        if (q > RICE_MAX_UNARY)
            return -1;
    }
    if (k > 0 && bitr_get_bits(r, k, &rem))
        return -1;
    *out = zig_decode((k > 0) ? ((q << k) | rem) : q);
    return 0;
}

typedef struct fir_state {
    float hist[FIR_N - 1];
} fir_state_t;

static void fir_reset(fir_state_t *s) { memset(s, 0, sizeof(*s)); }

static float fir_step(fir_state_t *s, const float *h, float x)
{
    float acc = h[0] * x;
    int i;
    for (i = 0; i < FIR_N - 1; i++)
        acc += h[i + 1] * s->hist[i];
    for (i = FIR_N - 2; i > 0; i--)
        s->hist[i] = s->hist[i - 1];
    if (FIR_N > 1)
        s->hist[0] = x;
    return acc;
}

/* Continuous-phase decimate: emit when phase==0, advance phase across frames. */
static int choose_rice_k(const int32_t *q, size_t n, int tip)
{
    size_t i;
    double mean = 0.0;
    int k;
    if (n == 0)
        return tip;
    for (i = 0; i < n; i++)
        mean += (double)zig_encode(q[i]);
    mean /= (double)n;
    k = 0;
    while (k < 14 && (double)(1u << (k + 1)) < mean)
        k++;
    /* Prefer tip when close — keeps streams stable */
    if (tip > k && tip - k <= 2)
        k = tip;
    if (k > 14)
        k = 14;
    return k;
}

static int quantize_pred_rice(const float *in, size_t n, int bits, int rice_tip, bitw_t *w,
                              float *pred_state)
{
    size_t off = 0;
    float prev = *pred_state;
    float err = 0.f;
    int32_t qmax = (1 << (bits - 1)) - 1;
    int32_t qbuf[BLOCK];
    if (qmax < 1)
        qmax = 1;

    while (off < n) {
        size_t len = n - off;
        size_t i;
        float peak = 1e-12f;
        float scale;
        float p = prev;
        float prev_blk = prev;
        float err_blk = err;
        int k;
        if (len > BLOCK)
            len = BLOCK;

        for (i = 0; i < len; i++) {
            float a = fabsf(in[off + i] - p);
            if (a > peak)
                peak = a;
            p = in[off + i];
        }
        peak *= SCALE_HEADROOM;
        scale = u8_to_scale(scale_to_u8(peak));

        for (i = 0; i < len; i++) {
            float target = in[off + i] - prev_blk - NS_COEF * err_blk;
            int32_t q = (int32_t)lrintf((target / scale) * (float)qmax);
            float recon;
            if (q > qmax)
                q = qmax;
            if (q < -qmax - 1)
                q = -qmax - 1;
            qbuf[i] = q;
            recon = ((float)q / (float)qmax) * scale;
            err_blk = recon - (in[off + i] - prev_blk);
            prev_blk += recon;
        }
        k = choose_rice_k(qbuf, len, rice_tip);
        if (bitw_put(w, (uint32_t)k, 4) || bitw_put(w, scale_to_u8(peak), 8))
            return -1;
        for (i = 0; i < len; i++) {
            if (rice_put(w, qbuf[i], k))
                return -1;
        }
        prev = prev_blk;
        err = err_blk;
        off += len;
    }
    *pred_state = prev;
    return 0;
}

static int dequantize_pred_rice(float *out, size_t n, int bits, int rice_tip, bitr_t *r,
                                float *pred_state)
{
    size_t off = 0;
    float prev = *pred_state;
    int32_t qmax = (1 << (bits - 1)) - 1;
    (void)rice_tip;
    if (qmax < 1)
        qmax = 1;

    while (off < n) {
        size_t len = n - off;
        size_t i;
        float scale;
        uint32_t sc, kbits;
        int k;
        if (len > BLOCK)
            len = BLOCK;
        if (bitr_get_bits(r, 4, &kbits) || bitr_get_bits(r, 8, &sc))
            return -1;
        k = (int)kbits;
        scale = u8_to_scale((uint8_t)sc);
        for (i = 0; i < len; i++) {
            int32_t q;
            float recon;
            if (rice_get(r, k, &q))
                return -1;
            recon = ((float)q / (float)qmax) * scale;
            prev += recon;
            out[off + i] = prev;
        }
        off += len;
    }
    *pred_state = prev;
    return 0;
}

static void refine_pilot_amp(const float *x, size_t n, uint64_t sample_base, float phase0,
                             float *amp)
{
    double sI = 0.0, sQ = 0.0;
    size_t i;
    double w = 2.0 * M_PI * PILOT_HZ / (double)NANOMPX_SAMPLE_RATE;
    for (i = 0; i < n; i++) {
        double ph = w * (double)(sample_base + i) + (double)phase0;
        sI += (double)x[i] * cos(ph);
        sQ += (double)x[i] * sin(ph);
    }
    sI *= 2.0 / (double)n;
    sQ *= 2.0 / (double)n;
    /* Project onto locked phase (phase0 already defines I/Q rotation) */
    *amp = (float)sqrt(sI * sI + sQ * sQ);
}

static void fit_pilot(const float *x, size_t n, uint64_t sample_base, float *amp, float *phase0)
{
    double sI = 0.0, sQ = 0.0;
    size_t i;
    double w = 2.0 * M_PI * PILOT_HZ / (double)NANOMPX_SAMPLE_RATE;
    for (i = 0; i < n; i++) {
        double t = (double)(sample_base + i);
        sI += (double)x[i] * cos(w * t);
        sQ += (double)x[i] * sin(w * t);
    }
    sI *= 2.0 / (double)n;
    sQ *= 2.0 / (double)n;
    *amp = (float)sqrt(sI * sI + sQ * sQ);
    *phase0 = (float)(atan2(sI, sQ) - w * (double)sample_base);
}

/* Continuous-phase decimate: emit when phase==0, advance phase across frames. */
static void fir_decimate(fir_state_t *fir, const float *h, const float *in, size_t n, int dec,
                         int *phase, float *out, size_t *out_n)
{
    size_t i, o = 0;
    int ph = *phase;
    for (i = 0; i < n; i++) {
        float y = fir_step(fir, h, in[i]);
        if (ph == 0)
            out[o++] = y;
        ph++;
        if (ph == dec)
            ph = 0;
    }
    *phase = ph;
    *out_n = o;
}

/* Continuous-phase upsample + synthesis FIR (gain = dec on nonzero taps). */
static void upsample_fir(fir_state_t *fir, const float *h, const float *in, size_t n_in, int dec,
                         int *phase, float *out, size_t n_out)
{
    size_t i, s = 0;
    int ph = *phase;
    for (i = 0; i < n_out; i++) {
        float inj = 0.f;
        if (ph == 0 && s < n_in) {
            inj = in[s++] * (float)dec;
        }
        out[i] = fir_step(fir, h, inj);
        ph++;
        if (ph == dec)
            ph = 0;
    }
    *phase = ph;
}

struct nanompx_comp_enc {
    nanompx_profile_t profile;
    float *x;
    float *work;
    size_t cap;
    uint64_t sample_phase;
    fir_state_t fir_mono;
    fir_state_t fir_st;
    fir_state_t fir_rds;
    int phase_mono;
    int phase_st;
    int phase_rds;
    float pred_mono;
    float pred_st;
    float pred_rds;
    float pilot_amp;
    float pilot_phase0;
    int pilot_locked;
};

struct nanompx_comp_dec {
    float *work;
    size_t cap;
    uint64_t sample_phase;
    fir_state_t fir_mono_i;
    fir_state_t fir_st_i;
    fir_state_t fir_rds_i;
    int phase_mono;
    int phase_st;
    int phase_rds;
    float pred_mono;
    float pred_st;
    float pred_rds;
};

/* work layout for encoder: res, mix, filt, mono_d, st_d, rds_d, mono_u, st_u, rds_u */
static int ensure_enc(nanompx_comp_enc_t *e, size_t n)
{
    size_t need = n * 9 + n / 2 + 64;
    if (e->cap >= need)
        return 0;
    free(e->x);
    e->x = (float *)malloc(sizeof(float) * (n + need));
    if (!e->x) {
        e->cap = 0;
        return -1;
    }
    e->work = e->x + n;
    e->cap = need;
    return 0;
}

static int ensure_dec(nanompx_comp_dec_t *d, size_t n)
{
    size_t need = n * 6 + n / 2 + 64;
    if (d->cap >= need)
        return 0;
    free(d->work);
    d->work = (float *)malloc(sizeof(float) * need);
    if (!d->work) {
        d->cap = 0;
        return -1;
    }
    d->cap = need;
    return 0;
}

nanompx_comp_enc_t *nanompx_comp_enc_create(nanompx_profile_t profile)
{
    nanompx_comp_enc_t *e = (nanompx_comp_enc_t *)calloc(1, sizeof(*e));
    if (!e)
        return NULL;
    if (profile < NANOMPX_PROFILE_L || profile > NANOMPX_PROFILE_S)
        profile = NANOMPX_PROFILE_L;
    e->profile = profile;
    return e;
}

void nanompx_comp_enc_destroy(nanompx_comp_enc_t *e)
{
    if (!e)
        return;
    free(e->x);
    free(e);
}

int nanompx_comp_enc_set_profile(nanompx_comp_enc_t *e, nanompx_profile_t profile)
{
    if (!e || profile < NANOMPX_PROFILE_L || profile > NANOMPX_PROFILE_S)
        return NANOMPX_ERR_INVALID;
    e->profile = profile;
    return NANOMPX_OK;
}

static void reset_enc_state(nanompx_comp_enc_t *e)
{
    e->sample_phase = 0;
    fir_reset(&e->fir_mono);
    fir_reset(&e->fir_st);
    fir_reset(&e->fir_rds);
    e->phase_mono = e->phase_st = e->phase_rds = 0;
    e->pred_mono = e->pred_st = e->pred_rds = 0.f;
    e->pilot_amp = 0.f;
    e->pilot_phase0 = 0.f;
    e->pilot_locked = 0;
}

static void reset_dec_state(nanompx_comp_dec_t *d)
{
    d->sample_phase = 0;
    fir_reset(&d->fir_mono_i);
    fir_reset(&d->fir_st_i);
    fir_reset(&d->fir_rds_i);
    d->phase_mono = d->phase_st = d->phase_rds = 0;
    d->pred_mono = d->pred_st = d->pred_rds = 0.f;
}

int nanompx_comp_encode(nanompx_comp_enc_t *e,
                        const int32_t *samples, size_t count,
                        uint8_t *dst, size_t dst_cap, size_t *out_len,
                        int keyframe)
{
    size_t i;
    profile_cfg_t cfg;
    bitw_t w;
    float peak = 0.f;
    float pamp, pphase;
    size_t hdr = 8;
    uint64_t base;
    float *res, *mix, *mono_d, *st_d, *rds_d;
    size_t n_mono = 0, n_st = 0, n_rds = 0;
    double w19;

    if (!e || !samples || !dst || !out_len || count == 0 || count > 65535)
        return NANOMPX_ERR_INVALID;
    if (count < (size_t)DEC_RDS)
        return NANOMPX_ERR_INVALID;
    if (ensure_enc(e, count) != 0)
        return NANOMPX_ERR_NOMEM;
    if (dst_cap < hdr + 64)
        return NANOMPX_ERR_OVERFLOW;

    if (keyframe)
        reset_enc_state(e);

    cfg = profile_cfg(e->profile);
    base = e->sample_phase;
    w19 = 2.0 * M_PI * PILOT_HZ / (double)NANOMPX_SAMPLE_RATE;

    res = e->work;
    mix = res + count;
    mono_d = mix + count;
    st_d = mono_d + (count / DEC_AUDIO + 8);
    rds_d = st_d + (count / DEC_AUDIO + 8);

    for (i = 0; i < count; i++) {
        e->x[i] = pcm24_to_float(samples[i]);
        {
            float a = fabsf(e->x[i]);
            if (a > peak)
                peak = a;
        }
    }

    if (!e->pilot_locked) {
        fit_pilot(e->x, count, base, &pamp, &pphase);
        e->pilot_amp = pamp;
        e->pilot_phase0 = pphase;
        e->pilot_locked = 1;
    } else {
        pphase = e->pilot_phase0;
        refine_pilot_amp(e->x, count, base, pphase, &pamp);
        e->pilot_amp = pamp;
    }
    pamp = e->pilot_amp;
    pphase = e->pilot_phase0;
    for (i = 0; i < count; i++) {
        double ph = w19 * (double)(base + i) + (double)pphase;
        float pilot = pamp * (float)sin(ph);
        res[i] = e->x[i] - pilot;
    }

    /* Mono: LP + decimate */
    fir_decimate(&e->fir_mono, fir_lp15, res, count, DEC_AUDIO, &e->phase_mono, mono_d, &n_mono);

    /* Stereo L−R: demod with 2× pilot phase, then LP + decimate */
    for (i = 0; i < count; i++) {
        double ph = w19 * (double)(base + i) + (double)pphase;
        mix[i] = res[i] * 2.f * (float)sin(2.0 * ph);
    }
    fir_decimate(&e->fir_st, fir_lp15, mix, count, DEC_AUDIO, &e->phase_st, st_d, &n_st);

    /* RDS: demod with 3× pilot phase */
    for (i = 0; i < count; i++) {
        double ph = w19 * (double)(base + i) + (double)pphase;
        mix[i] = res[i] * 2.f * (float)sin(3.0 * ph);
    }
    fir_decimate(&e->fir_rds, fir_lp25, mix, count, DEC_RDS, &e->phase_rds, rds_d, &n_rds);

    dst[0] = COMP_MAGIC;
    dst[1] = (uint8_t)e->profile;
    dst[2] = (uint8_t)(count & 0xff);
    dst[3] = (uint8_t)((count >> 8) & 0xff);
    dst[4] = (uint8_t)((cfg.bits_mono << 4) | (cfg.bits_stereo & 0x0f));
    dst[5] = (uint8_t)((cfg.bits_rds << 4) | (cfg.rice_mono & 0x0f));
    {
        uint16_t pk = (uint16_t)lrintf(peak * 65535.f);
        dst[6] = (uint8_t)(pk & 0xff);
        dst[7] = (uint8_t)((pk >> 8) & 0xff);
    }

    bitw_init(&w, dst + hdr, dst_cap - hdr);
    if (bitw_put_f32(&w, pamp) || bitw_put_f32(&w, pphase))
        return NANOMPX_ERR_OVERFLOW;
    if (bitw_put(&w, (uint32_t)cfg.rice_stereo, 4) || bitw_put(&w, (uint32_t)cfg.rice_rds, 4))
        return NANOMPX_ERR_OVERFLOW;
    if (bitw_put(&w, (uint32_t)n_mono, 16) || bitw_put(&w, (uint32_t)n_st, 16) ||
        bitw_put(&w, (uint32_t)n_rds, 16))
        return NANOMPX_ERR_OVERFLOW;

    if (quantize_pred_rice(mono_d, n_mono, cfg.bits_mono, cfg.rice_mono, &w, &e->pred_mono) ||
        quantize_pred_rice(st_d, n_st, cfg.bits_stereo, cfg.rice_stereo, &w, &e->pred_st) ||
        quantize_pred_rice(rds_d, n_rds, cfg.bits_rds, cfg.rice_rds, &w, &e->pred_rds))
        return NANOMPX_ERR_OVERFLOW;

    *out_len = hdr + bitw_bytes(&w);
    e->sample_phase += count;
    return NANOMPX_OK;
}

nanompx_comp_dec_t *nanompx_comp_dec_create(void)
{
    return (nanompx_comp_dec_t *)calloc(1, sizeof(nanompx_comp_dec_t));
}

void nanompx_comp_dec_destroy(nanompx_comp_dec_t *d)
{
    if (!d)
        return;
    free(d->work);
    free(d);
}

int nanompx_comp_decode(nanompx_comp_dec_t *d,
                        nanompx_profile_t profile,
                        const uint8_t *src, size_t src_len,
                        int32_t *samples, size_t max_samples, size_t *out_count,
                        int keyframe)
{
    size_t count;
    bitr_t r;
    float peak, pamp, pphase;
    size_t i;
    int bits_mono, bits_stereo, bits_rds, rice_mono, rice_stereo, rice_rds;
    uint64_t base;
    uint32_t tmp;
    size_t n_mono, n_st, n_rds;
    float *mono_d, *st_d, *rds_d, *mono_u, *st_u, *rds_u;
    double w19;

    (void)profile;
    if (!d || !src || !samples || !out_count || src_len < 8)
        return NANOMPX_ERR_INVALID;
    if (src[0] != COMP_MAGIC)
        return NANOMPX_ERR_INVALID;

    count = (size_t)src[2] | ((size_t)src[3] << 8);
    bits_mono = (src[4] >> 4) & 0x0f;
    bits_stereo = src[4] & 0x0f;
    bits_rds = (src[5] >> 4) & 0x0f;
    rice_mono = src[5] & 0x0f;
    peak = ((float)(src[6] | (src[7] << 8))) / 65535.f;
    if (count == 0 || count > max_samples)
        return NANOMPX_ERR_INVALID;
    if (bits_mono < 2 || bits_mono > 15 || bits_stereo < 2 || bits_stereo > 15 || bits_rds < 1 ||
        bits_rds > 15)
        return NANOMPX_ERR_INVALID;
    if (ensure_dec(d, count) != 0)
        return NANOMPX_ERR_NOMEM;

    if (keyframe)
        reset_dec_state(d);
    base = d->sample_phase;
    w19 = 2.0 * M_PI * PILOT_HZ / (double)NANOMPX_SAMPLE_RATE;

    mono_d = d->work;
    st_d = mono_d + (count / DEC_AUDIO + 8);
    rds_d = st_d + (count / DEC_AUDIO + 8);
    mono_u = rds_d + (count / DEC_RDS + 8);
    st_u = mono_u + count;
    rds_u = st_u + count;

    bitr_init(&r, src + 8, src_len - 8);
    if (bitr_get_f32(&r, &pamp) || bitr_get_f32(&r, &pphase))
        return NANOMPX_ERR_INVALID;
    if (bitr_get_bits(&r, 4, &tmp))
        return NANOMPX_ERR_INVALID;
    rice_stereo = (int)tmp;
    if (bitr_get_bits(&r, 4, &tmp))
        return NANOMPX_ERR_INVALID;
    rice_rds = (int)tmp;
    if (bitr_get_bits(&r, 16, &tmp))
        return NANOMPX_ERR_INVALID;
    n_mono = tmp;
    if (bitr_get_bits(&r, 16, &tmp))
        return NANOMPX_ERR_INVALID;
    n_st = tmp;
    if (bitr_get_bits(&r, 16, &tmp))
        return NANOMPX_ERR_INVALID;
    n_rds = tmp;

    if (n_mono > count / DEC_AUDIO + 8 || n_st > count / DEC_AUDIO + 8 ||
        n_rds > count / DEC_RDS + 8)
        return NANOMPX_ERR_INVALID;

    if (dequantize_pred_rice(mono_d, n_mono, bits_mono, rice_mono, &r, &d->pred_mono) ||
        dequantize_pred_rice(st_d, n_st, bits_stereo, rice_stereo, &r, &d->pred_st) ||
        dequantize_pred_rice(rds_d, n_rds, bits_rds, rice_rds, &r, &d->pred_rds))
        return NANOMPX_ERR_INVALID;

    upsample_fir(&d->fir_mono_i, fir_lp15, mono_d, n_mono, DEC_AUDIO, &d->phase_mono, mono_u,
                 count);
    upsample_fir(&d->fir_st_i, fir_lp15, st_d, n_st, DEC_AUDIO, &d->phase_st, st_u, count);
    upsample_fir(&d->fir_rds_i, fir_lp25, rds_d, n_rds, DEC_RDS, &d->phase_rds, rds_u, count);

    for (i = 0; i < count; i++) {
        /* Bands are delayed by CODEC_DELAY; carriers/pilot must match that time. */
        int64_t ti = (int64_t)base + (int64_t)i - (int64_t)CODEC_DELAY;
        double ph = w19 * (double)ti + (double)pphase;
        float y = mono_u[i] + st_u[i] * (float)sin(2.0 * ph) + rds_u[i] * (float)sin(3.0 * ph) +
                  pamp * (float)sin(ph);
        if (peak > 0.f) {
            if (y > peak)
                y = peak;
            if (y < -peak)
                y = -peak;
        }
        samples[i] = float_to_pcm24(y);
    }
    *out_count = count;
    d->sample_phase += count;
    return NANOMPX_OK;
}
