/* SPDX-License-Identifier: MIT */
/**
 * FM-aware NanoMPX compressed codec (v3 wire: magic 0xC3).
 *
 * - Continuous parametric 19 kHz pilot (protected)
 * - 1st-order predictive + noise-shaped block-float residual @ 192 kHz
 * - Profile bit depths chosen for SNR vs rate (L/M/S)
 * - Soft peak limit (no invented overshoots)
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

typedef struct profile_cfg {
    int bits;
} profile_cfg_t;

static profile_cfg_t profile_cfg(nanompx_profile_t p)
{
    profile_cfg_t c;
    switch (p) {
    case NANOMPX_PROFILE_L:
        c.bits = 10; /* ~1920 kbit/s — SNR priority */
        break;
    case NANOMPX_PROFILE_M:
        c.bits = 6; /* ~1152 kbit/s */
        break;
    case NANOMPX_PROFILE_S:
    default:
        c.bits = 4; /* ~768 kbit/s */
        break;
    }
    return c;
}

#define BLOCK 32
#define COMP_MAGIC 0xC3
#define PILOT_HZ 19000.0
#define NS_COEF 0.70f
#define SCALE_HEADROOM 1.10f

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

static int bitr_get(bitr_t *r, int bits, int32_t *out)
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
    if (bits > 0 && (val & (1u << (bits - 1))))
        val |= ~((1u << bits) - 1u);
    *out = (int32_t)val;
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
    uint32_t raw = 0;
    int32_t b;
    union {
        float f;
        uint32_t u;
    } u;
    if (bitr_get(r, 8, &b))
        return -1;
    raw = (uint8_t)b;
    if (bitr_get(r, 8, &b))
        return -1;
    raw |= (uint32_t)(uint8_t)b << 8;
    if (bitr_get(r, 8, &b))
        return -1;
    raw |= (uint32_t)(uint8_t)b << 16;
    if (bitr_get(r, 8, &b))
        return -1;
    raw |= (uint32_t)(uint8_t)b << 24;
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

static int quantize_pred(const float *in, size_t n, int bits, bitw_t *w)
{
    size_t off = 0;
    float prev = 0.f;
    float err = 0.f;
    int32_t qmax = (1 << (bits - 1)) - 1;
    if (qmax < 1)
        qmax = 1;

    while (off < n) {
        size_t len = n - off;
        size_t i;
        float peak = 1e-12f;
        float scale;
        float p = prev;
        if (len > BLOCK)
            len = BLOCK;

        for (i = 0; i < len; i++) {
            float a = fabsf(in[off + i] - p);
            if (a > peak)
                peak = a;
            p = in[off + i];
        }
        peak *= SCALE_HEADROOM;
        if (bitw_put(w, scale_to_u8(peak), 8))
            return -1;
        scale = u8_to_scale(scale_to_u8(peak));

        for (i = 0; i < len; i++) {
            float target = in[off + i] - prev - NS_COEF * err;
            int32_t q = (int32_t)lrintf((target / scale) * (float)qmax);
            float recon;
            if (q > qmax)
                q = qmax;
            if (q < -qmax - 1)
                q = -qmax - 1;
            if (bitw_put(w, (uint32_t)q, bits))
                return -1;
            recon = ((float)q / (float)qmax) * scale;
            err = recon - (in[off + i] - prev);
            prev += recon;
        }
        off += len;
    }
    return 0;
}

static int dequantize_pred(float *out, size_t n, int bits, bitr_t *r)
{
    size_t off = 0;
    float prev = 0.f;
    int32_t qmax = (1 << (bits - 1)) - 1;
    if (qmax < 1)
        qmax = 1;

    while (off < n) {
        size_t len = n - off;
        size_t i;
        float scale;
        int32_t sc;
        if (len > BLOCK)
            len = BLOCK;
        if (bitr_get(r, 8, &sc))
            return -1;
        scale = u8_to_scale((uint8_t)sc);
        for (i = 0; i < len; i++) {
            int32_t q;
            float recon;
            if (bitr_get(r, bits, &q))
                return -1;
            recon = ((float)q / (float)qmax) * scale;
            prev += recon;
            out[off + i] = prev;
        }
        off += len;
    }
    return 0;
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
    *phase0 = (float)(atan2(sQ, sI) - w * (double)sample_base);
}

static void synth_pilot(float *out, size_t n, uint64_t sample_base, float amp, float phase0)
{
    size_t i;
    double w = 2.0 * M_PI * PILOT_HZ / (double)NANOMPX_SAMPLE_RATE;
    for (i = 0; i < n; i++)
        out[i] = amp * (float)sin(w * (double)(sample_base + i) + (double)phase0);
}

struct nanompx_comp_enc {
    nanompx_profile_t profile;
    float *x;
    float *pilot;
    float *res;
    size_t cap;
    uint64_t sample_phase;
};

struct nanompx_comp_dec {
    float *pilot;
    float *res;
    size_t cap;
    uint64_t sample_phase;
};

static int ensure_enc(nanompx_comp_enc_t *e, size_t n)
{
    if (e->cap >= n)
        return 0;
    free(e->x);
    e->x = (float *)malloc(sizeof(float) * n * 3);
    if (!e->x) {
        e->cap = 0;
        return -1;
    }
    e->pilot = e->x + n;
    e->res = e->pilot + n;
    e->cap = n;
    return 0;
}

static int ensure_dec(nanompx_comp_dec_t *d, size_t n)
{
    if (d->cap >= n)
        return 0;
    free(d->pilot);
    d->pilot = (float *)malloc(sizeof(float) * n * 2);
    if (!d->pilot) {
        d->cap = 0;
        return -1;
    }
    d->res = d->pilot + n;
    d->cap = n;
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

    if (!e || !samples || !dst || !out_len || count == 0 || count > 65535)
        return NANOMPX_ERR_INVALID;
    if (ensure_enc(e, count) != 0)
        return NANOMPX_ERR_NOMEM;
    if (dst_cap < hdr + 64)
        return NANOMPX_ERR_OVERFLOW;

    if (keyframe)
        e->sample_phase = 0;

    cfg = profile_cfg(e->profile);
    base = e->sample_phase;

    for (i = 0; i < count; i++) {
        e->x[i] = pcm24_to_float(samples[i]);
        {
            float a = fabsf(e->x[i]);
            if (a > peak)
                peak = a;
        }
    }

    fit_pilot(e->x, count, base, &pamp, &pphase);
    synth_pilot(e->pilot, count, base, pamp, pphase);
    for (i = 0; i < count; i++)
        e->res[i] = e->x[i] - e->pilot[i];

    dst[0] = COMP_MAGIC;
    dst[1] = (uint8_t)e->profile;
    dst[2] = (uint8_t)(count & 0xff);
    dst[3] = (uint8_t)((count >> 8) & 0xff);
    dst[4] = (uint8_t)cfg.bits;
    dst[5] = 1;
    {
        uint16_t pk = (uint16_t)lrintf(peak * 65535.f);
        dst[6] = (uint8_t)(pk & 0xff);
        dst[7] = (uint8_t)((pk >> 8) & 0xff);
    }

    bitw_init(&w, dst + hdr, dst_cap - hdr);
    if (bitw_put_f32(&w, pamp) || bitw_put_f32(&w, pphase))
        return NANOMPX_ERR_OVERFLOW;
    if (bitw_put(&w, (uint32_t)count, 16) || quantize_pred(e->res, count, cfg.bits, &w))
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
    free(d->pilot);
    free(d);
}

int nanompx_comp_decode(nanompx_comp_dec_t *d,
                        nanompx_profile_t profile,
                        const uint8_t *src, size_t src_len,
                        int32_t *samples, size_t max_samples, size_t *out_count,
                        int keyframe)
{
    size_t count, n;
    bitr_t r;
    float peak, pamp, pphase;
    int32_t tmp;
    size_t i;
    int bits;
    uint64_t base;

    (void)profile;
    if (!d || !src || !samples || !out_count || src_len < 8)
        return NANOMPX_ERR_INVALID;
    if (src[0] != COMP_MAGIC)
        return NANOMPX_ERR_INVALID;

    count = (size_t)src[2] | ((size_t)src[3] << 8);
    bits = src[4];
    if (bits < 2 || bits > 15)
        return NANOMPX_ERR_INVALID;
    peak = ((float)(src[6] | (src[7] << 8))) / 65535.f;
    if (count == 0 || count > max_samples)
        return NANOMPX_ERR_INVALID;
    if (ensure_dec(d, count) != 0)
        return NANOMPX_ERR_NOMEM;

    if (keyframe)
        d->sample_phase = 0;
    base = d->sample_phase;

    bitr_init(&r, src + 8, src_len - 8);
    if (bitr_get_f32(&r, &pamp) || bitr_get_f32(&r, &pphase))
        return NANOMPX_ERR_INVALID;
    if (bitr_get(&r, 16, &tmp))
        return NANOMPX_ERR_INVALID;
    n = (size_t)(uint16_t)tmp;
    if (n != count)
        return NANOMPX_ERR_INVALID;
    if (dequantize_pred(d->res, n, bits, &r))
        return NANOMPX_ERR_INVALID;
    synth_pilot(d->pilot, count, base, pamp, pphase);

    for (i = 0; i < count; i++) {
        float y = d->res[i] + d->pilot[i];
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
