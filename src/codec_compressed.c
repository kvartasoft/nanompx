/* SPDX-License-Identifier: MIT */
/**
 * FM-aware NanoMPX compressed codec (v2 wire: magic 0xC2).
 *
 * Low-bit strategy focused on SNR:
 *  - parametric continuous 19 kHz pilot (tiny, protected)
 *  - DPCM + noise-shaped block-float on the residual
 *  - mild/no decimation so stereo/RDS spectrum is preserved
 *  - soft peak limit (no invented overshoots)
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

typedef struct biquad {
    float b0, b1, b2, a1, a2;
    float z1, z2;
} biquad_t;

static void bq_reset(biquad_t *f) { f->z1 = f->z2 = 0.f; }

static float bq_process(biquad_t *f, float x)
{
    float y = f->b0 * x + f->z1;
    f->z1 = f->b1 * x - f->a1 * y + f->z2;
    f->z2 = f->b2 * x - f->a2 * y;
    return y;
}

static void bq_lowpass(biquad_t *f, float fc, float q)
{
    float w0 = (float)(2.0 * M_PI * fc / (double)NANOMPX_SAMPLE_RATE);
    float alpha = sinf(w0) / (2.f * q);
    float cosw = cosf(w0);
    float a0 = 1.f + alpha;
    f->b0 = ((1.f - cosw) * 0.5f) / a0;
    f->b1 = (1.f - cosw) / a0;
    f->b2 = f->b0;
    f->a1 = (-2.f * cosw) / a0;
    f->a2 = (1.f - alpha) / a0;
    bq_reset(f);
}

typedef struct profile_cfg {
    int bits;
    int decim;
} profile_cfg_t;

static profile_cfg_t profile_cfg(nanompx_profile_t p)
{
    profile_cfg_t c;
    switch (p) {
    case NANOMPX_PROFILE_L:
        /* ~1536 kbit/s residual */
        c.bits = 8;
        c.decim = 1;
        break;
    case NANOMPX_PROFILE_M:
        /* ~960 kbit/s residual — keep full spectrum, fewer bits */
        c.bits = 5;
        c.decim = 1;
        break;
    case NANOMPX_PROFILE_S:
    default:
        /* ~576 kbit/s residual */
        c.bits = 3;
        c.decim = 1;
        break;
    }
    return c;
}

#define BLOCK 48
#define COMP_MAGIC 0xC2
#define PILOT_HZ 19000.0
#define NS_COEF 0.55f
#define SCALE_HEADROOM 1.20f

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

static void filter_buf(biquad_t *f, const float *in, float *out, size_t n)
{
    size_t i;
    bq_reset(f);
    for (i = 0; i < n; i++)
        out[i] = bq_process(f, in[i]);
}

static void decimate_pick(const float *in, size_t n, int decim, float *out, size_t *out_n)
{
    size_t i, k = 0;
    for (i = 0; i < n; i += (size_t)decim)
        out[k++] = in[i];
    *out_n = k;
}

static void upsample_lin(const float *in, size_t in_n, int decim, float *out, size_t out_n)
{
    size_t i;
    if (decim <= 1) {
        size_t n = in_n < out_n ? in_n : out_n;
        memcpy(out, in, n * sizeof(float));
        for (i = n; i < out_n; i++)
            out[i] = in_n ? in[in_n - 1] : 0.f;
        return;
    }
    for (i = 0; i < out_n; i++) {
        float pos = (float)i / (float)decim;
        size_t i0 = (size_t)pos;
        float frac = pos - (float)i0;
        float a, b;
        if (i0 >= in_n) {
            out[i] = in_n ? in[in_n - 1] : 0.f;
            continue;
        }
        a = in[i0];
        b = (i0 + 1 < in_n) ? in[i0 + 1] : a;
        out[i] = a + (b - a) * frac;
    }
}

static int quantize_pred(const float *in, size_t n, int bits, bitw_t *w)
{
    size_t off = 0;
    float prev = 0.f;
    float err = 0.f;
    int32_t qmax = (1 << (bits - 1)) - 1;

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
    biquad_t aa;
    float *tmp;
    float *scratch;
    float *pilot;
    size_t cap;
    uint64_t sample_phase;
};

struct nanompx_comp_dec {
    float *scratch;
    float *pilot;
    float *mainbuf;
    size_t cap;
    uint64_t sample_phase;
};

static int ensure_enc_buf(nanompx_comp_enc_t *e, size_t n)
{
    if (e->cap >= n)
        return 0;
    free(e->tmp);
    e->tmp = (float *)malloc(sizeof(float) * n * 3);
    if (!e->tmp) {
        e->cap = 0;
        return -1;
    }
    e->scratch = e->tmp + n;
    e->pilot = e->scratch + n;
    e->cap = n;
    return 0;
}

static int ensure_dec_buf(nanompx_comp_dec_t *d, size_t n)
{
    if (d->cap >= n)
        return 0;
    free(d->scratch);
    d->scratch = (float *)malloc(sizeof(float) * n * 3);
    if (!d->scratch) {
        d->cap = 0;
        return -1;
    }
    d->pilot = d->scratch + n;
    d->mainbuf = d->pilot + n;
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
    free(e->tmp);
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
    size_t n_main;
    size_t hdr = 8;
    uint64_t base;

    if (!e || !samples || !dst || !out_len || count == 0 || count > 65535)
        return NANOMPX_ERR_INVALID;
    if (ensure_enc_buf(e, count) != 0)
        return NANOMPX_ERR_NOMEM;
    if (dst_cap < hdr + 32)
        return NANOMPX_ERR_OVERFLOW;

    if (keyframe)
        e->sample_phase = 0;

    cfg = profile_cfg(e->profile);
    base = e->sample_phase;

    for (i = 0; i < count; i++) {
        e->tmp[i] = pcm24_to_float(samples[i]);
        {
            float a = fabsf(e->tmp[i]);
            if (a > peak)
                peak = a;
        }
    }

    fit_pilot(e->tmp, count, base, &pamp, &pphase);
    synth_pilot(e->pilot, count, base, pamp, pphase);
    for (i = 0; i < count; i++)
        e->scratch[i] = e->tmp[i] - e->pilot[i];

    if (cfg.decim > 1) {
        float fc = 0.45f * (float)NANOMPX_SAMPLE_RATE / (float)cfg.decim;
        bq_lowpass(&e->aa, fc, 0.707f);
        filter_buf(&e->aa, e->scratch, e->tmp, count);
        decimate_pick(e->tmp, count, cfg.decim, e->scratch, &n_main);
    } else {
        n_main = count;
        /* scratch already holds residual */
    }

    dst[0] = COMP_MAGIC;
    dst[1] = (uint8_t)e->profile;
    dst[2] = (uint8_t)(count & 0xff);
    dst[3] = (uint8_t)((count >> 8) & 0xff);
    dst[4] = (uint8_t)cfg.bits;
    dst[5] = (uint8_t)cfg.decim;
    {
        uint16_t pk = (uint16_t)lrintf(peak * 65535.f);
        dst[6] = (uint8_t)(pk & 0xff);
        dst[7] = (uint8_t)((pk >> 8) & 0xff);
    }

    bitw_init(&w, dst + hdr, dst_cap - hdr);
    if (bitw_put_f32(&w, pamp) || bitw_put_f32(&w, pphase))
        return NANOMPX_ERR_OVERFLOW;
    if (bitw_put(&w, (uint32_t)n_main, 16) ||
        quantize_pred(e->scratch, n_main, cfg.bits, &w))
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
    free(d->scratch);
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
    float peak;
    float pamp, pphase;
    size_t n_main;
    int32_t tmp;
    size_t i;
    int bits, decim;
    uint64_t base;

    (void)profile;
    if (!d || !src || !samples || !out_count || src_len < 8)
        return NANOMPX_ERR_INVALID;
    if (src[0] != COMP_MAGIC)
        return NANOMPX_ERR_INVALID;

    count = (size_t)src[2] | ((size_t)src[3] << 8);
    bits = src[4];
    decim = src[5];
    if (bits < 2 || bits > 15 || decim < 1 || decim > 16)
        return NANOMPX_ERR_INVALID;

    peak = ((float)(src[6] | (src[7] << 8))) / 65535.f;
    if (count == 0 || count > max_samples)
        return NANOMPX_ERR_INVALID;
    if (ensure_dec_buf(d, count) != 0)
        return NANOMPX_ERR_NOMEM;

    if (keyframe)
        d->sample_phase = 0;
    base = d->sample_phase;

    bitr_init(&r, src + 8, src_len - 8);
    if (bitr_get_f32(&r, &pamp) || bitr_get_f32(&r, &pphase))
        return NANOMPX_ERR_INVALID;
    if (bitr_get(&r, 16, &tmp))
        return NANOMPX_ERR_INVALID;
    n_main = (size_t)(uint16_t)tmp;
    if (n_main == 0 || n_main > count)
        return NANOMPX_ERR_INVALID;
    if (dequantize_pred(d->scratch, n_main, bits, &r))
        return NANOMPX_ERR_INVALID;
    upsample_lin(d->scratch, n_main, decim, d->mainbuf, count);
    synth_pilot(d->pilot, count, base, pamp, pphase);

    for (i = 0; i < count; i++) {
        float y = d->mainbuf[i] + d->pilot[i];
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
