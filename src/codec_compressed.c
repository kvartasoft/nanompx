/* SPDX-License-Identifier: MIT */
/**
 * FM-aware NanoMPX compressed codec (v1).
 *
 * Band-split composite → baseband (L+R), 19 kHz pilot, L−R, RDS.
 * Pilot is parametric (protected phase/amp). RDS uses high-depth decimated
 * PCM. Audio bands use profile-dependent block floating-point quantization.
 * Reconstruction is soft-limited to the source frame peak (no invented overshoots).
 */

#include "internal.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct biquad {
    float b0, b1, b2, a1, a2;
    float z1, z2;
} biquad_t;

static void bq_reset(biquad_t *f)
{
    f->z1 = f->z2 = 0.f;
}

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

static void bq_highpass(biquad_t *f, float fc, float q)
{
    float w0 = (float)(2.0 * M_PI * fc / (double)NANOMPX_SAMPLE_RATE);
    float alpha = sinf(w0) / (2.f * q);
    float cosw = cosf(w0);
    float a0 = 1.f + alpha;
    f->b0 = ((1.f + cosw) * 0.5f) / a0;
    f->b1 = (-(1.f + cosw)) / a0;
    f->b2 = f->b0;
    f->a1 = (-2.f * cosw) / a0;
    f->a2 = (1.f - alpha) / a0;
    bq_reset(f);
}

static void bq_bandpass(biquad_t *f, float fc, float q)
{
    float w0 = (float)(2.0 * M_PI * fc / (double)NANOMPX_SAMPLE_RATE);
    float alpha = sinf(w0) / (2.f * q);
    float cosw = cosf(w0);
    float a0 = 1.f + alpha;
    f->b0 = alpha / a0;
    f->b1 = 0.f;
    f->b2 = -alpha / a0;
    f->a1 = (-2.f * cosw) / a0;
    f->a2 = (1.f - alpha) / a0;
    bq_reset(f);
}

typedef struct band_bank {
    biquad_t bb_lp1, bb_lp2;
    biquad_t pilot_bp1, pilot_bp2;
    biquad_t diff_hp, diff_lp;
    biquad_t rds_bp1, rds_bp2;
} band_bank_t;

static void band_bank_init(band_bank_t *b)
{
    bq_lowpass(&b->bb_lp1, 15000.f, 0.707f);
    bq_lowpass(&b->bb_lp2, 15000.f, 0.707f);
    bq_bandpass(&b->pilot_bp1, 19000.f, 12.f);
    bq_bandpass(&b->pilot_bp2, 19000.f, 12.f);
    bq_highpass(&b->diff_hp, 23000.f, 0.707f);
    bq_lowpass(&b->diff_lp, 53000.f, 0.707f);
    bq_bandpass(&b->rds_bp1, 57000.f, 10.f);
    bq_bandpass(&b->rds_bp2, 57000.f, 10.f);
}

static void split_bands(band_bank_t *b, const float *in, size_t n,
                        float *bb, float *pilot, float *diff, float *rds)
{
    size_t i;
    for (i = 0; i < n; i++) {
        float x = in[i];
        float y;
        y = bq_process(&b->bb_lp1, x);
        bb[i] = bq_process(&b->bb_lp2, y);
        y = bq_process(&b->pilot_bp1, x);
        pilot[i] = bq_process(&b->pilot_bp2, y);
        y = bq_process(&b->diff_hp, x);
        diff[i] = bq_process(&b->diff_lp, y);
        y = bq_process(&b->rds_bp1, x);
        rds[i] = bq_process(&b->rds_bp2, y);
    }
}

typedef struct profile_cfg {
    int bb_bits;
    int diff_bits;
    int bb_decim;
    int diff_decim;
    int rds_decim;
} profile_cfg_t;

static profile_cfg_t profile_cfg(nanompx_profile_t p)
{
    profile_cfg_t c;
    switch (p) {
    case NANOMPX_PROFILE_L:
        c.bb_bits = 10;
        c.diff_bits = 8;
        c.bb_decim = 2;
        c.diff_decim = 4;
        c.rds_decim = 8;
        break;
    case NANOMPX_PROFILE_M:
        c.bb_bits = 8;
        c.diff_bits = 6;
        c.bb_decim = 4;
        c.diff_decim = 4;
        c.rds_decim = 8;
        break;
    case NANOMPX_PROFILE_S:
    default:
        c.bb_bits = 6;
        c.diff_bits = 5;
        c.bb_decim = 4;
        c.diff_decim = 8;
        c.rds_decim = 16;
        break;
    }
    return c;
}

#define BLOCK 64
#define COMP_MAGIC 0xC1
#define PILOT_HZ 19000.0

static int32_t float_to_pcm24(float x)
{
    if (x > 1.f)
        x = 1.f;
    if (x < -1.f)
        x = -1.f;
    return (int32_t)lrintf(x * 8388607.f);
}

static float pcm24_to_float(int32_t s)
{
    return (float)s / 8388608.f;
}

static float band_peak(const float *x, size_t n)
{
    float p = 1e-12f;
    size_t i;
    for (i = 0; i < n; i++) {
        float a = fabsf(x[i]);
        if (a > p)
            p = a;
    }
    return p;
}

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

static void decimate_avg(const float *in, size_t n, int decim, float *out, size_t *out_n)
{
    size_t i, k = 0;
    for (i = 0; i + (size_t)decim <= n; i += (size_t)decim) {
        float s = 0.f;
        int j;
        for (j = 0; j < decim; j++)
            s += in[i + (size_t)j];
        out[k++] = s / (float)decim;
    }
    *out_n = k;
}

static void upsample_lin(const float *in, size_t in_n, int decim, float *out, size_t out_n)
{
    size_t i;
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

static int quantize_band(const float *in, size_t n, int bits, bitw_t *w)
{
    size_t off = 0;
    while (off < n) {
        size_t len = n - off;
        size_t i;
        float peak;
        float scale;
        int32_t qmax;
        if (len > BLOCK)
            len = BLOCK;
        peak = band_peak(in + off, len);
        scale = peak > 1e-12f ? peak : 1e-12f;
        if (bitw_put_f32(w, scale))
            return -1;
        qmax = (1 << (bits - 1)) - 1;
        for (i = 0; i < len; i++) {
            int32_t q = (int32_t)lrintf((in[off + i] / scale) * (float)qmax);
            if (q > qmax)
                q = qmax;
            if (q < -qmax - 1)
                q = -qmax - 1;
            if (bitw_put(w, (uint32_t)q, bits))
                return -1;
        }
        off += len;
    }
    return 0;
}

static int dequantize_band(float *out, size_t n, int bits, bitr_t *r)
{
    size_t off = 0;
    while (off < n) {
        size_t len = n - off;
        size_t i;
        float scale;
        int32_t qmax;
        if (len > BLOCK)
            len = BLOCK;
        if (bitr_get_f32(r, &scale))
            return -1;
        qmax = (1 << (bits - 1)) - 1;
        for (i = 0; i < len; i++) {
            int32_t q;
            if (bitr_get(r, bits, &q))
                return -1;
            out[off + i] = ((float)q / (float)qmax) * scale;
        }
        off += len;
    }
    return 0;
}

static int write_i16_series(const float *in, size_t n, bitw_t *w)
{
    size_t i;
    for (i = 0; i < n; i++) {
        float x = in[i];
        int32_t q;
        if (x > 1.f)
            x = 1.f;
        if (x < -1.f)
            x = -1.f;
        q = (int32_t)lrintf(x * 32767.f);
        if (bitw_put(w, (uint32_t)(uint16_t)(int16_t)q, 16))
            return -1;
    }
    return 0;
}

static int read_i16_series(float *out, size_t n, bitr_t *r)
{
    size_t i;
    for (i = 0; i < n; i++) {
        int32_t q;
        if (bitr_get(r, 16, &q))
            return -1;
        out[i] = (float)(int16_t)q / 32767.f;
    }
    return 0;
}

static void fit_pilot(const float *pilot, size_t n, float *amp, float *phase)
{
    /* Correlate against 19 kHz sine/cosine over the frame. */
    double sI = 0.0, sQ = 0.0;
    size_t i;
    double w = 2.0 * M_PI * PILOT_HZ / (double)NANOMPX_SAMPLE_RATE;
    for (i = 0; i < n; i++) {
        double t = (double)i;
        sI += (double)pilot[i] * cos(w * t);
        sQ += (double)pilot[i] * sin(w * t);
    }
    sI *= 2.0 / (double)n;
    sQ *= 2.0 / (double)n;
    *amp = (float)sqrt(sI * sI + sQ * sQ);
    *phase = (float)atan2(sQ, sI);
}

static void synth_pilot(float *out, size_t n, float amp, float phase)
{
    size_t i;
    double w = 2.0 * M_PI * PILOT_HZ / (double)NANOMPX_SAMPLE_RATE;
    for (i = 0; i < n; i++)
        out[i] = amp * (float)sin(w * (double)i + (double)phase);
}

struct nanompx_comp_enc {
    nanompx_profile_t profile;
    band_bank_t bank;
    float *bb;
    float *pilot;
    float *diff;
    float *rds;
    float *tmp;
    float *scratch;
    size_t cap;
    uint64_t sample_phase; /* continuous time for future use */
};

struct nanompx_comp_dec {
    float *bb;
    float *pilot;
    float *diff;
    float *rds;
    float *scratch;
    size_t cap;
};

static int ensure_enc_buf(nanompx_comp_enc_t *e, size_t n)
{
    if (e->cap >= n)
        return 0;
    free(e->bb);
    e->bb = (float *)malloc(sizeof(float) * n * 6);
    if (!e->bb) {
        e->cap = 0;
        return -1;
    }
    e->pilot = e->bb + n;
    e->diff = e->pilot + n;
    e->rds = e->diff + n;
    e->tmp = e->rds + n;
    e->scratch = e->tmp + n;
    e->cap = n;
    return 0;
}

static int ensure_dec_buf(nanompx_comp_dec_t *d, size_t n)
{
    if (d->cap >= n)
        return 0;
    free(d->bb);
    d->bb = (float *)malloc(sizeof(float) * n * 5);
    if (!d->bb) {
        d->cap = 0;
        return -1;
    }
    d->pilot = d->bb + n;
    d->diff = d->pilot + n;
    d->rds = d->diff + n;
    d->scratch = d->rds + n;
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
    band_bank_init(&e->bank);
    return e;
}

void nanompx_comp_enc_destroy(nanompx_comp_enc_t *e)
{
    if (!e)
        return;
    free(e->bb);
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
    size_t n_bb, n_diff, n_rds;
    size_t hdr = 8;
    (void)keyframe;

    if (!e || !samples || !dst || !out_len || count == 0 || count > 65535)
        return NANOMPX_ERR_INVALID;
    if (ensure_enc_buf(e, count) != 0)
        return NANOMPX_ERR_NOMEM;
    if (dst_cap < hdr + 32)
        return NANOMPX_ERR_OVERFLOW;

    cfg = profile_cfg(e->profile);
    for (i = 0; i < count; i++) {
        e->tmp[i] = pcm24_to_float(samples[i]);
        {
            float a = fabsf(e->tmp[i]);
            if (a > peak)
                peak = a;
        }
    }

    split_bands(&e->bank, e->tmp, count, e->bb, e->pilot, e->diff, e->rds);
    fit_pilot(e->pilot, count, &pamp, &pphase);

    dst[0] = COMP_MAGIC;
    dst[1] = (uint8_t)e->profile;
    dst[2] = (uint8_t)(count & 0xff);
    dst[3] = (uint8_t)((count >> 8) & 0xff);
    dst[4] = (uint8_t)cfg.bb_bits;
    dst[5] = (uint8_t)cfg.diff_bits;
    {
        uint16_t pk = (uint16_t)lrintf(peak * 65535.f);
        dst[6] = (uint8_t)(pk & 0xff);
        dst[7] = (uint8_t)((pk >> 8) & 0xff);
    }

    bitw_init(&w, dst + hdr, dst_cap - hdr);
    if (bitw_put_f32(&w, pamp) || bitw_put_f32(&w, pphase))
        return NANOMPX_ERR_OVERFLOW;

    /* RDS protected */
    decimate_avg(e->rds, count, cfg.rds_decim, e->scratch, &n_rds);
    if (bitw_put(&w, (uint32_t)n_rds, 16) || write_i16_series(e->scratch, n_rds, &w))
        return NANOMPX_ERR_OVERFLOW;

    /* Baseband */
    decimate_avg(e->bb, count, cfg.bb_decim, e->scratch, &n_bb);
    if (bitw_put(&w, (uint32_t)n_bb, 16) || quantize_band(e->scratch, n_bb, cfg.bb_bits, &w))
        return NANOMPX_ERR_OVERFLOW;

    /* Difference */
    decimate_avg(e->diff, count, cfg.diff_decim, e->scratch, &n_diff);
    if (bitw_put(&w, (uint32_t)n_diff, 16) ||
        quantize_band(e->scratch, n_diff, cfg.diff_bits, &w))
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
    free(d->bb);
    free(d);
}

int nanompx_comp_decode(nanompx_comp_dec_t *d,
                        nanompx_profile_t profile,
                        const uint8_t *src, size_t src_len,
                        int32_t *samples, size_t max_samples, size_t *out_count,
                        int keyframe)
{
    size_t count;
    profile_cfg_t cfg;
    bitr_t r;
    float peak;
    float pamp, pphase;
    size_t n_bb, n_diff, n_rds;
    int32_t tmp;
    size_t i;
    (void)keyframe;

    if (!d || !src || !samples || !out_count || src_len < 8)
        return NANOMPX_ERR_INVALID;
    if (src[0] != COMP_MAGIC)
        return NANOMPX_ERR_INVALID;

    if (profile < NANOMPX_PROFILE_L || profile > NANOMPX_PROFILE_S)
        profile = (nanompx_profile_t)src[1];
    cfg = profile_cfg(profile);

    count = (size_t)src[2] | ((size_t)src[3] << 8);
    peak = ((float)(src[6] | (src[7] << 8))) / 65535.f;
    if (count == 0 || count > max_samples)
        return NANOMPX_ERR_INVALID;
    if (ensure_dec_buf(d, count) != 0)
        return NANOMPX_ERR_NOMEM;

    bitr_init(&r, src + 8, src_len - 8);
    if (bitr_get_f32(&r, &pamp) || bitr_get_f32(&r, &pphase))
        return NANOMPX_ERR_INVALID;

    if (bitr_get(&r, 16, &tmp))
        return NANOMPX_ERR_INVALID;
    n_rds = (size_t)(uint16_t)tmp;
    if (n_rds > count || read_i16_series(d->scratch, n_rds, &r))
        return NANOMPX_ERR_INVALID;
    upsample_lin(d->scratch, n_rds, cfg.rds_decim, d->rds, count);

    if (bitr_get(&r, 16, &tmp))
        return NANOMPX_ERR_INVALID;
    n_bb = (size_t)(uint16_t)tmp;
    if (n_bb > count || dequantize_band(d->scratch, n_bb, cfg.bb_bits, &r))
        return NANOMPX_ERR_INVALID;
    upsample_lin(d->scratch, n_bb, cfg.bb_decim, d->bb, count);

    if (bitr_get(&r, 16, &tmp))
        return NANOMPX_ERR_INVALID;
    n_diff = (size_t)(uint16_t)tmp;
    if (n_diff > count || dequantize_band(d->scratch, n_diff, cfg.diff_bits, &r))
        return NANOMPX_ERR_INVALID;
    upsample_lin(d->scratch, n_diff, cfg.diff_decim, d->diff, count);

    synth_pilot(d->pilot, count, pamp, pphase);

    for (i = 0; i < count; i++) {
        float y = d->bb[i] + d->pilot[i] + d->diff[i] + d->rds[i];
        if (peak > 0.f) {
            if (y > peak)
                y = peak;
            if (y < -peak)
                y = -peak;
        }
        samples[i] = float_to_pcm24(y);
    }
    *out_count = count;
    return NANOMPX_OK;
}
