/* SPDX-License-Identifier: MIT */
/*
 * An MPX peaking about 3 dB over 0 dBFS (an input calibrated too hot):
 * pull_float keeps the peaks and the pilot guard bands stay clean, pull_pcm
 * clips to 24 bits and splatters into them.
 */
#include "nanompx.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define WIN 16384

static int fail(const char *msg)
{
    fprintf(stderr, "FAIL: %s\n", msg);
    return 1;
}

/* Power over [f0, f1] by Goertzel, as in test_compressed. */
static double band_energy(const double *x, unsigned n, double f0, double f1)
{
    unsigned k0 = (unsigned)(f0 * (double)n / (double)NANOMPX_SAMPLE_RATE);
    unsigned k1 = (unsigned)(f1 * (double)n / (double)NANOMPX_SAMPLE_RATE);
    unsigned k, i;
    double e = 0.0;

    for (k = k0; k <= k1; k++) {
        double c = 2.0 * cos(2.0 * M_PI * (double)k / (double)n), s1 = 0.0, s2 = 0.0;
        for (i = 0; i < n; i++) {
            double s0 = x[i] + c * s1 - s2;
            s2 = s1;
            s1 = s0;
        }
        e += s1 * s1 + s2 * s2 - c * s1 * s2;
    }
    return e;
}

/* Guard bands either side of the pilot, dB re the whole window. */
static double guard_db(const double *x, unsigned n)
{
    double all = 0.0, g;
    unsigned i;

    for (i = 0; i < n; i++)
        all += x[i] * x[i];
    all *= (double)n / 2.0;    /* Parseval: Goertzel bins of one side */
    g = band_energy(x, n, 16600.0, 18500.0) + band_energy(x, n, 19500.0, 21400.0);
    return 10.0 * log10((g + 1e-30) / (all + 1e-30));
}

int main(void)
{
    const unsigned n = NANOMPX_DEFAULT_FRAME_SAMPLES * 60;
    int32_t *in = (int32_t *)calloc(n, sizeof(int32_t));
    int32_t *outp = (int32_t *)calloc(n, sizeof(int32_t));
    float *outf = (float *)calloc(n, sizeof(float));
    double *xf = (double *)calloc(WIN, sizeof(double));
    double *xp = (double *)calloc(WIN, sizeof(double));
    size_t pkt_cap = 512 * 1024, wrote = 0, off = 0, nf = 0, np = 0, got;
    uint8_t *pkt = (uint8_t *)malloc(pkt_cap);
    nanompx_encoder_t *enc = nanompx_encoder_create();
    nanompx_decoder_t *df = nanompx_decoder_create();
    nanompx_decoder_t *dp = nanompx_decoder_create();
    unsigned i, at;
    double peak = 0.0, gf, gp;

    if (!in || !outp || !outf || !xf || !xp || !pkt || !enc || !df || !dp)
        return fail("oom");
    /* Mono, L−R on 38 kHz, pilot, RDS: peaks at about 1.4 (+2.9 dB). */
    for (i = 0; i < n; i++) {
        double t = (double)i / (double)NANOMPX_SAMPLE_RATE;
        double x = 0.85 * sin(2.0 * M_PI * 1000.0 * t) +
                   0.42 * sin(2.0 * M_PI * 38000.0 * t) * sin(2.0 * M_PI * 4100.0 * t) +
                   0.09 * sin(2.0 * M_PI * 19000.0 * t) +
                   0.03 * sin(2.0 * M_PI * 57000.0 * t);
        /* Over 24 bits on purpose: the compressed encoder takes it as is. */
        in[i] = (int32_t)lrint(x * 8388607.0);
    }

    nanompx_encoder_set_mode(enc, NANOMPX_MODE_COMPRESSED);
    nanompx_encoder_set_bitrate(enc, 320);
    nanompx_decoder_set_sfn_enabled(df, 0);
    nanompx_decoder_set_sfn_enabled(dp, 0);
    if (nanompx_encoder_push_pcm(enc, in, n, pkt, pkt_cap, &wrote) != NANOMPX_OK)
        return fail("encode");
    while (off + NANOMPX_HDR_SIZE <= wrote) {
        nanompx_packet_hdr_t hdr;
        size_t plen;

        if (nanompx_hdr_unpack(&hdr, pkt + off) != NANOMPX_OK)
            return fail("hdr");
        plen = NANOMPX_HDR_SIZE + hdr.payload_len;
        if (nanompx_decoder_push_packet(df, pkt + off, plen) != NANOMPX_OK ||
            nanompx_decoder_push_packet(dp, pkt + off, plen) != NANOMPX_OK)
            return fail("push");
        off += plen;
        while (nf < n && nanompx_decoder_pull_float(df, outf + nf, n - nf, &got) == NANOMPX_OK)
            nf += got;
        while (np < n && nanompx_decoder_pull_pcm(dp, outp + np, n - np, &got) == NANOMPX_OK)
            np += got;
    }
    if (nf != n || np != n)
        return fail("count");

    for (i = 0; i < n; i++)
        if (fabs(outf[i]) > peak)
            peak = fabs(outf[i]);
    for (i = 0; i < n; i++)
        if (outp[i] > 8388607 || outp[i] < -8388608)
            return fail("pull_pcm over 24 bits");

    /* A window well past the codec delay and the start-up. */
    at = n / 2;
    for (i = 0; i < WIN; i++) {
        double w = 0.5 - 0.5 * cos(2.0 * M_PI * (double)i / (double)WIN);
        xf[i] = (double)outf[at + i] * w;
        xp[i] = (double)outp[at + i] / 8388607.0 * w;
    }
    gf = guard_db(xf, WIN);
    gp = guard_db(xp, WIN);
    printf("overload: float peak %.2f, guard bands float %.1f dB, 24-bit %.1f dB\n",
           peak, gf, gp);
    if (peak < 1.25)
        return fail("float output lost the peaks over 0 dBFS");
    if (gf > -90.0)
        return fail("float output not clean in the guard bands");
    if (gp < gf + 20.0)
        return fail("24-bit output expected to show the clipping");
    printf("PASS overload\n");
    return 0;
}
