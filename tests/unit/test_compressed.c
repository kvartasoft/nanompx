/* SPDX-License-Identifier: MIT */
#include "nanompx.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int fail(const char *msg)
{
    fprintf(stderr, "FAIL: %s\n", msg);
    return 1;
}

/* Band-weighted metrics: mono/stereo matter for FM; flat MPX SNR does not. */
static double band_energy(const double *x, unsigned n, double f0, double f1)
{
    /* Goertzel-ish power via DFT bins over [f0,f1] — rectangular, fine for gates */
    unsigned k0, k1, k, i;
    double e = 0.0;
    k0 = (unsigned)(f0 * (double)n / (double)NANOMPX_SAMPLE_RATE);
    k1 = (unsigned)(f1 * (double)n / (double)NANOMPX_SAMPLE_RATE);
    if (k1 >= n / 2)
        k1 = n / 2 - 1;
    if (k1 < k0)
        return 0.0;
    for (k = k0; k <= k1; k++) {
        double re = 0.0, im = 0.0;
        double w = 2.0 * M_PI * (double)k / (double)n;
        for (i = 0; i < n; i++) {
            re += x[i] * cos(w * (double)i);
            im -= x[i] * sin(w * (double)i);
        }
        e += re * re + im * im;
    }
    return e;
}

static double snr_db(double sig, double err)
{
    return 10.0 * log10((sig + 1e-20) / (err + 1e-20));
}

static int test_profile(nanompx_profile_t profile, const char *name,
                        double min_mono_db, double min_stereo_db, double min_weighted_db)
{
    const unsigned n = NANOMPX_DEFAULT_FRAME_SAMPLES * 4;
    int32_t *in;
    int32_t *out;
    uint8_t *pkt;
    double *xin, *xout, *err;
    size_t pkt_cap = 512 * 1024;
    size_t wrote = 0, decoded = 0, off = 0;
    nanompx_encoder_t *enc;
    nanompx_decoder_t *dec;
    nanompx_clock_t *clk;
    unsigned i;
    double peak_in = 0, peak_out = 0;
    uint32_t target = nanompx_profile_bitrate(profile);
    double bps;
    double raw_sig = 0, raw_err = 0;
    double mono_s, mono_e, st_s, st_e, rds_s, rds_e;
    double mono_snr, st_snr, rds_snr, raw_snr, weighted;

    in = (int32_t *)calloc(n, sizeof(int32_t));
    out = (int32_t *)calloc(n, sizeof(int32_t));
    xin = (double *)calloc(n, sizeof(double));
    xout = (double *)calloc(n, sizeof(double));
    err = (double *)calloc(n, sizeof(double));
    pkt = (uint8_t *)malloc(pkt_cap);
    if (!in || !out || !pkt || !xin || !xout || !err)
        return fail("oom");

    for (i = 0; i < n; i++) {
        double t = (double)i / (double)NANOMPX_SAMPLE_RATE;
        double audio = 0.35 * sin(2.0 * M_PI * 2200.0 * t);
        double pilot = 0.09 * sin(2.0 * M_PI * 19000.0 * t);
        double diff = 0.15 * sin(2.0 * M_PI * 38000.0 * t) *
                      sin(2.0 * M_PI * 3000.0 * t);
        double rds = 0.03 * sin(2.0 * M_PI * 57000.0 * t);
        double x = audio + pilot + diff + rds;
        in[i] = (int32_t)lrint(x * 8388607.0);
        if (fabs(x) > peak_in)
            peak_in = fabs(x);
    }

    enc = nanompx_encoder_create();
    dec = nanompx_decoder_create();
    clk = nanompx_clock_host_create();
    nanompx_encoder_set_clock(enc, clk);
    nanompx_decoder_set_clock(dec, clk);
    nanompx_encoder_set_mode(enc, NANOMPX_MODE_COMPRESSED);
    nanompx_encoder_set_profile(enc, profile);
    nanompx_decoder_set_sfn_enabled(dec, 0);

    if (nanompx_encoder_push_pcm(enc, in, n, pkt, pkt_cap, &wrote) != NANOMPX_OK)
        return fail("encode");

    while (off + NANOMPX_HDR_SIZE <= wrote) {
        nanompx_packet_hdr_t hdr;
        size_t plen;
        size_t got = 0;
        if (nanompx_hdr_unpack(&hdr, pkt + off) != NANOMPX_OK)
            return fail("hdr");
        if (hdr.mode != NANOMPX_MODE_COMPRESSED || hdr.profile != (uint8_t)profile)
            return fail("profile mismatch on wire");
        plen = NANOMPX_HDR_SIZE + hdr.payload_len;
        if (nanompx_decoder_push_packet(dec, pkt + off, plen) != NANOMPX_OK)
            return fail("push");
        off += plen;
        while (decoded < n &&
               nanompx_decoder_pull_pcm(dec, out + decoded, n - decoded, &got) == NANOMPX_OK) {
            decoded += got;
        }
    }

    if (decoded != n) {
        fprintf(stderr, "%s decoded %zu expected %u\n", name, decoded, n);
        return fail("count");
    }

    for (i = 0; i < n; i++) {
        xin[i] = (double)in[i] / 8388608.0;
        xout[i] = (double)out[i] / 8388608.0;
        if (fabs(xout[i]) > peak_out)
            peak_out = fabs(xout[i]);
        if (fabs(xout[i]) > peak_in + 0.02)
            return fail("invented overshoot");
        raw_sig += xin[i] * xin[i];
    }

    /* Codec has ~2×FIR group delay; align before SNR (fair vs MicroMPX-style checks). */
    {
        unsigned best_lag = 0;
        double best_c = -1e300;
        unsigned lag;
        unsigned max_lag = 256;
        for (lag = 0; lag <= max_lag; lag++) {
            double c = 0.0;
            unsigned j;
            for (j = 0; j + lag < n; j++)
                c += xin[j] * xout[j + lag];
            if (c > best_c) {
                best_c = c;
                best_lag = lag;
            }
        }
        for (i = 0; i + best_lag < n; i++)
            err[i] = xout[i + best_lag] - xin[i];
        for (; i < n; i++)
            err[i] = 0.0;
        raw_err = 0.0;
        for (i = 0; i + best_lag < n; i++)
            raw_err += err[i] * err[i];
        fprintf(stderr, "profile %s: align_lag=%u samples\n", name, best_lag);
    }

    /* Skip settle + use aligned error */
    {
        unsigned skip = NANOMPX_SAMPLE_RATE / 500;
        unsigned m = n - skip;
        if (m > 256)
            m -= 128;
        mono_s = band_energy(xin + skip, m, 200.0, 15000.0);
        mono_e = band_energy(err + skip, m, 200.0, 15000.0);
        st_s = band_energy(xin + skip, m, 23000.0, 53000.0);
        st_e = band_energy(err + skip, m, 23000.0, 53000.0);
        rds_s = band_energy(xin + skip, m, 54500.0, 59500.0);
        rds_e = band_energy(err + skip, m, 54500.0, 59500.0);
    }

    mono_snr = snr_db(mono_s, mono_e);
    st_snr = snr_db(st_s, st_e);
    rds_snr = snr_db(rds_s, rds_e);
    raw_snr = snr_db(raw_sig, raw_err);
    /* Emphasize mono + stereo (what listeners hear after demod) */
    weighted = 0.55 * mono_snr + 0.35 * st_snr + 0.10 * rds_snr;

    bps = (double)wrote * 8.0 * ((double)NANOMPX_SAMPLE_RATE / (double)n);
    fprintf(stderr,
            "profile %s: ~%.0f bps (target %u) | mono=%.1f stereo=%.1f rds=%.1f "
            "weighted=%.1f raw_mpx=%.1f dB | peak_in=%.3f peak_out=%.3f\n",
            name, bps, target, mono_snr, st_snr, rds_snr, weighted, raw_snr, peak_in, peak_out);

    if (bps > (double)target * 1.50) {
        fprintf(stderr, "bitrate too high for profile %s\n", name);
        return fail("bitrate");
    }
    if (mono_snr < min_mono_db) {
        fprintf(stderr, "mono SNR too low for %s (%.1f < %.1f)\n", name, mono_snr, min_mono_db);
        return fail("mono_snr");
    }
    if (st_snr < min_stereo_db) {
        fprintf(stderr, "stereo SNR too low for %s (%.1f < %.1f)\n", name, st_snr, min_stereo_db);
        return fail("stereo_snr");
    }
    if (weighted < min_weighted_db) {
        fprintf(stderr, "weighted SNR too low for %s (%.1f < %.1f)\n", name, weighted, min_weighted_db);
        return fail("weighted_snr");
    }

    nanompx_encoder_destroy(enc);
    nanompx_decoder_destroy(dec);
    nanompx_clock_destroy(clk);
    free(in);
    free(out);
    free(xin);
    free(xout);
    free(err);
    free(pkt);
    return 0;
}

int main(void)
{
    /* Gates are band-weighted (fairer than flat MPX waveform SNR). */
    /* Gates reflect band-weighted FM quality (not flat MPX SNR).
     * Filterbank PR ceilings ~40 dB on synthetic MPX; profiles allocate bits. */
    if (test_profile(NANOMPX_PROFILE_L, "L", 38.0, 30.0, 36.0))
        return 1;
    if (test_profile(NANOMPX_PROFILE_M, "M", 30.0, 24.0, 28.0))
        return 1;
    if (test_profile(NANOMPX_PROFILE_S, "S", 22.0, 16.0, 20.0))
        return 1;
    printf("Compressed L/M/S OK (FM band metrics)\n");
    return 0;
}
