/* SPDX-License-Identifier: MIT */
#include "nanompx.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static int fail(const char *msg)
{
    fprintf(stderr, "FAIL: %s\n", msg);
    return 1;
}

static int test_profile(nanompx_profile_t profile, const char *name)
{
    const unsigned n = NANOMPX_DEFAULT_FRAME_SAMPLES * 2;
    int32_t *in;
    int32_t *out;
    uint8_t *pkt;
    size_t pkt_cap = 256 * 1024;
    size_t wrote = 0, decoded = 0, off = 0;
    nanompx_encoder_t *enc;
    nanompx_decoder_t *dec;
    nanompx_clock_t *clk;
    unsigned i;
    double peak_in = 0, peak_out = 0;
    uint32_t target = nanompx_profile_bitrate(profile);
    double bps;

    in = (int32_t *)calloc(n, sizeof(int32_t));
    out = (int32_t *)calloc(n, sizeof(int32_t));
    pkt = (uint8_t *)malloc(pkt_cap);
    if (!in || !out || !pkt)
        return fail("oom");

    for (i = 0; i < n; i++) {
        double t = (double)i / (double)NANOMPX_SAMPLE_RATE;
        double audio = 0.35 * sin(2.0 * 3.141592653589793 * 2200.0 * t);
        double pilot = 0.09 * sin(2.0 * 3.141592653589793 * 19000.0 * t);
        double diff = 0.15 * sin(2.0 * 3.141592653589793 * 38000.0 * t) *
                      sin(2.0 * 3.141592653589793 * 3000.0 * t);
        double rds = 0.03 * sin(2.0 * 3.141592653589793 * 57000.0 * t);
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
        double x = fabs((double)out[i] / 8388608.0);
        if (x > peak_out)
            peak_out = x;
        /* No invented overshoots above source peak (+ small margin). */
        if (x > peak_in + 0.02)
            return fail("invented overshoot");
    }

    bps = (double)wrote * 8.0 * ((double)NANOMPX_SAMPLE_RATE / (double)n);
    fprintf(stderr, "profile %s: %zu bytes, ~%.0f bps (target %u), peak_in=%.3f peak_out=%.3f\n",
            name, wrote, bps, target, peak_in, peak_out);

    /* Allow generous margin vs target — v1 encoder is not rate-exact. */
    if (bps > (double)target * 2.5) {
        fprintf(stderr, "bitrate too high for profile %s\n", name);
        return fail("bitrate");
    }

    nanompx_encoder_destroy(enc);
    nanompx_decoder_destroy(dec);
    nanompx_clock_destroy(clk);
    free(in);
    free(out);
    free(pkt);
    return 0;
}

int main(void)
{
    if (test_profile(NANOMPX_PROFILE_L, "L"))
        return 1;
    if (test_profile(NANOMPX_PROFILE_M, "M"))
        return 1;
    if (test_profile(NANOMPX_PROFILE_S, "S"))
        return 1;
    printf("Compressed L/M/S OK\n");
    return 0;
}
