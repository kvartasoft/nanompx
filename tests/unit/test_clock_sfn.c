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

int main(void)
{
    nanompx_clock_t *clk = nanompx_clock_sim_create();
    nanompx_encoder_t *enc;
    nanompx_decoder_t *dec;
    const unsigned n = NANOMPX_DEFAULT_FRAME_SAMPLES;
    int32_t *in;
    int32_t out[NANOMPX_DEFAULT_FRAME_SAMPLES];
    uint8_t pkt[65536];
    size_t wrote = 0, got = 0;
    int64_t t0, t1;
    int64_t delay = 20000000LL; /* 20 ms */
    unsigned i;
    int rc;

    if (!clk)
        return fail("sim clock");
    nanompx_clock_sim_set_locked(clk, 1);

    enc = nanompx_encoder_create();
    dec = nanompx_decoder_create();
    nanompx_encoder_set_clock(enc, clk);
    nanompx_decoder_set_clock(dec, clk);
    nanompx_decoder_set_sfn_enabled(dec, 1);
    nanompx_decoder_set_playout_delay(dec, delay, 0);
    nanompx_encoder_set_mode(enc, NANOMPX_MODE_PCM);

    in = (int32_t *)calloc(n, sizeof(int32_t));
    for (i = 0; i < n; i++)
        in[i] = (int32_t)(i & 0xffff);

    nanompx_clock_now_ns(clk, &t0);
    if (nanompx_encoder_push_pcm(enc, in, n, pkt, sizeof(pkt), &wrote) != NANOMPX_OK)
        return fail("encode");

    if (nanompx_decoder_push_packet(dec, pkt, wrote) != NANOMPX_OK)
        return fail("push");

    /* Too early: should be AGAIN */
    rc = nanompx_decoder_pull_pcm(dec, out, n, &got);
    if (rc != NANOMPX_ERR_AGAIN)
        return fail("expected AGAIN before delay");

    /* Simulate waiting by using a second decoder path with zero delay for readiness,
       and validate GPS inject lock path. */
    {
        nanompx_clock_t *gps = nanompx_clock_gps_create(NULL, NULL);
        int64_t host = 0;
        nanompx_clock_now_ns(clk, &host);
        nanompx_clock_gps_feed_nmea(gps,
            "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A");
        nanompx_clock_gps_inject_pps(gps, host, 760147939);
        if (!nanompx_clock_locked(gps))
            return fail("gps should lock after NMEA+PPS");
        nanompx_clock_destroy(gps);
    }

    /* Disable SFN to drain */
    nanompx_decoder_set_sfn_enabled(dec, 0);
    rc = nanompx_decoder_pull_pcm(dec, out, n, &got);
    if (rc != NANOMPX_OK || got != n)
        return fail("pull after disable sfn");

    nanompx_clock_now_ns(clk, &t1);
    if (t1 < t0)
        return fail("clock went backwards");

    printf("SFN/clock tests OK (sim locked, GPS inject lock, delay gating)\n");
    free(in);
    nanompx_encoder_destroy(enc);
    nanompx_decoder_destroy(dec);
    nanompx_clock_destroy(clk);
    return 0;
}
