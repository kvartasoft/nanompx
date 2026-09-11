/* SPDX-License-Identifier: MIT */
#include "nanompx.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static int fail(const char *msg)
{
    fprintf(stderr, "FAIL: %s\n", msg);
    return 1;
}

int main(void)
{
    const unsigned n = NANOMPX_DEFAULT_FRAME_SAMPLES * 3;
    int32_t *in;
    int32_t *out;
    uint8_t *pkt;
    size_t pkt_cap = n * 4 + 4096;
    size_t wrote = 0;
    nanompx_encoder_t *enc;
    nanompx_decoder_t *dec;
    nanompx_clock_t *clk;
    unsigned i;
    size_t mism = 0;
    size_t off = 0;
    size_t decoded = 0;

    in = (int32_t *)calloc(n, sizeof(int32_t));
    out = (int32_t *)calloc(n, sizeof(int32_t));
    pkt = (uint8_t *)malloc(pkt_cap);
    if (!in || !out || !pkt)
        return fail("oom");

    for (i = 0; i < n; i++) {
        double t = (double)i / (double)NANOMPX_SAMPLE_RATE;
        /* Synthetic MPX-ish: audio + pilot */
        double audio = 0.4 * sin(2.0 * 3.141592653589793 * 1000.0 * t);
        double pilot = 0.1 * sin(2.0 * 3.141592653589793 * 19000.0 * t);
        in[i] = (int32_t)lrint((audio + pilot) * 8388607.0);
    }

    enc = nanompx_encoder_create();
    dec = nanompx_decoder_create();
    clk = nanompx_clock_host_create();
    nanompx_encoder_set_clock(enc, clk);
    nanompx_decoder_set_clock(dec, clk);
    nanompx_encoder_set_mode(enc, NANOMPX_MODE_PCM);
    nanompx_decoder_set_sfn_enabled(dec, 0);

    if (nanompx_encoder_push_pcm(enc, in, n, pkt, pkt_cap, &wrote) != NANOMPX_OK)
        return fail("encode");

    while (off + NANOMPX_HDR_SIZE <= wrote) {
        nanompx_packet_hdr_t hdr;
        size_t plen;
        size_t got = 0;
        if (nanompx_hdr_unpack(&hdr, pkt + off) != NANOMPX_OK)
            return fail("hdr");
        plen = NANOMPX_HDR_SIZE + hdr.payload_len;
        if (nanompx_decoder_push_packet(dec, pkt + off, plen) != NANOMPX_OK)
            return fail("push");
        off += plen;
        while (nanompx_decoder_pull_pcm(dec, out + decoded, n - decoded, &got) == NANOMPX_OK) {
            decoded += got;
            if (decoded >= n)
                break;
        }
    }

    if (decoded != n) {
        fprintf(stderr, "decoded %zu expected %u\n", decoded, n);
        return fail("count");
    }
    for (i = 0; i < n; i++) {
        if (in[i] != out[i])
            mism++;
    }
    if (mism) {
        fprintf(stderr, "mismatches %zu\n", mism);
        return fail("pcm mismatch");
    }

    printf("PCM roundtrip OK (%u samples, %zu packet bytes)\n", n, wrote);
    nanompx_encoder_destroy(enc);
    nanompx_decoder_destroy(dec);
    nanompx_clock_destroy(clk);
    free(in);
    free(out);
    free(pkt);
    return 0;
}
