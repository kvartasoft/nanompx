/* SPDX-License-Identifier: MIT */
/**
 * NanoMPX CLI — encode/decode raw s24le mono @ 192 kHz files,
 * and optional SRT live path.
 */

#include "nanompx.h"
#include "nanompx_srt.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

static void usage(const char *argv0)
{
    fprintf(stderr,
            "NanoMPX %s\n"
            "Usage:\n"
            "  %s encode [--mode pcm|comp] [--profile L|M|S] [-i in.s24] [-o out.nmpx]\n"
            "  %s decode [--sfn 0|1] [--delay-ms N] [-i in.nmpx] [-o out.s24]\n"
            "  %s roundtrip [--mode pcm|comp] [--profile L|M|S] [-i in.s24]\n"
#if NANOMPX_WITH_SRT
            "  %s srt-send --host H --port P [--mode pcm|comp] [--profile L|M|S] [-i in.s24]\n"
            "  %s srt-recv --host H --port P [--listener] [-o out.s24]\n"
#endif
            "\n"
            "Raw PCM is signed 24-bit little-endian mono @ 192000 Hz (3 bytes/sample).\n",
            nanompx_version_string(), argv0, argv0, argv0
#if NANOMPX_WITH_SRT
            ,
            argv0, argv0
#endif
    );
}

static nanompx_profile_t parse_profile(const char *s)
{
    if (!s)
        return NANOMPX_PROFILE_L;
    if (s[0] == 'L' || s[0] == 'l')
        return NANOMPX_PROFILE_L;
    if (s[0] == 'M' || s[0] == 'm')
        return NANOMPX_PROFILE_M;
    if (s[0] == 'S' || s[0] == 's')
        return NANOMPX_PROFILE_S;
    return NANOMPX_PROFILE_L;
}

static int read_all(const char *path, uint8_t **out, size_t *out_len)
{
    FILE *f = fopen(path, "rb");
    long sz;
    uint8_t *buf;
    if (!f)
        return -1;
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return -1;
    }
    sz = ftell(f);
    if (sz < 0) {
        fclose(f);
        return -1;
    }
    rewind(f);
    buf = (uint8_t *)malloc((size_t)sz);
    if (!buf) {
        fclose(f);
        return -1;
    }
    if (fread(buf, 1, (size_t)sz, f) != (size_t)sz) {
        free(buf);
        fclose(f);
        return -1;
    }
    fclose(f);
    *out = buf;
    *out_len = (size_t)sz;
    return 0;
}

static int write_all(const char *path, const uint8_t *data, size_t len)
{
    FILE *f = fopen(path, "wb");
    if (!f)
        return -1;
    if (fwrite(data, 1, len, f) != len) {
        fclose(f);
        return -1;
    }
    fclose(f);
    return 0;
}

static void s24_to_i32(const uint8_t *src, size_t nbytes, int32_t *dst, size_t *n_samp)
{
    size_t n = nbytes / 3;
    size_t i;
    for (i = 0; i < n; i++) {
        int32_t s = (int32_t)src[i * 3 + 0] | ((int32_t)src[i * 3 + 1] << 8) |
                    ((int32_t)src[i * 3 + 2] << 16);
        if (s & 0x800000)
            s |= ~0xFFFFFF;
        dst[i] = s;
    }
    *n_samp = n;
}

static void i32_to_s24(const int32_t *src, size_t n, uint8_t *dst)
{
    size_t i;
    for (i = 0; i < n; i++) {
        int32_t s = src[i];
        if (s > 0x7FFFFF)
            s = 0x7FFFFF;
        if (s < -0x800000)
            s = -0x800000;
        dst[i * 3 + 0] = (uint8_t)(s & 0xff);
        dst[i * 3 + 1] = (uint8_t)((s >> 8) & 0xff);
        dst[i * 3 + 2] = (uint8_t)((s >> 16) & 0xff);
    }
}

static int cmd_encode(int argc, char **argv)
{
    const char *in_path = "in.s24";
    const char *out_path = "out.nmpx";
    nanompx_mode_t mode = NANOMPX_MODE_PCM;
    nanompx_profile_t profile = NANOMPX_PROFILE_L;
    uint8_t *raw = NULL;
    size_t raw_len = 0;
    int32_t *pcm = NULL;
    size_t n_samp = 0;
    uint8_t *out = NULL;
    size_t out_cap, out_len = 0, produced = 0;
    nanompx_encoder_t *enc;
    nanompx_clock_t *clk;
    size_t off;
    int i;

    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--mode") && i + 1 < argc) {
            mode = (!strcmp(argv[++i], "comp")) ? NANOMPX_MODE_COMPRESSED : NANOMPX_MODE_PCM;
        } else if (!strcmp(argv[i], "--profile") && i + 1 < argc) {
            profile = parse_profile(argv[++i]);
        } else if (!strcmp(argv[i], "-i") && i + 1 < argc) {
            in_path = argv[++i];
        } else if (!strcmp(argv[i], "-o") && i + 1 < argc) {
            out_path = argv[++i];
        }
    }

    if (read_all(in_path, &raw, &raw_len) != 0) {
        fprintf(stderr, "failed to read %s\n", in_path);
        return 1;
    }
    n_samp = raw_len / 3;
    pcm = (int32_t *)malloc(n_samp * sizeof(int32_t));
    if (!pcm) {
        free(raw);
        return 1;
    }
    s24_to_i32(raw, raw_len, pcm, &n_samp);
    free(raw);

    enc = nanompx_encoder_create();
    clk = nanompx_clock_host_create();
    nanompx_encoder_set_clock(enc, clk);
    nanompx_encoder_set_mode(enc, mode);
    if (mode == NANOMPX_MODE_COMPRESSED)
        nanompx_encoder_set_profile(enc, profile);

    out_cap = n_samp * 4 + 65536;
    out = (uint8_t *)malloc(out_cap);
    if (!out) {
        nanompx_encoder_destroy(enc);
        nanompx_clock_destroy(clk);
        free(pcm);
        return 1;
    }

    off = 0;
    while (off < n_samp) {
        size_t chunk = n_samp - off;
        size_t wrote = 0;
        int rc;
        if (chunk > NANOMPX_DEFAULT_FRAME_SAMPLES)
            chunk = NANOMPX_DEFAULT_FRAME_SAMPLES;
        rc = nanompx_encoder_push_pcm(enc, pcm + off, chunk, out + out_len,
                                      out_cap - out_len, &wrote);
        if (rc != NANOMPX_OK) {
            fprintf(stderr, "encode error: %s\n", nanompx_strerror(rc));
            break;
        }
        out_len += wrote;
        produced += chunk;
        off += chunk;
    }
    {
        size_t wrote = 0;
        int rc = nanompx_encoder_flush(enc, out + out_len, out_cap - out_len, &wrote);
        if (rc == NANOMPX_OK)
            out_len += wrote;
    }

    if (write_all(out_path, out, out_len) != 0)
        fprintf(stderr, "failed to write %s\n", out_path);
    else
        fprintf(stderr, "encoded %zu samples -> %zu bytes (%s profile=%d)\n",
                produced, out_len, mode == NANOMPX_MODE_PCM ? "pcm" : "comp",
                (int)profile);

    free(out);
    free(pcm);
    nanompx_encoder_destroy(enc);
    nanompx_clock_destroy(clk);
    return 0;
}

static int cmd_decode(int argc, char **argv)
{
    const char *in_path = "out.nmpx";
    const char *out_path = "out.s24";
    int sfn = 0;
    int delay_ms = 0;
    uint8_t *pkt_stream = NULL;
    size_t stream_len = 0;
    nanompx_decoder_t *dec;
    nanompx_clock_t *clk;
    uint8_t *pcm_bytes = NULL;
    size_t pcm_cap = 0, pcm_len = 0;
    size_t off = 0;
    int i;

    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--sfn") && i + 1 < argc) {
            sfn = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--delay-ms") && i + 1 < argc) {
            delay_ms = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "-i") && i + 1 < argc) {
            in_path = argv[++i];
        } else if (!strcmp(argv[i], "-o") && i + 1 < argc) {
            out_path = argv[++i];
        }
    }

    if (read_all(in_path, &pkt_stream, &stream_len) != 0) {
        fprintf(stderr, "failed to read %s\n", in_path);
        return 1;
    }

    dec = nanompx_decoder_create();
    clk = nanompx_clock_host_create();
    nanompx_decoder_set_clock(dec, clk);
    nanompx_decoder_set_sfn_enabled(dec, sfn);
    nanompx_decoder_set_playout_delay(dec, (int64_t)delay_ms * 1000000LL, 0);

    while (off + NANOMPX_HDR_SIZE <= stream_len) {
        nanompx_packet_hdr_t hdr;
        size_t plen;
        int rc;
        if (nanompx_hdr_unpack(&hdr, pkt_stream + off) != NANOMPX_OK)
            break;
        plen = NANOMPX_HDR_SIZE + hdr.payload_len;
        if (off + plen > stream_len)
            break;
        rc = nanompx_decoder_push_packet(dec, pkt_stream + off, plen);
        if (rc != NANOMPX_OK) {
            fprintf(stderr, "push error: %s\n", nanompx_strerror(rc));
            break;
        }
        off += plen;

        for (;;) {
            int32_t tmp[NANOMPX_DEFAULT_FRAME_SAMPLES];
            size_t got = 0;
            rc = nanompx_decoder_pull_pcm(dec, tmp, NANOMPX_DEFAULT_FRAME_SAMPLES, &got);
            if (rc == NANOMPX_ERR_AGAIN)
                break;
            if (rc != NANOMPX_OK) {
                fprintf(stderr, "pull error: %s\n", nanompx_strerror(rc));
                break;
            }
            if (pcm_len + got * 3 > pcm_cap) {
                size_t ncap = (pcm_cap ? pcm_cap * 2 : 65536);
                uint8_t *nbuf;
                while (ncap < pcm_len + got * 3)
                    ncap *= 2;
                nbuf = (uint8_t *)realloc(pcm_bytes, ncap);
                if (!nbuf)
                    break;
                pcm_bytes = nbuf;
                pcm_cap = ncap;
            }
            i32_to_s24(tmp, got, pcm_bytes + pcm_len);
            pcm_len += got * 3;
        }
    }

    if (write_all(out_path, pcm_bytes, pcm_len) != 0)
        fprintf(stderr, "failed to write %s\n", out_path);
    else
        fprintf(stderr, "decoded -> %zu bytes\n", pcm_len);

    free(pcm_bytes);
    free(pkt_stream);
    nanompx_decoder_destroy(dec);
    nanompx_clock_destroy(clk);
    return 0;
}

static int cmd_roundtrip(int argc, char **argv)
{
    const char *in_path = "in.s24";
    char *tmp_argv_enc[] = { "--mode", "pcm", "--profile", "L", "-i", NULL, "-o", "roundtrip.nmpx" };
    char *tmp_argv_dec[] = { "--sfn", "0", "-i", "roundtrip.nmpx", "-o", "roundtrip.s24" };
    nanompx_mode_t mode = NANOMPX_MODE_PCM;
    nanompx_profile_t profile = NANOMPX_PROFILE_L;
    int i;
    int rc;

    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--mode") && i + 1 < argc) {
            mode = (!strcmp(argv[++i], "comp")) ? NANOMPX_MODE_COMPRESSED : NANOMPX_MODE_PCM;
        } else if (!strcmp(argv[i], "--profile") && i + 1 < argc) {
            profile = parse_profile(argv[++i]);
        } else if (!strcmp(argv[i], "-i") && i + 1 < argc) {
            in_path = argv[++i];
        }
    }

    {
        char mode_s[8];
        char prof_s[2] = { (char)('L' + (profile - 1)), 0 };
        if (profile == NANOMPX_PROFILE_L)
            prof_s[0] = 'L';
        else if (profile == NANOMPX_PROFILE_M)
            prof_s[0] = 'M';
        else
            prof_s[0] = 'S';
        snprintf(mode_s, sizeof(mode_s), "%s", mode == NANOMPX_MODE_PCM ? "pcm" : "comp");
        tmp_argv_enc[1] = mode_s;
        tmp_argv_enc[3] = prof_s;
        tmp_argv_enc[5] = (char *)in_path;
        rc = cmd_encode(8, tmp_argv_enc);
        if (rc)
            return rc;
        rc = cmd_decode(6, tmp_argv_dec);
        if (rc)
            return rc;
    }

    /* Compare PCM if mode is PCM */
    if (mode == NANOMPX_MODE_PCM) {
        uint8_t *a = NULL, *b = NULL;
        size_t la = 0, lb = 0;
        if (read_all(in_path, &a, &la) == 0 && read_all("roundtrip.s24", &b, &lb) == 0) {
            size_t n = la < lb ? la : lb;
            size_t mism = 0;
            size_t j;
            for (j = 0; j < n; j++)
                if (a[j] != b[j])
                    mism++;
            fprintf(stderr, "PCM roundtrip mismatches: %zu / %zu bytes\n", mism, n);
            free(a);
            free(b);
            return mism ? 2 : 0;
        }
        free(a);
        free(b);
    }
    return 0;
}

#if NANOMPX_WITH_SRT
static int cmd_srt_send(int argc, char **argv)
{
    const char *host = "127.0.0.1";
    int port = 9000;
    const char *in_path = "in.s24";
    nanompx_mode_t mode = NANOMPX_MODE_PCM;
    nanompx_profile_t profile = NANOMPX_PROFILE_L;
    nanompx_srt_config_t cfg;
    nanompx_srt_t *srt;
    nanompx_encoder_t *enc;
    nanompx_clock_t *clk;
    uint8_t *raw = NULL;
    size_t raw_len = 0, n_samp = 0, off = 0;
    int32_t *pcm = NULL;
    uint8_t pktbuf[65536];
    int i;

    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--host") && i + 1 < argc)
            host = argv[++i];
        else if (!strcmp(argv[i], "--port") && i + 1 < argc)
            port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-i") && i + 1 < argc)
            in_path = argv[++i];
        else if (!strcmp(argv[i], "--mode") && i + 1 < argc)
            mode = (!strcmp(argv[++i], "comp")) ? NANOMPX_MODE_COMPRESSED : NANOMPX_MODE_PCM;
        else if (!strcmp(argv[i], "--profile") && i + 1 < argc)
            profile = parse_profile(argv[++i]);
    }

    if (nanompx_srt_startup() != 0) {
        fprintf(stderr, "srt startup failed\n");
        return 1;
    }
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = NANOMPX_SRT_CALLER;
    cfg.host = host;
    cfg.port = port;
    cfg.sender = 1;
    cfg.latency_ms = 120;
    srt = nanompx_srt_open(&cfg);
    if (!srt) {
        fprintf(stderr, "srt open failed\n");
        nanompx_srt_cleanup();
        return 1;
    }

    if (read_all(in_path, &raw, &raw_len) != 0) {
        nanompx_srt_close(srt);
        nanompx_srt_cleanup();
        return 1;
    }
    n_samp = raw_len / 3;
    pcm = (int32_t *)malloc(n_samp * sizeof(int32_t));
    s24_to_i32(raw, raw_len, pcm, &n_samp);
    free(raw);

    enc = nanompx_encoder_create();
    clk = nanompx_clock_host_create();
    nanompx_encoder_set_clock(enc, clk);
    nanompx_encoder_set_mode(enc, mode);
    if (mode == NANOMPX_MODE_COMPRESSED)
        nanompx_encoder_set_profile(enc, profile);

    while (off < n_samp) {
        size_t chunk = n_samp - off;
        size_t wrote = 0;
        size_t p = 0;
        if (chunk > NANOMPX_DEFAULT_FRAME_SAMPLES)
            chunk = NANOMPX_DEFAULT_FRAME_SAMPLES;
        nanompx_encoder_push_pcm(enc, pcm + off, chunk, pktbuf, sizeof(pktbuf), &wrote);
        while (p < wrote) {
            nanompx_packet_hdr_t hdr;
            size_t plen;
            nanompx_hdr_unpack(&hdr, pktbuf + p);
            plen = NANOMPX_HDR_SIZE + hdr.payload_len;
            if (nanompx_srt_send(srt, pktbuf + p, plen) < 0) {
                fprintf(stderr, "srt send failed\n");
                break;
            }
            p += plen;
        }
        off += chunk;
    }
    {
        size_t wrote = 0, p = 0;
        nanompx_encoder_flush(enc, pktbuf, sizeof(pktbuf), &wrote);
        while (p < wrote) {
            nanompx_packet_hdr_t hdr;
            size_t plen;
            nanompx_hdr_unpack(&hdr, pktbuf + p);
            plen = NANOMPX_HDR_SIZE + hdr.payload_len;
            if (nanompx_srt_send(srt, pktbuf + p, plen) < 0) {
                fprintf(stderr, "srt send failed\n");
                break;
            }
            p += plen;
        }
    }

    nanompx_encoder_destroy(enc);
    nanompx_clock_destroy(clk);
    free(pcm);
    nanompx_srt_close(srt);
    nanompx_srt_cleanup();
    return 0;
}

static int cmd_srt_recv(int argc, char **argv)
{
    const char *host = "0.0.0.0";
    int port = 9000;
    int listener = 1;
    const char *out_path = "out.s24";
    nanompx_srt_config_t cfg;
    nanompx_srt_t *srt;
    nanompx_decoder_t *dec;
    nanompx_clock_t *clk;
    FILE *out;
    uint8_t buf[65536];
    int i, packets = 0;

    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--host") && i + 1 < argc)
            host = argv[++i];
        else if (!strcmp(argv[i], "--port") && i + 1 < argc)
            port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--listener"))
            listener = 1;
        else if (!strcmp(argv[i], "-o") && i + 1 < argc)
            out_path = argv[++i];
    }

    if (nanompx_srt_startup() != 0)
        return 1;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = listener ? NANOMPX_SRT_LISTENER : NANOMPX_SRT_CALLER;
    cfg.host = host;
    cfg.port = port;
    cfg.sender = 0;
    cfg.latency_ms = 120;
    srt = nanompx_srt_open(&cfg);
    if (!srt) {
        nanompx_srt_cleanup();
        return 1;
    }

    dec = nanompx_decoder_create();
    clk = nanompx_clock_host_create();
    nanompx_decoder_set_clock(dec, clk);
    nanompx_decoder_set_sfn_enabled(dec, 0);
    out = fopen(out_path, "wb");
    if (!out) {
        nanompx_srt_close(srt);
        nanompx_srt_cleanup();
        return 1;
    }

    fprintf(stderr, "listening for NanoMPX over SRT on %s:%d\n", host, port);
    while (packets < 5000) {
        int n = nanompx_srt_recv(srt, buf, sizeof(buf), 2000);
        int32_t tmp[NANOMPX_DEFAULT_FRAME_SAMPLES];
        size_t got = 0;
        if (n <= 0)
            break;
        if (nanompx_decoder_push_packet(dec, buf, (size_t)n) != NANOMPX_OK)
            continue;
        packets++;
        while (nanompx_decoder_pull_pcm(dec, tmp, NANOMPX_DEFAULT_FRAME_SAMPLES, &got) == NANOMPX_OK) {
            uint8_t s24[NANOMPX_DEFAULT_FRAME_SAMPLES * 3];
            i32_to_s24(tmp, got, s24);
            fwrite(s24, 1, got * 3, out);
        }
    }

    fclose(out);
    nanompx_decoder_destroy(dec);
    nanompx_clock_destroy(clk);
    nanompx_srt_close(srt);
    nanompx_srt_cleanup();
    fprintf(stderr, "received %d packets\n", packets);
    return 0;
}
#endif

int main(int argc, char **argv)
{
    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }
    if (!strcmp(argv[1], "encode"))
        return cmd_encode(argc - 2, argv + 2);
    if (!strcmp(argv[1], "decode"))
        return cmd_decode(argc - 2, argv + 2);
    if (!strcmp(argv[1], "roundtrip"))
        return cmd_roundtrip(argc - 2, argv + 2);
#if NANOMPX_WITH_SRT
    if (!strcmp(argv[1], "srt-send"))
        return cmd_srt_send(argc - 2, argv + 2);
    if (!strcmp(argv[1], "srt-recv"))
        return cmd_srt_recv(argc - 2, argv + 2);
#endif
    usage(argv[0]);
    return 1;
}
