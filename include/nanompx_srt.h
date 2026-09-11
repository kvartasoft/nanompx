/* SPDX-License-Identifier: MIT */
/**
 * Optional NanoMPX SRT transport helpers.
 * Build with -DNANOMPX_WITH_SRT=ON (requires libsrt).
 *
 * Copyright (c) 2026 Kvarta
 */

#ifndef NANOMPX_SRT_H
#define NANOMPX_SRT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NANOMPX_WITH_SRT
#define NANOMPX_WITH_SRT 0
#endif

typedef struct nanompx_srt nanompx_srt_t;

typedef enum nanompx_srt_mode {
    NANOMPX_SRT_CALLER = 0,
    NANOMPX_SRT_LISTENER = 1
} nanompx_srt_mode_t;

typedef struct nanompx_srt_config {
    nanompx_srt_mode_t mode;
    const char *host;       /**< listen bind or connect host */
    int port;
    const char *passphrase; /**< optional, 10–79 chars */
    int latency_ms;         /**< SRT latency; 0 = library default */
    int sender;             /**< non-zero if this side sends */
} nanompx_srt_config_t;

#if NANOMPX_WITH_SRT

int nanompx_srt_startup(void);
void nanompx_srt_cleanup(void);

nanompx_srt_t *nanompx_srt_open(const nanompx_srt_config_t *cfg);
void nanompx_srt_close(nanompx_srt_t *s);

/** Send one complete NanoMPX packet. Returns bytes sent or negative error. */
int nanompx_srt_send(nanompx_srt_t *s, const uint8_t *pkt, size_t len);

/** Receive one packet into buf. Returns length or negative / 0 on timeout. */
int nanompx_srt_recv(nanompx_srt_t *s, uint8_t *buf, size_t cap, int timeout_ms);

#else

static inline int nanompx_srt_startup(void) { return -7; }
static inline void nanompx_srt_cleanup(void) {}
static inline nanompx_srt_t *nanompx_srt_open(const nanompx_srt_config_t *cfg)
{
    (void)cfg;
    return NULL;
}
static inline void nanompx_srt_close(nanompx_srt_t *s) { (void)s; }
static inline int nanompx_srt_send(nanompx_srt_t *s, const uint8_t *pkt, size_t len)
{
    (void)s;
    (void)pkt;
    (void)len;
    return -7;
}
static inline int nanompx_srt_recv(nanompx_srt_t *s, uint8_t *buf, size_t cap, int timeout_ms)
{
    (void)s;
    (void)buf;
    (void)cap;
    (void)timeout_ms;
    return -7;
}

#endif /* NANOMPX_WITH_SRT */

#ifdef __cplusplus
}
#endif

#endif /* NANOMPX_SRT_H */
