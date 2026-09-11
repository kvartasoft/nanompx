/* SPDX-License-Identifier: MIT */
/**
 * Optional SRT transport for NanoMPX packets.
 */

#include "nanompx_srt.h"

#if NANOMPX_WITH_SRT

#include <srt/srt.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <arpa/inet.h>
#include <netinet/in.h>

struct nanompx_srt {
    SRTSOCKET sock;
    SRTSOCKET accepted; /* listener accepted peer */
    int is_listener;
    int is_sender;
};

int nanompx_srt_startup(void)
{
    if (srt_startup() != 0)
        return -8;
    return 0;
}

void nanompx_srt_cleanup(void)
{
    srt_cleanup();
}

static int apply_common(SRTSOCKET s, const nanompx_srt_config_t *cfg)
{
    int yes = 1;
    if (cfg->latency_ms > 0) {
        int lat = cfg->latency_ms;
        if (srt_setsockopt(s, 0, SRTO_LATENCY, &lat, sizeof(lat)) < 0)
            return -1;
    }
    if (cfg->passphrase && cfg->passphrase[0]) {
        size_t n = strlen(cfg->passphrase);
        int pbkeylen = 16;
        if (n < 10 || n > 79)
            return -1;
        if (srt_setsockopt(s, 0, SRTO_PASSPHRASE, cfg->passphrase, (int)n) < 0)
            return -1;
        if (srt_setsockopt(s, 0, SRTO_PBKEYLEN, &pbkeylen, sizeof(pbkeylen)) < 0)
            return -1;
    }
    if (cfg->sender) {
        if (srt_setsockopt(s, 0, SRTO_SENDER, &yes, sizeof(yes)) < 0)
            return -1;
    }
    return 0;
}

nanompx_srt_t *nanompx_srt_open(const nanompx_srt_config_t *cfg)
{
    nanompx_srt_t *s;
    struct sockaddr_in sa;
    SRTSOCKET sock;

    if (!cfg || !cfg->host || cfg->port <= 0)
        return NULL;

    s = (nanompx_srt_t *)calloc(1, sizeof(*s));
    if (!s)
        return NULL;
    s->is_listener = (cfg->mode == NANOMPX_SRT_LISTENER);
    s->is_sender = cfg->sender ? 1 : 0;
    s->accepted = SRT_INVALID_SOCK;

    sock = srt_create_socket();
    if (sock == SRT_INVALID_SOCK) {
        free(s);
        return NULL;
    }
    if (apply_common(sock, cfg) != 0) {
        srt_close(sock);
        free(s);
        return NULL;
    }

    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port = htons((uint16_t)cfg->port);
    if (inet_pton(AF_INET, cfg->host, &sa.sin_addr) != 1) {
        srt_close(sock);
        free(s);
        return NULL;
    }

    if (s->is_listener) {
        int no = 0;
        if (srt_setsockopt(sock, 0, SRTO_RCVSYN, &no, sizeof(no)) < 0) {
            srt_close(sock);
            free(s);
            return NULL;
        }
        if (srt_bind(sock, (struct sockaddr *)&sa, sizeof(sa)) == SRT_ERROR) {
            srt_close(sock);
            free(s);
            return NULL;
        }
        if (srt_listen(sock, 1) == SRT_ERROR) {
            srt_close(sock);
            free(s);
            return NULL;
        }
        s->sock = sock;
    } else {
        if (srt_connect(sock, (struct sockaddr *)&sa, sizeof(sa)) == SRT_ERROR) {
            srt_close(sock);
            free(s);
            return NULL;
        }
        s->sock = sock;
    }
    return s;
}

static SRTSOCKET active_sock(nanompx_srt_t *s)
{
    if (s->is_listener) {
        if (s->accepted == SRT_INVALID_SOCK) {
            struct sockaddr_storage peer;
            int peerlen = sizeof(peer);
            s->accepted = srt_accept(s->sock, (struct sockaddr *)&peer, &peerlen);
        }
        return s->accepted;
    }
    return s->sock;
}

void nanompx_srt_close(nanompx_srt_t *s)
{
    if (!s)
        return;
    if (s->accepted != SRT_INVALID_SOCK)
        srt_close(s->accepted);
    if (s->sock != SRT_INVALID_SOCK)
        srt_close(s->sock);
    free(s);
}

int nanompx_srt_send(nanompx_srt_t *s, const uint8_t *pkt, size_t len)
{
    SRTSOCKET sock;
    int ret;
    if (!s || !pkt || len == 0)
        return -1;
    sock = active_sock(s);
    if (sock == SRT_INVALID_SOCK)
        return -1;
    ret = srt_sendmsg(sock, (const char *)pkt, (int)len, -1, 0);
    if (ret == SRT_ERROR)
        return -8;
    return ret;
}

int nanompx_srt_recv(nanompx_srt_t *s, uint8_t *buf, size_t cap, int timeout_ms)
{
    SRTSOCKET sock;
    int ret;
    if (!s || !buf || cap == 0)
        return -1;
    sock = active_sock(s);
    if (sock == SRT_INVALID_SOCK)
        return -1;
    if (timeout_ms >= 0) {
        int to = timeout_ms;
        srt_setsockopt(sock, 0, SRTO_RCVTIMEO, &to, sizeof(to));
    }
    ret = srt_recvmsg(sock, (char *)buf, (int)cap);
    if (ret == SRT_ERROR)
        return 0;
    return ret;
}

#endif /* NANOMPX_WITH_SRT */
