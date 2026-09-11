/* SPDX-License-Identifier: MIT */
#define _GNU_SOURCE
#include "nanompx_clock.h"
#include "internal.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <time.h>

/* ---- Host clock ---- */

typedef struct {
    nanompx_clock_t base;
} host_clock_t;

static int host_now(nanompx_clock_t *c, int64_t *out_ns)
{
    (void)c;
    *out_ns = nanompx_host_time_ns();
    return 0;
}

static int host_locked(nanompx_clock_t *c)
{
    (void)c;
    return 1;
}

static void host_destroy(nanompx_clock_t *c)
{
    free(c);
}

static const nanompx_clock_vtbl_t host_vtbl = {
    host_now, host_locked, host_destroy
};

nanompx_clock_t *nanompx_clock_host_create(void)
{
    host_clock_t *c = (host_clock_t *)calloc(1, sizeof(*c));
    if (!c)
        return NULL;
    c->base.vtbl = &host_vtbl;
    return &c->base;
}

/* ---- Simulated clock ---- */

typedef struct {
    nanompx_clock_t base;
    int locked;
    int64_t origin_host_ns;
    int64_t origin_gps_ns;
} sim_clock_t;

static int sim_now(nanompx_clock_t *c, int64_t *out_ns)
{
    sim_clock_t *s = (sim_clock_t *)c;
    int64_t host = nanompx_host_time_ns();
    *out_ns = s->origin_gps_ns + (host - s->origin_host_ns);
    return 0;
}

static int sim_locked(nanompx_clock_t *c)
{
    return ((sim_clock_t *)c)->locked;
}

static void sim_destroy(nanompx_clock_t *c)
{
    free(c);
}

static const nanompx_clock_vtbl_t sim_vtbl = {
    sim_now, sim_locked, sim_destroy
};

nanompx_clock_t *nanompx_clock_sim_create(void)
{
    sim_clock_t *c = (sim_clock_t *)calloc(1, sizeof(*c));
    if (!c)
        return NULL;
    c->base.vtbl = &sim_vtbl;
    c->origin_host_ns = nanompx_host_time_ns();
    c->origin_gps_ns = 0;
    c->locked = 0;
    return &c->base;
}

void nanompx_clock_sim_set_locked(nanompx_clock_t *c, int locked)
{
    if (!c || c->vtbl != &sim_vtbl)
        return;
    ((sim_clock_t *)c)->locked = locked ? 1 : 0;
}

void nanompx_clock_sim_set_origin_ns(nanompx_clock_t *c, int64_t origin_host_ns)
{
    sim_clock_t *s;
    if (!c || c->vtbl != &sim_vtbl)
        return;
    s = (sim_clock_t *)c;
    s->origin_host_ns = origin_host_ns;
}

/* ---- GPS / PPS clock ---- */

typedef struct {
    nanompx_clock_t base;
    int locked;
    int64_t last_pps_host_ns;
    int64_t last_pps_gps_ns; /* GPS time at PPS (unix ns) */
    int64_t pending_unix_sec;
    int have_nmea;
    int have_pps;
    char nmea_path[256];
    char pps_path[256];
} gps_clock_t;

static int gps_now(nanompx_clock_t *c, int64_t *out_ns)
{
    gps_clock_t *g = (gps_clock_t *)c;
    int64_t host = nanompx_host_time_ns();
    if (!g->have_pps) {
        *out_ns = host;
        return 0;
    }
    *out_ns = g->last_pps_gps_ns + (host - g->last_pps_host_ns);
    return 0;
}

static int gps_locked(nanompx_clock_t *c)
{
    gps_clock_t *g = (gps_clock_t *)c;
    return g->locked && g->have_pps && g->have_nmea;
}

static void gps_destroy(nanompx_clock_t *c)
{
    free(c);
}

static const nanompx_clock_vtbl_t gps_vtbl = {
    gps_now, gps_locked, gps_destroy
};

nanompx_clock_t *nanompx_clock_gps_create(const char *nmea_path, const char *pps_path)
{
    gps_clock_t *c = (gps_clock_t *)calloc(1, sizeof(*c));
    if (!c)
        return NULL;
    c->base.vtbl = &gps_vtbl;
    if (nmea_path) {
        strncpy(c->nmea_path, nmea_path, sizeof(c->nmea_path) - 1);
    }
    if (pps_path) {
        strncpy(c->pps_path, pps_path, sizeof(c->pps_path) - 1);
    }
    /* Hardware open can be added by vendors; inject API works for tests. */
    return &c->base;
}

int nanompx_clock_gps_inject_pps(nanompx_clock_t *c, int64_t host_ns, int64_t gps_seconds)
{
    gps_clock_t *g;
    if (!c || c->vtbl != &gps_vtbl)
        return -1;
    g = (gps_clock_t *)c;
    g->last_pps_host_ns = host_ns;
    g->last_pps_gps_ns = gps_seconds * 1000000000LL;
    g->have_pps = 1;
    if (g->have_nmea)
        g->locked = 1;
    return 0;
}

int nanompx_clock_gps_inject_nmea_time(nanompx_clock_t *c, int64_t unix_seconds)
{
    gps_clock_t *g;
    if (!c || c->vtbl != &gps_vtbl)
        return -1;
    g = (gps_clock_t *)c;
    g->pending_unix_sec = unix_seconds;
    g->have_nmea = 1;
    if (g->have_pps)
        g->locked = 1;
    return 0;
}

static int parse_hhmmss(const char *p, int *h, int *m, int *s)
{
    if (strlen(p) < 6)
        return -1;
    *h = (p[0] - '0') * 10 + (p[1] - '0');
    *m = (p[2] - '0') * 10 + (p[3] - '0');
    *s = (p[4] - '0') * 10 + (p[5] - '0');
    return 0;
}

int nanompx_clock_gps_feed_nmea(nanompx_clock_t *c, const char *line)
{
    gps_clock_t *g = (gps_clock_t *)c;
    char buf[256];
    char *tok;
    char *save = NULL;
    int hour = 0, min = 0, sec = 0;
    int day = 1, month = 1, year = 1970;
    int have_time = 0, have_date = 0;

    if (!c || c->vtbl != &gps_vtbl || !line)
        return -1;
    (void)g;

    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;

    /* Strip checksum */
    {
        char *star = strchr(buf, '*');
        if (star)
            *star = 0;
    }

    tok = strtok_r(buf, ",", &save);
    if (!tok)
        return -1;

    if (strcmp(tok, "$GPRMC") == 0 || strcmp(tok, "$GNRMC") == 0) {
        /* time, status, ..., date */
        char *time_s = strtok_r(NULL, ",", &save);
        char *status = strtok_r(NULL, ",", &save);
        int i;
        if (!time_s || !status || status[0] != 'A')
            return 0;
        if (parse_hhmmss(time_s, &hour, &min, &sec) == 0)
            have_time = 1;
        for (i = 0; i < 6; i++)
            strtok_r(NULL, ",", &save); /* skip lat/lon/speed/course */
        {
            char *date_s = strtok_r(NULL, ",", &save);
            if (date_s && strlen(date_s) >= 6) {
                day = (date_s[0] - '0') * 10 + (date_s[1] - '0');
                month = (date_s[2] - '0') * 10 + (date_s[3] - '0');
                year = 2000 + (date_s[4] - '0') * 10 + (date_s[5] - '0');
                have_date = 1;
            }
        }
    } else if (strcmp(tok, "$GPZDA") == 0 || strcmp(tok, "$GNZDA") == 0) {
        char *time_s = strtok_r(NULL, ",", &save);
        char *d = strtok_r(NULL, ",", &save);
        char *mo = strtok_r(NULL, ",", &save);
        char *y = strtok_r(NULL, ",", &save);
        if (time_s && parse_hhmmss(time_s, &hour, &min, &sec) == 0)
            have_time = 1;
        if (d && mo && y) {
            day = atoi(d);
            month = atoi(mo);
            year = atoi(y);
            have_date = 1;
        }
    } else {
        return 0;
    }

    if (have_time && have_date) {
        /* Approximate unix time without full timegm portability: use timegm if available */
        struct tm tm;
        time_t t;
        memset(&tm, 0, sizeof(tm));
        tm.tm_year = year - 1900;
        tm.tm_mon = month - 1;
        tm.tm_mday = day;
        tm.tm_hour = hour;
        tm.tm_min = min;
        tm.tm_sec = sec;
#if defined(_GNU_SOURCE) || defined(__USE_BSD)
        t = timegm(&tm);
#else
        /* Fallback: treat as local — vendors should use timegm on POSIX */
        t = mktime(&tm);
#endif
        if (t >= 0)
            nanompx_clock_gps_inject_nmea_time(c, (int64_t)t);
    }
    return 0;
}
