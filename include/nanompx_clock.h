/* SPDX-License-Identifier: MIT */
/**
 * NanoMPX clock abstraction for SFN synchronization.
 *
 * Copyright (c) 2026 Kvarta
 */

#ifndef NANOMPX_CLOCK_H
#define NANOMPX_CLOCK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct nanompx_clock nanompx_clock_t;

typedef struct nanompx_clock_vtbl {
    int (*now_ns)(nanompx_clock_t *c, int64_t *out_ns);
    int (*locked)(nanompx_clock_t *c);
    void (*destroy)(nanompx_clock_t *c);
} nanompx_clock_vtbl_t;

struct nanompx_clock {
    const nanompx_clock_vtbl_t *vtbl;
};

static inline int nanompx_clock_now_ns(nanompx_clock_t *c, int64_t *out_ns)
{
    if (!c || !c->vtbl || !c->vtbl->now_ns || !out_ns)
        return -1;
    return c->vtbl->now_ns(c, out_ns);
}

static inline int nanompx_clock_locked(nanompx_clock_t *c)
{
    if (!c || !c->vtbl || !c->vtbl->locked)
        return 0;
    return c->vtbl->locked(c);
}

static inline void nanompx_clock_destroy(nanompx_clock_t *c)
{
    if (c && c->vtbl && c->vtbl->destroy)
        c->vtbl->destroy(c);
}

/**
 * Host monotonic/realtime clock (not SFN-grade). Always "locked".
 * Useful for non-SFN testing.
 */
nanompx_clock_t *nanompx_clock_host_create(void);

/**
 * Simulated PPS clock for CI / lab. Advances with host time after start,
 * reports locked once nanompx_clock_sim_set_locked(..., 1) is called.
 * Epoch of "GPS time" starts at create (t=0) unless set via set_origin.
 */
nanompx_clock_t *nanompx_clock_sim_create(void);
void nanompx_clock_sim_set_locked(nanompx_clock_t *c, int locked);
void nanompx_clock_sim_set_origin_ns(nanompx_clock_t *c, int64_t origin_host_ns);

/**
 * GPS NMEA + 1PPS disciplined clock.
 *
 * @param nmea_path  serial device for NMEA (e.g. "/dev/ttyUSB0"), or NULL
 * @param pps_path   PPS character device (e.g. "/dev/pps0"), or NULL
 *
 * Without hardware, create still succeeds but stays unlocked until
 * nanompx_clock_gps_inject_* is used (test/bench path).
 */
nanompx_clock_t *nanompx_clock_gps_create(const char *nmea_path, const char *pps_path);

/** Inject a PPS edge at capture host time (for tests / userspace PPS). */
int nanompx_clock_gps_inject_pps(nanompx_clock_t *c, int64_t host_ns, int64_t gps_seconds);

/** Inject NMEA RMC/ZDA-derived UTC second (whole seconds since Unix epoch). */
int nanompx_clock_gps_inject_nmea_time(nanompx_clock_t *c, int64_t unix_seconds);

/** Feed a NMEA sentence line (without or with CRLF). */
int nanompx_clock_gps_feed_nmea(nanompx_clock_t *c, const char *line);

#ifdef __cplusplus
}
#endif

#endif /* NANOMPX_CLOCK_H */
