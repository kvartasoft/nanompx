/* SPDX-License-Identifier: MIT */
#ifndef NANOMPX_INTERNAL_H
#define NANOMPX_INTERNAL_H

#include "nanompx.h"

#include <stdlib.h>
#include <string.h>

#define NANOMPX_VERSION_STR "0.1.0"

/* PCM codec */
size_t nanompx_pcm_encode(const int32_t *samples, size_t count,
                          uint8_t *dst, size_t dst_cap);
size_t nanompx_pcm_decode(const uint8_t *src, size_t src_len,
                          int32_t *samples, size_t max_samples);

/* Compressed codec */
typedef struct nanompx_comp_enc nanompx_comp_enc_t;
typedef struct nanompx_comp_dec nanompx_comp_dec_t;

nanompx_comp_enc_t *nanompx_comp_enc_create(nanompx_profile_t profile);
void nanompx_comp_enc_destroy(nanompx_comp_enc_t *e);
int nanompx_comp_enc_set_profile(nanompx_comp_enc_t *e, nanompx_profile_t profile);
/** Algorithmic delay (samples) of compressed encode→decode path. */
unsigned nanompx_comp_codec_delay_samples(void);
int nanompx_comp_encode(nanompx_comp_enc_t *e,
                        const int32_t *samples, size_t count,
                        uint8_t *dst, size_t dst_cap, size_t *out_len,
                        int keyframe, uint64_t sample_base);

nanompx_comp_dec_t *nanompx_comp_dec_create(void);
void nanompx_comp_dec_destroy(nanompx_comp_dec_t *d);
void nanompx_comp_dec_reset(nanompx_comp_dec_t *d);
int nanompx_comp_decode(nanompx_comp_dec_t *d,
                        nanompx_profile_t profile,
                        const uint8_t *src, size_t src_len,
                        int32_t *samples, size_t max_samples, size_t *out_count,
                        int keyframe, uint64_t sample_base);

/* Host time helper */
int64_t nanompx_host_time_ns(void);

#endif /* NANOMPX_INTERNAL_H */
