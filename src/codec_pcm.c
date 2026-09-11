/* SPDX-License-Identifier: MIT */
#include "internal.h"

/**
 * Pack signed 24-bit PCM little-endian (3 bytes/sample).
 * Input samples are int32 with meaningful low 24 bits (sign-extended).
 */
size_t nanompx_pcm_encode(const int32_t *samples, size_t count,
                          uint8_t *dst, size_t dst_cap)
{
    size_t i;
    size_t need = count * 3;
    if (!samples || !dst || need > dst_cap)
        return 0;
    for (i = 0; i < count; i++) {
        int32_t s = samples[i];
        /* Clamp to 24-bit */
        if (s > 0x7FFFFF)
            s = 0x7FFFFF;
        if (s < -0x800000)
            s = -0x800000;
        dst[i * 3 + 0] = (uint8_t)(s & 0xff);
        dst[i * 3 + 1] = (uint8_t)((s >> 8) & 0xff);
        dst[i * 3 + 2] = (uint8_t)((s >> 16) & 0xff);
    }
    return need;
}

size_t nanompx_pcm_decode(const uint8_t *src, size_t src_len,
                          int32_t *samples, size_t max_samples)
{
    size_t count = src_len / 3;
    size_t i;
    if (!src || !samples)
        return 0;
    if (count > max_samples)
        count = max_samples;
    for (i = 0; i < count; i++) {
        int32_t s = (int32_t)src[i * 3 + 0] |
                    ((int32_t)src[i * 3 + 1] << 8) |
                    ((int32_t)src[i * 3 + 2] << 16);
        /* Sign-extend 24 -> 32 */
        if (s & 0x800000)
            s |= ~0xFFFFFF;
        samples[i] = s;
    }
    return count;
}
