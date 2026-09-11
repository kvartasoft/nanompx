/* SPDX-License-Identifier: MIT */
#include "internal.h"

#include <stdio.h>
#include <time.h>

int64_t nanompx_host_time_ns(void)
{
    struct timespec ts;
#if defined(CLOCK_REALTIME)
    clock_gettime(CLOCK_REALTIME, &ts);
#else
    ts.tv_sec = time(NULL);
    ts.tv_nsec = 0;
#endif
    return (int64_t)ts.tv_sec * 1000000000LL + (int64_t)ts.tv_nsec;
}

uint16_t nanompx_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    size_t i;
    int b;
    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (b = 0; b < 8; b++) {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void wr_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)((v >> 8) & 0xff);
}

static void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)((v >> 8) & 0xff);
    p[2] = (uint8_t)((v >> 16) & 0xff);
    p[3] = (uint8_t)((v >> 24) & 0xff);
}

static void wr_u64(uint8_t *p, uint64_t v)
{
    int i;
    for (i = 0; i < 8; i++)
        p[i] = (uint8_t)((v >> (8 * i)) & 0xff);
}

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t rd_u64(const uint8_t *p)
{
    uint64_t v = 0;
    int i;
    for (i = 0; i < 8; i++)
        v |= (uint64_t)p[i] << (8 * i);
    return v;
}

int nanompx_hdr_pack(uint8_t *dst, const nanompx_packet_hdr_t *hdr)
{
    if (!dst || !hdr)
        return NANOMPX_ERR_INVALID;
    wr_u32(dst + 0, hdr->magic);
    dst[4] = hdr->version;
    dst[5] = hdr->mode;
    dst[6] = hdr->profile;
    dst[7] = hdr->flags;
    wr_u32(dst + 8, hdr->seq);
    wr_u64(dst + 12, hdr->sample_index);
    wr_u64(dst + 20, hdr->capture_time_ns);
    wr_u16(dst + 28, hdr->payload_len);
    wr_u16(dst + 30, nanompx_crc16(dst, 30));
    return NANOMPX_HDR_SIZE;
}

int nanompx_hdr_unpack(nanompx_packet_hdr_t *hdr, const uint8_t *src)
{
    uint16_t expect;
    if (!hdr || !src)
        return NANOMPX_ERR_INVALID;
    hdr->magic = rd_u32(src + 0);
    hdr->version = src[4];
    hdr->mode = src[5];
    hdr->profile = src[6];
    hdr->flags = src[7];
    hdr->seq = rd_u32(src + 8);
    hdr->sample_index = rd_u64(src + 12);
    hdr->capture_time_ns = rd_u64(src + 20);
    hdr->payload_len = rd_u16(src + 28);
    hdr->hdr_crc = rd_u16(src + 30);
    expect = nanompx_crc16(src, 30);
    if (expect != hdr->hdr_crc)
        return NANOMPX_ERR_CRC;
    if (hdr->magic != NANOMPX_MAGIC)
        return NANOMPX_ERR_INVALID;
    if (hdr->version != NANOMPX_PROTOCOL_VERSION)
        return NANOMPX_ERR_VERSION;
    return NANOMPX_OK;
}

int nanompx_packet_validate(const uint8_t *packet, size_t packet_len)
{
    nanompx_packet_hdr_t hdr;
    int rc;
    if (!packet || packet_len < NANOMPX_HDR_SIZE)
        return NANOMPX_ERR_INVALID;
    rc = nanompx_hdr_unpack(&hdr, packet);
    if (rc != NANOMPX_OK)
        return rc;
    if (packet_len < (size_t)NANOMPX_HDR_SIZE + hdr.payload_len)
        return NANOMPX_ERR_INVALID;
    return NANOMPX_OK;
}

const char *nanompx_strerror(int err)
{
    switch (err) {
    case NANOMPX_OK:
        return "ok";
    case NANOMPX_ERR_INVALID:
        return "invalid argument";
    case NANOMPX_ERR_NOMEM:
        return "out of memory";
    case NANOMPX_ERR_AGAIN:
        return "try again";
    case NANOMPX_ERR_OVERFLOW:
        return "buffer overflow";
    case NANOMPX_ERR_CRC:
        return "crc mismatch";
    case NANOMPX_ERR_VERSION:
        return "unsupported version";
    case NANOMPX_ERR_UNSUPPORTED:
        return "unsupported";
    case NANOMPX_ERR_IO:
        return "io error";
    default:
        return "unknown error";
    }
}

const char *nanompx_version_string(void)
{
    return NANOMPX_VERSION_STR;
}
