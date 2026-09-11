/* SPDX-License-Identifier: MIT */
/**
 * NanoMPX — open FM composite (MPX) over IP codec
 *
 * Copyright (c) 2026 Kvarta
 */

#ifndef NANOMPX_H
#define NANOMPX_H

#include <stddef.h>
#include <stdint.h>

#include "nanompx_clock.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NANOMPX_SAMPLE_RATE     192000
#define NANOMPX_PROTOCOL_VERSION 1

/** Wire magic: ASCII "NMPX" little-endian */
#define NANOMPX_MAGIC 0x58504D4Eu

typedef enum nanompx_mode {
    NANOMPX_MODE_PCM = 0,
    NANOMPX_MODE_COMPRESSED = 1
} nanompx_mode_t;

typedef enum nanompx_profile {
    NANOMPX_PROFILE_NONE = 0, /**< PCM / unused */
    NANOMPX_PROFILE_L = 1,    /**< ~1600 kbps */
    NANOMPX_PROFILE_M = 2,    /**< ~960 kbps */
    NANOMPX_PROFILE_S = 3     /**< ~640 kbps */
} nanompx_profile_t;

typedef enum nanompx_err {
    NANOMPX_OK = 0,
    NANOMPX_ERR_INVALID = -1,
    NANOMPX_ERR_NOMEM = -2,
    NANOMPX_ERR_AGAIN = -3,      /**< need more input / not due yet */
    NANOMPX_ERR_OVERFLOW = -4,
    NANOMPX_ERR_CRC = -5,
    NANOMPX_ERR_VERSION = -6,
    NANOMPX_ERR_UNSUPPORTED = -7,
    NANOMPX_ERR_IO = -8
} nanompx_err_t;

enum nanompx_flag {
    NANOMPX_FLAG_KEYFRAME = 1u << 0,
    NANOMPX_FLAG_DISCONTINUITY = 1u << 1,
    NANOMPX_FLAG_FEC = 1u << 2
};

/** Packed wire header (little-endian). Total 32 bytes. */
typedef struct nanompx_packet_hdr {
    uint32_t magic;
    uint8_t  version;
    uint8_t  mode;
    uint8_t  profile;
    uint8_t  flags;
    uint32_t seq;
    uint64_t sample_index;   /**< first sample index of this frame */
    uint64_t capture_time_ns;/**< GPS-tied capture time of first sample */
    uint16_t payload_len;
    uint16_t hdr_crc;        /**< CRC-16/CCITT over first 28 bytes */
} nanompx_packet_hdr_t;

#define NANOMPX_HDR_SIZE 32
#define NANOMPX_MAX_PAYLOAD 65535
#define NANOMPX_DEFAULT_FRAME_SAMPLES 1920 /**< 10 ms @ 192 kHz */

typedef struct nanompx_stats {
    uint64_t packets_in;
    uint64_t packets_out;
    uint64_t packets_lost;
    uint64_t packets_reorder;
    uint64_t crc_errors;
    uint64_t samples_encoded;
    uint64_t samples_decoded;
    uint32_t buffer_depth_packets;
    int      clock_locked;
} nanompx_stats_t;

typedef struct nanompx_encoder nanompx_encoder_t;
typedef struct nanompx_decoder nanompx_decoder_t;

/* --- Encoder --- */

nanompx_encoder_t *nanompx_encoder_create(void);
void nanompx_encoder_destroy(nanompx_encoder_t *enc);

int nanompx_encoder_set_mode(nanompx_encoder_t *enc, nanompx_mode_t mode);
int nanompx_encoder_set_profile(nanompx_encoder_t *enc, nanompx_profile_t profile);
int nanompx_encoder_set_clock(nanompx_encoder_t *enc, nanompx_clock_t *clock);
int nanompx_encoder_set_frame_samples(nanompx_encoder_t *enc, unsigned frame_samples);
/**
 * Periodic KEYFRAME interval in frames (compressed). 0 = only on discontinuity.
 * Default: 50 (~0.5 s at 10 ms frames) so decoders can recover after loss.
 */
int nanompx_encoder_set_keyframe_interval(nanompx_encoder_t *enc, unsigned frames);

/**
 * Push interleaved mono MPX PCM as signed 24-bit integers in the low 24 bits
 * of each int32 (sign-extended). Produces zero or more framed packets into
 * out_buf. Returns bytes written, or negative nanompx_err_t.
 */
int nanompx_encoder_push_pcm(nanompx_encoder_t *enc,
                             const int32_t *samples,
                             size_t count,
                             uint8_t *out_buf,
                             size_t out_cap,
                             size_t *out_bytes);

/**
 * Flush pending samples (zero-pad to a full frame if needed). Call at end of stream.
 */
int nanompx_encoder_flush(nanompx_encoder_t *enc,
                          uint8_t *out_buf,
                          size_t out_cap,
                          size_t *out_bytes);

void nanompx_encoder_get_stats(const nanompx_encoder_t *enc, nanompx_stats_t *out);

/** Force KEYFRAME on the next packet (profile change / stream recovery). */
int nanompx_encoder_signal_discontinuity(nanompx_encoder_t *enc);

/** Compressed codec algorithmic delay in samples (0 for PCM). */
unsigned nanompx_compressed_delay_samples(void);

/**
 * FEC sizing hint for future parity packets. Returns 0 in v1 (FEC not emitted).
 */
size_t nanompx_fec_suggest_parity(size_t payload_len, unsigned overhead_percent);

/* --- Decoder --- */

nanompx_decoder_t *nanompx_decoder_create(void);
void nanompx_decoder_destroy(nanompx_decoder_t *dec);

int nanompx_decoder_set_clock(nanompx_decoder_t *dec, nanompx_clock_t *clock);
/** Playout delay = network_delay_ns + user_offset_ns (same on all SFN sites). */
int nanompx_decoder_set_playout_delay(nanompx_decoder_t *dec,
                                      int64_t network_delay_ns,
                                      int64_t user_offset_ns);
/** If disable_sfn != 0, pull_pcm returns ASAP (ignore absolute time). */
int nanompx_decoder_set_sfn_enabled(nanompx_decoder_t *dec, int enabled);

/**
 * Push one complete NanoMPX packet (header + payload).
 * Returns NANOMPX_OK or error.
 */
int nanompx_decoder_push_packet(nanompx_decoder_t *dec,
                                const uint8_t *packet,
                                size_t packet_len);

/**
 * Pull decoded PCM when due (SFN) or when available.
 * Writes up to max_samples. Sets *out_count.
 * Returns NANOMPX_OK, NANOMPX_ERR_AGAIN, or other error.
 */
int nanompx_decoder_pull_pcm(nanompx_decoder_t *dec,
                             int32_t *samples,
                             size_t max_samples,
                             size_t *out_count);

void nanompx_decoder_get_stats(const nanompx_decoder_t *dec, nanompx_stats_t *out);

/* --- Framing helpers (for custom transports) --- */

uint16_t nanompx_crc16(const uint8_t *data, size_t len);
int nanompx_hdr_pack(uint8_t *dst, const nanompx_packet_hdr_t *hdr);
int nanompx_hdr_unpack(nanompx_packet_hdr_t *hdr, const uint8_t *src);
int nanompx_packet_validate(const uint8_t *packet, size_t packet_len);

/** Target bitrate (bps) for a compressed profile. */
uint32_t nanompx_profile_bitrate(nanompx_profile_t profile);

const char *nanompx_strerror(int err);
const char *nanompx_version_string(void);

#ifdef __cplusplus
}
#endif

#endif /* NANOMPX_H */
