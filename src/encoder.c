/* SPDX-License-Identifier: MIT */
#include "internal.h"

struct nanompx_encoder {
    nanompx_mode_t mode;
    nanompx_profile_t profile;
    nanompx_clock_t *clock;
    unsigned frame_samples;
    uint32_t seq;
    uint64_t sample_index;
    int32_t *pending;
    size_t pending_count;
    size_t pending_cap;
    nanompx_comp_enc_t *comp;
    nanompx_stats_t stats;
    int need_keyframe;
};

nanompx_encoder_t *nanompx_encoder_create(void)
{
    nanompx_encoder_t *enc = (nanompx_encoder_t *)calloc(1, sizeof(*enc));
    if (!enc)
        return NULL;
    enc->mode = NANOMPX_MODE_PCM;
    enc->profile = NANOMPX_PROFILE_L;
    enc->frame_samples = NANOMPX_DEFAULT_FRAME_SAMPLES;
    enc->need_keyframe = 1;
    enc->comp = nanompx_comp_enc_create(enc->profile);
    if (!enc->comp) {
        free(enc);
        return NULL;
    }
    return enc;
}

void nanompx_encoder_destroy(nanompx_encoder_t *enc)
{
    if (!enc)
        return;
    nanompx_comp_enc_destroy(enc->comp);
    free(enc->pending);
    free(enc);
}

int nanompx_encoder_set_mode(nanompx_encoder_t *enc, nanompx_mode_t mode)
{
    if (!enc || (mode != NANOMPX_MODE_PCM && mode != NANOMPX_MODE_COMPRESSED))
        return NANOMPX_ERR_INVALID;
    if (enc->mode != mode) {
        enc->mode = mode;
        enc->need_keyframe = 1;
    }
    return NANOMPX_OK;
}

int nanompx_encoder_set_profile(nanompx_encoder_t *enc, nanompx_profile_t profile)
{
    if (!enc)
        return NANOMPX_ERR_INVALID;
    if (profile < NANOMPX_PROFILE_L || profile > NANOMPX_PROFILE_S)
        return NANOMPX_ERR_INVALID;
    if (enc->profile != profile) {
        enc->profile = profile;
        enc->need_keyframe = 1;
        nanompx_comp_enc_set_profile(enc->comp, profile);
    }
    return NANOMPX_OK;
}

int nanompx_encoder_set_clock(nanompx_encoder_t *enc, nanompx_clock_t *clock)
{
    if (!enc)
        return NANOMPX_ERR_INVALID;
    enc->clock = clock;
    return NANOMPX_OK;
}

int nanompx_encoder_set_frame_samples(nanompx_encoder_t *enc, unsigned frame_samples)
{
    if (!enc || frame_samples == 0 || frame_samples > 65535)
        return NANOMPX_ERR_INVALID;
    enc->frame_samples = frame_samples;
    return NANOMPX_OK;
}

static int ensure_pending(nanompx_encoder_t *enc, size_t need)
{
    if (enc->pending_cap >= need)
        return 0;
    {
        int32_t *p = (int32_t *)realloc(enc->pending, need * sizeof(int32_t));
        if (!p)
            return -1;
        enc->pending = p;
        enc->pending_cap = need;
    }
    return 0;
}

static int emit_frame(nanompx_encoder_t *enc, const int32_t *frame, unsigned n,
                      uint8_t *out_buf, size_t out_cap, size_t *written)
{
    nanompx_packet_hdr_t hdr;
    uint8_t *payload;
    size_t payload_len = 0;
    int64_t now = 0;
    int rc;

    if (out_cap < NANOMPX_HDR_SIZE)
        return NANOMPX_ERR_OVERFLOW;

    memset(&hdr, 0, sizeof(hdr));
    hdr.magic = NANOMPX_MAGIC;
    hdr.version = NANOMPX_PROTOCOL_VERSION;
    hdr.mode = (uint8_t)enc->mode;
    hdr.profile = (enc->mode == NANOMPX_MODE_COMPRESSED) ? (uint8_t)enc->profile
                                                         : (uint8_t)NANOMPX_PROFILE_NONE;
    hdr.flags = 0;
    if (enc->need_keyframe) {
            hdr.flags |= NANOMPX_FLAG_KEYFRAME;
            hdr.flags |= NANOMPX_FLAG_DISCONTINUITY;
            enc->need_keyframe = 0;
    }
    hdr.seq = enc->seq++;
    hdr.sample_index = enc->sample_index;

    if (enc->clock && nanompx_clock_now_ns(enc->clock, &now) == 0)
        hdr.capture_time_ns = (uint64_t)now;
    else
        hdr.capture_time_ns = (uint64_t)nanompx_host_time_ns();

    payload = out_buf + NANOMPX_HDR_SIZE;
    if (enc->mode == NANOMPX_MODE_PCM) {
        payload_len = nanompx_pcm_encode(frame, n, payload, out_cap - NANOMPX_HDR_SIZE);
        if (payload_len == 0)
            return NANOMPX_ERR_OVERFLOW;
    } else {
        size_t clen = 0;
        rc = nanompx_comp_encode(enc->comp, frame, n, payload,
                                 out_cap - NANOMPX_HDR_SIZE, &clen,
                                 (hdr.flags & NANOMPX_FLAG_KEYFRAME) != 0);
        if (rc != NANOMPX_OK)
            return rc;
        payload_len = clen;
    }

    if (payload_len > 0xffff)
        return NANOMPX_ERR_OVERFLOW;
    hdr.payload_len = (uint16_t)payload_len;
    nanompx_hdr_pack(out_buf, &hdr);

    *written = NANOMPX_HDR_SIZE + payload_len;
    enc->sample_index += n;
    enc->stats.packets_out++;
    enc->stats.samples_encoded += n;
    enc->stats.clock_locked = enc->clock ? nanompx_clock_locked(enc->clock) : 0;
    return NANOMPX_OK;
}

int nanompx_encoder_push_pcm(nanompx_encoder_t *enc,
                             const int32_t *samples,
                             size_t count,
                             uint8_t *out_buf,
                             size_t out_cap,
                             size_t *out_bytes)
{
    size_t total = 0;
    size_t in_off = 0;

    if (!enc || !samples || !out_buf || !out_bytes)
        return NANOMPX_ERR_INVALID;
    *out_bytes = 0;

    if (ensure_pending(enc, enc->pending_count + count + enc->frame_samples) != 0)
        return NANOMPX_ERR_NOMEM;

    memcpy(enc->pending + enc->pending_count, samples, count * sizeof(int32_t));
    enc->pending_count += count;

    while (enc->pending_count >= enc->frame_samples) {
        size_t wrote = 0;
        int rc = emit_frame(enc, enc->pending, enc->frame_samples,
                            out_buf + total, out_cap - total, &wrote);
        if (rc != NANOMPX_OK)
            return rc;
        total += wrote;
        enc->pending_count -= enc->frame_samples;
        memmove(enc->pending, enc->pending + enc->frame_samples,
                enc->pending_count * sizeof(int32_t));
        (void)in_off;
    }

    *out_bytes = total;
    return NANOMPX_OK;
}

void nanompx_encoder_get_stats(const nanompx_encoder_t *enc, nanompx_stats_t *out)
{
    if (!enc || !out)
        return;
    *out = enc->stats;
}

int nanompx_encoder_signal_discontinuity(nanompx_encoder_t *enc)
{
    if (!enc)
        return NANOMPX_ERR_INVALID;
    enc->need_keyframe = 1;
    return NANOMPX_OK;
}

size_t nanompx_fec_suggest_parity(size_t payload_len, unsigned overhead_percent)
{
    (void)payload_len;
    (void)overhead_percent;
    return 0; /* FEC reserved — dual-stream + seq stats cover v1 recovery */
}
