/* SPDX-License-Identifier: MIT */
#include "internal.h"

#define DEC_Q_MAX 256

typedef struct queued_frame {
    nanompx_packet_hdr_t hdr;
    int32_t *pcm;
    size_t count;
    size_t pos;
    int active;
} queued_frame_t;

struct nanompx_decoder {
    nanompx_clock_t *clock;
    int64_t network_delay_ns;
    int64_t user_offset_ns;
    int sfn_enabled;
    nanompx_comp_dec_t *comp;
    queued_frame_t q[DEC_Q_MAX];
    int q_head;
    int q_count;
    uint32_t expect_seq;
    int have_seq;
    nanompx_profile_t last_profile;
    int wait_keyframe; /* compressed: discard until KEYFRAME after loss */
    nanompx_stats_t stats;
};

nanompx_decoder_t *nanompx_decoder_create(void)
{
    nanompx_decoder_t *dec = (nanompx_decoder_t *)calloc(1, sizeof(*dec));
    if (!dec)
        return NULL;
    dec->sfn_enabled = 1;
    dec->network_delay_ns = 500000000LL; /* 500 ms default */
    dec->comp = nanompx_comp_dec_create();
    if (!dec->comp) {
        free(dec);
        return NULL;
    }
    return dec;
}

void nanompx_decoder_destroy(nanompx_decoder_t *dec)
{
    int i;
    if (!dec)
        return;
    for (i = 0; i < DEC_Q_MAX; i++)
        free(dec->q[i].pcm);
    nanompx_comp_dec_destroy(dec->comp);
    free(dec);
}

int nanompx_decoder_set_clock(nanompx_decoder_t *dec, nanompx_clock_t *clock)
{
    if (!dec)
        return NANOMPX_ERR_INVALID;
    dec->clock = clock;
    return NANOMPX_OK;
}

int nanompx_decoder_set_playout_delay(nanompx_decoder_t *dec,
                                      int64_t network_delay_ns,
                                      int64_t user_offset_ns)
{
    if (!dec)
        return NANOMPX_ERR_INVALID;
    dec->network_delay_ns = network_delay_ns;
    dec->user_offset_ns = user_offset_ns;
    return NANOMPX_OK;
}

int nanompx_decoder_set_sfn_enabled(nanompx_decoder_t *dec, int enabled)
{
    if (!dec)
        return NANOMPX_ERR_INVALID;
    dec->sfn_enabled = enabled ? 1 : 0;
    return NANOMPX_OK;
}

static queued_frame_t *q_slot(nanompx_decoder_t *dec)
{
    int i;
    for (i = 0; i < DEC_Q_MAX; i++) {
        if (!dec->q[i].active)
            return &dec->q[i];
    }
    return NULL;
}

static int decode_payload(nanompx_decoder_t *dec, const nanompx_packet_hdr_t *hdr,
                          const uint8_t *payload, size_t payload_len,
                          int32_t **out_pcm, size_t *out_count)
{
    int32_t *pcm;
    size_t count = 0;
    int rc;

    if (hdr->mode == NANOMPX_MODE_PCM) {
        count = payload_len / 3;
        pcm = (int32_t *)malloc(count * sizeof(int32_t));
        if (!pcm)
            return NANOMPX_ERR_NOMEM;
        nanompx_pcm_decode(payload, payload_len, pcm, count);
        *out_pcm = pcm;
        *out_count = count;
        return NANOMPX_OK;
    }

    if (hdr->mode == NANOMPX_MODE_COMPRESSED) {
        size_t est = 0;
        if (payload_len >= 4)
            est = (size_t)payload[2] | ((size_t)payload[3] << 8);
        if (est == 0 || est > 65535)
            return NANOMPX_ERR_INVALID;
        pcm = (int32_t *)malloc(est * sizeof(int32_t));
        if (!pcm)
            return NANOMPX_ERR_NOMEM;
        rc = nanompx_comp_decode(dec->comp, (nanompx_profile_t)hdr->profile, payload,
                                 payload_len, pcm, est, &count,
                                 (hdr->flags & NANOMPX_FLAG_KEYFRAME) != 0, hdr->sample_index);
        if (rc != NANOMPX_OK) {
            free(pcm);
            return rc;
        }
        *out_pcm = pcm;
        *out_count = count;
        return NANOMPX_OK;
    }

    return NANOMPX_ERR_UNSUPPORTED;
}

int nanompx_decoder_push_packet(nanompx_decoder_t *dec,
                                const uint8_t *packet,
                                size_t packet_len)
{
    nanompx_packet_hdr_t hdr;
    queued_frame_t *slot;
    int32_t *pcm = NULL;
    size_t count = 0;
    int rc;
    int gap = 0;

    if (!dec || !packet)
        return NANOMPX_ERR_INVALID;

    rc = nanompx_packet_validate(packet, packet_len);
    if (rc != NANOMPX_OK) {
        if (rc == NANOMPX_ERR_CRC)
            dec->stats.crc_errors++;
        return rc;
    }
    nanompx_hdr_unpack(&hdr, packet);

    if (dec->have_seq) {
        uint32_t expect = dec->expect_seq;
        if (hdr.seq != expect) {
            if ((int32_t)(hdr.seq - expect) > 0) {
                dec->stats.packets_lost += (uint64_t)(hdr.seq - expect);
                gap = 1;
            } else {
                dec->stats.packets_reorder++;
            }
        }
    }
    dec->expect_seq = hdr.seq + 1;
    dec->have_seq = 1;

    if (hdr.mode == NANOMPX_MODE_COMPRESSED) {
        if (gap) {
            /* Predictors / FIRs are useless after a hole — wait for KEYFRAME. */
            nanompx_comp_dec_reset(dec->comp);
            dec->wait_keyframe = 1;
        }
        if (dec->wait_keyframe && !(hdr.flags & NANOMPX_FLAG_KEYFRAME))
            return NANOMPX_ERR_AGAIN;
        if (hdr.flags & NANOMPX_FLAG_KEYFRAME)
            dec->wait_keyframe = 0;

        if (dec->last_profile && dec->last_profile != (nanompx_profile_t)hdr.profile &&
            !(hdr.flags & NANOMPX_FLAG_KEYFRAME) && !(hdr.flags & NANOMPX_FLAG_DISCONTINUITY)) {
            return NANOMPX_ERR_INVALID;
        }
        dec->last_profile = (nanompx_profile_t)hdr.profile;
    }

    rc = decode_payload(dec, &hdr, packet + NANOMPX_HDR_SIZE, hdr.payload_len, &pcm, &count);
    if (rc != NANOMPX_OK)
        return rc;

    slot = q_slot(dec);
    if (!slot) {
        free(pcm);
        return NANOMPX_ERR_OVERFLOW;
    }
    free(slot->pcm);
    slot->hdr = hdr;
    slot->pcm = pcm;
    slot->count = count;
    slot->pos = 0;
    slot->active = 1;
    dec->q_count++;
    dec->stats.packets_in++;
    dec->stats.buffer_depth_packets = (uint32_t)dec->q_count;
    return NANOMPX_OK;
}

static int64_t frame_due_ns(const nanompx_decoder_t *dec, const queued_frame_t *f)
{
    int64_t due =
        (int64_t)f->hdr.capture_time_ns + dec->network_delay_ns + dec->user_offset_ns;
    /* Compressed output is late by the filterbank delay relative to capture stamp. */
    if (f->hdr.mode == NANOMPX_MODE_COMPRESSED) {
        due -= (int64_t)nanompx_comp_codec_delay_samples() * 1000000000LL /
               (int64_t)NANOMPX_SAMPLE_RATE;
    }
    return due;
}

static queued_frame_t *earliest_due(nanompx_decoder_t *dec, int64_t now_ns)
{
    int i;
    queued_frame_t *best = NULL;
    uint64_t best_t = 0;
    for (i = 0; i < DEC_Q_MAX; i++) {
        queued_frame_t *f = &dec->q[i];
        int64_t due;
        if (!f->active)
            continue;
        due = frame_due_ns(dec, f);
        if (dec->sfn_enabled && due > now_ns)
            continue;
        if (!best || f->hdr.capture_time_ns < best_t) {
            best = f;
            best_t = f->hdr.capture_time_ns;
        }
    }
    return best;
}

int nanompx_decoder_pull_pcm(nanompx_decoder_t *dec,
                             int32_t *samples,
                             size_t max_samples,
                             size_t *out_count)
{
    queued_frame_t *f;
    int64_t now = 0;
    size_t n;

    if (!dec || !samples || !out_count)
        return NANOMPX_ERR_INVALID;
    *out_count = 0;

    if (dec->clock)
        nanompx_clock_now_ns(dec->clock, &now);
    else
        now = nanompx_host_time_ns();

    dec->stats.clock_locked = dec->clock ? nanompx_clock_locked(dec->clock) : 0;

    f = earliest_due(dec, now);
    if (!f)
        return NANOMPX_ERR_AGAIN;

    n = f->count - f->pos;
    if (n > max_samples)
        n = max_samples;
    memcpy(samples, f->pcm + f->pos, n * sizeof(int32_t));
    f->pos += n;
    *out_count = n;
    dec->stats.samples_decoded += n;

    if (f->pos >= f->count) {
        free(f->pcm);
        f->pcm = NULL;
        f->active = 0;
        f->count = 0;
        f->pos = 0;
        if (dec->q_count > 0)
            dec->q_count--;
        dec->stats.buffer_depth_packets = (uint32_t)dec->q_count;
    }
    return NANOMPX_OK;
}

void nanompx_decoder_get_stats(const nanompx_decoder_t *dec, nanompx_stats_t *out)
{
    if (!dec || !out)
        return;
    *out = dec->stats;
}
