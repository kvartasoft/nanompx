# NanoMPX Protocol Specification

**Status:** normative for NanoMPX protocol version 1  
**License:** this document is published under MIT alongside the reference implementation.

NanoMPX transports a full FM composite (MPX) signal — baseband stereo multiplex including
19 kHz pilot and RDS — over IP. This specification defines the packet format so vendors can
interoperate without linking the reference library.

NanoMPX is **not** compatible with Thimeo MicroMPX.

## Sample format

| Parameter | Value |
|-----------|-------|
| Sample rate | 192000 Hz (fixed) |
| Sample format (API) | signed 24-bit in `int32_t` (sign-extended) |
| PCM wire packing | little-endian 24-bit, 3 bytes per sample |

## Packet layout

Every packet is: **32-byte header** + **payload**.

All multi-byte integer fields are **little-endian**.

### Header (32 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 4 | `magic` | `0x58504D4E` (“NMPX”) |
| 4 | 1 | `version` | Protocol version (`1`) |
| 5 | 1 | `mode` | `0` = PCM, `1` = COMPRESSED |
| 6 | 1 | `profile` | `0` none/PCM; `1`=L, `2`=M, `3`=S |
| 7 | 1 | `flags` | bit0 keyframe, bit1 discontinuity, bit2 FEC |
| 8 | 4 | `seq` | monotonically increasing sequence number |
| 12 | 8 | `sample_index` | index of first sample in this frame |
| 20 | 8 | `capture_time_ns` | GPS-tied capture time of first sample (ns) |
| 28 | 2 | `payload_len` | payload bytes following header |
| 30 | 2 | `hdr_crc` | CRC-16/CCITT-FALSE over bytes 0–29 (`init=0xFFFF`, poly `0x1021`) |

Receivers **must** drop packets with bad magic, unsupported version, or bad `hdr_crc`.

### Flags

| Bit | Name | Meaning |
|-----|------|---------|
| 0 | KEYFRAME | Independent frame; required after mode/profile change |
| 1 | DISCONTINUITY | Timeline break |
| 2 | FEC | Payload is FEC parity (reserved; ignore if unsupported) |

## Modes

### Mode 0 — PCM

Payload is packed s24le mono samples. Length must be a multiple of 3.

Approximate bitrate: `192000 × 24 ≈ 4.608 Mbit/s` plus header overhead.

### Mode 1 — COMPRESSED

FM-aware codec optimized for usable quality at L/M/S bitrates. Payload starts with:

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | `0xC2` compressed magic (v2) |
| 1 | 1 | profile (L/M/S) |
| 2 | 2 | `n_samples` |
| 4 | 1 | residual quantizer bits |
| 5 | 1 | residual decimation factor |
| 6 | 2 | source peak (`peak * 65535`) |
| 8 | … | bit-packed body |

Body (bit-packed, LSB-first within each byte):

1. Pilot amplitude `float32` + phase0 `float32` (phase referenced to absolute sample 0; continuous across frames)
2. `u16 n_main` + predictive block-float residual codes

Processing model:

1. Fit/subtract parametric 19 kHz pilot (protected).
2. Optionally anti-alias and decimate the residual (`decim` from header).
3. Encode with first-order DPCM + noise-shaped block floating-point (u8 log scale per 48 samples).
4. Reconstruct: `residual + pilot`, then soft-limit to source peak.

#### Profiles

| Profile | ID | Target bitrate | Intent |
|---------|----|----------------|--------|
| L (Large) | 1 | ~1600 kbit/s | Highest compressed quality |
| M (Medium) | 2 | ~960 kbit/s | Balanced |
| S (Small) | 3 | ~640 kbit/s | Constrained links |

v2 reference table (normative for profile IDs 1–3):

| Profile | bits | decim | Approx payload rate |
|---------|------|-------|---------------------|
| L | 8 | 1 | ~1536 kbit/s |
| M | 5 | 1 | ~960 kbit/s |
| S | 3 | 1 | ~576 kbit/s |

Log scale code: `scale = 2^((code - 140) / 16)` with `code` in 0…255.

Profile is present in **every** packet header. Switching profile on a stream requires
`KEYFRAME` or `DISCONTINUITY`. Mixing profiles without that is invalid.

Peak control: decoders must not emit samples whose absolute level exceeds the encoded
frame peak (soft limit). Encoders must not invent overshoots above the source frame peak.

## Transport

NanoMPX packets are transport-agnostic. Recommended mapping:

- **SRT**: one SRT message = one NanoMPX packet
- **UDP**: one datagram = one NanoMPX packet (add FEC/redundancy as needed)
- **File**: concatenation of packets (as produced by the reference CLI)

## SFN timing

`capture_time_ns` is the absolute time of the first sample according to a GPS-disciplined
clock (or equivalent). Decoders play at:

```
playout_time = capture_time_ns + network_delay_ns + user_offset_ns
```

All SFN sites **must** use the same `network_delay_ns`. `user_offset_ns` fine-tunes
interference nulls. Target relative accuracy: **< 1 µs** after GPS lock with matched I/O delay.

See [SFN.md](SFN.md).

## Versioning

- `version` increments on breaking header changes.
- Compressed payload magic/layout may evolve with new profile IDs without bumping
  protocol version if old profiles remain decodable.
- Reserved profile IDs `4…255` for future rates (e.g. XS).

## Conformance

Reference vectors live under `tests/vectors/`. A conforming implementation must:

1. Accept PCM mode packets and reconstruct bit-exact s24le samples.
2. Parse headers and reject CRC failures.
3. Decode L/M/S compressed profiles produced by the reference encoder (within documented quality).
4. Honor `capture_time_ns` when SFN mode is enabled.
