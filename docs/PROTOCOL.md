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
| 20 | 8 | `capture_time_ns` | capture time of first sample (sample-rate timeline, GPS-tied) |
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
Uncompressed payload is 3 bytes per sample (plus packet headers).

### Mode 1 — COMPRESSED

FM band codec (magic `0xC4`) with Rice-coded predictive residuals.

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | `0xC4` compressed magic (v4) |
| 1 | 1 | profile (L/M/S) |
| 2 | 2 | `n_samples` |
| 4 | 1 | `(bits_mono<<4) | bits_stereo` |
| 5 | 1 | `(bits_rds<<4) | rice_tip_mono` |
| 6 | 2 | source peak (`peak * 65535`) |
| 8 | … | bit-packed body |

Body (LSB-first bits):

1. Pilot amplitude `float32` + phase0 `float32` (phase locked after keyframe; `sin(ωt+φ)` model)
2. `rice_tip_stereo` u4 + `rice_tip_rds` u4
3. `n_mono`, `n_stereo`, `n_rds` as u16 each
4. For each band: blocks of `{rice_k u4, scale u8, Rice-coded DPCM codes}`

Processing model:

1. Fit/lock parametric 19 kHz pilot (protected). Subtract from MPX.
2. Demodulate FM bands with 1×/2×/3× pilot phase:
   - **Mono** (0–15 kHz): LP → decimate ×4 (48 kHz)
   - **Stereo L−R** (≈23–53 kHz): ×2sin(2θ) → LP → decimate ×4
   - **RDS** (≈57 kHz): ×2sin(3θ) → narrow LP → decimate ×32
3. Encode each band with noise-shaped DPCM + adaptive Rice (`k` per 32-sample block).
4. Reconstruct with matched upsample/LP, remodulate, add delayed pilot (filter delay compensated).
5. Soft-limit to source peak.

#### Profiles (reference bit allocation)

| Profile | ID | Mono bits | Stereo bits | RDS bits |
|---------|----|-----------|-------------|----------|
| L (Large) | 1 | 10 | 8 | 5 |
| M (Medium) | 2 | 7 | 5 | 3 |
| S (Small) | 3 | 5 | 4 | 2 |

Payload size is content- and Rice-dependent; profiles differ by quantizer depth, not a fixed rate.

Quality should be judged with **band-weighted** metrics (mono/stereo/RDS), not flat full-MPX waveform SNR. Flat MPX SNR unfairly weights ultrasonic stopbands the codec deliberately discards.

Log scale: `scale = 2^((code - 140) / 16)`.

Profile is present in **every** packet header. Switching profile requires `KEYFRAME` or `DISCONTINUITY`.

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
- Reserved profile IDs `4…255` for future profiles (e.g. XS).

## Conformance

Reference vectors live under `tests/vectors/`. A conforming implementation must:

1. Accept PCM mode packets and reconstruct bit-exact s24le samples.
2. Parse headers and reject CRC failures.
3. Decode L/M/S compressed profiles produced by the reference encoder (within documented quality).
4. Honor `capture_time_ns` when SFN mode is enabled.
