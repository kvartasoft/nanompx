# Validation notes

## Automated (CI)

```bash
cmake -S . -B build && cmake --build build -j
cd build && ctest --output-on-failure
```

| Test | Checks |
|------|--------|
| `test_roundtrip` | PCM encode/decode bit-exact |
| `test_clock_sfn` | SFN delay gating + GPS inject lock |
| `test_compressed` | L/M/S decode, no invented overshoots, bitrate bound |

## Interop vectors

See `tests/vectors/`:

- `tone.s24` — synthetic MPX-ish s24le @ 192 kHz
- `tone_pcm.nmpx` — PCM stream
- `tone_comp_L.nmpx` — compressed profile L

## Manual SRT check

Terminal A:

```bash
./build/nanompx_cli srt-recv --host 0.0.0.0 --port 9000 -o /tmp/out.s24
```

Terminal B:

```bash
./build/nanompx_cli srt-send --host 127.0.0.1 --port 9000 -i tests/vectors/tone.s24
```

## SFN lab

1. Two decoders, shared PPS or `nanompx_clock_sim` with common origin.
2. Same `network_delay_ns` on both.
3. Correlate outputs; calibrate hardware delay into `user_offset_ns`.
