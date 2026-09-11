# Contributing to NanoMPX

Thank you for helping build an open MPX-over-IP codec other vendors can use.

Project home: [https://www.kvarta.net](https://www.kvarta.net)

Much of this codebase was drafted with AI coding tools under human direction. When
contributing, prefer clear, reviewable changes and keep protocol docs in sync.

## Ground rules

- Keep the **wire protocol** documented in `docs/PROTOCOL.md` in sync with code changes.
- Do not break PCM bit-exact roundtrip without a protocol version bump.
- New compressed profiles should use new **profile IDs** (keep L/M/S stable).
- Prefer portable C99; avoid kunix or other proprietary dependencies.
- All source files should carry `SPDX-License-Identifier: MIT`.

## Development

```bash
cmake -S . -B build
cmake --build build -j
cd build && ctest --output-on-failure
```

Add interop fixtures under `tests/vectors/` when changing framing or compressed layouts.

## Pull requests

1. Describe the motivation and protocol impact.
2. Include tests for new behavior.
3. Update PROTOCOL.md / SFN.md when the wire format or timing model changes.

## Code of conduct

Be respectful. This project aims to be usable by competing broadcast vendors; keep
discussion technical and inclusive.
