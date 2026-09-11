# NanoMPX

**Open-source FM composite (MPX) over IP codec** — encode and decode a full MPX signal
(including stereo pilot and RDS) for STL distribution, with optional SRT transport and
GPS/1PPS SFN synchronization.

- Project site: [www.kvarta.net](https://www.kvarta.net)
- License: **MIT**
- Sample rate: **192 kHz / 24-bit**
- Modes: **PCM** (reference quality) and **COMPRESSED** profiles **L / M / S**
- Protocol: open — see [docs/PROTOCOL.md](docs/PROTOCOL.md)

NanoMPX is inspired by the problem space of products like MicroMPX, but it is an
**independent** open protocol and implementation (not interoperable with Thimeo MicroMPX).

## Project / attribution

NanoMPX is developed by **[Kvarta](https://www.kvarta.net)**. Most of the code in this
repository was **produced with AI coding assistants**, then reviewed, tested, and
directed by humans. Treat it like any other open-source C library: verify behavior for
your use case before production deployment.

## Build

```bash
cmake -S . -B build
cmake --build build -j
cd build && ctest --output-on-failure
```

Or: `make && make test`

### Options

| CMake option | Default | Meaning |
|--------------|---------|---------|
| `NANOMPX_WITH_SRT` | ON | Enable SRT helpers if `libsrt` is found |
| `NANOMPX_BUILD_TOOLS` | ON | Build `nanompx_cli` |
| `NANOMPX_BUILD_TESTS` | ON | Build unit tests |

If system `libsrt` headers are missing, point CMake at a libsrt install:

```bash
cmake -S . -B build -DNANOMPX_SRT_ROOT=/path/to/srt
# or: export NANOMPX_SRT_ROOT=...
```

The reference tree can also pick up headers from a sibling `../kunix/libsrt`.

## CLI

```bash
# Raw s24le mono @ 192 kHz
./build/nanompx_cli encode --mode pcm -i in.s24 -o out.nmpx
./build/nanompx_cli encode --mode comp --profile M -i in.s24 -o out.nmpx
./build/nanompx_cli decode --sfn 0 -i out.nmpx -o out.s24
./build/nanompx_cli roundtrip --mode pcm -i in.s24
```

## Library sketch

```c
#include "nanompx.h"

nanompx_encoder_t *enc = nanompx_encoder_create();
nanompx_encoder_set_mode(enc, NANOMPX_MODE_COMPRESSED);
nanompx_encoder_set_profile(enc, NANOMPX_PROFILE_L);
nanompx_encoder_set_clock(enc, nanompx_clock_gps_create("/dev/ttyUSB0", "/dev/pps0"));
/* nanompx_encoder_push_pcm(...) -> framed packets */
```

## Naming / trademark

“NanoMPX” may be used to describe implementations that conform to [docs/PROTOCOL.md](docs/PROTOCOL.md).
Please do not use the name for incompatible forks without a clear qualifier.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).
