# NanoMPX SFN Synchronization

Single Frequency Network (SFN) operation requires that every transmitter radiate the
**same** MPX waveform at the **same** absolute time (within about 1 µs).

## Model

1. Encoder stamps each packet with `capture_time_ns` from a GPS/1PPS-disciplined clock.
2. Packets traverse the network (SRT/UDP/…).
3. Each decoder buffers until:

   `now >= capture_time_ns + network_delay_ns + user_offset_ns`

4. All sites share the same `network_delay_ns`. Use `user_offset_ns` only for fine
   RF interference alignment.

```text
Studio  --encode+timestamp-->  IP  -->  Tx A decode@T
                                 \-->  Tx B decode@T
```

## Clock backends

| Backend | API | Notes |
|---------|-----|-------|
| Host | `nanompx_clock_host_create` | Testing only — not SFN-grade |
| Simulated PPS | `nanompx_clock_sim_create` | CI / lab without GPS hardware |
| GPS + 1PPS | `nanompx_clock_gps_create` | Production path; inject NMEA/PPS or wire devices |

Vendors may implement `nanompx_clock_vtbl_t` for PTP or proprietary timing hardware.

### GPS lock

The reference GPS clock reports locked when both:

- a valid NMEA time (e.g. RMC/ZDA) has been seen, and
- at least one PPS edge has been injected/received.

Until locked, SFN playout should be treated as degraded (do not put transmitters on air
in overlapping SFN regions).

## Hardware notes

- Prefer digital MPX I/O with **constant** capture/playout delay (ASIO-class / AES).
- Use identical decoder hardware/firmware across SFN sites when possible.
- Calibrate residual I/O delay into `user_offset_ns`.

## Lab verification

1. Share a PPS (or `nanompx_clock_sim` with common origin) across two decoders.
2. Feed the same NanoMPX stream.
3. Measure relative output alignment (scope / correlation); expect sub-microsecond
   software timeline alignment; hardware path delay must be calibrated separately.
