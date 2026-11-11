# Vendored: roboclaw_serial

| | |
|---|---|
| upstream | https://github.com/dumbotics/roboclaw_serial |
| commit | `cb14207447aa9179cc0c6da7a0e41f283ca595f3` |
| license | Apache-2.0 (see `LICENSE`), Copyright Eric Cox 2023 |
| vendored | 2026-11-11, replacing the git submodule. Upstream's `.github/` CI was dropped. |

This is a normal directory in our repo now: edit and commit it like any other package.
Local changes are marked `BILLEE:` in the source.

## Local changes

Changes so the library works over a real (TTL) UART, not just USB:

- **Baud rate** (`device.hpp`). `SerialDevice(device, baud_rate = 38400)` now sets the port
  speed with `cfsetispeed/cfsetospeed`. Upstream left it at `B0`, which only works over USB
  CDC-ACM, where the speed is ignored. Unsupported rates throw `std::invalid_argument`.
- **Full-reply reads** (`interface.hpp`). `Interface::read()` loops until the whole reply
  has arrived. Fixed-size replies need payload + CRC; the firmware string needs text, `\0`,
  then CRC. Upstream took one `read()`, which over a UART usually returns only the first
  byte(s).
- **Resync** (`interface.hpp`, `device.hpp`). Input is flushed (`flushInput()`, i.e.
  `tcflush`) before every request, so a late reply can't corrupt the next one.
- **ACK check** (`interface.hpp`). An empty read is treated as a missing ACK. Upstream
  compared against a stale buffer byte.
- **Tests** (8 passing; not yet run against real RoboClaw hardware). The dummy device can
  split replies across reads (`setMaxBytesPerRead`), and
  `flushInput()` is a no-op there. New tests cover 1- and 3-byte reads, the firmware string
  (whole and split), a reply that never arrives (throws instead of hanging), and baud
  validation.
