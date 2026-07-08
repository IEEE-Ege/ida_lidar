# Vendored: Slamtec RPLIDAR SDK

This directory contains a subset of the RPLIDAR SDK, vendored from:

- Source: https://github.com/slamtec/rplidar_ros
- Branch: `ros2`
- Commit: `24cc9b6dea97e045bda1408eaa867ce730fd3fc3` (2025-04-27)
- Subpath: `sdk/`

Only the Linux-target subset is included (the upstream project's own
`CMakeLists.txt` builds this same file set on non-Apple platforms): all of
`sdk/include/`, plus `sdk/src/*.cpp`, `sdk/src/arch/linux/`,
`sdk/src/dataunpacker/`, and `sdk/src/hal/`. The `macOS`/`win32` arch
backends were never copied in — unnecessary for this project.

Also dropped, as dead weight for a serial-only connection to a single C1 unit
(this package never opens a TCP/UDP channel, and RplidarNode.cpp only uses the
`sl::` namespaced API):
- `sl_tcp_channel.cpp`, `sl_udp_channel.cpp` — network channel implementations.
- `arch/linux/net_socket.cpp` — only used by the two files above.
- `rplidar_driver.cpp` — the legacy pre-`sl::`-namespace driver implementation
  (superseded by `sl_lidar_driver.cpp`, which is what this project uses).

The legacy headers (`rplidar.h`, `rplidar_driver.h`, `rplidar_cmd.h`,
`rplidar_protocol.h`, `rptypes.h`) are kept: `sdkcommon.h` — used by the code
this project does need — unconditionally includes `rplidar.h`, and vendored
files are kept unmodified rather than patched.

Remaining files are unmodified from upstream. License: BSD-2-Clause, see
`LICENSE` in this directory (also reproduced from the same commit).
