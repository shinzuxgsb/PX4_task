# 147 active MAVLink/Cartographer workspace

This branch is the active flight stack used on the 147 Orange Pi. It is
kept separate from `main`, which is the older `px4_ws` chain.

## Branch difference

| Item | `main` | `mavlink_ws` |
|---|---|---|
| Purpose | older `px4_ws` chain | active 147 localization/flight chain |
| Positioning | old chain | Cartographer + MAVLink ODOMETRY |
| Cartographer | old workspace setup | source-built ROS 2 2.0.9003 |
| Lidar | old configuration | serial LDS-50C-R on `/dev/ttyS4` |
| Safety | old behavior | fail-closed SLAM jump guard |
| Startup | old behavior | conditional NTP gate (Wi-Fi only) |

## Active hardware and links

- Orange Pi 5 Max, host `era-desktop`, address `10.23.87.147` on the
  debug Wi-Fi network.
- PX4 flight controller: MicoAir743 V2.
- Serial lidar: LDS-50C-R on `/dev/ttyS4` at 921600 baud.
- TOF rangefinder: TOFSense-M on `/dev/ttyS3`.
- PX4 MAVLink: `/dev/ttyS6` at 115200 baud.
- Systemd service: `mavlink-ws-chain.service`.

The active workspace is `/home/orangepi/mavlink_ws` on the vehicle. The
Cartographer source/build is in `/home/orangepi/cartographer_ws` and is
loaded as an overlay by `start_mavlink_ws.sh`.

## Important changes in this branch

### MAVLink ODOMETRY safety

`mavlink_bridge` sends the SLAM pose to PX4 as `ODOMETRY`. A fail-closed
jump guard rejects any pose jump larger than `0.5 m`:

- stop sending external vision;
- do not increment `reset_counter`;
- resume only if the pose returns to the last trusted position;
- if it remains at a new position, keep EV stopped and log an error.

This prevents PX4 from anchoring to a false SLAM relocation. The previous
272 m jump and the later 180 m jump were both detected by this guard.

### Cartographer

The branch uses Cartographer ROS 2 `2.0.9003`, built from the teammate
source tree. The active process must be:

```text
/home/orangepi/cartographer_ws/install/cartographer_ros/lib/cartographer_ros/cartographer_node
```

`amphi.lua` and the other six Lua files are byte-identical to the teammate
configuration.

### Conditional NTP gate

The serial lidar driver uses system time (`time_mode=0`) and reconstructs
scan stamps with `gettimeofday()`. If NTP steps the clock after SLAM has
started, Cartographer can interpret the time jump as hundreds of metres of
motion.

The startup script therefore does this:

- **Wi-Fi connected (debug):** wait for `NTPSynchronized=yes` before
  starting the chain.
- **Wi-Fi disconnected (flight):** start immediately using the RTC time;
  no NTP step is expected because the vehicle is offline.

Do not connect Wi-Fi after an offline start. For debug, connect Wi-Fi
before boot, or stop the chain, synchronize NTP, and restart the chain.

Measured on the current phone hotspot:

```text
boot -> NTP initial sync: 155.9 s
NTP -> EV warmup: 11.1 s
NTP -> first ekf_ok=True: 15.0 s
```

The bottleneck is Wi-Fi association, not NTP server latency. With Wi-Fi
already connected, a chain restart reaches `ekf_ok=True` in about 15 s.

### Remaining limitation

The startup gate does not make the lidar clock monotonic. A later NTP step
while the chain is running can still cause a timestamp jump. The robust
long-term fix is to stamp scans with a monotonic clock or add a scan
timestamp sanity gate. Until then, follow the offline/debug rule above.

## Run and service

```bash
# manual start (debug Wi-Fi connected: waits for NTP)
bash /home/orangepi/mavlink_ws/start_mavlink_ws.sh

# stop
bash /home/orangepi/mavlink_ws/start_mavlink_ws.sh stop

# service
sudo systemctl restart mavlink-ws-chain
sudo systemctl status mavlink-ws-chain --no-pager
```

The systemd unit is stored at `systemd/mavlink-ws-chain.service`.

## Excluded from Git

Generated and local-only files are intentionally excluded:

- `build/`, `install/`, `log/`
- `__pycache__/`, `*.pyc`
- `*.bak-*`, `*.new`
- wheel and tarball artifacts

The source archive in this branch contains the workspace source,
configuration, documentation, blackbox tools, and the unpacked
`third_party` Python packages used on the board.
