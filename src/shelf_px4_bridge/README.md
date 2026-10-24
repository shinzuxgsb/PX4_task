# shelf_px4_bridge

PX4 lifecycle adapter for the 2024 electronic-design D-theme shelf mission.

The mission-side code is kept from the working anonymous-flight-controller
Orange Pi workspace:

* `shelf_scan_pkg`: shelf geometry, route planner, QR reader, laser trigger and
  ground-station protocol;
* `pid_control_pkg`: map-frame position controller.

The anonymous STM32 serial protocol is not used. The PID output is rotated to
body FLU and feeds the already validated `mavlink_bridge`:

`/target_position -> position_pid_controller -> /target_velocity ->
mavlink_bridge -> PX4 SET_POSITION_TARGET_LOCAL_NED`

`shelf_px4_adapter` owns arming, Offboard prestreaming, the equivalent of the
anonymous `takeoff_if` signal, mission health monitoring and the final PX4 Land
handoff.

Before flying, the lidar must still publish `/scan` on `/dev/ttyS4` and the
camera node must have exclusive access to `/dev/video0`.
