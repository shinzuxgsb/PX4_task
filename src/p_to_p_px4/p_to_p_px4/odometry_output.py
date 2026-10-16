"""Build PX4 external-vision odometry without inventing missing measurements."""

import math


def horizontal_pose_ready(xy, now_ros, lidar_age, attitude_age,
                          last_output_stamp, last_full_age):
    """Fail closed unless a new, current lidar pose/yaw can stand alone."""
    if xy is None or last_full_age <= .2 or lidar_age > .3 or attitude_age > .3:
        return False
    stamp = xy[0]
    if not math.isfinite(stamp) or stamp <= last_output_stamp:
        return False
    if not -.05 <= now_ros - stamp <= .3:
        return False
    q = xy[4]
    return (all(math.isfinite(value) for value in (*q, *xy[3], *xy[8], *xy[6:8]))
            and abs(sum(value*value for value in q) - 1.) <= .01
            and math.isfinite(xy[5][2]) and xy[5][2] > 0)


def make_odometry(message_type, now_us, stamp, xy, reset, body_vz=None,
                  body_vz_var=None):
    nan = float('nan')
    xy_var = max(.15**2, *xy[3])
    msg = message_type()
    msg.timestamp = now_us
    msg.timestamp_sample = int(stamp*1e6)
    msg.pose_frame = message_type.POSE_FRAME_FRD
    msg.velocity_frame = message_type.VELOCITY_FRAME_FRD
    # PX4 1.15 requires finite XYZ position for horizontal position fusion.
    # EKF2_EV_CTRL vertical position must remain disabled: Z is a placeholder.
    msg.position = [float(xy[6]), float(xy[7]), 0.0]
    msg.q = [float(value) for value in xy[4]]
    msg.angular_velocity = [nan]*3
    if body_vz is None:
        # PX4 accepts the independent pose/yaw; missing TOF velocity must not
        # be misrepresented as zero or reused from an older sample.
        msg.velocity = [nan]*3
        msg.velocity_variance = [nan]*3
    else:
        # The preview's horizontal components are yaw-only body FRD. Undo
        # that rotation using its own time-aligned lidar yaw, never PX4 yaw.
        w, x, y, z = xy[4]
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        c, s = math.cos(yaw), math.sin(yaw)
        msg.velocity = [float(c*xy[1]-s*xy[2]),
                        float(s*xy[1]+c*xy[2]), float(body_vz)]
        msg.velocity_variance = [float(xy_var), float(xy_var), float(body_vz_var)]
    msg.position_variance = [float(max(.08**2, xy[8][0])),
                             float(max(.08**2, xy[8][1])), 100.]
    msg.orientation_variance = [100., 100., float(xy[5][2])]
    msg.reset_counter, msg.quality = reset, 0
    return msg
