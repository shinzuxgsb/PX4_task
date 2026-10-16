"""Strict, passive MAVLink v1 range decoding and bench-only range differentiation."""
import math
import struct
from collections import deque


def crc_x25(data, extra=85):
    crc = 65535
    for byte in bytes(data) + (bytes([extra]) if extra is not None else b''):
        value = byte ^ (crc & 255)
        value ^= (value << 4) & 255
        crc = ((crc >> 8) ^ (value << 8) ^ (value << 3) ^ (value >> 4)) & 65535
    return crc


class RangeDecoder:
    """Accept CRC-valid FC range and heartbeat, rejecting forwarded sensors/GCS.

    The bench link uses MAVLink v1; source sysid/compid must both equal one.
    MAVLink v2 is not supported; never guess its payload layout.
    """
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.heartbeats = deque(maxlen=8)

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 8:
            if self.buffer[0] != 254:
                del self.buffer[0]
                continue
            total = self.buffer[1] + 8
            if len(self.buffer) < total:
                break
            packet = bytes(self.buffer[:total])
            if packet[5] not in (0, 132):
                del self.buffer[:total]
                continue
            extra = 50 if packet[5] == 0 else 85
            if crc_x25(packet[1:-2], extra) != int.from_bytes(packet[-2:], 'little'):
                self.crc_errors += 1
                del self.buffer[0]
                continue
            del self.buffer[:total]
            if packet[3:5] != bytes([1, 1]):
                continue
            if packet[5] == 0:
                if packet[1] == 9 and packet[14] == 3:
                    self.heartbeats.append(bool(packet[12] & 128))
                continue
            if packet[1] != 14 or packet[3:5] != bytes([1, 1]):
                continue
            stamp, low, high, distance, kind, sensor, orientation, covariance = struct.unpack(
                '<IHHHBBBB', packet[6:20])
            if sensor == 0 and orientation == 25 and low < distance < high:
                result.append((stamp / 1000., distance / 100.))
        return result


def attitude_geometry(q):
    """PX4 [w,x,y,z] body-FRD -> NED quaternion: vertical projection and yaw."""
    if len(q) != 4 or not all(math.isfinite(x) for x in q):
        raise ValueError('Invalid attitude')
    norm = math.sqrt(sum(x*x for x in q))
    if not .95 < norm < 1.05:
        raise ValueError('Invalid quaternion norm')
    w, x, y, z = (float(v) / norm for v in q)
    return 1 - 2*(x*x + y*y), math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def body_down_from_ned_down(vx_body, vy_body, vz_ned, q, xy_var, vz_var):
    """Complete a body-FRD velocity using an earth-down velocity measurement.

    PX4 attitude rotates body FRD into NED.  The range derivative measures
    NED down, not body down, so solve the third row of that rotation for the
    missing body component.  Return its conservative diagonal variance too.
    """
    projection, _ = attitude_geometry(q)
    if projection <= .9 or not all(math.isfinite(v) for v in
                                    (vx_body, vy_body, vz_ned, xy_var, vz_var)):
        raise ValueError('Invalid body velocity geometry')
    if xy_var < 0 or vz_var < 0:
        raise ValueError('Invalid velocity variance')
    norm = math.sqrt(sum(float(v)*float(v) for v in q))
    w, x, y, z = (float(v)/norm for v in q)
    r31 = 2*(x*z-w*y)
    r32 = 2*(y*z+w*x)
    vz_body = (vz_ned-r31*vx_body-r32*vy_body)/projection
    body_z_var = (vz_var+(r31*r31+r32*r32)*xy_var)/(projection*projection)
    return vz_body, body_z_var


class DdsRangeClock:
    """DDS TimesyncStatus offset is FC minus companion, in microseconds."""
    def __init__(self):
        self.offset = None
        self.received = float('-inf')

    def update(self, offset_us, remote_us, rtt_us, now_ros, now_mono):
        if (rtt_us < 0 or rtt_us > 20000
                or abs(now_ros-remote_us/1e6) > 2.):
            return False
        new_offset = -offset_us/1e6
        changed = self.offset is not None and abs(new_offset-self.offset) > .05
        self.offset, self.received = new_offset, now_mono
        return changed

    def stamp(self, boot, now_ros, now_mono):
        # Bounded holdover across missed 1 Hz sync reports. Range timestamps
        # still have to pass the independent future/staleness check below.
        if self.offset is None or now_mono-self.received > 5.:
            return None
        stamp = boot+self.offset
        return stamp if -.05 <= now_ros-stamp <= .3 else None


def significant_attitude_reset(delta_q, threshold_deg=5.):
    """Only genuine frame jumps are propagated as external sensor resets."""
    if len(delta_q) != 4 or not all(math.isfinite(v) for v in delta_q):
        return True
    norm = math.sqrt(sum(float(v)**2 for v in delta_q))
    if not .95 < norm < 1.05:
        return True
    angle = 2*math.acos(min(1., abs(float(delta_q[0]))/norm))
    return angle > math.radians(threshold_deg)


def disarmed_preview_allowed(status_disarmed, status_age, control_armed, control_age,
                             heartbeat_armed=None, heartbeat_age=float('inf')):
    # No timeout relaxation: either fresh source can confirm disarmed, but
    # a known armed/conflicting state from either source always inhibits output.
    # PX4 status streams can cross 1.0 s by a few milliseconds even on a healthy
    # XRCE link, so retain a small, measured scheduling margin.
    return (status_disarmed is True and control_armed is not True and heartbeat_armed is not True
            and (status_age <= 1.2 or (control_armed is False and control_age <= 1.2)
                 or (heartbeat_armed is False and heartbeat_age <= 1.5)))


class DownVelocityEstimator:
    """Flat-ground range derivative, not an independently measured global VZ."""
    def __init__(self, window=.5, min_span=.3, max_speed=1., min_height=.05):
        self.window, self.min_span = window, min_span
        self.max_speed, self.min_height = max_speed, min_height
        self.samples = deque(maxlen=64)
        self.state = 'WARMUP'

    def invalidate(self, state='STALE'):
        self.samples.clear()
        self.state = state

    def update(self, stamp, distance, projection):
        if (not all(math.isfinite(v) for v in (stamp, distance, projection))
                or not math.cos(math.radians(20)) <= projection <= 1.):
            self.invalidate('INVALID_RANGE_OR_TILT')
            return None
        height = distance * projection
        if height < self.min_height:
            self.invalidate('HEIGHT_TOO_LOW')
            return None
        if self.samples:
            dt = stamp - self.samples[-1][0]
            if dt == 0:
                return None
            if (dt <= 0 or dt > .3
                    or abs(height-self.samples[-1][1])/dt > self.max_speed):
                self.invalidate('RANGE_RESET')
        self.samples.append((stamp, height))
        while len(self.samples) > 2 and stamp-self.samples[0][0] > self.window:
            self.samples.popleft()
        # At ~15 Hz a 0.30 s window may contain only four valid points under
        # normal timing jitter. Require the same time span and residual gates,
        # rather than spuriously restarting warmup at that count boundary.
        if len(self.samples) < 4 or stamp-self.samples[0][0] < self.min_span:
            self.state = 'WARMUP'
            return None
        mean_t = sum(t-stamp for t, h in self.samples)/len(self.samples)
        mean_h = sum(h for t, h in self.samples)/len(self.samples)
        denominator = sum((t-stamp-mean_t)**2 for t, h in self.samples)
        slope = sum((t-stamp-mean_t)*(h-mean_h) for t, h in self.samples)/denominator
        residual = sum((h-mean_h-slope*(t-stamp-mean_t))**2 for t, h in self.samples)
        noise = max(.01**2/12, residual/max(1, len(self.samples)-2))
        if abs(slope) > self.max_speed or math.sqrt(noise) > .025:
            self.invalidate('RANGE_NOISY')
            return None
        self.state = 'VALID'
        return stamp+mean_t, -slope, max(.05**2, noise/denominator)
