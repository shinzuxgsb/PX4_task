"""Passive TOFSense-M decoding and local sample transport helpers."""
import math
import statistics
import struct
from collections import deque

from .range_preview import crc_x25


TOF_IPC_MAGIC = b'TOF1'
TOF_IPC_FORMAT = '<4sQIf'


class TofHeightKalman:
    """Constant-velocity Kalman filter for fresh downward TOF distances.

    The bridge feeds monotonic receipt timestamps, not unverified device ticks.
    A rejected frame is never sent as a predicted measurement to PX4. The
    third consistent large innovation is treated as a real height step, so
    the estimator cannot remain latched to an old surface indefinitely.
    """

    def __init__(self, measurement_variance=.02**2, acceleration_variance=1.0,
                 max_gap=.3, max_speed=2.0):
        if (measurement_variance <= 0 or acceleration_variance <= 0
                or max_gap <= 0 or max_speed <= 0):
            raise ValueError('Kalman variances, gap and speed must be positive')
        self.measurement_variance = measurement_variance
        self.acceleration_variance = acceleration_variance
        self.max_gap = max_gap
        self.max_speed = max_speed
        self.reset()

    def reset(self):
        self.height = self.velocity = self.stamp = None
        self.p_height = self.p_cross = self.p_velocity = None
        self.rejected = 0
        self.last_rejected = None

    def _initialize(self, stamp, height):
        self.height, self.velocity, self.stamp = height, 0., stamp
        self.p_height = self.measurement_variance
        self.p_cross = 0.
        self.p_velocity = self.max_speed**2 / 4
        self.rejected = 0
        self.last_rejected = None
        return height

    def update(self, stamp, height):
        if (not math.isfinite(stamp) or not math.isfinite(height)
                or not .02 <= height <= 4.):
            self.reset()
            return None
        if self.stamp is None:
            return self._initialize(stamp, height)
        dt = stamp - self.stamp
        if dt <= 0:
            return None
        if dt > self.max_gap:
            return self._initialize(stamp, height)

        q = self.acceleration_variance
        predicted_height = self.height + self.velocity * dt
        predicted_velocity = self.velocity
        p_height = (self.p_height + 2*dt*self.p_cross
                    + dt*dt*self.p_velocity + q*dt**4/4)
        p_cross = self.p_cross + dt*self.p_velocity + q*dt**3/2
        p_velocity = self.p_velocity + q*dt*dt
        innovation = height - predicted_height
        innovation_variance = p_height + self.measurement_variance
        # Allow ordinary aircraft motion while rejecting isolated large echoes.
        gate = 4*math.sqrt(innovation_variance) + self.max_speed*dt
        if abs(innovation) > gate:
            if (self.last_rejected is not None
                    and abs(height-self.last_rejected) <= .08):
                self.rejected += 1
            else:
                self.rejected = 1
            self.last_rejected = height
            if self.rejected >= 3:
                return self._initialize(stamp, height)
            return None

        gain_height = p_height / innovation_variance
        gain_velocity = p_cross / innovation_variance
        self.height = predicted_height + gain_height*innovation
        self.velocity = max(-self.max_speed, min(self.max_speed,
                            predicted_velocity + gain_velocity*innovation))
        self.p_height = (1-gain_height)*p_height
        self.p_cross = (1-gain_height)*p_cross
        self.p_velocity = max(0., p_velocity-gain_velocity*p_cross)
        self.stamp = stamp
        self.rejected = 0
        self.last_rejected = None
        return max(.02, min(4., self.height))


def tof_ipc_packet(receipt_monotonic_ns, device_ticks, distance_m):
    """Encode one local-only TOF sample for the velocity process."""
    return struct.pack(TOF_IPC_FORMAT, TOF_IPC_MAGIC,
                       int(receipt_monotonic_ns), int(device_ticks),
                       float(distance_m))


def decode_tof_ipc(packet):
    if len(packet) != struct.calcsize(TOF_IPC_FORMAT):
        raise ValueError('Invalid TOF IPC packet length')
    magic, receipt_ns, device_ticks, distance = struct.unpack(TOF_IPC_FORMAT, packet)
    if magic != TOF_IPC_MAGIC or not math.isfinite(distance):
        raise ValueError('Invalid TOF IPC packet')
    return receipt_ns / 1e9, device_ticks, distance


def distance_sensor_v1(distance_m, sequence, time_boot_ms,
                       system_id=42, component_id=191, sensor_id=77,
                       covariance_cm2=3):
    """Encode one downward MAVLink-v1 DISTANCE_SENSOR message."""
    if not .02 <= distance_m <= 4.:
        raise ValueError('distance outside advertised limits')
    payload = struct.pack('<IHHHBBBB', int(time_boot_ms) & 0xffffffff,
                          2, 400, round(distance_m * 100),
                          0, sensor_id, 25, covariance_cm2)
    header = bytes((len(payload), sequence & 255, system_id, component_id, 132))
    return b'\xfe' + header + payload + struct.pack('<H', crc_x25(header + payload, 85))


class TofSenseMDecoder:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.frames_seen = 0
        self.frames_valid = 0
        self.mode_rejected = 0
        self.range_rejected = 0

    def feed(self, data):
        self.buffer.extend(data)
        results = []
        while len(self.buffer) >= 400:
            if self.buffer[:2] != b'\x57\x01':
                del self.buffer[0]
                continue
            packet = bytes(self.buffer[:400])
            self.frames_seen += 1
            if sum(packet[:-1]) & 255 != packet[-1]:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            del self.buffer[:400]
            # Only the verified 8x8 format is supported. Reject other modes.
            if packet[8] != 64:
                self.mode_rejected += 1
                continue
            distances = []
            for index in (27, 28, 35, 36):
                offset = 9 + index * 6
                distance = int.from_bytes(packet[offset:offset+3], 'little', signed=True) / 1e6
                # Match the MAVLink advertised range; never forward sub-minimum echoes.
                if packet[offset+3] == 0 and .02 <= distance <= 4.:
                    distances.append(distance)
            if len(distances) < 3 or max(distances)-min(distances) > .03:
                self.range_rejected += 1
                continue
            self.frames_valid += 1
            results.append((int.from_bytes(packet[4:8], 'little') / 1000.,
                            statistics.median(distances)))
        return results


class TofReceiptClock:
    """Receipt-only bench timestamps; device ticks used for ordering only.

    Observed device ticks do not track real milliseconds. No scale is guessed.
    Sensor processing and serial latency remain uncalibrated; not for fusion.
    """
    def __init__(self):
        self.offsets = deque(maxlen=150)
        self.last_boot = self.last_mono = self.last_ros = None
        self.reset = False

    def stamp(self, boot, ros, mono):
        self.reset = False
        if self.last_boot is not None:
            if boot == self.last_boot:
                return None
            if (boot < self.last_boot or mono-self.last_mono > .3
                    or abs((ros-self.last_ros)-(mono-self.last_mono)) > .05):
                self.offsets.clear()
                self.reset = True
        self.last_boot, self.last_ros, self.last_mono = boot, ros, mono
        self.offsets.append(ros-boot)
        if len(self.offsets) < 15:
            return None
        return ros
