"""Rate-limited position references, not a guarantee of aircraft speed."""
import math


class PositionRamp:
    def __init__(self, position, now):
        self.position = tuple(position)
        self.stamp = now

    def step(self, goal, now, horizontal_speed, vertical_speed, yaw_rate):
        if not all(math.isfinite(value) for value in (*goal, now)):
            raise ValueError('nonfinite position reference')
        # A delayed timer must not turn missed cycles into a large target jump.
        dt = min(.1, max(0., now - self.stamp))
        self.stamp = now
        x, y, z, yaw = self.position
        dx, dy = goal[0]-x, goal[1]-y
        distance = math.hypot(dx, dy)
        scale = min(1., horizontal_speed*dt/distance) if distance else 0.
        dz = max(-vertical_speed*dt, min(vertical_speed*dt, goal[2]-z))
        angle = math.atan2(math.sin(goal[3]-yaw), math.cos(goal[3]-yaw))
        angle = max(-yaw_rate*dt, min(yaw_rate*dt, angle))
        self.position = (x+dx*scale, y+dy*scale, z+dz,
                         math.atan2(math.sin(yaw+angle), math.cos(yaw+angle)))
        return self.position
