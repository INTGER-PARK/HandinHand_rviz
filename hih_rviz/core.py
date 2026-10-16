"""Frame math and bounded histories, independent of ROS and MuJoCo.

World is MuJoCo's right-handed Z-up frame. Quaternions use ROS xyzw order.
R_WB maps body components to world; RPY means Rz(yaw) Ry(pitch) Rx(roll).
"""
from collections import deque
from dataclasses import dataclass
import math
from typing import Optional

import numpy as np


def finite_vector(values, size=3):
    value = np.asarray(values, dtype=float)
    if value.shape != (size,) or not np.isfinite(value).all():
        raise ValueError(f'Expected {size} finite values')
    return value.copy()


def unit_quaternion(values):
    q = finite_vector(values, 4)
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        raise ValueError('Zero quaternion')
    return q / norm


def quaternion_rpy(rpy):
    r, p, y = finite_vector(rpy) * 0.5
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                     cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy])


def rotation(q):
    x, y, z, w = unit_quaternion(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def multiply_quaternions(a, b):
    x, y, z, w = unit_quaternion(a)
    X, Y, Z, W = unit_quaternion(b)
    return unit_quaternion([w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z])


def rpy_from_quaternion(q):
    x, y, z, w = unit_quaternion(q)
    return np.array([math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                     math.asin(float(np.clip(2*(w*y-z*x), -1., 1.))),
                     math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))])


def world_acceleration(acc, q, mode, gravity):
    """Convert specific force f_B = R_BW(a_W-g_W), or accept linear a_W."""
    acc = finite_vector(acc)
    if mode == 'body_specific_force':
        return rotation(q) @ acc + finite_vector(gravity)
    if mode == 'world_linear':
        return acc
    raise ValueError('acceleration_mode must be body_specific_force or world_linear')


def cop_world(position, q, face_center_body, contact_rpy, y, z):
    """p_WCoP = p_WB + R_WB (r_BC + R_BC [0,y_C,z_C]). No sign flip."""
    tangent = finite_vector([0., y, z])
    return (finite_vector(position) + rotation(q) @
            (finite_vector(face_center_body) + rotation(quaternion_rpy(contact_rpy)) @ tangent))


def arrow_endpoint(origin, vector, scale, maximum):
    origin, vector = finite_vector(origin), finite_vector(vector)
    delta = vector * scale
    length = float(np.linalg.norm(delta))
    if length < 1e-9:
        return None
    return origin + delta * min(1., maximum / length)


@dataclass
class BodyState:
    position: np.ndarray
    quaternion: np.ndarray
    velocity_world: np.ndarray
    omega_body: np.ndarray
    acceleration_world: Optional[np.ndarray]
    received: float
    source: str

    def __post_init__(self):
        self.position = finite_vector(self.position)
        self.quaternion = unit_quaternion(self.quaternion)
        self.velocity_world = finite_vector(self.velocity_world)
        self.omega_body = finite_vector(self.omega_body)
        if self.acceleration_world is not None:
            self.acceleration_world = finite_vector(self.acceleration_world)


class PathHistory:
    """Sample actual/command histories with hard memory and time bounds."""
    def __init__(self, max_points, max_age, interval, min_distance):
        self.samples = deque(maxlen=max_points)
        self.max_age, self.interval, self.min_distance = max_age, interval, min_distance

    def clear(self):
        self.samples.clear()

    def append(self, stamp, position, quaternion):
        position, quaternion = finite_vector(position), unit_quaternion(quaternion)
        if self.samples:
            previous_time, previous_pos, _ = self.samples[-1]
            if stamp < previous_time:
                self.clear()  # rosbag clock or simulator reset.
            elif stamp - previous_time < self.interval:
                return
            elif np.linalg.norm(position - previous_pos) < self.min_distance and stamp - previous_time < 1.:
                return
        self.samples.append((stamp, position, quaternion))
        self.prune(stamp)

    def prune(self, now):
        while self.samples and now - self.samples[0][0] > self.max_age:
            self.samples.popleft()


class VelocityDerivative:
    """Finite difference of WORLD linear/angular velocities using SIMULATION time.

    No acceleration is reported for the first sample, repeated time, reset or gap.
    These values are visualization derivatives, not independent sensors.
    """
    def __init__(self):
        self.previous = None

    def sample(self, sim_time, velocity):
        velocity = finite_vector(velocity, 6)
        old = self.previous
        self.previous = (float(sim_time), velocity)
        if old is None or not 1e-8 < sim_time - old[0] <= 1.:
            return None
        return (velocity - old[1]) / (sim_time - old[0])
