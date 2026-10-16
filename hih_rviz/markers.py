"""RViz marker constructors. All generated geometry is expressed in World."""
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from .core import arrow_endpoint, finite_vector, unit_quaternion


def xyz(target, values):
    target.x, target.y, target.z = map(float, values)


def xyzw(target, values):
    target.x, target.y, target.z, target.w = map(float, values)


def point(values):
    msg = Point()
    xyz(msg, values)
    return msg


class Markers:
    def __init__(self, frame, stamp, lifetime):
        self.frame, self.stamp, self.lifetime = frame, stamp, lifetime
        self.items = []

    def shape(self, namespace, marker_id, kind, position, quaternion, size, color):
        msg = Marker()
        msg.header.frame_id, msg.header.stamp = self.frame, self.stamp
        msg.ns, msg.id, msg.type, msg.action = namespace, marker_id, kind, Marker.ADD
        xyz(msg.pose.position, finite_vector(position))
        xyzw(msg.pose.orientation, unit_quaternion(quaternion))
        xyz(msg.scale, finite_vector(size))
        msg.color.r, msg.color.g, msg.color.b, msg.color.a = map(float, color)
        nanos = round(self.lifetime * 1e9)
        msg.lifetime.sec, msg.lifetime.nanosec = divmod(nanos, 1_000_000_000)
        self.items.append(msg)
        return msg

    def arrow(self, namespace, origin, vector, scale, maximum, color, marker_id=0):
        endpoint = arrow_endpoint(origin, vector, scale, maximum)
        if endpoint is None:
            return
        length = float(np.linalg.norm(endpoint - origin))
        msg = self.shape(namespace, marker_id, Marker.ARROW, [0., 0., 0.], [0., 0., 0., 1.],
                         [.012, .025, min(.045, .35 * length)], color)
        msg.points = [point(origin), point(endpoint)]

    def line(self, namespace, points, color, width=.006, marker_id=0):
        msg = self.shape(namespace, marker_id, Marker.LINE_STRIP, [0., 0., 0.],
                         [0., 0., 0., 1.], [width, 0., 0.], color)
        msg.points = [point(p) for p in points]

    def text(self, namespace, position, text, color, height=.055, marker_id=0):
        msg = self.shape(namespace, marker_id, Marker.TEXT_VIEW_FACING, position,
                         [0., 0., 0., 1.], [0., 0., height], color)
        msg.text = text

    def finish(self, previous):
        active = {(m.ns, m.id) for m in self.items}
        # Explicit deletes prevent zero/invalid/stale vectors leaving old arrows.
        for namespace, marker_id in previous - active:
            msg = Marker()
            msg.header.frame_id, msg.header.stamp = self.frame, self.stamp
            msg.ns, msg.id, msg.action = namespace, marker_id, Marker.DELETE
            self.items.append(msg)
        output = MarkerArray()
        output.markers = self.items
        return output, active
