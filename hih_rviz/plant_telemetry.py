"""Run the original CooperationPlant once and export its existing MjData.

Use this executable IN PLACE OF cooperation_plant, never alongside another plant.
Original physics, controllers, noise, force/CoP definitions and keyboard behavior
remain owned by HandinHand_sim. The only new outputs are visualization telemetry.
"""
import math
import numpy as np
import rclpy
from geometry_msgs.msg import AccelStamped
from hih_interfaces.msg import ContactDiagnostics
from nav_msgs.msg import Odometry

from .core import VelocityDerivative, finite_vector, rotation
from .markers import xyzw, xyz
from .mujoco_reader import body_snapshot
from .parameters import defaults


class GroundTruthExporter:
    def __init__(self, node, rate, world_frame):
        self.node, self.world_frame = node, world_frame
        self.bodies = {vehicle.label: vehicle.base for vehicle in node.vehicles}
        self.bodies['payload'] = node.payload_body
        self.derivatives = {role: VelocityDerivative() for role in self.bodies}
        self.publishers = {
            role: (node.create_publisher(Odometry, f'/hih/ground_truth/{role}/odometry', 2),
                   node.create_publisher(AccelStamped, f'/hih/ground_truth/{role}/acceleration', 2))
            for role in self.bodies}
        for vehicle in node.vehicles:
            self.tap_contacts(vehicle, rate)
        self.timer = node.create_timer(1. / rate, self.publish)

    def tap_contacts(self, vehicle, rate):
        """Reuse the contact diagnostic already computed by the original plant.

        No second contact traversal and no extra mj_step/mj_forward. The wrapper
        publishes at most `rate` times per simulation second, at the original
        diagnostic's step/time. This topic is not published by current main.
        """
        original = vehicle.plate_payload_contact_sample
        pub = self.node.create_publisher(
            ContactDiagnostics, f'/{vehicle.label}/minitrone/contact_diagnostics', 2)
        last_time = -math.inf

        def sample_and_publish():
            nonlocal last_time
            result = original()
            diagnostic = result[-3]
            sim_time = float(diagnostic.sim_time)
            if sim_time < last_time or sim_time - last_time >= 1. / rate:
                pub.publish(diagnostic)
                last_time = sim_time
            return result

        # Construction occurs while the owning plant lock is held.
        vehicle.plate_payload_contact_sample = sample_and_publish

    def publish(self):
        # Read this SAME running world. Hold the existing lock only for copies.
        with self.node.lock:
            snapshots = {role: body_snapshot(self.node.model, self.node.data, body_id)
                         for role, body_id in self.bodies.items()}
        stamp = self.node.get_clock().now().to_msg()
        for role, (sim_time, position, q, velocity_world) in snapshots.items():
            try:
                if not math.isfinite(sim_time):
                    continue
                R = rotation(q)
                finite_vector(position)
                finite_vector(velocity_world, 6)
            except ValueError:
                continue  # An invalid display sample must not stop the plant.
            out = Odometry()
            out.header.frame_id, out.header.stamp = self.world_frame, stamp
            out.child_frame_id = 'hih_gt/' + role
            xyz(out.pose.pose.position, position)
            xyzw(out.pose.pose.orientation, q)
            xyz(out.twist.twist.linear, R.T @ velocity_world[3:])
            xyz(out.twist.twist.angular, R.T @ velocity_world[:3])
            self.publishers[role][0].publish(out)
            acceleration = self.derivatives[role].sample(sim_time, velocity_world)
            if acceleration is not None:
                msg = AccelStamped()
                msg.header.frame_id, msg.header.stamp = self.world_frame, stamp
                xyz(msg.accel.linear, acceleration[3:])
                xyz(msg.accel.angular, acceleration[:3])
                self.publishers[role][1].publish(msg)


def main(args=None):
    # Preserve the current entry point's optional slip diagnostic augmentation.
    from plant import cooperation_plant_diagnostics
    cooperation_plant_diagnostics.install()
    from plant.cooperation_plant import CooperationPlant
    rclpy.init(args=args)
    node = None
    try:
        node = CooperationPlant()
        cfg = defaults()
        rate = float(node.declare_parameter('telemetry_rate_hz', cfg['publish_rate_hz']).value)
        world = node.declare_parameter('telemetry_world_frame', cfg['world_frame']).value
        if not math.isfinite(rate) or rate <= 0.:
            raise ValueError('telemetry_rate_hz must be finite and positive')
        with node.lock:
            exporter = GroundTruthExporter(node, rate, world)
        node.get_logger().info('Same-world GT telemetry: /hih/ground_truth/{leader,follower,payload}/odometry')
        rclpy.spin(node)
        del exporter
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()
