"""Synthetic visualization-only telemetry. Does not run flight controllers."""
import math
import numpy as np
import rclpy
from geometry_msgs.msg import AccelStamped
from hih_interfaces.msg import CenterOfPressure, Cmd, MinitroneState, Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .core import quaternion_rpy, rotation
from .markers import xyz, xyzw


class Demo(Node):
    def __init__(self):
        super().__init__('hih_visualization_demo')
        self.start = self.get_clock().now().nanoseconds * 1e-9
        self.pubs = {}
        for role in ('leader', 'follower'):
            for suffix, cls in (('state', MinitroneState), ('cmd', Cmd),
                                ('cop_hat', CenterOfPressure), ('cop_solver_real', CenterOfPressure),
                                ('external_wrench_hat_second_order', Wrench), ('contact_wrench_real', Wrench)):
                self.pubs[(role, suffix)] = self.create_publisher(cls, f'/{role}/minitrone/{suffix}', 2)
        self.payload_odom = self.create_publisher(Odometry, '/hih/ground_truth/payload/odometry', 2)
        self.payload_acc = self.create_publisher(AccelStamped, '/hih/ground_truth/payload/acceleration', 2)
        self.payload_wrench = self.create_publisher(Wrench, '/payload/contact_wrench_from_drones', 2)
        self.timer = self.create_timer(1./30., self.tick)
        self.get_logger().info('SYNTHETIC RViz demonstration; run separately from the real simulator')

    def tick(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9 - self.start
        y, z = .25*math.sin(.4*t), 1.3+.12*math.sin(.8*t)
        velocity = [0., .1*math.cos(.4*t), .096*math.cos(.8*t)]
        acceleration = np.array([0., -.04*math.sin(.4*t), -.0768*math.sin(.8*t)])
        for role, x, yaw in (('leader', .415, math.pi), ('follower', -.415, 0.)):
            pitch = .08*math.sin(.5*t)
            q = quaternion_rpy([0., pitch, yaw])
            msg = MinitroneState()
            msg.pos, msg.vel, msg.rpy = [x, y, z], velocity, [0., pitch, yaw]
            msg.acc = (rotation(q).T @ (acceleration - [0., 0., -9.81])).tolist()
            msg.w_rpy = [0., .04*math.cos(.5*t), 0.]
            self.pubs[(role, 'state')].publish(msg)
            cmd = Cmd()
            cmd.pos_cmd = [x, .25*math.sin(.4*t+.12), z]
            self.pubs[(role, 'cmd')].publish(cmd)
            for suffix, phase in (('cop_hat', .1), ('cop_solver_real', 0.)):
                cop = CenterOfPressure()
                cop.y, cop.z = .13*math.sin(.7*t+phase), .1*math.cos(.5*t+phase)
                cop.valid, cop.normal_force = True, 6.
                self.pubs[(role, suffix)].publish(cop)
            for suffix, normal in (('external_wrench_hat_second_order', -6.1), ('contact_wrench_real', -6.)):
                force = Wrench()
                force.force = [normal, .25*math.sin(t), -1.47]
                force.moment = [0., .05*math.sin(t), .1*math.cos(t)]
                self.pubs[(role, suffix)].publish(force)
        odom = Odometry()
        odom.header.frame_id, odom.header.stamp, odom.child_frame_id = 'world', now.to_msg(), 'hih_gt/payload'
        xyz(odom.pose.pose.position, [0., y, z])
        xyzw(odom.pose.pose.orientation, [0., 0., 0., 1.])
        xyz(odom.twist.twist.linear, velocity)
        self.payload_odom.publish(odom)
        acc = AccelStamped()
        acc.header.frame_id, acc.header.stamp = 'world', now.to_msg()
        xyz(acc.accel.linear, acceleration)
        self.payload_acc.publish(acc)
        force = Wrench()
        force.force = [0., 0., 2.94]
        self.payload_wrench.publish(force)


def main(args=None):
    rclpy.init(args=args)
    node = Demo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
