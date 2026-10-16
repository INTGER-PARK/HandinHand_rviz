"""Small recording ROS transport for CPU-only tests; never used by the package.

Only ROS-generated messages/transport are replaced. Tests execute the real
visualizer/exporter callbacks and real MuJoCo math where MuJoCo is installed.
"""
import copy
from concurrent.futures import Future
from pathlib import Path as FilePath
from types import ModuleType, SimpleNamespace as NS


def vector():
    return NS(x=0., y=0., z=0.)


def quaternion():
    return NS(x=0., y=0., z=0., w=0.)


def header():
    return NS(frame_id='', stamp=NS(sec=0, nanosec=0))


def pose():
    return NS(position=vector(), orientation=quaternion())


class Msg:
    def __init__(self, **values):
        self.__dict__.update(values)


class PoseStamped:
    def __init__(self):
        self.header, self.pose = header(), pose()


class TransformStamped:
    def __init__(self):
        self.header, self.child_frame_id = header(), ''
        self.transform = NS(translation=vector(), rotation=quaternion())


class Odometry:
    def __init__(self):
        self.header, self.child_frame_id = header(), ''
        self.pose = NS(pose=pose(), covariance=[0.]*36)
        self.twist = NS(twist=NS(linear=vector(), angular=vector()), covariance=[0.]*36)


class Path:
    def __init__(self):
        self.header, self.poses = header(), []


class AccelStamped:
    def __init__(self):
        self.header, self.accel = header(), NS(linear=vector(), angular=vector())


class Vector3Stamped:
    def __init__(self):
        self.header, self.vector = header(), vector()


class WrenchStamped:
    def __init__(self):
        self.header, self.wrench = header(), NS(force=vector(), torque=vector())


class JointState:
    def __init__(self):
        self.header, self.name, self.position = header(), [], []


class Marker:
    ARROW, CUBE, SPHERE, CYLINDER, LINE_STRIP, TEXT_VIEW_FACING = 0, 1, 2, 3, 4, 9
    ADD, DELETE = 0, 2

    def __init__(self):
        self.header, self.pose, self.scale = header(), pose(), vector()
        self.color = NS(r=0., g=0., b=0., a=0.)
        self.lifetime = NS(sec=0, nanosec=0)
        self.points, self.ns, self.id, self.action, self.type = [], '', 0, 0, 0


class MinitroneState:
    def __init__(self, **values):
        for name in ('pos', 'vel', 'acc', 'rpy', 'w_rpy', 'a_rpy'):
            setattr(self, name, [0.]*3)
        self.servo = [0.]*4
        self.__dict__.update(values)


class Cmd:
    def __init__(self, **values):
        self.pos_cmd = [0.]*3
        self.__dict__.update(values)


class Wrench:
    def __init__(self, **values):
        self.force, self.moment = [0.]*3, [0.]*3
        self.__dict__.update(values)


class CenterOfPressure:
    def __init__(self, **values):
        self.valid, self.y, self.z, self.normal_force = False, 0., 0., 0.
        self.corner_forces = [0.]*4
        self.__dict__.update(values)


class Publisher:
    def __init__(self, topic):
        self.topic, self.messages = topic, []

    def publish(self, msg):
        self.messages.append(copy.deepcopy(msg))


class Node:
    overrides = {}

    def __init__(self, name):
        self.name, self.time, self.publishers, self.parameters = name, 10., {}, {}
        self.subscriptions, self.logs = [], []

    def declare_parameter(self, name, value, descriptor=None):
        self.parameters[name] = self.overrides.get(name, value)
        return NS(value=self.parameters[name])

    def create_publisher(self, cls, topic, qos):
        pub = Publisher(topic)
        self.publishers[topic] = pub
        return pub

    def create_subscription(self, cls, topic, callback, qos):
        sub = NS(topic=topic, callback=callback, cls=cls)
        self.subscriptions.append(sub)
        return sub

    def create_service(self, *args):
        return NS()

    def create_timer(self, *args):
        return NS()

    def get_clock(self):
        def now():
            nanos = round(self.time * 1e9)
            return NS(nanoseconds=nanos, to_msg=lambda: NS(sec=nanos//10**9, nanosec=nanos % 10**9))
        return NS(now=now)

    def get_logger(self):
        return NS(info=self.logs.append, warning=self.logs.append)


class AsyncParameterClient:
    def __init__(self, node, remote):
        self.remote, self.ready, self.values = remote, False, [.1, .1]

    def services_are_ready(self):
        return self.ready

    def get_parameters(self, names):
        future = Future()
        future.set_result(NS(values=[NS(type=3, double_value=v) for v in self.values]))
        return future


class TransformBroadcaster:
    def __init__(self, node):
        self.messages = []

    def sendTransform(self, transforms):
        self.messages.extend(copy.deepcopy(transforms))


def install(monkeypatch):
    definitions = {
        'rclpy': {'init': lambda **kwargs: None, 'shutdown': lambda: None},
        'rclpy.node': {'Node': Node},
        'rclpy.parameter_client': {'AsyncParameterClient': AsyncParameterClient},
        'rclpy.qos': {'qos_profile_sensor_data': object()},
        'rcl_interfaces.msg': {'ParameterDescriptor': Msg},
        'geometry_msgs.msg': {'Point': lambda: vector(), 'PoseStamped': PoseStamped,
                              'TransformStamped': TransformStamped, 'AccelStamped': AccelStamped,
                              'Vector3Stamped': Vector3Stamped, 'WrenchStamped': WrenchStamped},
        'nav_msgs.msg': {'Odometry': Odometry, 'Path': Path},
        'sensor_msgs.msg': {'JointState': JointState},
        'std_msgs.msg': {'String': Msg},
        'std_srvs.srv': {'Trigger': Msg},
        'tf2_ros': {'TransformBroadcaster': TransformBroadcaster},
        'visualization_msgs.msg': {'Marker': Marker, 'MarkerArray': Msg},
        'hih_interfaces.msg': {'MinitroneState': MinitroneState, 'Cmd': Cmd, 'Wrench': Wrench,
                               'CenterOfPressure': CenterOfPressure, 'ContactDiagnostics': Msg},
        'ament_index_python.packages': {
            'get_package_share_directory': lambda package: str(FilePath(__file__).resolve().parents[1])},
    }
    for name in list(definitions):
        if '.' in name:
            definitions.setdefault(name.split('.')[0], {})
    for name, attributes in definitions.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(__import__('sys').modules, name, module)
    Node.overrides = {}
