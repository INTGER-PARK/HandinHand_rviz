"""Read simulator telemetry and publish TF, standard ROS messages and RViz markers.

This node never publishes to a simulator command, admittance enable or actuator
topic. Headerless hih_interfaces samples are stamped at visualization reception.
"""
import math
from pathlib import Path as FilePath
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import AccelStamped, PoseStamped, TransformStamped, Vector3Stamped, WrenchStamped
from hih_interfaces.msg import CenterOfPressure, Cmd, ContactDiagnostics, MinitroneState, Wrench
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .core import (BodyState, PathHistory, cop_world, finite_vector,
                   multiply_quaternions, quaternion_rpy, rotation,
                   rpy_from_quaternion, world_acceleration)
from .markers import Markers, xyzw, xyz
from .mjcf_geometry import MJCFGeometry
from .parameters import defaults

ROLES = ('leader', 'follower', 'payload')
COLORS = {'leader': (1., .48, .16, 1.), 'follower': (.22, .60, 1., 1.),
          'payload': (.75, .65, .95, .75)}
WHITE, MOB, SOLVER = (1., 1., 1., 1.), (1., .85, .1, 1.), (.1, .95, 1., 1.)


class HIHVisualizer(Node):
    def __init__(self):
        super().__init__('hih_rviz')
        self.cfg = {name: self.declare_parameter(
            name, value, ParameterDescriptor(read_only=True)).value
                    for name, value in defaults().items()}
        self._validate_config()
        self.geometry = None
        if self.cfg['show_models']:
            try:
                xml = self.cfg['model_xml_path'] or str(
                    FilePath(get_package_share_directory('hih_plant')) / 'xml/Cooperation.xml')
                self.geometry = MJCFGeometry(xml, self.cfg['model_geom_groups'])
                self.get_logger().info('Original simulator geometry: ' + self.geometry.xml_path)
            except (ImportError, OSError, ValueError, RuntimeError, LookupError) as error:
                self.get_logger().warning('Simulator geometry unavailable; overlays remain active: ' + str(error))
        self.servo_angles = {}
        self.states, self.feedback, self.commands, self.gt_acceleration = {}, {}, {}, {}
        self.boundaries, self.pending, self.last_warning = {}, {}, {}
        self.previous_markers = set()
        self.last_tick_time = self.seconds()
        self.histories = {(role, kind): PathHistory(
            self.cfg['history_max_points'], self.cfg['history_max_age_s'],
            self.cfg['history_interval_s'], self.cfg['history_min_distance_m'])
                          for role in ROLES for kind in ('actual', 'reference', 'admittance')}
        self.publishers_by_role = {}
        for role in ROLES:
            root = self.cfg['output_prefix'].rstrip('/') + '/' + role
            self.publishers_by_role[role] = {
                'odometry': self.create_publisher(Odometry, root + '/odometry', 2),
                'acceleration': self.create_publisher(Vector3Stamped, root + '/acceleration', 2),
                'path': self.create_publisher(Path, root + '/path', 2),
                'reference_path': self.create_publisher(Path, root + '/reference_path', 2),
                'admittance_path': self.create_publisher(Path, root + '/admittance_path', 2),
            }
        self.marker_pub = self.create_publisher(MarkerArray, self.cfg['output_prefix'] + '/markers', 2)
        self.status_pub = self.create_publisher(String, self.cfg['output_prefix'] + '/status', 2)
        self.wrench_pubs = {}
        self.tf = TransformBroadcaster(self)
        self.subscriptions_owned = []
        self.safety_clients = {}
        for role in ROLES:
            if role != 'payload' and self.cfg['drone_state_source'] == 'measured':
                self._subscribe(MinitroneState, 'state_topic_template', role,
                                lambda msg, r=role: self.on_state(r, msg))
            else:
                self._subscribe(Odometry, 'ground_truth_odom_topic_template', role,
                                lambda msg, r=role: self.on_odometry(r, msg))
                self._subscribe(AccelStamped, 'ground_truth_acceleration_topic_template', role,
                                lambda msg, r=role: self.on_acceleration(r, msg))
                if role != 'payload':
                    self._subscribe(JointState, 'ground_truth_joint_state_topic_template', role,
                                    lambda msg, r=role: self.on_joint_state(r, msg))
            if role == 'payload':
                continue
            for parameter, kind in (
                    ('cop_mob_topic_template', 'cop_mob'), ('cop_solver_topic_template', 'cop_solver')):
                self._subscribe(CenterOfPressure, parameter, role,
                                lambda msg, r=role, k=kind: self.cache(r, k, msg))
            for parameter, kind in (
                    ('estimated_wrench_topic_template', 'estimated'),
                    ('contact_wrench_topic_template', 'contact')):
                self._subscribe(Wrench, parameter, role,
                                lambda msg, r=role, k=kind: self.cache(r, k, msg))
                self.wrench_pubs[(role, kind)] = self.create_publisher(
                    WrenchStamped, self.cfg['output_prefix'] + f'/{role}/wrench_{kind}', 2)
            for parameter, kind in (
                    ('command_topic_template', 'reference'),
                    ('admittance_command_topic_template', 'admittance')):
                self._subscribe(Cmd, parameter, role,
                                lambda msg, r=role, k=kind: self.on_command(r, k, msg))
            self._subscribe(ContactDiagnostics, 'contact_diagnostics_topic_template', role,
                            lambda msg, r=role: self.cache(r, 'contacts', msg))
            self._subscribe(Wrench, 'payload_face_wrench_topic_template', role,
                            lambda msg, r=role: self.cache('payload', r, msg))
            for kind in ('attitude', 'translation'):
                remote = self.cfg[f'cop_{kind}_node_template'].format(role=role)
                # Only a GET client exists: no remote parameter write interface.
                self.safety_clients[(role, kind)] = self.create_client(
                    GetParameters, remote.rstrip('/') + '/get_parameters')
        self.subscriptions_owned.append(self.create_subscription(
            Wrench, self.cfg['payload_wrench_topic'],
            lambda msg: self.cache('payload', 'contact', msg), qos_profile_sensor_data))
        self.wrench_pubs[('payload', 'contact')] = self.create_publisher(
            WrenchStamped, self.cfg['output_prefix'] + '/payload/wrench_contact', 2)
        self.clear_service = self.create_service(Trigger, '~/clear_history', self.clear_history)
        self.safety_timer = self.create_timer(self.cfg['safety_poll_period_s'], self.poll_boundaries)
        self.timer = self.create_timer(1. / self.cfg['publish_rate_hz'], self.tick)
        self.get_logger().info('Read-only visualization attached; outputs under ' + self.cfg['output_prefix'])

    def _validate_config(self):
        for key in ('publish_rate_hz', 'state_timeout_s', 'feedback_timeout_s',
                    'marker_lifetime_s', 'history_max_age_s', 'history_interval_s',
                    'safety_poll_period_s', 'max_arrow_length_m', 'velocity_scale',
                    'acceleration_scale', 'angular_velocity_scale', 'force_scale', 'moment_scale'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0.:
                raise ValueError(key + ' must be finite and positive')
        if not isinstance(self.cfg['history_max_points'], int) or self.cfg['history_max_points'] < 2:
            raise ValueError('history_max_points must be an integer >= 2')
        if self.cfg['drone_state_source'] not in ('measured', 'ground_truth'):
            raise ValueError('drone_state_source must be measured or ground_truth')
        if self.cfg['acceleration_mode'] not in ('body_specific_force', 'world_linear'):
            raise ValueError('Unknown acceleration_mode')
        for key in ('gravity_world', 'plate_center_body_m', 'plate_half_size_m',
                    'contact_face_center_body_m', 'contact_rpy_body_rad', 'payload_size_m'):
            finite_vector(self.cfg[key])
        for key in ('plate_half_size_m', 'payload_size_m'):
            if min(self.cfg[key]) <= 0.:
                raise ValueError(key + ' must be positive')
        for key in ('manual_safety_half_y_m', 'manual_safety_half_z_m', 'history_min_distance_m'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] < 0.:
                raise ValueError(key + ' must be finite and nonnegative')
        if (self.cfg['manual_safety_half_y_m'] > self.cfg['plate_half_size_m'][1] or
                self.cfg['manual_safety_half_z_m'] > self.cfg['plate_half_size_m'][2]):
            raise ValueError('Manual safety boundary must fit inside the physical plate')
        for key in ('world_frame', 'frame_prefix'):
            if not self.cfg[key] or self.cfg[key].startswith('/'):
                raise ValueError(key + ' must be a nonempty TF frame without leading /')
        if not self.cfg['output_prefix'].startswith('/'):
            raise ValueError('output_prefix must be an absolute ROS topic prefix')
        if not self.cfg['model_geom_groups'] or any(
                not isinstance(group, int) or not 0 <= group <= 5 for group in self.cfg['model_geom_groups']):
            raise ValueError('model_geom_groups must contain group indices in [0, 5]')

    def _subscribe(self, cls, parameter, role, callback):
        topic = self.cfg[parameter].format(role=role)
        self.subscriptions_owned.append(self.create_subscription(cls, topic, callback, qos_profile_sensor_data))

    def seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def frame(self, role, suffix='base_link'):
        return self.cfg['frame_prefix'].rstrip('/') + '/' + role + '/' + suffix

    def warn(self, key, text):
        now = self.seconds()
        if now - self.last_warning.get(key, -math.inf) >= 5.:
            self.get_logger().warning(text)
            self.last_warning[key] = now

    def _accept_state(self, role, state):
        old = self.states.get(role)
        if old and (state.received < old.received or
                    state.received - old.received > self.cfg['state_timeout_s']):
            self.histories[(role, 'actual')].clear()
        self.states[role] = state

    def on_state(self, role, msg):
        try:
            try:
                self.servo_angles[role] = (np.deg2rad(finite_vector(msg.servo, 4)), self.seconds())
            except (AttributeError, ValueError):
                pass
            q = quaternion_rpy(msg.rpy)
            try:
                acc = world_acceleration(msg.acc, q, self.cfg['acceleration_mode'], self.cfg['gravity_world'])
            except ValueError:
                acc = None
            self._accept_state(role, BodyState(msg.pos, q, msg.vel, msg.w_rpy, acc, self.seconds(), 'measured'))
        except ValueError:
            self.warn(role, role + ': invalid state ignored')

    def on_joint_state(self, role, msg):
        try:
            values = [msg.position[msg.name.index(name)] for name in MJCFGeometry.servo_names(role)]
            self.servo_angles[role] = (finite_vector(values, 4), self.seconds())
        except (ValueError, IndexError):
            self.warn(role + '_servo', role + ': invalid/missing GT servo joint positions')

    def on_odometry(self, role, msg):
        if msg.header.frame_id != self.cfg['world_frame']:
            self.warn(role, role + ': GT odometry frame must match world_frame')
            return
        try:
            pose, twist = msg.pose.pose, msg.twist.twist
            q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            v_body = [twist.linear.x, twist.linear.y, twist.linear.z]
            w_body = [twist.angular.x, twist.angular.y, twist.angular.z]
            self._accept_state(role, BodyState(
                [pose.position.x, pose.position.y, pose.position.z], q, rotation(q) @ finite_vector(v_body),
                w_body, None, self.seconds(), 'ground_truth'))
        except ValueError:
            self.warn(role, role + ': invalid GT odometry ignored')

    def on_acceleration(self, role, msg):
        if msg.header.frame_id != self.cfg['world_frame']:
            return
        try:
            self.gt_acceleration[role] = (finite_vector(
                [msg.accel.linear.x, msg.accel.linear.y, msg.accel.linear.z]), self.seconds())
        except ValueError:
            pass

    def cache(self, role, kind, msg):
        self.feedback[(role, kind)] = (msg, self.seconds())

    def on_command(self, role, kind, msg):
        try:
            self.commands[(role, kind)] = (finite_vector(msg.pos_cmd), self.seconds())
        except ValueError:
            self.warn(role + kind, 'Non-finite reference ignored')

    def fresh_feedback(self, role, kind, now):
        cached = self.feedback.get((role, kind))
        if cached and 0. <= now - cached[1] <= self.cfg['feedback_timeout_s']:
            return cached[0]
        return None

    def clear_history(self, request, response):
        for history in self.histories.values():
            history.clear()
        response.success, response.message = True, 'Visualization histories cleared'
        return response

    def poll_boundaries(self):
        for key, client in self.safety_clients.items():
            if not client.service_is_ready():
                self.boundaries.pop(key, None)
                continue
            if key in self.pending and not self.pending[key].done():
                continue
            request = GetParameters.Request()
            request.names = ['safe_y_m', 'safe_z_m']
            future = client.call_async(request)
            self.pending[key] = future
            future.add_done_callback(lambda done, k=key: self.receive_boundary(k, done))

    def receive_boundary(self, key, future):
        try:
            values = future.result().values
            if len(values) != 2 or any(value.type not in (2, 3) for value in values):
                raise ValueError('Boundary parameters missing')
            y, z = [value.double_value if value.type == 3 else float(value.integer_value) for value in values]
            if not (math.isfinite(y) and math.isfinite(z) and
                    0. < y <= self.cfg['plate_half_size_m'][1] and
                    0. < z <= self.cfg['plate_half_size_m'][2]):
                raise ValueError('Boundary outside plate')
            self.boundaries[key] = (y, z, self.seconds())
        except (ValueError, AttributeError, TypeError, RuntimeError):
            self.boundaries.pop(key, None)

    def transform(self, parent, child, position, quaternion, stamp):
        msg = TransformStamped()
        msg.header.frame_id, msg.header.stamp, msg.child_frame_id = parent, stamp, child
        xyz(msg.transform.translation, position)
        xyzw(msg.transform.rotation, quaternion)
        return msg

    def publish_state(self, role, state, stamp, acc):
        msg = Odometry()
        msg.header.frame_id, msg.header.stamp = self.cfg['world_frame'], stamp
        msg.child_frame_id = self.frame(role)
        xyz(msg.pose.pose.position, state.position)
        xyzw(msg.pose.pose.orientation, state.quaternion)
        # Odometry twist belongs to child_frame_id, therefore BODY components.
        xyz(msg.twist.twist.linear, rotation(state.quaternion).T @ state.velocity_world)
        xyz(msg.twist.twist.angular, state.omega_body)
        self.publishers_by_role[role]['odometry'].publish(msg)
        if acc is not None:
            out = Vector3Stamped()
            out.header.frame_id, out.header.stamp = self.cfg['world_frame'], stamp
            xyz(out.vector, acc)
            self.publishers_by_role[role]['acceleration'].publish(out)

    def publish_path(self, role, kind, history, stamp, visible=True):
        out = Path()
        out.header.frame_id, out.header.stamp = self.cfg['world_frame'], stamp
        if visible:
            for _, position, quaternion in history.samples:
                pose = PoseStamped()
                pose.header.frame_id, pose.header.stamp = self.cfg['world_frame'], stamp
                xyz(pose.pose.position, position)
                xyzw(pose.pose.orientation, quaternion)
                out.poses.append(pose)
        key = {'actual': 'path', 'reference': 'reference_path', 'admittance': 'admittance_path'}[kind]
        self.publishers_by_role[role][key].publish(out)

    def draw_body(self, marks, role, state, acc):
        pos, q, color = state.position, state.quaternion, COLORS[role]
        R = rotation(q)
        world = lambda local: pos + R @ np.asarray(local)
        if self.cfg['show_models']:
            if self.geometry is not None:
                cached = self.servo_angles.get(role)
                servo = cached[0] if cached and 0. <= self.seconds()-cached[1] <= self.cfg['state_timeout_s'] else None
                kinds = {'box': Marker.CUBE, 'sphere': Marker.SPHERE, 'cylinder': Marker.CYLINDER}
                for part in self.geometry.parts(role, servo):
                    marks.shape(role + '/model', part.marker_id, kinds[part.kind], world(part.position),
                                multiply_quaternions(q, part.quaternion), part.scale, part.rgba)
                if self.geometry.unsupported:
                    self.warn('model_types', 'Unsupported MJCF geoms: ' + ', '.join(sorted(self.geometry.unsupported)))
        if self.cfg['show_axes']:
            for i, axis_color in enumerate(((1., .1, .1, 1.), (.1, 1., .1, 1.), (.1, .2, 1., 1.))):
                marks.arrow(role + '/axes', pos, R[:, i], .16, .16, axis_color, i)
        if self.cfg['show_kinematics']:
            marks.arrow(role + '/velocity', pos, state.velocity_world,
                        self.cfg['velocity_scale'], self.cfg['max_arrow_length_m'], (.2, 1., .25, 1.))
            marks.arrow(role + '/angular_velocity', pos, R @ state.omega_body,
                        self.cfg['angular_velocity_scale'], self.cfg['max_arrow_length_m'], (.9, .25, 1., 1.))
            if acc is not None:
                marks.arrow(role + '/acceleration', pos, acc, self.cfg['acceleration_scale'],
                            self.cfg['max_arrow_length_m'], (1., .25, .25, 1.))
        if self.cfg['show_labels']:
            fmt = lambda vector: ' '.join(f'{value:+.2f}' for value in vector)
            text = (f'{role.upper()} [{state.source}]\np W [m]: {fmt(pos)}\n'
                    f'RPY [deg]: {fmt(np.rad2deg(rpy_from_quaternion(q)))}\n'
                    f'v W [m/s]: {fmt(state.velocity_world)}\n'
                    f'a W [m/s2]: {fmt(acc) if acc is not None else "waiting"}\n'
                    f'omega B [rad/s]: {fmt(state.omega_body)}')
            marks.text(role + '/state', pos + [0., 0., .55], text, (*color[:3], 1.), .045)

    def draw_plate(self, marks, role, state, now):
        point = lambda y, z: cop_world(state.position, state.quaternion,
            self.cfg['contact_face_center_body_m'], self.cfg['contact_rpy_body_rad'], y, z)
        def rectangle(namespace, y, z, color):
            marks.line(namespace, [point(-y, -z), point(y, -z), point(y, z),
                                   point(-y, z), point(-y, -z)], color)
        rectangle(role + '/plate_boundary', *self.cfg['plate_half_size_m'][1:], WHITE)
        for kind, color in (('attitude', (.2, 1., .4, 1.)), ('translation', (.9, .4, 1., 1.))):
            boundary = self.boundaries.get((role, kind))
            if boundary and 0. <= now - boundary[2] < 3. * self.cfg['safety_poll_period_s']:
                rectangle(role + '/safe_' + kind, boundary[0], boundary[1], color)
        y, z = self.cfg['manual_safety_half_y_m'], self.cfg['manual_safety_half_z_m']
        if y > 0. and z > 0.:
            rectangle(role + '/safe_manual', y, z, (.5, 1., .5, 1.))
        for kind, color in (('cop_mob', MOB), ('cop_solver', SOLVER)):
            msg = self.fresh_feedback(role, kind, now)
            if msg is None or not msg.valid or not math.isfinite(msg.normal_force) or msg.normal_force <= 0.:
                continue
            try:
                position = point(msg.y, msg.z)
            except ValueError:
                continue
            marks.shape(role + '/' + kind, 0, Marker.SPHERE, position, [0., 0., 0., 1.],
                        [.022, .022, .022], color)
            if self.cfg['show_labels']:
                marks.text(role + '/' + kind + '_text', position + [0., 0., .04],
                           f'{kind}: y={msg.y:+.3f} z={msg.z:+.3f} m Fn={msg.normal_force:.2f} N', color, .032)
        if self.cfg['show_contact_points']:
            msg = self.fresh_feedback(role, 'contacts', now)
            if msg is not None:
                # Diagnostics origin is plate GEOM center (.212), not CoP face (.215).
                R = rotation(state.quaternion)
                origin = state.position + R @ np.asarray(self.cfg['plate_center_body_m'])
                count = min(int(msg.stored_contact_count), len(msg.contact_pos_x))
                for i in range(count):
                    if msg.efc_address[i] < 0 or msg.normal_force[i] <= 0.:
                        continue
                    local = [msg.contact_pos_x[i], msg.contact_pos_y[i], msg.contact_pos_z[i]]
                    try:
                        p = origin + R @ finite_vector(local)
                    except ValueError:
                        continue
                    marks.shape(role + '/contact_points', i, Marker.SPHERE, p, [0., 0., 0., 1.],
                                [.012, .012, .012], (1., .4, .7, 1.))

    def draw_wrench(self, marks, role, kind, msg, origin, R, color, stamp, frame):
        if msg is None:
            return
        try:
            force, moment = finite_vector(msg.force), finite_vector(msg.moment)
        except ValueError:
            return
        marks.arrow(role + '/' + kind + '_force', origin, R @ force,
                    self.cfg['force_scale'], self.cfg['max_arrow_length_m'], color)
        marks.arrow(role + '/' + kind + '_moment', origin, R @ moment,
                    self.cfg['moment_scale'], self.cfg['max_arrow_length_m'], (*color[:3], .65))
        if self.cfg['show_labels']:
            marks.text(role + '/' + kind + '_wrench', origin + [0., 0., -.30],
                       f'{kind}: |F|={np.linalg.norm(force):.2f} N |tau|={np.linalg.norm(moment):.2f} Nm',
                       color, .032)
        pub = self.wrench_pubs.get((role, kind))
        if pub is not None:
            out = WrenchStamped()
            out.header.frame_id, out.header.stamp = frame, stamp
            xyz(out.wrench.force, force)
            xyz(out.wrench.torque, moment)
            pub.publish(out)

    def tick(self):
        clock = self.get_clock().now()
        now, stamp = clock.nanoseconds * 1e-9, clock.to_msg()
        if now < self.last_tick_time:
            self.states.clear()
            self.feedback.clear()
            self.commands.clear()
            self.gt_acceleration.clear()
            self.servo_angles.clear()
            for history in self.histories.values():
                history.clear()
        self.last_tick_time = now
        marks = Markers(self.cfg['world_frame'], stamp, self.cfg['marker_lifetime_s'])
        transforms, status = [], []
        for role in ROLES:
            state = self.states.get(role)
            fresh = state is not None and 0. <= now - state.received <= self.cfg['state_timeout_s']
            for kind in ('actual', 'reference', 'admittance'):
                self.histories[(role, kind)].prune(now)
            if not fresh:
                status.append(role + ': waiting/stale')
                for kind in ('actual', 'reference', 'admittance'):
                    self.publish_path(role, kind, self.histories[(role, kind)], stamp, False)
                continue
            status.append(role + ': ' + state.source)
            if role != 'payload' and self.geometry is not None:
                servo = self.servo_angles.get(role)
                if not servo or not 0. <= now-servo[1] <= self.cfg['state_timeout_s']:
                    status.append(role + ': servo waiting/stale (moving model parts hidden)')
            acc = state.acceleration_world
            gt_acc = self.gt_acceleration.get(role)
            if gt_acc and 0. <= now - gt_acc[1] <= self.cfg['feedback_timeout_s']:
                acc = gt_acc[0]
            transforms.append(self.transform(self.cfg['world_frame'], self.frame(role),
                                             state.position, state.quaternion, stamp))
            self.publish_state(role, state, stamp, acc)
            self.histories[(role, 'actual')].append(state.received, state.position, state.quaternion)
            self.draw_body(marks, role, state, acc)
            R = rotation(state.quaternion)
            if role != 'payload':
                transforms.append(self.transform(self.frame(role), self.frame(role, 'contact'),
                    self.cfg['contact_face_center_body_m'], quaternion_rpy(self.cfg['contact_rpy_body_rad']), stamp))
                transforms.append(self.transform(self.frame(role), self.frame(role, 'plate_center'),
                    self.cfg['plate_center_body_m'], [0., 0., 0., 1.], stamp))
                self.draw_plate(marks, role, state, now)
                if self.cfg['show_estimated_wrench']:
                    self.draw_wrench(marks, role, 'estimated', self.fresh_feedback(role, 'estimated', now),
                                     state.position, R, MOB, stamp, self.frame(role))
                if self.cfg['show_contact_wrench']:
                    self.draw_wrench(marks, role, 'contact', self.fresh_feedback(role, 'contact', now),
                        state.position + R @ np.asarray(self.cfg['plate_center_body_m']),
                        R, SOLVER, stamp, self.frame(role, 'plate_center'))
                for kind in ('reference', 'admittance'):
                    command = self.commands.get((role, kind))
                    if command and 0. <= now - command[1] <= self.cfg['feedback_timeout_s']:
                        self.histories[(role, kind)].append(command[1], command[0], [0., 0., 0., 1.])
            elif self.cfg['show_contact_wrench']:
                self.draw_wrench(marks, role, 'contact', self.fresh_feedback(role, 'contact', now),
                                 state.position, R, SOLVER, stamp, self.frame(role))
                half_x = self.cfg['payload_size_m'][0] * .5
                for drone, x, yaw in (('follower', -half_x, 0.), ('leader', half_x, math.pi)):
                    sensor_R = R @ rotation(quaternion_rpy([0., 0., yaw]))
                    self.draw_wrench(marks, role, drone, self.fresh_feedback(role, drone, now),
                                     state.position + R @ [x, 0., 0.], sensor_R, COLORS[drone], stamp, '')
            for kind in ('actual', 'reference', 'admittance'):
                self.publish_path(role, kind, self.histories[(role, kind)], stamp)
        if transforms:
            self.tf.sendTransform(transforms)
        status_text = ' | '.join(status)
        if self.cfg['show_models'] and self.geometry is None:
            status_text += ' | MJCF model unavailable'
        marks.text('legend', [0., 0., 2.5], status_text +
                   '\nCoP/F: yellow=MoB cyan=solver | v=green a=red omega=purple', WHITE, .05)
        output, self.previous_markers = marks.finish(self.previous_markers)
        self.marker_pub.publish(output)
        msg = String()
        msg.data = status_text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HIHVisualizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
