import importlib
import math
from types import SimpleNamespace as NS
import numpy as np
import pytest

import ros_stubs as ros


@pytest.fixture
def node(monkeypatch):
    ros.install(monkeypatch)
    module = importlib.import_module('hih_rviz.visualizer')
    importlib.reload(module)
    return module.HIHVisualizer()


def state(yaw=0.):
    return ros.MinitroneState(pos=[0., 0., 1.], rpy=[0., 0., yaw],
                              acc=[0., 0., 9.81], vel=[1., 0., 0.], w_rpy=[1., 0., 0.])


def active_markers(node):
    return {m.ns: m for m in node.marker_pub.messages[-1].markers if m.action == ros.Marker.ADD}


def test_attach_without_payload_never_invents_a_payload_pose(node):
    node.on_state('leader', state())
    node.on_state('follower', state())
    node.tick()
    markers = active_markers(node)
    assert 'leader/state' in markers and 'follower/state' in markers
    assert 'payload/state' not in markers
    assert 'payload: waiting/stale' in node.status_pub.messages[-1].data


def test_outputs_never_command_a_vehicle(node):
    assert all(topic.startswith('/hih_viz/') for topic in node.publishers)
    assert all(not topic.endswith(('/cmd', '/input', '/admittance_enable')) for topic in node.publishers)
    inputs = {sub.topic for sub in node.subscriptions}
    assert inputs.isdisjoint(node.publishers)
    assert not any(pub.cls in (ros.Cmd, ros.MinitroneState, ros.Wrench, ros.CenterOfPressure)
                   for pub in node.publishers.values())


def test_receiving_telemetry_only_publishes_visualization_and_reads_parameters(node):
    # Deliver through every registered input, including command/force feedback.
    for sub in node.subscriptions:
        if sub.cls is ros.MinitroneState:
            msg = state()
        elif sub.cls is ros.Cmd:
            msg = ros.Cmd(pos_cmd=[1., 2., 3.])
        elif sub.cls is ros.Wrench:
            msg = ros.Wrench(force=[-5., 0., 0.])
        elif sub.cls is ros.CenterOfPressure:
            msg = ros.CenterOfPressure(valid=True, y=.05, z=.03, normal_force=5.)
        elif sub.cls is ros.Odometry:
            msg = ros.Odometry()
            msg.header.frame_id = 'world'
            msg.pose.pose.orientation.w = 1.
        elif sub.cls is ros.AccelStamped:
            msg = ros.AccelStamped()
            msg.header.frame_id = 'world'
        else:  # Optional contact diagnostic is not a command input.
            continue
        sub.callback(msg)
    for client in node.clients:
        assert client.cls is ros.GetParameters
        assert client.service.endswith('/get_parameters')
        client.ready = True
    node.poll_boundaries()
    node.tick()
    node.time += .1
    node.tick()
    node.clear_history(None, NS())
    assert all(topic.startswith('/hih_viz/') for topic in node.publishers)
    assert node.marker_pub.messages and node.tf.messages
    assert all(transform.child_frame_id.startswith('hih_viz/') for transform in node.tf.messages)
    assert len(node.services) == 1 and node.services[0].service == '~/clear_history'
    for client in node.clients:
        assert client.requests
        assert all(request.names == ['safe_y_m', 'safe_z_m'] for request in client.requests)


def test_body_omega_and_odom_twist_frame_are_correct(node):
    node.on_state('leader', state(math.pi/2.))
    node.tick()
    marker = active_markers(node)['leader/angular_velocity']
    tip = marker.points[-1]
    np.testing.assert_allclose([tip.x, tip.y, tip.z], [0., .15, 1.], atol=1e-12)
    odom = node.publishers_by_role['leader']['odometry'].messages[-1]
    np.testing.assert_allclose([odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                               odom.twist.twist.linear.z], [0., -1., 0.], atol=1e-12)
    assert odom.child_frame_id == 'hih_viz/leader/base_link'


def test_cop_invalid_and_stale_markers_are_deleted(node):
    node.on_state('follower', state())
    node.cache('follower', 'cop_mob', ros.CenterOfPressure(valid=True, y=.1, z=.05, normal_force=6.))
    node.tick()
    cop = active_markers(node)['follower/cop_mob']
    np.testing.assert_allclose([cop.pose.position.x, cop.pose.position.y, cop.pose.position.z], [.215, .1, 1.05])
    node.cache('follower', 'cop_mob', ros.CenterOfPressure(valid=False))
    node.tick()
    assert any(m.ns == 'follower/cop_mob' and m.action == ros.Marker.DELETE
               for m in node.marker_pub.messages[-1].markers)
    node.time += 1.
    node.tick()
    assert 'follower/state' not in active_markers(node)
    assert node.publishers_by_role['follower']['path'].messages[-1].poses == []


def test_force_anchors_keep_com_and_plate_moments_distinct(node):
    node.on_state('leader', state(math.pi))
    node.cache('leader', 'estimated', ros.Wrench(force=[-5., 0., 0.]))
    node.cache('leader', 'contact', ros.Wrench(force=[-5., 0., 0.]))
    node.tick()
    markers = active_markers(node)
    estimate = markers['leader/estimated_force']
    truth = markers['leader/contact_force']
    assert estimate.points[0].x == pytest.approx(0.)
    assert truth.points[0].x == pytest.approx(-.212)
    assert estimate.points[-1].x > estimate.points[0].x
    assert truth.points[-1].x > truth.points[0].x
    assert node.wrench_pubs[('leader', 'estimated')].messages[-1].header.frame_id.endswith('base_link')
    assert node.wrench_pubs[('leader', 'contact')].messages[-1].header.frame_id.endswith('plate_center')


def test_acceleration_is_gravity_corrected_and_nan_acc_keeps_pose(node):
    node.on_state('follower', state())
    node.tick()
    acc = node.publishers_by_role['follower']['acceleration'].messages[-1].vector
    np.testing.assert_allclose([acc.x, acc.y, acc.z], [0., 0., 0.])
    msg = state()
    msg.acc[0] = math.nan
    node.on_state('follower', msg)
    node.tick()
    assert 'follower/state' in active_markers(node)
    assert 'follower/acceleration' not in active_markers(node)


def test_live_safety_boundary_changes_without_cop_default_duplication(node):
    node.on_state('follower', state())
    node.tick()
    assert not any('/safe_' in name for name in active_markers(node))
    client = node.safety_clients[('follower', 'attitude')]
    client.ready, client.values = True, [.08, .09]
    node.poll_boundaries()
    node.tick()
    assert 'follower/safe_attitude' in active_markers(node)
    line = active_markers(node)['follower/safe_attitude']
    assert line.points[0].y == pytest.approx(-.08)
    client.values = [.06, .07]
    node.poll_boundaries()
    node.tick()
    assert active_markers(node)['follower/safe_attitude'].points[0].y == pytest.approx(-.06)
    client.ready = False
    node.poll_boundaries()
    node.tick()
    assert 'follower/safe_attitude' not in active_markers(node)


def test_payload_odom_twist_is_body_expressed(node):
    msg = ros.Odometry()
    msg.header.frame_id = 'world'
    msg.pose.pose.orientation.z = math.sqrt(.5)
    msg.pose.pose.orientation.w = math.sqrt(.5)
    msg.twist.twist.linear.x = 1.
    node.on_odometry('payload', msg)
    np.testing.assert_allclose(node.states['payload'].velocity_world, [0., 1., 0.], atol=1e-12)
    node.tick()
    assert 'payload/state' in active_markers(node)


def test_clock_reset_and_clear_history_service(node):
    node.on_state('leader', state())
    node.tick()
    assert node.histories[('leader', 'actual')].samples
    response = node.clear_history(None, NS())
    assert response.success and not node.histories[('leader', 'actual')].samples
    node.time -= 5.
    node.tick()
    assert not node.states


def test_reference_paths_are_separate_from_actual_and_admittance(node):
    node.on_state('follower', state())
    node.on_command('follower', 'reference', ros.Cmd(pos_cmd=[1., 2., 3.]))
    node.on_command('follower', 'admittance', ros.Cmd(pos_cmd=[4., 5., 6.]))
    node.tick()
    pubs = node.publishers_by_role['follower']
    assert pubs['path'].messages[-1].poses[-1].pose.position.x == 0.
    assert pubs['reference_path'].messages[-1].poses[-1].pose.position.x == 1.
    assert pubs['admittance_path'].messages[-1].poses[-1].pose.position.x == 4.
