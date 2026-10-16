import importlib
import math
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
import ros_stubs as ros
from hih_rviz.core import quaternion_rpy, rotation
from hih_rviz.mjcf_geometry import MJCFGeometry

mujoco = pytest.importorskip('mujoco')


@pytest.fixture
def scene(tmp_path):
    # Test geometry uses the same native mechanisms, not copied private assets.
    drones = ''
    for role, x, quat in (('leader', '.5', '0 0 0 1'), ('follower', '-.5', '1 0 0 0')):
        servos = ''.join(f'''<body pos=".1 0 0">
            <joint name="{role}_j_servo{i}" type="hinge" axis="1 0 0"/>
            <geom name="{role}_motor{i}" type="cylinder" pos="0 0 -.08" size=".02 .01"/>
          </body>''' for i in range(1, 5))
        drones += f'''<body name="{role}_drone_base" pos="{x} 0 1" quat="{quat}">
          <freejoint/>
          <geom name="{role}_plate" type="box" pos=".212 0 0" size=".003 .2 .19" material="plate"/>
          <geom name="{role}_pipe" type="capsule" fromto="-.2 -.2 0 .2 .2 0" size=".01"/>
          <geom name="{role}_hidden" type="sphere" size=".02" group="4"/>
          {servos}</body>'''
    path = tmp_path / 'scene.xml'
    path.write_text(f'''<mujoco><compiler angle="radian"/>
      <asset><material name="plate" rgba=".1 .2 .3 .7"/></asset><worldbody>
      {drones}<body name="payload_contact_box" pos="0 0 1"><freejoint/>
      <geom type="box" size=".2 .25 .25"/></body></worldbody></mujoco>''')
    return path


def test_original_dimensions_material_and_group_selection(scene):
    model = MJCFGeometry(scene, [0, 1, 2])
    for role in ('leader', 'follower'):
        parts = model.parts(role, [0., 0., 0., 0.])
        plate = next(part for part in parts if part.name == role + '_plate')
        np.testing.assert_allclose(plate.scale, [.006, .4, .38])
        np.testing.assert_allclose(plate.position, [.212, 0., 0.], atol=1e-12)
        np.testing.assert_allclose(plate.rgba, [.1, .2, .3, .7], atol=1e-7)
        assert not any(part.name.endswith('_hidden') for part in parts)
    all_groups = MJCFGeometry(scene, [0, 1, 2, 3, 4, 5])
    assert any(part.name.endswith('_hidden') for part in all_groups.parts('leader', [0.]*4))


def test_servo_transform_matches_native_geometry_at_real_world_pose(scene):
    display = MJCFGeometry(scene, [0, 1, 2])
    angles = [.5, -.4, .3, -.2]
    parts = display.parts('leader', angles)
    native, data = display.model, mujoco.MjData(display.model)
    body = display.roots['leader']
    free = int(native.body_jntadr[body])
    address = int(native.jnt_qposadr[free])
    q = quaternion_rpy([.25, -.35, 1.2])
    p = np.array([2., -1., 3.])
    data.qpos[address:address+3], data.qpos[address+3:address+7] = p, q[[3, 0, 1, 2]]
    for i, address in display.joints['leader']:
        data.qpos[address] = angles[i]
    mujoco.mj_kinematics(native, data)
    for part in parts:
        if not part.name.startswith('leader_motor'):
            continue
        geom = mujoco.mj_name2id(native, mujoco.mjtObj.mjOBJ_GEOM, part.name)
        np.testing.assert_allclose(p + rotation(q) @ part.position, data.geom_xpos[geom], atol=1e-12)
        np.testing.assert_allclose(rotation(q) @ rotation(part.quaternion),
                                   data.geom_xmat[geom].reshape(3, 3), atol=1e-12)


def test_unknown_servo_pose_hides_moving_parts_and_capsule_preserves_endpoints(scene):
    display = MJCFGeometry(scene, [0, 1, 2])
    parts = display.parts('follower')
    assert not any('motor' in part.name for part in parts)
    pipe = [part for part in parts if part.name == 'follower_pipe']
    assert len(pipe) == 3
    ends = [part.position for part in pipe if part.kind == 'sphere']
    np.testing.assert_allclose(sorted(ends, key=lambda p:p[0]), [[-.2, -.2, 0.], [.2, .2, 0.]], atol=1e-12)
    assert display.data.time == 0.  # Kinematics never advances a second world.


def test_cache_is_bounded_under_sensor_noise(scene):
    display = MJCFGeometry(scene, [0, 1, 2])
    for i in range(200):
        display.parts('leader', [i*.0001]*4)
        display.parts('follower', [-i*.0001]*4)
    assert len(display.cache) == 2


def test_visualizer_uses_original_parts_and_state_servo_degrees(scene, monkeypatch):
    ros.install(monkeypatch)
    ros.Node.overrides = {'model_xml_path': str(scene)}
    module = importlib.import_module('hih_rviz.visualizer')
    importlib.reload(module)
    node = module.HIHVisualizer()
    msg = ros.MinitroneState(pos=[0., 0., 1.], acc=[0., 0., 9.81], servo=[30., -20., 10., 0.])
    node.on_state('follower', msg)
    node.tick()
    rendered = [m for m in node.marker_pub.messages[-1].markers if m.ns == 'follower/model']
    assert len(rendered) == len(node.geometry.parts('follower', np.deg2rad(msg.servo)))
    np.testing.assert_allclose(node.servo_angles['follower'][0], np.deg2rad(msg.servo))
    assert not any(m.ns in ('follower/rotors', 'follower/arms') for m in node.marker_pub.messages[-1].markers)
    node.on_joint_state('leader', NS(name=MJCFGeometry.servo_names('leader'), position=[.1, .2, .3, .4]))
    np.testing.assert_allclose(node.servo_angles['leader'][0], [.1, .2, .3, .4])


def test_geometry_uses_private_data_and_never_steps_physics(scene, monkeypatch):
    # Fail if drawing ever starts a physics step, including during construction.
    def forbid_dynamics(*args, **kwargs):
        pytest.fail('The visualizer must never advance physics')

    for name in ('mj_step', 'mj_step1', 'mj_step2', 'mj_forward'):
        monkeypatch.setattr(mujoco, name, forbid_dynamics)
    live_model = mujoco.MjModel.from_xml_path(str(scene))
    live_data = mujoco.MjData(live_model)
    live_data.qvel[:] = .3
    before_qpos, before_qvel = live_data.qpos.copy(), live_data.qvel.copy()
    display = MJCFGeometry(scene, [0, 1, 2])
    assert display.model is not live_model and display.data is not live_data
    display.parts('leader', [.1, .2, .3, .4])
    display.parts('follower', [-.1, -.2, -.3, -.4])
    np.testing.assert_array_equal(live_data.qpos, before_qpos)
    np.testing.assert_array_equal(live_data.qvel, before_qvel)
    assert display.data.time == live_data.time == 0.


@pytest.mark.parametrize('drone_source', ['measured', 'ground_truth'])
def test_original_plant_payload_output_reaches_model_tf_path_and_vectors(scene, monkeypatch, drone_source):
    """Run the original plant's helper and actual RViz subscriber callbacks."""
    import threading
    ros.install(monkeypatch)
    telemetry_module = pytest.importorskip('plant.payload_telemetry')
    importlib.reload(telemetry_module)
    ros.Node.overrides = {'model_xml_path': str(scene), 'drone_state_source': drone_source}
    module = importlib.import_module('hih_rviz.visualizer')
    importlib.reload(module)
    visualizer = module.HIHVisualizer()
    plant = ros.Node('cooperation_plant')
    plant.model = mujoco.MjModel.from_xml_path(str(scene))
    plant.data = mujoco.MjData(plant.model)
    plant.lock = threading.Lock()
    plant.payload_body = plant.model.body('payload_contact_box').id
    joint = int(plant.model.body_jntadr[plant.payload_body])
    qpos = int(plant.model.jnt_qposadr[joint])
    qvel = int(plant.model.jnt_dofadr[joint])
    position = [1., -.4, 1.6]
    q = quaternion_rpy([.2, -.3, 1.1])
    plant.data.qpos[qpos:qpos+7] = [*position, *q[[3, 0, 1, 2]]]
    plant.data.qvel[qvel:qvel+6] = [.3, -.2, .1, .1, .2, -.3]
    mujoco.mj_forward(plant.model, plant.data)
    telemetry = telemetry_module.PayloadTelemetry(plant)
    subscribers = {sub.topic: sub.callback for sub in visualizer.subscriptions}
    assert telemetry_module.ODOMETRY_TOPIC in subscribers
    assert telemetry_module.ACCELERATION_TOPIC in subscribers
    telemetry.publish()
    subscribers[telemetry_module.ODOMETRY_TOPIC](telemetry.odometry_pub.messages[-1])

    # The second actual sample supplies a known World linear acceleration.
    plant.data.time += .1
    plant.data.qvel[qvel:qvel+3] += [.02, -.03, .04]
    mujoco.mj_forward(plant.model, plant.data)
    before = plant.data.qpos.copy(), plant.data.qvel.copy(), plant.data.ctrl.copy()
    telemetry.publish()
    subscribers[telemetry_module.ODOMETRY_TOPIC](telemetry.odometry_pub.messages[-1])
    subscribers[telemetry_module.ACCELERATION_TOPIC](telemetry.acceleration_pub.messages[-1])
    visualizer.tick()

    np.testing.assert_allclose(visualizer.states['payload'].position, position)
    np.testing.assert_allclose(visualizer.states['payload'].velocity_world, [.32, -.23, .14])
    np.testing.assert_allclose(visualizer.states['payload'].omega_body, [.1, .2, -.3], atol=1e-12)
    markers = visualizer.marker_pub.messages[-1].markers
    model_marker = next(m for m in markers if m.ns == 'payload/model' and m.action == ros.Marker.ADD)
    np.testing.assert_allclose([model_marker.pose.position.x, model_marker.pose.position.y,
                               model_marker.pose.position.z], position)
    orientation = model_marker.pose.orientation
    np.testing.assert_allclose(rotation([orientation.x, orientation.y, orientation.z, orientation.w]),
                               rotation(q), atol=1e-12)
    assert 'payload: ground_truth' in visualizer.status_pub.messages[-1].data
    assert any(m.ns == 'payload/state' for m in markers)
    assert any(m.ns == 'payload/velocity' for m in markers)
    assert any(m.ns == 'payload/angular_velocity' for m in markers)
    transform = next(msg for msg in visualizer.tf.messages if msg.child_frame_id == 'hih_viz/payload/base_link')
    assert transform.header.frame_id == 'world'
    np.testing.assert_allclose([transform.transform.translation.x, transform.transform.translation.y,
                               transform.transform.translation.z], position)
    path = visualizer.publishers_by_role['payload']['path'].messages[-1]
    assert path.poses[-1].pose.position.x == pytest.approx(position[0])
    acceleration = visualizer.publishers_by_role['payload']['acceleration'].messages[-1]
    np.testing.assert_allclose([acceleration.vector.x, acceleration.vector.y, acceleration.vector.z],
                               [.2, -.3, .4], atol=1e-12)
    assert all(topic.startswith('/hih_viz/') for topic in visualizer.publishers)
    np.testing.assert_array_equal(plant.data.qpos, before[0])
    np.testing.assert_array_equal(plant.data.qvel, before[1])
    np.testing.assert_array_equal(plant.data.ctrl, before[2])
