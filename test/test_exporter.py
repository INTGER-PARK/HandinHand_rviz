import importlib
import threading
from types import SimpleNamespace as NS
import numpy as np
import pytest
import ros_stubs as ros


def test_exporter_reads_one_world_and_reuses_contact_sample(monkeypatch):
    mujoco = pytest.importorskip('mujoco')
    ros.install(monkeypatch)
    module = importlib.import_module('hih_rviz.plant_telemetry')
    importlib.reload(module)
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="leader" pos="1 0 1"><freejoint/><geom size=".1"/></body>
      <body name="follower" pos="-1 0 1"><freejoint/><geom size=".1"/></body>
      <body name="payload" pos="0 0 1"><freejoint/><geom size=".1"/></body>
    </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    node = ros.Node('cooperation_plant')
    node.model, node.data, node.lock, node.payload_body = model, data, threading.Lock(), 3
    diagnostic = NS(sim_time=0.)
    original_result = ('unchanged force', 'unchanged moment', diagnostic, 'cop', 'cone')
    calls = []
    def original():
        calls.append(1)
        return original_result
    node.vehicles = [NS(label=role, base=i, plate_payload_contact_sample=original)
                     for role, i in (('leader', 1), ('follower', 2))]
    exporter = module.GroundTruthExporter(node, 30., 'world')
    before = data.qpos.copy()
    exporter.publish()
    assert exporter.publishers['payload'][0].messages[-1].pose.pose.position.z == 1.
    assert exporter.publishers['payload'][1].messages == []  # No fake first acceleration.
    node.vehicles[0].plate_payload_contact_sample()
    result = node.vehicles[0].plate_payload_contact_sample()
    assert result is original_result and len(calls) == 2
    diag_pub = node.publishers['/leader/minitrone/contact_diagnostics']
    assert len(diag_pub.messages) == 1  # Same simulation time is throttled.
    data.time = .1
    data.qvel[12] = .2  # Payload World-X velocity.
    mujoco.mj_forward(model, data)
    exporter.publish()
    acc = exporter.publishers['payload'][1].messages[-1]
    assert acc.accel.linear.x == pytest.approx(2.)
    np.testing.assert_array_equal(before, data.qpos)
    # A simulator reset cannot produce an acceleration spike.
    count = len(exporter.publishers['payload'][1].messages)
    data.time = 0.
    exporter.publish()
    assert len(exporter.publishers['payload'][1].messages) == count
