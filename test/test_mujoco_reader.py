import numpy as np
import pytest
from hih_rviz.mujoco_reader import body_snapshot

mujoco = pytest.importorskip('mujoco')


def test_same_world_reader_keeps_body_origin_separate_from_offset_com():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="test" pos="1 2 3"><freejoint/>
        <inertial pos=".1 0 0" mass="1" diaginertia=".1 .1 .1"/>
        <geom type="sphere" size=".1"/>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    data.qvel[0:3] = [1., 2., 3.]
    data.qvel[3:6] = [0., 0., 2.]
    mujoco.mj_forward(model, data)
    before = [data.qpos.copy(), data.qvel.copy(), data.ctrl.copy(), float(data.time)]
    stamp, position, q, velocity = body_snapshot(model, data, 1)
    np.testing.assert_allclose(position, [1., 2., 3.])
    np.testing.assert_allclose(q, [0., 0., 0., 1.])
    np.testing.assert_allclose(velocity, [0., 0., 2., 1., 2., 3.])
    assert stamp == before[3]
    for actual, previous in zip((data.qpos, data.qvel, data.ctrl), before[:3]):
        np.testing.assert_array_equal(actual, previous)
