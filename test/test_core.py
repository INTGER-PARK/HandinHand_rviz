import math
import numpy as np
import pytest
from hih_rviz.core import (BodyState, PathHistory, VelocityDerivative, arrow_endpoint,
                           cop_world, quaternion_rpy, rotation, rpy_from_quaternion,
                           unit_quaternion, world_acceleration)


@pytest.mark.parametrize('rpy', ([0., 0., 0.], [.2, -.3, 2.8], [-.4, .7, -2.2]))
def test_rpy_rotation_round_trip(rpy):
    q = quaternion_rpy(rpy)
    np.testing.assert_allclose(rpy_from_quaternion(q), rpy, atol=1e-12)
    R = rotation(q)
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
    assert np.linalg.det(R) == pytest.approx(1.)


def test_facing_drones_keep_their_own_cop_signs():
    follower = cop_world([-.4, 0., 1.], quaternion_rpy([0., 0., 0.]),
                         [.215, 0., 0.], [0., 0., 0.], .1, .05)
    leader = cop_world([.4, 0., 1.], quaternion_rpy([0., 0., math.pi]),
                       [.215, 0., 0.], [0., 0., 0.], .1, .05)
    np.testing.assert_allclose(follower, [-.185, .1, 1.05], atol=1e-12)
    np.testing.assert_allclose(leader, [.185, -.1, 1.05], atol=1e-12)


def test_tilted_hover_specific_force_is_not_world_acceleration():
    q = quaternion_rpy([.3, -.4, 1.2])
    gravity = np.array([0., 0., -9.81])
    imu = rotation(q).T @ -gravity
    np.testing.assert_allclose(world_acceleration(imu, q, 'body_specific_force', gravity), 0., atol=1e-12)
    np.testing.assert_array_equal(world_acceleration([1., 2., 3.], q, 'world_linear', gravity), [1., 2., 3.])


def test_body_angular_velocity_is_rotated_as_a_vector():
    R = rotation(quaternion_rpy([0., 0., math.pi / 2.]))
    np.testing.assert_allclose(R @ [1., 0., 0.], [0., 1., 0.], atol=1e-12)


def test_arrow_display_clip_keeps_direction_and_does_not_modify_input():
    v = np.array([-100., 0., 0.])
    np.testing.assert_allclose(arrow_endpoint([1., 2., 3.], v, .04, 1.), [0., 2., 3.])
    np.testing.assert_array_equal(v, [-100., 0., 0.])
    assert arrow_endpoint([1., 2., 3.], [0., 0., 0.], 1., 1.) is None


def test_bounded_history_and_time_reset():
    history = PathHistory(5, 2., .05, .01)
    for i in range(100):
        history.append(i*.1, [float(i), 0., 0.], [0., 0., 0., 1.])
    assert len(history.samples) == 5
    history.append(0., [0., 0., 0.], [0., 0., 0., 1.])
    assert len(history.samples) == 1
    history.prune(3.)
    assert not history.samples


def test_derivative_uses_simulation_time_and_rejects_first_reset_gap():
    diff = VelocityDerivative()
    assert diff.sample(1., np.zeros(6)) is None
    np.testing.assert_allclose(diff.sample(1.1, np.arange(6)*.1), np.arange(6), atol=1e-12)
    assert diff.sample(1.1, np.zeros(6)) is None
    assert diff.sample(.1, np.zeros(6)) is None
    assert diff.sample(2., np.zeros(6)) is None


@pytest.mark.parametrize('q', ([0., 0., 0., 0.], [math.nan, 0., 0., 1.]))
def test_invalid_quaternion_rejected(q):
    with pytest.raises(ValueError):
        unit_quaternion(q)


def test_invalid_state_cannot_reach_rviz():
    with pytest.raises(ValueError):
        BodyState([math.nan, 0., 0.], [0., 0., 0., 1.], [0., 0., 0.],
                  [0., 0., 0.], None, 1., 'test')
