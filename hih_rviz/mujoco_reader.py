"""Read one existing MuJoCo body's pose and world velocity; never step a model."""
import numpy as np


def body_snapshot(model, data, body_id):
    import mujoco
    spatial_velocity = np.zeros(6)
    # XBODY is the body origin/axes; BODY would use the inertial/CoM frame.
    # MuJoCo spatial vectors are [angular, linear], and flg_local=0 is World.
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY,
                            body_id, spatial_velocity, 0)
    w, x, y, z = np.asarray(data.xquat[body_id], dtype=float)
    return (float(data.time), np.asarray(data.xpos[body_id]).copy(),
            np.array([x, y, z, w]), spatial_velocity)
