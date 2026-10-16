"""Convert the installed simulator's compiled MJCF geometry to RViz primitives.

MuJoCo resolves includes, defaults, fromto, material and joint transforms. This
private MjData is used only for forward KINEMATICS: no dynamics or simulation
step runs here, and the live plant model/data are never accessed or modified.
"""
from dataclasses import dataclass
from pathlib import Path
import numpy as np

from .core import finite_vector


@dataclass
class GeometryPart:
    name: str
    marker_id: int
    kind: str
    position: np.ndarray
    quaternion: np.ndarray
    scale: np.ndarray
    rgba: np.ndarray


class MJCFGeometry:
    def __init__(self, xml_path, groups):
        import mujoco
        self.mj = mujoco
        self.xml_path = str(Path(xml_path).resolve())
        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.data = mujoco.MjData(self.model)
        self.roots = {}
        for role, name in (('leader', 'leader_drone_base'), ('follower', 'follower_drone_base'),
                           ('payload', 'payload_contact_box')):
            body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if body < 0:
                raise ValueError('MJCF body missing: ' + name)
            self.roots[role] = body
        self.joints, self.servo_bodies = {}, set()
        for role in ('leader', 'follower'):
            self.joints[role] = []
            for i in range(1, 5):
                joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f'{role}_j_servo{i}')
                if joint >= 0:
                    self.joints[role].append((i-1, int(self.model.jnt_qposadr[joint])))
                    self.servo_bodies.add(int(self.model.jnt_bodyid[joint]))
        self.geoms = {role: [] for role in self.roots}
        self.moving_geoms = set()
        for geom in range(self.model.ngeom):
            body = int(self.model.geom_bodyid[geom])
            moving = False
            while body > 0:
                moving |= body in self.servo_bodies
                role = next((r for r, root in self.roots.items() if body == root), None)
                if role is not None:
                    if int(self.model.geom_group[geom]) in groups:
                        self.geoms[role].append(geom)
                        if moving:
                            self.moving_geoms.add(geom)
                    break
                body = int(self.model.body_parentid[body])
        mujoco.mj_kinematics(self.model, self.data)
        # Native visualization resolves material-vs-geom RGBA exactly as the
        # MuJoCo viewer does. This scene operation requires no OpenGL context.
        scene = mujoco.MjvScene(self.model, maxgeom=max(1000, self.model.ngeom*3))
        option = mujoco.MjvOption()
        option.geomgroup[:] = [int(i in groups) for i in range(6)]
        option.sitegroup[:] = 0
        mujoco.mjv_updateScene(self.model, self.data, option, None,
                              mujoco.MjvCamera(), mujoco.mjtCatBit.mjCAT_ALL, scene)
        self.colors = {int(g.objid): np.asarray(g.rgba).copy()
                       for g in scene.geoms[:scene.ngeom] if g.objtype == mujoco.mjtObj.mjOBJ_GEOM}
        self.cache = {}
        self.unsupported = set()

    @staticmethod
    def servo_names(role):
        return [f'{role}_j_servo{i}' for i in range(1, 5)]

    def parts(self, role, servo_angles=None):
        """Return Body-local parts. Unknown servo pose hides moving descendants."""
        if role in self.joints:
            angles = None if servo_angles is None else tuple(finite_vector(servo_angles, 4))
            key = (role, angles)
            if key in self.cache:
                return self.cache[key]
            if angles is not None:
                for i, address in self.joints[role]:
                    self.data.qpos[address] = angles[i]
                self.mj.mj_kinematics(self.model, self.data)
        else:
            angles, key = None, (role, None)
            if key in self.cache:
                return self.cache[key]
        root = self.roots[role]
        body_R = np.asarray(self.data.xmat[root]).reshape(3, 3)
        body_pos = np.asarray(self.data.xpos[root])
        parts = []
        for geom in self.geoms[role]:
            if angles is None and geom in self.moving_geoms:
                continue
            rgba = self.colors.get(geom)
            if rgba is None or rgba[3] <= 0.:
                continue
            center = body_R.T @ (self.data.geom_xpos[geom] - body_pos)
            R = body_R.T @ np.asarray(self.data.geom_xmat[geom]).reshape(3, 3)
            quat = np.zeros(4)
            self.mj.mju_mat2Quat(quat, R.reshape(9))
            q = quat[[1, 2, 3, 0]]
            size = np.asarray(self.model.geom_size[geom])
            kind = self.mj.mjtGeom(int(self.model.geom_type[geom]))
            name = self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_GEOM, geom) or str(geom)

            def add(part, shape, position, scale):
                parts.append(GeometryPart(name, geom*3+part, shape, np.asarray(position).copy(),
                                          q.copy(), np.asarray(scale).copy(), rgba.copy()))

            if kind == self.mj.mjtGeom.mjGEOM_BOX:
                add(0, 'box', center, 2.*size)
            elif kind == self.mj.mjtGeom.mjGEOM_CYLINDER:
                add(0, 'cylinder', center, [2.*size[0], 2.*size[0], 2.*size[1]])
            elif kind == self.mj.mjtGeom.mjGEOM_SPHERE:
                add(0, 'sphere', center, [2.*size[0]]*3)
            elif kind == self.mj.mjtGeom.mjGEOM_ELLIPSOID:
                add(0, 'sphere', center, 2.*size)
            elif kind == self.mj.mjtGeom.mjGEOM_CAPSULE:
                # RViz has no capsule: same cylinder and two hemispherical ends.
                if size[1] > 0.:
                    add(0, 'cylinder', center, [2.*size[0], 2.*size[0], 2.*size[1]])
                    add(1, 'sphere', center+R[:, 2]*size[1], [2.*size[0]]*3)
                    add(2, 'sphere', center-R[:, 2]*size[1], [2.*size[0]]*3)
                else:
                    add(0, 'sphere', center, [2.*size[0]]*3)
            else:
                self.unsupported.add(name + ': ' + kind.name)
        # Bound the cache: servo noise must not accumulate geometry indefinitely.
        for old in [old for old in self.cache if old[0] == role]:
            del self.cache[old]
        self.cache[key] = parts
        return parts
