import numpy as np
import math

from spatialmath.base import rodrigues, r2t, tr2rpy, t2r


__all__ = [
    "deg2rad", "rad2deg", "transform_world_to_local",
]

def rad2deg(rad: float):
    return rad * 180.0 / math.pi

def deg2rad(deg: float):
    return deg * math.pi / 180.0


# ------- координатное преобразование ------------------
def transform_world_to_local(robot_node, global_pos, global_rot=None):
    tr = robot_node.getField("translation").getSFVec3f()
    rot = robot_node.getField("rotation").getSFRotation()
    T_wr = r2t(rodrigues(np.asarray(rot[:3])*rot[3])); T_wr[:3,3] = tr
    T_rw = np.linalg.inv(T_wr)
    p_loc = T_rw @ np.append(global_pos, 1.0)
    if global_rot is None:
        return p_loc[:3]
    R_wo = rodrigues(np.asarray(global_rot[:3])*global_rot[3])
    T_wo = r2t(R_wo)
    R_loc = t2r(T_rw @ T_wo)
    return p_loc[:3], tr2rpy(R_loc, order="xyz")