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


def to_xyzrpy(xyz: np.ndarray, rpy: np.ndarray):
    x, y, z = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
    r, p, yy = [float(rpy[0]), float(rpy[1]), float(rpy[2])]
    return [x, y, z, r, p, yy]


def split_xyz_rpy(p, default_rpy=None):
    a = np.asarray(p, dtype=float)
    if a.shape == (3,):
        xyz = a
        rpy = np.zeros(3) if default_rpy is None else np.asarray(default_rpy, dtype=float).reshape(3)
        return xyz, rpy
    if a.shape == (2, 3):
        xyz, rpy = a[0], a[1]
        return xyz.reshape(3), rpy.reshape(3)
    if a.size == 6:
        a = a.reshape(-1)
        return a[:3], a[3:6]
    raise ValueError(f"Неожиданный формат точки: shape={a.shape}, size={a.size}, value={a}")


def build_pick_place_commands(cords_list, 
                              target_pose, 
                              delta_z, 
                              rpy_pick_override=None,
                              place_step_z=0.05,
                              gap=0.002,
                              start_index=0):
    cmds = []
    target_pose = np.asarray(target_pose, dtype=float).reshape(-1)
    base_xyz  = target_pose[:3].copy()
    rpy_place = target_pose[3:].copy()

    for i, p in enumerate(cords_list, start=start_index):
        # разбор входа
        xyz, rpy_pick_auto = split_xyz_rpy(p, default_rpy=rpy_place)
        rpy_pick = rpy_pick_auto if rpy_pick_override is None else np.asarray(rpy_pick_override, float)
        
        # высота для текущего слоя (поднимаем z на i * step + gap)
        place_xyz = base_xyz.copy()
        place_xyz[2] = float(base_xyz[2] + i * float(place_step_z) + float(gap))

        # точки подлёта
        pre_pick  = xyz.copy()       
        pre_pick[2]  += float(delta_z)
        pre_place = place_xyz.copy() 
        pre_place[2] += float(delta_z)

        xyz[2] = abs(xyz[2] - 0.03)

        # подход к объекту
        cmds.append({"command": "move", "args": to_xyzrpy(pre_pick, rpy_pick)})
        cmds.append({"command": "move", "args": to_xyzrpy(xyz, rpy_pick)})
        cmds.append({"command": "grab", "args": True})
        cmds.append({"command": "move", "args": to_xyzrpy(pre_pick, rpy_pick)})

        # перенос и выкладка на слой i
        cmds.append({"command": "move", "args": to_xyzrpy(pre_place, rpy_place)})
        cmds.append({"command": "move", "args": to_xyzrpy(place_xyz, rpy_place)})

        # отпуск и безопасный уход вверх
        cmds.append({"command": "grab", "args": False})
        cmds.append({"command": "move", "args": to_xyzrpy(pre_place, rpy_place)})

    return cmds


def build_pick_place_pairwise(pick_list, target_poses, delta_z, rpy_pick_override=None):
    """
    pick_list: список целей подбора (XYZ | [XYZ,RPY] | XYZRPY)
    target_poses: список целевых поз [[x,y,z,a,b,c], ...] той же длины
    delta_z: подлёт
    """
    cmds_all = []
    for pick, tpose in zip(pick_list, target_poses):
        cmds = build_pick_place_commands(
            cords_list=[pick],
            target_pose=tpose,
            delta_z=delta_z,
            rpy_pick_override=rpy_pick_override,
            place_step_z=0.0,
            gap=0.0,
            start_index=0
        )
        cmds_all.extend(cmds)
    return cmds_all


def generate_pallet_poses(origin_xyz, base_rpy, nx, ny, nz, sx, sy, sz, order="zxy", limit=None):
    """
    origin_xyz: [x0,y0,z0] — центр первой ячейки (i=0,j=0,k=0)
    base_rpy:   [r,p,y]    — ориентация для всех ячеек
    nx,ny,nz:   размеры палеты в ячейках (int)
    sx,sy,sz:   шаги между ячейками (м)
    order:      порядок перебора индексов ('xyz','xzy','yxz','yzx','zxy','zyx')
    limit:      ограничить длину списка (например числом объектов)
    """
    idx_orders = {
        "xyz": ("i", "j", "k"),
        "xzy": ("i", "k", "j"),
        "yxz": ("j", "i", "k"),
        "yzx": ("j", "k", "i"),
        "zxy": ("k", "i", "j"),
        "zyx": ("k", "j", "i"),
    }
    if order not in idx_orders:
        order = "zxy"
    seq = idx_orders[order]

    ranges = {"i": range(nx), "j": range(ny), "k": range(nz)}

    poses = []
    for a in ranges[seq[0]]:
        for b in ranges[seq[1]]:
            for c in ranges[seq[2]]:
                idx_map = {seq[0]: a, seq[1]: b, seq[2]: c}
                i, j, k = idx_map["i"], idx_map["j"], idx_map["k"]
                x = origin_xyz[0] + i * sx
                y = origin_xyz[1] + j * sy
                z = origin_xyz[2] + k * sz
                poses.append([x, y, z, base_rpy[0], base_rpy[1], base_rpy[2]])
                if limit is not None and len(poses) >= limit:
                    return poses
    return poses

def aabb_from_bbox(node):
    bbox = node.getSFNode().getBBox()  # minX, minY, minZ, maxX, maxY, maxZ
    return bbox

def intersect(a, b):
    """True если AABB пересекаются"""
    return not (a[3] < b[0] or a[0] > b[3] or   # X
                a[4] < b[1] or a[1] > b[4] or   # Y
                a[5] < b[2] or a[2] > b[5])     # Z