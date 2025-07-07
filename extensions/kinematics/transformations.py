import numpy as np
from spatialmath.base import rodrigues, r2t, tr2rpy, t2r


def transform_world_to_local(robot_node, global_pos, global_rot=None):
    """
    Переводит мировую позу (позицию + ориентацию) в локальные координаты робота.

    :param robot_node: Webots Node — DEF KUKA
    :param global_pos: [x, y, z] — позиция цели в мировой системе
    :param global_rot: [axis_x, axis_y, axis_z, angle] — ориентация цели (необязательно)
    :return: (позиция_локальная, ориентация_RPY_локальная) или только позиция, если global_rot не передан
    """
    # Положение и ориентация робота
    translation = robot_node.getField("translation").getSFVec3f()
    rotation = robot_node.getField("rotation").getSFRotation()
    R_world_robot = rodrigues(np.array(rotation[:3]) * rotation[3])
    T_world_robot = r2t(R_world_robot)
    T_world_robot[:3, 3] = translation

    # Инверсия: T_robot_world
    T_robot_world = np.linalg.inv(T_world_robot)

    # Позиция
    p_world = np.append(global_pos, 1.0)
    p_local = T_robot_world @ p_world
    local_pos = p_local[:3]

    if global_rot is not None:
        # Ориентация
        R_world_obj = rodrigues(np.array(global_rot[:3]) * global_rot[3])
        T_world_obj = r2t(R_world_obj)
        T_robot_obj = T_robot_world @ T_world_obj

        # Извлекаем ориентацию как RPY
        R_local = t2r(T_robot_obj)
        local_rpy = tr2rpy(R_local, order="xyz", unit="rad")
        return local_pos, local_rpy

    return local_pos