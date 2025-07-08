import numpy as np
import roboticstoolbox as rtb

from typing import List, Callable
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


class TargetNodeComponent:
    def __init__(self, 
                 robot_node, 
                 target_node,):
        
        self._robot_node = robot_node
        self._translation_field = target_node.getField("translation")
        self._rotation_field = target_node.getField("rotation")

    def get_target_pose(self):
        pos = transform_world_to_local(self._robot_node, self._translation_field.getSFVec3f())
        rot = self._rotation_field.getSFRotation()
        rot_matrix = rodrigues(np.array(rot[:3]) * rot[3])
        T_rot = r2t(rot_matrix)
        rpy = tr2rpy(T_rot, order='xyz')
        return pos, rpy

    
class TargetGizmaComponent(TargetNodeComponent):
    def __init__(self, robot_node, target_node):
        super().__init__(robot_node, target_node)
        self._gripper_field = target_node.getField("gripper_open")
    
    def get_gripper_state(self):
        return self._gripper_field.getSFBool()


class IKComponent:
    def __init__(self, 
                 robot_model: rtb.DHRobot,
                 solver_ik: Callable):
        self._robot_model = robot_model
        self._target_q = None
        self._ik_computed = False
        self.__solver_ik = solver_ik

    @property
    def ik_computed(self):
        return self._ik_computed
    
    @property
    def target_q(self):
        return self._target_q

    def compute(self, current_q, target_pos, target_rpy):
        if not self.ik_computed:
            q = self.__solver_ik(self._robot_model, current_q, target_pos, target_rpy)
            if q is not None:
                self._target_q = q
                self._ik_computed = True

    def reset(self):
        self._ik_computed = False


class MotionTrackerComponent:
    def __init__(self, tolerance=0.01):
        self._tolerance = tolerance

    def is_target_reached(self, current_q, target_q) -> bool:
        error = np.linalg.norm(np.array(current_q) - np.array(target_q))
        return error < self._tolerance


class CommandBuilderManipulatorComponent:
    def __init__(self, 
                 joint_names,
                 add_joints: List=[]):
        self._joint_names = joint_names
        self._command = {
            "joints": {name: 0.0 for name in joint_names + add_joints},
            "gripper": True
        }

    def build_joint_command(self, target_q):
        for i, name in enumerate(self._joint_names):
            self._command["joints"][name] = target_q[i]

    def set_gripper_state(self, is_open):
        self._command["gripper"] = is_open

    def get_command(self):
        return self._command
