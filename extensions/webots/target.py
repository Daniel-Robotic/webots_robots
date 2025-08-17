from __future__ import annotations
import numpy as np
from controller import Node
from spatialmath.base import rodrigues, r2t, tr2rpy
from numpy.typing import ArrayLike
from ..core.target import BaseTarget
from ..utils.math import transform_world_to_local


class WebotsTargetGizmo(BaseTarget):
    """Чтение poz/rot gizmo‑ноды + булево поле gripper_open."""

    def __init__(self, robot_node: Node, gizmo_node: Node):
        self._robot_node = robot_node
        self._tr_field = gizmo_node.getField("translation")
        self._rot_field = gizmo_node.getField("rotation")
        self._gr_field = gizmo_node.getField("gripper_open")

    # ------------ BaseTarget ---------
    @property
    def pose(self) -> tuple[ArrayLike, ArrayLike]:
        xyz = transform_world_to_local(self._robot_node,
                                       self._tr_field.getSFVec3f())
        rot = self._rot_field.getSFRotation()  # axis‑angle
        R = rodrigues(np.asarray(rot[:3]) * rot[3])
        rpy = tr2rpy(r2t(R), order="xyz")
        return xyz, rpy

    @property
    def gripper(self) -> bool:
        return bool(self._gr_field.getSFBool())


class WebotsTargetObject(BaseTarget):
    def __init__(self, robot_node: Node, object_node: Node):
        self._robot_node = robot_node
        self._tr_field = object_node.getField("translation")
        self._rot_field = object_node.getField("rotation")
    
    @property
    def pose(self) -> tuple[ArrayLike, ArrayLike]:
        xyz = transform_world_to_local(self._robot_node,
                                       self._tr_field.getSFVec3f())
        rot = self._rot_field.getSFRotation()
        R = rodrigues(np.asarray(rot[:3]) * rot[3])
        rpy = tr2rpy(r2t(R), order="xyz")
        return xyz, rpy
    