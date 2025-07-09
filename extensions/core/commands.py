from __future__ import annotations
from typing import List, Dict
from numpy.typing import ArrayLike


class CommandBuilder:
    """Формирует словарь для отправки роботу."""

    def __init__(self, joint_names: List[str], extra_joints: List[str] | None = None):
        self._joint_names = joint_names
        self._cmd: Dict = {
            "joints": {name: 0.0 for name in (joint_names + (extra_joints or []))},
            "gripper": True,
        }
        self._target: ArrayLike | None = None

    @property
    def command(self) -> Dict:
        return self._cmd

    @property
    def has_target(self) -> bool:
        return self._target is not None

    @property
    def target(self):
        return self._target

    def set_target(self, q: ArrayLike):
        self._target = q
        for i, name in enumerate(self._joint_names):
            self._cmd["joints"][name] = float(q[i])

    def clear_target(self):
        self._target = None

    @property
    def gripper_open(self):
        return self._cmd["gripper"]

    @gripper_open.setter
    def gripper_open(self, val: bool):
        self._cmd["gripper"] = not bool(val)