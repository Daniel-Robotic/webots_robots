from __future__ import annotations
from abc import ABC, abstractmethod
from numpy.typing import ArrayLike


class BaseIKSolver(ABC):
    @abstractmethod
    def solve(self,
              q_current: ArrayLike,
              xyz_target: ArrayLike,
              rpy_target: ArrayLike): ...