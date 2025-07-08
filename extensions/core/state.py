from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Any, Dict, List


class BaseRobotState(ABC):
    """Описание текущего состояния манипулятора (q, TCP и пр.)."""

    @abstractmethod
    def update(self, messages: List[Dict[str, Any]]) -> None: ...

    @property
    @abstractmethod
    def joint_positions(self): ...