from __future__ import annotations
from abc import ABC, abstractmethod
from numpy.typing import ArrayLike


class BaseTarget(ABC):
    """Описание целевой позы и/или схватов."""

    @property
    @abstractmethod
    def pose(self) -> tuple[ArrayLike, ArrayLike]:
        """Возвращает (xyz, rpy)."""

    @property
    def gripper(self) -> bool:
        """True = открыть, False = закрыть.  По умолчанию – False."""
        return False