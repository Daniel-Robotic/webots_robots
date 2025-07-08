import numpy as np
from numpy.typing import ArrayLike


class MotionTracker:
    """Проверка достижения цели с погрешностью."""

    def __init__(self, tolerance: float = 1e-2):
        self._tol = float(tolerance)

    def target_reached(self, cur: ArrayLike, goal: ArrayLike) -> bool:
        return float(np.linalg.norm(np.asarray(cur) - np.asarray(goal))) < self._tol