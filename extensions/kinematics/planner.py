import numpy as np

from typing import Tuple
from spatialmath import SE3
from roboticstoolbox import DHRobot, jtraj


class TrajectoryPlannerComponent:
    def __init__(self,
                 robot: DHRobot,
                 dt: float = 0.01):
        """
        :param robot: модель робота (DHRobot)
        :param dt: шаг симуляции в секундах (например, 0.01 для 10 мс)
        """
        self._robot = robot
        self._trajectory: np.ndarray = np.zeros((0, robot.n))
        self._last_q: np.ndarray = np.zeros(robot.n)
        self._dt = dt

    def _euler_to_se3(self,
                      xyz: Tuple[float, float, float],
                      rpy: Tuple[float, float, float]) -> SE3:
        return SE3(*xyz) * SE3.RPY(rpy, order="xyz")

    def plan(self,
             current_q: np.ndarray,
             target_xyz: Tuple[float, float, float],
             target_rpy: Tuple[float, float, float],
             speed_scale: float = 1.0,
             q_target: np.ndarray = None) -> bool:
        """
        Планирует PTP траекторию от текущей позы к целевой позе в XYZ+RPY.

        :param current_q: текущие суставные углы
        :param target_xyz: целевая позиция [x, y, z]
        :param target_rpy: ориентация [roll, pitch, yaw] в радианах
        :param speed_scale: масштаб скорости (0.01–1.0)
        :param q_target: если указан — используется напрямую, без IK
        """
        self._last_q = current_q.copy()
        velocity = max(0.01, min(speed_scale, 1.0))

        # Получение целевых суставов
        if q_target is None:
            if target_xyz is None or target_rpy is None:
                print("[PLAN ERROR] Не заданы ни q_target, ни target_xyz+target_rpy")
                return False

            T_goal = self._euler_to_se3(target_xyz, target_rpy)
            sol = self._robot.ikine_LM(T_goal, q0=current_q, joint_limits=True)
            if not sol.success:
                print(f"[IK WARNING] IK не решена для: {target_xyz}, {target_rpy}")
                return False
            q_target = sol.q
        else:
            q_target = np.array(q_target)

        # Используем robot.qd как предел скорости
        qd = getattr(self._robot, "qd", np.ones_like(current_q))
        delta = np.abs(q_target - current_q)
        move_time = np.max(delta / (qd * velocity))
        steps = max(2, int(move_time / self._dt))

        traj = jtraj(current_q, q_target, steps)
        self._trajectory = traj.q
        self._last_q = q_target

        return True

    @property
    def trajectory(self) -> np.ndarray:
        return self._trajectory

    @property
    def current_q(self) -> np.ndarray:
        return self._last_q
