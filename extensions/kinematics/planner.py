from typing import Tuple
import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb


class TrajectoryPlannerComponent:
    def __init__(self, robot: rtb.DHRobot, joint_speed_deg: float = 98.0):
        self._robot = robot
        self._trajectory: np.ndarray = np.zeros((0, robot.n))
        self._last_q: np.ndarray = np.zeros(robot.n)
        self._joint_speed_rad = np.deg2rad(joint_speed_deg)  # ~1.710 rad/s

    def plan(self,
             current_q: np.ndarray,
             target_xyz: Tuple[float, float, float],
             target_rpy: Tuple[float, float, float],
             dt: float = 0.01,
             speed_scale: float = 1.0) -> bool:
        """
        Планирует траекторию с учётом ограничения скорости сустава.
        speed_scale — от 0.01 до 1.0 (процент от максимальной скорости)
        """
        self._last_q = current_q.copy()
        T = SE3(*target_xyz) * SE3.RPY(target_rpy, order="xyz")

        ik_solution = self._robot.ikine_LM(Tep=T, q0=current_q, joint_limits=True)
        if not ik_solution.success:
            print(f"[IK WARNING] IK не решена для позы: {target_xyz}, {target_rpy}")
            self._trajectory = np.zeros((0, self._robot.n))
            return False

        q_target = ik_solution.q

        # Расчёт максимального углового смещения
        max_delta = np.max(np.abs(q_target - current_q))  # [рад]

        # Расчёт времени движения по самому медленному суставу
        max_speed = self._joint_speed_rad * max(0.01, min(speed_scale, 1.0))  # рад/с
        move_time = max_delta / max_speed

        n_steps = max(2, int(move_time / dt))

        traj = rtb.jtraj(q0=current_q, qf=q_target, t=n_steps)
        self._trajectory = traj.q
        self._last_q = q_target
        
        return True

    @property
    def trajectory(self) -> np.ndarray:
        return self._trajectory

    @property
    def current_q(self) -> np.ndarray:
        return self._last_q
