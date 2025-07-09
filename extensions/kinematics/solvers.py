import numpy as np
import roboticstoolbox as rtb

from typing import Sequence
from spatialmath import SE3
from numpy.typing import ArrayLike


def solve_ik(robot: rtb.DHRobot,
             q_current: Sequence,
             target_position: ArrayLike,
             target_orientation_rpy: ArrayLike
            ):
    
    T_target = SE3(target_position) * SE3.RPY(target_orientation_rpy, order='xyz')
    sol = robot.ikine_LM(T_target, q0=q_current)
    
    if sol.success:
        return sol.q
    
    print("[WARN] IK not solving")
    return None


def solve_pzk(model: rtb.DHRobot,
              q: ArrayLike):
    
    T = model.fkine(q)
    return T.t, T.rpy(order="xyz")
