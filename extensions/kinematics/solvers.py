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
    return T.t, T.rpy()


def calculate_trajectory(robot: rtb.DHRobot,
                        commands: dict,
                        move_time: int,
                        gripper_time: int,
                        dt: float):
    
    last_q = [0] * 7
    last_gripper_pose = 1
    s = np.array([0]*7)
    gripper_pos = []
    for com in commands:

        if com["command"] == "move":
            position = SE3(*com["args"][:3])
            # TODO: как починиться вернуть на нужное место код
            orientation = SE3.RPY(-3.14, 0, 3.14)
            # orientation = sm.SE3.RPY(*com["args"][3:])
            
            q_pos = robot.ikine_LM(Tep=position*orientation,
                                    q0=last_q,
                                    joint_limits=True)
            traj = rtb.jtraj(q0=last_q, 
                            qf=q_pos.q, 
                            t=int(move_time / dt))
            last_q = q_pos.q

            s = np.vstack([s, traj.q])
            gripper_pos.extend([last_gripper_pose] * len(traj.q))

        if com["command"] == "grab":
            traj = np.tile(last_q, (int(gripper_time / dt), 1)) 
            s = np.vstack([s, traj])
            last_gripper_pose = int(not com["args"])
            gripper_pos.extend([last_gripper_pose] * traj.shape[0])
    
    return s[1:], gripper_pos