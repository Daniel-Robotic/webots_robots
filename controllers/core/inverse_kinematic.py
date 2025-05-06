import numpy as np
import spatialmath as sm
import roboticstoolbox as rtb

from typing import List
from spatialmath import SE3
from spatialmath.base import transl, rodrigues, r2t, trotz

class LBRiiwaR800Model(rtb.DHRobot):
     def __init__(self):

        qlim_one = [np.deg2rad(-170), np.deg2rad(170)]
        qlim_two = [np.deg2rad(-120), np.deg2rad(120)]

        mm = 1e-3
        tool_offset = 230 * mm
        flange = 107 * mm

        links = [
                    rtb.RevoluteDH(alpha=-np.pi/2, 
                                   d=0.34, 
                                   a=0, 
                                   qlim=qlim_one,
                                   m=3.4525,
                                   I=[0.02183, 0, 0, 0.007703, -0.003887, 0.02083],
                                   G=1),
                    rtb.RevoluteDH(alpha=np.pi/2, 
                                   d=0, 
                                   a=0, 
                                   qlim=qlim_two,
                                   m=3.4821,
                                   I=[0.02076, 0, -0.003626, 0.02179, 0, 0.00779],
                                   G=1),
                    rtb.RevoluteDH(alpha=np.pi/2, 
                                   d=0.4, 
                                   a=0, 
                                   qlim=qlim_one,
                                   m=4.05623,
                                   I=[0.03204, 0, 0, 0.00972, 0.006227, 0.03042],
                                   G=1),
                    rtb.RevoluteDH(alpha=-np.pi/2, 
                                   d=0, 
                                   a=0, 
                                   qlim=qlim_two,
                                   m=3.4822,
                                   I=[0.02178, 0, 0, 0.02075, -0.003625, 0.007785],
                                   G=1),
                    rtb.RevoluteDH(alpha=-np.pi/2, 
                                   d=0.4, 
                                   a=0, 
                                   qlim=qlim_one,
                                   m=2.1633,
                                   I=[0.01287, 0, 0, 0.005708, -0.003946, 0.01112],
                                   G=1),
                    rtb.RevoluteDH(alpha=np.pi/2, 
                                   d=0, 
                                   a=0, 
                                   qlim=qlim_two,
                                   m=2.3466,
                                   I=[0.006509, 0, 0, 0.006259, 0.00031891, 0.004527],
                                   G=1),
                    rtb.RevoluteDH(alpha=0,
                                   d=0.126, 
                                   a=0, 
                                   qlim=[np.deg2rad(-175), np.deg2rad(175)],
                                   m=3.129,
                                   I=[0.01464, 0.0005912, 0, 0.01465, 0, 0.002872],
                                   G=1)
                ]

        super().__init__(
            links=links,
            name="LBRiiwaR800",
            manufacturer="KUKA",
        )

        # tool = transl(0, 0, tool_offset) @ trotz(-np.pi / 4)
        tool = transl(0, 0, tool_offset) @ trotz(0)
        self.tool = tool
        self.qr = np.array([0, -0.3, 0, -1.9, 0, 1.5, 0])
        self.qz = np.zeros(7)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)
        

def solve_ik(robot: rtb.DHRobot,
             q_current: List,
             target_position: List,
             target_orientation_rpy: List
            ):
    
    T_target = SE3(target_position) * SE3.RPY(target_orientation_rpy, order='xyz')
    sol = robot.ikine_LM(T_target, q0=q_current)
    
    if sol.success:
        return sol.q
    
    print("[WARN] IK not solving")
    return None


def solve_pzk(robot: rtb.DHRobot,
              q: List):
    T = robot.fkine(q)
    
    positions = T.t
    orientation_rpy = T.rpy()
    
    return positions, orientation_rpy


def transform_world_to_local(robot_node, global_pos):
    """
    Переводит мировую позицию (например, цели) в систему координат робота.
    
    :param robot_node: Webots Node — DEF KUKA
    :param global_pos: [x, y, z] — позиция цели в мировой системе
    :return: позиция в локальных координатах робота
    """
    translation = robot_node.getField("translation").getSFVec3f()
    rotation = robot_node.getField("rotation").getSFRotation()  # [axis_x, axis_y, axis_z, angle]

    # Преобразуем rotation в матрицу поворота
    R = rodrigues(np.array(rotation[:3]) * rotation[3])
    T = r2t(R)
    T[:3, 3] = translation

    # Инвертируем трансформацию
    T_inv = np.linalg.inv(T)

    # Переводим мировую позицию в локальную
    p_world = np.append(global_pos, 1.0)
    p_local = T_inv @ p_world

    return p_local[:3]

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