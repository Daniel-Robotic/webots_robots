import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))
sys.path.append(str(Path(__file__).resolve().parents[2]))

from controller import Supervisor
from extensions.kinematics.robot_models import LBRiiwaR800Model
from extensions.webots.communication import WebotsJsonComm
from extensions.kinematics.solvers import solve_ik, solve_pzk
from extensions.webots.target import WebotsTargetGizmo
from extensions.core.motion import MotionTracker
from extensions.core.commands import CommandBuilder


# =============== Автоматически все выполниться =============================
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

comm = WebotsJsonComm(robot.getDevice("supervisor_receiver"),
                      robot.getDevice("supervisor_emitter"))
comm.enable(timestep)

model = LBRiiwaR800Model()
state_q = model.qz

robot_node = robot.getFromDef("KUKA")
trg_node = robot.getFromDef("TARGET_GRIPPER")
target = WebotsTargetGizmo(robot_node, trg_node)


ik_solver = lambda qc, xyz, rpy: solve_ik(model, qc, xyz, rpy)
motion = MotionTracker(0.01)
cmd_builder = CommandBuilder([f"lbr_A{i+1}" for i in range(7)], ["camera_motor"])


while robot.step(timestep) != -1:
    
    msgs = comm.receive()
    
    for m in msgs:
        if m.get("type") == "LBRiiwa7R800_current_pose":
            state_q = list(m["data"]["joints"].values())[:-1]
            xyz, rpy = solve_pzk(model, state_q)
            xyz_str = ", ".join(f"{v:.3f}" for v in xyz)
            rpy_str = ", ".join(f"{v:.3f}" for v in rpy)
            print(f"Позиция робота: [{xyz_str}]  [{rpy_str}]")

    if not cmd_builder.has_target:
        xyz, rpy = target.pose
        q_star = ik_solver(state_q, xyz, rpy)

        if q_star is not None:
            cmd_builder.set_target(q_star)

    if cmd_builder.has_target and motion.target_reached(state_q, cmd_builder.target):
        cmd_builder.clear_target()

    comm.send({"source": robot.getName(), 
                   "type": "robot_position", 
                   "data": cmd_builder.command})
    
    # Charuco
    comm.send({
        "source": robot.getName(),
        "type": "pattern_detection",
        "data": {
            "pattern_type": "charuco",
            "params": {
                "grid_cells": [5, 7],
                "cell_size_mm": 55,
                "marker_length_mm": 40,
                "aruco_dict_name": "4x4_50"
            }
        }
    })

    # Aruco
    # comm.send({
    #     "source": robot.getName(),
    #     "type": "pattern_detection",
    #     "data": {
    #         "pattern_type": "aruco",
    #         "params": {
    #             "aruco_dict_name": "4x4_50"
    #         }
    #     }
    # })

    # Chessboard если сгенерирован размер 5х7, то делаем -1 по каждой оси -> 4х6
    # comm.send({
    #     "source": robot.getName(),
    #     "type": "pattern_detection",
    #     "data": {
    #         "pattern_type": "chessboard",
    #         "params": {
    #             "grid_cells": [4, 6]
    #         }
    #     }
    # })

    # CircleGrid sym
    # comm.send({
    #     "source": robot.getName(),
    #     "type": "pattern_detection",
    #     "data": {
    #         "pattern_type": "circlegrid",
    #         "params": {
    #             "grid_cells": [5, 7],
    #             "asymmetric": False
    #         }
    #     }
    # })


    # CircleGrid asym указываем размер, сколько в первом и втором ряду кругляшков -> 5x4
    # comm.send({
    #     "source": robot.getName(),
    #     "type": "pattern_detection",
    #     "data": {
    #         "pattern_type": "circlegrid",
    #         "params": {
    #             "grid_cells": [5, 4],
    #             "asymmetric": True
    #         }
    #     }
    # })


comm.disable()
