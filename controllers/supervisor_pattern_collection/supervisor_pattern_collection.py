import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))
sys.path.append(str(Path(__file__).resolve().parents[2]))

import os
import json

from controller import Supervisor
from extensions.core.motion import MotionTracker
from extensions.kinematics.solvers import solve_ik
from extensions.core.commands import CommandBuilder
from extensions.utils.console import print_progress_bar
from extensions.webots.communication import WebotsJsonComm
from extensions.kinematics.robot_models import LBRiiwaR800Model
from extensions.kinematics.planner import TrajectoryPlannerComponent


# ================ Меням путь до файла и разделить ==========================
# JSON_CARTESIAN_PATH = "$/home/user/webots_project/controllers/supervisor_cartesian_move/example_move.json"
JSON_CARTESIAN_PATH = os.path.expandvars("${HOME}/dev/webots_projects/webots_robots/controllers/supervisor_cartesian_move/example_move.json")
# ===========================================================================


# =============== Автоматически все выполниться =============================
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

comm = WebotsJsonComm(robot.getDevice("supervisor_receiver"),
                      robot.getDevice("supervisor_emitter"))
comm.enable(timestep)

model = LBRiiwaR800Model()
state_q = model.qz

motion = MotionTracker(0.01)
cmd_builder = CommandBuilder([f"lbr_A{i+1}" for i in range(7)], ["camera_motor"])
cmd_builder.set_target(model.qr)


while robot.step(timestep) != -1:
    
    msgs = comm.receive()
    
    for m in msgs:
        if m.get("type") == "LBRiiwa7R800_current_pose":
            state_q = list(m["data"]["joints"].values())[:-1]

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

comm.disable()
