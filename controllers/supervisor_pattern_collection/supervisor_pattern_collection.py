import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))
sys.path.append(str(Path(__file__).resolve().parents[2]))

import os
import json
import pandas as pd

from controller import Supervisor
from extensions.core.motion import MotionTracker
from extensions.kinematics.solvers import solve_ik
from extensions.core.commands import CommandBuilder
from extensions.utils.console import print_progress_bar
from extensions.webots.communication import WebotsJsonComm
from extensions.kinematics.robot_models import LBRiiwaR800Model
from extensions.kinematics.planner import TrajectoryPlannerComponent


# ================ Меням путь до файла и разделить ==========================
# JSON_CARTESIAN_PATH = "/home/user/webots_project/controllers/supervisor_pattern_collection/collect_pattern.json"
# IMAGE_FOLDER = "/home/user/webots_project/pattern_collect"
 
JSON_CARTESIAN_PATH = os.path.expandvars("${HOME}/dev/webots_projects/webots_robots/controllers/supervisor_pattern_collection/collect_pattern.json")
IMAGE_FOLDER = os.path.expandvars("${HOME}/dev/webots_projects/webots_robots/pattern_collect")
# ===========================================================================


# =============== Автоматически все выполниться =============================
with open(JSON_CARTESIAN_PATH, encoding="utf-8") as f:
    commands = json.load(f)
if commands and isinstance(commands[0], list):
    commands = commands[0]

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
# dt = 10 / 1000.0
dt = timestep / 1000.0


model = LBRiiwaR800Model()
state_q = model.qz

comm = WebotsJsonComm(robot.getDevice("supervisor_receiver"),
                      robot.getDevice("supervisor_emitter"))
ik_solver = lambda qc, xyz, rpy: solve_ik(model, qc, xyz, rpy)
motion = MotionTracker(0.005)
cmd_builder = CommandBuilder([f"lbr_A{i+1}" for i in range(7)], ["camera_motor"])
planer = TrajectoryPlannerComponent(model)

comm.enable(timestep)

trajectory = []
current_q = model.qz

path_robot = []

success = planer.plan(current_q, target_rpy=None, target_xyz=None, dt=dt, q_target=model.qr, speed_scale=0.1)
if success:
    for point in planer.trajectory:
        trajectory.append({"joints": point, "collect": None})
        path_robot.append(point)
    current_q = planer.trajectory[-1]
else:
    print(f"[ERROR] Не удалось построить стартовую траекторию от qz к qr")

for cmd in commands:
    if cmd["command"] == "move":
        xyz = cmd["args"][:3]
        rpy = cmd["args"][3:]
        success = planer.plan(current_q, xyz, rpy, dt, speed_scale=0.5)

        if success:
            for point in planer.trajectory:
                trajectory.append({"joints": point, "collect": None})
                path_robot.append(point)
            current_q = planer.trajectory[-1]
        else:
            print(f"[WARN] Не удалось построить траекторию для: {xyz}, {rpy}")

    elif cmd["command"] == "collect":
        if len(trajectory) == 0:
            print("[ERROR] Нет предыдущей точки для команды collect.")
            continue

        last_q = trajectory[-1]["joints"]
        n_repeat = int(0.5 / dt)

        for _ in range(n_repeat):
            trajectory.append({"joints": last_q, "collect": None})
            path_robot.append(last_q)

        trajectory.append({"joints": last_q, "collect": cmd["args"]})
        path_robot.append(last_q)

print(f"[INFO] Сформирована траектория из {len(trajectory)} шагов")

df = pd.DataFrame(data=path_robot, columns=[f"A{i}" for i in range(1, 8)])
df.to_csv("./trajectory_collect.csv", index=False)

cmd_builder.set_target(model.qz)
cmd_builder.gripper_open = True
index = 0
first = True


while robot.step(timestep) != -1:
    
    msgs = comm.receive()
    
    for m in msgs:
        if m.get("type") == "LBRiiwa7R800_current_pose":
            state_q = list(m["data"]["joints"].values())[:-1]

    if first:
        comm.send({"source": robot.getName(), 
                   "type": "robot_position", 
                   "data": cmd_builder.command})
        first = False
        continue

    if cmd_builder.has_target and motion.target_reached(state_q, cmd_builder.target):
        cmd_builder.clear_target()

        if index < len(trajectory):
            point = trajectory[index]
            cmd_builder.set_target(point["joints"])

            if point["collect"] is not None:
                comm.send({"source": robot.getName(), 
                            "type": "save_image", 
                            "data": {
                                      "folder": IMAGE_FOLDER,
                                      "type": "rgb"
                                    } 
                            })

            print_progress_bar(index + 1, len(trajectory))
            index += 1

    comm.send({"source": robot.getName(), 
                   "type": "robot_position", 
                   "data": cmd_builder.command})

comm.disable()
