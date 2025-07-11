import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))
sys.path.append(str(Path(__file__).resolve().parents[2]))

import os
import pandas as pd

from controller import Supervisor
from extensions.core.motion import MotionTracker
from extensions.core.commands import CommandBuilder
from extensions.utils.console import print_progress_bar
from extensions.webots.communication import WebotsJsonComm
from extensions.kinematics.robot_models import LBRiiwaR800Model

# ================ Меням путь до файла и разделить ==========================
# CSV_PATH = "$/home/user/dev/webots_projects/webots_robots/controllers/supervisor_csv_moving/example_move.csv"
CSV_PATH = os.path.expandvars("${HOME}/dev/webots_projects/webots_robots/controllers/supervisor_csv_moving/example_move.csv")
SEPARATION = ","
# ===========================================================================


# =============== Автоматически все выполниться =============================
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
df = pd.read_csv(CSV_PATH, sep=SEPARATION)

model = LBRiiwaR800Model()
state_q = model.qz

comm = WebotsJsonComm(robot.getDevice("supervisor_receiver"),
                      robot.getDevice("supervisor_emitter"))
motion = MotionTracker(0.01)
cmd_builder = CommandBuilder([f"lbr_A{i+1}" for i in range(7)], ["camera_motor"])

comm.enable(timestep)

# Переводим робота в начальную точку
current_index = 0
cmd_builder.set_target(df.iloc[current_index].to_numpy())

while robot.step(timestep) != -1:
    msgs = comm.receive()

    for m in msgs:
        if m.get("type") == "LBRiiwa7R800_current_pose":
            state_q = list(m["data"]["joints"].values())[:-1]
    

    if cmd_builder.has_target and motion.target_reached(state_q, cmd_builder.target):
        cmd_builder.clear_target()

        if current_index + 1 < len(df):
            current_index += 1
            cmd_builder.set_target(df.iloc[current_index].to_numpy())

        print_progress_bar(current_index + 1, len(df))

    comm.send({"source": robot.getName(), 
               "type": "robot_position", 
               "data": cmd_builder.command})


comm.disable()