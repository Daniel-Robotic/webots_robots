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
from extensions.webots.communication import WebotsJsonComm
from extensions.kinematics.robot_models import LBRiiwaR800Model
from extensions.kinematics.planner import TrajectoryPlannerComponent

# ================ Меням путь до файла и разделить ==========================
# JSON_CARTESIAN_PATH = "$/home/user/webots_project/controllers/supervisor_cartesian_move/example_move.json"
JSON_CARTESIAN_PATH = os.path.expandvars("${HOME}/dev/webots_projects/webots_robots/controllers/supervisor_cartesian_move/example_move.json")
# ===========================================================================


# =============== Автоматически все выполниться =============================
def print_progress_bar(current: int, total: int, bar_width: int = 20):
    percent = int((current / total) * 100)
    bar_filled = int(bar_width * current / total)
    bar = "█" * bar_filled + "-" * (bar_width - bar_filled)
    print(f"[{percent:>3}%/100%] {bar} [{current}/{total}]")


with open(JSON_CARTESIAN_PATH, encoding="utf-8") as f:
    commands = json.load(f)
if commands and isinstance(commands[0], list):
    commands = commands[0]

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

model = LBRiiwaR800Model()
state_q = model.qz

comm = WebotsJsonComm(robot.getDevice("supervisor_receiver"),
                      robot.getDevice("supervisor_emitter"))
ik_solver = lambda qc, xyz, rpy: solve_ik(model, qc, xyz, rpy)
motion = MotionTracker(0.05)
cmd_builder = CommandBuilder([f"lbr_A{i+1}" for i in range(7)], ["camera_motor"])
planer = TrajectoryPlannerComponent(model)

comm.enable(timestep)

# ==== Построим полную траекторию ====
trajectory = []
current_q = state_q.copy()

i = 0
while i < len(commands):
    cmd = commands[i]

    if cmd["command"] == "move":
        xyz = cmd["args"][:3]
        rpy = cmd["args"][3:]
        success = planer.plan(current_q, xyz, rpy, dt, speed_scale=0.5)

        if success:
            for point in planer.trajectory:
                trajectory.append({"joints": point, "gripper": None})
            current_q = planer.trajectory[-1]
        else:
            print(f"[WARN] Не удалось построить траекторию для: {xyz}, {rpy}")

    elif cmd["command"] == "grab":
        if len(trajectory) == 0:
            print("[ERROR] Нет предыдущей точки для команды grab.")
            i += 1
            continue

        last_q = trajectory[-1]["joints"]
        n_repeat = int(0.5 / dt)

        for _ in range(n_repeat):
            trajectory.append({"joints": last_q, "gripper": None})

        trajectory.append({"joints": last_q, "gripper": cmd["args"]})

    i += 1

print(f"[INFO] Сформирована траектория из {len(trajectory)} шагов")

# ==== Основной цикл движения ====
cmd_builder.set_target([0] * model.n)
cmd_builder.gripper_open = True
index = 0

while robot.step(timestep) != -1:
    msgs = comm.receive()

    for m in msgs:
        if m.get("type") == "LBRiiwa7R800_current_pose":
            state_q = list(m["data"]["joints"].values())[:-1]

    if cmd_builder.has_target and motion.target_reached(state_q, cmd_builder.target):
        cmd_builder.clear_target()

        if index < len(trajectory):
            point = trajectory[index]
            cmd_builder.set_target(point["joints"])

            if point["gripper"] is not None:
                cmd_builder.gripper_open = not point["gripper"]

            print_progress_bar(index + 1, len(trajectory))
            index += 1

    comm.send({
        "source": robot.getName(),
        "type": "robot_position",
        "data": cmd_builder.command
    })

comm.disable()