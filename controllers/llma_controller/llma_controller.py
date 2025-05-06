import os
import sys
# Добавить родительскую папку (controllers/)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import json
import numpy as np
from controller import Supervisor, Receiver, Emitter
from core import (
    receive_all_messages,
    send_message,
    deg2rad,
    LBRiiwaR800Model,
    calculate_trajectory
)
from spatialmath.base import rpy2r, tr2angvec

# ────────────────────────────────────────────────────────────────
# RPY → Axis-Angle безопасно (даже если угол ≈ 0)
def rpy_to_axis_angle(rpy):
    R = rpy2r(*rpy, order="xyz")
    axis, angle = tr2angvec(R)
    axis = np.atleast_1d(axis).astype(float)
    if axis.size != 3 or np.isnan(axis).any():
        axis = np.array([0.0, 0.0, 1.0])
        angle = 0.0
    return [float(axis[0]), float(axis[1]), float(axis[2]), float(angle)]

# ────────────────────────────────────────────────────────────────
# Загрузка команд из файла
cmd_file = os.path.join(os.path.dirname(__file__), "commands.json")
with open(cmd_file, encoding="utf-8") as f:
    commands = json.load(f)
if commands and isinstance(commands[0], list):
    commands = commands[0]

# ────────────────────────────────────────────────────────────────
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
robot_model = LBRiiwaR800Model()

emitter: Emitter = robot.getDevice("supervisor_emitter")
receiver: Receiver = robot.getDevice("supervisor_receiver")
receiver.enable(timestep)

position_robot = {
    "joints": {
        "A1": deg2rad(0),
        "A2": deg2rad(0),
        "A3": deg2rad(0),
        "A4": deg2rad(0),
        "A5": deg2rad(0),
        "A6": deg2rad(0),
        "A7": deg2rad(0),
        "camera_motor": deg2rad(0),
    },
    "gripper": True,
}

joint_names = list(position_robot["joints"].keys())[:-1]
current_position = [0] * 7
current_index = 0

traj, gripper_pos = calculate_trajectory(robot=robot_model,
                                         commands=commands,
                                         move_time=3,
                                         gripper_time=1,
                                         dt=timestep/1000)

# robot_node = robot.getFromDef("KUKA")
# target_translation_field.setSFVec3f(target_xyz)
#             target_rotation_field.setSFRotation(
#                 rpy_to_axis_angle(current_target_rpy)
# )

# ────────────────────────────────────────────────────────────────
# Основной цикл
while robot.step(timestep) != -1:
    for msg in receive_all_messages(receiver):
        if msg["type"] == "LBRiiwa7R800_data":
            current_position = list(msg["data"]["joints"].values())[:-1]


    if current_index < traj.shape[0]:
        
        for i, pos in enumerate(traj[current_index]):
            joint_name = joint_names[i]
            position_robot["joints"][joint_name] = pos
            position_robot["gripper"] = gripper_pos[current_index]
            # print(position_robot["gripper"])
        current_index += 1

    send_message(
        emitter=emitter,
        source_name=robot.getName(),
        message_type="robot_position",
        data=position_robot,
    )
