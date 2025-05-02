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
    solve_ik,
    solve_pzk,
    transform_world_to_local,
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

cmd_index = 0
cmd_in_progress = False
current_target_rpy = [0.0, 0.0, 0.0]
gripper_state = True  # захват по умолчанию закрыт

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
    "gripper": gripper_state,
}

joint_names = list(position_robot["joints"].keys())[:-1]
current_position = [0] * 7
position_tolerance = 0.01
ik_computed = False
target_q = None

robot_node = robot.getFromDef("KUKA")
target_node = robot.getFromDef("TARGET")
gripper_open_field = target_node.getField("gripper_open")
target_translation_field = target_node.getField("translation")
target_rotation_field = target_node.getField("rotation")

# ────────────────────────────────────────────────────────────────
# Основной цикл
while robot.step(timestep) != -1:
    for msg in receive_all_messages(receiver):
        if msg["type"] == "LBRiiwa7R800_data":
            current_position = list(msg["data"]["joints"].values())[:-1]

    if cmd_index >= len(commands):
        continue

    current_cmd = commands[cmd_index]

    # ───────────── Команда MOVE ─────────────
    if current_cmd["command"] == "move":
        target_xyz = current_cmd["args"][:3]
        current_target_rpy = current_cmd["args"][3:]

        if not cmd_in_progress:
            target_translation_field.setSFVec3f(target_xyz)
            target_rotation_field.setSFRotation(
                rpy_to_axis_angle(current_target_rpy)
            )
            ik_computed = False
            cmd_in_progress = True

        if ik_computed and target_q is not None:
            err = np.linalg.norm(np.array(current_position) - np.array(target_q))
            if err < position_tolerance:
                print(f"[LLMA] ✔ Команда #{cmd_index} выполнена")
                cmd_index += 1
                cmd_in_progress = False

    # ───────────── Команда GRAB ─────────────
    elif current_cmd["command"] == "grab":
        want_open = bool(current_cmd["args"])      # true = открыть
        gripper_state = not want_open              # True = захват закрыт
        gripper_open_field.setSFBool(want_open)    # Webots поле (для визуализации)
        print(f"[LLMA] ✔ Команда #{cmd_index} выполнена")
        cmd_index += 1

    # ───────────── Решение ОЗК ─────────────
    if not ik_computed:
        target_position_world = target_translation_field.getSFVec3f()
        print(f"[LLMA] → Решение ОЗК для #{cmd_index}: {target_position_world}  {current_target_rpy}")
        q = solve_ik(
            robot=robot_model,
            q_current=current_position,
            target_position=target_position_world,
            target_orientation_rpy=current_target_rpy,
        )
        if q is not None:
            target_q = q
            ik_computed = True

    if ik_computed and target_q is not None:
        error = np.linalg.norm(np.array(current_position) - np.array(target_q))
        if error < position_tolerance:
            ik_computed = False
        else:
            for i, name in enumerate(joint_names):
                position_robot["joints"][name] = target_q[i]

    # ───────────── Отправка на робота ─────────────
    position_robot["gripper"] = gripper_state

    send_message(
        emitter=emitter,
        source_name=robot.getName(),
        message_type="robot_position",
        data=position_robot,
    )
