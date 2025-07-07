import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))
sys.path.append(str(Path(__file__).resolve().parents[2]))


import numpy as np
from spatialmath.base import rodrigues, tr2rpy, r2t
from controller import Supervisor, Receiver, Emitter

from extensions.utils import deg2rad
from extensions.kinematics.robot_models import LBRiiwaR800Model
from extensions.kinematics.kinematics import solve_ik, solve_pzk
from extensions.communication import send_message, receive_all_messages
from extensions.kinematics.transformations import transform_world_to_local


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
    "gripper": True
}

joint_names = list(position_robot["joints"].keys())[:-1]
current_position = [0] * 7
position_tolerance = 0.01
ik_computed = False
target_q = None


robot_node = robot.getFromDef("KUKA")
target_node = robot.getFromDef("TARGET_GRIPPER")
gripper_open_field = target_node.getField("gripper_open")
target_translation_field = target_node.getField("translation")
target_rotation_field = target_node.getField("rotation")


# obj1 = robot.getFromDef("OBJ1")
# obj2 = robot.getFromDef("OBJ2")
# obj3 = robot.getFromDef("OBJ3")
# pos_obj1 = obj1.getField("translation").getSFVec3f()
# pos_obj2 = obj2.getField("translation").getSFVec3f()
# pos_obj3 = obj3.getField("translation").getSFVec3f()

# obj1_position_world = transform_world_to_local(robot_node, pos_obj1)
# obj2_position_world = transform_world_to_local(robot_node, pos_obj2)
# obj3_position_world = transform_world_to_local(robot_node, pos_obj3)

# print(obj1_position_world)
# print(obj2_position_world)
# print(obj3_position_world)


while robot.step(timestep) != -1:

    # TODO: Получаем все данные с каждой камеры, потом можно отправлять данные на робота
    messages = receive_all_messages(receiver=receiver)
    # print(messages)
    
    for msg in messages:
        if msg['type'] == "LBRiiwa7R800_data":
            current_position = list(msg["data"]["joints"].values())[:-1]
            
            # Решение прямой задачи кинематки
            # ee_pos, ee_orient_rpy = solve_pzk(robot=robot_model,
            #                                   q=current_position)

    # TODO: Логика работы
    
    # Решение ОЗК
    # TODO Можно поменять на другой алгоритм
    if not ik_computed:
        # Цель в локальных координатах → мировые координаты
        target_position_world = transform_world_to_local(
            robot_node, 
            target_translation_field.getSFVec3f()
        )

        # Конвертация ориентации из axis-angle в RPY
        rotation = target_rotation_field.getSFRotation()  # [axis_x, axis_y, axis_z, angle]
        rot_matrix = rodrigues(np.array(rotation[:3]) * rotation[3])
        T_rot = r2t(rot_matrix)
        target_orientation_rpy = tr2rpy(T_rot, order='xyz')
        
        q = solve_ik(
            robot=robot_model,
            q_current=current_position,
            target_position=target_position_world,
            target_orientation_rpy=target_orientation_rpy
            # target_position=[0.5, 0.3, 0.3],
            # target_orientation_rpy=[-3.14, 0, -3.14]
        )

        print(target_position_world, target_orientation_rpy)
        
        if q is not None:
            target_q = q
            ik_computed = True
        else:
            continue

    # Если ОЗК уже решено — двигаемся к цели
    if ik_computed and target_q is not None:
        # Проверяем, насколько близко текущее положение к целевому
        error = np.linalg.norm(np.array(current_position) - np.array(target_q))

        if error < position_tolerance:
            ik_computed = False
        else:
            for i, name in enumerate(joint_names):
                position_robot["joints"][name] = target_q[i]
    
    # Открытие и закрытие захвата
    gripper_open = gripper_open_field.getSFBool()
    position_robot["gripper"] = gripper_open
    
    # Отправка данных на робота
    send_message(emitter=emitter,
                 source_name=robot.getName(),
                 message_type="robot_position",
                 data=position_robot)

receiver.disable()