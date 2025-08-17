import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))
sys.path.append(str(Path(__file__).resolve().parents[2]))

import os

import sys
import json
import argparse

from controller import Supervisor
from extensions.core.motion import MotionTracker
from extensions.kinematics.solvers import solve_ik
from extensions.core.commands import CommandBuilder
from extensions.utils.console import print_progress_bar
from extensions.webots.logger import MoveResultLogger, InitialObjectsSnapshotLogger
from extensions.webots.communication import WebotsJsonComm
from extensions.webots.target import WebotsTargetObject
from extensions.kinematics.robot_models import LBRiiwaR800Model
from extensions.kinematics.planner import TrajectoryPlannerComponent
from extensions.utils.math import (build_pick_place_commands, 
                                   build_pick_place_pairwise,
                                   generate_pallet_poses,
                                   split_xyz_rpy,
                                   transform_world_to_local)
from extensions.utils.normolize import (normalize_objects, 
                                        normalize_target_poses, 
                                        normalize_vec,
                                        have_pallet)
from extensions.utils.params import (load_objects_config,
                                     apply_objects_config)
from extensions.webots.spawner import ObstacleSpawner


JSON_PATH = os.path.expandvars("${HOME}/dev/webots_projects/webots_robots/controllers/supervisor_llm/example_move_llm.json")
RESULTS_JSON_PATH = os.path.expandvars(
    "${HOME}/dev/webots_projects/webots_robots/controllers/supervisor_llm/move_results.json"
)
OBJECTS_INFO_JSON_PATH = os.path.expandvars(
    "${HOME}/dev/webots_projects/webots_robots/controllers/supervisor_llm/objects_info.json"
)
PROMT_PATH = os.path.expandvars("")


# =============== Автоматически все выполниться =============================

# ==== Парсинг аргументов ====
parser = argparse.ArgumentParser(add_help=False)

parser.add_argument("--objects", "-o", nargs="+", metavar="DEF",
                    help="Список DEF-имен объектов (например: OBJ1 OBJ2 или OBJ1,OBJ2)")
parser.add_argument("--target-poses", "-tp", nargs="+", metavar="VAL",
                    help="Целевая поза укладки: 6 чисел (X Y Z A B C)")

parser.add_argument("--delta-z", type=float, default=0.05,
                    help="Безопасный подлёт/уход по Z (м)")
parser.add_argument("--place-step-z", type=float, default=0.05,
                    help="Высота шага стека по Z для каждого следующего объекта (м)")
parser.add_argument("--gap", type=float, default=0.002,
                    help="Зазор между слоями при стэкинге (м)")

parser.add_argument("--pallet-origin", nargs="+", metavar="VAL",
                    help="Начальная точка палеты: 3 числа (X Y Z) или одна строка")
parser.add_argument("--pallet-rpy", nargs="+", metavar="VAL",
                    help="Ориентация для всех ячеек палеты: 3 числа (A B C) или одна строка; по умолчанию (0,0,0)")
parser.add_argument("--pallet-nx", type=int, help="Кол-во ячеек по X")
parser.add_argument("--pallet-ny", type=int, help="Кол-во ячеек по Y")
parser.add_argument("--pallet-nz", type=int, default=1, help="Кол-во слоёв по Z (по умолчанию 1)")
parser.add_argument("--pallet-sx", type=float, help="Шаг между ячейками по X (м)")
parser.add_argument("--pallet-sy", type=float, help="Шаг между ячейками по Y (м)")
parser.add_argument("--pallet-sz", type=float, default=0.0, help="Шаг между слоями по Z (м), по умолчанию 0")
parser.add_argument("--pallet-order", choices=["xyz", "xzy", "yxz", "yzx", "zxy", "zyx"],
                    default="zxy", help="Порядок заполнения (по умолчанию zxy: слой→строка→столбец)")
parser.add_argument("--objects-config", type=str,
                    help="Путь к JSON с описанием объектов (DEF, цвет, стартовая позиция, random/ограничения)")


raw_args = sys.argv[1:]
opts, _ = parser.parse_known_args(raw_args)
opts.objects = normalize_objects(opts.objects)
opts.pallet_origin = normalize_vec(opts.pallet_origin, 3) if opts.pallet_origin else None
opts.pallet_rpy = normalize_vec(opts.pallet_rpy, 3) if opts.pallet_rpy else [0.0, 0.0, 0.0]
opts.target_poses = normalize_target_poses(opts.target_poses) if opts.target_poses else []

# ==== Настройка параметров Webots ====
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

if opts.objects_config:
    try:
        resolved_cfg = load_objects_config(opts.objects_config)
        apply_objects_config(robot, resolved_cfg)
        print(f"[INFO] Конфигурация объектов применена из: {opts.objects_config}")
    except Exception as e:
        print(f"[WARN] Ошибка применения конфигурации объектов: {e}")

# Спавн препятсвия
spawner = ObstacleSpawner(supervisor=robot)

# box_def = spawner.spawn_box(
#         name="OBJ_COLISION",
#         translation=(0.30, 0.0, 0.83),
#         size=(0.20, 0.10, 0.30),
#         color=(220, 80, 80),
#         static=True,
#         mass=10.0
#     )

model = LBRiiwaR800Model()
state_q = model.qz
comm = WebotsJsonComm(robot.getDevice("supervisor_receiver"),
                      robot.getDevice("supervisor_emitter"))
ik_solver = lambda qc, xyz, rpy: solve_ik(model, qc, xyz, rpy)
motion = MotionTracker(0.05)
cmd_builder = CommandBuilder([f"lbr_A{i+1}" for i in range(7)], ["camera_motor"])
planer = TrajectoryPlannerComponent(model)

comm.enable(timestep)
transform_fn = lambda xyz: transform_world_to_local(robot_node, xyz)

# ==== Выбор целей====
robot_node = robot.getFromDef("KUKA")
if robot_node is None:
    raise RuntimeError("DEF 'KUKA' не найден в мире. Убедись, что у робота есть DEF KUKA.")

missing = [name for name in opts.objects if robot.getFromDef(name) is None]
if missing:
    raise RuntimeError(f"Не найдены объекты с DEF: {missing}")

targets = [
    WebotsTargetObject(robot_node=robot_node, object_node=robot.getFromDef(w_def))
    for w_def in opts.objects
]
cords_rpy = [t.pose for t in targets] # [np.array, np.array], array shape 3
num_objs = len(cords_rpy)

InitialObjectsSnapshotLogger(
    robot=robot,
    robot_node=robot_node,
    object_defs=opts.objects,
    transform_fn=transform_fn,
    outfile_path=OBJECTS_INFO_JSON_PATH
).snapshot()

for i, cord in enumerate(cords_rpy):
    print(f"Позиция объекта {i+1}: {cord}")

# ==== Построение команд (ЗДЕСЬ ДОЛЖНА БЫТЬ LLM) ====
if have_pallet(opts):
    pallet_poses = generate_pallet_poses(
        origin_xyz=opts.pallet_origin,
        base_rpy=opts.pallet_rpy,
        nx=opts.pallet_nx,
        ny=opts.pallet_ny,
        nz=opts.pallet_nz,
        sx=opts.pallet_sx,
        sy=opts.pallet_sy,
        sz=opts.pallet_sz,
        order=opts.pallet_order,
        limit=num_objs
    )
    if len(pallet_poses) < num_objs:
        print(f"[WARN] На палете {len(pallet_poses)} мест, а объектов {num_objs}. "
              f"Будут разложены только первые {len(pallet_poses)}.")
        cords_rpy = cords_rpy[:len(pallet_poses)]
        targets = targets[:len(pallet_poses)]
        num_objs = len(cords_rpy)

    commands = build_pick_place_pairwise(
        pick_list=cords_rpy,
        target_poses=pallet_poses,
        delta_z=opts.delta_z
    )
else:
    if len(opts.target_poses) == 1:
        commands = build_pick_place_commands(
            cords_list=cords_rpy,
            target_pose=opts.target_poses[0],
            delta_z=opts.delta_z,
            place_step_z=opts.place_step_z,
            gap=opts.gap,
            start_index=0
        )
    elif len(opts.target_poses) == num_objs:
        commands = build_pick_place_pairwise(
            pick_list=cords_rpy,
            target_poses=opts.target_poses,
            delta_z=opts.delta_z
        )
    else:
        raise ValueError(
            f"Число целевых поз ({len(opts.target_poses)}) должно быть равно 1, "
            f"{num_objs} (как объектов) или задай параметры палеты."
        )

# with open(JSON_PATH, "w", encoding="utf-8") as f:
#     json.dump(commands, f, ensure_ascii=False, indent=2)

# ==== Получение данных с JSON ====
with open(JSON_PATH, "r", encoding="utf-8") as f:
    commands = json.load(f)
if commands and isinstance(commands[0], list):
    commands = commands[0]

# ==== план по объектам для оценки точности ====
objects_plan = []
planned_end_xyz_list = []

if have_pallet(opts):
    planned_end_xyz_list = [pose[:3] for pose in pallet_poses]
elif len(opts.target_poses) == 1:
    base_xyz = opts.target_poses[0][:3]
    step = float(opts.place_step_z)
    gap_val = float(opts.gap)
    for i in range(num_objs):
        planned_end_xyz_list.append([base_xyz[0], base_xyz[1], float(base_xyz[2] + i * step + gap_val)])
elif len(opts.target_poses) == num_objs:
    planned_end_xyz_list = [pose[:3] for pose in opts.target_poses]

for idx, (w_def, p) in enumerate(zip(opts.objects, cords_rpy)):
    start_xyz, _ = split_xyz_rpy(p, default_rpy=[0, 0, 0])
    if idx < len(planned_end_xyz_list):
        end_xyz = planned_end_xyz_list[idx]
    else:
        end_xyz = [float(start_xyz[0]), float(start_xyz[1]), float(start_xyz[2])]

    objects_plan.append({
        "def": w_def,
        "start_xyz": [float(start_xyz[0]), float(start_xyz[1]), float(start_xyz[2])],
        "end_xyz": [float(end_xyz[0]), float(end_xyz[1]), float(end_xyz[2])],
    })


# ==== Планирование траектоии ====
trajectory = []
current_q = state_q.copy()

place_done_markers = []
pending_release = False   # флаг: только что была команда 'grab': False
object_place_counter = 0  # счётчик уложенных объектов

for cmd in commands:
    if cmd["command"] == "move":
        xyz = cmd["args"][:3]
        rpy = cmd["args"][3:]
        success = planer.plan(current_q, xyz, rpy, speed_scale=0.5)
        if success:
            for point in planer.trajectory:
                trajectory.append({"joints": point, "gripper": None})
            current_q = planer.trajectory[-1]

            if pending_release:
                place_done_markers.append({
                    "traj_index": len(trajectory) - 1,
                    "object_idx": object_place_counter
                })
                object_place_counter += 1
                pending_release = False
        else:
            print(f"[WARN] Не удалось построить траекторию для: {xyz}, {rpy}")

    elif cmd["command"] == "grab":
        if len(trajectory) == 0:
            print("[ERROR] Нет предыдущей точки для команды grab.")
            continue

        last_q = trajectory[-1]["joints"]
        n_repeat = int(0.2 / dt)

        for i in range(n_repeat):
            if i == n_repeat // 2:
                trajectory.append({"joints": last_q, "gripper": cmd["args"]})
            trajectory.append({"joints": last_q, "gripper": None})

        if cmd["args"] is False:
            pending_release = True

print(f"[INFO] Сформирована траектория из {len(trajectory)} шагов")

logger = MoveResultLogger(
    robot=robot,
    robot_node=robot_node,
    objects_plan=objects_plan,
    place_done_markers=place_done_markers,
    results_path=RESULTS_JSON_PATH,
    transform_fn=transform_fn
)

# ==== Основной цикл ====
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

            logger.try_log(index - 1)

    comm.send({
        "source": robot.getName(),
        "type": "robot_position",
        "data": cmd_builder.command
    })

comm.disable()
logger.finalize()
