import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))
sys.path.append(str(Path(__file__).resolve().parents[2]))

from controller import Supervisor
from extensions.kinematics.robot_models import LBRiiwaR800Model
from extensions.kinematics.solvers import solve_ik
from extensions.webots.communication import WebotsJsonComm
from extensions.webots.target import WebotsTargetGizmo
from extensions.core.motion import MotionTracker
from extensions.core.commands import CommandBuilder


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
    
    # обновляем состояние робота
    for m in msgs:
        if m.get("type") == "LBRiiwa7R800_current_pose":
            state_q = list(m["data"]["joints"].values())[:-1]

    # IK — если ещё нет цели
    if not cmd_builder.has_target:
        xyz, rpy = target.pose
        q_star = ik_solver(state_q, xyz, rpy)

        if q_star is not None:
            cmd_builder.set_target(q_star)

    # Проверяем достижение цели
    if cmd_builder.has_target and motion.target_reached(state_q, cmd_builder.target):
        cmd_builder.clear_target()

    cmd_builder.gripper_open = target.gripper
    comm.send({"source": robot.getName(), 
               "type": "robot_position", 
               "data": cmd_builder.command})

comm.disable()
