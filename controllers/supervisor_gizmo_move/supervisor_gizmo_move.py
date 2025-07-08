import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))
sys.path.append(str(Path(__file__).resolve().parents[2]))


from controller import Supervisor, Receiver, Emitter

from extensions.kinematics.robot_models import LBRiiwaR800Model
from extensions.kinematics.kinematics import solve_ik
from extensions.components.communication import CommunicationComponent
from extensions.components.robot_state import RobotManipulatorStateComponent
from extensions.components.positions import (TargetGizmaComponent, 
                                             IKComponent, 
                                             MotionTrackerComponent, 
                                             CommandBuilderManipulatorComponent)

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

robot_model = LBRiiwaR800Model()
robot_node = robot.getFromDef("KUKA")                       # Получаем DEF робота
target_node = robot.getFromDef("TARGET_GRIPPER")            # Получаем DEF гизмы, чтобы перемещать вручную
joint_names = [f"A{i+1}" for i in range(len(robot_model.qz)-1)]
joint_names.append("camera_motor") 


emitter: Emitter = robot.getDevice("supervisor_emitter")
receiver: Receiver = robot.getDevice("supervisor_receiver")
comm = CommunicationComponent(receiver=receiver,
                             emitter=emitter,
                             robot_name=robot.getName())
comm.enable(timestep)

state = RobotManipulatorStateComponent(robot_model=robot_model)
target = TargetGizmaComponent(robot_node=robot_node,
                              target_node=target_node)
ik = IKComponent(robot_model=robot_model,
                 solver_ik=solve_ik)
motion_tracker = MotionTrackerComponent(tolerance=0.01)
command_builder = CommandBuilderManipulatorComponent(joint_names)

while robot.step(timestep) != -1:

    comm.receive()
    state.update(source="LBRiiwa7R800",
                 message_type="LBRiiwa7R800_current_pose",
                 messages=comm.messages)
    
    if not ik.ik_computed:
        pos, rpy = target.get_target_pose()
        ik.compute(state.current_position, pos, rpy)

    if ik.ik_computed and ik.target_q is not None:
        if motion_tracker.is_target_reached(state.current_position, ik.target_q):
            ik.reset()
        else:
            command_builder.build_joint_command(ik.target_q)

    command_builder.set_gripper_state(target.get_gripper_state())
    
    # Отправка данных на робота
    comm.send("robot_position", command_builder.get_command())

comm.disable()