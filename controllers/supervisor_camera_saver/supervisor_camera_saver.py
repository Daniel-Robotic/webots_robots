import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))
sys.path.append(str(Path(__file__).resolve().parents[2]))

import os
from controller import Supervisor
from extensions.kinematics.robot_models import LBRiiwaR800Model
from extensions.webots.communication import WebotsJsonComm
from extensions.core.motion import MotionTracker
from extensions.core.commands import CommandBuilder

# ================ Меням путь до файла и разделить ==========================

# IMAGE_FOLDER = "/home/user/dev/webots_projects/webots_robots/image_saver_example"
IMAGE_FOLDER = os.path.expandvars("${HOME}/dev/webots_projects/webots_robots/image_saver_example")
# ===========================================================================


# =============== Автоматически все выполниться =============================
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

comm = WebotsJsonComm(robot.getDevice("supervisor_receiver"),
                      robot.getDevice("supervisor_emitter"))
comm.enable(timestep)

model = LBRiiwaR800Model()
state_q = model.qz
step_counter = 0

motion = MotionTracker(0.01)
cmd_builder = CommandBuilder([f"lbr_A{i+1}" for i in range(7)], ["camera_motor"])
cmd_builder.set_target(model.qr)


while robot.step(timestep) != -1:
    step_counter += 1
    
    msgs = comm.receive()
    
    for m in msgs:
        if m.get("type") == "LBRiiwa7R800_current_pose":
            state_q = list(m["data"]["joints"].values())[:-1]

    if cmd_builder.has_target and motion.target_reached(state_q, cmd_builder.target):
        cmd_builder.clear_target()

    comm.send({"source": robot.getName(), 
                   "type": "robot_position", 
                   "data": cmd_builder.command})
    
    if step_counter % 100 == 0:
        step_counter = 0
        comm.send({"source": robot.getName(), 
               "type": "save_image", 
               "data": {"folder": IMAGE_FOLDER,
                        "type": "rgbd"} 
               })
        

comm.disable()
