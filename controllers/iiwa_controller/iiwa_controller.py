import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))
sys.path.append(str(Path(__file__).resolve().parents[2]))

from controller import Robot, Motor, PositionSensor
from extensions.webots.communication import WebotsJsonComm
from extensions.utils.device_search import is_gripper_motor

# — init —
robot = Robot()
ts = int(robot.getBasicTimeStep())
comm = WebotsJsonComm(robot.getDevice(f"{robot.getName()}_receiver"),
                      robot.getDevice(f"{robot.getName()}_emitter"))
comm.enable(ts)

arm_motors, grip_motors = {}, {}
arm_sensors, grip_sensors = {}, {}

for i in range(robot.getNumberOfDevices()):
    dev = robot.getDeviceByIndex(i)
    if isinstance(dev, Motor):
        name = dev.getName()
        if is_gripper_motor(name):
            grip_motors[name] = dev
        else:
            arm_motors[name] = dev
        sens_name = name + "_sensor"
        try:
            sens = robot.getDevice(sens_name)
            if isinstance(sens, PositionSensor):
                sens.enable(ts)
                (grip_sensors if is_gripper_motor(name) else arm_sensors)[name] = sens
        except Exception:
            pass

# — helper —
def apply_cmd(joints: dict, gr_open: bool):
    for n, v in joints.items():
        if n in arm_motors:
            arm_motors[n].setPosition(float(v))
    for n, m in grip_motors.items():
        off = 0.01 if (gr_open ^ ("r" in n.lower())) else 0.0
        m.setVelocity(0.5)
        m.setPosition(off if "l" in n.lower() else -off)


def build_state():
    return {
        "joints": {n: s.getValue() for n, s in arm_sensors.items()},
        "gripper": {n: s.getValue() for n, s in grip_sensors.items()},
    }


# — loop —
while robot.step(ts) != -1:

    # Получаем позиции от supervisor
    for m in comm.receive():
        if m.get("source") == "supervisor" and m.get("type") == "robot_position":           
            apply_cmd(m["data"]["joints"], m["data"]["gripper"])

    comm.send({"source": robot.getName(),
               "type": f"{robot.getName()}_current_pose",
               "data": build_state()})

comm.disable()
