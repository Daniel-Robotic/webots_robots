from typing import List, Type


def find_devices(robot, device_type: Type):
    out: List = []
    for i in range(robot.getNumberOfDevices()):
        d = robot.getDeviceByIndex(i)
        if isinstance(d, device_type):
            out.append(d)
    return out


def is_gripper_motor(name: str) -> bool:
    low = name.lower()
    return any(k in low for k in ("finger", "gripper", "hand"))