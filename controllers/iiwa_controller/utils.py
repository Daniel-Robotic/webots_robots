import math
from controller import Motor
from typing import Dict, List

def rad2deg(rad: float):
    return rad * (180 / math.pi)


def deg2rad(deg: float):
    return deg * (math.pi / 180)


def is_gripper_motor(name: str) -> bool:
    """
        Проверяет, является ли мотор захватом по названию.
    """
    name = name.lower()
    return ("finger" in name) or ("gripper" in name) or ("hand" in name)


def set_joint_positions(motors: Dict[str, Motor], positions: List[float]):
    """
        Устанавливает позиции всех суставов в указанном порядке.
        positions - список целевых позиций (по порядку моторов arm_motors) rad.
    """
    for motor, position in zip(motors.values(), positions):
        motor.setPosition(position)
        
def control_gripper(gripper_motors: Dict[str, Motor], open: bool):
    
    if not gripper_motors:
        print("[INFO] No gripper motors found. Skipping gripper control.")

    for name, motor in gripper_motors.items():
        motor.setVelocity(0.5)
        name = name.lower()

        target_pos = 0.0

        if "r" in name:  # Правый палец
            target_pos = -0.01 if open else 0.0
        if "l" in name:  # Левый палец
            target_pos = 0.01 if open else 0.0

        motor.setPosition(target_pos)