import os
import sys
# Добавить родительскую папку (controllers/)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from typing import Dict
from controller import Robot, Motor, PositionSensor
from core import (deg2rad, 
                 rad2deg, 
                 is_gripper_motor,
                 set_joint_positions,
                 control_gripper)



# create the Robot instance.
robot = Robot()
verbose = False

for arg in sys.argv[1:]:
    if arg == "--verbose" or arg == "verbose=True" or arg == "verbose=1":
        verbose = True

timestep = int(robot.getBasicTimeStep())

arm_motors: Dict[str, Motor] = {}
arm_sensors: Dict[str, PositionSensor] = {}

gripper_motors: Dict[str, Motor] = {}
gripper_sensors: Dict[str, PositionSensor] = {}

for i in range(robot.getNumberOfDevices()):
    device = robot.getDeviceByIndex(i)
    if isinstance(device, Motor):
        name = device.getName()

        if is_gripper_motor(name):
            gripper_motors[name] = device
            # Пробуем найти связанный сенсор
            sensor_name = name + "_sensor"
            try:
                sensor = robot.getDevice(sensor_name)
                if isinstance(sensor, PositionSensor):
                    gripper_sensors[name] = sensor
                    sensor.enable(timestep)
            except Exception:
                print(f"[WARN] No sensor found for gripper motor '{name}'")
        else:
            arm_motors[name] = device
            sensor_name = name + "_sensor"
            try:
                sensor = robot.getDevice(sensor_name)
                if isinstance(sensor, PositionSensor):
                    arm_sensors[name] = sensor
                    sensor.enable(timestep)
            except Exception:
                print(f"[WARN] No sensor found for arm motor '{name}'")

num_joints = len(arm_motors)

while robot.step(timestep) != -1:
    # TODO: Поменяется после получения данных с эмитера
    target_positions = [0.0 for _ in range(num_joints)]
    set_joint_positions(motors=arm_motors,
                        positions=target_positions)
    
    control_gripper(gripper_motors=gripper_motors,
                    open=False)
    
    if verbose:
        # Чтение текущих углов суставов
        for name, sensor in arm_sensors.items():
            position = sensor.getValue()
            print(f"Joint {name}: {rad2deg(position):.2f} deg")
        
        # Чтение текущих позиций пальцев захвата
        for name, sensor in gripper_sensors.items():
            position = sensor.getValue()
            print(f"Gripper {name}: {position:.3f} rad")

    break
