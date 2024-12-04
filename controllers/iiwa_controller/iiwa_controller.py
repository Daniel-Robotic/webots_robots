import math
from typing import List
from controller import (Robot, Motor, 
                        PositionSensor, Supervisor)


def rad2deg(rad: float):
    return rad * (180 / math.pi)


def deg2rad(deg: float):
    return deg * (math.pi / 180)



# create the Robot instance.
robot = Robot()


def gripper(mode: float):
    gripR: Motor = robot.getDevice("joint_main_fingerR")
    gripL: Motor = robot.getDevice("joint_main_fingerL")

    gripR.setVelocity(0.5)
    gripL.setVelocity(0.5)

    gripR.setPosition(0)
    gripL.setPosition(0)

    if mode:
        gripR.setPosition(-0.01)
        gripL.setPosition(0.01)


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

link_name = ["lbr_A1", "lbr_A2", "lbr_A3",
            "lbr_A4", "lbr_A5", "lbr_A6",
            "lbr_A7"]


joints: List[Motor] = [robot.getDevice(i) for i in link_name]
camera_frame: Motor = robot.getDevice("joint_main_camera_frame")


joints_sensor: List[PositionSensor] = [robot.getDevice(f"{i}_sensor") for i in link_name]

for sensor in joints_sensor:
    sensor.enable(10)

joints[3].setVelocity(0.7)
joints[3].setPosition(deg2rad(-90))

joints[5].setVelocity(0.7)
joints[5].setPosition(deg2rad(90))

camera_frame.setVelocity(0.5)
camera_frame.setPosition(deg2rad(89))

gripper(1)

while robot.step(timestep) != -1:
    
    pass

# Enter here exit cleanup code.
