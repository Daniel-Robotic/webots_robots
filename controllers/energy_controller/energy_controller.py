import math
import pandas as pd
from typing import List
from controller import (Robot, Motor, 
                        PositionSensor, Supervisor)


# create the Robot instance.
robot = Robot()

df = pd.read_csv("C:/Users/grabar.dm/Desktop/CalibTraj/calc_energy_webots.csv", sep=",")


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

link_name = ["lbr_A1", "lbr_A2", "lbr_A3",
            "lbr_A4", "lbr_A5", "lbr_A6",
            "lbr_A7"]


joints: List[Motor] = [robot.getDevice(i) for i in link_name]
joints_sensor: List[PositionSensor] = [robot.getDevice(f"{i}_sensor") for i in link_name]

for sensor in joints_sensor:
    sensor.enable(10)

current_step = 0

while robot.step(timestep) != -1:
    # Проверяем, есть ли еще точки для движения
    if current_step >= len(df):
        print("Траектория завершена.")
        break

    # Получаем текущие целевые позиции для суставов
    target_positions = df.iloc[current_step].values

    # Устанавливаем целевые позиции для каждого сустава
    for joint, target_position in zip(joints, target_positions):
        joint.setPosition(target_position)

    # Увеличиваем индекс текущей строки
    current_step += 1

# Enter here exit cleanup code.
