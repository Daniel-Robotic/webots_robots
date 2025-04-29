"""camera_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import sys
from typing import List
from utils import find_devices
from controller import Robot, Camera, RangeFinder

# Создание робота.
robot = Robot()

verbose = False

for arg in sys.argv[1:]:
    if arg == "--verbose" or arg == "verbose=True" or arg == "verbose=1":
        verbose = True

# Получение шага симуляции
timestep = int(robot.getBasicTimeStep())

# Список всех устройств камеры
cameras: List[Camera] = find_devices(robot=robot,
                                     device_type=Camera)
range_finders: List[RangeFinder] = find_devices(robot=robot,
                                                device_type=RangeFinder)

# Включение всех устройств
for camera in cameras:
    camera.enable(timestep)
    camera.recognitionEnable(timestep)

for range_finder in range_finders:
    range_finder.enable(timestep)

if verbose:
    print(f"Найдено камер для робота {robot.getName()}: {len(cameras)}")
    print(f"Найдено дальномеров для робота {robot.getName()}: {len(range_finders)}")

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    pass

# Отключение всех устройств
for camera in cameras:
    camera.recognitionDisable()
    camera.disable()

for range_finder in range_finders:
    range_finder.disable()