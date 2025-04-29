import os
import sys
# Добавить родительскую папку (controllers/)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from typing import List
from core import find_devices, send_message
from controller import Robot, Camera, RangeFinder, Emitter, Receiver

# Создание робота.
robot = Robot()
emitter: Emitter = robot.getDevice(robot.getName() + "_emitter")
receiver: Receiver = robot.getDevice(robot.getName() + "_receiver")

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


while robot.step(timestep) != -1:

    for camera in cameras:
        recognized_objects = camera.getRecognitionObjects()
        if verbose:
            print(f"[{camera.getName()}] Найдено объектов: {len(recognized_objects)}")
        
        obj_info = []
        for obj in recognized_objects:
            obj_info.append({
                "position": list(obj.getPosition()),
                "orientation": list(obj.getOrientation()),
                "size": list(obj.getSize()),
                "model": str(obj.getModel()),
                "position_on_image": list(obj.getPositionOnImage())
            })


            if verbose:
                print(f"  Объект: модель={obj_info[-1]['model']}, позиция={obj_info[-1]['position']}")

        send_message(emitter=emitter,
                     source_name=robot.getName(),
                     message_type="recognized_objects",
                     data={"objects": obj_info})
            

# Отключение всех устройств
for camera in cameras:
    camera.recognitionDisable()
    camera.disable()

for range_finder in range_finders:
    range_finder.disable()