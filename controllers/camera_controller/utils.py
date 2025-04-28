from typing import List
from controller import Robot

def find_devices(robot: Robot, 
                 device_type) -> List:
    """
    Ищет все устройства заданного типа у робота и возвращает их список.
    
    :param robot: объект Robot
    :param device_type: класс устройства, например Camera, RangeFinder и т.д.
    :return: список найденных устройств
    """
    devices = []
    for i in range(robot.getNumberOfDevices()):
        device = robot.getDeviceByIndex(i)
        if isinstance(device, device_type):
            devices.append(device)
    return devices