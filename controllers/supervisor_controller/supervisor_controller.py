import os
import sys
# Добавить родительскую папку (controllers/)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from controller import Supervisor, Receiver
from core import receive_all_messages

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

receiver: Receiver = robot.getDevice("supervisor_receiver")
receiver.enable(timestep)


while robot.step(timestep) != -1:
    # TODO: Получаем все данные с каждой камеры, потом можно отправлять данные на робота
    msg = receive_all_messages(receiver=receiver)
    print(msg)

    # TODO: Отправка данных на робота (DEV)

receiver.disable()