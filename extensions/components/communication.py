import json
from controller import Emitter, Receiver
from typing import List, Dict, Union

class CommunicationComponent:
    def __init__(self, 
                 receiver: Receiver,
                 emitter: Emitter,
                 robot_name: str):
        
        self._receiver = receiver
        self._emitter = emitter
        self.robot_name = robot_name
        self._messages = []

    def __dir__(self):
        self.disable()
    
    @property
    def messages(self) -> List:
        return self._messages

    @property
    def robot_name(self) -> str:
        return self._robot_name
    
    @robot_name.setter
    def robot_name(self, value: str):
        self._robot_name = value

    def enable(self, timestep: Union[float, int]) -> None:
        self._receiver.enable(timestep)

    def disable(self) -> None:
        self._receiver.disable()

    def receive(self) -> None:
        while self._receiver.getQueueLength() > 0:
            try:
                data = self._receiver.getString()
                message = json.loads(data)
                self._messages.append(message)
            except Exception as e:
                print(f"[ERROR] Failed to parse message: {e}")

            self._receiver.nextPacket()

    def send(self, msg_type: str, data: Union[Dict, str, List]):
        message = {
            "source": self._robot_name,
            "type": msg_type,
            "data": data
        }

        payload = json.dumps(message).encode("utf-8")
        self._emitter.send(payload)
    