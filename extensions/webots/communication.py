import json
from typing import Any, List
from controller import Receiver, Emitter
from ..core.communication import BaseCommunication


class WebotsJsonComm(BaseCommunication):
    def __init__(self, receiver: Receiver, emitter: Emitter):
        self._rx, self._tx = receiver, emitter
        self._messages: List[Any] = []

    # ------- BaseCommunication ----
    def enable(self, timestep: int) -> None:
        self._rx.enable(int(timestep))

    def disable(self) -> None:
        self._rx.disable()

    def receive(self) -> List[Any]:
        self._messages.clear()
        while self._rx.getQueueLength() > 0:
            raw = self._rx.getString()
            try:
                self._messages.append(json.loads(raw))
            finally:
                self._rx.nextPacket()
        return self._messages

    def send(self, payload: Any) -> None:
        self._tx.send(json.dumps(payload).encode("utf-8"))

    # ------- helper ---------------
    @property
    def messages(self) -> List[Any]:
        return self._messages