from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Any, List


class BaseCommunication(ABC):
    """Транспортный интерфейс: включает/выключает, шлёт, принимает."""

    @abstractmethod
    def enable(self, timestep: int) -> None: ...

    @abstractmethod
    def disable(self) -> None: ...

    @abstractmethod
    def receive(self) -> List[Any]: ...

    @abstractmethod
    def send(self, payload: Any) -> None: ...