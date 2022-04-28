from abc import abstractmethod
from collections import deque
from typing import Deque, Optional, Protocol, Tuple

from pymavlink.mavutil import mavfile


class Command(Protocol):
    @abstractmethod
    def __call__(self, mavlink_connection: mavfile) -> None:
        ...


class CommandReceiver:
    _inner_queue: Deque[Command]

    def __init__(self, inner_queue: Deque[Command]) -> None:
        self._inner_queue = inner_queue

    def receive(self) -> Optional[Command]:
        try:
            return self._inner_queue.popleft()
        except IndexError:
            return None


class CommandSender:
    _inner_queue: Deque[Command]

    def __init__(self, inner_queue: Deque[Command]) -> None:
        self._inner_queue = inner_queue

    def send(self, command: Command) -> None:
        self._inner_queue.append(command)


def create_command_queue_pair() -> Tuple[CommandSender, CommandReceiver]:
    inner_queue: Deque[Command] = deque()
    return (CommandSender(inner_queue), CommandReceiver(inner_queue))

class Arm(Command):
    def __call__(self, mavlink_connection: mavfile) -> None:
        mavlink_connection.arducopter_arm()
