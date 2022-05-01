from abc import abstractmethod
from collections import deque
from dataclasses import dataclass
import logging
from math import nan
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
        logging.info("Executing Arm")
        mavlink_connection.arducopter_arm()


@dataclass
class LocalPositionNED:
    '''Position in meters. Origin is where the '''
    north: float = 0
    east: float = 0
    down: float = 0

class Takeoff(Command):
    def __init__(self, height_m: float, yaw_angle: float = nan, ascend_rate_mps: float = 10.0) -> None:
        super().__init__()
        self.height_m = height_m
        self.yaw_angle = yaw_angle
        self.ascend_rate_mps = ascend_rate_mps
        self.minimum_pitch = 0.0
        self.position = LocalPositionNED(down=-height_m)

    def __call__(self, mavlink_connection: mavfile) -> None:
        logging.info("Executing takeoff")
        from pymavlink.dialects.v20.ardupilotmega import MAVLink, MAV_CMD_NAV_TAKEOFF_LOCAL
        mav: MAVLink = mavlink_connection.mav
        mav.command_long_send(
            target_system=mavlink_connection.target_system,
            target_component=mavlink_connection.target_component,
            command=MAV_CMD_NAV_TAKEOFF_LOCAL,
            confirmation=0,
            param1=self.minimum_pitch,
            param2=0.0,
            param3=self.ascend_rate_mps,
            param4=self.yaw_angle,
            param5=self.position.east,
            param6=self.position.north,
            param7=self.position.down,
        )