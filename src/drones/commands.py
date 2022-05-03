from abc import abstractmethod
from collections import deque
import logging
from math import nan
from typing import Deque, Optional, Protocol, Tuple

from pymavlink.mavutil import mavfile

from drones.mavlink_types import FlightMode


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


class Takeoff(Command):
    def __init__(
        self, height_m: float, yaw_angle: float = nan, ascend_rate_mps: float = 10.0
    ) -> None:
        super().__init__()
        self.height_m = height_m

    def __call__(self, mavlink_connection: mavfile) -> None:
        logging.info("Executing takeoff")
        from pymavlink.dialects.v20.ardupilotmega import MAVLink, MAV_CMD_NAV_TAKEOFF

        mav: MAVLink = mavlink_connection.mav
        mav.command_long_send(
            target_system=mavlink_connection.target_system,
            target_component=mavlink_connection.target_component,
            command=MAV_CMD_NAV_TAKEOFF,
            confirmation=0,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=nan,
            param5=0.0,
            param6=0.0,
            param7=self.height_m,
        )


class FlightMode(Enum):
    """
    For a description of the different supported flight modes, see https://ardupilot.org/copter/docs/flight-modes.html

    See https://mavlink.io/en/messages/ardupilotmega.html#COPTER_MODE for the full list of flight modes and their enum
    values
    """

    GUIDED = 4
    LAND = 9


class SetFlightMode(Command):
    def __init__(self, mode: FlightMode) -> None:
        super().__init__()
        self.mode = mode

    def __call__(self, mavlink_connection: mavfile) -> None:
        logging.info("Setting flight mode")
        mavlink_connection.set_mode(self.mode.value)
