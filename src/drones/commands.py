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


class ChangeAltitude(Command):
    def __init__(self, height_m: float, alt_rate_of_change: float = 10.0) -> None:
        """
        Change target altitude at a given rate. This slews the vehicle at a controllable rate between it's previous
        altitude and the new one. (affects GUIDED only. Outside GUIDED, aircraft ignores these commands.

        :param height_m: Target height in meters
        :param alt_rate_of_change: Rate of change, towards new altitude. 0 for maximum rate change.
        """
        super().__init__()
        self.height_m = height_m
        assert (
            alt_rate_of_change >= 0
        ), "Rate of change must be positive, negative numbers will not converge to target altitude!"
        self.alt_rate_of_change = alt_rate_of_change

    def __call__(self, mavlink_connection: mavfile) -> None:
        logging.info(f"Ascending/descending to altitude {self.height_m}")
        from pymavlink.dialects.v20.ardupilotmega import (
            MAVLink,
            MAV_CMD_GUIDED_CHANGE_ALTITUDE,
        )

        mav: MAVLink = mavlink_connection.mav
        mav.command_long_send(
            target_system=mavlink_connection.target_system,
            target_component=mavlink_connection.target_component,
            command=MAV_CMD_GUIDED_CHANGE_ALTITUDE,
            confirmation=0,
            param1=0.0,
            param2=0.0,
            param3=self.alt_rate_of_change,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=self.height_m,
        )


class ChangeHeading(Command):
    def __init__(self, heading: float, rate_of_change: float = 1.0) -> None:
        """
        Change to target heading at a given rate, overriding previous heading/s. This slews the vehicle at a
        controllable rate between it's previous heading and the new one. (affects GUIDED only. Exiting GUIDED returns
        aircraft to normal behaviour defined elsewhere.
        :param heading:
        :param rate_of_change:
        """
        # TODO choose a sane default rate_of_change
        super().__init__()
        assert 0 <= heading < 360
        self.heading = heading
        self.rate_of_change = rate_of_change

    def __call__(self, mavlink_connection: mavfile) -> None:
        logging.info(f"Turning to heading {self.heading}")
        from pymavlink.dialects.v20.ardupilotmega import (
            MAVLink,
            MAV_CMD_GUIDED_CHANGE_HEADING,
            HEADING_TYPE_HEADING,
        )

        mav: MAVLink = mavlink_connection.mav
        mav.command_long_send(
            target_system=mavlink_connection.target_system,
            target_component=mavlink_connection.target_component,
            command=MAV_CMD_GUIDED_CHANGE_HEADING,
            confirmation=0,
            param1=HEADING_TYPE_HEADING,
            param2=self.heading,
            param3=self.rate_of_change,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=0.0,
        )


class SetFlightMode(Command):
    def __init__(self, mode: FlightMode) -> None:
        super().__init__()
        self.mode = mode

    def __call__(self, mavlink_connection: mavfile) -> None:
        logging.info("Setting flight mode")
        mavlink_connection.set_mode(self.mode.value)
