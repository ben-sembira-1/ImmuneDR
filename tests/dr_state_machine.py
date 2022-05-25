from dataclasses import dataclass
import enum
from functools import lru_cache
import logging
from typing import TYPE_CHECKING, Protocol

from pymavlink.dialects.v20.ardupilotmega import MAVLink_global_position_int_message

from async_state_machine.transitions.combinators import all_of
from async_state_machine.transitions.types import (
    TransitionCheckerFactory,
    TransitionChecker,
)

from async_state_machine import StateMachine, State
from async_state_machine.transitions import timeout

from drones.drone_client import DroneClient
from drones.geo import GeoLocation
from drones.latest_messages import LatestMessagesCache
from drones.mavlink_types import FlightMode, GlobalPositionInt

# A work around https://github.com/python/mypy/issues/9489
# callback as a class field is tricky, and mypy currently has no support (support reverted)
class LazyFloat(Protocol):
    def __call__(self) -> float:
        ...


class ApproximateDistanceTimeoutCheckerFactory(TransitionCheckerFactory):
    _distance_meter_getter: LazyFloat 
    _expected_speed_mps: float

    def __init__(
        self, *, distance_meter_getter: LazyFloat, expected_speed_mps: float
    ) -> None:
        super().__init__()
        self._distance_meter_getter = distance_meter_getter
        self._expected_speed_mps = expected_speed_mps

    def create_checker(self) -> "TransitionChecker":
        super().create_checker()
        distance_m = self._distance_meter_getter()
        time_estimation_secs = distance_m / self._expected_speed_mps
        logging.info(
            f"Estimates {time_estimation_secs} [sec] to get to destination {distance_m} [m] away"
        )
        return timeout(secs=time_estimation_secs).create_checker()


approximate_distance_timeout = ApproximateDistanceTimeoutCheckerFactory


@enum.unique
class DRStateNames(enum.Enum):
    CANCELLING_DR = "Cancelling DR"
    IN_THE_AIR = "In the air"
    ERROR = "Error"

    GUIDED_NO_GPS = "Entering GUIDED_NOGPS Mode"
    CLIMBING = "Climbing"
    TURNING = "Turning"
    INBOUND = "Inbound"
    LEVELING = "Leveling"
    DESCENDING = "Descending"
    LANDING = "Landing"
    LANDED = "Landed"


def get_dr_state_machine(
    flying_sim_drone: DroneClient,
    dr_target: GeoLocation,
    dr_flying_pitch_deg: float, # neg is forward
    dr_estimated_speed_mps: float,
    dr_climb_throttle: float,
    dr_descending_throttle: float,
    dr_climb_to_height_meters_before_returning: float,
    dr_descending_height_meters_to_change_to_landing: float,
    default_dr_distance: float = float('inf'),
    default_dr_bearing_deg: float = 180,
) -> StateMachine:

    assert 0.5 < dr_climb_throttle <= 1.0, "Climb throttle must be bigger then 0.5 (keep height) and at most 1.0"
    assert 0 <= dr_descending_throttle < 5.0, "Descending throttle must be bigger then 0 (stop engines) and at most 5.0 (keep height)"
    latest_messages = LatestMessagesCache(flying_sim_drone)

    def get_bearing_to_home() -> float:
        latest = latest_messages.get_latest_message(MAVLink_global_position_int_message)
        logging.info(f"Calculating the bearing to dr target based on latest location: {latest}")
        if latest is None:
            logging.warn(
                f"No latest location was cached, using default heading: {default_dr_bearing_deg}"
            )
            return default_dr_bearing_deg
        assert isinstance(latest, MAVLink_global_position_int_message)
        latest_location = GeoLocation.from_message(latest)

        bearing = latest_location.bearing_to(dr_target)
        logging.info(f"Using bearing {bearing}")
        return bearing

    def get_distance_to_home() -> float:
        latest = latest_messages.get_latest_message(MAVLink_global_position_int_message)
        logging.info(f"Calculating the distance to dr target based on latest location: {latest}")
        if latest is None:
            logging.warn(
                f"No latest location was cached, using default distance: {default_dr_bearing_deg}"
            )
            return default_dr_distance
        assert isinstance(latest, MAVLink_global_position_int_message)
        latest_location = GeoLocation.from_message(latest)

        distance = latest_location.distance_to(dr_target)
        logging.info(f"Using distance {distance}")
        return distance

    return StateMachine(
        [
            State(
                name=DRStateNames.IN_THE_AIR,
                transitions={
                    DRStateNames.GUIDED_NO_GPS: flying_sim_drone.ekf_bad(),
                    # TODO Actually only when we lose connection
                },
            ),
            State(
                name=DRStateNames.GUIDED_NO_GPS,
                transitions={
                    DRStateNames.CLIMBING: flying_sim_drone.set_flight_mode(
                        FlightMode.GUIDED_NO_GPS
                    )
                },
            ),
            State(
                name=DRStateNames.CLIMBING,
                transitions={
                    DRStateNames.TURNING: flying_sim_drone.when_global_position(
                        lambda p: p.height_above_ground_m >= dr_climb_to_height_meters_before_returning
                    ),
                    DRStateNames.CLIMBING: flying_sim_drone.set_throttle(
                        dr_climb_throttle
                    ),
                    DRStateNames.CANCELLING_DR: flying_sim_drone.dr_cancelled(),
                    # TODO what do we do if this times out? Probably want to DR anyway
                },
            ),
            State(
                name=DRStateNames.TURNING,
                transitions={
                    DRStateNames.INBOUND: flying_sim_drone.turn(
                        heading_deg=get_bearing_to_home
                    ),
                    DRStateNames.CANCELLING_DR: flying_sim_drone.dr_cancelled(),
                },
            ),
            State(
                name=DRStateNames.INBOUND,
                transitions={
                    DRStateNames.LEVELING: all_of(
                        [
                            approximate_distance_timeout(distance_meter_getter=get_distance_to_home, expected_speed_mps=dr_estimated_speed_mps ),
                            flying_sim_drone.set_attitude(
                                pitch_deg=dr_flying_pitch_deg,
                                heading_deg=get_bearing_to_home,
                                # end_condition=timeout(secs=3)
                            ),
                        ]
                    ),
                    DRStateNames.CANCELLING_DR: flying_sim_drone.dr_cancelled(),  # Level out?
                    # TODO what do we do if this times out?
                },
            ),
            State(
                name=DRStateNames.LEVELING,
                transitions={
                    DRStateNames.DESCENDING: flying_sim_drone.set_attitude(
                        heading_deg=get_bearing_to_home, pitch_deg=0
                    ),
                    # TODO cancellable?
                },
            ),
            State(
                name=DRStateNames.DESCENDING,
                transitions={
                    DRStateNames.LANDING: flying_sim_drone.when_global_position(
                        lambda p: p.height_above_ground_m <= dr_descending_height_meters_to_change_to_landing
                    ),
                    DRStateNames.DESCENDING: flying_sim_drone.set_throttle(
                        dr_descending_throttle
                    ),
                    DRStateNames.CANCELLING_DR: flying_sim_drone.dr_cancelled(),
                    # TODO what do we do if this times out?
                },
            ),
            State(
                name=DRStateNames.LANDING,
                transitions={
                    DRStateNames.LANDED: flying_sim_drone.land(),
                    # TODO what happens if DR is cancelled here?
                    # TODO can this fail/time out?
                },
            ),
            State(
                name=DRStateNames.LANDED,
                transitions={},
            ),
            State(
                name=DRStateNames.CANCELLING_DR,
                transitions={
                    DRStateNames.IN_THE_AIR: flying_sim_drone.set_flight_mode(
                        FlightMode.ALT_HOLD
                    ),
                },
            ),
            State(name=DRStateNames.ERROR, transitions={}),
        ],
    )
