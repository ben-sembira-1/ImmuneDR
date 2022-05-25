from dataclasses import dataclass
import enum
from functools import lru_cache
import logging
import math

from pymavlink.dialects.v20.ardupilotmega import MAVLink_global_position_int_message

from async_state_machine.transitions.combinators import all_of

from async_state_machine import StateMachine, State
from async_state_machine.transitions import timeout

from drones.drone_client import DroneClient
from drones.latest_messages import LatestMessagesCache
from drones.mavlink_types import FlightMode, GlobalPositionInt


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


# Click: -35.36116244 149.16625641 (-35°21'40.18" 149°09'58.52") (S 55 696817 6084750)
# Click: -35.36192590 149.16296565
# Upper left = -35.36151054 149.16379178 (-35°21'41.44" 149°09'49.65") (S 55 696592 6084716)
# home = -35.36326086 149.16523056
@dataclass
class LonLat:
    lon: float
    lat: float


upper_left = LonLat(lat=-35.36151054, lon=149.16379178)
home = LonLat(lat=-35.36326086, lon=149.16523056)


def get_bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    d_lon = lon2 - lon1
    x = math.cos(math.radians(lat2)) * math.sin(math.radians(d_lon))
    y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(
        math.radians(lat1)
    ) * math.cos(math.radians(lat2)) * math.cos(math.radians(d_lon))
    brng = math.atan2(x, y)
    brng = math.degrees(brng) % 360

    return brng


def get_dr_state_machine(
    flying_sim_drone: DroneClient,
    dr_target: LonLat,
    default_dr_bearing_deg: float = 180,
) -> StateMachine:
    latest_messages = LatestMessagesCache(flying_sim_drone)

    def get_bearing_to_home() -> float:
        latest = latest_messages.get_latest_message(MAVLink_global_position_int_message)
        logging.info(f"Calculating the home heading based on latest location: {latest}")
        if latest is None:
            logging.warn(
                f"No latest location was cached, using default heading: {default_dr_bearing_deg}"
            )
            return default_dr_bearing_deg
        assert isinstance(latest, MAVLink_global_position_int_message)
        latest_location = GlobalPositionInt.from_message(latest)

        bearing = get_bearing_deg(
            latest_location.latitude_deg,
            latest_location.longitude_deg,
            dr_target.lat,
            dr_target.lon,
        )
        logging.info(f"Using bearing {bearing}")
        return bearing

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
                    # TODO const
                    DRStateNames.TURNING: flying_sim_drone.when_global_position(
                        lambda p: p.height_above_ground_m >= 50
                    ),
                    DRStateNames.CLIMBING: flying_sim_drone.set_throttle(
                        0.8
                    ),  # TODO speed?
                    DRStateNames.CANCELLING_DR: flying_sim_drone.dr_cancelled(),
                    # TODO what do we do if this times out? Probably want to DR anyway
                },
            ),
            State(
                name=DRStateNames.TURNING,
                transitions={
                    DRStateNames.INBOUND: flying_sim_drone.turn(
                        heading_deg=get_bearing_to_home
                    ),  # TODO choose heading
                    DRStateNames.CANCELLING_DR: flying_sim_drone.dr_cancelled(),
                },
            ),
            State(
                name=DRStateNames.INBOUND,
                transitions={
                    # TODO calculate distance
                    DRStateNames.LEVELING: all_of(
                        [
                            timeout(secs=3),
                            flying_sim_drone.set_attitude(
                                pitch_deg=-15,
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
                        lambda p: p.height_above_ground_m <= 15
                    ),
                    DRStateNames.DESCENDING: flying_sim_drone.set_throttle(
                        0.2
                    ),  # TODO speed?
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
