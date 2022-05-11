import enum

from async_state_machine.transitions.combinators import all_of

from async_state_machine import StateMachine, State
from async_state_machine.transitions import timeout

from drones.drone_client import DroneClient
from drones.mavlink_types import FlightMode


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


def get_dr_state_machine(flying_sim_drone: DroneClient) -> StateMachine:
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
                        lambda p: abs(50 - p.height_above_ground_m) <= 5
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
                        heading=15
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
                                heading_deg=15,
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
                        heading_deg=15, pitch_deg=0
                    ),
                    # TODO cancellable?
                },
            ),
            State(
                name=DRStateNames.DESCENDING,
                transitions={
                    DRStateNames.LANDING: flying_sim_drone.when_global_position(
                        lambda p: abs(15 - p.height_above_ground_m) <= 3,
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
        ]
    )
