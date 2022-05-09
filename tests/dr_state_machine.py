import enum

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
    PITCHING = "Pitching forward"
    INBOUND = "Inbound"
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
                    # TODO
                    # DRStateNames.TURNING: flying_sim_drone.change_altitude(
                    #     height_m=50
                    # ),  # TODO const
                    DRStateNames.TURNING: timeout(secs=3),
                    DRStateNames.CANCELLING_DR: flying_sim_drone.dr_cancelled(),
                    # TODO what do we do if this times out? Probably want to DR anyway
                },
            ),
            State(
                name=DRStateNames.TURNING,
                transitions={
                    DRStateNames.PITCHING: flying_sim_drone.turn(
                        heading=15
                    ),  # TODO choose heading
                    DRStateNames.CANCELLING_DR: flying_sim_drone.dr_cancelled(),
                },
            ),
            State(
                name=DRStateNames.PITCHING,
                transitions={
                    DRStateNames.INBOUND: flying_sim_drone.set_attitude(
                        pitch_deg=-3, heading_deg=15
                    ),
                    DRStateNames.CANCELLING_DR: flying_sim_drone.dr_cancelled(),
                    # TODO what do we do if this times out?
                },
            ),
            State(
                name=DRStateNames.INBOUND,
                transitions={
                    DRStateNames.DESCENDING: timeout(secs=5),  # TODO calculate distance
                    DRStateNames.CANCELLING_DR: flying_sim_drone.dr_cancelled(),
                    # TODO what do we do if this times out?
                },
            ),
            State(
                name=DRStateNames.DESCENDING,
                transitions={
                    # TODO
                    # DRStateNames.LANDING: flying_sim_drone.change_altitude(
                    #     height_m=10
                    # ),  # TODO const
                    DRStateNames.LANDING: timeout(secs=3),
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
