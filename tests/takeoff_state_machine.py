import enum

from async_state_machine import StateMachine, State
from async_state_machine.transitions import timeout

from drones.drone_client import DroneClient
from drones.mavlink_types import FlightMode


@enum.unique
class TakeoffStateNames(enum.Enum):
    AWAIT_PREFLIGHT = "Await Preflight"
    AWAIT_GUIDED_MODE = "Await Guided Mode"
    AWAIT_ARM = "Await Arm"
    TAKING_OFF = "Taking off"

    IN_THE_AIR = "In the air"
    ERROR = "Error"


def get_takeoff_state_machine(sim_drone: DroneClient, altitude: float = 15) -> StateMachine:
    return StateMachine(
        [
            State(
                name=TakeoffStateNames.AWAIT_PREFLIGHT,
                transitions={
                    TakeoffStateNames.AWAIT_GUIDED_MODE: sim_drone.preflight_finished(),
                    TakeoffStateNames.ERROR: timeout(secs=120),
                },
            ),
            State(
                name=TakeoffStateNames.AWAIT_GUIDED_MODE,
                transitions={
                    TakeoffStateNames.AWAIT_ARM: sim_drone.set_flight_mode(
                        FlightMode.GUIDED
                    ),
                    TakeoffStateNames.ERROR: timeout(secs=10),
                },
            ),
            State(
                name=TakeoffStateNames.AWAIT_ARM,
                transitions={
                    TakeoffStateNames.TAKING_OFF: sim_drone.arm(),
                    TakeoffStateNames.ERROR: timeout(secs=10),
                },
            ),
            State(
                name=TakeoffStateNames.TAKING_OFF,
                transitions={
                    TakeoffStateNames.IN_THE_AIR: sim_drone.takeoff(height_m=altitude),
                    TakeoffStateNames.ERROR: timeout(secs=25),
                },
            ),
            State(name=TakeoffStateNames.IN_THE_AIR, transitions={}),
            State(name=TakeoffStateNames.ERROR, transitions={}),
        ]
    )
