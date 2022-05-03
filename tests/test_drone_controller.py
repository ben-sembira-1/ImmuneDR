import enum

from async_state_machine import StateMachine, State
from async_state_machine.transitions.timeout import timeout
from drones.mavlink_types import FlightMode

from drones.drone_client import DroneClient

from tests.state_machine_utils import run_until


def test_drone_heartbeat(sim_drone: DroneClient) -> None:
    @enum.unique
    class StateNames(enum.Enum):
        AWAIT_HEARTBEAT = "Await heartbeat"
        SUCCESS = "Success"
        ERROR = "Error"

    sm = StateMachine(
        [
            State(
                name=StateNames.AWAIT_HEARTBEAT,
                transitions={
                    StateNames.SUCCESS: sim_drone.heartbeat(),
                    StateNames.ERROR: timeout(secs=10),
                },
            ),
            State(name=StateNames.SUCCESS, transitions={}),
            State(name=StateNames.ERROR, transitions={}),
        ]
    )

    run_until(sm, target=StateNames.SUCCESS, error_states={StateNames.ERROR})


def test_drone_armed(sim_drone: DroneClient) -> None:
    @enum.unique
    class StateNames(enum.Enum):
        AWAIT_PREFLIGHT = "Await Preflight"
        AWAIT_ARM = "Await Arm"
        ARMED = "Armed"
        ERROR = "Error"

    sm = StateMachine(
        [
            State(
                name=StateNames.AWAIT_PREFLIGHT,
                transitions={
                    StateNames.AWAIT_ARM: sim_drone.preflight_finished(),
                    StateNames.ERROR: timeout(secs=120),
                },
            ),
            State(
                name=StateNames.AWAIT_ARM,
                transitions={
                    StateNames.ARMED: sim_drone.arm(),
                    StateNames.ERROR: timeout(secs=10),
                },
            ),
            State(name=StateNames.ARMED, transitions={}),
            State(name=StateNames.ERROR, transitions={}),
        ]
    )

    run_until(sm, target=StateNames.ARMED, error_states={StateNames.ERROR})


def test_drone_takeoff(sim_drone: DroneClient) -> None:
    @enum.unique
    class StateNames(enum.Enum):
        AWAIT_PREFLIGHT = "Await Preflight"
        AWAIT_GUIDED_MODE = "Await Guided Mode"
        AWAIT_ARM = "Await Arm"
        TAKING_OFF = "Taking off"

        IN_THE_AIR = "In the air"
        ERROR = "Error"

    sm = StateMachine(
        [
            State(
                name=StateNames.AWAIT_PREFLIGHT,
                transitions={
                    StateNames.AWAIT_GUIDED_MODE: sim_drone.preflight_finished(),
                    StateNames.ERROR: timeout(secs=120),
                },
            ),
            State(
                name=StateNames.AWAIT_GUIDED_MODE,
                transitions={
                    StateNames.AWAIT_ARM: sim_drone.set_flight_mode(FlightMode.GUIDED),
                    StateNames.ERROR: timeout(secs=10),
                },
            ),
            State(
                name=StateNames.AWAIT_ARM,
                transitions={
                    StateNames.TAKING_OFF: sim_drone.arm(),
                    StateNames.ERROR: timeout(secs=10),
                },
            ),
            State(
                name=StateNames.TAKING_OFF,
                transitions={
                    StateNames.IN_THE_AIR: sim_drone.takeoff(height_m=15),
                    StateNames.ERROR: timeout(secs=25),
                },
            ),
            State(name=StateNames.IN_THE_AIR, transitions={}),
            State(name=StateNames.ERROR, transitions={}),
        ]
    )

    run_until(sm, target=StateNames.IN_THE_AIR, error_states={StateNames.ERROR})
