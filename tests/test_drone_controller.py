import enum

from async_state_machine import StateMachine, State
from async_state_machine.transitions.timeout import timeout

from drones.drone_client import DroneClient
from drones.drone_daemon import disable_gps

from tests.state_machine_utils import run_until
from tests.takeoff_state_machine import get_takeoff_state_machine, TakeoffStateNames


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
    sm = get_takeoff_state_machine(sim_drone)
    run_until(
        sm, target=TakeoffStateNames.IN_THE_AIR, error_states={TakeoffStateNames.ERROR}
    )


def test_disable_gps(flying_sim_drone: DroneClient, mavlink_connection):
    @enum.unique
    class StateNames(enum.Enum):
        FLYING = "Flying"
        NO_GPS = "Lost GPS"
        ERROR = "Error"

    sm = StateMachine(
        [
            State(
                name=StateNames.FLYING,
                transitions={
                    StateNames.NO_GPS: flying_sim_drone.ekf_bad(),
                    StateNames.ERROR: timeout(secs=5),
                },
            ),
            State(name=StateNames.NO_GPS, transitions={}),
            State(name=StateNames.ERROR, transitions={}),
        ]
    )

    disable_gps(mavlink_connection)
    run_until(sm, target=StateNames.NO_GPS, error_states={StateNames.ERROR})
