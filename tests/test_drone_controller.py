import enum
from datetime import timedelta

from async_state_machine import StateMachine, State
from async_state_machine.transitions.timeout import timeout

from drones.drone_client import DroneClient
from drones.mavlink_types import FlightMode
from drones.testing import SimulationDroneClient

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
    altitude = 15
    sm = get_takeoff_state_machine(sim_drone, altitude)
    run_until(
        sm, target=TakeoffStateNames.IN_THE_AIR, error_states={TakeoffStateNames.ERROR}
    )
    # TODO assert abs(sim_drone.sim_state.altitude_m - altitude) < 1


def test_land(flying_sim_drone: DroneClient) -> None:
    # TODO start at different heights
    # TODO check that drone is disarmed
    # TODO test when GPS is inactive
    @enum.unique
    class StateNames(enum.Enum):
        LANDING = "Landing"
        DONE = "Landed"
        ERROR = "Error"

    sm = StateMachine(
        [
            State(
                name=StateNames.LANDING,
                transitions={
                    StateNames.DONE: flying_sim_drone.land(),
                    StateNames.ERROR: timeout(secs=10),
                },
            ),
            State(name=StateNames.DONE, transitions={}),
            State(name=StateNames.ERROR, transitions={}),
        ]
    )
    run_until(sm, target=StateNames.DONE, error_states={StateNames.ERROR})


def test_disable_gps(flying_sim_drone: SimulationDroneClient) -> None:
    @enum.unique
    class StateNames(enum.Enum):
        FLYING = "Flying"
        TURNING_OFF_GPS = "Disabling GPS"
        NO_GPS = "Lost GPS"
        ERROR = "Error"

    sm = StateMachine(
        [
            State(
                name=StateNames.FLYING,
                transitions={
                    StateNames.TURNING_OFF_GPS: flying_sim_drone.turn_off_gps(),
                    StateNames.ERROR: timeout(secs=5),
                },
            ),
            State(
                name=StateNames.TURNING_OFF_GPS,
                transitions={
                    StateNames.NO_GPS: flying_sim_drone.ekf_bad(),
                    StateNames.ERROR: timeout(secs=5),
                },
            ),
            State(name=StateNames.NO_GPS, transitions={}),
            State(name=StateNames.ERROR, transitions={}),
        ]
    )

    run_until(sm, target=StateNames.NO_GPS, error_states={StateNames.ERROR})


def test_turn(flying_sim_drone: DroneClient) -> None:
    # TODO parametrize with different start and target headings
    # TODO test illegal heading (out of range)
    # TODO test when GPS is inactive
    @enum.unique
    class StateNames(enum.Enum):
        TURNING = "Turning"
        DONE = "Done turning"

    sm = StateMachine(
        [
            State(
                name=StateNames.TURNING,
                transitions={
                    StateNames.DONE: flying_sim_drone.turn(heading=0),
                },
            ),
            State(name=StateNames.DONE, transitions={}),
        ]
    )

    run_until(sm, target=StateNames.DONE, timeout=timedelta(seconds=3))


def test_set_attitude(flying_sim_drone: DroneClient) -> None:
    # TODO does this even work in GUIDED with GPS?
    # TODO parametrize
    @enum.unique
    class StateNames(enum.Enum):
        CHANGING_MODE = "Changing mode to GUIDED_NOGPS"
        TURNING = "Turning"
        DONE = "Done turning"

    sm = StateMachine(
        [
            State(
                name=StateNames.CHANGING_MODE,
                transitions={
                    StateNames.TURNING: flying_sim_drone.set_flight_mode(
                        FlightMode.GUIDED_NO_GPS,
                    ),
                },
            ),
            State(
                name=StateNames.TURNING,
                transitions={
                    StateNames.DONE: flying_sim_drone.set_attitude(
                        heading_deg=0, pitch_deg=-5, roll_deg=0
                    ),
                },
            ),
            State(name=StateNames.DONE, transitions={}),
        ]
    )

    run_until(sm, target=StateNames.DONE, timeout=timedelta(seconds=3))
