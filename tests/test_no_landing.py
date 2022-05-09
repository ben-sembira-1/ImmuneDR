import enum

from async_state_machine import StateMachine, State
from async_state_machine.transitions.timeout import timeout
from drones.commands import FlightMode

from drones.testing import SimulationDroneClient


def test_drone_does_not_land_on_gps_failure(sim_drone: SimulationDroneClient) -> None:
    @enum.unique
    class StateNames(enum.Enum):
        AWAIT_PREFLIGHT = "Await Preflight"
        AWAIT_GUIDED_MODE = "Await Guided Mode"
        AWAIT_ARM = "Await Arm"
        TAKING_OFF = "Taking off"
        IN_THE_AIR_WITHOUT_GPS = "In the air"
        TURNING_OFF_GPS = "Turning off GPS"

        STAYIED_IN_THE_AIR = "Stayied in the air"
        ERROR = "Error"

    FLIGHT_HEIGHT = 15

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
                    StateNames.TURNING_OFF_GPS: sim_drone.takeoff(
                        height_m=FLIGHT_HEIGHT
                    ),
                    StateNames.ERROR: timeout(secs=25),
                },
            ),
            State(
                name=StateNames.TURNING_OFF_GPS,
                transitions={
                    StateNames.IN_THE_AIR_WITHOUT_GPS: sim_drone.turn_off_gps(),
                    StateNames.ERROR: timeout(secs=10),
                },
            ),
            State(
                name=StateNames.IN_THE_AIR_WITHOUT_GPS,
                transitions={
                    StateNames.STAYIED_IN_THE_AIR: timeout(secs=20),
                    StateNames.ERROR: sim_drone.when_local_position(
                        lambda p: -p.down < 0.9 * FLIGHT_HEIGHT
                    ),
                },
            ),
            State(name=StateNames.STAYIED_IN_THE_AIR, transitions={}),
            State(name=StateNames.ERROR, transitions={}),
        ]
    )

    while sm.current_state.name not in {StateNames.ERROR, StateNames.STAYIED_IN_THE_AIR}:
        sm.tick()

    assert sm.current_state.name == StateNames.STAYIED_IN_THE_AIR
