import enum
import logging
import os
from pathlib import Path
import random
from sre_constants import SUCCESS
from typing import Generator
import pytest

from async_state_machine import StateMachine, State
from async_state_machine.transitions.timeout import timeout

from drones.drone_client import DroneClient
from drones.drone_daemon import DroneDaemon
from drones.testing import TcpSerialConnectionDef, simulation_context


@pytest.fixture(scope="function")
def sim_drone(tmpdir: str) -> Generator[DroneClient, None, None]:
    port = random.randrange(5900, 6100)
    logging.info(f"Using random tcp port {port} for simulation connection")
    serial_ports_override = {}

    external_mavproxy_port = os.environ.get("MAVPROXY_PORT")
    if external_mavproxy_port is not None and len(external_mavproxy_port) > 0:
        external_mavproxy_connection = TcpSerialConnectionDef(
            port=int(external_mavproxy_port), wait_for_connection=True
        )
        serial_ports_override[1] = external_mavproxy_connection

    serial_ports_override[0] = TcpSerialConnectionDef(
        port=port, wait_for_connection=True
    )
    with simulation_context(
        cwd=Path(tmpdir),
        serial_ports_override=serial_ports_override,
    ) as sim:
        mavlink = sim.mavlink_connect_to_serial(0)
        mavlink.wait_heartbeat(timeout=500)
        logging.debug("Connected successfully to simulation, received hearteat.")
        daemon = DroneDaemon(mavlink_connection=mavlink)
        client = daemon.create_client()
        try:
            yield client
        finally:
            del daemon


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

    while sm.current_state.name not in {StateNames.ERROR, StateNames.SUCCESS}:
        sm.tick()

    assert sm.current_state.name == StateNames.SUCCESS


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
                    StateNames.ERROR: timeout(secs=25),
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

    while sm.current_state.name not in {StateNames.ERROR, StateNames.ARMED}:
        sm.tick()

    assert sm.current_state.name == StateNames.ARMED


def test_drone_takeoff(sim_drone: DroneClient) -> None:
    @enum.unique
    class StateNames(enum.Enum):
        AWAIT_PREFLIGHT = "Await Preflight"
        AWAIT_ARM = "Await Arm"
        TAKING_OFF = "Taking off"

        IN_THE_AIR = "In the air"
        ERROR = "Error"

    sm = StateMachine(
        [
            State(
                name=StateNames.AWAIT_PREFLIGHT,
                transitions={
                    StateNames.AWAIT_ARM: sim_drone.preflight_finished(),
                    StateNames.ERROR: timeout(secs=25),
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

    while sm.current_state.name not in {StateNames.ERROR, StateNames.IN_THE_AIR}:
        sm.tick()

    assert sm.current_state.name == StateNames.IN_THE_AIR
