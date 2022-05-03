import logging
import os
import random
from pathlib import Path
from typing import Generator

import pymavlink.mavutil
import pytest

from tests.state_machine_utils import run_until
from tests.takeoff_state_machine import get_takeoff_state_machine, TakeoffStateNames
from drones.drone_client import DroneClient
from drones.drone_daemon import DroneDaemon
from drones.testing import TcpSerialConnectionDef, simulation_context

@pytest.fixture
def mavlink_connection(tmpdir: str) -> pymavlink.mavutil.mavfile:
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
        logging.debug("Connected successfully to simulation, received heartbeat.")
        yield mavlink


@pytest.fixture(scope="function")
def sim_drone(
    mavlink_connection: pymavlink.mavutil.mavfile,
) -> Generator[DroneClient, None, None]:
    daemon = DroneDaemon(mavlink_connection=mavlink_connection)
    client = daemon.create_client()
    try:
        yield client
    finally:
        del daemon


@pytest.fixture
def flying_sim_drone(sim_drone: DroneClient) -> DroneClient:
    """
    Returns a DroneClient after takeoff
    """
    sm = get_takeoff_state_machine(sim_drone)
    run_until(
        sm, target=TakeoffStateNames.IN_THE_AIR, error_states={TakeoffStateNames.ERROR}
    )
    return sim_drone
