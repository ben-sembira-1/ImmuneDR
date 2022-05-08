import random
import logging
import os
from typing import Generator
from pathlib import Path

import pytest

from drones.testing import TcpSerialConnectionDef, simulation_context, SimulationDroneClient, SimulationDroneDaemon


@pytest.fixture(scope="function")
def sim_drone(tmpdir: str) -> Generator[SimulationDroneClient, None, None]:
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
        daemon = SimulationDroneDaemon(mavlink_connection=mavlink)
        client = daemon.create_client()
        try:
            yield client
        finally:
            del daemon