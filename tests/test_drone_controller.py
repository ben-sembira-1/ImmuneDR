import logging
import random
from typing import Generator
import pytest

from drones.drone_client import DroneClient
from drones.testing import TcpSerialConnectionDef, simulation_context


@pytest.fixture(scope="function")
def sim_drone(tmpdir) -> Generator[DroneClient, None, None]:
    port = random.randrange(5900, 6100)
    logging.info(f"Using random tcp port {port} for simulation connection")
    external_mavproxy_connection = TcpSerialConnectionDef(
        port=5763, wait_for_connection=False
    )
    with simulation_context(
        cwd=tmpdir,
        serial={
            1: TcpSerialConnectionDef(port=port, wait_for_connection=True),
            2: external_mavproxy_connection,
        },
    ) as sim:
        mavlink = sim.mavlink_connect_to_serial(1)
        mavlink.wait_heartbeat()
        logging.debug("Connected successfully to simulation, received hearteat.")
        yield DroneClient(mavlink)


def test_drone_takeoff(sim_drone: DroneClient):
    sim_drone.take_off(10)
