import pytest
from drones.testing import TcpSerialConnectionDef, simulation_context

@pytest.mark.repeat(10)
def test_simulation_context_can_control_the_connection_port():
    port = 55656
    with simulation_context(serial={0: TcpSerialConnectionDef(port, wait_for_connection=True)}) as sim:
        d = sim.mavlink_connect_to_serial(0)
        d.wait_heartbeat()
        d.close()