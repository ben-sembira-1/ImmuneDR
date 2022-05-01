from pathlib import Path
import pytest
from drones.testing import TcpSerialConnectionDef, simulation_context


@pytest.mark.repeat(10)
def test_simulation_context_can_control_the_connection_port(tmpdir: str) -> None:
    port = 55656
    serial_port = 1
    dummy_connection = TcpSerialConnectionDef(port=5762, wait_for_connection=False)
    with simulation_context(
        cwd=Path(tmpdir),
        serial_ports_override={
            0: TcpSerialConnectionDef(port=5980, wait_for_connection=False),
            serial_port: TcpSerialConnectionDef(port, wait_for_connection=True),
            3: dummy_connection,
        }
    ) as sim:
        d = sim.mavlink_connect_to_serial(serial_port)
        d.wait_heartbeat()
        d.wait_heartbeat()
        d.close()
