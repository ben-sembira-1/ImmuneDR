import subprocess

import pytest

from system_tests import config
from libs.mavlink_drone.src import drone_controller

@pytest.fixture(scope="class")
def connected_drone(simulation: subprocess.Popen, drone: drone_controller.Drone) -> drone_controller.Drone:
    drone.connect(
        address=config.TESTS_ADDRESS,
        connection_timeout_sec=config.DRONE_CONNECT_TO_SIMULATION_TIMEOUT_SEC
    )
    return drone

class TestParametersSetGet:
    @pytest.mark.system
    def test_sysid_mygcs(self, connected_drone: drone_controller.Drone):
        parameter_name = "SYSID_MYGCS"
        for parameter_value in [0, 1, 10, 255]:
            connected_drone.sysid_mygcs = parameter_value
            assert connected_drone.sysid_mygcs == parameter_value, f"Failed to set {parameter_name} to {parameter_value}"
        for parameter_value in [-1, 256, 48576]:
            with pytest.raises(AssertionError):
                connected_drone.sysid_mygcs = parameter_value, f"Setting an invalid value did not raise exception."
        
    def test_sysid_mygcs(self, connected_drone: drone_controller.Drone):
        parameter_name = 
        for parameter_value in :
            connected_drone. = parameter_value
            assert connected_drone. == parameter_value, f"Failed to set {parameter_name} to {parameter_value}"
        for parameter_value in :
            with pytest.raises(AssertionError):
                connected_drone. = parameter_value, f"Setting an invalid value to {parameter_name} did not raise exception."
    