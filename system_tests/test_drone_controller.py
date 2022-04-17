import time
import subprocess
from typing import Iterable

import pytest

from system_tests import config, utils
from libs.mavlink_drone.src import drone_controller

BINARY_VALID_PARAMETRIZE = (0, 1)
VALID_PARAMETERS_VALUES = {
    "sysid_mygcs": range(0, 256),
    "fs_ekf_action": (1, 2, 3),
    "sim_gps_disable": (True, False),
    "sim_rc_fail": (True, False),
}
INVALID_PARAMETERS_VALUES = {
    "sysid_mygcs": (-1, 256, 2000),
    "fs_ekf_action": (-1, 0, 4, 10),
}


@pytest.fixture()
def connected_drone(
    simulation: subprocess.Popen, drone: drone_controller.Drone
) -> drone_controller.Drone:
    drone.connect(
        address=config.TESTS_ADDRESS,
        connection_timeout_sec=config.DRONE_CONNECT_TO_SIMULATION_TIMEOUT_SEC,
    )
    drone.run()
    yield drone
    drone.stop()


@pytest.mark.system
# @pytest.mark.skip("Tested")
@pytest.mark.parametrize("parameter_attribute, values", VALID_PARAMETERS_VALUES.items())
# @pytest.mark.parametrize("repeat", range(10))
def test_parameter_valid_values_get_set(
    connected_drone: drone_controller.Drone,
    parameter_attribute: str,
    values: Iterable,
    # repeat: int,
):
    for parameter_value in values:
        connected_drone.__setattr__(parameter_attribute, parameter_value)
        recieved_value = connected_drone.__getattribute__(parameter_attribute)
        assert recieved_value is not None and (
            int(recieved_value) == int(parameter_value)
        ), f"Failed to set {parameter_attribute} to {parameter_value}"


@pytest.mark.system
# @pytest.mark.skip("Tested")
@pytest.mark.parametrize(
    "parameter_attribute, values", INVALID_PARAMETERS_VALUES.items()
)
def test_parameter_invalid_values_get_set(
    connected_drone: drone_controller.Drone, parameter_attribute: str, values: Iterable
):
    for parameter_value in values:
        raised = False
        try:
            connected_drone.__setattr__(parameter_attribute, parameter_value)
        except drone_controller.DroneInvalidParameterValueError:
            raised = True
        assert (
            raised
        ), f"Did not raise when trying to set {parameter_attribute} to be {parameter_value}"


# @pytest.mark.system
# def test_get_message_


@pytest.mark.system
@pytest.mark.skip("sandbox")
def test_test(connected_drone: drone_controller.Drone):
    print(">>>>>>>>>>>> Requesting")
    time_started = time.time()
    connected_drone.sysid_mygcs = 4
    connected_drone._receive(
        drone_controller.Parameter.SYSID_MYGCS, timeout=2, request=False
    )
    print(f"Time passed: {utils.time_since(time_started)}")
    connected_drone._request_parameter(drone_controller.Parameter.SYSID_MYGCS)
    connected_drone._receive_parameter(
        drone_controller.Parameter.SYSID_MYGCS, timeout=2, request=False
    )
    print(f"Time passed: {utils.time_since(time_started)}")
    print(
        f"Time since last sysid_mygcs: {connected_drone._drone.time_since('SYSID_MYGCS')}"
    )
    assert False
