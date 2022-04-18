import time
import subprocess
from typing import Iterable

import pytest

from . import config, utils
from drones import drone_controller

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
@pytest.mark.parametrize("parameter_attribute, values", VALID_PARAMETERS_VALUES.items())
def test_parameter_valid_values_get_set(
    connected_drone: drone_controller.Drone,
    parameter_attribute: str,
    values: Iterable,
):
    for parameter_value in values:
        connected_drone.__setattr__(parameter_attribute, parameter_value)
        recieved_value = connected_drone.__getattribute__(parameter_attribute)
        assert recieved_value is not None and (
            int(recieved_value) == int(parameter_value)
        ), f"Failed to set {parameter_attribute} to {parameter_value}"


@pytest.mark.system
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
