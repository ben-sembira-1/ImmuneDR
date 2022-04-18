import pytest
import os
import shutil
import subprocess
import signal
from src import drone_controller

from . import config


@pytest.fixture(scope="function")
def simulation() -> subprocess.Popen:
    if os.path.exists(config.SIMULATION_DIRECTORY_PATH):
        shutil.rmtree(config.SIMULATION_DIRECTORY_PATH)
    os.makedirs(config.SIMULATION_DIRECTORY_PATH)
    shutil.copy2(config.MAV_PARAM_FILE_PATH, config.SIMULATION_DIRECTORY_PATH)
    sitl_cmd = [
        config.SIM_VEHICLE_PATH,
        "-v",
        "ArduCopter",
        "--add-param-file=mav.parm",
        "--no-rebuild",
        "--no-mavproxy",
    ]
    sitl = subprocess.Popen(
        sitl_cmd,
        cwd=config.SIMULATION_DIRECTORY_PATH,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        stdin=subprocess.PIPE,
    )
    try:
        yield sitl
    finally:
        sitl.send_signal(signal.SIGINT)
        # Wait for the simulation to close itself.
        print(sitl.communicate())


@pytest.fixture(scope="function")
def drone(simulation) -> drone_controller.Drone:
    my_drone = drone_controller.Drone(
        source_system_id=config.MISSION_COMPUTER_MAVLINK_SYSTEM_ID,
        print_logs=True,
        simulation_speedup=config.SPEED_UP,
    )
    return my_drone
