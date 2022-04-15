import pytest
import os
import shutil
import subprocess
import signal
from libs.mavlink_drone.src import drone_controller

from system_tests import config


@pytest.fixture
def simulation() -> subprocess.Popen:
    if os.path.exists(config.SIMULATION_DIRECTORY_PATH):
        shutil.rmtree(config.SIMULATION_DIRECTORY_PATH)
    os.makedirs(config.SIMULATION_DIRECTORY_PATH)
    shutil.copy2(config.MAV_PARAM_FILE_PATH, config.SIMULATION_DIRECTORY_PATH)
    mavproxy_args = [
        f'--cmd="set heartbeat {2 * config.SPEED_UP}"',
        f"--out={config.TESTS_ADDRESS}",
        # f"--out=udp:localhost:{config.MISSION_PLANNER_PORT}",
    ]
    sitl_cmd = [
        "python3",
        config.SIM_VEHICLE_PATH,
        "-v",
        "ArduCopter",
        f"--add-param-file=mav.parm",
        f"--mavproxy-args={' '.join(mavproxy_args)}",
        "--map",
        "--console",
    ]
    sitl = subprocess.Popen(
        sitl_cmd,
        cwd=config.SIMULATION_DIRECTORY_PATH,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        stdin=subprocess.PIPE,
    )
    yield sitl
    sitl.send_signal(signal.SIGINT)

@pytest.fixture
def drone() -> drone_controller.Drone:
    my_drone = drone_controller.Drone(
        source_system_id=config.MISSION_COMPUTER_MAVLINK_SYSTEM_ID
    )
    return my_drone
