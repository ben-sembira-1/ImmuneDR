import math


# -------------------- Simulation --------------------
# For chaning the speedup, change the SIM_SPEEDUP in the system_tests/mav.param as well.
SPEED_UP: int = 5

SIM_VEHICLE_PATH = "/home/pilot/ardupilot/Tools/autotest/sim_vehicle.py"
MISSION_PLANNER_PORT = 14580  # For future use
TESTS_PORT = 5760
SITL_PORT = 5760
TESTS_ADDRESS = f"tcp:localhost:{TESTS_PORT}"
SITL_ADDRESS = f"tcp:localhost:{SITL_PORT}"
SIMULATION_DIRECTORY_PATH = "system_tests/simulation_directory"
MAV_PARAM_FILE_PATH = "system_tests/mav.parm"
MISSION_COMPUTER_MAVLINK_SYSTEM_ID = 2

# -------------------- Distances --------------------
FLIGHT_ALTITUDE_METERS = 50
# This is aproximetly 1 KM for each axis.
DR_AXIS_DISTANCE_DEGREES = math.radians(1e-2)
RETURN_HOME_MIN_PROPEGRATION_METERS_PER_SECOND = 5
# For each meter of distance from the home position, we allow 15 cm of error.
DR_PERCISION_METERS_PER_DISTANCE = 0.15

# -------------------- Timeouts --------------------
INITIALIZATION_TIMEOUT_SEC = 590
DRONE_CONNECT_TO_SIMULATION_TIMEOUT_SEC = 10
MAX_SET_HOME_VALIDATION_TIMEOUT_SECONDS = 3
MAX_DR_YAW_INITIALIZATION_TIMEOUT_SECONDS = 30
DR_TIMEOUT_SECONDS_PER_METER = 0.15
GENERIC_FAILSAFE_TRIGGER_TIMEOUT_SECONDS = 10
AIR_STOP_TIME_SECONDS = 10

FLOAT_ERROR = 1e-7
