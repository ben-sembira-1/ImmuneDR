import math
from typing import Optional, NamedTuple
import time
import pytest
import subprocess
from pymavlink import mavlink

from system_tests import config, utils

from libs.mavlink_drone.src import drone_controller
from libs.immune_dr.src import auto_pilot
from libs.immune_dr.src import geo_misc


class Position(NamedTuple):
    latitude: float
    longitude: float
    altitude: float


class TestDr:

    current_simulation: Optional[subprocess.Popen] = None

    # ----------------------------- Common functions -----------------------------

    def distance_from_home(
        self, drone_obj: drone_controller.Drone, home_position: Position
    ) -> float:
        simulation_state = drone_obj.sim_state
        simulation_lat = math.radians(simulation_state.lat / 1e7)
        simulation_lon = math.radians(simulation_state.lon / 1e7)
        distance = geo_misc.distance_between_2_coordinates(
            (simulation_lat, simulation_lon),
            (home_position.latitude, home_position.longitude),
        )
        return distance

    def set_home(
        self,
        drone_obj: drone_controller.Drone,
        latitude: float,
        longitude: float,
        altitude: Optional[int] = None,
        absolute: bool = False,
        validation_timeout: Optional[
            float
        ] = config.MAX_SET_HOME_VALIDATION_TIMEOUT_SECONDS,
    ) -> Position:
        """
        This funcion sets the drones home to be the given latitude-longitude-altitude.
        In default: the latitude and longitude are offsets that will be aplied to the drone current location,
        if you want to set absolute coordinates, set absolute=True.
        The altitude in default will be the same as the previous home position,
        in case it is given, it will be the height above the sea.
        If validation_timeout is given, the function will vaidate that the home_point was changed,
        if it is None, it will not.

        The function returns the new home location.
        """
        if altitude is not None:
            new_altitude = altitude
        else:
            new_altitude = drone_obj.get_home_position(blocking=True).altitude

        if absolute:
            new_home_position = Position(
                latitude=latitude, longitude=longitude, altitude=new_altitude
            )
        else:
            current_location = drone_obj.current_position
            new_home_position = Position(
                latitude=current_location.lat + latitude,
                longitude=current_location.lon + longitude,
                altitude=new_altitude,
            )

        drone_obj.home_position = new_home_position
        if validation_timeout is not None:
            start_time = time.time()
            current_home_position = drone_obj.get_home_position(blocking=True)
            while (
                math.fabs(current_home_position.latitude - new_home_position.latitude)
                > config.FLOAT_ERROR
                and math.fabs(
                    current_home_position.longitude - new_home_position.longitude
                )
                > config.FLOAT_ERROR
            ):
                assert (
                    utils.time_since(start_time) < validation_timeout
                ), f"set_home validation failed, current: {current_home_position}, updated: {new_home_position}"
                utils.wait(1)
                current_home_position = drone_obj.get_home_position(blocking=True)
        return new_home_position

    def got_closer(
        self,
        previous_distance: float,
        new_distance: float,
        min_propegration: float = config.RETURN_HOME_MIN_PROPEGRATION_METERS_PER_SECOND,
    ):
        return new_distance < previous_distance - min_propegration

    def wait_until_drone_gets_home(
        self,
        drone_obj: drone_controller.Drone,
        home_position: Position,
        percision_meters: Optional[float] = None,
        timeout: Optional[float] = None,
        absolute: bool = False,
    ) -> None:
        """
        Waits until the drone is `percision_meters` away from home, then returns True.
        If at any part of the way it does not get closer to the home position (i.e. the
        distance from home got bigger), the function will return False.
        For waiting forever, set timeout=-1

        If percision or timeout are None, they will be set relatively to the distance
        from the home position using: config.DR_PERCISION_METERS_PER_METER and
        config.DR_TIMEOUT_SECONDS_PER_METER
        """
        start_time = time.time()
        previous_distance = math.inf
        current_distance = self.distance_from_home(drone_obj, home_position)

        if percision_meters is None:
            percision_meters = (
                current_distance * config.DR_PERCISION_METERS_PER_DISTANCE
            )
        if timeout is None:
            timeout = current_distance * config.DR_TIMEOUT_SECONDS_PER_METER

        not_propegating_counter = 0

        while current_distance > percision_meters:
            if (
                timeout >= 0
                and utils.time_since(start_time, absolute=absolute) > timeout
            ):
                assert False, "Get home timeout"
            # Check that the drone is actually getting closer to the home point
            if not self.got_closer(
                previous_distance=previous_distance, new_distance=current_distance
            ):
                not_propegating_counter += 1
                if not_propegating_counter > 3:
                    assert False, "The drone failed to propegate to the home direction"
            previous_distance = current_distance
            utils.wait(1, absolute=absolute)
            current_distance = self.distance_from_home(drone_obj, home_position)

    def wait_for_mode_change(
        self,
        drone_obj: drone_controller.Drone,
        mode: str,
        timeout: Optional[float] = config.GENERIC_FAILSAFE_TRIGGER_TIMEOUT_SECONDS,
        absolute: bool = False,
    ) -> None:
        """For waiting forever, set timeout=None"""
        start_time = time.time()
        while (
            timeout is None or utils.time_since(start_time, absolute=absolute) < timeout
        ) and drone_obj.mode != mode:
            utils.wait(1, absolute=absolute)
        assert drone_obj.mode == mode, f"Mode did not change to {mode}"

    def wait_for_dr_start(
        self,
        drone_obj: drone_controller.Drone,
        home_position: Position,
        timeout: Optional[float] = config.MAX_DR_YAW_INITIALIZATION_TIMEOUT_SECONDS,
        absolute: bool = False,
    ) -> None:
        """For waiting forever, set timeout=None"""
        start_time = time.time()
        current_distance = self.distance_from_home(drone_obj, home_position)
        previous_distance = current_distance
        while (
            timeout is None or utils.time_since(start_time, absolute=absolute) < timeout
        ):
            if self.got_closer(
                previous_distance=previous_distance, new_distance=current_distance
            ):
                return
            previous_distance = current_distance
            utils.wait(1, absolute=absolute)
            current_distance = self.distance_from_home(drone_obj, home_position)
        assert False, "Timeout over, the drone did not start DR"

    # ----------------------------- Fixtures -----------------------------

    @pytest.fixture
    def flying_drone(self, simulation: subprocess.Popen, drone: drone_controller.Drone) -> drone_controller.Drone:
        TestDr.current_simulation = simulation
        start_time = time.time()
        def time_valid() -> bool:
            return utils.time_since(start_time) < config.INITIALIZATION_TIMEOUT_SEC
        
        drone.connect(
            address=config.TESTS_ADDRESS,
            connection_timeout_sec=config.DRONE_CONNECT_TO_SIMULATION_TIMEOUT_SEC,
        )
        last_gps_raw = drone.gps_raw
        while time_valid() and (
            last_gps_raw is None
            or (
                last_gps_raw.fix_type
                in {mavlink.GPS_FIX_TYPE_NO_FIX, mavlink.GPS_FIX_TYPE_NO_GPS}
            )
        ):
            print(time_valid())
            utils.wait(1)
            last_gps_raw = drone.gps_raw
        drone.mode = "GUIDED"
        while time_valid() and (drone.local_position is None):
            utils.wait(1)
        drone.arm()
        drone.takeoff(config.FLIGHT_ALTITUDE_METERS)
        # local_position.z returns negetive height (the z axis are flipped).
        while time_valid() and (
            -1 * drone.local_position.z < config.FLIGHT_ALTITUDE_METERS - 2
        ):
            utils.wait(1)
        assert time_valid(), "Initialization timeout"
        # There is a wreird bug that from the second run, the messages requests in the drone constructor does not arrive.
        # Asking again after some time, solves it.
        drone.request_all_messages()
        return drone

    # ----------------------------- Tests -----------------------------

    @pytest.mark.system
    # @pytest.mark.skip(reason="Tested")
    def test_simulation_not_crashing_on_startup(self, simulation: subprocess.Popen):
        utils.wait(10, absolute=True)
        assert simulation.poll() is None, "Simulation crashed"

    @pytest.mark.system
    # @pytest.mark.skip(reason="Tested")
    def test_drone_takeoff(self, flying_drone: drone_controller.Drone):
        assert flying_drone.armed, "The drone failed to takeoff, it is not armed"
        assert (
            config.FLIGHT_ALTITUDE_METERS - 2
            <= -1 * flying_drone.local_position.z
            <= config.FLIGHT_ALTITUDE_METERS + 2
        ), "The drone failed to takeoff, its heigt is wrong."

    @pytest.mark.system
    # @pytest.mark.skip(reason="WIP")
    # @pytest.mark.parametrize(
    #     "offset_latitude",
    #     [config.DR_AXIS_DISTANCE_DEGREES, -config.DR_AXIS_DISTANCE_DEGREES, 0],
    # )
    # @pytest.mark.parametrize(
    #     "offset_longitude",
    #     [config.DR_AXIS_DISTANCE_DEGREES, -config.DR_AXIS_DISTANCE_DEGREES, 0],
    # )
    # Use this and comment the 2 above markers for a single test:
    @pytest.mark.parametrize(
        "offset_latitude, offset_longitude",
        [(config.DR_AXIS_DISTANCE_DEGREES, config.DR_AXIS_DISTANCE_DEGREES)],
    )
    def test_dr_direction(
        self,
        flying_drone: drone_controller.Drone,
        offset_latitude: int,
        offset_longitude: int,
    ):
        """
        Test DR for when the home is configured to be in the
        given offsets from the initial drone position.
        """
        if offset_latitude == 0 and offset_longitude == 0:
            # This means that the drone is already at home.
            # Todo: should it land in that case?
            return
        new_home_position = self.set_home(
            flying_drone,
            offset_latitude,
            offset_longitude,
        )
        with auto_pilot.AutoPilot(
            flying_drone, print_logs=True, simulation_speedup=config.SPEED_UP
        ):
            assert (
                flying_drone.mode == "GUIDED"
            ), "Something wrong happend, the drone is not in GUIDED mode"
            # Let the autopilot get the drone position before we disable it.
            utils.wait(60, absolute=True)
            flying_drone.sim_gps_disable = True
            self.wait_for_mode_change(
                drone_obj=flying_drone,
                mode="ALT_HOLD",
            )
            # Let the drone yaw itself to the correct angle.
            self.wait_for_dr_start(
                drone_obj=flying_drone,
                home_position=new_home_position,
            )
            self.wait_until_drone_gets_home(
                drone_obj=flying_drone,
                home_position=new_home_position,
            )

    @pytest.mark.system
    # @pytest.mark.skip(reason="WIP")
    def test_dr_trigger_rc_failure_then_gps_failure(
        self, flying_drone: drone_controller.Drone
    ):
        """Should trigger RTL, then DR"""
        new_home_position = self.set_home(
            drone_obj=flying_drone,
            latitude=config.DR_AXIS_DISTANCE_DEGREES,
            longitude=config.DR_AXIS_DISTANCE_DEGREES,
        )
        with auto_pilot.AutoPilot(flying_drone, simulation_speedup=config.SPEED_UP):
            assert (
                flying_drone.mode == "GUIDED"
            ), "Something wrong happend, the drone is not in GUIDED mode"
            # Let the autopilot get the drone position before we disable it.
            utils.wait(2)
            # Set the main GCS ID to be something that does not exist
            # so the ArduPilot will not get heartbeats and RC overrides
            # (see chapter 8.2. GCS Failsafe Trigger Code Research in DisableAutoLand.md).
            flying_drone.sysid_mygcs = 17
            self.wait_for_mode_change(
                drone_obj=flying_drone,
                mode="RTL",
            )
            flying_drone.sim_gps_disable = True
            self.wait_for_mode_change(
                drone_obj=flying_drone,
                mode="ALT_HOLD",
            )
            utils.wait(config.RTL_TO_ALTHOLD_STOP_TIME_SECONDS)
            self.wait_for_dr_start(
                drone_obj=flying_drone,
                home_position=new_home_position,
            )
            self.wait_until_drone_gets_home(
                drone_obj=flying_drone,
                home_position=new_home_position,
            )

    @pytest.mark.system
    @pytest.mark.skip(reason="WIP")
    def test_dr_trigger_gps_failure_then_rc_failure(self):
        """Should trigger DR"""
        assert False

    @pytest.mark.system
    @pytest.mark.skip(reason="WIP")
    def test_dr_trigger_rc_failure_gps_healthy(self):
        """Should trigger RTL"""
        assert False

    @pytest.mark.system
    @pytest.mark.skip(reason="WIP")
    def test_dr_trigger_rc_healthy_gps_failure(self):
        """Should trigger DR"""
        assert False

    @pytest.mark.system
    @pytest.mark.skip(reason="WIP")
    def test_dr_trigger_gps_failure_and_rc_failure_imidiatly_after(self):
        """Should change to mode LAND and then cancel it and trigger DR"""
        assert False

    @pytest.mark.system
    @pytest.mark.skip(reason="WIP")
    def test_dr_trigger_rc_failure_and_gps_failure_imidiatly_after(self):
        """Should change mode to LAND and then cancel it and trigger DR"""
        assert False

    @pytest.mark.system
    @pytest.mark.skip(reason="WIP")
    def test_dr_gps_returns_no_rc():
        assert False

    @pytest.mark.system
    @pytest.mark.skip(reason="WIP")
    def test_dr_gps_returns_rc_healthy():
        assert False

    @pytest.mark.system
    @pytest.mark.skip(reason="WIP")
    def test_dr_rc_returns_no_gps():
        assert False
