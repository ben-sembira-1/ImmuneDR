import math
import queue
import time
from typing import Tuple, Optional

from pymavlink import mavlink

from libs.immune_dr.src import hz_loop, geo_misc
from libs.mavlink_drone.src import drone_controller


class AutoPilot:
    # This should be a high value.
    # If not - the random errors in the drone will have a big impact and will create weird movements.
    NO_GPS_DR_PITCH = 1
    NO_GPS_DR_YAW = 0.1
    MAX_GCS_LOSSES = 3

    def __init__(
        self,
        drone: drone_controller.Drone,
        print_logs: bool = False,
        simulation_speedup: float = 1.0,
    ):
        self._loop = hz_loop.HzLoop(2 * simulation_speedup)
        self._print_logs = print_logs
        self._start_time = time.time()

        self._drone = drone
        self._drone.request_all_messages()
        # Auto navigation with GPS
        self._waypoints = queue.Queue
        # Auto navigation no GPS
        self._ekf_failsafe: bool = False
        self._gcs_loss_counter: bool = False
        self._next_rc_commands: Optional[Tuple[float, float, float, float]] = None
        self._home_position: Tuple[float, float] = (0, 0)  # Latitude, Longitude
        self._home_compass_bearing: float = 0
        self._last_position = None
        self._counter: int = 0
        self._dr_active: bool = False

    def __enter__(self):
        self.start_mission()
        return self

    def __exit__(self, type, value, traceback):
        self.stop_mission()

    def add_waypoint(self, latitude, longitude):
        raise NotImplementedError()

    def start_mission(self):
        """
        Adds all the routines to the loop and starts it.
        """
        self._loop.add_routine(self._rout_check_failsafes())
        self._loop.add_routine(self._rout_enable_dr_if_needed())
        self._loop.add_routine(self._rout_change_mode_if_needed())
        self._loop.add_routine(self._rout_update_home_compass_bearing())
        self._loop.add_routine(self._rout_update_rc_commands())
        self._loop.add_routine(self._rout_send_rc_commands())
        self._loop.add_routine(self._rout_log_attitude())
        self._loop.add_routine(self._rout_update_counter())
        self._loop.start()

    def stop_mission(self):
        self._loop.stop()

    def _log(self, message: str):
        if self._print_logs:
            print(
                f"[{self._counter:03d} :: +{time.time() - self._start_time:.3f}] {message}"
            )

    def _rout_check_failsafes(self):
        while True:
            yield
            self._log("Checking failsafes")
            self._ekf_failsafe = self._drone.gps_raw.fix_type in {
                mavlink.GPS_FIX_TYPE_NO_FIX,
                mavlink.GPS_FIX_TYPE_NO_GPS,
            }
            if self._drone.get_gcs_heartbeat() is None:
                self._gcs_loss_counter += 1
            else:
                self._gcs_loss_counter = 0

            self._log(f"EKF failsafe -> {self._ekf_failsafe}")
            self._log(f"GCS loss count -> {self._gcs_loss_counter}")

    def _rout_enable_dr_if_needed(self):
        while True:
            yield
            self._log("Enabling DR if needed")
            if (
                not self._dr_active
                and self._ekf_failsafe
                and self._gcs_loss_counter > AutoPilot.MAX_GCS_LOSSES
            ):
                self._log("Enabling DR...")
                self._drone.sysid_mygcs = self._drone.source_system_id
                self._dr_active = True

    def _rout_change_mode_if_needed(self):
        while True:
            yield
            if self._dr_active and self._drone.mode == "LAND":
                self._drone.mode = "ALT_HOLD"

    def _rout_update_home_compass_bearing(self):
        new_position = False
        while True:
            yield
            self._log("Updating home comapss bearing")
            if not self._ekf_failsafe:
                self._last_position = self._drone.current_position
                new_position = self._last_position is not None
            if new_position:
                self._home_position = self._drone.home_position
                self._home_compass_bearing = (
                    geo_misc.compass_bearing_between_2_coordinates(
                        (self._home_position.latitude, self._home_position.longitude),
                        (self._last_position.lat, self._last_position.lon),
                    )
                )
                self._log(f"home compass bearing: {self._home_compass_bearing}")
                self._log(f"home position: {self._home_position}")
                self._log(f"last position: {self._last_position}")

    def _rout_update_rc_commands(self):
        angle_good_streak = 0
        while True:
            yield
            self._log("Updating RC commands")
            if self._dr_active:
                current_yaw_compass_bearing = self._drone.get_message(
                    drone_controller.Message.ATTITUDE, max_age=1e-1, timeout=5e-1
                ).yaw % (2 * math.pi)
                self._log(f"current compass bearing: {current_yaw_compass_bearing}")
                d_angle = self._home_compass_bearing - current_yaw_compass_bearing
                yaw_rc = 0
                self._log(f"Delta angle: {d_angle}")

                if math.fabs(d_angle) > math.pi * 1e-2:
                    # If the home bearing is bigger, it means that we need to go clockwise to get to it, else counter-clockwise.
                    # If the delta angle is bigger then PI (more then half circle) it means that going to the other direction will be shorter.
                    yaw_rc = math.copysign(
                        AutoPilot.NO_GPS_DR_YAW,
                        (d_angle) * (math.pi - math.fabs(d_angle)),
                    )
                if math.fabs(d_angle) < math.pi * 2e-1:
                    # Soft the yawing if close to the correct angle
                    yaw_rc /= 2
                if math.fabs(d_angle) < math.pi * 2e-2:
                    angle_good_streak += 1
                else:
                    angle_good_streak = 0
                pitch_rc = -AutoPilot.NO_GPS_DR_PITCH if angle_good_streak > 15 else 0
                self._next_rc_commands = (pitch_rc, 0, yaw_rc, 0)
            else:
                self._next_rc_commands = None

    def _rout_send_rc_commands(self):
        while True:
            yield
            if self._next_rc_commands is not None:
                self._drone.rc(*self._next_rc_commands)
                self._log(f"Sending RC commands: {self._next_rc_commands}")

    def _rout_log_attitude(self):
        while True:
            yield
            self._log("Logging attitude")
            attitude = self._drone.attitude
            self._log(
                f"Pitch: {attitude.pitch:.2f}, Roll: {attitude.roll:.2f}, Yaw: {attitude.yaw:.2f}"
            )

    def _rout_update_counter(self):
        while True:
            yield
            self._counter += 1
