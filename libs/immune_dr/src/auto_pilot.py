import math
import queue
import time
from typing import Tuple, Optional

from pymavlink import mavlink

from libs.immune_dr.src import hz_loop, geo_misc
from libs.mavlink_drone.src.drone_controller import Drone


class AutoPilot:
    # This should be a high value.
    # If not - the random errors in the drone will have a big impact and will create weird movements that look like curves.
    NO_GPS_DR_PITCH = 1
    NO_GPS_DR_YAW = 0.1
    MESSAGES_TO_REQUEST_INTERVAL = [
        mavlink.MAVLINK_MSG_ID_HOME_POSITION,
        mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        mavlink.MAVLINK_MSG_ID_ATTITUDE,
        mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
    ]

    def __init__(
        self, drone: Drone, print_logs: bool = False, simulation_speedup: float = 1.0
    ):
        self._loop = hz_loop.HzLoop(2 * simulation_speedup)
        self._print_logs = print_logs
        self._start_time = time.time()

        self._drone = drone
        self._request_messages_interval()
        self._drone.update()
        # Auto navigation with GPS
        self._waypoints = queue.Queue
        # Auto navigation no GPS
        self._ekf_failsafe: bool = False
        self._next_rc_commands: Optional[Tuple[float, float, float, float]] = None
        self._home_position: Tuple[float, float] = (0, 0)  # Latitude, Longitude
        self._home_compass_bearing: float = 0
        self._last_position = None
        self._counter = 0

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
        self._loop.add_routine(self._rout_receive_updated_mavlink_messages())
        self._loop.add_routine(self._rout_check_failsafes())
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

    def _rout_update_counter(self):
        while True:
            yield
            self._counter += 1

    def _rout_receive_updated_mavlink_messages(self):
        while True:
            yield
            self._drone.update()

    def _rout_send_rc_commands(self):
        while True:
            yield
            if self._next_rc_commands:
                self._drone.rc(*self._next_rc_commands)
                self._log(f"Sending RC commands: {self._next_rc_commands}")

    def _rout_log_attitude(self):
        while True:
            yield
            attitude = self._drone.attitude
            self._log(
                f"Pitch: {attitude.pitch}, Roll: {attitude.roll}, Yaw: {attitude.yaw}"
            )

    def _rout_update_home_compass_bearing(self):
        while True:
            yield
            if not self._ekf_failsafe:
                self._last_position = self._drone.current_position
            if self._last_position is not None:
                self._home_position = self._drone.home_position
                self._home_compass_bearing = (
                    geo_misc.compass_bearing_between_2_coordinates(
                        (self._home_position.latitude, self._home_position.longitude),
                        (self._last_position.lat, self._last_position.lon),
                    )
                )
                self._log(f"home compass bearing: {self._home_compass_bearing}")
                self._log(f"home position: {self._home_position}")

    def _rout_update_rc_commands(self):
        angle_good_streak = 0
        while True:
            yield
            if self._ekf_failsafe:
                # Todo: This needs to be done somewhere because
                #  if the rc failsafe will take place before the
                #  program will manage to override the RC commands,
                #  the vehicle will be set on LAND mode.
                #  self._drone.set_mode("ALT_HOLD")
                current_yaw_compass_bearing = self._drone.attitude.yaw % (2 * math.pi)
                self._log(f"current compass bearing: {current_yaw_compass_bearing}")
                d_angle = self._home_compass_bearing - current_yaw_compass_bearing
                pitch_rc = 0
                yaw_rc = 0
                self._log(f"Delta angle: {d_angle}")

                if math.fabs(d_angle) > math.pi * 0.02:
                    # If the home bearing is bigger, it means that we need to go clockwise to get to it, else counter-clockwise.
                    # If the delta angle is bigger then PI (more then half circle) it means that going to the other direction will be shorter.
                    yaw_rc = math.copysign(
                        AutoPilot.NO_GPS_DR_YAW,
                        (d_angle) * (math.pi - math.fabs(d_angle)),
                    )

                if math.fabs(d_angle) < math.pi * 0.05:
                    # Soft the yawing if close to the correct angle
                    yaw_rc /= 2
                    angle_good_streak += 1
                else:
                    angle_good_streak = 0
                if angle_good_streak > 15:
                    pitch_rc = -AutoPilot.NO_GPS_DR_PITCH
                self._next_rc_commands = (pitch_rc, 0, yaw_rc, 0)
            else:
                self._next_rc_commands = None

    def _rout_check_failsafes(self):
        while True:
            yield
            self._ekf_failsafe = self._drone.gps_raw.fix_type in {
                mavlink.GPS_FIX_TYPE_NO_FIX,
                mavlink.GPS_FIX_TYPE_NO_GPS,
            }
            self._log(f"gps failsafe -> {self._ekf_failsafe}")
            self._counter += 1

    def _request_messages_interval(self):
        for m in AutoPilot.MESSAGES_TO_REQUEST_INTERVAL:
            self._drone.request_message_interval(m, interval=4e5)
