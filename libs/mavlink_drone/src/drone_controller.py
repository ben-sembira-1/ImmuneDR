import enum
import math
import time
from typing import NamedTuple, Optional, Tuple, Union

from pymavlink import mavutil, mavlink


ParameterValue = mavlink.MAVLink_param_value_message


class DroneError(Exception):
    pass


class DroneTakeoffInvalidHeightError(DroneError):
    pass


class DroneArmingMotorsFailed(DroneError):
    pass


class DroneConditionYawWithoutGuidedModeError(DroneError):
    pass


class FailedToConnectDroneError(DroneError):
    pass


class Message(int, enum.Enum):
    HOME_POSITION = mavlink.MAVLINK_MSG_ID_HOME_POSITION
    GLOBAL_POSITION_INT = mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT
    GPS_RAW_INT = mavlink.MAVLINK_MSG_ID_GPS_RAW_INT
    ATTITUDE = mavlink.MAVLINK_MSG_ID_ATTITUDE
    LOCAL_POSITION_NED = mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED
    # We may need SIMSTATE instead of SIM_STATE
    SIM_STATE = mavlink.MAVLINK_MSG_ID_SIM_STATE
    HEARTBEAT = mavlink.MAVLINK_MSG_ID_HEARTBEAT


class Parameter(enum.Enum):
    SYSID_MYGCS = enum.auto()
    FS_EKF_ACTION = enum.auto()
    SIM_GPS_DISABLE = enum.auto()
    SIM_RC_FAIL = enum.auto()


PARAMETER_TYPE_MAP = {
    Parameter.SYSID_MYGCS: mavlink.MAV_PARAM_TYPE_UINT8,
    Parameter.FS_EKF_ACTION: mavlink.MAV_PARAM_TYPE_UINT8,
    Parameter.SIM_GPS_DISABLE: mavlink.MAV_PARAM_TYPE_UINT8,
    Parameter.SIM_RC_FAIL: mavlink.MAV_PARAM_TYPE_UINT8,
}


class Drone:
    RECEIVE_TIMEOUT_SEC = 2
    def __init__(
        self, source_system: int, source_component: int = mavlink.MAV_COMP_ID_AUTOPILOT1
    ):
        assert 0 < source_system < 255
        assert 0 < source_component < 255
        self.source_system_id = source_system
        self.source_component_id = source_component
        self._boot_time_sec: float = time.time()
        self._drone: Optional[mavutil.mavfile] = None

    def _log(self, message: str) -> None:
        print(f"[+{time.time() - self._boot_time_sec%.3}] {message}")

    def connect(
        self, address: str, connection_timeout_sec: Optional[float] = None
    ) -> None:
        """The real connection time can be different then connectin_timeout_sec but it will be close."""
        self._log("Connecting...")
        self._drone = mavutil.mavlink_connection(
            address,
            source_system=self.source_system_id,
            source_component=self.source_component_id,
        )
        # The function wait_heartbeat represent "no timeout" as timeout=None.
        if self._drone.wait_heartbeat(timeout=connection_timeout_sec) is None:
            raise FailedToConnectDroneError("End of timeout")
        self._log("Connected!")

        time.sleep(2)
        # Make sure that all the messages arrive successfully
        self.request_all_messages()

    def _request_message_interval(self, message_id: int, interval=0) -> None:
        self._command_long_send(
            command_id=mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            param1=message_id,
            param2=interval,
        )

    def request_all_messages(self) -> None:
        for message in Message:
            self._log(f"Requesting {repr(message)}...")
            self._request_message_interval(message_id=message, interval=4e5)

    def update(self) -> None:
        # Because the condition is False this will receive all the new messages (It will not stop).
        # self._drone.recv_match(condition="False")
        for message_id in Message:
            while (
                self._drone.recv_match(
                    type=mavlink.mavlink_map[message_id].name, blocking=False
                )
                is not None
            ):
                pass

    # ------------------- Parameters -------------------

    @property
    def sysid_mygcs(self) -> ParameterValue:
        return self._get_parameter(Parameter.SYSID_MYGCS)

    @sysid_mygcs.setter
    def sysid_mygcs(self, new_gcs_id: int) -> None:
        assert 0 <= new_gcs_id <= 255
        self._set_parameter(Parameter.SYSID_MYGCS, new_gcs_id)

    @property
    def fs_ekf_action(self) -> ParameterValue:
        return self._get_parameter(Parameter.FS_EKF_ACTION)

    @fs_ekf_action.setter
    def fs_ekf_action(self, action: int) -> None:
        assert 1 <= action <= 3
        self._set_parameter(Parameter.FS_EKF_ACTION, action)

    @property
    def sim_gps_disable(self) -> ParameterValue:
        return self._get_parameter(Parameter.SIM_GPS_DISABLE)

    @sim_gps_disable.setter
    def sim_gps_disable(self, value: bool) -> None:
        self._set_parameter(Parameter.SIM_GPS_DISABLE, int(value))

    @property
    def sim_rc_fail(self) -> ParameterValue:
        return self._get_parameter(Parameter.SIM_RC_FAIL)

    @sim_rc_fail.setter
    def sim_rc_fail(self, value: bool) -> None:
        self._set_parameter(Parameter.SIM_RC_FAIL, int(value))

    # ------------------- Messages -------------------

    @property
    def gcs_heartbeat(self) -> mavlink.MAVLink_heartbeat_message:
        heartbeat_message = self._receive(
            message_or_parameter=Message.HEARTBEAT,
            condition=f"HEARTBEAT.get_srcSystem=={self.sysid_mygcs}",
            blocking=True,
        )

    @property
    def home_position(self) -> mavlink.MAVLink_home_position_message:
        """
        :return: Home drone position in SI units (meters, seconds, radians)
        """
        position = self._receive(Message.HOME_POSITION)
        position.latitude = math.radians(position.latitude / 1e7)
        position.longitude = math.radians(position.longitude / 1e7)
        position.altitude /= 1e3

        return position

    @property
    def current_position(self) -> mavlink.MAVLink_global_position_int_message:
        """
        :return: Current drone position in SI units (meters, seconds, radians)
        """
        position = self._receive(Message.GLOBAL_POSITION_INT)
        position.lat = math.radians(position.lat / 1e7)
        position.lon = math.radians(position.lon / 1e7)
        position.alt /= 1e1
        position.relative_alt /= 1e1  # Todo 3999 instead of 39
        position.vx /= 1e2
        position.vy /= 1e2
        position.vz /= 1e2
        position.hdg = math.radians(position.hdg / 1e2)

        return position

    @property
    def attitude(self) -> mavlink.MAVLink_attitude_message:
        return self._receive(Message.ATTITUDE)

    @property
    def local_position(self) -> mavlink.MAVLink_local_position_ned_message:
        """Local position and speed in north(x)-east(y)-down(z) convention"""
        return self._receive(Message.LOCAL_POSITION_NED)

    @property
    def gps_raw(self) -> mavlink.MAVLink_gps_raw_int_message:
        return self._receive(Message.GPS_RAW_INT)

    @property
    def sim_state(self) -> mavlink.MAVLink_sim_state_message:
        return self._receive(Message.SIM_STATE)

    @property
    def armed(self) -> bool:
        self._drone.wait_heartbeat()
        return self._drone.motors_armed()

    def set_home(
        self, position_radians_meters: Optional[Tuple[float, float, float]] = None
    ) -> None:
        """
        Sets the home position.
        `position_radians` is the tuple: (latitude, longitude, altitude) with the units (radians, radians, meters).
        If position is not given, the home will be set to the current location.
        """
        USE_SPECIFIED_LOCATION = 0
        USE_CURRENT_LOCATION = 1

        if position_radians_meters is None:
            self._command_int_send(
                mavlink.MAV_CMD_DO_SET_HOME, param1=USE_CURRENT_LOCATION
            )
        else:
            (
                latitude_radians,
                longitude_radians,
                altitude_meters,
            ) = position_radians_meters

            latitude_degrees_e7 = int(math.degrees(latitude_radians) * 1e7)
            longitude_degrees_e7 = int(math.degrees(longitude_radians) * 1e7)

            self._command_int_send(
                mavlink.MAV_CMD_DO_SET_HOME,
                param1=USE_SPECIFIED_LOCATION,
                param5=latitude_degrees_e7,
                param6=longitude_degrees_e7,
                param7=altitude_meters,
            )

    @property
    def mode(self) -> str:
        heartbeat: mavlink.MAVLink_heartbeat_message = self._receive(Message.HEARTBEAT)
        return mavutil.mode_string_v10(heartbeat)

    @mode.setter
    def mode(self, mode: str) -> None:
        print(f"Setting mode to {mode.upper()}...")
        self._drone.mav.set_mode_send(
            self._drone.target_system,
            mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            self._drone.mode_mapping()[mode.upper()],
        )

    def arm(self) -> None:
        print("Arming...")
        self._command_long_send(
            command_id=mavlink.MAV_CMD_COMPONENT_ARM_DISARM, param1=1, param2=0
        )

    def takeoff(self, height: int) -> None:
        if height <= 0:
            raise DroneTakeoffInvalidHeightError()
        print(f"Taking off to {height}...")
        self._command_long_send(command_id=mavlink.MAV_CMD_NAV_TAKEOFF, param7=height)

    def rotate_to(self, angle: int, rotation_speed: int = 10) -> None:
        """
        Rotates the drone to the fixed angle given.
        If the drone is not in mode guided, use the method: 'maintain_angle'.
        :param rotation_speed: The rotation speed in deg/sec.
        :param angle: Absolute angle (0 is north) in degrees.
        :return:
        """
        if self._drone.flightmode != "GUIDED":
            raise DroneConditionYawWithoutGuidedModeError()

        print(f"Rotating to {angle}...")
        self._command_long_send(
            command_id=mavlink.MAV_CMD_CONDITION_YAW,
            param1=angle,
            param2=rotation_speed,
            param4=0,
        )

    def rc(
        self, pitch: float = 0, roll: float = 0, yaw: float = 0, throttle: float = 0
    ) -> None:
        """
        Sends an RC command to the drone.
        The RC command is active for 1 second from the moment it
        is received.
        For generating a permanent RC command, a user should call
        this method repeatedly.
        The values are converted to integers in the range: -1000 to 1000
        so sending a value with ten thousandths in its factorial portion
        is worthless.
        :param pitch: Value between -1 to 1
        :param roll: Value between -1 to 1
        :param yaw: Value between -1 to 1
        :param throttle: Value between -1 to 1
        :return: None
        """
        assert -1 <= pitch <= 1
        assert -1 <= roll <= 1
        assert -1 <= yaw <= 1
        assert -1 <= throttle <= 1

        def convert_to_pwm(value: float) -> int:
            return int(1500 + 500 * value)

        pwm_pitch = convert_to_pwm(pitch)
        pwm_roll = convert_to_pwm(roll)
        pwm_yaw = convert_to_pwm(yaw)
        pwm_throttle = convert_to_pwm(throttle)

        # There are lots of messages that look the same, but only this message works with ArduPilot.
        # For every message sent, the RC is being overridden for 1 second.
        self._drone.mav.rc_channels_override_send(
            target_system=self._drone.target_system,
            target_component=self._drone.target_component,
            chan1_raw=pwm_roll,
            chan2_raw=pwm_pitch,
            chan3_raw=pwm_throttle,
            chan4_raw=pwm_yaw,
            chan5_raw=0,
            chan6_raw=0,
            chan7_raw=0,
            chan8_raw=0,
        )

    def _receive(
        self,
        message_or_parameter: Union[Parameter, Message] = None,
        condition: Optional[str] = None,
        blocking: bool = True,
        timeout: Optional[float] = RECEIVE_TIMEOUT_SEC,
    ) -> mavlink.MAVLink_message:
        """Returns the latest message available. If there is no latest message, waits for one."""
        if isinstance(message_or_parameter, Message):
            self.update()
        elif isinstance(message_or_parameter, Parameter):
            self._request_parameter(message_or_parameter)
        # Todo: add timeout and check with tests.
        return self._drone.recv_match(
            condition=condition,
            type=message_or_parameter.name,
            blocking=blocking,
            timeout=timeout,
        )

    def _get_parameter(self, parameter: Parameter) -> Union[int, float]:
        parameter_received: ParameterValue = self._receive(parameter)
        return parameter_received.param_value

    def _command_int_send(
        self,
        command_id: int,
        frame=mavlink.MAV_FRAME_GLOBAL,
        current=0,
        autocontinue=0,
        param1=0,
        param2=0,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    ) -> None:
        self._drone.mav.command_int_send(
            target_system=self._drone.target_system,
            target_component=self._drone.target_component,
            frame=frame,
            command=command_id,
            current=current,
            autocontinue=autocontinue,
            param1=param1,
            param2=param2,
            param3=param3,
            param4=param4,
            x=param5,
            y=param6,
            z=param7,
        )

    def _command_long_send(
        self,
        command_id: int,
        confirmation=0,
        param1=0,
        param2=0,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    ) -> None:
        self._drone.mav.command_long_send(
            self._drone.target_system,
            self._drone.target_component,
            command_id,
            confirmation,
            param1,
            param2,
            param3,
            param4,
            param5,
            param6,
            param7,
        )

    def _set_parameter(self, parameter: Parameter, value) -> None:
        # Request the parameter
        self._drone.mav.param_set_send(
            self._drone.target_system,
            self._drone.target_component,
            parameter.name.encode("UTF-8"),
            value,
            PARAMETER_TYPE_MAP[parameter],
        )

    def _request_parameter(self, parameter: Parameter) -> None:
        IGNORE_PARAM_INDEX = -1
        self._drone.mav.param_request_read_send(
            target_system=self._drone.target_system,
            target_component=self._drone.target_component,
            param_id=parameter.name.encode("UTF-8"),
            param_index=IGNORE_PARAM_INDEX,
        )
