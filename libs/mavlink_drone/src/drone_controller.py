import enum
import math
import threading
import time
from typing import NamedTuple, Optional, Tuple, Union, Dict
import copy

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from pytest import param


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


class DroneInvalidParameterValueError(DroneError):
    pass


class FailedToSetPrameterError(DroneError):
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
    PARAM_VALUE = mavlink.MAVLINK_MSG_ID_PARAM_VALUE


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


class MavlinkData(NamedTuple):
    message: Optional[mavlink.MAVLink_message]
    timestamp: float


class Drone:
    MAX_SET_PARAMETER_ACK_TIMEOUT = 2e-1
    DEFAULT_GET_MESSAGE_MAX_AGE_SEC = 4e-1
    DEFAULT_GET_MESSAGE_TIMEOUT_SEC = 2e-1
    GET_PARAMETER_MAX_AGE_SEC = 1e-1
    GET_HEARTBEAT_MAX_AGE_SEC = 2
    PARAMETER_RECEIVE_TIMEOUT_SEC = 1

    def __init__(
        self,
        source_system_id: int,
        source_component_id: int = mavlink.MAV_COMP_ID_AUTOPILOT1,
        print_logs: bool = False,
        simulation_speedup: float = 1,
    ):
        assert 0 <= source_system_id <= 255
        assert 0 <= source_component_id <= 255
        self.source_system_id = source_system_id
        self.source_component_id = source_component_id
        self._print_logs = print_logs
        self._simulation_speedup = simulation_speedup
        self._boot_time_sec: float = time.time()
        self._drone: Optional[mavutil.mavfile] = None
        self._latest_mavlink_data: Dict[str, MavlinkData] = {}
        self._latest_heartbeats: Dict[int, MavlinkData] = {}
        self._recv_thread: threading.Thread = threading.Thread(
            target=self._thr_update_mavlink_messages_loop, daemon=True
        )
        self._kill_recv_thread: bool = False

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

        # Make sure that all the messages arrive successfully
        self.request_all_messages()

    def run(self):
        """This will open a thread that actively listens to all incoming relevant messages."""
        if self._drone is None:
            raise RuntimeError("Call connect before you call run")
        self._recv_thread.start()

    def stop(self):
        MAX_JOIN_TIMEOUT_SEC = 1
        self._kill_recv_thread = True
        self._recv_thread.join(timeout=MAX_JOIN_TIMEOUT_SEC)
        if self._recv_thread.is_alive():
            self._log("!!! mavlink updater thread did not stop. !!!")

    # ------------------- Parameters -------------------

    def set_parameter(
        self, parameter: Union[Parameter, str], value, retries: int = 1
    ) -> None:
        if retries < 0:
            raise FailedToSetPrameterError()
        # Request the parameter
        if isinstance(parameter, str):
            parameter_name = parameter
        elif isinstance(parameter, Parameter):
            parameter_name = parameter.name
        else:
            raise TypeError(
                f"The parameter should be a string or Parameter, but it is: {type(parameter)}."
            )
        self._log(f"Setting {parameter_name} to be {value}")
        self._drone.mav.param_set_send(
            self._drone.target_system,
            self._drone.target_component,
            parameter_name.encode("UTF-8"),
            value,
            PARAMETER_TYPE_MAP[parameter],
        )
        if (
            self._receive(
                data_type=parameter,
                timeout=Drone.MAX_SET_PARAMETER_ACK_TIMEOUT,
            )
            is None
        ):
            self.set_parameter(parameter=parameter, value=value, retries=retries - 1)

    def get_parameter(
        self,
        parameter: Parameter,
        timeout: float = PARAMETER_RECEIVE_TIMEOUT_SEC,
        max_age: float = GET_PARAMETER_MAX_AGE_SEC,
    ) -> Optional[float]:
        # Todo: Add test for if the parameter was set, if it did - do not request
        if data := self._new_data(data_name=parameter.name, max_age=max_age):
            data: Optional[mavlink.MAVLink_param_value_message]
            return data.param_value
        self._request_parameter(parameter)
        if data := self._receive(data_type=parameter, timeout=timeout):
            data: Optional[mavlink.MAVLink_param_value_message]
            return data.param_value
        return None

    @property
    def sysid_mygcs(self) -> ParameterValue:
        # import pdb; pdb.set_trace()
        return self.get_parameter(Parameter.SYSID_MYGCS)

    @sysid_mygcs.setter
    def sysid_mygcs(self, new_gcs_id: int) -> None:
        if not 0 <= new_gcs_id <= 255:
            raise DroneInvalidParameterValueError("Value should be between 0-255")
        self.set_parameter(Parameter.SYSID_MYGCS, new_gcs_id)

    @property
    def fs_ekf_action(self) -> ParameterValue:
        return self.get_parameter(Parameter.FS_EKF_ACTION)

    @fs_ekf_action.setter
    def fs_ekf_action(self, action: int) -> None:
        if not 1 <= action <= 3:
            raise DroneInvalidParameterValueError("Value should be between 1-3")
        self.set_parameter(Parameter.FS_EKF_ACTION, action)

    @property
    def sim_gps_disable(self) -> ParameterValue:
        return self.get_parameter(Parameter.SIM_GPS_DISABLE)

    @sim_gps_disable.setter
    def sim_gps_disable(self, value: bool) -> None:
        self.set_parameter(Parameter.SIM_GPS_DISABLE, int(value))

    @property
    def sim_rc_fail(self) -> ParameterValue:
        return self.get_parameter(Parameter.SIM_RC_FAIL)

    @sim_rc_fail.setter
    def sim_rc_fail(self, value: bool) -> None:
        if int(value) not in {0, 1}:
            raise DroneInvalidParameterValueError("Value should be 0 or 1")
        self.set_parameter(Parameter.SIM_RC_FAIL, int(value))

    # ------------------- Messages -------------------

    def request_all_messages(self) -> None:
        for message in Message:
            self._log(f"Requesting {repr(message)}...")
            self._request_message_interval(message_id=message, interval=4e5)

    def get_message(
        self,
        message: Union[str, Message],
        max_age: float = DEFAULT_GET_MESSAGE_MAX_AGE_SEC,
        timeout: float = DEFAULT_GET_MESSAGE_TIMEOUT_SEC,
    ):
        """Returns the latest message of the kind specified.
        If there is no message with age smaller then the age specified, return None."""
        CHECK_MESSAGE_AGAIN_COOLDOWN_SEC = 1e-1
        if isinstance(message, Message):
            message_name = message.name
        elif isinstance(message, str):
            message_name = message
        else:
            raise TypeError(
                f"Excepted message of type Message or str, got {type(message)}"
            )
        start_time = time.time()
        new_data = self._new_data(data_name=message_name, max_age=max_age)
        while new_data is None and self._time_since(start_time) < timeout:
            self._wait(CHECK_MESSAGE_AGAIN_COOLDOWN_SEC)
            new_data = self._new_data(data_name=message_name, max_age=max_age)
        return new_data

    def get_gcs_heartbeat(
        self, max_age: float = GET_HEARTBEAT_MAX_AGE_SEC
    ) -> Optional[mavlink.MAVLink_heartbeat_message]:
        return self._new_heartbeat(system_id=self.sysid_mygcs, max_age=max_age)

    @property
    def home_position(self) -> mavlink.MAVLink_home_position_message:
        position = self.get_message(Message.HOME_POSITION)

        position.latitude = math.radians(position.latitude / 1e7)
        position.longitude = math.radians(position.longitude / 1e7)
        position.altitude /= 1e3

        return position

    @home_position.setter
    def home_position(
        self, position_radians_meters: Optional[Tuple[float, float, float]]
    ) -> None:
        """
        Sets the home position.
        `position_radians` is the tuple: (latitude, longitude, altitude) with the units (radians, radians, meters).
        If position is None, the home will be set to the current location.
        """
        USE_SPECIFIED_LOCATION = 0
        USE_CURRENT_LOCATION = 1

        location_type = USE_CURRENT_LOCATION
        latitude_degrees_e7 = 0
        longitude_degrees_e7 = 0
        altitude_meters = 0

        if position_radians_meters is not None:
            location_type = USE_SPECIFIED_LOCATION
            (
                latitude_radians,
                longitude_radians,
                altitude_meters,
            ) = position_radians_meters

            latitude_degrees_e7 = int(math.degrees(latitude_radians) * 1e7)
            longitude_degrees_e7 = int(math.degrees(longitude_radians) * 1e7)

        self._command_int_send(
            mavlink.MAV_CMD_DO_SET_HOME,
            param1=location_type,
            param5=latitude_degrees_e7,
            param6=longitude_degrees_e7,
            param7=altitude_meters,
        )

    @property
    def current_position(self) -> mavlink.MAVLink_global_position_int_message:
        position = self.get_message(Message.GLOBAL_POSITION_INT)
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
        return self.get_message(Message.ATTITUDE)

    @property
    def local_position(self) -> mavlink.MAVLink_local_position_ned_message:
        """Local position and speed in north(x)-east(y)-down(z) convention"""
        return self.get_message(Message.LOCAL_POSITION_NED)

    @property
    def gps_raw(self) -> mavlink.MAVLink_gps_raw_int_message:
        return self.get_message(Message.GPS_RAW_INT)

    @property
    def sim_state(self) -> mavlink.MAVLink_sim_state_message:
        return self.get_message(Message.SIM_STATE)

    @property
    def armed(self) -> bool:
        return (
            self._new_heartbeat(
                system_id=self._drone.target_system,
                max_age=Drone.GET_HEARTBEAT_MAX_AGE_SEC,
            )
            and self._drone.motors_armed()
        )

    @property
    def mode(self) -> str:
        heartbeat: mavlink.MAVLink_heartbeat_message = self._new_heartbeat(
            system_id=self._drone.target_system, max_age=Drone.GET_HEARTBEAT_MAX_AGE_SEC
        )
        return mavutil.mode_string_v10(heartbeat)

    @mode.setter
    def mode(self, mode: str) -> None:
        self._log(f"Setting mode to {mode.upper()}...")
        self._drone.mav.set_mode_send(
            self._drone.target_system,
            mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            self._drone.mode_mapping()[mode.upper()],
        )

    # ------------------- Commands -------------------

    def arm(self) -> None:
        self._log("Arming...")
        self._command_long_send(
            command_id=mavlink.MAV_CMD_COMPONENT_ARM_DISARM, param1=1, param2=0
        )

    def takeoff(self, height: int) -> None:
        if height <= 0:
            raise DroneTakeoffInvalidHeightError()
        self._log(f"Taking off to {height}...")
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

        self._log(f"Rotating to {angle}...")
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

    # ------------------- MAVLink interface -------------------

    def _receive(
        self,
        data_type: Union[Message, Parameter],
        timeout: float = 0,
        max_age: float = 0,
    ) -> Optional[mavlink.MAVLink_message]:
        """
        Returns the latest message available of maximum age.
        If there is no available message wait the given timeout.

        In case there are no available messages, return None.
        """
        COOLDOWN_SEC = 0.01
        request_time = time.time()
        data = self._new_data(
            data_name=data_type.name, max_age=max_age, basetime=request_time
        )
        while self._time_since(request_time) < timeout and data is None:
            self._wait(COOLDOWN_SEC)
            data = self._new_data(
                data_name=data_type.name, max_age=max_age, basetime=request_time
            )
        return data

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

    def _request_parameter(self, parameter: Parameter) -> None:
        self._log(f"Requesing the parameter {repr(parameter)}")
        IGNORE_PARAM_INDEX = -1
        self._drone.mav.param_request_read_send(
            target_system=self._drone.target_system,
            target_component=self._drone.target_component,
            param_id=parameter.name.encode("UTF-8"),
            param_index=IGNORE_PARAM_INDEX,
        )

    def _request_message_interval(self, message_id: int, interval=0) -> None:
        self._command_long_send(
            command_id=mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            param1=message_id,
            param2=interval,
        )

    # ------------------- Utils -------------------

    def _log(self, message: str) -> None:
        if self._print_logs:
            print(f"[+{time.time() - self._boot_time_sec:.3f}] {message}")

    def _wait(self, time_sec: float):
        time.sleep(time_sec / self._simulation_speedup)

    def _time_since(self, base_time_sec: float):
        return (time.time() - base_time_sec) * self._simulation_speedup

    # ------------------- Mavlink listener -------------------

    def _new_heartbeat(
        self, system_id: int, max_age: float, basetime: Optional[float] = None
    ):
        if basetime is None:
            basetime = time.time()
        self._log(
            f"Checking for heartbeat from {system_id} of max age: {max_age} in: {self._latest_heartbeats}"
        )
        if system_id in self._latest_heartbeats and self._time_since(
            self._latest_heartbeats[system_id].timestamp
        ) < self._time_since(basetime - max_age):
            return copy.deepcopy(self._latest_heartbeats[system_id].message)
        return None

    def _new_data(
        self, data_name: str, max_age: float, basetime: Optional[float] = None
    ) -> MavlinkData:
        if basetime is None:
            basetime = time.time()
        if data_name in self._latest_mavlink_data and self._time_since(
            (mavlink_data := self._latest_mavlink_data[data_name]).timestamp
        ) < self._time_since(basetime - max_age):
            return copy.deepcopy(mavlink_data.message)
        return None

    def _thr_update_mavlink_messages_loop(self):
        COOLDOWN_SEC = 0.01
        while not self._kill_recv_thread:
            if (new_message := self._drone.recv_match(blocking=False)) is not None:
                # self._log(
                #     f"Received new mavlink packet (type: {new_message.get_type()}): {repr(new_message)}"
                # )
                new_message: mavlink.MAVLink_message
                new_mavlink_data = MavlinkData(
                    message=new_message, timestamp=time.time()
                )
                self._latest_mavlink_data[new_message.get_type()] = new_mavlink_data
                if new_message.get_type() == Message.PARAM_VALUE.name:
                    new_message: mavlink.MAVLink_param_value_message
                    self._latest_mavlink_data[new_message.param_id] = new_mavlink_data
                    # self._log(f"Received parameter: {new_message.param_id}")
                if new_message.get_type() == Message.HEARTBEAT.name:
                    self._latest_heartbeats[
                        new_message.get_srcSystem()
                    ] = new_mavlink_data
            else:
                self._wait(COOLDOWN_SEC)
                # self._log("Cooling down")
