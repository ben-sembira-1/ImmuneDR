import logging
from threading import Event, Thread
from typing import Dict, Optional
from pymavlink.mavutil import mavfile
from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink,
    MAVLink_gps_raw_int_message,
    MAVLink_message,
    MAVLink_ekf_status_report_message,
    MAVLink_local_position_ned_message,
    MAV_CMD_SET_MESSAGE_INTERVAL,
    MAVLink_attitude_message,
    MAVLink_global_position_int_message,
    MAVLink_attitude_target_message,
    MAVLink_statustext_message,
    MAV_SEVERITY_CRITICAL,
    MAV_SEVERITY_ERROR,
    MAV_SEVERITY_WARNING,
    MAV_SEVERITY_NOTICE,
)
from async_state_machine.client import Client

from drones.commands import CommandReceiver, CommandSender, create_command_queue_pair
from drones.drone_client import DroneClient

MESSAGES_INTERVAL_US: Dict[MAVLink_message, float] = {
    MAVLink_ekf_status_report_message: 10_000.0,
    MAVLink_local_position_ned_message: 10_000.0,
    MAVLink_gps_raw_int_message: 1_000_000.0,
    MAVLink_attitude_message: 10_000.0,
    MAVLink_global_position_int_message: 100_000.0,
    MAVLink_attitude_target_message: 100_000.0,
}


def _log_statustext_message(msg: MAVLink_statustext_message) -> None:
    if msg.severity <= MAV_SEVERITY_CRITICAL:
        level = logging.CRITICAL
    elif msg.severity <= MAV_SEVERITY_ERROR:
        level = logging.ERROR
    elif msg.severity <= MAV_SEVERITY_WARNING:
        level = logging.WARNING
    elif msg.severity <= MAV_SEVERITY_NOTICE:
        level = logging.INFO
    else:
        level = logging.DEBUG
    logging.log(level, f"Received status message: {msg.text}")


def _mavlink_control_loop(
    mavlink_connection: mavfile,
    sender_client: Client[MAVLink_message],
    command_rx_queue: CommandReceiver,
    stop_loop: Event,
    target_system_id: int,
    target_component_id: int,
    message_intervals: Dict[MAVLink_message, float],
) -> None:
    try:
        mav: MAVLink = mavlink_connection.mav
        for msg_type, interval in message_intervals.items():
            logging.info(
                f"Set message {msg_type.name} ({msg_type.id}) to {interval}[us]"
            )
            mav.command_long_send(
                target_system=target_system_id,
                target_component=target_component_id,
                command=MAV_CMD_SET_MESSAGE_INTERVAL,
                confirmation=0,
                param1=msg_type.id,
                param2=float(interval),
                param3=0.0,
                param4=0.0,
                param5=0.0,
                param6=0.0,
                param7=0.0,
            )

        logging.info(f"Starting mavlink message loop")
        while not stop_loop.is_set():
            msg = mavlink_connection.recv_msg()
            if msg is not None:
                if msg.get_msgId() == MAVLink_statustext_message.id:
                    _log_statustext_message(msg)
                else:
                    logging.debug(f"drone daemon received mavlink message: {msg}")
                sender_client.to_state_machine(msg)
            cmd = command_rx_queue.receive()
            if cmd is not None:
                cmd(mavlink_connection=mavlink_connection)
    except Exception as e:
        logging.error(f"Drone daemon loop failed: {e}")
        raise
    finally:
        mavlink_connection.close()


class DroneDaemon:
    drone_system_id: int
    drone_component_id: int
    _mavlink_connection: mavfile
    _sender_client: Client[MAVLink_message]
    _stop_loop_event: Event
    _command_sender: CommandSender

    def __init__(
        self,
        mavlink_connection: mavfile,
        drone_system_id: int = 1,
        drone_component_id: int = 1,
        message_intervals: Optional[Dict[MAVLink_message, float]] = None,
    ) -> None:
        self.drone_system_id = drone_system_id
        self.drone_component_id = drone_component_id
        self._mavlink_connection = mavlink_connection
        self._sender_client = Client()
        self._stop_loop_event = Event()
        (self._command_sender, command_receiver) = create_command_queue_pair()
        message_intervals = message_intervals or MESSAGES_INTERVAL_US
        self._thread = Thread(
            target=_mavlink_control_loop,
            kwargs=dict(
                mavlink_connection=self._mavlink_connection,
                sender_client=self._sender_client,
                stop_loop=self._stop_loop_event,
                command_rx_queue=command_receiver,
                target_system_id=self.drone_system_id,
                target_component_id=self.drone_component_id,
                message_intervals=message_intervals,
            ),
        )
        self._thread.start()

    def create_client(self) -> DroneClient:
        return DroneClient(self._sender_client.events(), self._command_sender)

    def __del__(self) -> None:
        self._stop_loop_event.set()
        self._thread.join(10)
