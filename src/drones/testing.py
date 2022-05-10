import enum
from collections import Counter
from dataclasses import dataclass
import logging
import signal
from typing import Dict, Iterator, Optional, Generator, Mapping, cast
from pathlib import Path
from contextlib import contextmanager
import os
from subprocess import Popen
import pymavlink.mavutil
from pymavlink.mavutil import mavfile
from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_message,
    MAVLink_gps_raw_int_message,
    GPS_FIX_TYPE_NO_FIX,
)

from async_state_machine.transitions.types import TransitionCheckerFactory

from drones.commands import Command

from drones.drone_client import ActAndWaitForCheckerFactory, DroneClient
from drones.drone_daemon import DroneDaemon

PREALLOCATED_SIMULATION_PORTS = {5763}


class Parameter(enum.Enum):
    SYSID_MYGCS = enum.auto()
    FS_EKF_ACTION = enum.auto()
    SIM_GPS_DISABLE = enum.auto()
    SIM_RC_FAIL = enum.auto()


@dataclass
class TcpSerialConnectionDef:
    """Represents a tcp connection configuration to the simulation.
    if wait_for_connection is True, the simulation will not start until this connection was made.
    """

    port: int
    host: str = "localhost"
    wait_for_connection: bool = False

    def as_ardupilot_cli_argument(self) -> str:
        return f"tcp:{self.port}" + (":wait" if self.wait_for_connection else "")

    def as_pymavlink_connection_argument(self) -> str:
        return f"tcp:{self.host}:{self.port}"


def _gps_fix_is_no_fix(message: MAVLink_message) -> Optional[bool]:
    if message.get_msgId() != MAVLink_gps_raw_int_message.id:
        return None
    assert isinstance(message, MAVLink_gps_raw_int_message)
    logging.debug(f"GPS raw int: {str(message)}")
    return cast(bool, message.fix_type == GPS_FIX_TYPE_NO_FIX)


class SetParameter(Command):
    def __init__(self, parameter: Parameter, value):
        super().__init__()
        self.parameter = parameter
        self.value = value

    def __call__(self, mavlink_connection: mavfile) -> None:
        mavlink_connection.param_set_send(self.parameter.name, self.value)


class TurnOffGps(SetParameter):
    def __init__(self):
        super(TurnOffGps, self).__init__(Parameter.SIM_GPS_DISABLE, 1)


class SimulationDroneClient(DroneClient):
    def turn_off_gps(self) -> TransitionCheckerFactory:
        return ActAndWaitForCheckerFactory(
            action_callback=lambda: self._commands_queue_tx.send(TurnOffGps()),
            wait_for=self._event_client.when(_gps_fix_is_no_fix),
        )


class SimulationDroneDaemon(DroneDaemon):
    def create_client(self) -> SimulationDroneClient:
        return SimulationDroneClient(self._sender_client.events(), self._command_sender)


class _RunningSimulation:
    process: Popen
    _serial_mappings: Dict[int, TcpSerialConnectionDef]

    def __init__(
        self, process: Popen, serial: Dict[int, TcpSerialConnectionDef]
    ) -> None:
        self.process = process
        self._serial_mappings = serial

    def mavlink_connect_to_serial(
        self, serial_number: int
    ) -> pymavlink.mavutil.mavfile:
        serial_def = self._serial_mappings[serial_number]
        logging.debug(
            f"Create mavlink connection to serial {serial_number}: {serial_def}"
        )
        return pymavlink.mavutil.mavlink_connection(
            serial_def.as_pymavlink_connection_argument()
        )


# Defined by the simulation binary of sim_vehicle - do not change
DEFAULT_SERIAL_MAPPING = {
    0: TcpSerialConnectionDef(port=5760, wait_for_connection=True),
    1: TcpSerialConnectionDef(port=5762, wait_for_connection=False),
    2: TcpSerialConnectionDef(port=5763, wait_for_connection=False),
}


def all_unique(it: Iterator) -> bool:
    return all(count == 1 for count in Counter(it).values())


def updated(dict: Dict, overlay: Mapping) -> Dict:
    r = dict.copy()
    r.update(overlay)
    return r


@contextmanager
def simulation_context(
    *,
    cwd: Optional[Path] = None,
    serial_ports_override: Optional[Dict[int, TcpSerialConnectionDef]],
    parameter_file: Optional[Path] = None,
    speedup: int = 1,
) -> Generator[_RunningSimulation, None, None]:
    """
    When wating multiple ports, the order of the serial_ports determain the order for which ports will be opened and waited on.
    """
    cwd = cwd if cwd is not None else Path.cwd()
    serial_ports_override = (
        serial_ports_override if serial_ports_override is not None else {}
    )
    serial_ports = updated(DEFAULT_SERIAL_MAPPING, serial_ports_override)
    SIM_VEHICLE_PATH = os.environ.get(
        "SIM_VEHICLE_PATH", "/home/pilot/ardupilot/Tools/autotest/sim_vehicle.py"
    )
    if parameter_file is not None:
        assert (
            parameter_file.is_file()
        ), f"The parameter file {parameter_file} path is not a file"

    assert all(
        con_def.port.bit_length() <= 16 for con_def in serial_ports.values()
    ), "Ports should be u16 representable."
    assert all_unique(
        con_def.port for con_def in serial_ports.values()
    ), "Ports shouldn't overlap"

    logging.info(
        f"Creating simulation with the following serial mapping {serial_ports}"
    )

    # sim_vehicle.py passes -A argument (single string expected) to the simulation binary
    args_to_binary = [
        "-A",
        " ".join(
            f"--serial{n}={s.as_ardupilot_cli_argument()}"
            for n, s in serial_ports.items()
        ),
    ]

    sitl_args = [
        "python3",
        SIM_VEHICLE_PATH,
        "-v",
        "ArduCopter",
        "--no-rebuild",
        "--no-mavproxy",
        "--speedup",
        str(speedup),
    ]
    if parameter_file is not None:
        sitl_args.extend(["--add-param-file", parameter_file])

    if len(args_to_binary) > 1:
        sitl_args.extend(args_to_binary)

    logging.info(f"Starting simulation with parameters: {sitl_args} in directory {cwd}")

    with Popen(args=sitl_args, cwd=cwd) as sitl_process:
        try:
            yield _RunningSimulation(sitl_process, serial_ports_override)
        finally:
            logging.info("Stopping simulation")
            sitl_process.send_signal(signal.SIGINT)
            ret = sitl_process.wait(10)
            if ret != 0:
                logging.error(f"Simulation exited with non zero code {ret}")
            else:
                logging.info("Simulation ended successfully")
            simulation_log = Path("/tmp/ArduCopter.log").read_text()
            logging.debug(f"Simulation log is\n{simulation_log}")
