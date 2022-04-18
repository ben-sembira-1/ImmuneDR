from dataclasses import dataclass
import logging
from platform import mac_ver
import signal
from typing import Dict, Optional, Generator
from pathlib import Path
from contextlib import contextmanager
import os
from subprocess import Popen 
import pymavlink.mavutil

@dataclass
class TcpSerialConnectionDef:
    '''Represents a tcp connection configuration to the simulation.
    if wait_for_connection is True, the simulation will not start until this connection was made.
    '''
    port: int
    host: str = 'localhost'
    wait_for_connection: bool = False

    def as_ardupilot_cli_argument(self) -> str:
        return f'tcp:{self.port}' + (':wait' if self.wait_for_connection else '')
    
    def as_pymavlink_connection_argument(self) -> str:
        return f'tcp:{self.host}:{self.port}'


class _RunningSimulation:
    process: Popen
    _serial_mappings: Dict[int, TcpSerialConnectionDef]

    def __init__(self, process: Popen, serial: Dict[int, TcpSerialConnectionDef]) -> None:
        self.process = process
        self._serial_mappings = serial
    
    def mavlink_connect_to_serial(self, serial_number: int) -> pymavlink.mavutil.mavfile:
        serial_def = self._serial_mappings[serial_number]
        logging.debug(f"Create mavlink connection to serial {serial_number}: {serial_def}")
        return pymavlink.mavutil.mavlink_connection(serial_def.as_pymavlink_connection_argument())
        

DEFAULT_SERIAL_MAPPING = {
    0: TcpSerialConnectionDef(port=5760, wait_for_connection=True),
    1: TcpSerialConnectionDef(port=5762, wait_for_connection=False),
    2: TcpSerialConnectionDef(port=5763, wait_for_connection=False),
}

@contextmanager
def simulation_context(*, cwd: Optional[Path] = None, serial: Optional[Dict[int, TcpSerialConnectionDef]]) -> Generator[_RunningSimulation, None, None]:
    cwd = cwd if cwd is not None else Path.cwd()
    serial = serial if serial is not None else DEFAULT_SERIAL_MAPPING 
    SIM_VEHICLE_PATH = os.environ.get("SIM_VEHICLE_PATH", "/home/pilot/ardupilot/Tools/autotest/sim_vehicle.py")

    args_to_binary = ["-A"] + [f"--serial{n}={s.as_ardupilot_cli_argument()}" for n, s in serial.items()]
    
    sitl_args = [
        SIM_VEHICLE_PATH,
        "-v",
        "ArduCopter",
        "--no-rebuild",
        "--no-mavproxy",
    ]

    if len(args_to_binary) > 1:
        sitl_args.extend(args_to_binary)
    
    logging.info(f"Starting simulation with parameters: {sitl_args} in directory {cwd}")

    with Popen(args=sitl_args, cwd=cwd) as sitl_process:
        try: 
            yield _RunningSimulation(sitl_process, serial)
        finally:
            logging.info("Stoping simulation")
            sitl_process.send_signal(signal.SIGINT)
            ret = sitl_process.wait(10)
            if ret != 0:
                logging.error(f"Simulation exited with non zero code {ret}")
            else:
                logging.info("Simulation ended successfully")