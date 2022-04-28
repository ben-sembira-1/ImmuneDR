from collections import Counter
from dataclasses import dataclass
import logging
import signal
from typing import Dict, Iterator, Optional, Generator
from pathlib import Path
from contextlib import contextmanager
import os
from subprocess import Popen 
import pymavlink.mavutil
from pyparsing import Mapping


PREALLOCATED_SIMULATION_PORTS = {5763}
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
def simulation_context(*, cwd: Optional[Path] = None, serial_ports_override: Optional[Dict[int, TcpSerialConnectionDef]]) -> Generator[_RunningSimulation, None, None]:
    '''
    When wating multiple ports, the order of the serial_ports determain the order for which ports will be opened and waited on.
    '''
    cwd = cwd if cwd is not None else Path.cwd()
    serial_ports_override = serial_ports_override if serial_ports_override is not None else {}
    serial_ports = updated(DEFAULT_SERIAL_MAPPING, serial_ports_override)
    SIM_VEHICLE_PATH = os.environ.get("SIM_VEHICLE_PATH", "/home/pilot/ardupilot/Tools/autotest/sim_vehicle.py")

    assert all(con_def.port.bit_length() <= 16 for con_def in serial_ports.values()), "Ports should be u16 representable."
    assert all_unique(con_def.port for con_def in serial_ports.values()), "Ports shouldn't overlap"

    logging.info(f"Creating simulation with the following serial mapping {serial_ports}")

    # sim_vehicle.py passes -A argument (single string expected) to the simulation binary
    args_to_binary = ["-A", ' '.join(f"--serial{n}={s.as_ardupilot_cli_argument()}" for n, s in serial_ports.items())]
    
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
            yield _RunningSimulation(sitl_process, serial_ports_override)
        finally:
            logging.info("Stoping simulation")
            sitl_process.send_signal(signal.SIGINT)
            ret = sitl_process.wait(10)
            if ret != 0:
                logging.error(f"Simulation exited with non zero code {ret}")
            else:
                logging.info("Simulation ended successfully")
            simulation_log = Path('/tmp/ArduCopter.log').read_text()
            logging.debug(f"Simulation log is\n{simulation_log}")