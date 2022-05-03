from dataclasses import dataclass
from enum import Enum


class FlightMode(Enum):
    """
    For a description of the different supported flight modes, see https://ardupilot.org/copter/docs/flight-modes.html

    See https://mavlink.io/en/messages/ardupilotmega.html#COPTER_MODE for the full list of flight modes and their enum
    values
    """

    GUIDED = 4
    LAND = 9


@dataclass
class LocalPositionNED:
    """Position in meters. Origin is where the"""

    north: float = 0
    east: float = 0
    down: float = 0


@dataclass
class GlobalPositionInt:
    """
    The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up).
    It is designed as scaled integer message since the resolution of float is not sufficient.

    https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT
    """

    time_boot_ms: int  # Timestamp since system boot
    latitude: int  # degE7
    longitude: int  # degE7
    altitude: int  # Altitude in mm (MSL)
    altitude_above_ground: int  # mm
    vx: int  # Ground X speed in cm/s
    vy: int  # Ground Y speed in cm/s
    vz: int  # Ground Z speed in cm/s
    heading_cdeg: int  # cdeg

    @property
    def heading_deg(self) -> float:
        return self.heading_cdeg / 100
