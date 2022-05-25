from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin, atan2, radians, degrees, sqrt
from typing import Any, ClassVar, Union
from functools import singledispatchmethod


from pymavlink.dialects.v20.ardupilotmega import MAVLink_global_position_int_message, MAVLink_sim_state_message


@dataclass
class GeoLocation:
    '''Represent a geo-location. longitude and latitude are in deg'''
    EARTH_RADIUS_KM: ClassVar[float] = 6371.009
    longitude: float
    latitude: float


    def bearing_to(self, destination: GeoLocation) -> float:
        '''calculates the bearing angle (azimuth) from self to destination. In degrees [0, 360)'''
        d_longitude = destination.longitude - self.longitude
        x = cos(radians(destination.latitude)) * sin(radians(d_longitude))
        y = cos(radians(self.latitude)) * sin(radians(destination.latitude)) - sin(
            radians(self.latitude)
        ) * cos(radians(destination.latitude)) * cos(radians(d_longitude))
        bearing_rad = atan2(x, y)
        return degrees(bearing_rad) % 360

    def distance_to(self, destination: GeoLocation) -> float:
        '''Calculates the distance in meters
        based on: https://github.com/geopy/geopy/blob/ade9c1b68c83a1fc76d90bc0ff603f6b34dfbbbf/geopy/distance.py
        '''

        lat1, lng1 = radians(self.latitude), radians(self.longitude)
        lat2, lng2 = radians(destination.latitude), radians(destination.longitude)

        sin_lat1, cos_lat1 = sin(lat1), cos(lat1)
        sin_lat2, cos_lat2 = sin(lat2), cos(lat2)

        delta_lng = lng2 - lng1
        cos_delta_lng, sin_delta_lng = cos(delta_lng), sin(delta_lng)

        d = atan2(sqrt((cos_lat2 * sin_delta_lng) ** 2 +
                       (cos_lat1 * sin_lat2 -
                        sin_lat1 * cos_lat2 * cos_delta_lng) ** 2),
                  sin_lat1 * sin_lat2 + cos_lat1 * cos_lat2 * cos_delta_lng)

        return d * self.EARTH_RADIUS_KM * 1e3
    
    @singledispatchmethod
    @classmethod
    def from_message(cls, message: Any) -> GeoLocation:
        raise NotImplementedError(f"What type of message is it? {message}")
    
    @from_message.register(MAVLink_global_position_int_message)
    @from_message.register(MAVLink_sim_state_message)
    @classmethod
    def _(cls, message: Union[MAVLink_global_position_int_message, MAVLink_sim_state_message]) -> GeoLocation:
        return cls(
            longitude=message.lon * 1e-7,
            latitude=message.lat * 1e-7,
        )