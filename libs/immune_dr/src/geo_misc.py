import math
from typing import Tuple


def compass_bearing_between_2_coordinates(
    point_from: Tuple[float, float], point_to: Tuple[float, float]
):
    """
    This function calculates the compass bearing
    between 2 latitude-longitude points.

    The compass bearing is the angle between
    a given direction to the true north (clockwise).

    :param point_from: A tuple of latitude,longitude in radians
    :param point_to: A tuple of latitude,longitude in radians
    """
    from_lat, from_lon = point_from
    to_lat, to_lon = point_to
    delta_lon = from_lon - to_lon

    x = math.cos(from_lat) * math.sin(delta_lon)
    y = math.cos(to_lat) * math.sin(from_lat) - math.sin(to_lat) * math.cos(
        from_lat
    ) * math.cos(delta_lon)

    return math.atan2(x, y) % (2 * math.pi)


def distance_between_2_coordinates(
    point_from: Tuple[float, float], point_to: Tuple[float, float]
):
    """
    This function calculates the distance
    between 2 latitude-longitude points in meters.

    :param point_from: A tuple of latitude,longitude in radians
    :param point_to: A tuple of latitude,longitude in radians
    """
    EARTH_MEAN_RADIUS_METERS = 6371e3

    lat1, lon1 = point_from
    lat2, lon2 = point_to
    delta_lon = lon1 - lon2
    delta_lat = lat1 - lat2

    a = (
        math.sin(delta_lat / 2) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(delta_lon / 2) ** 2
    )

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return c * EARTH_MEAN_RADIUS_METERS

    # R = 6371e3
    # φ1 = lat1
    # φ2 = lat2
    # Δφ = lat2 - lat1
    # Δλ = lon2 - lon1

    # a = math.sin(Δφ / 2) * math.sin(Δφ / 2) + math.cos(φ1) * math.cos(φ2) * math.sin(
    #     Δλ / 2
    # ) * math.sin(Δλ / 2)
    # c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # d = R * c

    # return d
