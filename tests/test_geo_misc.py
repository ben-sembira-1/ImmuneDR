import math
from typing import Tuple

import pytest

from drones import geo_misc

@pytest.mark.parametrize(
    "point_from, point_to, expected_bearing",
    [
        (
            (math.radians(38.8976), math.radians(-75.1511)),
            (math.radians(39.9496), math.radians(-77.0391)),
            125.8,
        ),
        (
            (math.radians(55.7393), math.radians(37.5925)),
            (math.radians(55.7388), math.radians(37.5956)),
            285.99,
        ),
    ],
)
def test_compass_bearing(point_from: Tuple[float, float],
    point_to: Tuple[float, float],
    expected_bearing: float):
    bearing = geo_misc.compass_bearing_between_2_coordinates(
        point_from=point_from,
        point_to=point_to,
    )
    assert math.fabs(bearing - math.radians(expected_bearing)) < 0.1


@pytest.mark.parametrize(
    "point_from, point_to, expected_distance_meters",
    [
        (
            (math.radians(38.8976), math.radians(-77.0391)),
            (math.radians(39.9496), math.radians(-75.1511)),
            200_000.00,
        ),
        (
            (math.radians(55.7393), math.radians(37.5925)),
            (math.radians(55.7388), math.radians(37.5956)),
            201.9,
        ),
    ],
)
def test_distance(
    point_from: Tuple[float, float],
    point_to: Tuple[float, float],
    expected_distance_meters: float,
):
    for point1, point2 in [(point_from, point_to), (point_to, point_from)]:
        distance = geo_misc.distance_between_2_coordinates(
            point_from=point1, point_to=point2
        )
        assert distance > 0
        # Expect 1/1000 of the expected distance vairance
        variance = expected_distance_meters * 1e-3
        assert math.fabs(distance - expected_distance_meters) < variance
