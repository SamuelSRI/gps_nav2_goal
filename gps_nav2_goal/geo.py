"""Small-distance WGS84 to local ENU conversion helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


@dataclass(frozen=True)
class LocalOffset:
    east: float
    north: float
    up: float


def geodetic_to_enu_offset(
    latitude: float,
    longitude: float,
    altitude: float,
    origin_latitude: float,
    origin_longitude: float,
    origin_altitude: float,
) -> LocalOffset:
    """Convert a nearby WGS84 position into a local tangent-plane offset.

    This curvature-aware approximation is intended for rover-scale navigation
    (typically a few kilometres around the configured anchor).
    """
    lat0 = math.radians(origin_latitude)
    d_lat = math.radians(latitude - origin_latitude)
    d_lon = math.radians(longitude - origin_longitude)

    sin_lat = math.sin(lat0)
    denom = math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    prime_vertical_radius = WGS84_A / denom
    meridian_radius = WGS84_A * (1.0 - WGS84_E2) / (denom ** 3)

    east = d_lon * (prime_vertical_radius + origin_altitude) * math.cos(lat0)
    north = d_lat * (meridian_radius + origin_altitude)
    up = altitude - origin_altitude
    return LocalOffset(east=east, north=north, up=up)


def rotate_enu_to_map(east: float, north: float, yaw_rad: float) -> tuple[float, float]:
    """Rotate ENU coordinates into the Nav2 map frame.

    yaw_rad is the counter-clockwise rotation from ENU axes to map axes.
    With yaw_rad=0, map X points east and map Y points north.
    """
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    map_x = cos_yaw * east - sin_yaw * north
    map_y = sin_yaw * east + cos_yaw * north
    return map_x, map_y
