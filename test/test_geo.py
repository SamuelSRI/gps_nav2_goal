import math

from gps_nav2_goal.geo import geodetic_to_enu_offset, rotate_enu_to_map


def test_same_position_is_zero():
    offset = geodetic_to_enu_offset(48.0, 2.0, 100.0, 48.0, 2.0, 100.0)
    assert abs(offset.east) < 1e-9
    assert abs(offset.north) < 1e-9
    assert abs(offset.up) < 1e-9


def test_rotation_90_degrees():
    x, y = rotate_enu_to_map(10.0, 0.0, math.pi / 2.0)
    assert abs(x) < 1e-9
    assert abs(y - 10.0) < 1e-9
